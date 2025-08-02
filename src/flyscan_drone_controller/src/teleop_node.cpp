#include <iostream>
#include <chrono>
#include <signal.h>

#include "flyscan_common/request_util.hpp"
#include "flyscan_drone_controller/teleop_node.hpp"
#include "flyscan_drone_controller/constants.hpp"

using namespace std::chrono_literals;
using flyscan::common::sync_send_request;

namespace flyscan {
namespace drone_controller {

TeleopNode::TeleopNode() 
    : Node("teleop_node"), terminal_configured_(false), teleop_active_(false) {
    
    // Initialize ROS2 interfaces
    namespace srv = flyscan::drone_controller::constants::srv;
    namespace topic = flyscan::drone_controller::constants::topic;
    
    mode_client_ = this->create_client<flyscan_interfaces::srv::SetControlMode>(srv::SET_CONTROL_MODE);
    teleop_pub_ = this->create_publisher<flyscan_interfaces::msg::TeleopCommand>(topic::TELEOP_COMMAND, 10);

    // Initialize terminal for keyboard input
    if (!InitializeTerminal()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize terminal. Teleop will not work.");
        return;
    }

    auto request = std::make_shared<flyscan_interfaces::srv::SetControlMode::Request>();
    request->mode = 1; // TELEOP mode

    flyscan_interfaces::srv::SetControlMode::Response::SharedPtr response;

    try {
        response = sync_send_request<flyscan_interfaces::srv::SetControlMode>(this->get_node_base_interface(), mode_client_, request, 20s, 20s);
    } catch(const std::runtime_error &e) {
        RCLCPP_ERROR(this->get_logger(), "Service call fail in %s: %s", this->get_name(), e.what());
        CleanupTerminal();
        return;
    } catch(const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception caught in %s: %s", this->get_name(), e.what());
        CleanupTerminal();
        return;
    }

    if (!response->success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to switch to teleop mode: %s", response->message.c_str());
        CleanupTerminal();
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Successfully switched to teleop mode");
    teleop_active_ = true;
    PrintInstructions();

    input_timer_ = this->create_wall_timer(50ms, std::bind(&TeleopNode::InputLoop, this));
    
    // Initialize command message
    current_command_.command = "stop";
    
    // Initialize 8-shape pattern state
    executing_eight_shape_ = false;
    eight_shape_step_ = 0;
}

TeleopNode::~TeleopNode() {
    if (teleop_active_) {
        SendExitCommand();
    }
    
    // Clean up 8-shape timer if running
    if (eight_shape_timer_) {
        eight_shape_timer_->cancel();
        eight_shape_timer_.reset();
    }
    executing_eight_shape_ = false;
    
    CleanupTerminal();
}

void TeleopNode::InputLoop() {
    if (!teleop_active_) return;
    
    if (Kbhit()) {
        char key = this->Getch();
        // Only process input if not executing 8-shape pattern (except ESC key)
        if (!executing_eight_shape_ || key == 27) {
            this->ProcessInput(key);
        }
    }
}

void TeleopNode::ProcessInput(char key) {
    
    switch (key) {
        case 'w':
        case 'W':
            RCLCPP_INFO(this->get_logger(), "W: Forward");
            current_command_.command = "forward";
            break;
        case 's':
        case 'S':
            RCLCPP_INFO(this->get_logger(), "S: Backward");
            current_command_.command = "backward";
            break;
        case 'a':
        case 'A':
            RCLCPP_INFO(this->get_logger(), "A: Left");
            current_command_.command = "left";
            break;
        case 'd':
        case 'D':
            RCLCPP_INFO(this->get_logger(), "D: Right");
            current_command_.command = "right";
            break;
        case 'q':
        case 'Q':
            RCLCPP_INFO(this->get_logger(), "Q: Up");
            current_command_.command = "up";
            break;
        case 'e':
        case 'E':
            RCLCPP_INFO(this->get_logger(), "E: Down");
            current_command_.command = "down";
            break;
        case 'j':
        case 'J':
            RCLCPP_INFO(this->get_logger(), "J: Yaw left");
            current_command_.command = "yaw_left";
            break;
        case 'l':
        case 'L':
            RCLCPP_INFO(this->get_logger(), "L: Yaw right");
            current_command_.command = "yaw_right";
            break;
        case ' ':
            RCLCPP_INFO(this->get_logger(), "SPACE: Hold position");
            current_command_.command = "hold_position";
            break;
        case 't':
        case 'T':
            RCLCPP_INFO(this->get_logger(), "T: Takeoff");
            current_command_.command = "takeoff";
            break;
        case 'p':
        case 'P':
            RCLCPP_INFO(this->get_logger(), "P: Land");
            current_command_.command = "land";
            break;
        case '8':
            if (!executing_eight_shape_) {
                RCLCPP_INFO(this->get_logger(), "8: Starting 8-shape pattern");
                StartEightShapePattern();
            } else {
                RCLCPP_INFO(this->get_logger(), "8-shape pattern already executing");
            }
            return;
        case 27: // ESC key
            RCLCPP_INFO(this->get_logger(), "ESC: Exit teleop mode");
            current_command_.command = "exit_teleop";
            this->SendExitCommand();
            return;
        default:
            // Unknown key, don't send command
            return;
    }
    
    this->SendTeleopCommand();
}

void TeleopNode::SendTeleopCommand() {
    teleop_pub_->publish(current_command_);
}

void TeleopNode::SendExitCommand() {
    current_command_.command = "exit_teleop";
    teleop_pub_->publish(current_command_);
    teleop_active_ = false;
    
    // Stop the input timer
    if (input_timer_) {
        input_timer_->cancel();
    }
}

bool TeleopNode::InitializeTerminal() {
    if (tcgetattr(STDIN_FILENO, &original_termios_) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get terminal attributes");
        return false;
    }
    
    struct termios new_termios = original_termios_;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    new_termios.c_cc[VMIN] = 0;
    new_termios.c_cc[VTIME] = 0;
    
    if (tcsetattr(STDIN_FILENO, TCSANOW, &new_termios) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set terminal attributes");
        return false;
    }
    
    if (fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set terminal to non-blocking mode");
        return false;
    }
    
    terminal_configured_ = true;
    RCLCPP_INFO(this->get_logger(), "Terminal controller initialized");
    return true;
}

void TeleopNode::CleanupTerminal() {
    if (terminal_configured_) {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
        terminal_configured_ = false;
        RCLCPP_INFO(this->get_logger(), "Terminal controller cleaned up");
    }
}

bool TeleopNode::Kbhit() {
    fd_set readfds;
    struct timeval timeout;
    
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    return select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout) > 0;
}

char TeleopNode::Getch() {
    if (Kbhit()) {
        return getchar();
    }
    return 0;
}

void TeleopNode::PrintInstructions() {
    std::cout << "\n=== TELEOP CONTROL ACTIVE ===\n";
    std::cout << "W/S: Move Forward/Backward\n";
    std::cout << "A/D: Move Left/Right\n"; 
    std::cout << "Q/E: Move Up/Down\n";
    std::cout << "J/L: Yaw Left/Right\n";
    std::cout << "SPACE: Hold current position\n";
    std::cout << "T: Takeoff to NED (0,0,-1.5)\n";
    std::cout << "P: Land to NED (0,0,0)\n";
    std::cout << "8: Fly in 8-shape pattern\n";
    std::cout << "ESC: Exit teleop mode\n";
    std::cout << "=============================\n\n";
}

void TeleopNode::StartEightShapePattern() {
    if (executing_eight_shape_) {
        return;
    }
    
    executing_eight_shape_ = true;
    eight_shape_step_ = 0;
    eight_shape_start_ = std::chrono::steady_clock::now();
    
    // Create timer for 8-shape execution (20Hz update rate)
    eight_shape_timer_ = this->create_wall_timer(
        50ms, 
        std::bind(&TeleopNode::ExecuteEightShapeStep, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "8-shape pattern started");
}

void TeleopNode::ExecuteEightShapeStep() {
    if (!executing_eight_shape_) {
        return;
    }
    
    // Calculate time elapsed since start
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - eight_shape_start_).count();
    
    // 8-shape pattern duration: 20 seconds for slow movement
    const double total_duration = 20000.0; // 20 seconds in milliseconds
    double t = static_cast<double>(elapsed) / total_duration;
    
    if (t >= 1.0) {
        // Pattern complete, return to original position and stop
        current_command_.command = "hold_position";
        
        SendTeleopCommand();
        
        // Clean up timer and reset state
        if (eight_shape_timer_) {
            eight_shape_timer_->cancel();
            eight_shape_timer_.reset();
        }
        executing_eight_shape_ = false;
        
        RCLCPP_INFO(this->get_logger(), "8-shape pattern completed. You can now use manual controls.");
        return;
    }
    
    // Calculate 8-shape trajectory
    // Using parametric equations for figure-8: x = sin(2πt), y = sin(4πt)/2
    const double pi = 3.14159265359;
    const double angle1 = 2.0 * pi * t;      // Main frequency
    const double angle2 = 4.0 * pi * t;      // Double frequency for figure-8
    
    // Calculate velocities (derivatives of position)
    namespace config = flyscan::drone_controller::constants::config;
    double vel_forward = config::EIGHT_SHAPE_SPEED * 2.0 * pi * cos(angle1);
    double vel_right = config::EIGHT_SHAPE_SPEED * 2.0 * pi * cos(angle2);
    
    // Convert to discrete commands based on dominant velocity
    const double threshold = 0.1;
    if (std::abs(vel_forward) > std::abs(vel_right) && std::abs(vel_forward) > threshold) {
        current_command_.command = (vel_forward > 0) ? "forward" : "backward";
    } else if (std::abs(vel_right) > threshold) {
        current_command_.command = (vel_right > 0) ? "right" : "left";
    } else {
        current_command_.command = "hold_position";
    }
    
    SendTeleopCommand();
}

} // namespace drone_controller
} // namespace flyscan


std::shared_ptr<flyscan::drone_controller::TeleopNode> teleop_node = nullptr;

void signalHandler(int signum) {
    if (teleop_node) {
        RCLCPP_INFO(teleop_node->get_logger(), "Caught signal %d, shutting down teleop node", signum);
        rclcpp::shutdown();
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // Set up signal handler
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try {
        teleop_node = std::make_shared<flyscan::drone_controller::TeleopNode>();
        rclcpp::spin(teleop_node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("teleop_main"), "Exception caught: %s", e.what());
    }
    
    teleop_node.reset();
    rclcpp::shutdown();
    return 0;
}