#include <iostream>
#include <chrono>

#include "flyscan_common/util.hpp"
#include "flyscan_drone_controller/teleop_node.hpp"

using namespace std::chrono_literals;
using flyscan::common::sync_send_request;

namespace flyscan {
namespace drone_controller {

TeleopNode::TeleopNode() 
    : Node("teleop_node"), terminal_configured_(false), teleop_active_(false) {
    
    // Initialize ROS2 interfaces
    mode_client_ = this->create_client<flyscan_interfaces::srv::SetControlMode>("/px4_controller/set_control_mode");
    teleop_pub_ = this->create_publisher<flyscan_interfaces::msg::TeleopCommand>("/px4_controller/teleop_command", 10);

    // Initialize terminal for keyboard input
    if (!initializeTerminal()) {
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
        cleanupTerminal();
        return;
    } catch(const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception caught in %s: %s", this->get_name(), e.what());
        cleanupTerminal();
        return;
    }

    if (!response->success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to switch to teleop mode: %s", response->message.c_str());
        cleanupTerminal();
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Successfully switched to teleop mode");
    teleop_active_ = true;
    printInstructions();

    input_timer_ = this->create_wall_timer(50ms, std::bind(&TeleopNode::inputLoop, this));
    
    // Initialize command message
    current_command_.forward = 0.0f;
    current_command_.right = 0.0f;
    current_command_.up = 0.0f;
    current_command_.yaw_rate = 0.0f;
    current_command_.hold_position = false;
    current_command_.exit_teleop = false;
    current_command_.takeoff = false;
    current_command_.land = false;
}

TeleopNode::~TeleopNode() {
    if (teleop_active_) {
        sendExitCommand();
    }
    cleanupTerminal();
}

void TeleopNode::inputLoop() {
    if (!teleop_active_) return;
    
    if (kbhit()) {
        char key = getch();
        processInput(key);
    }
}

void TeleopNode::processInput(char key) {
    // Reset command
    current_command_.forward = 0.0f;
    current_command_.right = 0.0f;
    current_command_.up = 0.0f;
    current_command_.yaw_rate = 0.0f;
    current_command_.hold_position = false;
    current_command_.exit_teleop = false;
    current_command_.takeoff = false;
    current_command_.land = false;
    
    switch (key) {
        case 'w':
        case 'W':
            current_command_.forward = 1.0f;
            RCLCPP_INFO(this->get_logger(), "Moving forward");
            break;
        case 's':
        case 'S':
            current_command_.forward = -1.0f;
            RCLCPP_INFO(this->get_logger(), "Moving backward");
            break;
        case 'a':
        case 'A':
            current_command_.right = -1.0f;
            RCLCPP_INFO(this->get_logger(), "Moving left");
            break;
        case 'd':
        case 'D':
            current_command_.right = 1.0f;
            RCLCPP_INFO(this->get_logger(), "Moving right");
            break;
        case 'q':
        case 'Q':
            current_command_.up = 1.0f;
            RCLCPP_INFO(this->get_logger(), "Moving up");
            break;
        case 'e':
        case 'E':
            current_command_.up = -1.0f;
            RCLCPP_INFO(this->get_logger(), "Moving down");
            break;
        case 'j':
        case 'J':
            current_command_.yaw_rate = -1.0f;
            RCLCPP_INFO(this->get_logger(), "Yawing left");
            break;
        case 'l':
        case 'L':
            current_command_.yaw_rate = 1.0f;
            RCLCPP_INFO(this->get_logger(), "Yawing right");
            break;
        case ' ':
            current_command_.hold_position = true;
            RCLCPP_INFO(this->get_logger(), "Holding position");
            break;
        case 't':
        case 'T':
            current_command_.takeoff = true;
            RCLCPP_INFO(this->get_logger(), "Takeoff command - going to NED (0,0,-1.5)");
            break;
        case 'p':
        case 'P':
            current_command_.land = true;
            RCLCPP_INFO(this->get_logger(), "Land command - going to NED (0,0,0)");
            break;
        case 27: // ESC key
            RCLCPP_INFO(this->get_logger(), "Exiting teleop mode");
            current_command_.exit_teleop = true;
            sendExitCommand();
            return;
        default:
            // Unknown key, don't send command
            return;
    }
    
    sendTeleopCommand();
}

void TeleopNode::sendTeleopCommand() {
    teleop_pub_->publish(current_command_);
}

void TeleopNode::sendExitCommand() {
    current_command_.exit_teleop = true;
    teleop_pub_->publish(current_command_);
    teleop_active_ = false;
    
    // Stop the input timer
    if (input_timer_) {
        input_timer_->cancel();
    }
}

bool TeleopNode::initializeTerminal() {
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

void TeleopNode::cleanupTerminal() {
    if (terminal_configured_) {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
        terminal_configured_ = false;
        RCLCPP_INFO(this->get_logger(), "Terminal controller cleaned up");
    }
}

bool TeleopNode::kbhit() {
    fd_set readfds;
    struct timeval timeout;
    
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    return select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout) > 0;
}

char TeleopNode::getch() {
    if (kbhit()) {
        return getchar();
    }
    return 0;
}

void TeleopNode::printInstructions() {
    std::cout << "\n=== TELEOP CONTROL ACTIVE ===\n";
    std::cout << "W/S: Move Forward/Backward\n";
    std::cout << "A/D: Move Left/Right\n"; 
    std::cout << "Q/E: Move Up/Down\n";
    std::cout << "J/L: Yaw Left/Right\n";
    std::cout << "SPACE: Hold current position\n";
    std::cout << "T: Takeoff to NED (0,0,-1.5)\n";
    std::cout << "P: Land to NED (0,0,0)\n";
    std::cout << "ESC: Exit teleop mode\n";
    std::cout << "=============================\n\n";
}

} // namespace drone_controller
} // namespace flyscan