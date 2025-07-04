#include "flyscan_drone_controller/px4_controller.hpp"
#include "flyscan_drone_controller/util.hpp"

#include <rclcpp/rclcpp.hpp>
#include <csignal>
#include <iostream>
#include <thread>
#include <chrono>

using namespace flyscan_drone_controller;

void teleopControlLoop(PX4Controller& controller) {
    util::TerminalController::printTeleopInstructions();
    
    const float POSITION_STEP = 0.5f;
    const float YAW_STEP = 10.0f;
    const int COMMAND_RATE_LIMIT_MS = 100; // Minimum 200ms between position commands
    
    TeleopCommand target_position{};
    target_position.north_m = 0.0f;
    target_position.east_m = 0.0f;
    target_position.down_m = -1.5f;  // Default takeoff position
    target_position.yaw_deg = 0.0f;
    
    std::cout << "Ready for teleop commands. Press 'T' to takeoff, 'P' to land" << std::endl;
    
    bool offboard_started = false;
    bool should_exit = false;
    auto last_command_time = std::chrono::steady_clock::now();
    char last_key = 0;
    
    while (!should_exit) {
        char key = util::TerminalController::getch();
        
        if (key != 0) {
            // Rate limiting for movement commands
            auto current_time = std::chrono::steady_clock::now();
            auto time_since_last_command = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - last_command_time).count();
            
            bool is_movement_key = (key == 'w' || key == 'W' || key == 's' || key == 'S' ||
                                   key == 'a' || key == 'A' || key == 'd' || key == 'D' ||
                                   key == 'q' || key == 'Q' || key == 'e' || key == 'E' ||
                                   key == 'j' || key == 'J' || key == 'l' || key == 'L');
            
            // Skip if it's a movement key and rate limit hasn't passed, unless it's a different key
            if (is_movement_key && time_since_last_command < COMMAND_RATE_LIMIT_MS && key == last_key) {
                continue;
            }
            
            std::string feedback = ">> ";
            bool position_changed = false;
            
            switch (key) {
                case 't': case 'T':
                    if (!offboard_started) {
                        std::cout << "Starting takeoff..." << std::endl;
                        controller.takeoffToPosition();
                        offboard_started = true;
                        feedback += "TAKEOFF INITIATED";
                    } else {
                        feedback += "ALREADY IN OFFBOARD MODE";
                    }
                    break;
                    
                case 'w': case 'W':
                    if (offboard_started) {
                        target_position.north_m -= POSITION_STEP;
                        feedback += "FORWARD";
                        position_changed = true;
                    } else {
                        feedback += "TAKEOFF FIRST (press T)";
                    }
                    break;
                    
                case 's': case 'S':
                    if (offboard_started) {
                        target_position.north_m += POSITION_STEP;
                        feedback += "BACKWARD";
                        position_changed = true;
                    } else {
                        feedback += "TAKEOFF FIRST (press T)";
                    }
                    break;
                    
                case 'a': case 'A':
                    if (offboard_started) {
                        target_position.east_m += POSITION_STEP;
                        feedback += "LEFT";
                        position_changed = true;
                    } else {
                        feedback += "TAKEOFF FIRST (press T)";
                    }
                    break;
                    
                case 'd': case 'D':
                    if (offboard_started) {
                        target_position.east_m -= POSITION_STEP;
                        feedback += "RIGHT";
                        position_changed = true;
                    } else {
                        feedback += "TAKEOFF FIRST (press T)";
                    }
                    break;
                    
                case 'q': case 'Q':
                    if (offboard_started) {
                        target_position.down_m -= POSITION_STEP;
                        feedback += "UP";
                        position_changed = true;
                    } else {
                        feedback += "TAKEOFF FIRST (press T)";
                    }
                    break;
                    
                case 'e': case 'E':
                    if (offboard_started) {
                        target_position.down_m += POSITION_STEP;
                        feedback += "DOWN";
                        position_changed = true;
                    } else {
                        feedback += "TAKEOFF FIRST (press T)";
                    }
                    break;
                    
                case 'j': case 'J':
                    if (offboard_started) {
                        target_position.yaw_deg -= YAW_STEP;
                        target_position.yaw_deg = util::MathUtils::constrainAngle(target_position.yaw_deg);
                        feedback += "YAW LEFT";
                        position_changed = true;
                    } else {
                        feedback += "TAKEOFF FIRST (press T)";
                    }
                    break;
                    
                case 'l': case 'L':
                    if (offboard_started) {
                        target_position.yaw_deg += YAW_STEP;
                        target_position.yaw_deg = util::MathUtils::constrainAngle(target_position.yaw_deg);
                        feedback += "YAW RIGHT";
                        position_changed = true;
                    } else {
                        feedback += "TAKEOFF FIRST (press T)";
                    }
                    break;
                    
                case 'p': case 'P':
                    if (offboard_started) {
                        std::cout << "Landing..." << std::endl;
                        controller.land();
                        feedback += "LANDING INITIATED";
                        offboard_started = false;
                    } else {
                        feedback += "NOT IN OFFBOARD MODE";
                    }
                    break;
                    
                case ' ':
                    if (offboard_started) {
                        feedback += "HOLD POSITION";
                    } else {
                        feedback += "TAKEOFF FIRST (press T)";
                    }
                    break;
                    
                case 27: // ESC
                    std::cout << "Exiting teleop mode..." << std::endl;
                    should_exit = true;
                    break;
                    
                default:
                    continue;
            }
            
            if (!should_exit) {
                if (position_changed && offboard_started) {
                    controller.updateTeleopCommand(target_position);
                    feedback += " -> " + util::StringUtils::formatPosition(
                        target_position.north_m, target_position.east_m, target_position.down_m);
                    // Update rate limiting tracking for movement commands
                    if (is_movement_key) {
                        last_command_time = current_time;
                        last_key = key;
                    }
                }
                
                std::cout << "\r" << std::string(80, ' ') << "\r";
                std::cout << feedback << std::flush;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    std::cout << "Teleop control loop ended" << std::endl;
}

int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Initialize terminal control
    if (!util::TerminalController::initialize()) {
        std::cerr << "Failed to initialize terminal control" << std::endl;
        return 1;
    }
    
    // Setup signal handler
    std::signal(SIGINT, [](int) {
        util::TerminalController::cleanup();
        rclcpp::shutdown();
        std::exit(0);
    });

    try {
        // Create and initialize PX4 controller
        auto controller = std::make_shared<PX4Controller>("px4_teleop_controller");
        
        if (!controller->initialize()) {
            std::cerr << "Failed to initialize PX4 Controller" << std::endl;
            util::TerminalController::cleanup();
            rclcpp::shutdown();
            return 1;
        }
        
        std::cout << "Starting teleop control..." << std::endl;
        
        // Start ROS2 spinning in a separate thread
        std::thread ros_thread([controller]() {
            rclcpp::spin(controller);
        });
        
        // Enter teleop mode immediately
        teleopControlLoop(*controller);
        
        // Clean up
        rclcpp::shutdown();
        ros_thread.join();
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        util::TerminalController::cleanup();
        rclcpp::shutdown();
        return 1;
    }
    
    util::TerminalController::cleanup();
    return 0;
}