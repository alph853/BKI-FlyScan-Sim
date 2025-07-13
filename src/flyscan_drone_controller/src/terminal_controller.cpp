#include "flyscan_drone_controller/terminal_controller.hpp"
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <rclcpp/rclcpp.hpp>

namespace flyscan_drone_controller {
namespace util {

// Static member definitions
struct termios TerminalController::original_termios_;
bool TerminalController::terminal_configured_ = false;

bool TerminalController::initialize() {
    if (tcgetattr(STDIN_FILENO, &original_termios_) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("PX4Controller"), "Failed to get terminal attributes");
        return false;
    }
    
    struct termios new_termios = original_termios_;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    new_termios.c_cc[VMIN] = 0;
    new_termios.c_cc[VTIME] = 0;
    
    if (tcsetattr(STDIN_FILENO, TCSANOW, &new_termios) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("PX4Controller"), "Failed to set terminal attributes");
        return false;
    }
    
    if (fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("PX4Controller"), "Failed to set terminal to non-blocking mode");
        return false;
    }

    terminal_configured_ = true;
    RCLCPP_INFO(rclcpp::get_logger("PX4Controller"), "Terminal controller initialized");
    return true;
}

void TerminalController::cleanup() {
    if (terminal_configured_) {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
        terminal_configured_ = false;
        RCLCPP_INFO(rclcpp::get_logger("PX4Controller"), "Terminal controller cleaned up");
    }
}

bool TerminalController::kbhit() {
    fd_set readfds;
    struct timeval timeout;
    
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    return select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout) > 0;
}

char TerminalController::getch() {
    if (kbhit()) {
        return getchar();
    }
    return 0;
}

void TerminalController::printTeleopInstructions() {
    std::cout << "\n=== POSITION-BASED TELEOP CONTROL ===\n";
    std::cout << "W/S: Move Forward/Backward (0.5m steps)\n";
    std::cout << "A/D: Move Left/Right (0.5m steps)\n"; 
    std::cout << "Q/E: Move Up/Down (0.5m steps)\n";
    std::cout << "J/L: Yaw Left/Right (10° steps)\n";
    std::cout << "SPACE: Hold current position\n";
    std::cout << "ESC: Exit teleop mode\n";
    std::cout << "======================================\n\n";
}

} // namespace util
} // namespace flyscan_drone_controller