#include "flyscan_drone_controller/util.hpp"
#include <iostream>

namespace flyscan_drone_controller {
namespace util {

struct termios TerminalController::original_termios_;
bool TerminalController::terminal_configured_ = false;

bool TerminalController::initialize() {
    if (tcgetattr(STDIN_FILENO, &original_termios_) == -1) {
        return false;
    }
    
    struct termios new_termios = original_termios_;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    new_termios.c_cc[VMIN] = 0;
    new_termios.c_cc[VTIME] = 0;
    
    if (tcsetattr(STDIN_FILENO, TCSANOW, &new_termios) == -1) {
        return false;
    }
    
    if (fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK) == -1) {
        return false;
    }
    
    terminal_configured_ = true;
    return true;
}

void TerminalController::cleanup() {
    if (terminal_configured_) {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
        terminal_configured_ = false;
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

float MathUtils::constrainAngle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

float MathUtils::calculateDistance3D(float x1, float y1, float z1, float x2, float y2, float z2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dz = z2 - z1;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

float MathUtils::calculateDistance2D(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return std::sqrt(dx*dx + dy*dy);
}

bool MathUtils::isWithinTolerance(float value, float target, float tolerance) {
    return std::abs(value - target) <= tolerance;
}

std::string StringUtils::formatPosition(float north, float east, float down, int precision) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision);
    ss << "N:" << north << "m, E:" << east << "m, D:" << down << "m";
    return ss.str();
}

std::string StringUtils::formatFloat(float value, int precision) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision) << value;
    return ss.str();
}

std::string StringUtils::formatPercentage(float value, int precision) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision) << (value * 100.0f) << "%";
    return ss.str();
}

void printUsage(const std::string& bin_name) {
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP server: tcpin://<our_ip>:<port>\n"
              << " For TCP client: tcpout://<remote_ip>:<port>\n"
              << " For UDP server: udp://<our_ip>:<port>\n"
              << " For UDP client: udp://<remote_ip>:<port>\n"
              << " For Serial : serial://</path/to/serial/dev>:<baudrate>]\n"
              << "For example, to connect to the simulator use URL: udpin://0.0.0.0:14540\n";
}

} // namespace util
} // namespace flyscan_drone_controller