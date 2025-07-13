#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <flyscan_interfaces/srv/set_control_mode.hpp>
#include <flyscan_interfaces/msg/teleop_command.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>

namespace flyscan {
namespace drone_controller {

class TeleopNode : public rclcpp::Node {
public:
    TeleopNode();
    ~TeleopNode();

private:
    void inputLoop();
    void processInput(char key);
    void sendTeleopCommand();
    void sendExitCommand();
    bool initializeTerminal();
    void cleanupTerminal();
    bool kbhit();
    char getch();
    void printInstructions();

    // ROS2 interfaces
    rclcpp::Client<flyscan_interfaces::srv::SetControlMode>::SharedPtr mode_client_;
    rclcpp::Publisher<flyscan_interfaces::msg::TeleopCommand>::SharedPtr teleop_pub_;
    rclcpp::TimerBase::SharedPtr input_timer_;

    // Terminal handling
    struct termios original_termios_;
    bool terminal_configured_;
    
    // Current teleop state
    flyscan_interfaces::msg::TeleopCommand current_command_;
    bool teleop_active_;
    
    // Movement parameters
    static constexpr float MOVE_STEP = 0.5f;  // meters
    static constexpr float YAW_STEP = 10.0f;  // degrees
};

} // namespace drone_controller
} // namespace flyscan