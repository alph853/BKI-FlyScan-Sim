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
    void InputLoop();
    void ProcessInput(char key);
    void SendTeleopCommand();
    void SendExitCommand();
    bool InitializeTerminal();
    void CleanupTerminal();
    bool Kbhit();
    char Getch();
    void PrintInstructions();
    void StartEightShapePattern();
    void ExecuteEightShapeStep();

    // ROS2 interfaces
    rclcpp::Client<flyscan_interfaces::srv::SetControlMode>::SharedPtr set_control_mode_client_;
    rclcpp::Publisher<flyscan_interfaces::msg::TeleopCommand>::SharedPtr teleop_pub_;
    rclcpp::TimerBase::SharedPtr input_timer_;

    // Terminal handling
    struct termios original_termios_;
    bool terminal_configured_;
    
    // Current teleop state
    flyscan_interfaces::msg::TeleopCommand current_command_;
    bool teleop_active_;
    
    // 8-shape pattern state
    rclcpp::TimerBase::SharedPtr eight_shape_timer_;
    bool executing_eight_shape_;
    int eight_shape_step_;
    std::chrono::steady_clock::time_point eight_shape_start_;
    
    // Movement parameters
    static constexpr float MOVE_STEP = 0.5f;  // meters
    static constexpr float YAW_STEP = 10.0f;  // degrees
    static constexpr float EIGHT_SHAPE_RADIUS = 2.0f;  // meters
    static constexpr float EIGHT_SHAPE_SPEED = 0.3f;   // m/s
};

} // namespace drone_controller
} // namespace flyscan