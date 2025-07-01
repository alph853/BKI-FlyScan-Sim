/**
 * @file px4_controller.hpp
 * @brief Comprehensive PX4 autopilot controller using MAVSDK C++
 * @author Your Name
 * @date 2025
 * 
 * This header defines the PX4Controller class for complete drone control
 * including teleop, autonomous flights, and all MAVSDK functionalities.
 */

#ifndef PX4_CONTROLLER_HPP
#define PX4_CONTROLLER_HPP

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/follow_me/follow_me.h>
#include <mavsdk/plugins/gimbal/gimbal.h>
#include <mavsdk/plugins/camera/camera.h>
#include <mavsdk/plugins/info/info.h>
#include <mavsdk/plugins/calibration/calibration.h>
#include <mavsdk/plugins/param/param.h>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <future>
#include <memory>
#include <thread>
#include <vector>
#include <iomanip>
#include <atomic>
#include <functional>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace mavsdk;

/**
 * @enum FlightMode
 * @brief Available flight modes for the controller
 */
enum class FlightMode {
    MANUAL_TELEOP,    ///< Manual teleoperation with keyboard
    SAMPLE_FLIGHT,    ///< Predefined sample flight pattern
    MISSION,          ///< Mission execution
    OFFBOARD_DEMO,    ///< Offboard control demonstration
    IDLE              ///< Idle state
};

/**
 * @struct TeleopCommand
 * @brief Structure for teleop velocity commands
 */
struct TeleopCommand {
    float velocity_x_m_s = 0.0f;    ///< Forward/backward velocity (m/s)
    float velocity_y_m_s = 0.0f;    ///< Left/right velocity (m/s)
    float velocity_z_m_s = 0.0f;    ///< Up/down velocity (m/s)
    float yaw_rate_deg_s = 0.0f;    ///< Yaw rate (deg/s)
};

/**
 * @class PX4Controller
 * @brief Comprehensive PX4 drone controller with multiple flight modes
 * 
 * This class provides complete control over PX4-based drones including:
 * - Connection management and system monitoring
 * - Telemetry data streaming
 * - Manual teleoperation via keyboard
 * - Autonomous sample flights
 * - Mission planning and execution
 * - Offboard control capabilities
 * - Camera and gimbal control
 * - Parameter management
 */
class PX4Controller {
private:
    // MAVSDK components
    std::unique_ptr<Mavsdk> mavsdk_;
    std::shared_ptr<System> system_;
    std::unique_ptr<Action> action_;
    std::unique_ptr<Telemetry> telemetry_;
    std::unique_ptr<Offboard> offboard_;
    std::unique_ptr<Mission> mission_;
    std::unique_ptr<FollowMe> follow_me_;
    std::unique_ptr<Gimbal> gimbal_;
    std::unique_ptr<Camera> camera_;
    std::unique_ptr<Info> info_;
    std::unique_ptr<Calibration> calibration_;
    std::unique_ptr<Param> param_;

    // State management
    std::atomic<bool> connected_{false};
    std::atomic<bool> armed_{false};
    std::atomic<bool> in_air_{false};
    std::atomic<FlightMode> current_mode_{FlightMode::IDLE};
    std::atomic<bool> shutdown_requested_{false};

    // Telemetry data (thread-safe access)
    std::atomic<float> altitude_m_{0.0f};
    std::atomic<float> battery_percent_{0.0f};
    std::atomic<float> latitude_{0.0};
    std::atomic<float> longitude_{0.0};

    // Teleop control
    TeleopCommand current_teleop_cmd_;
    std::thread teleop_thread_;
    std::thread flight_thread_;
    std::atomic<bool> teleop_active_{false};

    // Terminal control for raw keyboard input
    struct termios original_termios_;
    bool terminal_configured_{false};

    // Callback functions for external integration
    std::function<void(const std::string&)> log_callback_;
    std::function<void(float, float, float)> position_callback_;
    std::function<void(float)> battery_callback_;

public:
    /**
     * @brief Constructor
     */
    PX4Controller();

    /**
     * @brief Destructor - cleanup resources
     */
    ~PX4Controller();

    // === Connection Management ===
    
    /**
     * @brief Connect to PX4 autopilot
     * @param connection_url Connection string (e.g., "udp://:14540")
     * @return true if connection successful
     */
    bool connect(const std::string& connection_url);

    /**
     * @brief Disconnect from autopilot
     */
    void disconnect();

    /**
     * @brief Check if connected to autopilot
     * @return true if connected
     */
    bool isConnected() const { return connected_.load(); }

    // === System Information ===

    /**
     * @brief Get comprehensive system information
     */
    void getSystemInfo();

    /**
     * @brief Setup telemetry monitoring with callbacks
     */
    void setupTelemetry();

    /**
     * @brief Wait for system to be ready for operations
     * @param timeout_s Maximum wait time in seconds
     * @return true if ready within timeout
     */
    bool waitForReady(int timeout_s = 30);

    // === Basic Flight Control ===

    /**
     * @brief Arm the vehicle
     * @return true if successful
     */
    bool arm();

    /**
     * @brief Disarm the vehicle
     * @return true if successful
     */
    bool disarm();

    /**
     * @brief Takeoff to specified altitude
     * @param altitude_m Target altitude in meters
     * @return true if successful
     */
    bool takeoff(float altitude_m = 5.0f);

    /**
     * @brief Land the vehicle
     * @return true if successful
     */
    bool land();

    /**
     * @brief Return to launch position
     * @return true if successful
     */
    bool returnToLaunch();

    /**
     * @brief Emergency stop - immediate disarm
     */
    void emergencyStop();

    // === Flight Modes ===

    /**
     * @brief Start manual teleoperation mode
     * Controls:
     * - W/S: Forward/Backward
     * - A/D: Left/Right  
     * - Q/E: Up/Down
     * - J/L: Yaw Left/Right
     * - SPACE: Stop all movement
     * - ESC: Exit teleop mode
     */
    void startTeleopMode();

    /**
     * @brief Stop teleoperation mode
     */
    void stopTeleopMode();

    /**
     * @brief Execute predefined sample flight pattern
     * Includes takeoff, figure-8 pattern, photo capture, and landing
     */
    void executeSampleFlight();

    /**
     * @brief Get current flight mode
     * @return Current FlightMode
     */
    FlightMode getCurrentMode() const { return current_mode_.load(); }

    // === Advanced Control ===

    /**
     * @brief Execute offboard position control
     * @param north_m North position (meters)
     * @param east_m East position (meters) 
     * @param down_m Down position (meters, negative for up)
     * @param yaw_deg Yaw angle (degrees)
     */
    void setOffboardPosition(float north_m, float east_m, float down_m, float yaw_deg);

    /**
     * @brief Execute offboard velocity control
     * @param vx_m_s Velocity forward (m/s)
     * @param vy_m_s Velocity right (m/s)
     * @param vz_m_s Velocity down (m/s) 
     * @param yaw_rate_deg_s Yaw rate (deg/s)
     */
    void setOffboardVelocity(float vx_m_s, float vy_m_s, float vz_m_s, float yaw_rate_deg_s);

    /**
     * @brief Start offboard mode
     * @return true if successful
     */
    bool startOffboard();

    /**
     * @brief Stop offboard mode
     * @return true if successful
     */
    bool stopOffboard();

    // === Mission Management ===

    /**
     * @brief Create and upload a sample mission
     * @param pattern Mission pattern type (0=square, 1=circle, 2=figure8)
     * @param size_m Pattern size in meters
     * @return true if successful
     */
    bool createSampleMission(int pattern = 0, float size_m = 20.0f);

    /**
     * @brief Upload custom mission
     * @param mission_items Vector of mission items
     * @return true if successful
     */
    bool uploadMission(const std::vector<Mission::MissionItem>& mission_items);

    /**
     * @brief Start uploaded mission
     * @return true if successful
     */
    bool startMission();

    /**
     * @brief Pause current mission
     * @return true if successful
     */
    bool pauseMission();

    /**
     * @brief Clear current mission
     * @return true if successful
     */
    bool clearMission();

    // === Camera and Gimbal ===

    /**
     * @brief Control gimbal orientation
     * @param tilt_deg Tilt angle in degrees (-90 to +30)
     * @param pan_deg Pan angle in degrees
     * @return true if successful
     */
    bool controlGimbal(float tilt_deg, float pan_deg = 0.0f);

    /**
     * @brief Take a photo
     * @return true if successful
     */
    bool takePhoto();

    /**
     * @brief Start video recording
     * @return true if successful
     */
    bool startVideoRecording();

    /**
     * @brief Stop video recording
     * @return true if successful
     */
    bool stopVideoRecording();

    // === Parameter Management ===

    /**
     * @brief Get integer parameter
     * @param name Parameter name
     * @return Parameter value (or 0 if failed)
     */
    int getParamInt(const std::string& name);

    /**
     * @brief Get float parameter
     * @param name Parameter name
     * @return Parameter value (or 0.0 if failed)
     */
    float getParamFloat(const std::string& name);

    /**
     * @brief Set integer parameter
     * @param name Parameter name
     * @param value Parameter value
     * @return true if successful
     */
    bool setParamInt(const std::string& name, int value);

    /**
     * @brief Set float parameter
     * @param name Parameter name
     * @param value Parameter value
     * @return true if successful
     */
    bool setParamFloat(const std::string& name, float value);

    // === Telemetry Access ===

    /**
     * @brief Get current position
     * @return Current GPS position
     */
    Telemetry::Position getCurrentPosition();

    /**
     * @brief Get current attitude
     * @return Current attitude (Euler angles)
     */
    Telemetry::EulerAngle getCurrentAttitude();

    /**
     * @brief Get current velocity
     * @return Current velocity in NED frame
     */
    Telemetry::VelocityNed getCurrentVelocity();

    /**
     * @brief Get battery status
     * @return Battery information
     */
    Telemetry::Battery getBatteryStatus();

    /**
     * @brief Get flight mode
     * @return Current PX4 flight mode
     */
    Telemetry::FlightMode getPX4FlightMode();

    // === Callback Registration ===

    /**
     * @brief Set logging callback for external integration
     * @param callback Function to call for log messages
     */
    void setLogCallback(std::function<void(const std::string&)> callback);

    /**
     * @brief Set position callback for external integration
     * @param callback Function to call with position updates (lat, lon, alt)
     */
    void setPositionCallback(std::function<void(float, float, float)> callback);

    /**
     * @brief Set battery callback for external integration
     * @param callback Function to call with battery percentage updates
     */
    void setBatteryCallback(std::function<void(float)> callback);

    // === Utility Functions ===

    /**
     * @brief Print current system status
     */
    void printStatus();

    /**
     * @brief Request shutdown of all operations
     */
    void requestShutdown();

    /**
     * @brief Check if shutdown was requested
     * @return true if shutdown requested
     */
    bool isShutdownRequested() const { return shutdown_requested_.load(); }

private:
    // === Private Helper Methods ===

    /**
     * @brief Initialize all MAVSDK plugins
     */
    void initializePlugins();

    /**
     * @brief Configure terminal for raw keyboard input
     */
    void configureTerminal();

    /**
     * @brief Restore terminal to original state
     */
    void restoreTerminal();

    /**
     * @brief Check if keyboard input is available
     * @return true if key pressed
     */
    bool kbhit();

    /**
     * @brief Get single character from keyboard (non-blocking)
     * @return Character pressed or 0 if none
     */
    char getch();

    /**
     * @brief Teleoperation thread function
     */
    void teleopThreadFunction();

    /**
     * @brief Sample flight thread function
     */
    void sampleFlightThreadFunction();

    /**
     * @brief Log message with timestamp
     * @param message Message to log
     */
    void logMessage(const std::string& message);

    /**
     * @brief Calculate distance between two GPS coordinates
     * @param lat1 Latitude 1 (degrees)
     * @param lon1 Longitude 1 (degrees)
     * @param lat2 Latitude 2 (degrees) 
     * @param lon2 Longitude 2 (degrees)
     * @return Distance in meters
     */
    float calculateDistance(double lat1, double lon1, double lat2, double lon2);

    /**
     * @brief Generate GPS waypoint offset from current position
     * @param north_offset_m North offset in meters
     * @param east_offset_m East offset in meters
     * @return GPS coordinates
     */
    std::pair<double, double> generateWaypoint(float north_offset_m, float east_offset_m);
};

#endif // PX4_CONTROLLER_HPP