/**
 * @file px4_controller.cpp
 * @brief Implementation of comprehensive PX4 autopilot controller
 * @author Your Name
 * @date 2025
 */

#include "flyscan_drone_controller/px4_controller.hpp"
#include <cmath>
#include <sstream>
#include <iomanip>

using std::chrono::seconds;
using std::chrono::milliseconds;
using std::this_thread::sleep_for;

// Constructor
PX4Controller::PX4Controller() : 
    mavsdk_(std::make_unique<Mavsdk>(Mavsdk::Configuration{mavsdk::ComponentType::GroundStation})) {
    logMessage("PX4Controller initialized");
}

// Destructor
PX4Controller::~PX4Controller() {
    requestShutdown();
    disconnect();
    if (terminal_configured_) {
        restoreTerminal();
    }
    logMessage("PX4Controller destroyed");
}

// === Connection Management ===

bool PX4Controller::connect(const std::string& connection_url) {
    logMessage("Attempting to connect to: " + connection_url);

    // Add connection
    ConnectionResult connection_result = mavsdk_->add_any_connection(connection_url);
    if (connection_result != ConnectionResult::Success) {
        logMessage("Connection failed: " + std::to_string(static_cast<int>(connection_result)));
        return false;
    }

    // Wait for system discovery
    logMessage("Waiting to discover system...");
    auto prom = std::promise<std::shared_ptr<System>>{};
    auto fut = prom.get_future();

    Mavsdk::NewSystemHandle handle = mavsdk_->subscribe_on_new_system([this, &prom, &handle]() {
        auto system = mavsdk_->systems().back();
        if (system->has_autopilot()) {
            logMessage("Discovered autopilot");
            mavsdk_->unsubscribe_on_new_system(handle);
            prom.set_value(system);
        }
    });

    // Wait for discovery with timeout
    if (fut.wait_for(seconds(10)) == std::future_status::timeout) {
        logMessage("No autopilot found within timeout");
        return false;
    }

    // Get the system and initialize plugins
    system_ = fut.get();
    initializePlugins();
    connected_ = true;
    
    logMessage("Connected successfully!");
    return true;
}

void PX4Controller::disconnect() {
    if (connected_) {
        stopTeleopMode();
        connected_ = false;
        logMessage("Disconnected from autopilot");
    }
}

// === System Information ===

void PX4Controller::getSystemInfo() {
    if (!connected_) return;

    logMessage("=== SYSTEM INFORMATION ===");
    
    // Flight information
    auto flight_info = info_->get_flight_information();
    if (flight_info.first == Info::Result::Success) {
        logMessage("Flight time: " + std::to_string(flight_info.second.duration_since_takeoff_ms/1000) + "s");
    }

    // Identification
    auto identification = info_->get_identification();
    if (identification.first == Info::Result::Success) {
        logMessage("Hardware UID: " + identification.second.hardware_uid);
    }

    // Version
    auto version = info_->get_version();
    if (version.first == Info::Result::Success) {
        std::stringstream ss;
        ss << "Flight SW version: " << static_cast<int>(version.second.flight_sw_major)
           << "." << static_cast<int>(version.second.flight_sw_minor)
           << "." << static_cast<int>(version.second.flight_sw_patch);
        logMessage(ss.str());
    }
}

void PX4Controller::setupTelemetry() {
    if (!connected_) return;

    logMessage("Setting up telemetry...");

    // Set telemetry rates
    telemetry_->set_rate_position(2.0);
    telemetry_->set_rate_velocity_ned(2.0);
    telemetry_->set_rate_altitude(5.0);
    telemetry_->set_rate_battery(1.0);

    // Subscribe to position updates
    telemetry_->subscribe_position([this](Telemetry::Position position) {
        latitude_ = position.latitude_deg;
        longitude_ = position.longitude_deg;
        altitude_m_ = position.relative_altitude_m;
        
        if (position_callback_) {
            position_callback_(position.latitude_deg, position.longitude_deg, position.relative_altitude_m);
        }
    });

    // Subscribe to battery updates
    telemetry_->subscribe_battery([this](Telemetry::Battery battery) {
        battery_percent_ = battery.remaining_percent * 100.0f;
        
        if (battery_callback_) {
            battery_callback_(battery_percent_.load());
        }
    });

    // Subscribe to armed state
    telemetry_->subscribe_armed([this](bool armed) {
        armed_ = armed;
        logMessage(armed ? "Vehicle ARMED" : "Vehicle DISARMED");
    });

    // Subscribe to in-air state
    telemetry_->subscribe_in_air([this](bool in_air) {
        in_air_ = in_air;
        logMessage(in_air ? "Vehicle IN AIR" : "Vehicle ON GROUND");
    });

    // Subscribe to flight mode changes
    telemetry_->subscribe_flight_mode([this](Telemetry::FlightMode flight_mode) {
        std::stringstream ss;
        ss << "PX4 Flight mode: " << flight_mode;
        logMessage(ss.str());
    });

    logMessage("Telemetry setup complete");
}

bool PX4Controller::waitForReady(int timeout_s) {
    if (!connected_) return false;

    logMessage("Waiting for system to be ready...");
    
    int elapsed = 0;
    while (elapsed < timeout_s && !shutdown_requested_) {
        if (telemetry_->health_all_ok()) {
            logMessage("System is ready!");
            return true;
        }
        
        sleep_for(seconds(1));
        elapsed++;
        
        if (elapsed % 5 == 0) {
            logMessage("Still waiting... (" + std::to_string(elapsed) + "s)");
        }
    }

    logMessage("System not ready within timeout");
    return false;
}

// === Basic Flight Control ===

bool PX4Controller::arm() {
    if (!connected_) return false;

    logMessage("Arming vehicle...");
    
    Action::Result result = action_->arm();
    if (result != Action::Result::Success) {
        logMessage("Arming failed: " + std::to_string(static_cast<int>(result)));
        return false;
    }

    logMessage("Vehicle armed successfully!");
    return true;
}

bool PX4Controller::disarm() {
    if (!connected_) return false;

    logMessage("Disarming vehicle...");
    
    Action::Result result = action_->disarm();
    if (result != Action::Result::Success) {
        logMessage("Disarming failed: " + std::to_string(static_cast<int>(result)));
        return false;
    }

    logMessage("Vehicle disarmed successfully!");
    return true;
}

bool PX4Controller::takeoff(float altitude_m) {
    if (!connected_) return false;

    logMessage("Taking off to " + std::to_string(altitude_m) + "m...");
    
    // Set takeoff altitude
    action_->set_takeoff_altitude(altitude_m);
    
    Action::Result result = action_->takeoff();
    if (result != Action::Result::Success) {
        logMessage("Takeoff failed: " + std::to_string(static_cast<int>(result)));
        return false;
    }

    // Wait for takeoff to complete
    int timeout = 30; // 30 seconds timeout
    while (timeout > 0 && !in_air_ && !shutdown_requested_) {
        sleep_for(milliseconds(500));
        timeout--;
    }
    
    if (in_air_) {
        logMessage("Takeoff completed successfully!");
        return true;
    } else {
        logMessage("Takeoff timeout or interrupted");
        return false;
    }
}

bool PX4Controller::land() {
    if (!connected_) return false;

    logMessage("Landing...");
    
    Action::Result result = action_->land();
    if (result != Action::Result::Success) {
        logMessage("Land command failed: " + std::to_string(static_cast<int>(result)));
        return false;
    }

    // Wait for landing to complete
    int timeout = 60; // 60 seconds timeout
    while (timeout > 0 && in_air_ && !shutdown_requested_) {
        sleep_for(milliseconds(500));
        timeout--;
    }
    
    if (!in_air_) {
        logMessage("Landing completed successfully!");
        return true;
    } else {
        logMessage("Landing timeout or interrupted");
        return false;
    }
}

bool PX4Controller::returnToLaunch() {
    if (!connected_) return false;

    logMessage("Returning to launch...");
    
    Action::Result result = action_->return_to_launch();
    if (result != Action::Result::Success) {
        logMessage("RTL failed: " + std::to_string(static_cast<int>(result)));
        return false;
    }

    logMessage("RTL command sent successfully!");
    return true;
}

void PX4Controller::emergencyStop() {
    if (!connected_) return;

    logMessage("EMERGENCY STOP!");
    
    // Try to disarm immediately
    action_->disarm();
    
    // Stop all ongoing operations
    stopTeleopMode();
    current_mode_ = FlightMode::IDLE;
}

// === Flight Modes ===

void PX4Controller::startTeleopMode() {
    if (!connected_ || teleop_active_) return;

    logMessage("Starting teleoperation mode...");
    logMessage("Controls: W/S=Forward/Back, A/D=Left/Right, Q/E=Up/Down, J/L=Yaw, SPACE=Stop, ESC=Exit");
    
    current_mode_ = FlightMode::MANUAL_TELEOP;
    teleop_active_ = true;
    
    // Configure terminal for raw input
    configureTerminal();
    
    // Start teleop thread
    teleop_thread_ = std::thread(&PX4Controller::teleopThreadFunction, this);
}

void PX4Controller::stopTeleopMode() {
    if (!teleop_active_) return;

    logMessage("Stopping teleoperation mode...");
    
    teleop_active_ = false;
    current_mode_ = FlightMode::IDLE;
    
    // Stop offboard if active
    stopOffboard();
    
    // Join teleop thread
    if (teleop_thread_.joinable()) {
        teleop_thread_.join();
    }
    
    // Restore terminal
    if (terminal_configured_) {
        restoreTerminal();
    }
    
    logMessage("Teleoperation stopped");
}

void PX4Controller::executeSampleFlight() {
    if (!connected_ || current_mode_ != FlightMode::IDLE) {
        logMessage("Cannot start sample flight - system busy or not connected");
        return;
    }

    logMessage("Starting sample flight...");
    current_mode_ = FlightMode::SAMPLE_FLIGHT;
    
    // Start sample flight in separate thread
    flight_thread_ = std::thread(&PX4Controller::sampleFlightThreadFunction, this);
}

// === Advanced Control ===

void PX4Controller::setOffboardPosition(float north_m, float east_m, float down_m, float yaw_deg) {
    if (!connected_) return;

    Offboard::PositionNedYaw position_ned_yaw{};
    position_ned_yaw.north_m = north_m;
    position_ned_yaw.east_m = east_m;
    position_ned_yaw.down_m = down_m;
    position_ned_yaw.yaw_deg = yaw_deg;

    offboard_->set_position_ned(position_ned_yaw);
}

void PX4Controller::setOffboardVelocity(float vx_m_s, float vy_m_s, float vz_m_s, float yaw_rate_deg_s) {
    if (!connected_) return;

    Offboard::VelocityBodyYawspeed velocity_body_yawspeed{};
    velocity_body_yawspeed.forward_m_s = vx_m_s;
    velocity_body_yawspeed.right_m_s = vy_m_s;
    velocity_body_yawspeed.down_m_s = vz_m_s;
    velocity_body_yawspeed.yawspeed_deg_s = yaw_rate_deg_s;

    offboard_->set_velocity_body(velocity_body_yawspeed);
}

bool PX4Controller::startOffboard() {
    if (!connected_) return false;

    // Set initial setpoint
    setOffboardVelocity(0.0f, 0.0f, 0.0f, 0.0f);
    
    Offboard::Result result = offboard_->start();
    if (result != Offboard::Result::Success) {
        logMessage("Offboard start failed: " + std::to_string(static_cast<int>(result)));
        return false;
    }

    logMessage("Offboard mode started");
    return true;
}

bool PX4Controller::stopOffboard() {
    if (!connected_) return false;

    Offboard::Result result = offboard_->stop();
    if (result != Offboard::Result::Success) {
        logMessage("Offboard stop failed: " + std::to_string(static_cast<int>(result)));
        return false;
    }

    logMessage("Offboard mode stopped");
    return true;
}

// === Mission Management ===

bool PX4Controller::createSampleMission(int pattern, float size_m) {
    if (!connected_) return false;

    logMessage("Creating sample mission (pattern=" + std::to_string(pattern) + ", size=" + std::to_string(size_m) + "m)");

    auto home_position = getCurrentPosition();
    std::vector<Mission::MissionItem> mission_items;

    switch (pattern) {
        case 0: { // Square pattern
            auto wp1 = generateWaypoint(size_m, 0.0f);
            auto wp2 = generateWaypoint(size_m, size_m);
            auto wp3 = generateWaypoint(0.0f, size_m);
            auto wp4 = generateWaypoint(0.0f, 0.0f);

            for (auto& wp : {wp1, wp2, wp3, wp4}) {
                Mission::MissionItem item{};
                item.latitude_deg = wp.first;
                item.longitude_deg = wp.second;
                item.relative_altitude_m = 15.0f;
                item.speed_m_s = 5.0f;
                item.is_fly_through = true;
                mission_items.push_back(item);
            }
            break;
        }
        case 1: { // Circle pattern (approximated with 8 points)
            for (int i = 0; i < 8; ++i) {
                float angle = i * M_PI / 4.0f; // 45 degree increments
                float north = size_m * cos(angle);
                float east = size_m * sin(angle);
                auto wp = generateWaypoint(north, east);

                Mission::MissionItem item{};
                item.latitude_deg = wp.first;
                item.longitude_deg = wp.second;
                item.relative_altitude_m = 12.0f;
                item.speed_m_s = 4.0f;
                item.is_fly_through = true;
                mission_items.push_back(item);
            }
            break;
        }
        case 2: { // Figure-8 pattern
            for (int i = 0; i < 16; ++i) {
                float t = i * M_PI / 8.0f; // Parameter for figure-8
                float north = size_m * sin(t);
                float east = size_m * sin(t) * cos(t);
                auto wp = generateWaypoint(north, east);

                Mission::MissionItem item{};
                item.latitude_deg = wp.first;
                item.longitude_deg = wp.second;
                item.relative_altitude_m = 10.0f + 5.0f * sin(t); // Varying altitude
                item.speed_m_s = 3.0f;
                item.is_fly_through = true;
                mission_items.push_back(item);
            }
            break;
        }
        default:
            logMessage("Unknown pattern type, using square");
            return createSampleMission(0, size_m);
    }

    return uploadMission(mission_items);
}

bool PX4Controller::uploadMission(const std::vector<Mission::MissionItem>& mission_items) {
    if (!connected_) return false;

    Mission::MissionPlan mission_plan{};
    mission_plan.mission_items = mission_items;

    logMessage("Uploading mission with " + std::to_string(mission_items.size()) + " waypoints...");
    
    Mission::Result result = mission_->upload_mission(mission_plan);
    if (result != Mission::Result::Success) {
        logMessage("Mission upload failed: " + std::to_string(static_cast<int>(result)));
        return false;
    }

    logMessage("Mission uploaded successfully!");
    return true;
}

bool PX4Controller::startMission() {
    if (!connected_) return false;

    logMessage("Starting mission...");
    
    Mission::Result result = mission_->start_mission();
    if (result != Mission::Result::Success) {
        logMessage("Mission start failed: " + std::to_string(static_cast<int>(result)));
        return false;
    }

    // Subscribe to mission progress
    mission_->subscribe_mission_progress([this](Mission::MissionProgress progress) {
        logMessage("Mission progress: " + std::to_string(progress.current) + "/" + std::to_string(progress.total));
    });

    logMessage("Mission started successfully!");
    return true;
}

bool PX4Controller::pauseMission() {
    if (!connected_) return false;

    Mission::Result result = mission_->pause_mission();
    if (result != Mission::Result::Success) {
        logMessage("Mission pause failed: " + std::to_string(static_cast<int>(result)));
        return false;
    }

    logMessage("Mission paused");
    return true;
}

bool PX4Controller::clearMission() {
    if (!connected_) return false;

    Mission::Result result = mission_->clear_mission();
    if (result != Mission::Result::Success) {
        logMessage("Mission clear failed: " + std::to_string(static_cast<int>(result)));
        return false;
    }

    logMessage("Mission cleared");
    return true;
}

// === Camera and Gimbal ===

bool PX4Controller::controlGimbal(float tilt_deg, float pan_deg) {
    if (!connected_) return false;

    logMessage("Controlling gimbal: tilt=" + std::to_string(tilt_deg) + "°, pan=" + std::to_string(pan_deg) + "°");

    // Control gimbal attitude using set_angles (new API)
    // Gimbal ID 1 is typically the gimbal component
    Gimbal::Result result = gimbal_->set_angles(
        1, // gimbal_id
        0.0f, // roll_deg
        tilt_deg, // pitch_deg
        pan_deg, // yaw_deg
        Gimbal::GimbalMode::YawLock, // gimbal_mode
        Gimbal::SendMode::Once // send_mode
    );
    if (result != Gimbal::Result::Success) {
        logMessage("Failed to control gimbal: " + std::to_string(static_cast<int>(result)));
        return false;
    }

    return true;
}

bool PX4Controller::takePhoto() {
    if (!connected_) return false;

    logMessage("Taking photo...");

    // Set camera to photo mode (new API requires component_id)
    Camera::Result result = camera_->set_mode(1, Camera::Mode::Photo);
    if (result != Camera::Result::Success) {
        logMessage("Failed to set camera mode: " + std::to_string(static_cast<int>(result)));
        return false;
    }

    // Take photo (new API requires component_id)
    result = camera_->take_photo(1);
    if (result != Camera::Result::Success) {
        logMessage("Failed to take photo: " + std::to_string(static_cast<int>(result)));
        return false;
    }

    logMessage("Photo taken successfully!");
    return true;
}

bool PX4Controller::startVideoRecording() {
    if (!connected_) return false;

    logMessage("Starting video recording...");

    // Set camera to video mode (new API requires component_id)
    Camera::Result result = camera_->set_mode(1, Camera::Mode::Video);
    if (result != Camera::Result::Success) {
        logMessage("Failed to set camera mode: " + std::to_string(static_cast<int>(result)));
        return false;
    }

    // Start video recording (new API requires component_id)
    result = camera_->start_video(1);
    if (result != Camera::Result::Success) {
        logMessage("Failed to start video: " + std::to_string(static_cast<int>(result)));
        return false;
    }

    logMessage("Video recording started!");
    return true;
}

bool PX4Controller::stopVideoRecording() {
    if (!connected_) return false;

    logMessage("Stopping video recording...");

    // Stop video recording (new API requires component_id)
    Camera::Result result = camera_->stop_video(1);
    if (result != Camera::Result::Success) {
        logMessage("Failed to stop video: " + std::to_string(static_cast<int>(result)));
        return false;
    }

    logMessage("Video recording stopped!");
    return true;
}

// === Parameter Management ===

int PX4Controller::getParamInt(const std::string& name) {
    if (!connected_) return 0;

    auto result = param_->get_param_int(name);
    if (result.first == Param::Result::Success) {
        return result.second;
    }
    return 0;
}

float PX4Controller::getParamFloat(const std::string& name) {
    if (!connected_) return 0.0f;

    auto result = param_->get_param_float(name);
    if (result.first == Param::Result::Success) {
        return result.second;
    }
    return 0.0f;
}

bool PX4Controller::setParamInt(const std::string& name, int value) {
    if (!connected_) return false;

    Param::Result result = param_->set_param_int(name, value);
    return (result == Param::Result::Success);
}

bool PX4Controller::setParamFloat(const std::string& name, float value) {
    if (!connected_) return false;

    Param::Result result = param_->set_param_float(name, value);
    return (result == Param::Result::Success);
}

// === Telemetry Access ===

Telemetry::Position PX4Controller::getCurrentPosition() {
    if (connected_) {
        return telemetry_->position();
    }
    return Telemetry::Position{};
}

Telemetry::EulerAngle PX4Controller::getCurrentAttitude() {
    if (connected_) {
        return telemetry_->attitude_euler();
    }
    return Telemetry::EulerAngle{};
}

Telemetry::VelocityNed PX4Controller::getCurrentVelocity() {
    if (connected_) {
        return telemetry_->velocity_ned();
    }
    return Telemetry::VelocityNed{};
}

Telemetry::Battery PX4Controller::getBatteryStatus() {
    if (connected_) {
        return telemetry_->battery();
    }
    return Telemetry::Battery{};
}

Telemetry::FlightMode PX4Controller::getPX4FlightMode() {
    if (connected_) {
        return telemetry_->flight_mode();
    }
    return Telemetry::FlightMode::Unknown;
}

// === Callback Registration ===

void PX4Controller::setLogCallback(std::function<void(const std::string&)> callback) {
    log_callback_ = callback;
}

void PX4Controller::setPositionCallback(std::function<void(float, float, float)> callback) {
    position_callback_ = callback;
}

void PX4Controller::setBatteryCallback(std::function<void(float)> callback) {
    battery_callback_ = callback;
}

// === Utility Functions ===

void PX4Controller::printStatus() {
    if (!connected_) {
        logMessage("Status: Not connected");
        return;
    }

    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "=== SYSTEM STATUS ===" << std::endl;
    ss << "Connected: " << (connected_ ? "YES" : "NO") << std::endl;
    ss << "Armed: " << (armed_ ? "YES" : "NO") << std::endl;
    ss << "In Air: " << (in_air_ ? "YES" : "NO") << std::endl;
    ss << "Mode: ";
    
    switch (current_mode_.load()) {
        case FlightMode::IDLE: ss << "IDLE"; break;
        case FlightMode::MANUAL_TELEOP: ss << "TELEOP"; break;
        case FlightMode::SAMPLE_FLIGHT: ss << "SAMPLE_FLIGHT"; break;
        case FlightMode::MISSION: ss << "MISSION"; break;
        case FlightMode::OFFBOARD_DEMO: ss << "OFFBOARD_DEMO"; break;
    }
    
    ss << std::endl;
    ss << "Position: " << latitude_.load() << ", " << longitude_.load() << std::endl;
    ss << "Altitude: " << altitude_m_.load() << "m" << std::endl;
    ss << "Battery: " << battery_percent_.load() << "%" << std::endl;
    
    logMessage(ss.str());
}

void PX4Controller::requestShutdown() {
    shutdown_requested_ = true;
    stopTeleopMode();
    
    if (flight_thread_.joinable()) {
        flight_thread_.join();
    }
}

// === Private Helper Methods ===

void PX4Controller::initializePlugins() {
    action_ = std::make_unique<Action>(system_);
    telemetry_ = std::make_unique<Telemetry>(system_);
    offboard_ = std::make_unique<Offboard>(system_);
    mission_ = std::make_unique<Mission>(system_);
    follow_me_ = std::make_unique<FollowMe>(system_);
    gimbal_ = std::make_unique<Gimbal>(system_);
    camera_ = std::make_unique<Camera>(system_);
    info_ = std::make_unique<Info>(system_);
    calibration_ = std::make_unique<Calibration>(system_);
    param_ = std::make_unique<Param>(system_);
    
    logMessage("All plugins initialized");
}

void PX4Controller::configureTerminal() {
    tcgetattr(STDIN_FILENO, &original_termios_);
    
    struct termios new_termios = original_termios_;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    new_termios.c_cc[VMIN] = 0;
    new_termios.c_cc[VTIME] = 0;
    
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    
    terminal_configured_ = true;
}

void PX4Controller::restoreTerminal() {
    if (terminal_configured_) {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
        terminal_configured_ = false;
    }
}

bool PX4Controller::kbhit() {
    int ch = getchar();
    if (ch != EOF) {
        ungetc(ch, stdin);
        return true;
    }
    return false;
}

char PX4Controller::getch() {
    return getchar();
}

void PX4Controller::teleopThreadFunction() {
    logMessage("Teleop thread started");
    
    // Start offboard mode for velocity control
    if (!startOffboard()) {
        logMessage("Failed to start offboard mode for teleop");
        teleop_active_ = false;
        return;
    }
    
    const float MAX_VELOCITY = 5.0f; // m/s
    const float MAX_YAW_RATE = 45.0f; // deg/s
    
    while (teleop_active_ && !shutdown_requested_) {
        char key = getch();
        
        // Reset velocities
        current_teleop_cmd_ = TeleopCommand{};
        
        switch (key) {
            case 'w': case 'W':
                current_teleop_cmd_.velocity_x_m_s = MAX_VELOCITY;
                break;
            case 's': case 'S':
                current_teleop_cmd_.velocity_x_m_s = -MAX_VELOCITY;
                break;
            case 'a': case 'A':
                current_teleop_cmd_.velocity_y_m_s = -MAX_VELOCITY;
                break;
            case 'd': case 'D':
                current_teleop_cmd_.velocity_y_m_s = MAX_VELOCITY;
                break;
            case 'q': case 'Q':
                current_teleop_cmd_.velocity_z_m_s = -MAX_VELOCITY;
                break;
            case 'e': case 'E':
                current_teleop_cmd_.velocity_z_m_s = MAX_VELOCITY;
                break;
            case 'j': case 'J':
                current_teleop_cmd_.yaw_rate_deg_s = -MAX_YAW_RATE;
                break;
            case 'l': case 'L':
                current_teleop_cmd_.yaw_rate_deg_s = MAX_YAW_RATE;
                break;
            case ' ':
                // Stop all movement
                current_teleop_cmd_ = TeleopCommand{};
                break;
            case 27: // ESC
                teleop_active_ = false;
                continue;
        }
        
        // Send velocity command
        setOffboardVelocity(
            current_teleop_cmd_.velocity_x_m_s,
            current_teleop_cmd_.velocity_y_m_s, 
            current_teleop_cmd_.velocity_z_m_s,
            current_teleop_cmd_.yaw_rate_deg_s
        );
        
        sleep_for(milliseconds(50)); // 20Hz update rate
    }
    
    // Stop movement and exit offboard
    setOffboardVelocity(0.0f, 0.0f, 0.0f, 0.0f);
    sleep_for(milliseconds(100));
    stopOffboard();
    
    logMessage("Teleop thread ended");
}

void PX4Controller::sampleFlightThreadFunction() {
    logMessage("=== STARTING SAMPLE FLIGHT ===");
    
    try {
        // Phase 1: System check and takeoff
        logMessage("Phase 1: System preparation");
        if (!waitForReady(30)) {
            logMessage("System not ready, aborting sample flight");
            return;
        }
        
        if (!arm()) {
            logMessage("Failed to arm, aborting sample flight");
            return;
        }
        
        if (!takeoff(15.0f)) {
            logMessage("Failed to takeoff, aborting sample flight");
            return;
        }
        
        sleep_for(seconds(3));
        
        // Phase 2: Offboard figure-8 pattern
        logMessage("Phase 2: Figure-8 pattern with offboard control");
        if (startOffboard()) {
            auto start_pos = getCurrentPosition();
            float start_alt = start_pos.relative_altitude_m;
            
            for (int i = 0; i < 32 && !shutdown_requested_; ++i) {
                float t = i * M_PI / 16.0f;
                float north = 15.0f * sin(t);
                float east = 10.0f * sin(t) * cos(t);
                float altitude_offset = 3.0f * sin(t * 2.0f);
                float yaw = 90.0f * sin(t / 2.0f);
                
                setOffboardPosition(north, east, -(start_alt + altitude_offset), yaw);
                sleep_for(seconds(2));
                
                if (i % 8 == 0) {
                    logMessage("Figure-8 progress: " + std::to_string((i * 100) / 32) + "%");
                }
            }
            
            stopOffboard();
            sleep_for(seconds(2));
        }
        
        // Phase 3: Gimbal control and photo capture
        logMessage("Phase 3: Gimbal control and photography");
        for (int angle = -60; angle <= 0; angle += 20) {
            controlGimbal(angle, 0.0f);
            sleep_for(seconds(1));
            takePhoto();
            sleep_for(seconds(1));
        }
        
        // Phase 4: Mission execution
        logMessage("Phase 4: Autonomous mission");
        if (createSampleMission(1, 20.0f)) { // Circle pattern
            current_mode_ = FlightMode::MISSION;
            if (startMission()) {
                // Monitor mission for 60 seconds max
                int mission_timeout = 120;
                while (mission_timeout > 0 && !shutdown_requested_) {
                    sleep_for(seconds(1));
                    mission_timeout--;
                }
            }
        }
        
        // Phase 5: Return and land
        logMessage("Phase 5: Return to launch and landing");
        returnToLaunch();
        sleep_for(seconds(10));
        
        if (in_air_) {
            land();
        }
        
        disarm();
        
        logMessage("=== SAMPLE FLIGHT COMPLETED SUCCESSFULLY ===");
        
    } catch (const std::exception& e) {
        logMessage("Sample flight error: " + std::string(e.what()));
    }
    
    current_mode_ = FlightMode::IDLE;
}

void PX4Controller::logMessage(const std::string& message) {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << "[" << std::put_time(std::localtime(&time_t), "%H:%M:%S")
       << "." << std::setfill('0') << std::setw(3) << ms.count() << "] "
       << message;
    
    std::string timestamped_message = ss.str();
    std::cout << timestamped_message << std::endl;
    
    if (log_callback_) {
        log_callback_(timestamped_message);
    }
}

float PX4Controller::calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0; // Earth radius in meters
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dLat/2) * sin(dLat/2) + cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) * sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
}

std::pair<double, double> PX4Controller::generateWaypoint(float north_offset_m, float east_offset_m) {
    auto current_pos = getCurrentPosition();
    
    // Approximate GPS coordinate offsets (this is a simplification)
    double lat_offset = north_offset_m / 111320.0; // ~111.32 km per degree latitude
    double lon_offset = east_offset_m / (111320.0 * cos(current_pos.latitude_deg * M_PI / 180.0));
    
    return {current_pos.latitude_deg + lat_offset, current_pos.longitude_deg + lon_offset};
}