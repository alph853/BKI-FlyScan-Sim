/**
 * @file main.cpp
 * @brief Main application for PX4 MAVSDK controller with teleop and sample flight modes
 * @author Your Name
 * @date 2025
 * 
 * Usage: ./px4_control <connection_url> <mode>
 * Modes: teleop, sample, demo
 * 
 * Example: ./px4_control udp://:14540 teleop
 */

#include "flyscan_drone_controller/px4_controller.hpp"
#include <iostream>
#include <string>
#include <signal.h>

// Global controller instance for signal handling
std::unique_ptr<PX4Controller> g_controller = nullptr;

/**
 * @brief Signal handler for graceful shutdown
 */
void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
    if (g_controller) {
        g_controller->emergencyStop();
        g_controller->requestShutdown();
    }
    exit(0);
}

/**
 * @brief Print usage information
 */
void printUsage(const std::string& program_name) {
    std::cout << "PX4 MAVSDK Controller" << std::endl;
    std::cout << "=====================" << std::endl << std::endl;
    
    std::cout << "Usage: " << program_name << " <connection_url> <mode>" << std::endl << std::endl;
    
    std::cout << "Connection URLs:" << std::endl;
    std::cout << "  udp://:14540          - UDP connection (PX4 SITL default)" << std::endl;
    std::cout << "  tcp://192.168.1.100   - TCP connection to specific IP" << std::endl;
    std::cout << "  serial:///dev/ttyUSB0:57600 - Serial connection" << std::endl << std::endl;
    
    std::cout << "Modes:" << std::endl;
    std::cout << "  teleop    - Manual teleoperation with keyboard controls" << std::endl;
    std::cout << "            Controls: W/S=Forward/Back, A/D=Left/Right" << std::endl;
    std::cout << "                     Q/E=Up/Down, J/L=Yaw, SPACE=Stop, ESC=Exit" << std::endl;
    std::cout << "  sample    - Execute predefined sample flight pattern" << std::endl;
    std::cout << "            Includes: Takeoff → Figure-8 → Photos → Mission → Land" << std::endl;
    std::cout << "  demo      - Interactive demo with menu options" << std::endl << std::endl;
    
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << program_name << " udp://:14540 teleop" << std::endl;
    std::cout << "  " << program_name << " udp://:14540 sample" << std::endl;
    std::cout << "  " << program_name << " tcp://192.168.1.100:5760 demo" << std::endl << std::endl;
    
    std::cout << "Prerequisites:" << std::endl;
    std::cout << "  - PX4 SITL running: make px4_sitl gazebo" << std::endl;
    std::cout << "  - Or real drone with MAVSDK-compatible autopilot" << std::endl;
}

/**
 * @brief Interactive demo mode with menu
 */
void runDemoMode(PX4Controller& controller) {
    std::cout << "\n=== INTERACTIVE DEMO MODE ===" << std::endl;
    
    while (!controller.isShutdownRequested()) {
        std::cout << "\nSelect an option:" << std::endl;
        std::cout << "1. System Info" << std::endl;
        std::cout << "2. Arm/Disarm" << std::endl;
        std::cout << "3. Takeoff" << std::endl;
        std::cout << "4. Land" << std::endl;
        std::cout << "5. Start Teleop Mode" << std::endl;
        std::cout << "6. Execute Sample Flight" << std::endl;
        std::cout << "7. Create & Start Mission" << std::endl;
        std::cout << "8. Take Photo" << std::endl;
        std::cout << "9. Control Gimbal" << std::endl;
        std::cout << "10. Print Status" << std::endl;
        std::cout << "11. Emergency Stop" << std::endl;
        std::cout << "0. Exit" << std::endl;
        std::cout << "Choice: ";
        
        int choice;
        std::cin >> choice;
        
        switch (choice) {
            case 1:
                controller.getSystemInfo();
                break;
                
            case 2: {
                char action;
                std::cout << "Arm (a) or Disarm (d)? ";
                std::cin >> action;
                if (action == 'a' || action == 'A') {
                    controller.arm();
                } else {
                    controller.disarm();
                }
                break;
            }
            
            case 3: {
                float altitude;
                std::cout << "Takeoff altitude (m): ";
                std::cin >> altitude;
                controller.takeoff(altitude);
                break;
            }
            
            case 4:
                controller.land();
                break;
                
            case 5:
                std::cout << "Starting teleop mode... (ESC to exit)" << std::endl;
                controller.startTeleopMode();
                // Wait for teleop to complete
                while (controller.getCurrentMode() == FlightMode::MANUAL_TELEOP) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
                break;
                
            case 6:
                std::cout << "Executing sample flight..." << std::endl;
                controller.executeSampleFlight();
                break;
                
            case 7: {
                int pattern;
                float size;
                std::cout << "Mission pattern (0=square, 1=circle, 2=figure8): ";
                std::cin >> pattern;
                std::cout << "Pattern size (m): ";
                std::cin >> size;
                
                if (controller.createSampleMission(pattern, size)) {
                    controller.startMission();
                }
                break;
            }
            
            case 8:
                controller.takePhoto();
                break;
                
            case 9: {
                float tilt, pan;
                std::cout << "Gimbal tilt angle (deg, -90 to +30): ";
                std::cin >> tilt;
                std::cout << "Gimbal pan angle (deg): ";
                std::cin >> pan;
                controller.controlGimbal(tilt, pan);
                break;
            }
            
            case 10:
                controller.printStatus();
                break;
                
            case 11:
                controller.emergencyStop();
                break;
                
            case 0:
                std::cout << "Exiting demo mode..." << std::endl;
                return;
                
            default:
                std::cout << "Invalid choice!" << std::endl;
                break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

/**
 * @brief Main function
 */
int main(int argc, char* argv[]) {
    // Check arguments
    if (argc != 3) {
        printUsage(argv[0]);
        return 1;
    }
    
    std::string connection_url = argv[1];
    std::string mode = argv[2];
    
    // Setup signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    std::cout << "PX4 MAVSDK Controller Starting..." << std::endl;
    std::cout << "Connection: " << connection_url << std::endl;
    std::cout << "Mode: " << mode << std::endl << std::endl;
    
    try {
        // Create controller
        g_controller = std::make_unique<PX4Controller>();
        
        // Connect to autopilot
        if (!g_controller->connect(connection_url)) {
            std::cerr << "Failed to connect to autopilot!" << std::endl;
            return 1;
        }
        
        // Setup telemetry
        g_controller->setupTelemetry();
        
        // Wait for system ready
        if (!g_controller->waitForReady(30)) {
            std::cerr << "System not ready within timeout!" << std::endl;
            return 1;
        }
        
        // Get system info
        g_controller->getSystemInfo();
        
        // Execute based on mode
        if (mode == "teleop") {
            std::cout << "\n=== TELEOPERATION MODE ===" << std::endl;
            std::cout << "Make sure the drone is armed and in the air before starting teleop!" << std::endl;
            std::cout << "Press Enter to continue..." << std::endl;
            std::cin.ignore();
            std::cin.get();
            
            g_controller->startTeleopMode();
            
            // Wait for teleop to complete
            while (g_controller->getCurrentMode() == FlightMode::MANUAL_TELEOP && 
                   !g_controller->isShutdownRequested()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            
        } else if (mode == "sample") {
            std::cout << "\n=== SAMPLE FLIGHT MODE ===" << std::endl;
            std::cout << "Starting automated sample flight sequence..." << std::endl;
            
            g_controller->executeSampleFlight();
            
            // Wait for sample flight to complete
            while (g_controller->getCurrentMode() == FlightMode::SAMPLE_FLIGHT && 
                   !g_controller->isShutdownRequested()) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            
        } else if (mode == "demo") {
            runDemoMode(*g_controller);
            
        } else {
            std::cerr << "Unknown mode: " << mode << std::endl;
            std::cerr << "Valid modes: teleop, sample, demo" << std::endl;
            return 1;
        }
        
        std::cout << "\nOperation completed. Cleaning up..." << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    // Cleanup
    if (g_controller) {
        g_controller->requestShutdown();
        g_controller.reset();
    }
    
    std::cout << "PX4 Controller exited successfully." << std::endl;
    return 0;
}