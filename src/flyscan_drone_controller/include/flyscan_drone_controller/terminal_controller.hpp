#pragma once

#include <termios.h>

namespace flyscan_drone_controller {
namespace util {

/**
 * @brief Terminal controller for handling keyboard input in non-blocking mode
 * 
 * This class provides functionality to configure the terminal for non-blocking
 * keyboard input, useful for teleop control applications. It manages terminal
 * settings and provides methods to detect key presses and read characters.
 */
struct TerminalController {
    /**
     * @brief Initialize the terminal controller
     * 
     * Configures the terminal to disable canonical mode and echo,
     * and sets it to non-blocking mode for keyboard input detection.
     * 
     * @return true if initialization successful, false otherwise
     */
    static bool initialize();
    
    /**
     * @brief Cleanup and restore original terminal settings
     * 
     * Restores the terminal to its original configuration before
     * the controller was initialized.
     */
    static void cleanup();
    
    /**
     * @brief Check if a key has been pressed
     * 
     * Non-blocking function to detect if there is keyboard input available.
     * 
     * @return true if a key press is available, false otherwise
     */
    static bool kbhit();
    
    /**
     * @brief Read a character from keyboard input
     * 
     * Reads a single character from keyboard input if available.
     * Should be called after kbhit() returns true.
     * 
     * @return The character pressed, or 0 if no input available
     */
    static char getch();
    
    /**
     * @brief Print teleop control instructions
     * 
     * Displays the available keyboard controls for position-based
     * teleop operation.
     */
    static void printTeleopInstructions();
    
private:
    static struct termios original_termios_;  ///< Original terminal settings
    static bool terminal_configured_;         ///< Flag indicating if terminal is configured
};

} // namespace util
} // namespace flyscan_drone_controller