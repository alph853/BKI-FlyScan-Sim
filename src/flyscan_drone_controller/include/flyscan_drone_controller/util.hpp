#pragma once

#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>

namespace flyscan_drone_controller {
namespace util {

struct TerminalController {
    static bool initialize();
    static void cleanup();
    static bool kbhit();
    static char getch();
    static void printTeleopInstructions();
    
private:
    static struct termios original_termios_;
    static bool terminal_configured_;
};

class MathUtils {
public:
    static float constrainAngle(float angle);
    static float calculateDistance3D(float x1, float y1, float z1, float x2, float y2, float z2);
    static float calculateDistance2D(float x1, float y1, float x2, float y2);
    static bool isWithinTolerance(float value, float target, float tolerance);
    
    static constexpr float DEG_TO_RAD = M_PI / 180.0f;
    static constexpr float RAD_TO_DEG = 180.0f / M_PI;
};

class StringUtils {
public:
    static std::string formatPosition(float north, float east, float down, int precision = 2);
    static std::string formatFloat(float value, int precision = 2);
    static std::string formatPercentage(float value, int precision = 1);
};

void printUsage(const std::string& bin_name);

} // namespace util
} // namespace flyscan_drone_controller