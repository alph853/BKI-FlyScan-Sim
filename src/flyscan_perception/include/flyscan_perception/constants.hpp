#ifndef FLYSCAN_PERCEPTION_CONSTANTS_HPP_
#define FLYSCAN_PERCEPTION_CONSTANTS_HPP_

#include <string>
#include <vector>

namespace flyscan {
namespace perception {

constexpr const char* VIDEO_TOPIC = "/camera/image_raw";

// Display and UI constants
constexpr int DEFAULT_WINDOW_WIDTH = 1280;
constexpr int DEFAULT_WINDOW_HEIGHT = 720;
constexpr int WAITING_FRAME_WIDTH = 1280;
constexpr int WAITING_FRAME_HEIGHT = 720;

// Video recording constants
constexpr double DEFAULT_FPS = 30.0;
constexpr char FOURCC_CODEC[4] = {'m', 'p', '4', 'v'};

// Screenshot and recording filename patterns
constexpr const char* SCREENSHOT_PREFIX = "drone_screenshot_";
constexpr const char* VIDEO_PREFIX = "drone_video_";
constexpr const char* SCREENSHOT_EXTENSION = ".jpg";
constexpr const char* VIDEO_EXTENSION = ".mp4";
constexpr int FILENAME_PADDING = 6;

// Text overlay constants
constexpr int FRAME_TEXT_X = 10;
constexpr int FRAME_TEXT_Y = 30;
constexpr int REC_CIRCLE_RADIUS = 10;
constexpr int REC_CIRCLE_X_OFFSET = 30;
constexpr int REC_CIRCLE_Y_OFFSET = 30;
constexpr int REC_TEXT_X_OFFSET = 70;
constexpr int REC_TEXT_Y_OFFSET = 35;
constexpr int TOPIC_TEXT_X = 10;
constexpr int TOPIC_TEXT_Y_OFFSET = 20;
constexpr int HELP_TEXT_X = 10;
constexpr int HELP_TEXT_Y_OFFSET = 40;

// Text formatting constants
constexpr double FRAME_TEXT_SCALE = 0.7;
constexpr double REC_TEXT_SCALE = 0.5;
constexpr double TOPIC_TEXT_SCALE = 0.5;
constexpr double HELP_TEXT_SCALE = 0.4;
constexpr double WAITING_TEXT_SCALE = 1.0;
constexpr double WAITING_TOPIC_SCALE = 0.7;
constexpr double DETECTION_LABEL_SCALE = 0.5;
constexpr int TEXT_THICKNESS = 2;
constexpr int THIN_TEXT_THICKNESS = 1;

// Waiting screen text positions
constexpr int WAITING_TEXT_X = 50;
constexpr int WAITING_TEXT_Y = 200;
constexpr int WAITING_TOPIC_Y = 250;
constexpr int WAITING_QUIT_Y = 300;

// YOLO model constants
constexpr const char* MODEL_PATH = "src/flyscan_perception/best.onnx";
constexpr int YOLO_INPUT_SIZE = 640;
constexpr float CONFIDENCE_THRESHOLD = 0.5f;
constexpr float NMS_THRESHOLD = 0.4f;

// YOLO model parameters
constexpr int YOLO_NUM_CHANNELS = 3;
constexpr int YOLO_BBOX_COORDS = 4;
constexpr float YOLO_NORMALIZE_FACTOR = 255.0f;

// Detection drawing constants
constexpr int BBOX_THICKNESS = 2;
constexpr int LABEL_Y_OFFSET = 10;

// Keyboard controls
constexpr char KEY_QUIT_UPPER = 'Q';
constexpr char KEY_QUIT_LOWER = 'q';
constexpr char KEY_SCREENSHOT_UPPER = 'S';
constexpr char KEY_SCREENSHOT_LOWER = 's';
constexpr char KEY_RECORD_UPPER = 'R';
constexpr char KEY_RECORD_LOWER = 'r';
constexpr char KEY_INFO_UPPER = 'I';
constexpr char KEY_INFO_LOWER = 'i';
constexpr char KEY_INCREASE_SKIP = '+';
constexpr char KEY_INCREASE_SKIP_ALT = '=';
constexpr char KEY_DECREASE_SKIP = '-';
constexpr char KEY_DECREASE_SKIP_ALT = '_';

// Frame skip limits
constexpr int MAX_INFERENCE_FRAME_SKIP = 10;
constexpr int MIN_INFERENCE_FRAME_SKIP = 1;

// OpenCV wait key timeout
constexpr int CV_WAIT_KEY_TIMEOUT = 1;
constexpr int CV_KEY_MASK = 0xFF;

// ONNX Runtime constants
constexpr const char* ORT_ENV_NAME = "YOLOv8";

// Control help text
constexpr const char* CONTROLS_HELP_TEXT = "Controls: S=Screenshot, R=Record, I=Info, +/- Skip, Q=Quit";

// Class names for detection (QR codes and barcodes)
const std::vector<std::string> CLASS_NAMES = {
    "QR", "Barcode"
};

// Console output messages
constexpr const char* CONSOLE_HEADER = "=== Gazebo Video Streamer with YOLOv8 Detection ===";
constexpr const char* CONTROLS_HEADER = "Controls:";
constexpr const char* CONTROL_SCREENSHOT = "  S - Save screenshot";
constexpr const char* CONTROL_RECORD = "  R - Toggle recording";
constexpr const char* CONTROL_INFO = "  I - Toggle info overlay";
constexpr const char* CONTROL_SPEED_UP = "  + - Increase frame skip (faster FPS, less frequent detection)";
constexpr const char* CONTROL_SPEED_DOWN = "  - - Decrease frame skip (slower FPS, more frequent detection)";
constexpr const char* CONTROL_QUIT = "  Q - Quit";

} // namespace perception
} // namespace flyscan

#endif // FLYSCAN_PERCEPTION_CONSTANTS_HPP_