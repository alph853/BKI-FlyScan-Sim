# DetectionArray.msg
# Header with timestamp and frame info
std_msgs/Header header
# Array of detected objects
flyscan_interfaces/Detection[] detections
# Frame count when detections were captured
int32 frame_count