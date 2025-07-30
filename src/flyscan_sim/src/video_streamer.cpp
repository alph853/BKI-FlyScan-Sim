#include <rclcpp/qos.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <sstream>
#include <iomanip>

#include "flyscan_sim/video_streamer.hpp"

namespace flyscan {
namespace sim {

VideoStreamer::VideoStreamer()
    : Node("video_streamer")
    , frame_count_(0)
    , fps_frame_count_(0)
    , current_fps_(0.0)
    , window_name_("Drone Camera Feed with Detections")
    , show_info_overlay_(true)
{
    RCLCPP_INFO(this->get_logger(), "VideoStreamer initializing");

    video_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 
        rclcpp::SensorDataQoS(),
        std::bind(&VideoStreamer::VideoCallback, this, std::placeholders::_1)
    );
    
    last_fps_time_ = std::chrono::steady_clock::now();

    detection_subscription_ = this->create_subscription<flyscan_interfaces::msg::DetectionArray>(
        "detections",
        10,
        std::bind(&VideoStreamer::DetectionCallback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(this->get_logger(), "Waiting for video stream and detection messages...");
}

VideoStreamer::~VideoStreamer()
{
    cv::destroyAllWindows();
}

void VideoStreamer::VideoCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        current_frame_ = cv_ptr_->image;
        frame_count_++;
        UpdateFpsCalculation();
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error converting image: %s", e.what());
    }
}

void VideoStreamer::UpdateFpsCalculation()
{
    fps_frame_count_++;
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_fps_time_).count();
    
    // Update FPS every 1000ms (1 second)
    if (elapsed >= 1000) {
        current_fps_ = static_cast<double>(fps_frame_count_) * 1000.0 / elapsed;
        fps_frame_count_ = 0;
        last_fps_time_ = current_time;
    }
}

cv::Mat VideoStreamer::AddInfoOverlay(const cv::Mat& frame)
{
    if (!show_info_overlay_) {
        return frame;
    }

    cv::Mat overlay_frame = frame.clone();
    std::stringstream fps_stream;
    fps_stream << "FPS: " << std::fixed << std::setprecision(1) << current_fps_;
    cv::putText(overlay_frame, fps_stream.str(),
               cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    
    cv::putText(overlay_frame, "Topic: /camera/image_raw",
               cv::Point(10, frame.rows - 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    cv::putText(overlay_frame, "Controls: S=Screenshot, I=Info, Q=Quit",
               cv::Point(10, frame.rows - 40), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
    
    return overlay_frame;
}

void VideoStreamer::SaveScreenshot()
{
    if (!current_frame_.empty()) {
        std::stringstream ss;
        ss << "drone_screenshot_" << std::setfill('0') << std::setw(6) << frame_count_ << ".jpg";
        std::string filename = ss.str();
        
        cv::imwrite(filename, current_frame_);
        RCLCPP_INFO(this->get_logger(), "Screenshot saved: %s", filename.c_str());
        std::cout << "Screenshot saved: " << filename << std::endl;
    }
}

cv::Mat VideoStreamer::DrawDetections(const cv::Mat& frame, const std::vector<DetectionDisplay>& detections)
{
    cv::Mat result = frame.clone();
    
    for (const auto& detection : detections) {
        if (detection.confidence < 0.7) {
            continue;
        }

        // Draw bounding box
        cv::rectangle(result, detection.bbox, cv::Scalar(0, 255, 0), 2);
        
        // Prepare label text
        std::stringstream ss;
        ss << detection.class_name << ": " << std::fixed << std::setprecision(2) << detection.confidence;
        if (!detection.decoded_data.empty()) {
            ss << " [" << detection.decoded_data << "]";
        }
        std::string label_text = ss.str();
        
        // Calculate label size and position
        int baseline = 0;
        cv::Size label_size = cv::getTextSize(label_text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        
        cv::Point label_pos(detection.bbox.x, detection.bbox.y - 10);
        if (label_pos.y - label_size.height < 0) {
            label_pos.y = detection.bbox.y + label_size.height + 10;
        }
        
        // Draw label background
        cv::rectangle(result, 
                     cv::Point(label_pos.x, label_pos.y - label_size.height),
                     cv::Point(label_pos.x + label_size.width, label_pos.y + baseline),
                     cv::Scalar(0, 255, 0), -1);
        
        // Draw label text
        cv::putText(result, label_text, label_pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        
        // If QR data exists, draw it separately below the main label
        if (!detection.decoded_data.empty()) {
            cv::Point qr_pos(detection.bbox.x, detection.bbox.y + detection.bbox.height + 25);
            if (qr_pos.y > result.rows - 10) {
                qr_pos.y = detection.bbox.y - 25;
            }
            
            std::string qr_text = "QR: " + detection.decoded_data;
            int qr_baseline = 0;
            cv::Size qr_size = cv::getTextSize(qr_text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &qr_baseline);
            
            // Draw QR text background
            cv::rectangle(result, 
                         cv::Point(qr_pos.x, qr_pos.y - qr_size.height),
                         cv::Point(qr_pos.x + qr_size.width, qr_pos.y + qr_baseline),
                         cv::Scalar(255, 255, 0), -1);
            
            // Draw QR text
            cv::putText(result, qr_text, qr_pos, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
        }
    }
    
    return result;
}

void VideoStreamer::DetectionCallback(const flyscan_interfaces::msg::DetectionArray::SharedPtr msg)
{
    current_detections_.clear();

    for (const auto& det_msg : msg->detections) {
        DetectionDisplay det;
        det.bbox = cv::Rect(det_msg.bbox.x, det_msg.bbox.y, 
                           det_msg.bbox.width, det_msg.bbox.height);
        det.confidence = det_msg.confidence;
        det.class_id = det_msg.class_id;
        det.class_name = det_msg.class_name;
        det.decoded_data = det_msg.decoded_data;
        current_detections_.push_back(det);
    }
}

void VideoStreamer::DisplayVideo()
{
    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name_, 1280, 720);
    
    std::cout << "=== Minimal Video Streamer with Detection Service ===" << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  S - Save screenshot" << std::endl;
    std::cout << "  I - Toggle info overlay" << std::endl;
    std::cout << "  Q - Quit" << std::endl;
    std::cout << "\nWaiting for video on topic: /camera/image_raw" << std::endl;
    
    while (rclcpp::ok()) {
        // Process ROS2 callbacks
        rclcpp::spin_some(shared_from_this());
        
        if (!current_frame_.empty()) {
            // Draw detections on frame using current detections
            cv::Mat processed_frame = DrawDetections(current_frame_, current_detections_);
            
            // Add overlay information
            cv::Mat display_frame = AddInfoOverlay(processed_frame);
            
            // Display the frame
            cv::imshow(window_name_, display_frame);
            
            // Handle key presses
            int key = cv::waitKey(1) & 0xFF;
            
            if (key == 'q' || key == 'Q') {
                break;
            } else if (key == 's' || key == 'S') {
                SaveScreenshot();
            } else if (key == 'i' || key == 'I') {
                show_info_overlay_ = !show_info_overlay_;
                std::cout << "Info overlay: " << (show_info_overlay_ ? "ON" : "OFF") << std::endl;
            }
        } else {
            // Show waiting message if no frame received
            cv::Mat waiting_frame = cv::Mat::zeros(720, 1280, CV_8UC3);
            
            cv::putText(waiting_frame, "Waiting for video stream...",
                       cv::Point(50, 200), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
            cv::putText(waiting_frame, "Topic: /camera/image_raw",
                       cv::Point(50, 250), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);
            cv::putText(waiting_frame, "Press Q to quit",
                       cv::Point(50, 300), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
            
            cv::imshow(window_name_, waiting_frame);
            
            if ((cv::waitKey(1) & 0xFF) == 'q') {
                break;
            }
        }
    }
    
    cv::destroyAllWindows();
    RCLCPP_INFO(this->get_logger(), "Video streamer stopped");
}

} // namespace sim
} // namespace flyscan

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto streamer = std::make_shared<flyscan::sim::VideoStreamer>();
        
        std::thread display_thread([&streamer]() {
            streamer->DisplayVideo();
        });
        
        display_thread.join();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}