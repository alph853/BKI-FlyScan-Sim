#include "flyscan_perception/video_streamer.hpp"
#include <rclcpp/qos.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <filesystem>
#include <thread>

namespace flyscan_perception
{

VideoStreamer::VideoStreamer()
    : Node("gazebo_video_streamer")
    , frame_count_(0)
    , window_name_("Drone Camera Feed")
    , show_info_overlay_(true)
    , video_topic_("/camera/image_raw")
    , recording_(false)
    , yolo_initialized_(false)
    , inference_frame_skip_(INFERENCE_FRAME_SKIP)
    , frame_skip_counter_(0)
{
    // Create video subscriber with sensor data QoS
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);
    
    video_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        video_topic_, qos,
        std::bind(&VideoStreamer::video_callback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(this->get_logger(), "Subscribed to video topic: %s", video_topic_.c_str());
    
    // Initialize COCO class names
    class_names_ = {
        "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
        "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
        "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
        "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
        "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
        "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
        "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
        "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
        "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
        "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
    };
    
    // Try to initialize YOLO model
    std::string model_path = "src/flyscan_perception/yolov8n.onnx";
    if (std::filesystem::exists(model_path)) {
        if (initialize_yolo_model(model_path)) {
            RCLCPP_INFO(this->get_logger(), "YOLOv8 model loaded successfully: %s", model_path.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to load YOLOv8 model: %s", model_path.c_str());
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "YOLOv8 model not found: %s", model_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Run the conversion script to convert .pt to .onnx");
    }
    
    RCLCPP_INFO(this->get_logger(), "Waiting for video stream...");
}

VideoStreamer::~VideoStreamer()
{
    if (recording_ && video_writer_.isOpened()) {
        video_writer_.release();
    }
    cv::destroyAllWindows();
}

void VideoStreamer::video_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        current_frame_ = cv_ptr_->image;
        frame_count_++;
        
        if (frame_count_ == 1) {
            RCLCPP_INFO(this->get_logger(), "Video stream started!");
            RCLCPP_INFO(this->get_logger(), "Frame size: %dx%d", 
                       current_frame_.cols, current_frame_.rows);
        }
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error converting image: %s", e.what());
    }
}

cv::Mat VideoStreamer::add_info_overlay(const cv::Mat& frame)
{
    if (!show_info_overlay_) {
        return frame;
    }
    
    cv::Mat overlay_frame = frame.clone();
    
    // Add frame counter
    cv::putText(overlay_frame, "Frame: " + std::to_string(frame_count_),
               cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    
    // Add recording indicator
    if (recording_) {
        cv::circle(overlay_frame, cv::Point(frame.cols - 30, 30), 10, cv::Scalar(0, 0, 255), -1);
        cv::putText(overlay_frame, "REC",
                   cv::Point(frame.cols - 70, 35), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
    }
    
    // Add topic name
    cv::putText(overlay_frame, "Topic: " + video_topic_,
               cv::Point(10, frame.rows - 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    
    // Add controls help
    std::string help_text = "Controls: S=Screenshot, R=Record, I=Info, +/- Skip, Q=Quit";
    cv::putText(overlay_frame, help_text,
               cv::Point(10, frame.rows - 40), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
    
    return overlay_frame;
}

void VideoStreamer::save_screenshot()
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

void VideoStreamer::toggle_recording()
{
    if (!recording_) {
        // Start recording
        if (!current_frame_.empty()) {
            std::stringstream ss;
            ss << "drone_video_" << std::setfill('0') << std::setw(6) << frame_count_ << ".mp4";
            std::string filename = ss.str();
            
            int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
            cv::Size frame_size(current_frame_.cols, current_frame_.rows);
            
            video_writer_.open(filename, fourcc, FPS, frame_size);
            if (video_writer_.isOpened()) {
                recording_ = true;
                RCLCPP_INFO(this->get_logger(), "Recording started: %s", filename.c_str());
                std::cout << "Recording started: " << filename << std::endl;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to open video writer");
            }
        }
    } else {
        // Stop recording
        if (video_writer_.isOpened()) {
            video_writer_.release();
            recording_ = false;
            RCLCPP_INFO(this->get_logger(), "Recording stopped");
            std::cout << "Recording stopped" << std::endl;
        }
    }
}

void VideoStreamer::display_video()
{
    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name_, 1280, 720);
    
    std::cout << "=== Gazebo Video Streamer with YOLOv8 Detection ===" << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  S - Save screenshot" << std::endl;
    std::cout << "  R - Toggle recording" << std::endl;
    std::cout << "  I - Toggle info overlay" << std::endl;
    std::cout << "  + - Increase frame skip (faster FPS, less frequent detection)" << std::endl;
    std::cout << "  - - Decrease frame skip (slower FPS, more frequent detection)" << std::endl;
    std::cout << "  Q - Quit" << std::endl;
    std::cout << "\nWaiting for video on topic: " << video_topic_ << std::endl;
    
    while (rclcpp::ok()) {
        // Process ROS2 callbacks
        rclcpp::spin_some(shared_from_this());
        
        if (!current_frame_.empty()) {
            // Run YOLO inference with frame skipping for performance
            cv::Mat processed_frame = current_frame_.clone();
            if (yolo_initialized_) {
                // Only run inference every N frames
                if (frame_skip_counter_ == 0) {
                    last_detections_ = run_yolo_inference(current_frame_);
                }
                frame_skip_counter_ = (frame_skip_counter_ + 1) % inference_frame_skip_;
                
                // Always draw the last detections
                processed_frame = draw_detections(current_frame_, last_detections_);
            }
            
            // Add overlay information
            cv::Mat display_frame = add_info_overlay(processed_frame);
            
            // Record frame if recording is active
            if (recording_ && video_writer_.isOpened()) {
                video_writer_.write(current_frame_);
            }
            
            // Display the frame
            cv::imshow(window_name_, display_frame);
            
            // Handle key presses
            int key = cv::waitKey(1) & 0xFF;
            
            if (key == 'q' || key == 'Q') {
                break;
            } else if (key == 's' || key == 'S') {
                save_screenshot();
            } else if (key == 'r' || key == 'R') {
                toggle_recording();
            } else if (key == 'i' || key == 'I') {
                show_info_overlay_ = !show_info_overlay_;
                std::cout << "Info overlay: " << (show_info_overlay_ ? "ON" : "OFF") << std::endl;
            } else if (key == '+' || key == '=') {
                if (inference_frame_skip_ < 10) {
                    inference_frame_skip_++;
                    std::cout << "Inference frame skip: " << inference_frame_skip_ << " (faster FPS)" << std::endl;
                }
            } else if (key == '-' || key == '_') {
                if (inference_frame_skip_ > 1) {
                    inference_frame_skip_--;
                    std::cout << "Inference frame skip: " << inference_frame_skip_ << " (slower FPS)" << std::endl;
                }
            }
        } else {
            // Show waiting message if no frame received
            cv::Mat waiting_frame = cv::Mat::zeros(WAITING_FRAME_HEIGHT, WAITING_FRAME_WIDTH, CV_8UC3);
            
            cv::putText(waiting_frame, "Waiting for video stream...",
                       cv::Point(50, 200), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
            cv::putText(waiting_frame, "Topic: " + video_topic_,
                       cv::Point(50, 250), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);
            cv::putText(waiting_frame, "Press Q to quit",
                       cv::Point(50, 300), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
            
            cv::imshow(window_name_, waiting_frame);
            
            if ((cv::waitKey(1) & 0xFF) == 'q') {
                break;
            }
        }
    }
    
    // Cleanup
    if (recording_ && video_writer_.isOpened()) {
        video_writer_.release();
    }
    
    cv::destroyAllWindows();
    RCLCPP_INFO(this->get_logger(), "Video streamer stopped");
}

bool VideoStreamer::initialize_yolo_model(const std::string& model_path)
{
    try {
        // Initialize ONNX Runtime environment
        ort_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "YOLOv8");
        
        // Create session options for better performance
        session_options_ = std::make_unique<Ort::SessionOptions>();
        session_options_->SetIntraOpNumThreads(std::thread::hardware_concurrency());
        session_options_->SetInterOpNumThreads(1);
        session_options_->SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session_options_->SetExecutionMode(ExecutionMode::ORT_PARALLEL);
        
        // Create inference session
        ort_session_ = std::make_unique<Ort::Session>(*ort_env_, model_path.c_str(), *session_options_);
        
        // Get input and output information
        Ort::AllocatorWithDefaultOptions allocator;
        
        // Input information
        size_t num_input_nodes = ort_session_->GetInputCount();
        if (num_input_nodes != 1) {
            RCLCPP_ERROR(this->get_logger(), "Expected 1 input, got %zu", num_input_nodes);
            return false;
        }
        
        // Get input and output names using newer API
        auto input_name = ort_session_->GetInputNameAllocated(0, allocator);
        input_names_str_.push_back(std::string(input_name.get()));
        
        Ort::TypeInfo input_type_info = ort_session_->GetInputTypeInfo(0);
        auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
        input_shape_ = input_tensor_info.GetShape();
        
        // Output information
        size_t num_output_nodes = ort_session_->GetOutputCount();
        for (size_t i = 0; i < num_output_nodes; i++) {
            auto output_name = ort_session_->GetOutputNameAllocated(i, allocator);
            output_names_str_.push_back(std::string(output_name.get()));
        }
        
        // Convert string vectors to char* vectors for inference
        for (const auto& name : input_names_str_) {
            input_names_.push_back(name.c_str());
        }
        for (const auto& name : output_names_str_) {
            output_names_.push_back(name.c_str());
        }
        
        RCLCPP_INFO(this->get_logger(), "Model input shape: [%ld, %ld, %ld, %ld]", 
                   input_shape_[0], input_shape_[1], input_shape_[2], input_shape_[3]);
        RCLCPP_INFO(this->get_logger(), "Model has %zu outputs", num_output_nodes);
        
        yolo_initialized_ = true;
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize YOLO model: %s", e.what());
        return false;
    }
}

std::vector<Detection> VideoStreamer::run_yolo_inference(const cv::Mat& frame)
{
    std::vector<Detection> detections;
    
    if (!yolo_initialized_ || frame.empty()) {
        return detections;
    }
    
    try {
        // Preprocess the frame
        cv::Mat preprocessed = preprocess_frame(frame);
        
        // Create input tensor
        std::vector<int64_t> input_shape = {1, 3, YOLO_INPUT_SIZE, YOLO_INPUT_SIZE};
        size_t input_tensor_size = 1 * 3 * YOLO_INPUT_SIZE * YOLO_INPUT_SIZE;
        std::vector<float> input_tensor_values(input_tensor_size);
        
        // Fill input tensor (HWC to CHW format)
        for (int c = 0; c < 3; ++c) {
            for (int h = 0; h < YOLO_INPUT_SIZE; ++h) {
                for (int w = 0; w < YOLO_INPUT_SIZE; ++w) {
                    input_tensor_values[c * YOLO_INPUT_SIZE * YOLO_INPUT_SIZE + h * YOLO_INPUT_SIZE + w] =
                        preprocessed.at<cv::Vec3f>(h, w)[c];
                }
            }
        }
        
        // Create input tensor
        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, input_tensor_values.data(), input_tensor_size,
            input_shape.data(), input_shape.size());
        
        // Run inference
        std::vector<Ort::Value> output_tensors = ort_session_->Run(
            Ort::RunOptions{nullptr}, input_names_.data(), &input_tensor, 1,
            output_names_.data(), output_names_.size());
        
        // Post-process outputs
        detections = postprocess_outputs(output_tensors, frame.cols, frame.rows);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "YOLO inference failed: %s", e.what());
    }
    
    return detections;
}

cv::Mat VideoStreamer::preprocess_frame(const cv::Mat& frame)
{
    cv::Mat resized, normalized;
    
    // Resize to model input size
    cv::resize(frame, resized, cv::Size(YOLO_INPUT_SIZE, YOLO_INPUT_SIZE));
    
    // Convert to float and normalize to [0, 1]
    resized.convertTo(normalized, CV_32F, 1.0 / 255.0);
    
    return normalized;
}

std::vector<Detection> VideoStreamer::postprocess_outputs(const std::vector<Ort::Value>& outputs, 
                                                          float img_width, float img_height)
{
    std::vector<Detection> detections;
    
    if (outputs.empty()) {
        return detections;
    }
    
    // Get output tensor data
    const float* output_data = outputs[0].GetTensorData<float>();
    auto output_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    
    // YOLOv8 output format: [batch, 84, 8400] where 84 = 4 (bbox) + 80 (classes)
    int num_detections = output_shape[2];  // 8400
    int num_classes = output_shape[1] - 4; // 80 classes (84 - 4 bbox coords)
    
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;
    
    // Parse detections - YOLOv8 has transposed output format
    for (int i = 0; i < num_detections; ++i) {
        // Extract box coordinates (center_x, center_y, width, height)
        float cx = output_data[i];                           // First row
        float cy = output_data[num_detections + i];          // Second row  
        float w = output_data[2 * num_detections + i];       // Third row
        float h = output_data[3 * num_detections + i];       // Fourth row
        
        // Find the class with highest confidence
        float max_confidence = 0.0f;
        int best_class_id = -1;
        
        for (int c = 0; c < num_classes; ++c) {
            float confidence = output_data[(4 + c) * num_detections + i];
            if (confidence > max_confidence) {
                max_confidence = confidence;
                best_class_id = c;
            }
        }
        
        // Filter by confidence threshold
        if (max_confidence >= CONFIDENCE_THRESHOLD) {
            // Convert to corner coordinates and scale to original image size
            float x1 = (cx - w / 2) * img_width / YOLO_INPUT_SIZE;
            float y1 = (cy - h / 2) * img_height / YOLO_INPUT_SIZE;
            float x2 = (cx + w / 2) * img_width / YOLO_INPUT_SIZE;
            float y2 = (cy + h / 2) * img_height / YOLO_INPUT_SIZE;
            
            boxes.push_back(cv::Rect(static_cast<int>(x1), static_cast<int>(y1), 
                                   static_cast<int>(x2 - x1), static_cast<int>(y2 - y1)));
            confidences.push_back(max_confidence);
            class_ids.push_back(best_class_id);
        }
    }
    
    // Apply Non-Maximum Suppression
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, CONFIDENCE_THRESHOLD, NMS_THRESHOLD, indices);
    
    // Create final detections
    for (int idx : indices) {
        Detection det;
        det.bbox = boxes[idx];
        det.confidence = confidences[idx];
        det.class_id = class_ids[idx];
        detections.push_back(det);
    }
    
    return detections;
}

cv::Mat VideoStreamer::draw_detections(const cv::Mat& frame, const std::vector<Detection>& detections)
{
    cv::Mat result = frame.clone();
    
    for (const auto& detection : detections) {
        // Draw bounding box
        cv::rectangle(result, detection.bbox, cv::Scalar(0, 255, 0), 2);
        
        // Prepare label text
        std::string label;
        if (detection.class_id >= 0 && detection.class_id < static_cast<int>(class_names_.size())) {
            label = class_names_[detection.class_id];
        } else {
            label = "Unknown";
        }
        
        std::stringstream ss;
        ss << label << ": " << std::fixed << std::setprecision(2) << detection.confidence;
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
    }
    
    return result;
}

} // namespace flyscan_perception

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto streamer = std::make_shared<flyscan_perception::VideoStreamer>();
        streamer->display_video();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}