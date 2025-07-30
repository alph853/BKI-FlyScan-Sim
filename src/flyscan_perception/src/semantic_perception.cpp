#include <rclcpp/qos.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <filesystem>
#include <thread>
#include <future>
#include <nlohmann/json.hpp>

#include "flyscan_perception/semantic_perception.hpp"

namespace flyscan {
namespace perception {

SemanticPerception::SemanticPerception()
    : BaseNode(rclcpp::NodeOptions(), "semantic_perception", flyscan::common::NodeType::kPerception, {"object_detection", "yolo_inference"})
    , frame_count_(0)
    , inference_frame_skip_(3)
    , frame_skip_counter_(0)
    , yolo_initialized_(false)
    , class_names_({"Barcode", "QR"})
{
    RCLCPP_INFO(this->get_logger(), "SemanticPerception initializing");
}

SemanticPerception::~SemanticPerception()
{
    RCLCPP_INFO(this->get_logger(), "SemanticPerception shutting down");
}

flyscan::common::OperationStatus SemanticPerception::HandleConfigure()
{
    RCLCPP_INFO(this->get_logger(), "Configuring SemanticPerception");
    
    video_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 
        rclcpp::SensorDataQoS(),
        std::bind(&SemanticPerception::VideoCallback, this, std::placeholders::_1)
    );
    
    detection_publisher_ = this->create_publisher<flyscan_interfaces::msg::DetectionArray>(
        "detections", 10);
    
    std::string model_path = "src/flyscan_perception/best.onnx";
    if (std::filesystem::exists(model_path)) {
        if (InitializeYoloModel(model_path)) {
            RCLCPP_INFO(this->get_logger(), "YOLOv8 model loaded successfully: %s", model_path.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to load YOLOv8 model: %s", model_path.c_str());
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "YOLOv8 model not found: %s", model_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Run the conversion script to convert .pt to .onnx");
    }
    
    return flyscan::common::OperationStatus::kOK;
}

flyscan::common::OperationStatus SemanticPerception::HandleActivate()
{
    RCLCPP_INFO(this->get_logger(), "Activating SemanticPerception");
    return flyscan::common::OperationStatus::kOK;
}

flyscan::common::OperationStatus SemanticPerception::HandleDeactivate()
{
    RCLCPP_INFO(this->get_logger(), "Deactivating SemanticPerception");
    return flyscan::common::OperationStatus::kOK;
}

void SemanticPerception::VideoCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        current_frame_ = cv_ptr_->image;
        frame_count_++;
        
        // Run YOLO inference with frame skipping for performance
        if (yolo_initialized_) {
            if (frame_skip_counter_ == 0) {
                last_detections_ = RunYoloInference(current_frame_);
                PublishDetections(last_detections_);
            }
            frame_skip_counter_ = (frame_skip_counter_ + 1) % inference_frame_skip_;
        }
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error converting image: %s", e.what());
    }
}

void SemanticPerception::PublishDetections(const std::vector<Detection>& detections)
{
    auto detection_array_msg = flyscan_interfaces::msg::DetectionArray();
    detection_array_msg.header.stamp = this->now();
    detection_array_msg.header.frame_id = "camera";
    detection_array_msg.frame_count = frame_count_;
    
    for (const auto& detection : detections) {
        flyscan_interfaces::msg::Detection det_msg;
        det_msg.bbox.x = detection.bbox.x;
        det_msg.bbox.y = detection.bbox.y;
        det_msg.bbox.width = detection.bbox.width;
        det_msg.bbox.height = detection.bbox.height;
        det_msg.confidence = detection.confidence;
        det_msg.class_id = detection.class_id;
        det_msg.decoded_data = detection.decoded_data;
        det_msg.decode_params = detection.decode_params;
        if (detection.class_id >= 0 && detection.class_id < static_cast<int>(class_names_.size())) {
            det_msg.class_name = class_names_[detection.class_id];
        } else {
            det_msg.class_name = "Unknown";
        }
        detection_array_msg.detections.push_back(det_msg);
    }
    
    detection_publisher_->publish(detection_array_msg);
}

bool SemanticPerception::InitializeYoloModel(const std::string& model_path)
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

std::vector<Detection> SemanticPerception::RunYoloInference(const cv::Mat& frame)
{
    std::vector<Detection> detections;
    
    if (!yolo_initialized_ || frame.empty()) {
        return detections;
    }
    
    try {
        // Preprocess the frame
        cv::Mat preprocessed = PreprocessFrame(frame);
        
        // Create input tensor
        std::vector<int64_t> input_shape = {1, 3, 640, 640}; // YOLO input size hardcoded
        size_t input_tensor_size = 1 * 3 * 640 * 640;
        std::vector<float> input_tensor_values(input_tensor_size);
        
        // Fill input tensor (HWC to CHW format)
        for (int c = 0; c < 3; ++c) {
            for (int h = 0; h < 640; ++h) {
                for (int w = 0; w < 640; ++w) {
                    input_tensor_values[c * 640 * 640 + h * 640 + w] =
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
        detections = PostprocessOutputs(output_tensors, frame.cols, frame.rows);
        
        // Process QR/Barcode decoding for detected objects
        ProcessQrDecoding(frame, detections);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "YOLO inference failed: %s", e.what());
    }
    
    return detections;
}

cv::Mat SemanticPerception::PreprocessFrame(const cv::Mat& frame)
{
    cv::Mat resized, normalized;
    
    // Resize to model input size
    cv::resize(frame, resized, cv::Size(640, 640));
    
    // Convert to float and normalize to [0, 1]
    resized.convertTo(normalized, CV_32F, 1.0 / 255.0);
    
    return normalized;
}

std::vector<Detection> SemanticPerception::PostprocessOutputs(const std::vector<Ort::Value>& outputs, 
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
    int num_classes = output_shape[1] - 4; // classes (total - 4 bbox coords)
    
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
        if (max_confidence >= 0.5f) { // hardcoded confidence threshold
            // Convert to corner coordinates and scale to original image size
            float x1 = (cx - w / 2) * img_width / 640;
            float y1 = (cy - h / 2) * img_height / 640;
            float x2 = (cx + w / 2) * img_width / 640;
            float y2 = (cy + h / 2) * img_height / 640;
            
            boxes.push_back(cv::Rect(static_cast<int>(x1), static_cast<int>(y1), 
                                   static_cast<int>(x2 - x1), static_cast<int>(y2 - y1)));
            confidences.push_back(max_confidence);
            class_ids.push_back(best_class_id);
        }
    }
    
    // Apply Non-Maximum Suppression
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, 0.5f, 0.4f, indices); // hardcoded thresholds
    
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

void SemanticPerception::ProcessQrDecoding(const cv::Mat& frame, std::vector<Detection>& detections)
{
    for (auto& detection : detections) {
        if (detection.class_id == 0 || detection.class_id == 1) {
            int margin_x = static_cast<int>(detection.bbox.width * 0.2 / 2);
            int margin_y = static_cast<int>(detection.bbox.height * 0.2 / 2);
            
            int x1 = std::max(0, detection.bbox.x - margin_x);
            int y1 = std::max(0, detection.bbox.y - margin_y);
            int x2 = std::min(frame.cols, detection.bbox.x + detection.bbox.width + margin_x);
            int y2 = std::min(frame.rows, detection.bbox.y + detection.bbox.height + margin_y);
            
            cv::Rect roi_rect(x1, y1, x2 - x1, y2 - y1);
            cv::Mat roi = frame(roi_rect);
            
            if (!roi.empty()) {
                detection.decoded_data = DecodeQrFromRoi(roi);
                if (!detection.decoded_data.empty()) {
                    nlohmann::json params;
                    params["method"] = "opencv_qr_detector";
                    params["roi_x"] = x1;
                    params["roi_y"] = y1;
                    params["roi_width"] = x2 - x1;
                    params["roi_height"] = y2 - y1;
                    detection.decode_params = params.dump();
                }
            }
        }
    }
}

std::string SemanticPerception::DecodeQrFromRoi(const cv::Mat& roi)
{
    cv::QRCodeDetector qr_detector;
    std::string data;
    
    data = qr_detector.detectAndDecode(roi);
    if (!data.empty()) {
        return data;
    }
    
    cv::Mat upscaled;
    cv::resize(roi, upscaled, cv::Size(roi.cols * 2, roi.rows * 2), 0, 0, cv::INTER_LANCZOS4);
    
    data = qr_detector.detectAndDecode(upscaled);
    if (!data.empty()) {
        return data;
    }
    
    try {
        cv::Mat deskewed = DeskewQr(roi);
        data = qr_detector.detectAndDecode(deskewed);
        if (!data.empty()) {
            return data;
        }
    } catch (...) {
    }
    
    cv::Mat sharpened = SharpenImage(roi);
    data = qr_detector.detectAndDecode(sharpened);
    if (!data.empty()) {
        return data;
    }
    
    std::vector<float> alphas = {0.5f, 1.0f, 1.5f, 2.0f, 2.5f, 3.0f};
    std::vector<int> betas = {-60, -40, -20, 0, 20, 40, 60};
    std::vector<int> angles = {0, 90, 180, 270};
    
    for (float alpha : alphas) {
        for (int beta : betas) {
            for (int angle : angles) {
                std::string result = TryDecodeVariant(roi, alpha, beta, 2.0f, cv::Size(8, 8), angle);
                if (!result.empty()) {
                    return result;
                }
            }
        }
    }
    
    return "";
}

cv::Mat SemanticPerception::SharpenImage(const cv::Mat& image)
{
    cv::Mat blurred;
    cv::GaussianBlur(image, blurred, cv::Size(0, 0), 3.0, 3.0);
    cv::Mat sharpened;
    cv::addWeighted(image, 1.5, blurred, -0.5, 0, sharpened);
    return sharpened;
}

cv::Mat SemanticPerception::DeskewQr(const cv::Mat& img)
{
    cv::QRCodeDetector detector;
    std::vector<cv::Point2f> points;
    
    if (!detector.detect(img, points) || points.size() != 4) {
        throw std::runtime_error("Could not detect QR code corners");
    }
    
    std::vector<float> distances;
    for (int i = 0; i < 4; ++i) {
        distances.push_back(cv::norm(points[i] - points[(i + 1) % 4]));
    }
    int side = static_cast<int>(*std::max_element(distances.begin(), distances.end()));
    
    std::vector<cv::Point2f> dest = {
        cv::Point2f(0, 0),
        cv::Point2f(side - 1, 0),
        cv::Point2f(side - 1, side - 1),
        cv::Point2f(0, side - 1)
    };
    
    cv::Mat transform = cv::getPerspectiveTransform(points, dest);
    cv::Mat result;
    cv::warpPerspective(img, result, transform, cv::Size(side, side));
    return result;
}

std::string SemanticPerception::TryDecodeVariant(const cv::Mat& img, float alpha, int beta, 
                                               float clahe_clip, cv::Size clahe_grid, int angle)
{
    cv::Mat adjusted;
    img.convertTo(adjusted, -1, alpha, beta);
    
    cv::Mat lab;
    cv::cvtColor(adjusted, lab, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> lab_planes;
    cv::split(lab, lab_planes);
    
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(clahe_clip, clahe_grid);
    clahe->apply(lab_planes[0], lab_planes[0]);
    
    cv::Mat enhanced;
    cv::merge(lab_planes, enhanced);
    cv::cvtColor(enhanced, enhanced, cv::COLOR_Lab2BGR);
    
    cv::Mat sharpened = SharpenImage(enhanced);
    
    cv::Mat img_to_decode;
    if (angle != 0) {
        cv::Point2f center(sharpened.cols / 2.0f, sharpened.rows / 2.0f);
        cv::Mat rotation_matrix = cv::getRotationMatrix2D(center, angle, 1.0);
        cv::warpAffine(sharpened, img_to_decode, rotation_matrix, sharpened.size());
    } else {
        img_to_decode = sharpened;
    }
    
    cv::QRCodeDetector detector;
    std::string data = detector.detectAndDecode(img_to_decode);
    if (!data.empty()) {
        return data;
    }
    
    return "";
}

} // namespace perception
} // namespace flyscan


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto semantic_perception = std::make_shared<flyscan::perception::SemanticPerception>();
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(semantic_perception->get_node_base_interface());

        auto configure_result = semantic_perception->configure();
        if (configure_result.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
            RCLCPP_ERROR(rclcpp::get_logger("semantic_perception_main"), "Failed to configure semantic_perception_main");
            return 1;
        }

        // Activate the node
        auto activate_result = semantic_perception->activate();
        if (activate_result.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            RCLCPP_ERROR(rclcpp::get_logger("semantic_perception_main"), "Failed to activate semantic_perception_main");
            return 1;
        }

        executor.spin();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}