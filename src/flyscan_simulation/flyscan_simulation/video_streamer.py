#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from flyscan_interfaces.msg import DetectionArray
import threading
import time


class VideoStreamer(Node):
    def __init__(self):
        super().__init__('video_streamer')
        
        # Multi-drone support - list of active drone IDs to visualize
        self.declare_parameter('active_drone_ids', [0])
        self.active_drone_ids = self.get_parameter('active_drone_ids').get_parameter_value().integer_array_value
        
        self.window_name = "Multi-Drone Camera Feed with Detections"
        self.show_info_overlay = True
        
        self.bridge = CvBridge()
        
        # Per-drone data structures
        self.drone_frames = {}  # drone_id -> current frame
        self.drone_detections = {}  # drone_id -> current detections
        self.drone_frame_counts = {}  # drone_id -> frame count
        self.drone_fps = {}  # drone_id -> current fps
        self.drone_fps_counters = {}  # drone_id -> fps frame counter
        self.drone_fps_times = {}  # drone_id -> last fps calculation time
        
        # Subscriptions storage
        self.video_subscriptions = {}
        self.detection_subscriptions = {}
        
        # Initialize per-drone data and subscriptions
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        for drone_id in self.active_drone_ids:
            self.drone_frames[drone_id] = None
            self.drone_detections[drone_id] = []
            self.drone_frame_counts[drone_id] = 0
            self.drone_fps[drone_id] = 0.0
            self.drone_fps_counters[drone_id] = 0
            self.drone_fps_times[drone_id] = time.time()
            
            # Create subscriptions for each drone
            camera_topic = self.get_camera_topic_name(drone_id, '/camera/image_raw')
            detection_topic = self.get_camera_topic_name(drone_id, '/detections')
            
            self.video_subscriptions[drone_id] = self.create_subscription(
                Image,
                camera_topic,
                lambda msg, d_id=drone_id: self.video_callback(msg, d_id),
                sensor_qos
            )
            
            self.detection_subscriptions[drone_id] = self.create_subscription(
                DetectionArray,
                detection_topic,
                lambda msg, d_id=drone_id: self.detection_callback(msg, d_id),
                10
            )
            
            self.get_logger().info(f"Subscribed to drone {drone_id}: {camera_topic}, {detection_topic}")
        
        self.get_logger().info(f"Multi-Drone VideoStreamer initializing for drones: {self.active_drone_ids}")

    def video_callback(self, msg, drone_id):
        try:
            self.drone_frames[drone_id] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.drone_frame_counts[drone_id] += 1
            self.update_fps_calculation(drone_id)
        except Exception as e:
            self.get_logger().error(f"Error converting image for drone {drone_id}: {e}")

    def update_fps_calculation(self, drone_id):
        self.drone_fps_counters[drone_id] += 1
        current_time = time.time()
        elapsed = (current_time - self.drone_fps_times[drone_id]) * 1000  # Convert to milliseconds
        
        # Update FPS every 1000ms (1 second)
        if elapsed >= 1000:
            self.drone_fps[drone_id] = self.drone_fps_counters[drone_id] * 1000.0 / elapsed
            self.drone_fps_counters[drone_id] = 0
            self.drone_fps_times[drone_id] = current_time

    def add_info_overlay(self, frame, drone_id):
        if not self.show_info_overlay:
            return frame
        
        overlay_frame = frame.copy()
        fps_text = f"Drone {drone_id} FPS: {self.drone_fps[drone_id]:.1f}"
        cv2.putText(overlay_frame, fps_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        camera_topic = self.get_camera_topic_name(drone_id, '/camera/image_raw')
        cv2.putText(overlay_frame, f"Topic: {camera_topic}", 
                   (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(overlay_frame, "Controls: S=Screenshot, I=Info, Q=Quit", 
                   (10, frame.shape[0] - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
        
        return overlay_frame

    def save_screenshot(self, combined_frame):
        if combined_frame is not None:
            timestamp = int(time.time())
            filename = f"multi_drone_screenshot_{timestamp}.jpg"
            cv2.imwrite(filename, combined_frame)
            self.get_logger().info(f"Screenshot saved: {filename}")
            print(f"Screenshot saved: {filename}")

    def draw_detections(self, frame, detections):
        result = frame.copy()
        
        for detection in detections:
            if detection.confidence < 0.7:
                continue
            
            # Draw bounding box
            x, y, w, h = detection.bbox.x, detection.bbox.y, detection.bbox.width, detection.bbox.height
            cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Prepare label text
            label_text = f"{detection.class_name}: {detection.confidence:.2f}"
            if detection.decoded_data:
                label_text += f" [{detection.decoded_data}]"
            
            # Calculate label size and position
            (label_width, label_height), baseline = cv2.getTextSize(
                label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            
            label_y = y - 10
            if label_y - label_height < 0:
                label_y = y + label_height + 10
            
            # Draw label background
            cv2.rectangle(result, (x, label_y - label_height), 
                         (x + label_width, label_y + baseline), (0, 255, 0), -1)
            
            # Draw label text
            cv2.putText(result, label_text, (x, label_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            
            # If QR data exists, draw it separately below the main label
            if detection.decoded_data:
                qr_y = y + h + 25
                if qr_y > result.shape[0] - 10:
                    qr_y = y - 25
                
                qr_text = f"QR: {detection.decoded_data}"
                (qr_width, qr_height), qr_baseline = cv2.getTextSize(
                    qr_text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
                
                # Draw QR text background
                cv2.rectangle(result, (x, qr_y - qr_height), 
                             (x + qr_width, qr_y + qr_baseline), (255, 255, 0), -1)
                
                # Draw QR text
                cv2.putText(result, qr_text, (x, qr_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
        
        return result

    def detection_callback(self, msg, drone_id):
        self.drone_detections[drone_id] = msg.detections

    def get_camera_topic_name(self, drone_id, topic_name):
        """Get camera topic name with appropriate drone prefix"""
        if drone_id == 0:
            return topic_name
        else:
            return f'/px4_{drone_id}{topic_name}'
    
    def calculate_grid_layout(self, num_drones):
        """Calculate optimal grid layout for given number of drones"""
        if num_drones == 1:
            return 1, 1
        elif num_drones == 2:
            return 1, 2
        elif num_drones <= 4:
            return 2, 2
        elif num_drones <= 6:
            return 2, 3
        elif num_drones <= 9:
            return 3, 3
        else:
            # For more than 9 drones, use square root approach
            import math
            cols = math.ceil(math.sqrt(num_drones))
            rows = math.ceil(num_drones / cols)
            return rows, cols
    
    def create_combined_view(self, target_width=1280, target_height=720):
        """Create a combined view with all drone feeds in subwindows"""
        num_drones = len(self.active_drone_ids)
        if num_drones == 0:
            return None
            
        rows, cols = self.calculate_grid_layout(num_drones)
        
        # Calculate subwindow dimensions
        sub_width = target_width // cols
        sub_height = target_height // rows
        
        # Create empty canvas
        combined_frame = np.zeros((target_height, target_width, 3), dtype=np.uint8)
        
        # Track active drones with actual frames
        active_drones_with_frames = []
        for drone_id in self.active_drone_ids:
            if self.drone_frames[drone_id] is not None:
                active_drones_with_frames.append(drone_id)
        
        # If no frames available, show waiting message
        if not active_drones_with_frames:
            cv2.putText(combined_frame, f"Waiting for {num_drones} drone streams...", 
                       (50, target_height//2), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            
            for i, drone_id in enumerate(self.active_drone_ids):
                y_text = target_height//2 + 50 + (i * 30)
                camera_topic = self.get_camera_topic_name(drone_id, '/camera/image_raw')
                cv2.putText(combined_frame, f"Drone {drone_id}: {camera_topic}", 
                           (50, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
            
            cv2.putText(combined_frame, "Press Q to quit", 
                       (50, target_height - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            return combined_frame
        
        # Place each drone's feed in its grid position
        for i, drone_id in enumerate(self.active_drone_ids):
            row = i // cols
            col = i % cols
            
            # Calculate position in combined frame
            y_start = row * sub_height
            y_end = min((row + 1) * sub_height, target_height)
            x_start = col * sub_width
            x_end = min((col + 1) * sub_width, target_width)
            
            actual_sub_height = y_end - y_start
            actual_sub_width = x_end - x_start
            
            if self.drone_frames[drone_id] is not None:
                # Process frame with detections
                frame_with_detections = self.draw_detections(
                    self.drone_frames[drone_id], 
                    self.drone_detections[drone_id]
                )
                
                # Add info overlay
                processed_frame = self.add_info_overlay(frame_with_detections, drone_id)
                
                # Resize to fit subwindow
                resized_frame = cv2.resize(processed_frame, (actual_sub_width, actual_sub_height))
                
                # Place in combined frame
                combined_frame[y_start:y_end, x_start:x_end] = resized_frame
                
            else:
                # Show waiting message for this drone
                drone_area = np.zeros((actual_sub_height, actual_sub_width, 3), dtype=np.uint8)
                
                # Draw border
                cv2.rectangle(drone_area, (0, 0), (actual_sub_width-1, actual_sub_height-1), (100, 100, 100), 2)
                
                # Add text
                text = f"Waiting for Drone {drone_id}"
                text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)[0]
                text_x = (actual_sub_width - text_size[0]) // 2
                text_y = (actual_sub_height - text_size[1]) // 2 + text_size[1]
                
                cv2.putText(drone_area, text, (text_x, text_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                camera_topic = self.get_camera_topic_name(drone_id, '/camera/image_raw')
                topic_text = f"Topic: {camera_topic}"
                topic_size = cv2.getTextSize(topic_text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0]
                topic_x = (actual_sub_width - topic_size[0]) // 2
                topic_y = text_y + 30
                
                if topic_y < actual_sub_height - 10:
                    cv2.putText(drone_area, topic_text, (topic_x, topic_y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
                
                combined_frame[y_start:y_end, x_start:x_end] = drone_area
        
        return combined_frame

    def display_video(self):
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1280, 720)
        
        print("=== Multi-Drone Video Streamer with Detection Service ===")
        print("Controls:")
        print("  S - Save screenshot")
        print("  I - Toggle info overlay")
        print("  Q - Quit")
        print(f"\nWaiting for video streams from drones: {self.active_drone_ids}")
        
        for drone_id in self.active_drone_ids:
            camera_topic = self.get_camera_topic_name(drone_id, '/camera/image_raw')
            print(f"  Drone {drone_id}: {camera_topic}")
        
        while rclpy.ok():
            # Process ROS2 callbacks
            rclpy.spin_once(self, timeout_sec=0.001)
            
            # Create combined view with all drone feeds
            combined_frame = self.create_combined_view()
            
            if combined_frame is not None:
                # Display the combined frame
                cv2.imshow(self.window_name, combined_frame)
                
                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q') or key == ord('Q'):
                    break
                elif key == ord('s') or key == ord('S'):
                    self.save_screenshot(combined_frame)
                elif key == ord('i') or key == ord('I'):
                    self.show_info_overlay = not self.show_info_overlay
                    print(f"Info overlay: {'ON' if self.show_info_overlay else 'OFF'}")
        
        cv2.destroyAllWindows()
        self.get_logger().info("Multi-drone video streamer stopped")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        streamer = VideoStreamer()
        
        # Run display in main thread since OpenCV requires it
        streamer.display_video()
        
    except Exception as e:
        print(f"Error: {e}")
        return 1
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    
    return 0


if __name__ == '__main__':
    exit(main())