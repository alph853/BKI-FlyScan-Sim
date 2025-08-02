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
        
        self.frame_count = 0
        self.fps_frame_count = 0
        self.current_fps = 0.0
        self.window_name = "Drone Camera Feed with Detections"
        self.show_info_overlay = True
        
        self.bridge = CvBridge()
        self.current_frame = None
        self.current_detections = []
        self.last_fps_time = time.time()
        
        self.get_logger().info("VideoStreamer initializing")
        
        # Create subscriptions with appropriate QoS
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.video_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.video_callback,
            sensor_qos
        )
        
        self.detection_subscription = self.create_subscription(
            DetectionArray,
            'detections',
            self.detection_callback,
            10
        )
        
        self.get_logger().info("Waiting for video stream and detection messages...")

    def video_callback(self, msg):
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.frame_count += 1
            self.update_fps_calculation()
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def update_fps_calculation(self):
        self.fps_frame_count += 1
        current_time = time.time()
        elapsed = (current_time - self.last_fps_time) * 1000  # Convert to milliseconds
        
        # Update FPS every 1000ms (1 second)
        if elapsed >= 1000:
            self.current_fps = self.fps_frame_count * 1000.0 / elapsed
            self.fps_frame_count = 0
            self.last_fps_time = current_time

    def add_info_overlay(self, frame):
        if not self.show_info_overlay:
            return frame
        
        overlay_frame = frame.copy()
        fps_text = f"FPS: {self.current_fps:.1f}"
        cv2.putText(overlay_frame, fps_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv2.putText(overlay_frame, "Topic: /camera/image_raw", 
                   (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(overlay_frame, "Controls: S=Screenshot, I=Info, Q=Quit", 
                   (10, frame.shape[0] - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        return overlay_frame

    def save_screenshot(self):
        if self.current_frame is not None:
            filename = f"drone_screenshot_{self.frame_count:06d}.jpg"
            cv2.imwrite(filename, self.current_frame)
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

    def detection_callback(self, msg):
        self.current_detections = msg.detections

    def display_video(self):
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1280, 720)
        
        print("=== Minimal Video Streamer with Detection Service ===")
        print("Controls:")
        print("  S - Save screenshot")
        print("  I - Toggle info overlay")
        print("  Q - Quit")
        print("\nWaiting for video on topic: /camera/image_raw")
        
        while rclpy.ok():
            # Process ROS2 callbacks
            rclpy.spin_once(self, timeout_sec=0.001)
            
            if self.current_frame is not None:
                # Draw detections on frame using current detections
                processed_frame = self.draw_detections(self.current_frame, self.current_detections)
                
                # Add overlay information
                display_frame = self.add_info_overlay(processed_frame)
                
                # Display the frame
                cv2.imshow(self.window_name, display_frame)
                
                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q') or key == ord('Q'):
                    break
                elif key == ord('s') or key == ord('S'):
                    self.save_screenshot()
                elif key == ord('i') or key == ord('I'):
                    self.show_info_overlay = not self.show_info_overlay
                    print(f"Info overlay: {'ON' if self.show_info_overlay else 'OFF'}")
            else:
                # Show waiting message if no frame received
                waiting_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
                
                cv2.putText(waiting_frame, "Waiting for video stream...", 
                           (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                cv2.putText(waiting_frame, "Topic: /camera/image_raw", 
                           (50, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                cv2.putText(waiting_frame, "Press Q to quit", 
                           (50, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                cv2.imshow(self.window_name, waiting_frame)
                
                if (cv2.waitKey(1) & 0xFF) == ord('q'):
                    break
        
        cv2.destroyAllWindows()
        self.get_logger().info("Video streamer stopped")


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