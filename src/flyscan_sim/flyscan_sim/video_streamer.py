#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data


class VideoStreamer(Node):
    def __init__(self):
        super().__init__('gazebo_video_streamer')
        
        # ROS2 setup for video streaming
        self.bridge = CvBridge()
        self.current_frame = None
        self.frame_count = 0
        
        # TODO: Replace with your actual Gazebo video topic
        # Common examples:
        # - '/camera/image_raw'
        # - '/iris/camera/image_raw' 
        # - '/X3/camera/image_raw'
        # - '/drone/camera/image_raw'
        self.video_topic = "/camera/image_raw"
        
        # Create video subscriber
        self.video_subscription = self.create_subscription(
            Image,
            self.video_topic,
            self.video_callback,
            qos_profile_sensor_data
        )
        
        self.get_logger().info(f"Subscribed to video topic: {self.video_topic}")
        self.get_logger().info("Waiting for video stream...")
        
        # Display settings
        self.window_name = 'Drone Camera Feed'
        self.show_info_overlay = True
        self.recording = False
        self.video_writer = None
        
    def video_callback(self, msg):
        """Callback function for video stream"""
        try:
            # Convert ROS Image message to OpenCV format
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.frame_count += 1
            
            # Log first frame received
            if self.frame_count == 1:
                self.get_logger().info("Video stream started!")
                self.get_logger().info(f"Frame size: {self.current_frame.shape[1]}x{self.current_frame.shape[0]}")
                
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
    
    def add_info_overlay(self, frame):
        """Add information overlay to the frame"""
        if not self.show_info_overlay:
            return frame
            
        # Create a copy to avoid modifying the original
        overlay_frame = frame.copy()
        
        # Add frame counter
        cv2.putText(overlay_frame, f"Frame: {self.frame_count}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Add recording indicator
        if self.recording:
            cv2.circle(overlay_frame, (frame.shape[1] - 30, 30), 10, (0, 0, 255), -1)
            cv2.putText(overlay_frame, "REC", 
                       (frame.shape[1] - 70, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # Add topic name
        cv2.putText(overlay_frame, f"Topic: {self.video_topic}", 
                   (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Add controls help
        help_text = "Controls: S=Screenshot, R=Record, I=Info, Q=Quit"
        cv2.putText(overlay_frame, help_text, 
                   (10, frame.shape[0] - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        return overlay_frame
    
    def save_screenshot(self):
        """Save current frame as screenshot"""
        if self.current_frame is not None:
            filename = f"drone_screenshot_{self.frame_count:06d}.jpg"
            cv2.imwrite(filename, self.current_frame)
            self.get_logger().info(f"Screenshot saved: {filename}")
            print(f"Screenshot saved: {filename}")
    
    def toggle_recording(self):
        """Toggle video recording"""
        if not self.recording:
            # Start recording
            if self.current_frame is not None:
                filename = f"drone_video_{self.frame_count:06d}.mp4"
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                fps = 30.0
                frame_size = (self.current_frame.shape[1], self.current_frame.shape[0])
                
                self.video_writer = cv2.VideoWriter(filename, fourcc, fps, frame_size)
                self.recording = True
                self.get_logger().info(f"Recording started: {filename}")
                print(f"Recording started: {filename}")
        else:
            # Stop recording
            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None
                self.recording = False
                self.get_logger().info("Recording stopped")
                print("Recording stopped")
    
    def display_video(self):
        """Main video display loop with ROS2 spinning"""
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        
        print("=== Gazebo Video Streamer ===")
        print("Controls:")
        print("  S - Save screenshot")
        print("  R - Toggle recording")
        print("  I - Toggle info overlay")
        print("  Q - Quit")
        print(f"\nWaiting for video on topic: {self.video_topic}")
        
        while rclpy.ok():
            # Process ROS2 callbacks
            rclpy.spin_once(self, timeout_sec=0.01)
            
            if self.current_frame is not None:
                # Add overlay information
                display_frame = self.add_info_overlay(self.current_frame)
                
                # Record frame if recording is active
                if self.recording and self.video_writer is not None:
                    self.video_writer.write(self.current_frame)
                
                # Display the frame
                cv2.imshow(self.window_name, display_frame)
                
                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q') or key == ord('Q'):
                    break
                elif key == ord('s') or key == ord('S'):
                    self.save_screenshot()
                elif key == ord('r') or key == ord('R'):
                    self.toggle_recording()
                elif key == ord('i') or key == ord('I'):
                    self.show_info_overlay = not self.show_info_overlay
                    print(f"Info overlay: {'ON' if self.show_info_overlay else 'OFF'}")
                    
            else:
                # Show waiting message if no frame received
                waiting_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(waiting_frame, "Waiting for video stream...", 
                           (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.putText(waiting_frame, f"Topic: {self.video_topic}", 
                           (50, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                cv2.putText(waiting_frame, "Press Q to quit", 
                           (50, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.imshow(self.window_name, waiting_frame)
                
                if cv2.waitKey(100) & 0xFF == ord('q'):
                    break
        
        # Cleanup
        if self.recording and self.video_writer is not None:
            self.video_writer.release()
        
        cv2.destroyAllWindows()
        self.get_logger().info("Video streamer stopped")

def main():
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create video streamer node
        streamer = VideoStreamer()
        
        # Run video display in main thread
        streamer.display_video()
        
    except KeyboardInterrupt:
        print("\nShutting down video streamer...")
    
    finally:
        # Cleanup
        try:
            streamer.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == "__main__":
    main()