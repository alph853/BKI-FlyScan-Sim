#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import quaternion_from_euler


class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')
        
        # Declare and get parameters
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        
        if not self.has_parameter('max_path_length'):
            self.declare_parameter('max_path_length', 1000)
        
        self.max_path_length = self.get_parameter('max_path_length').get_parameter_value().integer_value
        
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.path_pub = self.create_publisher(Path, '/odom_path', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Initialize path
        self.odom_path = Path()
        
        self.get_logger().info("Visualization node started with path visualization")
        
        # Timer for publishing markers
        self.timer = self.create_timer(1.0, self.publish_markers)

    def publish_markers(self):
        marker_array = MarkerArray()
        
        # Clear all markers first
        clear_marker = Marker()
        clear_marker.header.frame_id = "map"
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        # Add coordinate system at origin
        self.add_coordinate_system(marker_array, 0.0, 0.0, 0.0, "map_origin", 0)
        
        # Add text marker
        self.add_text_marker(marker_array, 0.0, 0.0, 2.0, "origin", "origin_text", 200)
        
        self.marker_pub.publish(marker_array)

    def add_coordinate_system(self, marker_array, x, y, z, ns, id_base):
        # X-axis (i) - Red
        qx, qy, qz, qw = quaternion_from_euler(0, 0, 0)
        x_axis = self.create_axis_marker(x, y, z, qx, qy, qz, qw, 
                                       1.0, 0.0, 0.0, 1.0, f"{ns}_x", id_base)
        marker_array.markers.append(x_axis)
        
        # Y-axis (j) - Green
        qx, qy, qz, qw = quaternion_from_euler(0, 0, math.pi/2)
        y_axis = self.create_axis_marker(x, y, z, qx, qy, qz, qw,
                                       0.0, 1.0, 0.0, 1.0, f"{ns}_y", id_base + 1)
        marker_array.markers.append(y_axis)
        
        # Z-axis (k) - Blue
        qx, qy, qz, qw = quaternion_from_euler(0, -math.pi/2, 0)
        z_axis = self.create_axis_marker(x, y, z, qx, qy, qz, qw,
                                       0.0, 0.0, 1.0, 1.0, f"{ns}_z", id_base + 2)
        marker_array.markers.append(z_axis)

    def create_axis_marker(self, x, y, z, qx, qy, qz, qw, r, g, b, a, ns, id):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        
        marker.scale.x = 10.0  # Length
        marker.scale.y = 0.05  # Width
        marker.scale.z = 0.05  # Height
        
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a
        
        marker.lifetime = Duration(seconds=0).to_msg()
        
        return marker

    def add_text_marker(self, marker_array, x, y, z, text, ns, id):
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = ns
        text_marker.id = id
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        text_marker.pose.position.x = x
        text_marker.pose.position.y = y
        text_marker.pose.position.z = z
        text_marker.pose.orientation.x = 0.0
        text_marker.pose.orientation.y = 0.0
        text_marker.pose.orientation.z = 0.0
        text_marker.pose.orientation.w = 1.0
        
        text_marker.scale.z = 0.5  # Text height
        
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        
        text_marker.text = text
        text_marker.lifetime = Duration(seconds=0).to_msg()
        
        marker_array.markers.append(text_marker)

    def odom_callback(self, msg):
        # Create PoseStamped from odometry message
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        
        # Update path
        self.odom_path.header = msg.header
        self.odom_path.poses.append(pose_stamped)
        
        # Limit path length to prevent memory issues
        if len(self.odom_path.poses) > self.max_path_length:
            self.odom_path.poses.pop(0)
        
        # Publish path visualization
        self.path_pub.publish(self.odom_path)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()