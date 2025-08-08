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
            
        # Multi-drone support - list of active drone IDs to visualize
        if not self.has_parameter('active_drone_ids'):
            self.declare_parameter('active_drone_ids', [0])  # Default to single drone
        
        self.max_path_length = self.get_parameter('max_path_length').get_parameter_value().integer_value
        self.active_drone_ids = self.get_parameter('active_drone_ids').get_parameter_value().integer_array_value
        
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        
        # Multi-drone data structures
        self.path_publishers = {}  # drone_id -> path publisher
        self.odom_subscribers = {}  # drone_id -> odometry subscriber  
        self.drone_paths = {}  # drone_id -> Path message
        
        # Create publishers and subscribers for each active drone
        for drone_id in self.active_drone_ids:
            # Create path publisher for each drone
            path_topic = f'/odom_path_drone_{drone_id}' if drone_id != 0 else '/odom_path'
            self.path_publishers[drone_id] = self.create_publisher(Path, path_topic, 10)
            
            # Create odometry subscriber for each drone
            odom_topic = f'/px4_{drone_id}/odom' if drone_id != 0 else '/odom'
            self.odom_subscribers[drone_id] = self.create_subscription(
                Odometry,
                odom_topic,
                lambda msg, did=drone_id: self.odom_callback(msg, did),
                10
            )
            
            # Initialize path for each drone
            self.drone_paths[drone_id] = Path()
        
        # Subscribe to frontier array instead of individual exploration goal
        # TODO: Update to use /frontiers_ranked topic with FrontierArray message type
        # self.frontier_sub = self.create_subscription(
        #     FrontierArray,  # Will need to import flyscan_interfaces.msg.FrontierArray
        #     '/frontiers_ranked',
        #     self.frontiers_callback,
        #     10
        # )
        
        # Initialize frontier data (shared across all drones)
        self.latest_frontier = None
        self.frontier_history = []  # Keep track of recent frontiers
        self.max_frontier_history = 50  # Maximum number of frontier points to keep
        
        # Define colors for different drones
        self.drone_colors = {
            1: (1.0, 0.0, 0.0),  # Red
            2: (0.0, 1.0, 0.0),  # Green  
            3: (0.0, 0.0, 1.0),  # Blue
            4: (1.0, 1.0, 0.0),  # Yellow
            5: (1.0, 0.0, 1.0),  # Magenta
            6: (0.0, 1.0, 1.0),  # Cyan
        }
        
        self.get_logger().info(f"Visualization node started for drones: {self.active_drone_ids}")
        
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
        
        # Add frontier visualization
        self.add_frontier_markers(marker_array)
        
        # Add drone position markers
        self.add_drone_markers(marker_array)
        
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

    def odom_callback(self, msg, drone_id):
        # Create PoseStamped from odometry message
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        
        # Update path for specific drone
        if drone_id in self.drone_paths:
            self.drone_paths[drone_id].header = msg.header
            self.drone_paths[drone_id].poses.append(pose_stamped)
            
            # Limit path length to prevent memory issues
            if len(self.drone_paths[drone_id].poses) > self.max_path_length:
                self.drone_paths[drone_id].poses.pop(0)
            
            # Publish path visualization for this drone
            if drone_id in self.path_publishers:
                self.path_publishers[drone_id].publish(self.drone_paths[drone_id])
    
    def frontier_callback(self, msg):
        """Handle frontier goal messages and update visualization."""
        self.latest_frontier = msg
        
        # Add to frontier history
        frontier_point = {
            'position': msg.pose.position,
            'timestamp': msg.header.stamp
        }
        self.frontier_history.append(frontier_point)
        
        # Limit history size
        if len(self.frontier_history) > self.max_frontier_history:
            self.frontier_history.pop(0)
        
        self.get_logger().debug(f"Received frontier goal at ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})")
    
    def add_frontier_markers(self, marker_array):
        """Add frontier visualization markers to the marker array."""
        id_base = 1000  # Use high IDs to avoid conflicts with other markers
        
        # Current frontier marker (larger, bright red)
        if self.latest_frontier is not None:
            frontier_marker = Marker()
            frontier_marker.header.frame_id = "map"
            frontier_marker.header.stamp = self.get_clock().now().to_msg()
            frontier_marker.ns = "current_frontier"
            frontier_marker.id = id_base
            frontier_marker.type = Marker.SPHERE
            frontier_marker.action = Marker.ADD
            
            frontier_marker.pose.position = self.latest_frontier.pose.position
            frontier_marker.pose.orientation.w = 1.0
            
            frontier_marker.scale.x = 0.8
            frontier_marker.scale.y = 0.8
            frontier_marker.scale.z = 0.8
            
            frontier_marker.color.r = 1.0
            frontier_marker.color.g = 0.0
            frontier_marker.color.b = 0.0
            frontier_marker.color.a = 1.0
            
            frontier_marker.lifetime = Duration(seconds=0).to_msg()
            marker_array.markers.append(frontier_marker)
        
        # Historical frontier markers (smaller, fading)
        for i, frontier_data in enumerate(self.frontier_history[:-1]):  # Exclude the latest one
            if i % 3 == 0:  # Show every 3rd historical frontier to avoid clutter
                hist_marker = Marker()
                hist_marker.header.frame_id = "map"
                hist_marker.header.stamp = self.get_clock().now().to_msg()
                hist_marker.ns = "frontier_history"
                hist_marker.id = id_base + 1 + i
                hist_marker.type = Marker.SPHERE
                hist_marker.action = Marker.ADD
                
                hist_marker.pose.position = frontier_data['position']
                hist_marker.pose.orientation.w = 1.0
                
                hist_marker.scale.x = 0.3
                hist_marker.scale.y = 0.3
                hist_marker.scale.z = 0.3
                
                # Fade older frontiers
                alpha = max(0.2, (i / len(self.frontier_history)) * 0.8)
                hist_marker.color.r = 1.0
                hist_marker.color.g = 0.5
                hist_marker.color.b = 0.0
                hist_marker.color.a = alpha
                
                hist_marker.lifetime = Duration(seconds=0).to_msg()
                marker_array.markers.append(hist_marker)

    def add_drone_markers(self, marker_array):
        """Add drone position markers to the marker array."""
        id_base = 2000  # Use high IDs to avoid conflicts
        
        for drone_id in self.active_drone_ids:
            if drone_id in self.drone_paths and self.drone_paths[drone_id].poses:
                # Get latest position for this drone
                latest_pose = self.drone_paths[drone_id].poses[-1]
                
                # Get color for this drone
                color = self.drone_colors.get(drone_id, (0.5, 0.5, 0.5))  # Default gray
                
                # Create drone marker
                drone_marker = Marker()
                drone_marker.header.frame_id = "map"
                drone_marker.header.stamp = self.get_clock().now().to_msg()
                drone_marker.ns = f"drone_{drone_id}"
                drone_marker.id = id_base + drone_id
                drone_marker.type = Marker.CUBE
                drone_marker.action = Marker.ADD
                
                drone_marker.pose = latest_pose.pose
                
                # Drone size
                drone_marker.scale.x = 0.5
                drone_marker.scale.y = 0.5
                drone_marker.scale.z = 0.2
                
                # Drone color
                drone_marker.color.r = color[0]
                drone_marker.color.g = color[1]
                drone_marker.color.b = color[2]
                drone_marker.color.a = 1.0
                
                drone_marker.lifetime = Duration(seconds=0).to_msg()
                marker_array.markers.append(drone_marker)
                
                # Add drone label
                label_marker = Marker()
                label_marker.header.frame_id = "map"
                label_marker.header.stamp = self.get_clock().now().to_msg()
                label_marker.ns = f"drone_label_{drone_id}"
                label_marker.id = id_base + drone_id + 100
                label_marker.type = Marker.TEXT_VIEW_FACING
                label_marker.action = Marker.ADD
                
                label_marker.pose.position.x = latest_pose.pose.position.x
                label_marker.pose.position.y = latest_pose.pose.position.y
                label_marker.pose.position.z = latest_pose.pose.position.z + 0.5
                label_marker.pose.orientation.w = 1.0
                
                label_marker.scale.z = 0.3
                
                label_marker.color.r = 1.0
                label_marker.color.g = 1.0
                label_marker.color.b = 1.0
                label_marker.color.a = 1.0
                
                label_marker.text = f"Drone {drone_id}"
                label_marker.lifetime = Duration(seconds=0).to_msg()
                marker_array.markers.append(label_marker)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()