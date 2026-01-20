#!/usr/bin/env python3
"""
Waypoint Follower Node for Autonomous Forklift.

This node reads routes from the warehouse graph and commands the robot
to navigate through waypoints sequentially using velocity commands.

Usage:
  ros2 run autonomous_forklift waypoint_follower.py --ros-args -p route:="A1,A2,A3,B3"
"""

import json
import os
import math
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
import tf_transformations


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        
        # Parameters
        self.declare_parameter('route', 'A1,A2,A3')  # Comma-separated waypoint names
        self.declare_parameter('graph_file', '')
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('distance_tolerance', 0.3)
        self.declare_parameter('angle_tolerance', 0.1)
        
        route_str = self.get_parameter('route').get_parameter_value().string_value
        graph_file = self.get_parameter('graph_file').get_parameter_value().string_value
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        self.distance_tolerance = self.get_parameter('distance_tolerance').get_parameter_value().double_value
        self.angle_tolerance = self.get_parameter('angle_tolerance').get_parameter_value().double_value
        
        # Default graph file
        if not graph_file:
            pkg_dir = get_package_share_directory('autonomous_forklift')
            graph_file = os.path.join(pkg_dir, 'config', 'warehouse_graph.geojson')
        
        # Load graph
        self.nodes = {}
        self.load_graph(graph_file)
        
        # Parse route
        self.waypoint_names = [w.strip() for w in route_str.split(',')]
        self.waypoints = self.get_waypoint_coordinates(self.waypoint_names)
        
        if not self.waypoints:
            self.get_logger().error('No valid waypoints found!')
            return
        
        self.get_logger().info(f'Route: {" -> ".join(self.waypoint_names)}')
        self.get_logger().info(f'Waypoints: {len(self.waypoints)} points')
        
        # State
        self.current_waypoint_idx = 0
        self.current_pose = None
        self.state = 'WAITING'  # WAITING, ROTATING, MOVING, REACHED, DONE
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        # Control timer (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Marker timer (1 Hz)
        self.marker_timer = self.create_timer(1.0, self.publish_markers)
        
        self.get_logger().info('Waypoint Follower initialized. Waiting for odometry...')
    
    def load_graph(self, filepath: str):
        """Load node positions from GeoJSON."""
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            
            for feature in data.get('features', []):
                if feature['geometry']['type'] == 'Point':
                    coords = feature['geometry']['coordinates']
                    name = feature.get('properties', {}).get('name', feature.get('id', ''))
                    self.nodes[name] = {'x': coords[0], 'y': coords[1]}
                    self.get_logger().debug(f'Loaded node: {name} at ({coords[0]}, {coords[1]})')
            
            self.get_logger().info(f'Loaded {len(self.nodes)} nodes from graph')
        except Exception as e:
            self.get_logger().error(f'Failed to load graph: {e}')
    
    def get_waypoint_coordinates(self, names: list) -> list:
        """Convert waypoint names to (x, y) coordinates."""
        coords = []
        for name in names:
            if name in self.nodes:
                coords.append((self.nodes[name]['x'], self.nodes[name]['y']))
            else:
                self.get_logger().warn(f'Unknown waypoint: {name}')
        return coords
    
    def odom_callback(self, msg: Odometry):
        """Update current robot pose from odometry."""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        # Extract yaw from quaternion
        quaternion = [ori.x, ori.y, ori.z, ori.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)
        
        self.current_pose = {
            'x': pos.x,
            'y': pos.y,
            'yaw': yaw
        }
        
        if self.state == 'WAITING':
            self.state = 'ROTATING'
            self.get_logger().info('Odometry received. Starting navigation!')
    
    def control_loop(self):
        """Main control loop - runs at 10 Hz."""
        if self.current_pose is None or self.state == 'DONE' or self.state == 'WAITING':
            return
        
        if self.current_waypoint_idx >= len(self.waypoints):
            self.state = 'DONE'
            self.stop_robot()
            self.get_logger().info('🎉 All waypoints reached! Route complete.')
            return
        
        # Get current target
        target_x, target_y = self.waypoints[self.current_waypoint_idx]
        target_name = self.waypoint_names[self.current_waypoint_idx]
        
        # Calculate distance and angle to target
        dx = target_x - self.current_pose['x']
        dy = target_y - self.current_pose['y']
        distance = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.current_pose['yaw'])
        
        cmd = Twist()
        
        if distance < self.distance_tolerance:
            # Waypoint reached
            self.get_logger().info(f'✅ Reached waypoint {target_name} '
                                   f'({self.current_waypoint_idx + 1}/{len(self.waypoints)})')
            self.current_waypoint_idx += 1
            self.state = 'ROTATING'
            self.stop_robot()
            return
        
        if self.state == 'ROTATING':
            # First rotate to face the target
            if abs(angle_error) > self.angle_tolerance:
                cmd.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
            else:
                self.state = 'MOVING'
                self.get_logger().info(f'➡️  Moving towards {target_name} (distance: {distance:.2f}m)')
        
        elif self.state == 'MOVING':
            # Move towards target while adjusting heading
            cmd.linear.x = self.linear_speed
            # Proportional heading correction
            cmd.angular.z = 2.0 * angle_error
            cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, cmd.angular.z))
        
        self.cmd_vel_pub.publish(cmd)
    
    def stop_robot(self):
        """Send zero velocity command."""
        self.cmd_vel_pub.publish(Twist())
    
    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def publish_markers(self):
        """Publish visualization markers for waypoints."""
        marker_array = MarkerArray()
        
        for i, (x, y) in enumerate(self.waypoints):
            # Waypoint sphere
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.2
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 0.4
            
            # Color: green if reached, yellow if current, red if pending
            if i < self.current_waypoint_idx:
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
            elif i == self.current_waypoint_idx:
                marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
            else:
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.6)
            
            marker_array.markers.append(marker)
            
            # Label
            label = Marker()
            label.header.frame_id = 'odom'
            label.header.stamp = self.get_clock().now().to_msg()
            label.ns = 'waypoint_labels'
            label.id = i + 100
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.6
            label.pose.orientation.w = 1.0
            label.scale.z = 0.3
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = self.waypoint_names[i]
            marker_array.markers.append(label)
        
        # Path line
        if len(self.waypoints) > 1:
            path_marker = Marker()
            path_marker.header.frame_id = 'odom'
            path_marker.header.stamp = self.get_clock().now().to_msg()
            path_marker.ns = 'path'
            path_marker.id = 0
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.scale.x = 0.1
            path_marker.color = ColorRGBA(r=0.0, g=0.8, b=1.0, a=0.7)
            
            for x, y in self.waypoints:
                p = Point()
                p.x = x
                p.y = y
                p.z = 0.1
                path_marker.points.append(p)
            
            marker_array.markers.append(path_marker)
        
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    
    node = WaypointFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping waypoint follower...')
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
