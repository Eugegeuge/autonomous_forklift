#!/usr/bin/env python3
"""
Navigation Node for Autonomous Forklift.

Integrates with the Interface Node to execute navigation tasks.
Calculates paths using BFS on the warehouse graph.
"""

import json
import os
import math
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Point
import tf_transformations

from rclpy.qos import qos_profile_sensor_data

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        # Parameters
        self.declare_parameter('graph_file', '')
        self.declare_parameter('linear_speed', 2.0)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('distance_tolerance', 0.8)
        self.declare_parameter('angle_tolerance', 0.1)
        
        graph_file = self.get_parameter('graph_file').get_parameter_value().string_value
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        self.distance_tolerance = self.get_parameter('distance_tolerance').get_parameter_value().double_value
        self.angle_tolerance = self.get_parameter('angle_tolerance').get_parameter_value().double_value
        
        # TF Buffer for Localization (Map Frame)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Default graph file
        if not graph_file:
            pkg_dir = get_package_share_directory('autonomous_forklift')
            graph_file = os.path.join(pkg_dir, 'config', 'warehouse_graph.geojson')
        
        # Load graph
        self.nodes = {}
        self.edges = {}
        self.load_graph(graph_file)
        
        # MAPPING (Dynamic from Graph)
        self.location_map = {}
        for nid, data in self.nodes.items():
            if 'name' in data and data['name']:
                self.location_map[data['name']] = nid
        
        # Fallback if empty
        if not self.location_map:
            self.get_logger().warn("No named nodes found in graph! Using default mapping.")
            self.location_map = {"HOME": "node_4"}
            
        self.get_logger().info(f"Location Map loaded: {self.location_map}")
        
        # State
        self.current_pose = None
        self.state = 'IDLE'  # IDLE, PLANNING, ROTATING, MOVING, DONE
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.active_goal = None
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        self.pub_nav_status = self.create_publisher(String, 'navigation_status', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        
        # Interface Subscribers
        self.sub_nav = self.create_subscription(String, 'navegacion', self.nav_callback, 10)
        self.sub_goal = self.create_subscription(String, 'navigation_goal', self.goal_callback, 10)
        
        # Timers
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.marker_timer = self.create_timer(1.0, self.publish_markers)
        
        self.obstacle_detected = False
        self.nav_enabled = True # Default to True for testing
        self.get_logger().info('Navigation Node initialized. Waiting for commands...')

    def nav_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        if command == 'ON':
            self.nav_enabled = True
        elif command == 'OFF':
            self.nav_enabled = False
            self.stop_robot()
            self.state = 'IDLE'

    def goal_callback(self, msg):
        goal_name = msg.data
        self.get_logger().info(f'Received goal: {goal_name}')
        self.active_goal = goal_name
        self.plan_path(goal_name)

    def load_graph(self, filepath):
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            
            for feature in data.get('features', []):
                props = feature.get('properties', {})
                fid = feature.get('id')
                
                if feature['geometry']['type'] == 'Point':
                    coords = feature['geometry']['coordinates']
                    self.nodes[fid] = {'x': coords[0], 'y': coords[1], 'name': props.get('name', fid)}
                    self.edges[fid] = [] # Initialize adjacency
                    
                elif feature['geometry']['type'] == 'LineString':
                    u = props.get('from')
                    v = props.get('to')
                    if u and v:
                        # Add edges (assuming bidirectional for now unless specified)
                        if u not in self.edges: self.edges[u] = []
                        if v not in self.edges: self.edges[v] = []
                        
                        self.edges[u].append(v)
                        self.edges[v].append(u) # Assuming bidirectional for simplicity
                        
            self.get_logger().info(f'Loaded {len(self.nodes)} nodes and edges.')
        except Exception as e:
            self.get_logger().error(f'Failed to load graph: {e}')

    def get_nearest_node(self):
        if not self.current_pose: return None
        min_dist = float('inf')
        nearest = None
        for nid, data in self.nodes.items():
            dist = math.sqrt((data['x'] - self.current_pose['x'])**2 + (data['y'] - self.current_pose['y'])**2)
            if dist < min_dist:
                min_dist = dist
                nearest = nid
        return nearest

    def plan_path(self, goal_name):
        self.get_logger().info(f'Attempting to plan path to: {goal_name}')
        
        target_node_id = self.location_map.get(goal_name)
        if not target_node_id:
            self.get_logger().error(f'Unknown location: {goal_name}')
            return

        start_node_id = self.get_nearest_node()
        if not start_node_id:
            self.get_logger().warn('Cannot plan: Robot position unknown (No Odom yet?)')
            return

        self.get_logger().info(f'Planning path: {start_node_id} -> {target_node_id}')
        
        # BFS Pathfinding
        queue = [[start_node_id]]
        visited = {start_node_id}
        path = []
        
        if start_node_id == target_node_id:
            path = [start_node_id]
        else:
            while queue:
                path = queue.pop(0)
                node = path[-1]
                if node == target_node_id:
                    break
                for neighbor in self.edges.get(node, []):
                    if neighbor not in visited:
                        visited.add(neighbor)
                        new_path = list(path)
                        new_path.append(neighbor)
                        queue.append(new_path)
            else:
                self.get_logger().error('No path found!')
                return

        # Convert path nodes to coordinates
        self.waypoints = []
        for nid in path:
            n = self.nodes[nid]
            self.waypoints.append((n['x'], n['y']))
            
        self.current_waypoint_idx = 0
        self.state = 'ROTATING'
        self.publish_status("MOVING")
        self.get_logger().info(f'Path found with {len(self.waypoints)} waypoints. Starting navigation.')

    def scan_callback(self, msg):
        # Obstacle detection disabled by user request
        self.obstacle_detected = False

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        quaternion = [ori.x, ori.y, ori.z, ori.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)
        
        if self.current_pose is None:
             self.get_logger().info(f'✅ First Odom received! Pose: ({pos.x:.2f}, {pos.y:.2f})')
             
        self.current_pose = {'x': pos.x, 'y': pos.y, 'yaw': yaw}

    def control_loop(self):
        if self.state in ['IDLE', 'DONE'] or not self.current_pose:
            return

        # Obstacle check removed

        if self.current_waypoint_idx >= len(self.waypoints):
            # Reached last waypoint position, now ALIGN
            if self.state != 'ALIGNING':
                self.state = 'ALIGNING'
                # Calculate nearest cardinal angle (0, 90, 180, 270 deg)
                yaw = self.current_pose['yaw']
                # Normalize to 0..2pi for easier rounding? No, -pi..pi is fine.
                # pi/2 = 1.5708
                # Divide by pi/2, round, multiply by pi/2
                steps = round(yaw / (math.pi / 2.0))
                self.target_yaw = steps * (math.pi / 2.0)
                self.get_logger().info(f'📍 Position reached. Aligning to nearest cardinal: {math.degrees(self.target_yaw):.0f} deg')
            
            # Execute Alignment
            angle_error = self.normalize_angle(self.target_yaw - self.current_pose['yaw'])
            
            if abs(angle_error) < 0.05: # ~3 degrees tolerance
                self.state = 'DONE'
                self.stop_robot()
                self.publish_status("REACHED")
                self.active_goal = None
                self.get_logger().info('🎉 Destination reached & Aligned!')
            else:
                cmd = Twist()
                cmd.angular.z = 0.8 * angle_error
                cmd.angular.z = max(-0.5, min(0.5, cmd.angular.z)) # Slower rotation for alignment
                self.cmd_vel_pub.publish(cmd)
            
            return

        target_x, target_y = self.waypoints[self.current_waypoint_idx]
        dx = target_x - self.current_pose['x']
        dy = target_y - self.current_pose['y']
        distance = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.current_pose['yaw'])

        cmd = Twist()

        if distance < self.distance_tolerance:
            self.current_waypoint_idx += 1
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_idx}')
            return
        
        # State machine navigation controller
        if self.state == 'ROTATING':
            if abs(angle_error) > self.angle_tolerance:
                cmd.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
            else:
                self.state = 'MOVING'
        elif self.state == 'MOVING':
            # Slow down when close to target
            target_speed = self.linear_speed
            if distance < 1.5:
                target_speed = self.linear_speed * 0.5
            
            cmd.linear.x = target_speed
            
            # Smoother angular control
            cmd.angular.z = 0.8 * angle_error
            cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, cmd.angular.z))
            
            self.get_logger().info(f'NAV: v={cmd.linear.x:.2f} w={cmd.angular.z:.2f} dist={distance:.2f}', throttle_duration_sec=0.5)
            
            # If we overshoot significantly (angle error > 45 deg), switch back to rotating
            if abs(angle_error) > 0.8: # ~45 degrees
                self.state = 'ROTATING'
                cmd.linear.x = 0.0

        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.pub_nav_status.publish(msg)

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def publish_markers(self):
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'path'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.2
            marker.scale.x = 0.3; marker.scale.y = 0.3; marker.scale.z = 0.3
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

def main(args=None):
    print("🚀 STARTING WAYPOINT FOLLOWER NODE...", flush=True)
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
