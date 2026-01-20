#!/usr/bin/env python3
"""
Map Builder Tool for Autonomous Forklift.

This node allows creating a warehouse graph by clicking points in RViz.
- Subscribes to /clicked_point (RViz "Publish Point" tool)
- Saves nodes to a GeoJSON file
- Automatically names nodes (N1, N2, N3...)

Usage:
  ros2 run autonomous_forklift map_builder.py
"""

import json
import os
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from ament_index_python.packages import get_package_share_directory


class MapBuilder(Node):
    def __init__(self):
        super().__init__('map_builder')
        
        # Parameters
        self.declare_parameter('output_file', 'new_graph.geojson')
        output_filename = self.get_parameter('output_file').get_parameter_value().string_value
        
        # Determine output path
        pkg_dir = get_package_share_directory('autonomous_forklift')
        self.output_path = os.path.join(pkg_dir, 'config', output_filename)
        
        self.nodes = []
        self.node_counter = 1
        
        # Subscribers
        self.click_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.click_callback,
            10
        )
        
        # Publishers (for visualization)
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/map_builder_markers', qos_profile)
        
        self.get_logger().info('🗺️  MAP BUILDER STARTED')
        self.get_logger().info(f'   Output file: {self.output_path}')
        self.get_logger().info('   👉 Click "Publish Point" in RViz to add nodes.')
        self.get_logger().info('   💾 Press Ctrl+C to save and exit.')

    def click_callback(self, msg):
        """Handle clicked points from RViz."""
        x = msg.point.x
        y = msg.point.y
        
        node_name = f"N{self.node_counter}"
        
        # Add to list
        self.nodes.append({
            "id": f"node_{self.node_counter}",
            "name": node_name,
            "x": x,
            "y": y
        })
        
        self.get_logger().info(f'   📍 Added Node: {node_name} at ({x:.2f}, {y:.2f})')
        
        self.node_counter += 1
        self.publish_markers()

    def publish_markers(self):
        """Visualize current nodes."""
        marker_array = MarkerArray()
        
        for i, node in enumerate(self.nodes):
            # Sphere
            marker = Marker()
            marker.header.frame_id = 'map' # Assuming map frame for building
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'builder_nodes'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = node['x']
            marker.pose.position.y = node['y']
            marker.pose.position.z = 0.2
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0) # Purple
            marker_array.markers.append(marker)
            
            # Text
            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = 'builder_labels'
            text.id = i + 1000
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = node['x']
            text.pose.position.y = node['y']
            text.pose.position.z = 0.6
            text.pose.orientation.w = 1.0
            text.scale.z = 0.3
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = node['name']
            marker_array.markers.append(text)
            
        self.marker_pub.publish(marker_array)

    def save_geojson(self):
        """Save nodes to GeoJSON file."""
        if not self.nodes:
            self.get_logger().warn('No nodes to save.')
            return

        features = []
        
        # Create Node Features
        for node in self.nodes:
            feature = {
                "type": "Feature",
                "id": node["id"],
                "geometry": {
                    "type": "Point",
                    "coordinates": [node["x"], node["y"]]
                },
                "properties": {
                    "name": node["name"],
                    "description": "Created with Map Builder"
                }
            }
            features.append(feature)
        
        # Create Edge Features (Simple chain for now: N1-N2-N3...)
        # User can edit edges manually later, or we can improve this tool
        for i in range(len(self.nodes) - 1):
            n1 = self.nodes[i]
            n2 = self.nodes[i+1]
            
            edge_feature = {
                "type": "Feature",
                "id": f"edge_{i+1}_{i+2}",
                "geometry": {
                    "type": "LineString",
                    "coordinates": [
                        [n1["x"], n1["y"]],
                        [n2["x"], n2["y"]]
                    ]
                },
                "properties": {
                    "from": n1["id"],
                    "to": n2["id"],
                    "weight": 1.0,
                    "direction": "bidirectional"
                }
            }
            features.append(edge_feature)

        geojson = {
            "type": "FeatureCollection",
            "features": features
        }
        
        try:
            with open(self.output_path, 'w') as f:
                json.dump(geojson, f, indent=2)
            self.get_logger().info(f'✅ Saved graph to: {self.output_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to save file: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MapBuilder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_geojson()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
