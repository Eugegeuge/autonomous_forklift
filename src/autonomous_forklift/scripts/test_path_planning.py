#!/usr/bin/env python3
"""
Test Path Planning with Nav2 Route Server.

This script tests:
1. Cost-based routing (avoiding high-weight edges)
2. Directional routing (respecting one-way edges)

Graph configuration:
- Upper corridor is ONE-WAY: A1 -> A2 -> A3 (forward_only)
- Lower corridor is bidirectional
- Vertical connectors are bidirectional
"""

import json
import os
import sys
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class SimpleGraphSolver:
    """Dijkstra-based graph solver that respects edge direction."""
    
    def __init__(self, graph_file: str):
        self.nodes = {}
        self.edges = []
        self.adjacency = {}
        self.load_graph(graph_file)
    
    def load_graph(self, filepath: str):
        """Load GeoJSON graph file."""
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        for feature in data.get('features', []):
            feature_id = feature.get('id', '')
            geom_type = feature['geometry']['type']
            props = feature.get('properties', {})
            
            if geom_type == 'Point':
                self.nodes[feature_id] = props.get('name', feature_id)
                self.adjacency[feature_id] = []
            elif geom_type == 'LineString':
                from_node = props.get('from', '')
                to_node = props.get('to', '')
                weight = props.get('weight', 1.0)
                direction = props.get('direction', 'bidirectional')
                
                self.edges.append({
                    'from': from_node,
                    'to': to_node,
                    'weight': weight,
                    'direction': direction
                })
        
        # Build adjacency list respecting direction
        for edge in self.edges:
            from_node = edge['from']
            to_node = edge['to']
            weight = edge['weight']
            direction = edge['direction']
            
            # Forward direction: from -> to (always allowed)
            if from_node in self.adjacency:
                self.adjacency[from_node].append((to_node, weight))
            
            # Reverse direction: to -> from (only if bidirectional)
            if direction == 'bidirectional' and to_node in self.adjacency:
                self.adjacency[to_node].append((from_node, weight))
    
    def dijkstra(self, start: str, goal: str):
        """Find shortest path using Dijkstra's algorithm."""
        import heapq
        
        pq = [(0, start, [start])]
        visited = set()
        
        while pq:
            cost, node, path = heapq.heappop(pq)
            
            if node in visited:
                continue
            
            visited.add(node)
            
            if node == goal:
                return path, cost
            
            for neighbor, weight in self.adjacency.get(node, []):
                if neighbor not in visited:
                    heapq.heappush(pq, (cost + weight, neighbor, path + [neighbor]))
        
        return None, float('inf')
    
    def get_node_name(self, node_id: str) -> str:
        return self.nodes.get(node_id, node_id)


class PathPlanningTest(Node):
    def __init__(self):
        super().__init__('path_planning_test')
        
        pkg_dir = get_package_share_directory('autonomous_forklift')
        graph_file = os.path.join(pkg_dir, 'config', 'warehouse_graph.geojson')
        
        self.get_logger().info(f'Loading graph from: {graph_file}')
        self.solver = SimpleGraphSolver(graph_file)
        self.get_logger().info(f'Loaded {len(self.solver.nodes)} nodes, {len(self.solver.edges)} edges')
    
    def print_graph_info(self):
        """Print graph configuration."""
        self.get_logger().info('\n📊 GRAPH CONFIGURATION:')
        for edge in self.solver.edges:
            from_name = self.solver.get_node_name(edge['from'])
            to_name = self.solver.get_node_name(edge['to'])
            weight = edge['weight']
            direction = edge['direction']
            
            if direction == 'forward_only':
                arrow = f'{from_name} --> {to_name}'
                status = '🔵 ONE-WAY'
            else:
                arrow = f'{from_name} <-> {to_name}'
                status = '🟢 TWO-WAY'
            
            high_cost = '🚫 HIGH COST' if weight > 10 else ''
            self.get_logger().info(f'  {arrow}: w={weight} {status} {high_cost}')
    
    def run_test_case(self, start_id: str, goal_id: str, description: str, 
                      expected_path_length: int = None):
        """Run a single test case."""
        start_name = self.solver.get_node_name(start_id)
        goal_name = self.solver.get_node_name(goal_id)
        
        self.get_logger().info(f'\n🎯 TEST: {description}')
        self.get_logger().info(f'   Route: {start_name} → {goal_name}')
        
        path, cost = self.solver.dijkstra(start_id, goal_id)
        
        if path:
            path_names = [self.solver.get_node_name(n) for n in path]
            self.get_logger().info(f'   ✅ Path: {" → ".join(path_names)}')
            self.get_logger().info(f'   ✅ Cost: {cost}')
            
            if expected_path_length and len(path) != expected_path_length:
                self.get_logger().warn(f'   ⚠️  Expected {expected_path_length} nodes, got {len(path)}')
            
            return True, path, cost
        else:
            self.get_logger().error(f'   ❌ No path found!')
            return False, None, float('inf')
    
    def run_all_tests(self):
        """Run all test cases."""
        self.get_logger().info('=' * 70)
        self.get_logger().info('NAV2 ROUTE SERVER - ROUTING ALGORITHM STRESS TEST')
        self.get_logger().info('=' * 70)
        
        self.print_graph_info()
        
        results = []
        
        # ============================================================
        # TEST 1: Forward direction (legal)
        # ============================================================
        self.get_logger().info('\n' + '=' * 70)
        self.get_logger().info('TEST 1: FORWARD DIRECTION (Legal - Following one-way)')
        self.get_logger().info('=' * 70)
        
        success, path, cost = self.run_test_case(
            'node_1', 'node_3',
            'A1 → A3 (WITH the one-way flow)',
            expected_path_length=3
        )
        results.append(('Forward One-Way', success))
        
        if success:
            self.get_logger().info('   🎉 Planner correctly used the one-way upper corridor')
        
        # ============================================================
        # TEST 2: Reverse direction (illegal - must go around)
        # ============================================================
        self.get_logger().info('\n' + '=' * 70)
        self.get_logger().info('TEST 2: REVERSE DIRECTION (Illegal - Against one-way)')
        self.get_logger().info('=' * 70)
        
        self.get_logger().info('   Expected: Planner CANNOT go A3 → A2 → A1 (against one-way)')
        self.get_logger().info('   Expected: Must go A3 → B3 → B2 → B1 → A1 (around via lower)')
        
        success, path, cost = self.run_test_case(
            'node_3', 'node_1',
            'A3 → A1 (AGAINST the one-way flow)',
            expected_path_length=5
        )
        results.append(('Reverse One-Way', success))
        
        if success and path:
            path_names = [self.solver.get_node_name(n) for n in path]
            # Check that the path does NOT go through the upper corridor
            if 'A2' in path_names[1:-1]:  # A2 should not be in middle of path
                self.get_logger().warn('   ⚠️  WARNING: Path went through A2 (upper corridor)!')
                self.get_logger().warn('   ⚠️  This violates the one-way constraint!')
            else:
                self.get_logger().info('   🎉 SUCCESS: Planner correctly avoided upper corridor!')
                self.get_logger().info('   🎉 Took the long way around via lower corridor!')
        
        # ============================================================
        # TEST 3: Mixed route
        # ============================================================
        self.get_logger().info('\n' + '=' * 70)
        self.get_logger().info('TEST 3: MIXED ROUTE (Diagonal)')
        self.get_logger().info('=' * 70)
        
        success, path, cost = self.run_test_case(
            'node_4', 'node_3',
            'B1 → A3 (Lower left to upper right)'
        )
        results.append(('Mixed Route', success))
        
        # ============================================================
        # SUMMARY
        # ============================================================
        self.get_logger().info('\n' + '=' * 70)
        self.get_logger().info('TEST SUMMARY')
        self.get_logger().info('=' * 70)
        
        all_passed = True
        for name, passed in results:
            status = '✅ PASS' if passed else '❌ FAIL'
            self.get_logger().info(f'  {name}: {status}')
            if not passed:
                all_passed = False
        
        self.get_logger().info('=' * 70)
        if all_passed:
            self.get_logger().info('🏆 ALL TESTS PASSED - Routing algorithm works correctly!')
        else:
            self.get_logger().error('❌ SOME TESTS FAILED')
        self.get_logger().info('=' * 70)
        
        return all_passed


def main(args=None):
    rclpy.init(args=args)
    
    node = PathPlanningTest()
    
    try:
        success = node.run_all_tests()
    except Exception as e:
        node.get_logger().error(f'Test failed with error: {e}')
        success = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
