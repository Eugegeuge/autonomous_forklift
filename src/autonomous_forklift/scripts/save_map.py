#!/usr/bin/env python3
"""
Script to save the current SLAM map.

This script saves the current map from SLAM Toolbox to the maps directory.
It can be run while the mapping launch file is active.

Usage:
    ros2 run autonomous_forklift save_map.py [map_name]
    
Example:
    ros2 run autonomous_forklift save_map.py warehouse_new
    ros2 run autonomous_forklift save_map.py  # Uses default name: mundo_map
"""

import os
import sys
import subprocess
from datetime import datetime


def main():
    # Default map name
    map_name = "mundo_map"
    
    # Check if custom name provided
    if len(sys.argv) > 1:
        map_name = sys.argv[1]
    
    # Determine save path
    # Try to find the package source directory
    possible_paths = [
        os.path.expanduser("~/ros2_ws/autonomous_forklift/src/autonomous_forklift/maps"),
        "/root/ros2_ws/autonomous_forklift/src/autonomous_forklift/maps",
    ]
    
    save_dir = None
    for path in possible_paths:
        if os.path.exists(path):
            save_dir = path
            break
    
    if save_dir is None:
        # Create in current directory
        save_dir = os.getcwd()
        print(f"Warning: Could not find maps directory, saving to: {save_dir}")
    
    map_path = os.path.join(save_dir, map_name)
    
    # Create backup of existing map if it exists
    if os.path.exists(f"{map_path}.pgm"):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_name = f"{map_name}_backup_{timestamp}"
        backup_pgm = os.path.join(save_dir, f"{backup_name}.pgm")
        backup_yaml = os.path.join(save_dir, f"{backup_name}.yaml")
        
        os.rename(f"{map_path}.pgm", backup_pgm)
        if os.path.exists(f"{map_path}.yaml"):
            os.rename(f"{map_path}.yaml", backup_yaml)
        print(f"Existing map backed up as: {backup_name}")
    
    print(f"\n{'='*50}")
    print(f"Saving map to: {map_path}")
    print(f"{'='*50}\n")
    
    # Run the map saver
    try:
        result = subprocess.run(
            [
                "ros2", "run", "nav2_map_server", "map_saver_cli",
                "-f", map_path,
                "--ros-args", "-p", "use_sim_time:=true"
            ],
            capture_output=True,
            text=True,
            timeout=30
        )
        
        if result.returncode == 0:
            print(f"\n✅ Map saved successfully!")
            print(f"   - {map_path}.pgm")
            print(f"   - {map_path}.yaml")
            
            # Check file sizes
            pgm_size = os.path.getsize(f"{map_path}.pgm") / 1024
            print(f"   - Size: {pgm_size:.1f} KB")
        else:
            print(f"\n❌ Error saving map:")
            print(result.stderr)
            
    except subprocess.TimeoutExpired:
        print("\n❌ Timeout: Map saving took too long. Is the map topic available?")
        print("   Make sure SLAM Toolbox is running and publishing the map.")
    except FileNotFoundError:
        print("\n❌ Error: nav2_map_server not found.")
        print("   Install it with: sudo apt install ros-humble-nav2-map-server")
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")


if __name__ == "__main__":
    main()
