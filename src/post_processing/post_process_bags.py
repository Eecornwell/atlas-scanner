#!/usr/bin/env python3
# DEPRECATED: This script is not currently used (ENABLE_POST_PROCESSING=false by default)
# It has dependencies on external scripts that need to be migrated

import os
import sys
import subprocess
import yaml

def post_process_bags(scan_dir):
    """Post-process all bag files in scan directory"""
    print("ERROR: post_process_bags.py is deprecated and not fully migrated.")
    print("Use the built-in post-processing in terrestrial_fusion_with_lio.sh instead.")
    print("Set AUTO_CREATE_COLORED=true to automatically color point clouds.")
    return
    
    print(f"Post-processing bags in: {scan_dir}")
    
    # Load fusion calibration
    # Use relative path from script location
    config_paths = [
        os.path.join(os.path.dirname(__file__), '..', 'config', 'fusion_calibration.yaml')
    ]
    
    config_path = None
    for path in config_paths:
        if os.path.exists(path):
            config_path = path
            print(f"Using calibration from: {config_path}")
            break
    
    if config_path is None:
        raise FileNotFoundError(f"fusion_calibration.yaml not found in any of: {config_paths}")
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    # Temporarily set skip_rate to 1 for full resolution
    original_skip_rate = config.get('skip_rate', 1)
    config['skip_rate'] = 1
    
    # Save temporary config in scan directory instead of hardcoded path
    temp_config_path = os.path.join(scan_dir, 'fusion_calibration_full.yaml')
    with open(temp_config_path, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)
    
    print(f"Using full resolution (skip_rate=1) instead of {original_skip_rate}")
    
    # Find all scan directories
    scan_dirs = []
    for item in os.listdir(scan_dir):
        item_path = os.path.join(scan_dir, item)
        if os.path.isdir(item_path) and item.startswith('fusion_scan_'):
            scan_dirs.append(item_path)
    
    scan_dirs.sort()
    print(f"Found {len(scan_dirs)} scans to process")
    
    first_scan_dir = None
    
    for i, individual_scan_dir in enumerate(scan_dirs, 1):
        scan_name = os.path.basename(individual_scan_dir)
        print(f"\nProcessing scan {i}/{len(scan_dirs)}: {scan_name}")
        
        # Find bag file
        bag_dirs = [d for d in os.listdir(individual_scan_dir) 
                   if d.startswith('rosbag_') and os.path.isdir(os.path.join(individual_scan_dir, d))]
        
        if not bag_dirs:
            print(f"  No bag file found in {scan_name}")
            continue
            
        bag_dir = os.path.join(individual_scan_dir, bag_dirs[0])
        print(f"  Processing bag: {bag_dirs[0]}")
        
        # Extract data from bag
        try:
            # Use rosbag2 to extract point cloud and image data
            result = subprocess.run([
                'python3', '/home/orion/ros2_ws/extract_from_bag.py', 
                bag_dir, individual_scan_dir, temp_config_path
            ], capture_output=True, text=True, timeout=300)
            
            if result.returncode == 0:
                print(f"  ✓ Extracted data from bag")
                
                # Apply fusion coloring
                if i == 1:
                    first_scan_dir = individual_scan_dir
                    subprocess.run([
                        'python3', '/home/orion/ros2_ws/exact_fusion_fix.py', 
                        individual_scan_dir
                    ], timeout=120)
                    print(f"  ✓ Applied fusion coloring (reference scan)")
                else:
                    subprocess.run([
                        'python3', '/home/orion/ros2_ws/world_coord_fusion.py', 
                        individual_scan_dir, first_scan_dir
                    ], timeout=120)
                    print(f"  ✓ Applied fusion coloring (world coordinates)")
                    
            else:
                print(f"  ✗ Failed to extract from bag: {result.stderr}")
                
        except subprocess.TimeoutExpired:
            print(f"  ✗ Timeout processing {scan_name}")
        except Exception as e:
            print(f"  ✗ Error processing {scan_name}: {e}")
    
    # Restore original config
    config['skip_rate'] = original_skip_rate
    with open(config_path, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)
    
    # Clean up temp config
    os.remove(temp_config_path)
    
    print(f"\n✓ Post-processing complete!")
    print(f"Original skip_rate ({original_skip_rate}) restored")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 post_process_bags.py <scan_directory>")
        sys.exit(1)
    
    post_process_bags(sys.argv[1])