#!/usr/bin/env python3
"""
Quick verification script for dual LIDAR fusion setup.
Run: python3 verify_lidar_fusion.py
"""

import subprocess
import time
import sys

def run_command(cmd, desc=""):
    """Run a shell command and return output."""
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=5
        )
        if desc:
            print(f"\n{'='*60}")
            print(f"✓ {desc}")
            print(f"{'='*60}")
            if result.stdout:
                print(result.stdout[:500])
            if result.stderr and "WARN" not in result.stderr:
                print("STDERR:", result.stderr[:500])
        return result.returncode == 0
    except Exception as e:
        print(f"✗ Error: {desc}")
        print(f"  {e}")
        return False

def check_topics():
    """Check if topics are being published."""
    print("\nChecking topic publication...")
    topics_to_check = [
        ("/lidar1/scan", "LIDAR 1 raw scan"),
        ("/lidar2/scan", "LIDAR 2 raw scan"),
        ("/scan_fused", "Fused scan (should be in base_link)"),
        ("/costmap", "Costmap for planning"),
        ("/tf", "Transform tree"),
    ]
    
    all_ok = True
    for topic, desc in topics_to_check:
        cmd = f"ros2 topic list | grep -q '^{topic}$'"
        try:
            result = subprocess.run(cmd, shell=True, timeout=2)
            status = "✓" if result.returncode == 0 else "✗"
            print(f"  {status} {topic:20} - {desc}")
            all_ok = all_ok and (result.returncode == 0)
        except:
            print(f"  ✗ {topic:20} - {desc} (timeout)")
            all_ok = False
    return all_ok

def check_transforms():
    """Check if transform tree is valid."""
    print("\nChecking transform tree...")
    frames_to_check = [
        ("world", "Root frame"),
        ("odom", "Odometry frame"),
        ("base_link", "Robot frame"),
        ("lidar1_link", "LIDAR 1 frame"),
        ("lidar2_link", "LIDAR 2 frame"),
    ]
    
    all_ok = True
    for frame, desc in frames_to_check:
        cmd = f"ros2 frame list | grep -q '^{frame}$'"
        try:
            result = subprocess.run(cmd, shell=True, timeout=2)
            status = "✓" if result.returncode == 0 else "✗"
            print(f"  {status} {frame:15} - {desc}")
            all_ok = all_ok and (result.returncode == 0)
        except:
            print(f"  ✗ {frame:15} - {desc} (timeout)")
            all_ok = False
    return all_ok

def check_node():
    """Check if waypoint_traj node is running."""
    print("\nChecking node status...")
    cmd = "ros2 node list | grep -q 'waypoint_traj'"
    try:
        result = subprocess.run(cmd, shell=True, timeout=2)
        status = "✓" if result.returncode == 0 else "✗"
        print(f"  {status} waypoint_traj node {'is running' if result.returncode == 0 else 'is NOT running'}")
        return result.returncode == 0
    except:
        print(f"  ✗ waypoint_traj node (timeout)")
        return False

def main():
    print("\n" + "="*60)
    print("DUAL LIDAR FUSION - VERIFICATION SCRIPT")
    print("="*60)
    
    print("\nNote: Run this after launching with:")
    print("  ros2 launch omni_traj dual_sllidar_with_mock_and_traj.launch.py \\")
    print("    use_mock_lidar:=true use_rviz:=true")
    
    time.sleep(2)
    
    # Perform checks
    results = {
        "Topics": check_topics(),
        "Transforms": check_transforms(),
        "Node": check_node(),
    }
    
    # Summary
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    all_ok = all(results.values())
    for check, status in results.items():
        symbol = "✓" if status else "✗"
        print(f"  {symbol} {check}")
    
    print("\n" + "="*60)
    if all_ok:
        print("✓ All checks passed! System is ready.")
        print("\nIn RViz:")
        print("  1. Set Fixed Frame to 'odom'")
        print("  2. Look for green points (fused scan at origin)")
        print("  3. Look for costmap (light gray grid)")
        print("  4. Raw scans should appear offset (y=±0.10m)")
    else:
        print("✗ Some checks failed. See above for details.")
        sys.exit(1)
    print("="*60)

if __name__ == "__main__":
    main()
