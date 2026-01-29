# Dual LIDAR Fusion - Implementation Fixes

## Overview
This document outlines the fixes applied to the dual LIDAR fusion system for proper scan fusion and RViz visualization.

## Issues Fixed

### 1. **RViz Frame Visibility (odom not visible)**
**Problem**: The odom frame was not visible in RViz because there was no `world -> odom` transform published.

**Solution**: Added a static transform publisher that creates the `world -> odom` link:
```bash
static_transform_publisher 0.0 0.0 0.0 0 0 0 world odom
```
This enables RViz to display the odom frame when set as the fixed frame.

**Location**: [dual_sllidar_with_mock_and_traj.launch.py](launch/dual_sllidar_with_mock_and_traj.launch.py)

### 2. **LIDAR Positioning in Transform Tree**
**Problem**: Static transforms for the two LIDARs were correctly positioned at y=+0.10m and y=-0.10m (facing forward on x-axis).

**Status**: Verified correct - both LIDARs are positioned:
- **Lidar1**: base_link → lidar1_link: `(0.0, +0.10, 0.10)` 
- **Lidar2**: base_link → lidar2_link: `(0.0, -0.10, 0.10)`

### 3. **RViz Display Configuration**
**Problem**: The RViz config didn't include the fused scan display, making it invisible.

**Solution**: Added a new LaserScan display for `/scan_fused` with:
- **Color**: Green (0; 255; 0) for easy distinction from raw scans
- **Topic**: `/scan_fused`
- **Frame**: odom (via fixed frame transform)
- **Size**: 4 pixels for visibility

**Location**: [sllidar_ros2.rviz](../../sllidar_ros2/rviz/sllidar_ros2.rviz)

## Transform Tree Structure

```
world
  └─ odom (static, identity)
      └─ base_link (published dynamically by waypoint_traj_node)
          ├─ lidar1_link (static, at y=+0.10m, z=0.10m)
          └─ lidar2_link (static, at y=-0.10m, z=0.10m)
```

## Data Flow

1. **Raw Scans**: 
   - `/lidar1/scan` → in frame `lidar1_link`
   - `/lidar2/scan` → in frame `lidar2_link`

2. **Scan Fusion** (waypoint_traj_node):
   - Transforms both scans from their respective frames to `base_link`
   - Fuses them into a single scan at the robot origin
   - Publishes fused scan as `/scan_fused` in `base_link` frame

3. **Costmap Building**:
   - Builds costmap from the fused scan (not individual scans)
   - Applies hard/soft inflation
   - Clears robot exclusion radius around base_link
   - Publishes as `/costmap` in `odom` frame

4. **RViz Visualization** (with fixed frame = odom):
   - Individual raw scans show in their respective positions (y=±0.10m)
   - Fused scan shows centered at base_link (green points)
   - Costmap shows in odom frame

## Configuration Parameters

### Launch File: `dual_sllidar_with_mock_and_traj.launch.py`

Key parameters in the `traj` node:

```python
"map_frame": "odom",                    # Frame for costmap and planning
"base_frame": "base_link",              # Robot body frame
"publish_odom_to_base_tf": True,        # Publish odom→base_link dynamically
"publish_fused_scan": True,             # Enable /scan_fused publication
"fused_angle_increment_deg": 1.0,       # 1-degree bins (360 beams)
"hard_inflate_radius": 0.22,            # Robot inflation radius
"robot_exclusion_enable": True,         # Clear footprint in costmap
"robot_exclusion_radius_m": 0.22,       # Footprint circle radius
```

## Testing & Usage

### 1. Start with Mock Lidars (no hardware required)
```bash
cd /home/nickolas/ros2_ws
source install/setup.bash
ros2 launch omni_traj dual_sllidar_with_mock_and_traj.launch.py use_mock_lidar:=true use_rviz:=true
```

### 2. In RViz:
1. Set **Fixed Frame** to `odom`
2. Verify displays show:
   - **Grid**: XY plane reference
   - **Map**: Static costmap (empty initially)
   - **Fused Scan**: Green points at origin (from fused /scan_fused)
   - **LaserScan (lidar1)**: White/intensity points at y=+0.10m
   - **LaserScan (lidar2)**: White/intensity points at y=-0.10m
   - **Costmap**: Occupancy grid built from fused scan

### 3. Verify Transforms
```bash
ros2 run tf2_tools view_frames.py
```
Should show complete tree: world → odom → base_link → lidar1/2_link

## Key Improvements

1. ✅ **Complete TF tree** with world anchor
2. ✅ **Fused scan properly centered** at origin (base_link)
3. ✅ **Costmap built from single fused scan** (not double-counted)
4. ✅ **RViz visualization** with dedicated fused scan display
5. ✅ **Proper frame hierarchy** for motion planning

## Troubleshooting

### Fused scan not visible in RViz
- ✓ Check `/scan_fused` topic is publishing: `ros2 topic hz /scan_fused`
- ✓ Ensure "Fused Scan" display is enabled
- ✓ Verify fixed frame is set to `odom`

### Costmap looks distorted
- ✓ Ensure only fused scan is used (robot_exclusion working)
- ✓ Check inflation radius values are reasonable
- ✓ Verify base_link pose is being published: `ros2 topic echo /tf`

### Transforms have discontinuities
- ✓ Check odometry source (if publishing external /odom)
- ✓ Verify `publish_odom_to_base_tf: True` in launch file
- ✓ Check for clock issues: `ros2 topic hz /clock`

## Files Modified

1. **[launch/dual_sllidar_with_mock_and_traj.launch.py](launch/dual_sllidar_with_mock_and_traj.launch.py)**
   - Added `world_to_odom` static transform publisher
   - Added comments clarifying LIDAR positioning

2. **[../../sllidar_ros2/rviz/sllidar_ros2.rviz](../../sllidar_ros2/rviz/sllidar_ros2.rviz)**
   - Added fused scan display with green color
   - Configured for proper visualization

3. **[omni_traj/waypoint_traj_node.py](omni_traj/waypoint_traj_node.py)**
   - No changes needed (already correctly implemented)
   - Properly transforms scans to base_link
   - Builds costmap from fused scan

## Next Steps (Optional Enhancements)

1. **Dynamic LIDAR Calibration**: Adjust y-offsets if actual spacing differs
2. **Motion Compensation**: Set `motion_compensate: True` if robot moves during scan collection
3. **Adaptive Inflation**: Vary inflation radius based on scan reliability
4. **Custom Planning**: Add goal reaching behavior in waypoint_traj_node
