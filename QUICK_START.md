# Dual LIDAR Fusion - Quick Start Guide

## 🚀 One-Command Startup

```bash
# Build the workspace
cd ~/ros2_ws && colcon build

# Run with mock LIDARs (no hardware needed)
source install/setup.bash
ros2 launch omni_traj dual_sllidar_with_mock_and_traj.launch.py use_mock_lidar:=true use_rviz:=true
```

## 📊 What You Should See in RViz

| Component | Color | Location | Meaning |
|-----------|-------|----------|---------|
| **Fused Scan** | 🟢 Green | Center (0,0) | Merged lidar data at robot origin |
| **Costmap** | Gray | Entire grid | Navigation obstacles from fused scan |
| **Raw Scan 1** | ⚪ White | Y = +0.1m | LIDAR 1 raw data (right side) |
| **Raw Scan 2** | ⚪ White | Y = -0.1m | LIDAR 2 raw data (left side) |
| **Grid** | Light gray | Background | Reference grid (1m spacing) |

## ✅ Verification Checklist

```bash
# 1. Check if topics exist
ros2 topic list | grep -E "scan|costmap"

# 2. Check transform tree
ros2 run tf2_tools view_frames.py  # Creates frames.pdf

# 3. Run verification script
python3 ~/ros2_ws/src/omni_src/verify_lidar_fusion.py

# 4. Monitor fused scan rate
ros2 topic hz /scan_fused  # Should show ~10 Hz

# 5. Check costmap updates
ros2 topic hz /costmap  # Should show ~5 Hz
```

## 🔧 Configuration Changes

### LIDAR Positions
Edit `dual_sllidar_with_mock_and_traj.launch.py`:
- **Lidar1 position**: Change `"0.0", "0.10", "0.10"` (y, z offsets)
- **Lidar2 position**: Change `"0.0", "-0.10", "0.10"` (y, z offsets)

### Fused Scan Resolution
Edit `dual_sllidar_with_mock_and_traj.launch.py`:
- **Angle resolution**: `"fused_angle_increment_deg": 1.0`
  - 1.0° = 360 beams (current)
  - 0.5° = 720 beams (more detail, slower)
  - 2.0° = 180 beams (faster, less detail)

### Costmap Size/Resolution
Edit `dual_sllidar_with_mock_and_traj.launch.py`:
- **Grid resolution**: `"global_map_res": 0.02` (cm per pixel)
- **Map width**: `"global_map_width_m": 6.0` (meters)
- **Map height**: `"global_map_height_m": 6.0` (meters)

### Robot Footprint
Edit `dual_sllidar_with_mock_and_traj.launch.py`:
- **Inflation radius**: `"hard_inflate_radius": 0.22` (meters)
- **Exclusion radius**: `"robot_exclusion_radius_m": 0.22` (meters)

## 🎮 RViz Setup

### Recommended View
1. **Fixed Frame**: `odom`
2. **View Type**: `Orbit (rviz)`
3. **Camera Distance**: ~5-8 meters
4. **Camera Pitch**: ~60° (looking down at angle)

### Recommended Displays (all enabled)
- ✓ Grid (1m spacing)
- ✓ Fused Scan (green dots)
- ✓ Costmap (gray with obstacles)
- ✓ LaserScan (lidar1) - white dots right side
- ✓ LaserScan (lidar2) - white dots left side

### Save Custom RViz Config
```bash
# After configuring RViz, save it:
# File → Save Config As → my_lidar_fusion.rviz
# Then use it:
ros2 launch omni_traj dual_sllidar_with_mock_and_traj.launch.py \
  use_mock_lidar:=true use_rviz:=true
# (Edit launch file to use your config path)
```

## 🧪 Testing Scenarios

### Test 1: Verify Fusion Works
```bash
# Open 2 terminals in ros2_ws:

# Terminal 1: Launch system
source install/setup.bash
ros2 launch omni_traj dual_sllidar_with_mock_and_traj.launch.py \
  use_mock_lidar:=true use_rviz:=true

# Terminal 2: Monitor fused scan
source install/setup.bash
ros2 echo /scan_fused | head -20
# Should see: angle_min, angle_max, ranges array
```

### Test 2: Check Costmap Quality
```bash
# Terminal 2: Get costmap info
ros2 service call /costmap_pub/get_map nav2_msgs/srv/GetCostmap
# Should return valid grid with origin at (0, 0)
```

### Test 3: Verify Transforms
```bash
# Terminal 2: Check specific transform
ros2 run tf2_ros tf2_echo odom base_link
# Should show transform updating at ~5 Hz
# Position should move if robot moving

ros2 run tf2_ros tf2_echo base_link lidar1_link
# Should show static transform: (0, 0.1, 0.1)
```

## 📡 Real Hardware Setup

When using real LIDARs instead of mock:

```bash
# 1. Find USB devices
ls -la /dev/ttyUSB*
# Should show /dev/ttyUSB0 and /dev/ttyUSB1

# 2. Or use by-id paths (more reliable)
ls -la /dev/serial/by-id/USB_*

# 3. Update launch file with correct ports:
ros2 launch omni_traj dual_sllidar_with_mock_and_traj.launch.py \
  use_mock_lidar:=false \
  use_rviz:=true \
  lidar1_serial_port:=/dev/ttyUSB0 \
  lidar2_serial_port:=/dev/ttyUSB1

# 4. If permission denied, add user to dialout:
sudo usermod -a -G dialout $USER
# (then log out and back in)
```

## 🐛 Common Issues & Solutions

### "No transforms received"
```bash
# Check if tf is being published
ros2 topic hz /tf
# If empty, check launch file - world_to_odom should be present
```

### "Fused scan empty/inf values"
```bash
# Check individual scans first
ros2 echo /lidar1/scan | head
ros2 echo /lidar2/scan | head
# If they show valid ranges, issue is in fusion code
```

### "Costmap shows nothing"
```bash
# Check if fused scan is valid
ros2 topic hz /scan_fused
# Check costmap frame
ros2 topic echo /costmap | head -10
# frame_id should be "odom"
```

### "High CPU usage"
- Reduce `fused_angle_increment_deg` (fewer rays)
- Reduce `global_map_width_m` or `global_map_height_m`
- Increase `scan_beam_stride` to skip beams

## 📚 Documentation Files

Created for reference:
- `LIDAR_FUSION_FIXES.md` - Detailed technical fixes
- `LIDAR_FUSION_ARCHITECTURE.md` - System architecture & data flow
- `verify_lidar_fusion.py` - Automated verification script

## 🎯 Next Steps

1. ✅ Run mock LIDAR test (use commands above)
2. ✅ Verify all displays visible in RViz
3. ✅ Test with real hardware (update ports in launch)
4. ✅ Adjust parameters for your environment
5. 🔄 Iterate on inflation & costmap resolution

## 💡 Pro Tips

- **Motion compensation**: If robot moves during scan, set `motion_compensate: true`
- **Dual scan debugging**: Disable one LIDAR at a time to verify each
- **Slow motion**: For tuning, move robot slowly (<0.5 m/s)
- **Better fusion**: Increase `fused_angle_increment_deg` precision if CPU allows

## 📞 Support

Check these for issues:
1. ROS2 topic frequency: `ros2 topic hz [topic]`
2. Transform delays: `ros2 run tf2_ros tf2_echo [frame1] [frame2]`
3. Message contents: `ros2 echo [topic]`
4. Node logs: Check terminal output for ERROR/WARN messages
