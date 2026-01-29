# Dual LIDAR Fusion - Visual Architecture

## System Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    DUAL LIDAR SYSTEM                        │
└─────────────────────────────────────────────────────────────┘

HARDWARE LAYOUT (Top View):
═════════════════════════════════════════════════════════════

                     Forward (X+)
                          ▲
                          │
    LIDAR 1              │              LIDAR 2
    (y=+0.1m)            │              (y=-0.1m)
         ◯ ─────────  BASE_LINK  ─────────  ◯
                          ●
                      Origin (0,0)


TRANSFORM TREE:
═════════════════════════════════════════════════════════════

    world (map anchor)
         │
         └─── [static, identity] ───→ odom
                                       │
                                       ├─── [dynamic] ───→ base_link
                                       │                      │
                                       │                      ├─ [static] → lidar1_link
                                       │                      │   (x:0, y:+0.1, z:0.1)
                                       │                      │
                                       │                      └─ [static] → lidar2_link
                                       │                          (x:0, y:-0.1, z:0.1)
                                       │
                                       └─── [optional] ───→ other_frames


RViz VISUALIZATION (fixed frame = odom):
═════════════════════════════════════════════════════════════

        N
        │
    ────┼──── E
    W   │
        │
        S

Global View:
─────────────────────────────────────────────────────────────

    odom                (origin for costmap)
      ●
      │
      ├─── grid (light gray) ────────────────┐
      │                                       │
      ├─── raw lidar1 scan (white dots)       │
      │    └─ positioned at y=+0.1m          │
      │                                       │
      ├─── raw lidar2 scan (white dots)       │
      │    └─ positioned at y=-0.1m          │
      │                                       │
      ├─── fused scan (GREEN dots) ◆◆◆◆◆    │  Costmap shows
      │    └─ centered at origin              │  obstacles from
      │                                       │  fused scan only
      └─── costmap (light gray with obstacles)│
                                              │


DATA FLOW:
═════════════════════════════════════════════════════════════

    Hardware Lidars (or Emulators)
            │
            ├─→ /lidar1/scan (frame: lidar1_link)
            │        │
            │        └─ [TF: lidar1_link ← base_link]
            │           [TF: base_link ← odom]
            │           [TF: odom ← world]
            │
            └─→ /lidar2/scan (frame: lidar2_link)
                     │
                     └─ [TF: lidar2_link ← base_link]
                        [TF: base_link ← odom]
                        [TF: odom ← world]
    
    ┌─────────────────────────────────────────┐
    │    WaypointTrajNode (Fusion Logic)      │
    │  ─────────────────────────────────────  │
    │  1. Read both raw scans                 │
    │  2. Transform to base_link              │
    │  3. Merge into single scan              │
    │  4. Apply robot exclusion radius        │
    │  5. Publish /scan_fused in base_link    │
    │                                         │
    │  6. Build costmap from fused scan       │
    │  7. Apply hard/soft inflation           │
    │  8. Clear robot footprint               │
    │  9. Publish /costmap in odom            │
    └─────────────────────────────────────────┘
            │
            ├─→ /scan_fused (frame: base_link)
            │   └─ Used for RViz visualization
            │      and path planning
            │
            └─→ /costmap (frame: odom)
                └─ Used for A* path planning
                   and navigation


PARAMETER FLOW:
═════════════════════════════════════════════════════════════

Launch Arguments:
  use_mock_lidar (bool)  → Use empty scans or real hardware
  use_rviz (bool)        → Start RViz visualization
  channel_type           → serial (default)
  serial_baudrate        → 460800 (default)
  scan_mode              → Standard (default)
  
Node Parameters:
  map_frame              → "odom" (for costmap)
  base_frame             → "base_link" (robot center)
  publish_odom_to_base_tf → true (publish dynamic transform)
  
  Scan Fusion:
    lidar1_topic         → "/lidar1/scan"
    lidar2_topic         → "/lidar2/scan"
    publish_fused_scan   → true
    fused_angle_increment_deg → 1.0 (360 beams)
    motion_compensate    → false (set true if robot moves)
    
  Costmap Building:
    global_map_res       → 0.02 m/pixel
    global_map_width_m   → 6.0 m
    global_map_height_m  → 6.0 m
    hard_inflate_radius  → 0.22 m (robot size)
    soft_inflate_radius  → 0.0 m (optional uncertainty)
    
  Robot Footprint:
    robot_exclusion_enable → true
    robot_exclusion_radius_m → 0.22 m


TYPICAL WORKFLOW:
═════════════════════════════════════════════════════════════

    Terminal 1:
    $ ros2 launch omni_traj dual_sllidar_with_mock_and_traj.launch.py \
        use_mock_lidar:=true use_rviz:=true
    
    Terminal 2 (verify):
    $ python3 verify_lidar_fusion.py
    
    RViz:
    1. Fixed Frame = "odom"
    2. See "Fused Scan" display in GREEN
    3. See "Costmap" display as gray grid
    4. See individual scans offset by ±0.1m in Y


TROUBLESHOOTING VISUALLY:
═════════════════════════════════════════════════════════════

Problem: Fused scan not visible
─────────────────────────────────
  ✗ No green dots at all
  → Check: /scan_fused topic (should publish at ~10Hz)
  → Check: RViz "Fused Scan" display is enabled
  → Check: Fixed frame is "odom"
  → Check: scan_max_age_s not exceeded

Problem: Fused scan off-center
─────────────────────────────────
  ✗ Green dots offset from (0,0)
  → Check: TF base_link ← odom is correct
  → Check: Static TF base_link ← lidar1/2 are correct
  → Check: Odometry source (/odom) if available

Problem: Costmap looks doubled/overlapped
─────────────────────────────────────────
  ✗ Costmap shows obstacle pattern twice
  → Check: robot_exclusion_enable = true
  → Check: Only fused scan used (not raw scans)
  → Check: Inflation not too large

Problem: Raw scans not offset
──────────────────────────────
  ✗ LIDAR 1 and 2 scans overlap at center
  → Check: Static TF for lidar1_link/lidar2_link Y values
  → Check: TF lookup_transform working (check logs)

Problem: Costmap empty
──────────────────────
  ✗ All white (zero occupancy)
  → Check: Fused scan has valid ranges
  → Check: scan_max_age_s not too small
  → Check: Motion compensation correct if robot moving
