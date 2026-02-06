# OMNI TCP Server Architecture

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                          STM32 Nucleo H755ZI                        │
│                          (192.168.1.10)                             │
│                                                                     │
│  ┌──────────────┐         ┌──────────────┐                         │
│  │ Motion       │         │   Sensors    │                         │
│  │ Control      │         │   (Encoders) │                         │
│  └──────┬───────┘         └──────┬───────┘                         │
│         │                        │                                 │
│         │                        v                                 │
│         │                ┌────────────────┐                        │
│         └───────────────►│  TCP Client    │                        │
│                          │  (Sends POSE)  │                        │
│                          │  (Recvs TRAJ)  │                        │
│                          │  (Sends CMD)   │                        │
│                          └────────┬───────┘                        │
└──────────────────────────────────┼─────────────────────────────────┘
                                   │
                                   │ TCP/IP
                                   │ Port 9000
                                   │
┌──────────────────────────────────┼─────────────────────────────────┐
│                                  v                                  │
│                   Raspberry Pi 5 (192.168.1.100)                   │
│                                                                     │
│  ┌──────────────────────────────────────────────────────┐          │
│  │              TCP Server (tcp_server.py)              │          │
│  │                                                      │          │
│  │  ┌──────────────┐  ┌──────────────┐  ┌───────────┐ │          │
│  │  │   Accept     │  │   Receive    │  │   Send    │ │          │
│  │  │   Thread     │  │   Thread     │  │   Thread  │ │          │
│  │  │              │  │              │  │           │ │          │
│  │  │ - Listen     │  │ - Parse POSE │  │ - Send    │ │          │
│  │  │ - Accept     │  │ - Parse CMD  │  │   TRAJ    │ │          │
│  │  │ - Reconnect  │  │ - Store pose │  │   @ 5 Hz  │ │          │
│  │  └──────────────┘  └──────┬───────┘  └─────▲─────┘ │          │
│  │                           │                │       │          │
│  └───────────────────────────┼────────────────┼───────┘          │
│                              │                │                   │
│                    ┌─────────▼────────┐       │                   │
│                    │  Pose Storage    │       │                   │
│                    │  (Thread-safe)   │       │                   │
│                    └─────────┬────────┘       │                   │
│                              │                │                   │
│  ┌───────────────────────────┼────────────────┼─────────────────┐ │
│  │         Main Integration (omni_main.py)    │                 │ │
│  │                           │                │                 │ │
│  │  ┌────────────────────────▼────┐    ┌──────┴──────────────┐ │ │
│  │  │  on_pose_callback()         │    │ get_trajectory()     │ │ │
│  │  │  - Publishes to ROS2        │    │ - Gets setpoint      │ │ │
│  │  └────────────┬────────────────┘    └──────▲──────────────┘ │ │
│  │               │                            │                │ │
│  │  ┌────────────▼────────────┐               │                │ │
│  │  │  on_cmd_callback()      │               │                │ │
│  │  │  - CMD=1: Start traj    │───────────────┤                │ │
│  │  │  - CMD=2: Stop traj     │               │                │ │
│  │  └─────────────────────────┘               │                │ │
│  └────────────────────────────────────────────┼────────────────┘ │
│                                               │                   │
│  ┌────────────────────────────────────────────┼────────────────┐ │
│  │                ROS2 Layer                  │                │ │
│  │                                            │                │ │
│  │  ┌────────────────────────┐   ┌────────────▼─────────────┐ │ │
│  │  │  Pose Publisher Node   │   │ Trajectory Generator     │ │ │
│  │  │  (ros2_pose_node.py)   │   │ (ros2_trajectory_node.py)│ │ │
│  │  │                        │   │                          │ │ │
│  │  │  Publishes:            │   │  Modes:                  │ │ │
│  │  │  - /robot/pose         │   │  - hold                  │ │ │
│  │  │  - /robot/twist        │   │  - waypoint              │ │ │
│  │  │  - /robot/odom         │   │  - circle                │ │ │
│  │  │  - /initialpose        │   │                          │ │ │
│  │  │                        │   │  Publishes:              │ │ │
│  │  │                        │   │  - /robot/trajectory     │ │ │
│  │  └────────────────────────┘   └──────────────────────────┘ │ │
│  │                                                            │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                   │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │                    ROS2 Topics                              │  │
│  │  /robot/pose → PoseStamped                                  │  │
│  │  /robot/twist → TwistStamped                                │  │
│  │  /robot/odom → Odometry                                     │  │
│  │  /robot/trajectory → Path                                   │  │
│  │  /initialpose → PoseWithCovarianceStamped                   │  │
│  └────────────────────────────────────────────────────────────┘  │
│                                                                   │
└───────────────────────────────────────────────────────────────────┘
```

## Message Flow

### POSE Message Flow (5 Hz)
```
STM32 → TCP → Receive Thread → Pose Storage → ROS2 Pose Node → /robot/pose
                                                               → /robot/odom
```

### TRAJ Message Flow (5 Hz when active)
```
Trajectory Node → get_trajectory() → Send Thread → TCP → STM32
```

### CMD Message Flow
```
STM32 → TCP → Receive Thread → CMD Handler → Start/Stop Trajectory Node
```

## Threading Model

```
Main Thread
├── ROS2 Executor Thread
│   ├── Pose Publisher Node (spin)
│   └── Trajectory Generator Node (spin, when active)
│
└── TCP Server
    ├── Accept Thread
    │   └── Waits for client connections
    │
    ├── Receive Thread (per client)
    │   ├── Receives data
    │   ├── Parses messages
    │   └── Calls callbacks
    │
    └── Send Thread (per client)
        ├── Checks if trajectory active
        ├── Gets trajectory setpoint
        └── Sends TRAJ messages @ 5 Hz
```

## Data Flow Diagram

```
┌──────────┐
│  STM32   │
└────┬─────┘
     │
     │ POSE (5 Hz)
     │ ┌──────────────────────────────────────┐
     │ │ Header: magic, version, type, seq... │
     │ │ Payload: t_ms, x, y, yaw, vx, vy, wz │
     │ └──────────────────────────────────────┘
     │
     ▼
┌────────────────┐
│  TCP Server    │
│  (Receive)     │
└────┬───────────┘
     │
     │ Parse & Store
     │
     ▼
┌────────────────┐     ┌─────────────────┐
│  Pose Storage  │────►│ ROS2 Publisher  │
│  (Latest)      │     └────────┬────────┘
└────────────────┘              │
                                │
                                ▼
                         /robot/pose
                         /robot/odom


┌──────────┐
│  STM32   │
└────┬─────┘
     │
     │ CMD
     │ ┌────────────────────┐
     │ │ command=1 (START)  │
     │ └────────────────────┘
     │
     ▼
┌────────────────┐
│  TCP Server    │
│  (Receive)     │
└────┬───────────┘
     │
     │ Parse CMD
     │
     ▼
┌────────────────┐
│  CMD Handler   │
└────┬───────────┘
     │
     │ Start/Stop
     │
     ▼
┌────────────────────┐
│ Trajectory Node    │
│ (if CMD=1)         │
└────┬───────────────┘
     │
     │ Generate setpoint
     │
     ▼
┌────────────────┐
│  TCP Server    │
│  (Send)        │
└────┬───────────┘
     │
     │ TRAJ (5 Hz)
     │ ┌────────────────────────────────────┐
     │ │ Header: magic, version, type...    │
     │ │ Payload: x_des, y_des, yaw_des,... │
     │ └────────────────────────────────────┘
     │
     ▼
┌──────────┐
│  STM32   │
└──────────┘
```

## Protocol State Machine

```
┌─────────────┐
│  CONNECTED  │
└──────┬──────┘
       │
       │ Receive POSE @ 5 Hz
       ├──────────────────────►
       │
       │ Receive CMD=1 (START_TRAJ)
       ├──────────────────────►
       │
       ▼
┌──────────────────┐
│ TRAJECTORY_ACTIVE │
└──────┬───────────┘
       │
       │ Send TRAJ @ 5 Hz
       ├──────────────────────►
       │
       │ Continue receiving POSE
       ├──────────────────────►
       │
       │ Receive CMD=2 (STOP_TRAJ)
       ├──────────────────────►
       │
       ▼
┌─────────────┐
│  CONNECTED  │
└─────────────┘
```

## Key Components

### Protocol Layer
- **protocol.py**: Message definitions, pack/unpack, parser

### Network Layer
- **tcp_server.py**: Socket handling, threading, connection management

### Integration Layer
- **omni_main.py**: Coordinates TCP ↔ ROS2

### ROS2 Layer
- **ros2_pose_node.py**: Publishes pose data
- **ros2_trajectory_node.py**: Generates trajectories

### Testing Layer
- **test_stm32_client.py**: Simulates STM32 behavior
