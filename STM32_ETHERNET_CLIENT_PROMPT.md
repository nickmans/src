# STM32H755 UDP Client Implementation Prompt (Current Runtime)

This prompt reflects the checked-in runtime contract between CM7 firmware and the Pi UDP bridge.

## Objective

Implement/maintain a non-blocking UDP client on STM32H755 CM7 that:

- Sends POSE to the Pi
- Receives TRAJ from the Pi
- Sends CMD messages from the STM32 shell path
- Never blocks the 100 Hz control loop

## Network defaults

```c
#define PI5_IP_ADDR    "192.168.1.100"
#define PI5_PORT       9000

#define STM32_IP_ADDR  "192.168.1.10"
#define NETMASK_ADDR   "255.255.255.0"
#define GATEWAY_ADDR   "192.168.1.1"
```

## Protocol constants

```c
#define MAGIC       0x4F4D4E49u   // "OMNI"
#define VERSION     1u
#define HEADER_SIZE 24u
```

Message types:

```c
typedef enum {
    MSG_TYPE_POSE = 1,
    MSG_TYPE_TRAJ = 10,
    MSG_TYPE_CMD  = 20,
    MSG_TYPE_ACK  = 12,
    MSG_TYPE_STATUS = 15
} MessageType;
```

## Runtime timing targets

- CM7 control loop: 100 Hz
- POSE heartbeat to Pi: 10 Hz (`posePeriod = 100 ms`)
- Pi TRAJ send default: 10 Hz (`OMNI_TRAJ_SEND_HZ=10`)
- CM7 trajectory deadman timeout: 700 ms

## Message payload formats

### POSE (STM32 -> Pi)

```c
typedef struct __attribute__((packed)) {
    uint32_t pose_t_ms;
    float x;
    float y;
    float yaw;
    float vx;
    float vy;
    float wz;
} PosePayload;
```

### TRAJ (Pi -> STM32)

```c
typedef struct __attribute__((packed)) {
    uint32_t reply_to_pose_seq;
    uint32_t traj_t0_ms;
    uint16_t n_knots;
    uint16_t flags;      // bit0=idle_traj, bit1=has_vel
    uint32_t reserved;
    float dt;
} TrajectoryHeader;

typedef struct __attribute__((packed)) {
    float x;
    float y;
    float yaw;
    float vx;
    float vy;
} TrajectoryKnot;
```

### CMD (STM32 -> Pi)

```c
typedef struct __attribute__((packed)) {
    uint16_t cmd_id;
    uint16_t arg_len;
    // followed by arg bytes
} CommandPayload;
```

Important command IDs used by CM7/Pi:

```c
CMD_STOP_ROS2                  = 0,
CMD_START_TRAJ                 = 1,   // traj 1
CMD_STOP_TRAJ                  = 2,   // traj 0
CMD_START_RESTART_ROS2         = 3,   // traj2 2
CMD_SHUTDOWN_PI5               = 4,
CMD_START_MAPPING              = 5,   // map 1
CMD_FINISH_MAPPING             = 6,   // map 0
CMD_USE_LIVE_MAP               = 7,   // map 2
CMD_USE_FROZEN_MAP             = 8,   // map 3
CMD_START_TRAJ_LOCAL           = 9,   // traj 3
CMD_START_TERMINAL_PASSTHROUGH = 10,
CMD_TERMINAL_PASSTHROUGH_DATA  = 11,
CMD_STOP_TERMINAL_PASSTHROUGH  = 12,
CMD_WP_TEST_PATTERN            = 13   // wp t
```

## Non-blocking behavior requirements

- Use UDP datagrams with non-blocking send/recv.
- Never wait in a way that can stall motor control or estimator timing.
- If transmit would block, drop/retry later.
- Keep command retransmit bounded and non-blocking.

## Runtime semantics to preserve

- Boot in manual mode (`traj_mode = 0`).
- Invalidate cached trajectories before switching into `traj 1` or `traj 3`.
- `traj 0`: manual/standby, no trajectory following.
- `traj 1`: autonomous localization + trajectory follow.
- `traj2 2`: manual STM32 + localization on Pi, trajectory streaming disabled.
- `traj 3`: blank global map mode with local obstacle avoidance.
- `map 1`: start mapping mode.
- `map 0`: finish mapping -> localization.
- `map 2`: use live map mode.
- `map 3`: frozen/localization mode path.
- `wp t`: centered waypoint test pattern request.
- `term`: terminal passthrough mode.

## Validation checklist

- [ ] POSE frames sent at ~10 Hz during runtime
- [ ] TRAJ frames accepted and consumed when trajectory mode is active
- [ ] Deadman timeout (700 ms) triggers safe hold behavior
- [ ] CMD forwarding works for `traj`, `traj2`, `map`, `wp`, `term`, `shutdown`
- [ ] No blocking calls that disturb the 100 Hz control loop
- [ ] Network drop/reconnect is tolerated without hard fault

## Service-side context (Pi)

- Normal production boot path is `omni_udp_server.service`.
- `omni_ros2_stack.service` is debug-only.
- `omni_virtual_stm32.service` is optional simulation/testing only.
