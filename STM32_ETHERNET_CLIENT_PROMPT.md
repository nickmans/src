# STM32 Nucleo H755ZI - Ethernet TCP Client Implementation

## Mission Critical Requirements

**You are implementing a NON-BLOCKING TCP client for the STM32 Nucleo H755ZI that communicates with a Raspberry Pi 5 server. The STM32 is the primary real-time controller and must NEVER block or wait for the Pi5.**

### Core Principles
1. **Deterministic control loops have absolute priority** - Motor control, safety, and USART command reception cannot be interrupted
2. **The robot must function without the Pi5** - Degraded mode operation required
3. **Ethernet is opportunistic** - Connection attempts are non-blocking with exponential backoff
4. **Command flow:** Phone → STM32 (via USART cmd.c) → Pi5 (via Ethernet)

---

## Network Configuration

### Static IP Setup (Direct Ethernet Cable)
```c
// Pi5 Server (already configured and running)
#define PI5_IP_ADDR    "192.168.1.100"
#define PI5_PORT       9000

// STM32 Client (YOU configure this)
#define STM32_IP_ADDR  "192.168.1.10"
#define NETMASK        "255.255.255.0"
#define GATEWAY        "192.168.1.1"  // Not used for direct connection
```

### LwIP Configuration Requirements
In `lwipopts.h`:
```c
#define LWIP_DHCP              0     // Static IP only
#define LWIP_TCP               1
#define LWIP_SOCKET            1     // BSD socket API
#define SO_REUSE               1
#define LWIP_SO_RCVTIMEO       1     // Enable receive timeout
#define LWIP_SO_SNDTIMEO       1     // Enable send timeout
#define LWIP_NONBLOCKING       1     // Non-blocking sockets
```

---

## Connection State Machine

### States
```c
typedef enum {
    TCP_DISCONNECTED,      // Not connected, no attempt in progress
    TCP_CONNECTING,        // Connection attempt in progress
    TCP_CONNECTED,         // Connected and operational
    TCP_ERROR              // Error occurred, need backoff
} TCPState;
```

### Connection Management
- **Check connection every 10 seconds** when disconnected
- Use exponential backoff: Start at 1s delay, max 30s
- Reset backoff on successful connection
- All connection attempts must be non-blocking
- Continue normal robot operation regardless of connection state

### Recommended Implementation
```c
// Connection check task (runs every 10s)
void tcp_connection_task(void) {
    static uint32_t last_check = 0;
    static uint32_t backoff_delay_ms = 1000;
    
    uint32_t now = HAL_GetTick();
    
    switch(tcp_state) {
        case TCP_DISCONNECTED:
            if (now - last_check >= backoff_delay_ms) {
                // Attempt non-blocking connection
                if (tcp_try_connect() == SUCCESS) {
                    tcp_state = TCP_CONNECTED;
                    backoff_delay_ms = 1000;  // Reset backoff
                } else {
                    tcp_state = TCP_ERROR;
                    backoff_delay_ms = MIN(backoff_delay_ms * 2, 30000);
                }
                last_check = now;
            }
            break;
            
        case TCP_CONNECTED:
            // Wait in between checks (idle)
            // Connection maintenance happens in separate receive task
            break;
            
        case TCP_ERROR:
            if (now - last_check >= 10000) {  // Try again in 10s
                tcp_state = TCP_DISCONNECTED;
                last_check = now;
            }
            break;
    }
}
```

---

## Binary Protocol Specification

### Message Frame Structure
```
┌─────────────────────────────────────────────────┐
│ HEADER (24 bytes)   │   PAYLOAD (0-65535 bytes) │
└─────────────────────────────────────────────────┘
```

### Header Format (Little-Endian, ALL fields)
```c
typedef struct __attribute__((packed)) {
    uint32_t magic;         // 0x4F4D4E49 = 'OMNI' in ASCII
    uint16_t version;       // Always 1
    uint16_t msg_type;      // Message type enum
    uint32_t seq;           // Sequence number (increment per message)
    uint32_t t_ms;          // Timestamp (HAL_GetTick())
    uint32_t payload_len;   // Payload size in bytes
    uint32_t crc32;         // CRC32 of payload (use 0 to skip validation)
} MessageHeader;

#define MAGIC 0x4F4D4E49
#define VERSION 1
#define HEADER_SIZE 24
```

### Message Types
```c
typedef enum {
    MSG_TYPE_POSE = 1,    // STM32 → Pi5 (you send)
    MSG_TYPE_TRAJ = 10,   // Pi5 → STM32 (you receive)
    MSG_TYPE_CMD  = 20    // STM32 → Pi5 (you send)
} MessageType;
```

---

## Message Formats

### 1. POSE Message (STM32 → Pi5)
**When:** Send at 5 Hz (every 200ms) when connected  
**Type:** `MSG_TYPE_POSE` (1)  
**Total Size:** 52 bytes (24 header + 28 payload)

```c
typedef struct __attribute__((packed)) {
    uint32_t pose_t_ms;    // Timestamp when pose was measured
    float x;               // X position in meters (odometry)
    float y;               // Y position in meters (odometry)
    float yaw;             // Yaw angle in radians [-π, π]
    float vx;              // X velocity m/s (world frame)
    float vy;              // Y velocity m/s (world frame)
    float wz;              // Angular velocity rad/s
} PosePayload;

// Example: Sending POSE
void send_pose_to_pi5(void) {
    static uint32_t pose_seq = 0;
    
    if (tcp_state != TCP_CONNECTED) return;
    
    // Prepare payload
    PosePayload pose;
    pose.pose_t_ms = HAL_GetTick();
    pose.x = current_state.x;
    pose.y = current_state.y;
    pose.yaw = current_state.yaw;
    pose.vx = current_state.vx;
    pose.vy = current_state.vy;
    pose.wz = current_state.wz;
    
    // Pack message
    uint8_t buffer[HEADER_SIZE + sizeof(PosePayload)];
    pack_message(MSG_TYPE_POSE, pose_seq++, &pose, sizeof(pose), buffer);
    
    // Non-blocking send (do NOT wait if fails)
    tcp_send_nonblocking(buffer, sizeof(buffer));
}
```

### 2. TRAJ Message (Pi5 → STM32)
**When:** Pi5 sends at 5 Hz when trajectory is active  
**Type:** `MSG_TYPE_TRAJ` (10)  
**Total Size:** 56 bytes (24 header + 32 payload)

```c
typedef struct __attribute__((packed)) {
    uint32_t reply_to_pose_seq;  // References your POSE message
    uint32_t traj_t0_ms;         // Trajectory start timestamp
    uint16_t n_knots;            // Number of trajectory knots
    uint16_t flags;              // bit0=idle_traj, bit1=has_velocity
    uint32_t reserved;           // Reserved for future use
    float dt;                    // Time step between knots (seconds)
    // Followed by n_knots * 16 bytes: (x, y, yaw, velocity) as 4 floats each
} TrajectoryHeader;

typedef struct __attribute__((packed)) {
    float x;          // Desired X position (meters)
    float y;          // Desired Y position (meters)
    float yaw;        // Desired yaw (radians)
    float velocity;   // Desired velocity magnitude (m/s)
} TrajectoryKnot;

// Example: Receiving TRAJ
void handle_traj_message(uint8_t *payload, uint32_t payload_len) {
    if (payload_len < sizeof(TrajectoryHeader)) return;
    
    TrajectoryHeader *hdr = (TrajectoryHeader*)payload;
    TrajectoryKnot *knots = (TrajectoryKnot*)(payload + sizeof(TrajectoryHeader));
    
    // Verify payload size
    uint32_t expected_size = sizeof(TrajectoryHeader) + hdr->n_knots * sizeof(TrajectoryKnot);
    if (payload_len < expected_size) return;
    
    // Update trajectory setpoint for controller
    // (Thread-safe copy recommended)
    update_trajectory_reference(knots, hdr->n_knots, hdr->dt, hdr->flags);
}
```

### 3. CMD Message (STM32 → Pi5)
**When:** User-triggered via phone commands received in cmd.c  
**Type:** `MSG_TYPE_CMD` (20)  
**Total Size:** 28 bytes (24 header + 4 payload)

```c
typedef enum {
    CMD_STOP_ROS2  = 0,   // Stop ROS2 stack (unused by phone)
    CMD_START_TRAJ = 1,   // START trajectory generation ← Phone sends this
    CMD_STOP_TRAJ  = 2    // STOP trajectory generation  ← Phone sends this
} CommandID;

typedef struct __attribute__((packed)) {
    uint16_t cmd_id;      // CommandID value
    uint16_t arg_len;     // Length of argument (0 for no arg)
    // Followed by arg_len bytes of argument data (usually empty)
} CommandPayload;

// Example: Sending commands from phone -> STM -> Pi5
// This is triggered when cmd.c receives START/STOP from phone
void forward_phone_command_to_pi5(CommandID cmd) {
    static uint32_t cmd_seq = 0;
    
    if (tcp_state != TCP_CONNECTED) {
        // Pi5 not connected - log warning and continue
        log_warning("Pi5 disconnected, cannot forward command");
        return;
    }
    
    // Pack command payload
    CommandPayload payload;
    payload.cmd_id = cmd;
    payload.arg_len = 0;  // No arguments
    
    // Pack message
    uint8_t buffer[HEADER_SIZE + sizeof(CommandPayload)];
    pack_message(MSG_TYPE_CMD, cmd_seq++, &payload, sizeof(payload), buffer);
    
    // Send to Pi5 (non-blocking)
    tcp_send_nonblocking(buffer, sizeof(buffer));
    
    log_info("Forwarded %s to Pi5", cmd == CMD_START_TRAJ ? "START" : "STOP");
}
```

---

## Integration with cmd.c

### Phone Command Flow
```
[Phone/Joystick] 
      ↓ USART
[cmd.c on STM32] 
      ↓ Parse command
[forward_phone_command_to_pi5()]
      ↓ Ethernet TCP
[Pi5 Server]
      ↓ Starts/stops ROS2 trajectory node
[Pi5 sends TRAJ messages back to STM32]
```

### Expected Integration Points in cmd.c
```c
// When phone sends START command (button press, joystick mode, etc.)
void on_phone_start_command(void) {
    // Your existing robot startup logic here...
    
    // NEW: Notify Pi5 to start trajectory generation
    forward_phone_command_to_pi5(CMD_START_TRAJ);
}

// When phone sends STOP command (button release, safe mode, etc.)
void on_phone_stop_command(void) {
    // Your existing robot stop logic here...
    
    // NEW: Notify Pi5 to stop trajectory generation
    forward_phone_command_to_pi5(CMD_STOP_TRAJ);
}
```

**CRITICAL:** The Pi5 START/STOP commands are:
- **CMD_START_TRAJ = 1** (tells Pi5 to launch trajectory generation)
- **CMD_STOP_TRAJ = 2** (tells Pi5 to stop trajectory generation)

These are the ONLY commands the phone should trigger for Pi5 communication.

---

## Message Packing/Unpacking Helpers

```c
// Pack message with header
void pack_message(uint16_t msg_type, uint32_t seq, void *payload, 
                  uint32_t payload_len, uint8_t *out_buffer) {
    MessageHeader *hdr = (MessageHeader*)out_buffer;
    hdr->magic = MAGIC;
    hdr->version = VERSION;
    hdr->msg_type = msg_type;
    hdr->seq = seq;
    hdr->t_ms = HAL_GetTick();
    hdr->payload_len = payload_len;
    hdr->crc32 = 0;  // Skip CRC validation
    
    // Copy payload after header
    if (payload_len > 0) {
        memcpy(out_buffer + HEADER_SIZE, payload, payload_len);
    }
}

// Validate and parse received message
bool parse_message(uint8_t *buffer, uint32_t len, MessageHeader *out_hdr, 
                   uint8_t **out_payload) {
    if (len < HEADER_SIZE) return false;
    
    MessageHeader *hdr = (MessageHeader*)buffer;
    
    // Validate magic
    if (hdr->magic != MAGIC) return false;
    
    // Validate version
    if (hdr->version != VERSION) return false;
    
    // Validate payload length
    if (HEADER_SIZE + hdr->payload_len > len) return false;
    
    // Output
    *out_hdr = *hdr;
    *out_payload = buffer + HEADER_SIZE;
    
    return true;
}
```

---

## Stream Parser for Receive Buffer

The Pi5 server uses a streaming parser that handles:
- Partial messages across multiple TCP packets
- Resynchronization on corrupted data
- Buffer management

### Recommended STM32 Implementation
```c
#define RX_BUFFER_SIZE 4096

typedef struct {
    uint8_t buffer[RX_BUFFER_SIZE];
    uint32_t write_pos;     // Where to write new data
    uint32_t read_pos;      // Where to read from
    uint32_t bytes_in_buf;  // Total bytes available
} StreamParser;

void parser_init(StreamParser *p) {
    p->write_pos = 0;
    p->read_pos = 0;
    p->bytes_in_buf = 0;
}

void parser_feed(StreamParser *p, uint8_t *data, uint32_t len) {
    // Add received data to circular buffer
    for (uint32_t i = 0; i < len; i++) {
        p->buffer[p->write_pos] = data[i];
        p->write_pos = (p->write_pos + 1) % RX_BUFFER_SIZE;
        p->bytes_in_buf++;
    }
}

bool parser_try_parse(StreamParser *p, MessageHeader *out_hdr, 
                      uint8_t *out_payload, uint32_t payload_capacity) {
    // Need at least header
    if (p->bytes_in_buf < HEADER_SIZE) return false;
    
    // Peek at header
    uint8_t hdr_buf[HEADER_SIZE];
    uint32_t pos = p->read_pos;
    for (int i = 0; i < HEADER_SIZE; i++) {
        hdr_buf[i] = p->buffer[pos];
        pos = (pos + 1) % RX_BUFFER_SIZE;
    }
    
    MessageHeader *hdr = (MessageHeader*)hdr_buf;
    
    // Validate magic (if invalid, search for next magic)
    if (hdr->magic != MAGIC) {
        // Discard one byte and try again
        p->read_pos = (p->read_pos + 1) % RX_BUFFER_SIZE;
        p->bytes_in_buf--;
        return false;
    }
    
    // Check if full message available
    uint32_t total_size = HEADER_SIZE + hdr->payload_len;
    if (p->bytes_in_buf < total_size) return false;
    
    // Extract payload
    pos = (p->read_pos + HEADER_SIZE) % RX_BUFFER_SIZE;
    for (uint32_t i = 0; i < hdr->payload_len && i < payload_capacity; i++) {
        out_payload[i] = p->buffer[pos];
        pos = (pos + 1) % RX_BUFFER_SIZE;
    }
    
    // Consume message from buffer
    p->read_pos = (p->read_pos + total_size) % RX_BUFFER_SIZE;
    p->bytes_in_buf -= total_size;
    
    *out_hdr = *hdr;
    return true;
}
```

---

## Non-Blocking Socket Operations

### Setup Non-Blocking Socket
```c
int tcp_setup_nonblocking_socket(void) {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return -1;
    
    // Set non-blocking mode
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    
    // Set timeouts (100ms max)
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // 100ms
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    
    return sock;
}
```

### Non-Blocking Connect
```c
bool tcp_try_connect(void) {
    if (tcp_socket < 0) {
        tcp_socket = tcp_setup_nonblocking_socket();
        if (tcp_socket < 0) return false;
    }
    
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PI5_PORT);
    inet_pton(AF_INET, PI5_IP_ADDR, &server_addr.sin_addr);
    
    int result = connect(tcp_socket, (struct sockaddr*)&server_addr, 
                        sizeof(server_addr));
    
    if (result == 0 || (result < 0 && errno == EISCONN)) {
        // Connected successfully
        return true;
    }
    
    if (result < 0 && errno == EINPROGRESS) {
        // Connection in progress - check later
        // For simplicity, wait briefly with select()
        fd_set write_fds;
        FD_ZERO(&write_fds);
        FD_SET(tcp_socket, &write_fds);
        
        struct timeval tv = {0, 50000};  // 50ms timeout
        if (select(tcp_socket + 1, NULL, &write_fds, NULL, &tv) > 0) {
            int error = 0;
            socklen_t len = sizeof(error);
            getsockopt(tcp_socket, SOL_SOCKET, SO_ERROR, &error, &len);
            
            if (error == 0) return true;  // Connected!
        }
    }
    
    // Connection failed
    close(tcp_socket);
    tcp_socket = -1;
    return false;
}
```

### Non-Blocking Send
```c
void tcp_send_nonblocking(uint8_t *data, uint32_t len) {
    if (tcp_socket < 0) return;
    
    int sent = send(tcp_socket, data, len, MSG_DONTWAIT);
    
    if (sent < 0) {
        if (errno == EWOULDBLOCK || errno == EAGAIN) {
            // Buffer full - drop message (prefer fresh data)
            log_warning("TCP send buffer full, dropping message");
        } else {
            // Connection error
            close(tcp_socket);
            tcp_socket = -1;
            tcp_state = TCP_DISCONNECTED;
            log_error("TCP send error, disconnected");
        }
    }
}
```

### Non-Blocking Receive
```c
void tcp_receive_nonblocking(StreamParser *parser) {
    if (tcp_socket < 0) return;
    
    uint8_t temp_buf[512];
    int received = recv(tcp_socket, temp_buf, sizeof(temp_buf), MSG_DONTWAIT);
    
    if (received > 0) {
        // Feed to parser
        parser_feed(parser, temp_buf, received);
        
        // Try to parse messages
        MessageHeader hdr;
        uint8_t payload[2048];
        
        while (parser_try_parse(parser, &hdr, payload, sizeof(payload))) {
            handle_received_message(&hdr, payload);
        }
        
    } else if (received == 0) {
        // Connection closed
        close(tcp_socket);
        tcp_socket = -1;
        tcp_state = TCP_DISCONNECTED;
        log_info("Pi5 disconnected");
        
    } else if (errno != EWOULDBLOCK && errno != EAGAIN) {
        // Error
        close(tcp_socket);
        tcp_socket = -1;
        tcp_state = TCP_DISCONNECTED;
        log_error("TCP receive error");
    }
}
```

---

## Main Task Structure

### Recommended Task Loop
```c
void ethernet_task(void *params) {
    StreamParser parser;
    parser_init(&parser);
    
    uint32_t last_pose_send = 0;
    uint32_t last_conn_check = 0;
    
    while (1) {
        uint32_t now = HAL_GetTick();
        
        // 1. Connection management (every 10s when disconnected)
        tcp_connection_task();
        
        // 2. Send POSE at 5 Hz (200ms interval) if connected
        if (tcp_state == TCP_CONNECTED && now - last_pose_send >= 200) {
            send_pose_to_pi5();
            last_pose_send = now;
        }
        
        // 3. Receive and process messages (non-blocking)
        if (tcp_state == TCP_CONNECTED) {
            tcp_receive_nonblocking(&parser);
        }
        
        // 4. Small delay to prevent CPU thrashing (DO NOT block control loop)
        osDelay(10);  // 10ms sleep if using RTOS
    }
}
```

---

## Testing & Verification

### Expected Pi5 Server Logs (Success)
```
[INFO] Server listening on 0.0.0.0:9000 (all interfaces - accessible from 192.168.1.10)
[INFO] Waiting for STM32 to connect from 192.168.1.10...
[INFO] Client connected from ('192.168.1.10', XXXXX)
[DEBUG] Received message: type=1 (POSE), seq=0, len=28
[INFO] CMD seq=0 cmd_id=1 (START_TRAJ)
[DEBUG] Queued TRAJ for POSE seq=1, 24 knots
```

### Common Issues
1. **Connection timeouts** - Check static IP, cables, firewall
2. **Magic number mismatch** - Ensure little-endian byte order
3. **Parser stuck** - Check buffer overflow, add resync on bad magic
4. **Control loop blocking** - Profile task execution time (<1ms target)

---

## Summary Checklist

✅ Configure STM32 static IP: 192.168.1.10  
✅ Configure Pi5 server IP: 192.168.1.100:9000  
✅ Implement non-blocking socket operations (fcntl O_NONBLOCK)  
✅ Connection check every 10s with exponential backoff  
✅ Send POSE at 5 Hz when connected  
✅ Receive and parse TRAJ messages (stream parser)  
✅ Forward phone START/STOP commands to Pi5 (CMD messages)  
✅ Handle disconnection gracefully (continue robot operation)  
✅ Verify control loop is never blocked  
✅ Test with Pi5 powered off - robot should still function  

**The Pi5 expects:**
- **CMD_START_TRAJ = 1** (from phone button press)
- **CMD_STOP_TRAJ = 2** (from phone button release/stop)

Good luck! The STM32 should run autonomously and treat Pi5 as an optional enhancement.
