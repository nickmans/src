# STM32 Nucleo H755ZI - TCP Client Implementation Prompt

## Overview
Implement a **non-blocking** TCP client for the STM32 Nucleo H755ZI (dual-core Cortex-M7/M4) that communicates with a Raspberry Pi 5 server over Ethernet. The STM32 is the **main controller node** and must never block waiting for the Pi5 connection.

---

## CRITICAL REQUIREMENTS ⚠️

### 1. **NON-BLOCKING OPERATION**
- The STM32 CANNOT block waiting for a Pi5 connection
- The controller loop, USART command reception, and motor control must run continuously
- TCP operations must be asynchronous/polled with immediate timeout
- Connection attempts should use exponential backoff to avoid CPU thrashing
- If Pi5 is disconnected, operations continue normally with default/safe behavior

### 2. **PRIORITY HIERARCHY**
1. **Primary:** Motor control, safety monitoring, USART command reception
2. **Secondary:** State estimation, odometry updates
3. **Tertiary:** TCP communication with Pi5 (trajectory reception)

### 3. **ARCHITECTURE**
- Use **LwIP** with non-blocking sockets (`fcntl` or `NETCONN_NONBLOCKING`)
- Recommended: Run TCP client on **CM4 core** (secondary) if using dual-core
- Use inter-core communication (shared memory/message queue) to pass data to CM7
- Alternatively: Run on CM7 with strict non-blocking guarantees

---

## NETWORK CONFIGURATION

### Static IP Setup (Direct Ethernet Connection)
```c
// Pi5 Configuration (already set)
#define PI5_IP_ADDR    "192.168.1.100"
#define PI5_PORT       9000

// STM32 Configuration
#define STM32_IP_ADDR  "192.168.1.10"
#define NETMASK        "255.255.255.0"
#define GATEWAY        "192.168.1.1"  // Not needed for direct connection
```

### LwIP Configuration
Set in `lwipopts.h`:
```c
#define LWIP_DHCP              0     // Use static IP
#define LWIP_TCP               1
#define LWIP_SOCKET            1     // Enable BSD socket API
#define SO_REUSE               1
#define LWIP_SO_RCVTIMEO       1     // Enable receive timeout
#define LWIP_SO_SNDTIMEO       1     // Enable send timeout
```

---

## BINARY PROTOCOL SPECIFICATION

### Message Structure
All messages consist of a **24-byte header** + **variable-length payload**:

```
[HEADER: 24 bytes] [PAYLOAD: 0-65535 bytes]
```

### Header Format (Little-Endian)
```c
typedef struct {
    uint32_t magic;         // 0x4F4D4E49 ('OMNI' in hex)
    uint16_t version;       // Protocol version = 1
    uint16_t msg_type;      // Message type (see below)
    uint32_t seq;           // Sequence number
    uint32_t t_ms;          // Timestamp in milliseconds
    uint32_t payload_len;   // Payload length in bytes
    uint32_t crc32;         // CRC32 of payload (0 = skip validation)
} __attribute__((packed)) MessageHeader;

#define MAGIC 0x4F4D4E49
#define VERSION 1
#define HEADER_SIZE 24
```

### Message Types
```c
typedef enum {
    MSG_TYPE_POSE = 1,    // STM32 → Pi5: Robot pose/odometry
    MSG_TYPE_TRAJ = 10,   // Pi5 → STM32: Trajectory setpoint
    MSG_TYPE_CMD  = 20    // STM32 → Pi5: Commands
} MessageType;
```

---

## MESSAGE FORMATS

### 1. POSE Message (STM32 → Pi5)
**Rate:** 5 Hz (every 200ms)  
**Type:** `MSG_TYPE_POSE` (1)  
**Payload:** 28 bytes

```c
typedef struct {
    uint32_t pose_t_ms;    // Timestamp of pose measurement
    float x;               // X position (meters, odom frame)
    float y;               // Y position (meters, odom frame)
    float yaw;             // Yaw angle (radians, odom->base_link)
    float vx;              // X velocity (m/s, base_link/body frame)
    float vy;              // Y velocity (m/s, base_link/body frame)
    float wz;              // Angular velocity (rad/s, base_link/body frame)
} __attribute__((packed)) PosePayload;
```

**Example sending:**
```c
void send_pose(int socket, uint32_t *seq) {
    // Pack payload
    PosePayload pose;
    pose.pose_t_ms = HAL_GetTick();
    pose.x = current_x;
    pose.y = current_y;
    pose.yaw = current_yaw;
    pose.vx = current_vx;
    pose.vy = current_vy;
    pose.wz = current_wz;
    
    // Create message
    uint8_t buffer[HEADER_SIZE + sizeof(PosePayload)];
    create_message(MSG_TYPE_POSE, (*seq)++, (uint8_t*)&pose, 
                   sizeof(PosePayload), buffer, false);
    
    // Non-blocking send
    send(socket, buffer, sizeof(buffer), MSG_DONTWAIT);
}
```

### 2. TRAJ Message (Pi5 → STM32)
**Rate:** 5 Hz (when trajectory active)  
**Type:** `MSG_TYPE_TRAJ` (10)  
**Payload:** 20 bytes

```c
typedef struct {
    float x_des;       // Desired X position (meters)
    float y_des;       // Desired Y position (meters)
    float yaw_des;     // Desired yaw angle (radians)
    float vx_world;    // Desired X velocity (m/s, world/odom frame)
    float vy_world;    // Desired Y velocity (m/s, world/odom frame)
} __attribute__((packed)) TrajectoryPayload;
```

**Example receiving:**
```c
// In non-blocking receive loop
void process_traj_message(uint8_t *payload) {
    TrajectoryPayload traj;
    memcpy(&traj, payload, sizeof(TrajectoryPayload));
    
    // Update trajectory setpoint (thread-safe)
    update_trajectory_setpoint(&traj);
}
```

### 3. CMD Message (STM32 → Pi5)
**Rate:** On event (user command)  
**Type:** `MSG_TYPE_CMD` (20)  
**Payload:** 4 bytes

```c
typedef enum {
    CMD_START_TRAJ = 1,  // Start trajectory generation
    CMD_STOP_TRAJ  = 2   // Stop trajectory generation
} CommandID;

typedef struct {
    uint32_t command;  // CommandID value
} __attribute__((packed)) CommandPayload;
```

**Example sending:**
```c
void send_command(int socket, uint32_t *seq, CommandID cmd) {
    CommandPayload payload;
    payload.command = cmd;
    
    uint8_t buffer[HEADER_SIZE + sizeof(CommandPayload)];
    create_message(MSG_TYPE_CMD, (*seq)++, (uint8_t*)&payload,
                   sizeof(CommandPayload), buffer, false);
    
    send(socket, buffer, sizeof(buffer), MSG_DONTWAIT);
}
```

---

## IMPLEMENTATION STRUCTURE

### Main TCP Client State Machine
```c
typedef enum {
    TCP_DISCONNECTED,
    TCP_CONNECTING,
    TCP_CONNECTED,
    TCP_ERROR
} TCPState;

typedef struct {
    int socket_fd;
    TCPState state;
    uint32_t pose_seq;
    uint32_t cmd_seq;
    uint32_t last_send_ms;
    uint32_t reconnect_delay_ms;
    uint8_t rx_buffer[1024];
    size_t rx_buffer_pos;
    TrajectoryPayload latest_traj;
    bool traj_valid;
} TCPClient;

// Global instance
TCPClient tcp_client;
```

### Non-Blocking Connection
```c
void tcp_client_connect(TCPClient *client) {
    if (client->state == TCP_CONNECTING || client->state == TCP_CONNECTED)
        return;
    
    // Create non-blocking socket
    client->socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (client->socket_fd < 0) {
        client->state = TCP_ERROR;
        return;
    }
    
    // Set non-blocking
    int flags = fcntl(client->socket_fd, F_GETFL, 0);
    fcntl(client->socket_fd, F_SETFL, flags | O_NONBLOCK);
    
    // Set timeouts
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;  // 100ms
    setsockopt(client->socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(client->socket_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    
    // Attempt connection
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PI5_PORT);
    inet_pton(AF_INET, PI5_IP_ADDR, &server_addr.sin_addr);
    
    int result = connect(client->socket_fd, (struct sockaddr*)&server_addr, 
                        sizeof(server_addr));
    if (result == 0) {
        client->state = TCP_CONNECTED;
        client->reconnect_delay_ms = 1000;  // Reset backoff
    } else if (errno == EINPROGRESS) {
        client->state = TCP_CONNECTING;
    } else {
        close(client->socket_fd);
        client->state = TCP_DISCONNECTED;
    }
}
```

### Non-Blocking Poll Loop (Call from main loop or RTOS task)
```c
void tcp_client_poll(TCPClient *client) {
    uint32_t now_ms = HAL_GetTick();
    
    switch (client->state) {
        case TCP_DISCONNECTED:
            // Attempt reconnect with exponential backoff
            if (now_ms - client->last_send_ms > client->reconnect_delay_ms) {
                tcp_client_connect(client);
                client->last_send_ms = now_ms;
                // Exponential backoff: 1s, 1.5s, 2.25s, ... up to 10s
                client->reconnect_delay_ms = MIN(client->reconnect_delay_ms * 3 / 2, 10000);
            }
            break;
            
        case TCP_CONNECTING:
            // Check if connected (non-blocking)
            fd_set write_fds;
            FD_ZERO(&write_fds);
            FD_SET(client->socket_fd, &write_fds);
            struct timeval tv = {0, 0};  // Immediate return
            
            if (select(client->socket_fd + 1, NULL, &write_fds, NULL, &tv) > 0) {
                int error = 0;
                socklen_t len = sizeof(error);
                getsockopt(client->socket_fd, SOL_SOCKET, SO_ERROR, &error, &len);
                if (error == 0) {
                    client->state = TCP_CONNECTED;
                } else {
                    close(client->socket_fd);
                    client->state = TCP_DISCONNECTED;
                }
            }
            break;
            
        case TCP_CONNECTED:
            // Send POSE at 5 Hz
            if (now_ms - client->last_send_ms >= 200) {
                send_pose(client->socket_fd, &client->pose_seq);
                client->last_send_ms = now_ms;
            }
            
            // Non-blocking receive
            tcp_client_receive(client);
            break;
            
        case TCP_ERROR:
            close(client->socket_fd);
            client->state = TCP_DISCONNECTED;
            break;
    }
}
```

### Non-Blocking Receive with Stream Parser
```c
void tcp_client_receive(TCPClient *client) {
    uint8_t temp_buf[512];
    
    // Non-blocking recv
    int received = recv(client->socket_fd, temp_buf, sizeof(temp_buf), MSG_DONTWAIT);
    
    if (received > 0) {
        // Append to buffer
        size_t available = sizeof(client->rx_buffer) - client->rx_buffer_pos;
        size_t to_copy = MIN(received, available);
        memcpy(client->rx_buffer + client->rx_buffer_pos, temp_buf, to_copy);
        client->rx_buffer_pos += to_copy;
        
        // Parse messages
        parse_messages(client);
    } else if (received == 0) {
        // Connection closed
        close(client->socket_fd);
        client->state = TCP_DISCONNECTED;
    } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
        // Error
        close(client->socket_fd);
        client->state = TCP_DISCONNECTED;
    }
    // EAGAIN/EWOULDBLOCK = no data available, continue
}

void parse_messages(TCPClient *client) {
    while (client->rx_buffer_pos >= HEADER_SIZE) {
        MessageHeader *header = (MessageHeader*)client->rx_buffer;
        
        // Validate magic
        if (header->magic != MAGIC) {
            // Resync: search for magic
            // ... (implement magic search and shift buffer)
            return;
        }
        
        // Check if full message available
        uint32_t total_size = HEADER_SIZE + header->payload_len;
        if (client->rx_buffer_pos < total_size)
            return;  // Wait for more data
        
        // Parse message
        uint8_t *payload = client->rx_buffer + HEADER_SIZE;
        
        if (header->msg_type == MSG_TYPE_TRAJ) {
            memcpy(&client->latest_traj, payload, sizeof(TrajectoryPayload));
            client->traj_valid = true;
        }
        
        // Consume message from buffer
        memmove(client->rx_buffer, client->rx_buffer + total_size, 
                client->rx_buffer_pos - total_size);
        client->rx_buffer_pos -= total_size;
    }
}
```

---

## HELPER FUNCTIONS

### Create Message
```c
void create_message(uint16_t msg_type, uint32_t seq, uint8_t *payload, 
                   uint32_t payload_len, uint8_t *out_buffer, bool use_crc) {
    MessageHeader header;
    header.magic = MAGIC;
    header.version = VERSION;
    header.msg_type = msg_type;
    header.seq = seq;
    header.t_ms = HAL_GetTick();
    header.payload_len = payload_len;
    header.crc32 = 0;  // CRC disabled for efficiency
    
    if (use_crc) {
        header.crc32 = crc32_compute(payload, payload_len);
    }
    
    memcpy(out_buffer, &header, HEADER_SIZE);
    memcpy(out_buffer + HEADER_SIZE, payload, payload_len);
}
```

---

## INTEGRATION WITH CONTROLLER

### Controller Loop (CM7 Core)
```c
void controller_loop(void) {
    while (1) {
        // 1. Read sensors (CRITICAL)
        read_encoders();
        read_imu();
        
        // 2. Update state estimation (HIGH PRIORITY)
        update_odometry();
        
        // 3. Get trajectory setpoint (non-blocking)
        TrajectoryPayload traj_setpoint;
        if (tcp_client.traj_valid) {
            traj_setpoint = tcp_client.latest_traj;
        } else {
            // Default: hold position
            traj_setpoint.x_des = current_x;
            traj_setpoint.y_des = current_y;
            traj_setpoint.yaw_des = current_yaw;
            traj_setpoint.vx_world = 0.0f;
            traj_setpoint.vy_world = 0.0f;
        }
        
        // 4. Run controller
        compute_motor_commands(&traj_setpoint);
        
        // 5. Send motor commands
        send_motor_pwm();
        
        // 6. Poll TCP client (non-blocking, quick)
        tcp_client_poll(&tcp_client);
        
        osDelay(10);  // 100 Hz loop
    }
}
```

---

## TESTING CHECKLIST

1. **No Pi5 connected:** STM32 boots, controller runs normally, attempts reconnection with backoff
2. **Pi5 disconnects mid-operation:** STM32 continues with default trajectory, no crashes
3. **Pi5 reconnects:** STM32 reconnects and resumes trajectory following
4. **High network latency:** Controller remains responsive, no blocking
5. **USART commands:** Always processed immediately regardless of TCP state

---

## SUMMARY

**Implement a non-blocking TCP client that:**
- Uses non-blocking sockets with immediate timeouts
- Polls connection state without blocking
- Sends POSE at 5 Hz when connected
- Receives TRAJ at 5 Hz when active
- Degrades gracefully when Pi5 is unavailable
- Never interferes with primary control loops

**Key principle:** The STM32 is the master; the Pi5 is an optional enhancement for advanced trajectory planning.
