import socket
import time
import struct

# Server configuration
SERVER_IP = "127.0.0.1"
SERVER_PORT = 9000

# Message type and constants
MAGIC = 0x4F4D4E49  # 'OMNI'
VERSION = 1
HEADER_SIZE = 24

class MessageType:
    POSE = 1
    CMD = 20

class Pose:
    @staticmethod
    def pack(pose_t_ms, x, y, yaw, vx, vy, wz):
        return struct.pack("<Iffffff", pose_t_ms, x, y, yaw, vx, vy, wz)

def create_message(msg_type, seq, payload):
    payload_len = len(payload)
    header = struct.pack(
        "<IHHIIII",
        MAGIC,  # magic
        VERSION,  # version
        msg_type,  # message type
        seq,  # sequence number
        int(time.time() * 1000) % (2**32),  # timestamp in ms
        payload_len,  # payload length
        0,  # CRC (not used)
    )
    return header + payload

def main():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    seq = 0
    try:
        while True:
            # Create a POSE message
            pose_t_ms = int(time.time() * 1000) % (2**32)
            x, y, yaw = 1.0, 2.0, 0.5  # Example pose
            vx, vy, wz = 0.1, 0.2, 0.05  # Example velocities
            payload = Pose.pack(pose_t_ms, x, y, yaw, vx, vy, wz)
            message = create_message(MessageType.POSE, seq, payload)

            # Send the message to the server
            client_socket.sendto(message, (SERVER_IP, SERVER_PORT))
            print(f"Sent POSE message: seq={seq}, x={x}, y={y}, yaw={yaw}, vx={vx}, vy={vy}, wz={wz}")

            # Add functionality to send a CMD message with traj 1
            # Send a CMD message to start trajectory generation
            cmd_payload = struct.pack("<I", 1)  # Command ID for traj 1
            cmd_message = create_message(MessageType.CMD, seq, cmd_payload)
            client_socket.sendto(cmd_message, (SERVER_IP, SERVER_PORT))
            print(f"Sent CMD message: traj 1")

            seq += 1
            time.sleep(0.2)  # Send at 0.2s intervals

    except KeyboardInterrupt:
        print("Client stopped.")
    finally:
        client_socket.close()

if __name__ == "__main__":
    main()