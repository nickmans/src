#!/usr/bin/env python3
"""
Simple test server to verify network connectivity for STM32.
Listens on port 9000 and accepts any TCP connection.
"""

import socket
import sys

HOST = "0.0.0.0"  # Listen on all interfaces
PORT = 9000

print("=" * 60)
print("STM32 Connection Test Server")
print("=" * 60)
print(f"Binding to {HOST}:{PORT}")
print()

try:
    # Create socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    
    print(f"✓ Server listening on port {PORT}")
    print(f"  Accessible from 192.168.1.10")
    print()
    print("Waiting for STM32 connection...")
    print("(Press Ctrl+C to stop)")
    print()
    
    while True:
        # Accept connection
        client_socket, client_address = server_socket.accept()
        print(f"✓ CONNECTION from {client_address}")
        
        # Receive and print data
        try:
            while True:
                data = client_socket.recv(1024)
                if not data:
                    print(f"  Client {client_address} disconnected")
                    break
                print(f"  Received {len(data)} bytes: {data.hex()}")
                
                # Echo back
                client_socket.sendall(data)
                print(f"  Echoed {len(data)} bytes back")
                
        except Exception as e:
            print(f"  Error: {e}")
        finally:
            client_socket.close()
            print(f"  Connection to {client_address} closed")
            print()
            print("Waiting for next connection...")
            print()
            
except KeyboardInterrupt:
    print("\nShutting down...")
except Exception as e:
    print(f"ERROR: {e}")
    sys.exit(1)
finally:
    if 'server_socket' in locals():
        server_socket.close()
    print("Server stopped")
