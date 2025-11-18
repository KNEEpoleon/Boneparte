#!/usr/bin/env python3
"""
Simple TCP server to test AVP connection - sends fake drill poses
Run with: python3 test_avp_server.py
"""

import socket
import time

# Server configuration
HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 5001  # Different from ROS2 server (5000)

# Fake drill pose data (5 drill sites in aruco frame)
# Format: POSES|x,y,z,qx,qy,qz,qw|x,y,z,qx,qy,qz,qw|...
FAKE_POSES = "POSES|0.1,0.0,0.2,0,0,0,1|0.2,0.0,0.2,0,0,0,1|0.3,0.0,0.2,0,0,0,1|0.1,0.1,0.2,0,0,0,1|0.2,0.1,0.2,0,0,0,1\n"

def main():
    # Create socket
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((HOST, PORT))
    server_sock.listen(1)
    
    print(f"TCP Server listening on {HOST}:{PORT}")
    print(f"Connect your AVP to this Mac's IP address")
    
    # Get Mac's IP address
    import subprocess
    result = subprocess.run(['ipconfig', 'getifaddr', 'en0'], capture_output=True, text=True)
    if result.returncode == 0:
        mac_ip = result.stdout.strip()
        print(f"Mac IP: {mac_ip}")
        print(f"Update AVP app to connect to: {mac_ip}:{PORT}")
    
    print("\nWaiting for AVP connection...\n")
    
    try:
        while True:
            # Accept connection
            client_sock, addr = server_sock.accept()
            print(f"AVP connected from {addr}")
            
            try:
                # Send fake drill poses repeatedly
                while True:
                    client_sock.sendall(FAKE_POSES.encode('utf-8'))
                    print(f"Sent 5 fake drill poses to AVP")
                    time.sleep(0.1)  # Send at ~10 Hz
                    
            except (BrokenPipeError, ConnectionResetError):
                print(f"AVP disconnected")
                client_sock.close()
                print("\nWaiting for new connection...\n")
                
    except KeyboardInterrupt:
        print("\n\nServer stopped")
        server_sock.close()

if __name__ == '__main__':
    main()

