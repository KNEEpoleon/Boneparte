#!/usr/bin/env python3
"""
Simple mock AVP annotation sender - bypasses IMAGE transfer
Only sends annotations directly to test the annotation handling
"""

import socket
import json
import sys
import time

def send_annotations_only(host='localhost', port=5000):
    """Send annotations without going through the full annotate flow"""
    
    # Mock annotation points (normalized coordinates 0.0 to 1.0)
    annotations = [
        {"x": 0.45, "y": 0.40},  # Femur point
        {"x": 0.55, "y": 0.60}   # Tibia point
    ]
    
    try:
        print(f"Connecting to {host}:{port}...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((host, port))
        print("✓ Connected!")
        
        time.sleep(0.2)
        
        # Send ANNOTATIONS directly (simulating AVP response)
        annotations_json = json.dumps(annotations)
        message = f"ANNOTATIONS:{annotations_json}\n"
        
        print(f"\n📤 Sending: {message.strip()}")
        sock.sendall(message.encode('utf-8'))
        print("✓ Sent!")
        
        time.sleep(0.5)
        
        # Try to receive acknowledgment
        try:
            sock.settimeout(2.0)
            response = sock.recv(1024)
            print(f"📥 Server response: {response.decode('utf-8').strip()}")
        except socket.timeout:
            print("⏱ No response (timeout)")
        
        sock.close()
        print("\n✓ Done! Check ROS logs for processing.")
        
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    host = sys.argv[1] if len(sys.argv) > 1 else 'localhost'
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 5000
    
    print("=" * 60)
    print("Quick Mock AVP - Direct Annotation Sender")
    print("=" * 60)
    print(f"Target: {host}:{port}\n")
    
    send_annotations_only(host, port)

