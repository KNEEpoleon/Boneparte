#!/usr/bin/env python3
"""
Test script to simulate ROS sending a segmented image to AVP
Run this on the same machine as the AVP simulator/device to see logs
"""

import socket
import base64
import cv2
import numpy as np
import time

def create_test_segmented_image():
    """Create a test segmented image with colored overlays"""
    # Create a 640x480 test image
    image = np.ones((480, 640, 3), dtype=np.uint8) * 50  # Dark gray background
    
    # Add some colored regions to simulate segmentation
    # Femur region (blue)
    cv2.rectangle(image, (100, 100), (300, 350), (30, 144, 255), -1)
    cv2.putText(image, "FEMUR", (150, 225), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    # Tibia region (orange)
    cv2.rectangle(image, (350, 150), (550, 400), (255, 144, 30), -1)
    cv2.putText(image, "TIBIA", (400, 275), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    # Add some transparency effect by blending
    overlay = image.copy()
    alpha = 0.6
    image = cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0)
    
    return image

def send_segmented_image(host='127.0.0.1', port=5000):
    """Send a test segmented image to AVP"""
    print(f"Connecting to {host}:{port}...")
    
    try:
        # Create socket and connect
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.connect((host, port))
        print("Connected!")
        
        # Wait for initial connection acknowledgment
        time.sleep(0.5)
        
        # Create test segmented image
        print("Creating test segmented image...")
        segmented_image = create_test_segmented_image()
        
        # Convert to PNG and encode as base64
        print("Encoding image to PNG and base64...")
        _, buffer = cv2.imencode('.png', segmented_image)
        image_base64 = base64.b64encode(buffer).decode('utf-8')
        print(f"Base64 encoding complete, size: {len(image_base64)} chars")
        
        # Send segmented image with proper prefix
        message = f"SEGMENTED_IMAGE:{image_base64}\n"
        print(f"Sending message, total size: {len(message)} chars")
        print("Sending...")
        
        sock.sendall(message.encode('utf-8'))
        print("Segmented image sent successfully!")
        
        # Wait a bit for response
        print("\nWaiting for AVP response (accept/reject)...")
        sock.settimeout(30.0)  # 30 second timeout
        
        try:
            response = sock.recv(1024).decode('utf-8').strip()
            print(f"Received response: {response}")
        except socket.timeout:
            print("No response received (timeout)")
        
        # Keep connection open for a bit
        time.sleep(2)
        
    except ConnectionRefusedError:
        print("Connection refused. Is the AVP app running and listening on port 5000?")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        sock.close()
        print("\nConnection closed.")

def send_annotation_image_first(host='127.0.0.1', port=5000):
    """Send an annotation image first to simulate the full workflow"""
    print(f"STEP 1: Sending annotation image")
    print(f"Connecting to {host}:{port}...")
    
    try:
        # Create socket and connect
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.connect((host, port))
        print("Connected!")
        
        # Wait for initial connection
        time.sleep(0.5)
        
        # Create test annotation image (original image)
        print("Creating test annotation image...")
        annotation_image = np.ones((480, 640, 3), dtype=np.uint8) * 100
        cv2.putText(annotation_image, "Original Image for Annotation", 
                   (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Encode and send
        _, buffer = cv2.imencode('.png', annotation_image)
        image_base64 = base64.b64encode(buffer).decode('utf-8')
        message = f"IMAGE:{image_base64}\n"
        print(f"Sending annotation image ({len(message)} chars)...")
        sock.sendall(message.encode('utf-8'))
        print("Annotation image sent!")
        
        # Wait for annotations response
        print("\nWaiting for annotations from AVP...")
        sock.settimeout(60.0)
        
        try:
            response = sock.recv(4096).decode('utf-8').strip()
            print(f"Received: {response}")
            
            if "ANNOTATIONS:" in response:
                print("\nGot annotations! Now sending segmented image...")
                time.sleep(1)
                
                # Send segmented image
                segmented_image = create_test_segmented_image()
                _, buffer = cv2.imencode('.png', segmented_image)
                image_base64 = base64.b64encode(buffer).decode('utf-8')
                message = f"SEGMENTED_IMAGE:{image_base64}\n"
                print(f"Sending segmented image ({len(message)} chars)...")
                sock.sendall(message.encode('utf-8'))
                print("Segmented image sent!")
                
                # Wait for accept/reject
                print("\nWaiting for accept/reject from AVP...")
                response = sock.recv(1024).decode('utf-8').strip()
                print(f"Received: {response}")
                
        except socket.timeout:
            print("Timeout waiting for response")
        
        time.sleep(2)
        sock.close()
        print("\nTest complete!")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    import sys
    
    print("=" * 60)
    print("Segmented Image Test Script")
    print("=" * 60)
    
    # Get host from command line or use localhost
    host = sys.argv[1] if len(sys.argv) > 1 else '127.0.0.1'
    
    print("\nOptions:")
    print("1. Send only segmented image (test segmentation display)")
    print("2. Full workflow (annotation → segmentation)")
    
    choice = input("\nEnter choice (1 or 2): ").strip()
    
    if choice == "1":
        send_segmented_image(host)
    elif choice == "2":
        send_annotation_image_first(host)
    else:
        print("Invalid choice. Use 1 or 2")
        
    print("\nTest complete!")

