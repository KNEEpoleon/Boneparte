#!/usr/bin/env python3
"""
Test SERVER to simulate ROS receiving annotations and sending segmented image to AVP
Run this on your Mac, then connect AVP app to it
"""

import socket
import base64
import cv2
import numpy as np
import time
import json

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

def create_test_annotation_image():
    """Create a test original image for annotation"""
    image = np.ones((480, 640, 3), dtype=np.uint8) * 100
    cv2.putText(image, "Test Image - Annotate Here", 
               (100, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.circle(image, (320, 240), 100, (200, 200, 200), 2)
    return image

def run_server(host='0.0.0.0', port=5000):
    """Run test server that simulates ROS TCP server"""
    print("=" * 60)
    print("Segmented Image Test Server")
    print("=" * 60)
    print(f"\nStarting server on {host}:{port}")
    print("Waiting for AVP connection...")
    print("\nInstructions:")
    print("1. Make sure AVP app can reach this IP")
    print("2. Connect AVP app")
    print("3. Send 'annotate' command OR click Annotate button")
    print("4. Annotate 2 points")
    print("5. Server will send segmented image back")
    print("=" * 60)
    
    # Create server socket
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    server_sock.bind((host, port))
    server_sock.listen(1)
    
    print(f"\n✅ Server listening on {host}:{port}")
    print("Waiting for connection...")
    
    try:
        while True:
            # Accept connection
            client_sock, client_addr = server_sock.accept()
            print(f"\n✅ Connected to AVP at {client_addr}")
            client_sock.setblocking(False)
            
            receive_buffer = ""
            
            try:
                while True:
                    # Non-blocking receive
                    try:
                        data = client_sock.recv(4096)
                        if not data:
                            print("Client disconnected")
                            break
                        
                        receive_buffer += data.decode('utf-8')
                        
                        # Process complete messages (ending with \n)
                        while '\n' in receive_buffer:
                            newline_idx = receive_buffer.index('\n')
                            message = receive_buffer[:newline_idx].strip()
                            receive_buffer = receive_buffer[newline_idx + 1:]
                            
                            if message:
                                print(f"\n📨 Received: {message[:100]}...")
                                
                                # Handle different commands
                                if message == "annotate":
                                    print("📤 Sending annotation image...")
                                    img = create_test_annotation_image()
                                    _, buffer = cv2.imencode('.png', img)
                                    image_base64 = base64.b64encode(buffer).decode('utf-8')
                                    response = f"IMAGE:{image_base64}\n"
                                    client_sock.sendall(response.encode('utf-8'))
                                    print(f"✅ Annotation image sent ({len(response)} chars)")
                                    
                                elif message.startswith("ANNOTATIONS:"):
                                    print("📥 Received annotations!")
                                    json_str = message[12:]
                                    annotations = json.loads(json_str)
                                    print(f"   Annotations: {annotations}")
                                    
                                    # Send acknowledgment
                                    client_sock.sendall(b'acknowledged\n')
                                    print("✅ Sent acknowledgment")
                                    
                                    # Wait a moment, then send segmented image
                                    time.sleep(0.5)
                                    print("\n📤 Sending SEGMENTED image...")
                                    img = create_test_segmented_image()
                                    _, buffer = cv2.imencode('.png', img)
                                    image_base64 = base64.b64encode(buffer).decode('utf-8')
                                    response = f"SEGMENTED_IMAGE:{image_base64}\n"
                                    print(f"   Sending {len(response)} chars...")
                                    client_sock.sendall(response.encode('utf-8'))
                                    print("✅ Segmented image sent! Check AVP for popup.")
                                    
                                elif message == "accept":
                                    print("✅ AVP ACCEPTED segmentation!")
                                    client_sock.sendall(b'acknowledged\n')
                                    
                                elif message == "reject":
                                    print("❌ AVP REJECTED segmentation")
                                    client_sock.sendall(b'acknowledged\n')
                                    
                                else:
                                    print(f"   Other command: {message}")
                                    client_sock.sendall(b'acknowledged\n')
                    
                    except BlockingIOError:
                        # No data available, sleep briefly
                        time.sleep(0.01)
                        continue
                    
            except Exception as e:
                print(f"Error: {e}")
            finally:
                client_sock.close()
                print("\nConnection closed. Waiting for new connection...")
                
    except KeyboardInterrupt:
        print("\n\n🛑 Server stopped by user")
    finally:
        server_sock.close()
        print("Server shut down.")

if __name__ == "__main__":
    import sys
    
    # Get host/port from command line or use defaults
    host = sys.argv[1] if len(sys.argv) > 1 else '0.0.0.0'
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 5000
    
    run_server(host, port)

