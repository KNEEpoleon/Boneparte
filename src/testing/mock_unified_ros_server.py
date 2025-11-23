#!/usr/bin/env python3
"""
Comprehensive Mock ROS Server for Testing AVP App
Simulates all 3 TCP ports with realistic data

Port 5000: Control commands + FSM state updates
Port 5001: Drill poses stream (10Hz)
Port 5002: Images (annotation/segmentation)
"""

import socket
import threading
import time
import json
import base64
import sys
from datetime import datetime
import random
from PIL import Image, ImageDraw, ImageFont
import io


class MockUnifiedROSServer:
    def __init__(self, host='0.0.0.0'):
        self.host = host
        self.running = True
        
        # Port 5000: Control Channel
        self.control_server = None
        self.control_client = None
        self.control_client_addr = None
        
        # Port 5001: Drill Poses Channel
        self.poses_server = None
        self.poses_client = None
        self.poses_client_addr = None
        
        # Port 5002: Images Channel
        self.images_server = None
        self.images_client = None
        self.images_client_addr = None
        
        # FSM State Machine
        self.fsm_states = [
            'start',
            'await_surgeon_input',
            'bring_manipulator',
            'auto_reposition',
            'segment_and_register',
            'finished'
        ]
        self.current_fsm_state = 'await_surgeon_input'
        self.state_lock = threading.Lock()
        
        # Drill poses data
        self.drill_poses = []
        self.generate_sample_drill_poses()
        
        # Image annotation state
        self.annotation_received = False
        self.segmentation_sent = False
        
    # ============================================================================
    # PORT 5000: CONTROL CHANNEL
    # ============================================================================
    
    def start_control_server(self):
        """Port 5000: Commands + FSM State"""
        self.control_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.control_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.control_server.bind((self.host, 5000))
        self.control_server.listen(1)
        self.control_server.setblocking(False)
        
        print(f"[PORT 5000] Control channel listening on {self.host}:5000")
        
        while self.running:
            # Accept new connections
            if self.control_client is None:
                try:
                    client, addr = self.control_server.accept()
                    self.control_client = client
                    self.control_client_addr = addr
                    self.control_client.setblocking(False)
                    print(f"[PORT 5000] ✅ AVP connected from {addr}")
                    
                    # Send initial FSM state
                    self.send_fsm_state()
                except BlockingIOError:
                    pass
            
            # Receive commands from AVP
            if self.control_client is not None:
                try:
                    data = self.control_client.recv(1024)
                    if not data:
                        print(f"[PORT 5000] ❌ AVP disconnected")
                        self.control_client.close()
                        self.control_client = None
                        self.control_client_addr = None
                    else:
                        message = data.decode('utf-8').strip()
                        self.handle_command(message)
                        
                        # Send acknowledgment
                        try:
                            self.control_client.sendall(b'acknowledged\n')
                        except:
                            pass
                except BlockingIOError:
                    pass
                except Exception as e:
                    print(f"[PORT 5000] Error: {e}")
                    try:
                        self.control_client.close()
                    except:
                        pass
                    self.control_client = None
            
            time.sleep(0.1)
    
    def send_fsm_state(self):
        """Push current FSM state to AVP"""
        if self.control_client is None:
            return
        
        with self.state_lock:
            message = f"STATE:{self.current_fsm_state}\n"
        
        try:
            self.control_client.sendall(message.encode('utf-8'))
            print(f"[PORT 5000] 📤 Sent FSM state: {self.current_fsm_state}")
        except Exception as e:
            print(f"[PORT 5000] Failed to send state: {e}")
    
    def handle_command(self, command: str):
        """Handle commands received from AVP"""
        print(f"[PORT 5000] 📥 Command received: '{command}'")
        
        if command == "annotate":
            print("[PORT 5000] 🎨 Annotate command - triggering image send on Port 5002")
            threading.Thread(target=self.send_image_for_annotation, daemon=True).start()
        
        elif command == "proceed_mission":
            print("[PORT 5000] ▶️  Proceed mission")
            self.transition_fsm_state('bring_manipulator')
            time.sleep(2)
            self.transition_fsm_state('auto_reposition')
            time.sleep(2)
            self.transition_fsm_state('await_surgeon_input')
        
        elif command == "reset_mission":
            print("[PORT 5000] 🔄 Reset mission")
            self.transition_fsm_state('bring_manipulator')
        
        elif command == "accept":
            print("[PORT 5000] ✅ Segmentation accepted")
            self.transition_fsm_state('await_surgeon_input')
            # Generate new drill poses
            self.generate_sample_drill_poses()
        
        elif command == "reject":
            print("[PORT 5000] ❌ Segmentation rejected")
            self.transition_fsm_state('await_surgeon_input')
        
        elif command == "robot_home":
            print("[PORT 5000] 🏠 Moving robot home")
        
        elif command == "robot_away":
            print("[PORT 5000] ✈️  Moving robot away")
        
        elif command.startswith("drill_"):
            print(f"[PORT 5000] 🔧 Drill command: {command}")
        
        elif command.startswith("clear_"):
            print(f"[PORT 5000] 🗑️  Clear obstacle command: {command}")
            # In the real system, this would remove the obstacle from the planning stack
            print(f"[PORT 5000] ✅ Obstacle cleared (mock)")
        
        elif command == "KILLALL":
            print("[PORT 5000] 🛑 EMERGENCY STOP!")
        
        else:
            print(f"[PORT 5000] ⚠️  Unknown command: {command}")
    
    def transition_fsm_state(self, new_state: str):
        """Change FSM state and broadcast"""
        with self.state_lock:
            old_state = self.current_fsm_state
            self.current_fsm_state = new_state
        
        print(f"[PORT 5000] 🔄 FSM: {old_state} → {new_state}")
        self.send_fsm_state()
    
    # ============================================================================
    # PORT 5001: DRILL POSES CHANNEL
    # ============================================================================
    
    def start_poses_server(self):
        """Port 5001: Drill Poses Stream (10Hz)"""
        self.poses_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.poses_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.poses_server.bind((self.host, 5001))
        self.poses_server.listen(1)
        self.poses_server.setblocking(False)
        
        print(f"[PORT 5001] Drill poses channel listening on {self.host}:5001")
        
        while self.running:
            # Accept new connections
            if self.poses_client is None:
                try:
                    client, addr = self.poses_server.accept()
                    self.poses_client = client
                    self.poses_client_addr = addr
                    self.poses_client.setblocking(False)
                    print(f"[PORT 5001] ✅ AVP connected from {addr}")
                except BlockingIOError:
                    pass
            
            # Send drill poses at 10Hz
            if self.poses_client is not None and len(self.drill_poses) > 0:
                try:
                    message = self.format_poses_message()
                    self.poses_client.sendall(message.encode('utf-8'))
                    # Don't spam logs - only print occasionally
                    if random.random() < 0.1:  # 10% of the time
                        print(f"[PORT 5001] 📤 Sent {len(self.drill_poses)} drill poses")
                except (BrokenPipeError, ConnectionResetError, OSError) as e:
                    print(f"[PORT 5001] ❌ AVP disconnected: {e}")
                    try:
                        self.poses_client.close()
                    except:
                        pass
                    self.poses_client = None
            
            time.sleep(0.1)  # 10Hz
    
    def generate_sample_drill_poses(self):
        """Generate realistic drill poses"""
        self.drill_poses = [
            # Femur holes (3)
            {'pos': [0.05, 0.03, 0.15], 'ori': [0.0, 0.0, 0.707, 0.707]},
            {'pos': [0.08, 0.03, 0.15], 'ori': [0.0, 0.0, 0.707, 0.707]},
            {'pos': [0.11, 0.03, 0.15], 'ori': [0.0, 0.0, 0.707, 0.707]},
            # Tibia holes (2)
            {'pos': [0.05, -0.05, 0.12], 'ori': [0.0, 0.0, -0.707, 0.707]},
            {'pos': [0.08, -0.05, 0.12], 'ori': [0.0, 0.0, -0.707, 0.707]},
        ]
        print(f"[PORT 5001] 🎯 Generated {len(self.drill_poses)} drill poses")
    
    def format_poses_message(self) -> str:
        """Format poses as: POSES|x,y,z,qx,qy,qz,qw|..."""
        pose_strings = []
        for pose in self.drill_poses:
            pos = pose['pos']
            ori = pose['ori']
            pose_str = f"{pos[0]:.6f},{pos[1]:.6f},{pos[2]:.6f},{ori[0]:.6f},{ori[1]:.6f},{ori[2]:.6f},{ori[3]:.6f}"
            pose_strings.append(pose_str)
        
        return "POSES|" + "|".join(pose_strings) + "\n"
    
    # ============================================================================
    # PORT 5002: IMAGES CHANNEL
    # ============================================================================
    
    def start_images_server(self):
        """Port 5002: Images + Annotations"""
        self.images_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.images_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.images_server.bind((self.host, 5002))
        self.images_server.listen(1)
        self.images_server.setblocking(False)
        
        print(f"[PORT 5002] Images channel listening on {self.host}:5002")
        
        while self.running:
            # Accept new connections
            if self.images_client is None:
                try:
                    client, addr = self.images_server.accept()
                    self.images_client = client
                    self.images_client_addr = addr
                    self.images_client.setblocking(False)
                    print(f"[PORT 5002] ✅ AVP connected from {addr}")
                except BlockingIOError:
                    pass
            
            # Receive annotations from AVP
            if self.images_client is not None:
                try:
                    data = self.images_client.recv(4096)
                    if not data:
                        print(f"[PORT 5002] ❌ AVP disconnected")
                        self.images_client.close()
                        self.images_client = None
                        self.images_client_addr = None
                    else:
                        message = data.decode('utf-8').strip()
                        if message.startswith("ANNOTATIONS:"):
                            self.handle_annotations(message)
                            # Send acknowledgment
                            try:
                                self.images_client.sendall(b'acknowledged\n')
                            except:
                                pass
                except BlockingIOError:
                    pass
                except Exception as e:
                    print(f"[PORT 5002] Error: {e}")
                    try:
                        self.images_client.close()
                    except:
                        pass
                    self.images_client = None
            
            time.sleep(0.1)
    
    def send_image_for_annotation(self):
        """Send a mock RGB image to AVP for annotation"""
        if self.images_client is None:
            print("[PORT 5002] ⚠️  No images client connected")
            return
        
        try:
            # Generate a mock image (720p)
            img = Image.new('RGB', (1280, 720), color=(40, 40, 50))
            draw = ImageDraw.Draw(img)
            
            # Draw some mock bones
            # Femur (orange)
            draw.ellipse([400, 250, 600, 450], fill=(255, 153, 51), outline=(255, 100, 0), width=3)
            draw.text((480, 340), "FEMUR", fill=(255, 255, 255))
            
            # Tibia (light orange)
            draw.ellipse([680, 350, 880, 550], fill=(255, 178, 102), outline=(255, 150, 50), width=3)
            draw.text((760, 440), "TIBIA", fill=(255, 255, 255))
            
            # Instructions
            draw.text((400, 50), "TAP ON FEMUR (left) AND TIBIA (right)", fill=(255, 255, 255))
            
            # Convert to base64
            buffer = io.BytesIO()
            img.save(buffer, format='JPEG', quality=85)
            image_bytes = buffer.getvalue()
            image_base64 = base64.b64encode(image_bytes).decode('utf-8')
            
            message = f"IMAGE:{image_base64}\n"
            
            print(f"[PORT 5002] 📤 Sending RGB image ({len(image_base64)} chars)")
            
            # Send in blocking mode for large data
            original_blocking = self.images_client.getblocking()
            self.images_client.setblocking(True)
            try:
                self.images_client.sendall(message.encode('utf-8'))
                print(f"[PORT 5002] ✅ Image sent successfully")
            finally:
                self.images_client.setblocking(original_blocking)
            
            self.annotation_received = False
            self.segmentation_sent = False
            
        except Exception as e:
            print(f"[PORT 5002] ❌ Failed to send image: {e}")
    
    def handle_annotations(self, message: str):
        """Handle annotation response from AVP"""
        try:
            json_str = message[12:]  # Remove "ANNOTATIONS:" prefix
            annotations = json.loads(json_str)
            
            print(f"[PORT 5002] 📥 Received annotations: {annotations}")
            
            self.annotation_received = True
            
            # Wait a moment, then send segmented image
            time.sleep(1.0)
            self.send_segmented_image()
            
        except Exception as e:
            print(f"[PORT 5002] ❌ Failed to parse annotations: {e}")
    
    def send_segmented_image(self):
        """Send a mock segmented image to AVP for review"""
        if self.images_client is None:
            print("[PORT 5002] ⚠️  No images client connected")
            return
        
        try:
            # Generate a mock segmented image (overlay)
            img = Image.new('RGB', (1280, 720), color=(40, 40, 50))
            draw = ImageDraw.Draw(img)
            
            # Draw segmented bones with colored overlays
            # Femur (red overlay)
            draw.ellipse([400, 250, 600, 450], fill=(255, 100, 100, 128), outline=(255, 0, 0), width=5)
            draw.text((460, 340), "FEMUR", fill=(255, 255, 255))
            draw.text((450, 360), "SEGMENTED", fill=(255, 200, 200))
            
            # Tibia (blue overlay)
            draw.ellipse([680, 350, 880, 550], fill=(100, 100, 255, 128), outline=(0, 0, 255), width=5)
            draw.text((740, 440), "TIBIA", fill=(255, 255, 255))
            draw.text((730, 460), "SEGMENTED", fill=(200, 200, 255))
            
            # Instructions
            draw.text((400, 50), "REVIEW SEGMENTATION - ACCEPT or REJECT", fill=(255, 255, 255))
            
            # Convert to base64
            buffer = io.BytesIO()
            img.save(buffer, format='JPEG', quality=85)
            image_bytes = buffer.getvalue()
            image_base64 = base64.b64encode(image_bytes).decode('utf-8')
            
            message = f"SEGMENTED_IMAGE:{image_base64}\n"
            
            print(f"[PORT 5002] 📤 Sending segmented image ({len(image_base64)} chars)")
            
            # Send in blocking mode for large data
            original_blocking = self.images_client.getblocking()
            self.images_client.setblocking(True)
            try:
                self.images_client.sendall(message.encode('utf-8'))
                print(f"[PORT 5002] ✅ Segmented image sent successfully")
                self.segmentation_sent = True
            finally:
                self.images_client.setblocking(original_blocking)
            
        except Exception as e:
            print(f"[PORT 5002] ❌ Failed to send segmented image: {e}")
    
    # ============================================================================
    # MAIN SERVER CONTROL
    # ============================================================================
    
    def start(self):
        """Start all three servers"""
        print("=" * 70)
        print("🚀 MOCK UNIFIED ROS SERVER STARTING")
        print("=" * 70)
        print(f"Host: {self.host}")
        print(f"Port 5000: Control (Commands + FSM State)")
        print(f"Port 5001: Drill Poses (10Hz stream)")
        print(f"Port 5002: Images (Annotation + Segmentation)")
        print("=" * 70)
        
        # Start server threads
        control_thread = threading.Thread(target=self.start_control_server, daemon=True)
        poses_thread = threading.Thread(target=self.start_poses_server, daemon=True)
        images_thread = threading.Thread(target=self.start_images_server, daemon=True)
        
        control_thread.start()
        poses_thread.start()
        images_thread.start()
        
        print("\n✅ All servers started. Waiting for AVP connections...")
        print("\n📋 Available Commands (send from AVP):")
        print("   - annotate")
        print("   - proceed_mission")
        print("   - reset_mission")
        print("   - accept / reject")
        print("   - robot_home / robot_away")
        print("   - drill_femur_1, drill_femur_2, drill_femur_3")
        print("   - drill_tibia_1, drill_tibia_2")
        print("   - clear_femur_1, clear_femur_2, clear_femur_3")
        print("   - clear_tibia_1, clear_tibia_2")
        print("   - KILLALL (emergency stop)")
        print("\n🎮 Interactive Commands:")
        print("   - Type 'state <state_name>' to change FSM state")
        print("   - Type 'poses' to toggle drill poses")
        print("   - Type 'quit' to exit")
        print("\n" + "=" * 70)
        
        # Interactive console
        try:
            while self.running:
                user_input = input("\n[SERVER] Command: ").strip().lower()
                
                if user_input == 'quit':
                    self.running = False
                    print("Shutting down...")
                    break
                
                elif user_input.startswith('state '):
                    new_state = user_input[6:]
                    if new_state in self.fsm_states:
                        self.transition_fsm_state(new_state)
                    else:
                        print(f"❌ Unknown state. Available: {', '.join(self.fsm_states)}")
                
                elif user_input == 'poses':
                    self.generate_sample_drill_poses()
                    print("✅ Drill poses regenerated")
                
                elif user_input == 'help':
                    print("\n📋 Server Commands:")
                    print("   state <name>  - Change FSM state")
                    print("   poses         - Regenerate drill poses")
                    print("   quit          - Exit server")
                
                else:
                    print(f"❌ Unknown command. Type 'help' for options.")
        
        except KeyboardInterrupt:
            print("\n\n⚠️  Interrupted by user")
            self.running = False
        
        finally:
            # Cleanup
            print("\n🛑 Cleaning up...")
            if self.control_server:
                self.control_server.close()
            if self.poses_server:
                self.poses_server.close()
            if self.images_server:
                self.images_server.close()
            print("✅ Server stopped")


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else '0.0.0.0'
    
    server = MockUnifiedROSServer(host=host)
    server.start()


if __name__ == "__main__":
    main()

