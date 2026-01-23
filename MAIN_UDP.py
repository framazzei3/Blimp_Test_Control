import socket
import threading
import pygame 
import time
from datetime import datetime
import os
from prompt_toolkit import PromptSession
from prompt_toolkit.patch_stdout import patch_stdout

# =============================================
# UDP COMMUNICATION SETTINGS
# =============================================
ARDUINO_IP = "194.12.159.116"  # Arduino's IP
ARDUINO_PORT = 2390            # Arduino's UDP port 
LOCAL_PORT = 2391              # Our listening port

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', LOCAL_PORT))
sock.settimeout(1.0)  # Set timeout for response

# =============================================
# SYSTEM CONFIGURATION
# =============================================
# Global flags to control threads
running = True
joystick_active = False

def send_command(command):
    """Send a command to Arduino and wait for acknowledgment"""
    # print(f"[SENDING] {command}")
    sock.sendto(command.encode(), (ARDUINO_IP, ARDUINO_PORT))

def receive_messages():
    """Thread function to continuously listen for UDP messages"""
    while running:
        try:
            data, addr = sock.recvfrom(1024)
            print(f"[ARDUINO] {data.decode()}")
        except socket.timeout:
            continue  # Silently handle timeout
        except Exception as e:
            print(f"[ERROR] Receive failed: {e}")
            break

def print_legend():
    """Show available commands for both Python and Arduino"""
    print("\n" + "="*60)
    print("BLIMP CONTROL SYSTEM - COMMAND LEGEND")
    print("="*60)
    
    print("\nPYTHON CONTROLLER COMMANDS:")
    print("-"*40)
    print("  exit      - Shut down controller")
    print("  help      - Show this legend")
    
    print("\nARDUINO DIRECT COMMANDS:")
    print("-"*40)
    print("  s         - Safety mode (stop all motors)")
    print("  m         - Manual control mode")
    print("  ?         - Show system status")
    
    print("="*60 + "\n")

def map_joystick_to_thrusters():
    """Convert joystick axes to thruster commands"""
    global joystick_active
    pygame.init()
    pygame.joystick.init()
    
    try:
        joy = pygame.joystick.Joystick(0)
        joy.init()
        print(f"\nJoystick '{joy.get_name()}' connected")
        print("Press PS/Start button to return to terminal")
        
        while joystick_active:
            pygame.event.pump()
            
            # Read axes (normalized to -1.0 to +1.0)
            surge = -joy.get_axis(1)  # Forward/Backward 
            heave = -joy.get_axis(3)  # Up/Down 
            yaw = -joy.get_axis(2)     # Rotation 
            
            # Deadzone handling
            if abs(surge) < 0.1: surge = 0
            if abs(heave) < 0.1: heave = 0
            if abs(yaw) < 0.1: yaw = 0
            
            # Convert to Arduino expected format:
            # S:surge,Y:yaw,H:heave (values -100 to 100)
            surge_val = int(surge * 100)
            heave_val = int(heave * 100)
            yaw_val = int(yaw * 100)
            
            # Create command string
            cmd = f"S:{surge_val},Y:{yaw_val},H:{heave_val}"
            send_command(cmd)
            
            # Single-line dynamic output
            # print(f"Surge: {surge_val:3d} | Yaw: {yaw_val:3d} | Heave: {heave_val:3d}", end='\r', flush=True)
            print(f"Surge: {surge_val:3d} | Yaw: {yaw_val:3d} | Heave: {heave_val:3d}    ", end='\r', flush=True) # Aggiungo spazio extra alla fine per sovrascrivere eventuali caratteri residui.

            # Check for exit button (PS/Start)
            if joy.get_button(0):  # Typical PS button index
                print("\nExiting joystick control...")
                joystick_active = False
                send_command("s")
                # return
            
            time.sleep(0.05)  # Control loop at ~20Hz
                        
    except Exception as e:
        print(f"Joystick error: {e}")
    finally:
        pygame.quit()
    

def main():
    global running, joystick_active, camera_active
    
    print("Blimp Control System Initializing...")
    print(f"Target: {ARDUINO_IP}:{ARDUINO_PORT}")
    
    # Start receiver thread
    receiver = threading.Thread(target=receive_messages, daemon=True)
    receiver.start()
    
    session = PromptSession()
    
    try:
        print_legend()
        with patch_stdout():
            while running:
                cmd = session.prompt("Command > ").strip().lower()
                
                if cmd == "exit":
                    running = False
                elif cmd == "help":
                    print_legend()
                elif cmd == "m":
                    send_command("m")  # Send manual mode to Arduino
                    joystick_active = True
                    map_joystick_to_thrusters()
                else:
                    send_command(cmd)
                    
    except KeyboardInterrupt:
        print("\nShutdown requested...")
    finally:
        running = False
        joystick_active = False
        camera_active = False
        sock.close()
        print("System shut down cleanly")

if __name__ == "__main__":
    main()