#!/usr/bin/env python3
"""
Self-Balancing Robot Controller
Simple Python script to control the robot via serial USB connection

Usage:
    python3 robot_controller.py /dev/ttyUSB0
    python3 robot_controller.py COM3  (Windows)

Requirements:
    pip install pyserial
"""

import serial
import time
import sys
import os

# Try to import keyboard library for interactive control
try:
    import keyboard
    KEYBOARD_AVAILABLE = True
except ImportError:
    KEYBOARD_AVAILABLE = False
    print("Note: Install 'keyboard' library for interactive control: pip install keyboard")

class RobotController:
    """Controller for self-balancing robot via serial connection"""
    
    def __init__(self, port, baud_rate=115200):
        """
        Initialize robot controller
        
        Args:
            port: Serial port (e.g., '/dev/ttyUSB0' or 'COM3')
            baud_rate: Communication speed (default: 115200)
        """
        try:
            self.serial = serial.Serial(port, baud_rate, timeout=1)
            print(f"Connected to robot on {port} at {baud_rate} baud")
            time.sleep(2)  # Wait for Arduino to reset
            print("Robot ready!")
        except serial.SerialException as e:
            print(f"Error connecting to {port}: {e}")
            print("\nAvailable ports:")
            self.list_ports()
            sys.exit(1)
    
    def send_command(self, command):
        """
        Send a single character command to the robot
        
        Args:
            command: Single character ('f', 'b', 'l', 'r', 's')
        """
        if command in ['f', 'b', 'l', 'r', 's']:
            self.serial.write(command.encode())
            return True
        else:
            print(f"Invalid command: {command}")
            return False
    
    def forward(self):
        """Move forward"""
        self.send_command('f')
        print("→ Moving forward")
    
    def backward(self):
        """Move backward"""
        self.send_command('b')
        print("← Moving backward")
    
    def left(self):
        """Turn left"""
        self.send_command('l')
        print("↺ Turning left")
    
    def right(self):
        """Turn right"""
        self.send_command('r')
        print("↻ Turning right")
    
    def stop(self):
        """Stop all movement"""
        self.send_command('s')
        print("■ Stopped")
    
    def read_serial(self):
        """Read and print any data from Arduino"""
        if self.serial.in_waiting > 0:
            data = self.serial.readline().decode('utf-8', errors='ignore').strip()
            if data:
                print(f"[Arduino]: {data}")
    
    def close(self):
        """Close serial connection"""
        self.stop()  # Stop robot before disconnecting
        time.sleep(0.1)
        self.serial.close()
        print("Connection closed")
    
    @staticmethod
    def list_ports():
        """List available serial ports"""
        try:
            from serial.tools import list_ports
            ports = list_ports.comports()
            for port in ports:
                print(f"  {port.device} - {port.description}")
        except Exception as e:
            print(f"  Could not list ports: {e}")


def demo_sequence(robot):
    """Run a demo movement sequence"""
    print("\n=== Demo Sequence ===")
    
    sequences = [
        ("forward", 2),
        ("stop", 1),
        ("backward", 2),
        ("stop", 1),
        ("left", 1),
        ("stop", 1),
        ("right", 1),
        ("stop", 1),
    ]
    
    for command, duration in sequences:
        getattr(robot, command)()
        time.sleep(duration)
    
    print("Demo complete!")


def interactive_control(robot):
    """Interactive keyboard control (requires 'keyboard' library)"""
    if not KEYBOARD_AVAILABLE:
        print("Keyboard library not available. Install with: pip install keyboard")
        return
    
    print("\n=== Interactive Control ===")
    print("Controls:")
    print("  W - Forward")
    print("  S - Backward")
    print("  A - Left")
    print("  D - Right")
    print("  SPACE - Stop")
    print("  Q - Quit")
    print("\nPress keys to control robot...")
    
    try:
        while True:
            if keyboard.is_pressed('w'):
                robot.forward()
                time.sleep(0.1)
            elif keyboard.is_pressed('s'):
                robot.backward()
                time.sleep(0.1)
            elif keyboard.is_pressed('a'):
                robot.left()
                time.sleep(0.1)
            elif keyboard.is_pressed('d'):
                robot.right()
                time.sleep(0.1)
            elif keyboard.is_pressed('space'):
                robot.stop()
                time.sleep(0.1)
            elif keyboard.is_pressed('q'):
                print("\nQuitting...")
                break
            
            # Read any debug messages from Arduino
            robot.read_serial()
            time.sleep(0.05)
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")


def command_line_control(robot):
    """Simple command-line control"""
    print("\n=== Command Line Control ===")
    print("Commands: f=forward, b=backward, l=left, r=right, s=stop, q=quit")
    print("Type command and press Enter")
    
    command_map = {
        'f': robot.forward,
        'b': robot.backward,
        'l': robot.left,
        'r': robot.right,
        's': robot.stop,
    }
    
    try:
        while True:
            cmd = input("> ").lower().strip()
            
            if cmd == 'q' or cmd == 'quit':
                break
            elif cmd in command_map:
                command_map[cmd]()
            else:
                print("Invalid command")
            
            # Read any messages from Arduino
            robot.read_serial()
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")


def main():
    """Main program"""
    print("=" * 50)
    print("Self-Balancing Robot Controller")
    print("=" * 50)
    
    # Check for port argument
    if len(sys.argv) < 2:
        print("\nUsage: python3 robot_controller.py <serial_port>")
        print("\nExamples:")
        print("  python3 robot_controller.py /dev/ttyUSB0  (Linux)")
        print("  python3 robot_controller.py /dev/tty.usbserial  (Mac)")
        print("  python3 robot_controller.py COM3  (Windows)")
        print("\nAvailable ports:")
        RobotController.list_ports()
        sys.exit(1)
    
    port = sys.argv[1]
    
    # Connect to robot
    robot = RobotController(port)
    
    try:
        # Menu
        while True:
            print("\n" + "=" * 50)
            print("Select control mode:")
            print("  1 - Demo sequence")
            print("  2 - Interactive control (keyboard)")
            print("  3 - Command line control")
            print("  4 - Quit")
            print("=" * 50)
            
            choice = input("Enter choice (1-4): ").strip()
            
            if choice == '1':
                demo_sequence(robot)
            elif choice == '2':
                interactive_control(robot)
            elif choice == '3':
                command_line_control(robot)
            elif choice == '4':
                break
            else:
                print("Invalid choice")
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    finally:
        robot.close()


if __name__ == "__main__":
    main()

