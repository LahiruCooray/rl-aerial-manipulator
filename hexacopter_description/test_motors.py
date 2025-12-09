#!/usr/bin/env python3
"""
Test motor speeds for custom hexacopter
Publishes motor commands via Gazebo topics
"""

import subprocess
import sys
import time
import os
import shutil

def find_gz_command():
    """Find the gz command in system"""
    # Check common locations
    possible_paths = [
        shutil.which('gz'),  # In PATH
        '/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz',
        '/usr/bin/gz',
        '/usr/local/bin/gz',
    ]
    
    for path in possible_paths:
        if path and os.path.exists(path):
            return path
    
    return None

class MotorTester:
    def __init__(self, model_name=None, gz_cmd=None):
        # Find gz command
        self.gz_cmd = gz_cmd or find_gz_command()
        if not self.gz_cmd:
            raise RuntimeError("Could not find 'gz' command")
        
        # Try to auto-detect model name (PX4 vs standalone Gazebo)
        if model_name is None:
            # Try PX4 naming first (custom_hexa_0)
            self.model_name = "custom_hexa_0"
            # If that doesn't work, will fall back to "custom_hexa"
        else:
            self.model_name = model_name
            
        self.topic = f"/model/{self.model_name}/command/motor_speed"
        self.hover_speed = 459  # rad/s for hover
        self.max_speed = 820    # rad/s maximum
        
    def publish_speeds(self, velocities):
        """Publish motor velocities to Gazebo topic"""
        if len(velocities) != 6:
            print("Error: Need exactly 6 motor velocities")
            return False
            
        vel_str = ", ".join(str(v) for v in velocities)
        cmd = [
            self.gz_cmd, "topic", "-t", self.topic,
            "-m", "gz.msgs.Actuators",
            "-p", f"velocity: [{vel_str}]"
        ]
        
        try:
            subprocess.run(cmd, check=True, capture_output=True)
            return True
        except subprocess.CalledProcessError as e:
            print(f"Error publishing: {e}")
            return False
    
    def stop_all(self):
        """Stop all motors"""
        print("🛑 Stopping all motors")
        self.publish_speeds([0, 0, 0, 0, 0, 0])
    
    def test_all(self, speed):
        """Test all motors at same speed"""
        print(f"🚁 Testing all motors at {speed} rad/s")
        self.publish_speeds([speed] * 6)
    
    def test_single(self, motor_num, speed):
        """Test single motor"""
        if motor_num < 0 or motor_num > 5:
            print("Error: Motor number must be 0-5")
            return
        
        velocities = [0] * 6
        velocities[motor_num] = speed
        print(f"🔧 Testing Motor {motor_num} at {speed} rad/s")
        self.publish_speeds(velocities)
    
    def test_hover(self):
        """Test hover configuration"""
        print(f"✈️  Testing hover at {self.hover_speed} rad/s per motor")
        self.publish_speeds([self.hover_speed] * 6)
    
    def sequential_test(self, speed=400, duration=2):
        """Test each motor sequentially"""
        print(f"🔄 Sequential motor test at {speed} rad/s")
        for i in range(6):
            print(f"  Motor {i} ON")
            self.test_single(i, speed)
            time.sleep(duration)
            self.stop_all()
            time.sleep(1)
        print("✅ Sequential test complete")
    
    def ramp_test(self, start=0, end=500, steps=10, delay=0.5):
        """Ramp all motors from start to end speed"""
        print(f"📈 Ramping motors from {start} to {end} rad/s")
        for speed in range(start, end, (end-start)//steps):
            print(f"  Speed: {speed} rad/s")
            self.test_all(speed)
            time.sleep(delay)
        self.test_all(end)
        print("✅ Ramp test complete")

def print_motor_diagram():
    """Print hexacopter motor layout and rotation directions"""
    print("\n" + "="*60)
    print("Hexacopter Top-Down View (FRD Frame):")
    print("="*60)
    print("""
                    FRONT
                      ↑
                      |
         M3(FL)       |       M1(FR)
          CW⟳        |        CCW⟲
            \\        |        /
             \\       |       /
   M4(L) ----+-------+-------+---- M5(R)
   CCW⟲      |   CENTER   |      CW⟳
             /       |       \\
            /        |        \\
          M0(BL)     |       M2(BR)
          CW⟳        |        CCW⟲
                     |
                   BACK
""")
    print("\nExpected Motor Rotation Directions:")
    print("="*60)
    print(f"{'Motor':<8} {'Position':<15} {'Direction':<12} {'Visual':<10}")
    print("="*60)
    print(f"  0      {'Back Left':<15} {'CW ⟳':<12} {'Red':<10}")
    print(f"  1      {'Front Right':<15} {'CCW ⟲':<12} {'Green':<10}")
    print(f"  2      {'Back Right':<15} {'CCW ⟲':<12} {'Green':<10}")
    print(f"  3      {'Front Left':<15} {'CW ⟳':<12} {'Red':<10}")
    print(f"  4      {'Left Center':<15} {'CCW ⟲':<12} {'Green':<10}")
    print(f"  5      {'Right Center':<15} {'CW ⟳':<12} {'Red':<10}")
    print("="*60)
    print("Note: CW=Clockwise (Red), CCW=Counter-Clockwise (Green)")
    print("="*60)

def show_menu():
    """Display menu options"""
    print("\n" + "="*50)
    print("🚁 Custom Hexacopter Motor Test")
    print("="*50)
    print("1) Test all motors - Low (200 rad/s)")
    print("2) Test all motors - Medium (400 rad/s)")
    print("3) Test all motors - Hover (459 rad/s)")
    print("4) Test all motors - High (600 rad/s)")
    print("5) Test single motor")
    print("6) Custom speed all motors")
    print("7) Stop all motors")
    print("8) Sequential test (each motor)")
    print("9) Ramp test (0 → 500 rad/s)")
    print("d) Display motor diagram")
    print("0) Exit")
    print("="*50)

def main():
    import argparse
    parser = argparse.ArgumentParser(description='Test hexacopter motors')
    parser.add_argument('-m', '--model', default=None, 
                       help='Model name (default: auto-detect custom_hexa_0 or custom_hexa)')
    args = parser.parse_args()
    
    # Find gz command first
    gz_cmd = find_gz_command()
    if not gz_cmd:
        print("❌ Error: 'gz' command not found.")
        print("💡 Try: source /opt/ros/jazzy/setup.bash")
        print("   Or make sure Gazebo Harmonic is installed")
        sys.exit(1)
    
    print(f"✅ Found gz command: {gz_cmd}")
    
    # Try to detect if using standalone Gazebo or PX4
    model_name = args.model
    if model_name is None:
        # List available topics to auto-detect
        try:
            result = subprocess.run([gz_cmd, "topic", "-l"], 
                                  capture_output=True, text=True, check=True, timeout=5)
            topics = result.stdout
            
            # Check for different topic patterns
            if "/model/custom_hexa_0/command/motor_speed" in topics:
                model_name = "custom_hexa_0"
                print("🔍 Detected PX4 SITL model: custom_hexa_0 (with /model/ prefix)")
            elif "/custom_hexa_0/command/motor_speed" in topics:
                model_name = "custom_hexa_0"
                print("🔍 Detected model: custom_hexa_0 (direct topic)")
            elif "/model/custom_hexa/command/motor_speed" in topics:
                model_name = "custom_hexa"
                print("🔍 Detected standalone Gazebo model: custom_hexa")
            elif "/custom_hexa/command/motor_speed" in topics:
                model_name = "custom_hexa"
                print("🔍 Detected model: custom_hexa (direct topic)")
            else:
                print("⚠️  Could not auto-detect model. Trying custom_hexa_0")
                print("📋 Available topics (first 10):")
                for line in topics.split('\n')[:10]:
                    print(f"   {line}")
                model_name = "custom_hexa_0"
        except subprocess.TimeoutExpired:
            print("⚠️  Timeout detecting model. Is Gazebo running?")
            model_name = "custom_hexa"
        except Exception as e:
            print(f"⚠️  Error detecting model: {e}")
            model_name = "custom_hexa"
    
    try:
        tester = MotorTester(model_name=model_name, gz_cmd=gz_cmd)
    except RuntimeError as e:
        print(f"❌ Error: {e}")
        sys.exit(1)
    
    # Check if gz command exists
    print(f"📡 Publishing to: {tester.topic}")
    
    # Test connection and try to find the correct topic
    try:
        result = subprocess.run([gz_cmd, "topic", "-l"], 
                              capture_output=True, text=True, check=True, timeout=5)
        
        # Check if our topic exists (with or without /model/ prefix)
        topics = result.stdout
        topic_found = False
        actual_topic = None
        
        # Try both formats
        for topic_variant in [
            f"/model/{model_name}/command/motor_speed",
            f"/{model_name}/command/motor_speed"
        ]:
            if topic_variant in topics:
                topic_found = True
                actual_topic = topic_variant
                break
        
        if topic_found:
            print(f"✅ Found topic: {actual_topic}")
            # Update tester to use correct topic
            tester.topic = actual_topic
        else:
            print(f"⚠️  Warning: Motor command topic not found")
            print("   Looking for motor-related topics...")
            for line in topics.split('\n'):
                if 'motor' in line.lower() or 'command' in line.lower():
                    print(f"   Found: {line}")
            
    except Exception as e:
        print(f"⚠️  Warning: Could not verify topic: {e}")
    
    # Display motor diagram at startup
    print_motor_diagram()
    
    while True:
        show_menu()
        choice = input("\nSelect option: ").strip().lower()
        
        try:
            if choice == '1':
                tester.test_all(200)
            elif choice == '2':
                tester.test_all(400)
            elif choice == '3':
                tester.test_hover()
            elif choice == '4':
                tester.test_all(600)
            elif choice == '5':
                motor = int(input("Enter motor number (0-5): "))
                speed = float(input("Enter speed (rad/s): "))
                tester.test_single(motor, speed)
            elif choice == '6':
                speed = float(input("Enter speed (rad/s): "))
                tester.test_all(speed)
            elif choice == '7':
                tester.stop_all()
            elif choice == '8':
                tester.sequential_test()
            elif choice == '9':
                tester.ramp_test()
            elif choice == 'd':
                print_motor_diagram()
            elif choice == '0':
                tester.stop_all()
                print("👋 Exiting...")
                break
            else:
                print("❌ Invalid option")
        except KeyboardInterrupt:
            print("\n⚠️  Interrupted")
            tester.stop_all()
            break
        except ValueError:
            print("❌ Invalid input")
        except Exception as e:
            print(f"❌ Error: {e}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n🛑 Stopping all motors and exiting...")
        MotorTester().stop_all()
