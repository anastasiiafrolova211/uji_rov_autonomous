#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State  # Ensure mavros_msgs is installed

class MissionNarrator(Node):
    def __init__(self):
        super().__init__('mission_narrator')
        
        # Publishers
        self.chat_pub = self.create_publisher(String, '/mission_chat', 10)
        
        # Subscriptions
        self.batt_sub = self.create_subscription(BatteryState, '/mavros/battery', self.batt_callback, 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)

        # State Tracking
        self.last_action_time = self.get_clock().now()
        self.current_action = "IDLE"
        self.last_mode = "UNKNOWN"
        self.is_armed = False
        self.low_batt_warned = False
        
        self.speak("System Online. Mission Control connected.")

    def speak(self, text):
        msg = String()
        msg.data = f"ROV: {text}"
        self.chat_pub.publish(msg)
        self.get_logger().info(f"Narrated: {text}")

    def batt_callback(self, msg):
        # Warn if below 20%, reset warning if above 25% (charging/swap)
        if msg.percentage < 0.20 and not self.low_batt_warned:
            self.speak(f"CRITICAL: Battery levels at {msg.percentage*100:.1f}%. RTB immediately.")
            self.low_batt_warned = True
        elif msg.percentage > 0.25:
            self.low_batt_warned = False

    def state_callback(self, msg):
        # Narrate Mode Changes (e.g., MANUAL -> STABILIZE)
        if msg.mode != self.last_mode:
            self.speak(f"Flight mode switched to {msg.mode}.")
            self.last_mode = msg.mode
        
        # Narrate Arming Status
        if msg.armed != self.is_armed:
            status = "ARMED" if msg.armed else "DISARMED"
            self.speak(f"Systems {status}. Caution.")
            self.is_armed = msg.armed

    def cmd_callback(self, msg):
        # Throttle narrations to avoid flooding the chat (e.g., once every 3 seconds max for movement)
        now = self.get_clock().now()
        if (now - self.last_action_time).nanoseconds < 3e9: 
            return

        # Simple threshold to ignore joystick drift
        threshold = 0.1
        action = None

        if msg.linear.z < -threshold:
            action = "Descending."
        elif msg.linear.z > threshold:
            action = "Ascending."
        elif msg.linear.x > threshold:
            action = "Moving forward."
        elif msg.angular.z != 0:
            # Differentiate turning vs moving
            action = "Adjusting heading."

        # Only speak if the action has changed or it's a significant maneuver
        if action and action != self.current_action:
            self.speak(action)
            self.current_action = action
            self.last_action_time = now
        elif not action and self.current_action != "IDLE":
             # Reset to IDLE if no input detected
             self.current_action = "IDLE"

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MissionNarrator())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

