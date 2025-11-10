#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import MountControl, OverrideRCIn
from time import sleep
import numpy as np


class BlueROVJoystick(Node):
    def __init__(self):
        super().__init__('bluerov_joystick_node')
        
        self.get_logger().info('Starting BlueROV joystick')
        
        # Publishers
        self.pub_msg_override = self.create_publisher(OverrideRCIn, "rc/override", 10)
        self.mount_pub = self.create_publisher(MountControl, 'mount_control/command', 10)
        
        # Subscribers
        self.subscription = self.create_subscription(Joy, 'joy', self.joyCallback, 10)

        # Timer for continuous control (20 Hz)
        self.timer = self.create_timer(0.05, self.continuous_control_callback)
        
        # Arm/disarm
        self.armDisarm(False)

        # Button state tracking for continuous control
        self.lb_held = False
        self.rb_held = False
        self.dpad_left_held = False
        self.dpad_right_held = False

        # Mode variables
        self.set_mode = [True, False, False]  # [manual, auto, correction]
        self.arming = False
        
        # Gripper control
        self.rt_was_pressed = False
        self.lt_was_pressed = False

        # Pin assignments
        self.light_pin = 11.0
        self.gripper_pin = 13.0
        self.camera_servo_pin = 16.0

        # Light values 
        self.light = 1100.0
        self.light_min = 1100.0
        self.light_max = 1900.0
        self.light_step = 15.0
        
        # Camera tilt 
        self.tilt = 0.0
        self.tilt_step = 2.0
        self.tilt_min = -60.0
        self.tilt_max = 60.0

        # Gripper
        self.gripper = 1150.0
        self.gripper_min = 1150.0
        self.gripper_max = 1580.0

        # Initialization test
        self.run_initialization_test = False
        if self.run_initialization_test:
            self.initialization_test()

    def initialization_test(self):
        """Test light and camera servo"""
        self.get_logger().info("Testing light and camera servo...")
        
        # Flash light
        self.light = self.light_max
        self.send_servo_command(self.light_pin, self.light)
        sleep(0.5)
        self.light = self.light_min
        self.send_servo_command(self.light_pin, self.light)
        sleep(0.5)
        
        # Move camera
        self.tilt = 0.0
        self.send_camera_tilt_command(self.tilt)
        sleep(0.5)
        self.tilt = self.tilt_max
        self.send_camera_tilt_command(self.tilt)
        sleep(0.5)
        self.tilt = self.tilt_min
        self.send_camera_tilt_command(self.tilt)
        sleep(0.5)
        self.tilt = 0.0
        self.send_camera_tilt_command(self.tilt)

    def continuous_control_callback(self):
        """Timer callback for continuous camera tilt and light control"""
        changed_tilt = False
        changed_light = False
        
        # Continuous camera tilt
        if self.lb_held:
            self.tilt = min(self.tilt + self.tilt_step, self.tilt_max)
            changed_tilt = True
        elif self.rb_held:
            self.tilt = max(self.tilt - self.tilt_step, self.tilt_min)
            changed_tilt = True
        
        # Continuous light control
        if self.dpad_right_held:
            self.light = min(self.light + self.light_step, self.light_max)
            changed_light = True
        elif self.dpad_left_held:
            self.light = max(self.light - self.light_step, self.light_min)
            changed_light = True
        
        # Publish updates
        if changed_tilt:
            self.send_camera_tilt_command(self.tilt)
        if changed_light:
            self.send_servo_command(self.light_pin, self.light)

    def send_servo_command(self, pin_number, value):
        """Send servo command via MAVROS"""
        client = self.create_client(CommandLong, 'cmd/command')
        
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('MAVROS service not available!')
            return
        
        request = CommandLong.Request()
        request.command = 183  # MAV_CMD_DO_SET_SERVO
        request.param1 = pin_number
        request.param2 = value
        request.param3 = 0.0
        request.param4 = 0.0

        client.call_async(request)
        self.get_logger().debug(f'Servo pin {pin_number}, value {value}')

    def armDisarm(self, armed):
        """Arm or disarm vehicle"""
        cli = self.create_client(CommandLong, 'cmd/command')
        result = False
        while not result:
            result = cli.wait_for_service(timeout_sec=4.0)
            self.get_logger().info(f"{'Arming' if armed else 'Disarming'} requested, waiting for service: {result}")
        
        req = CommandLong.Request()
        req.broadcast = False
        req.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
        req.confirmation = 0
        req.param1 = 1.0 if armed else 0.0
        req.param2 = 0.0
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0
        
        self.get_logger().info("Sending command...")
        cli.call_async(req)
        self.get_logger().info(f"{'Arming' if armed else 'Disarming'} Succeeded")

    def send_camera_tilt_command(self, tilt_angle_deg):
        """Publish camera tilt angle"""
        msg = MountControl()
        msg.mode = 2  # MAV_MOUNT_MODE_MAVLINK_TARGETING
        msg.pitch = tilt_angle_deg
        msg.roll = 0.0
        msg.yaw = 0.0
        msg.altitude = 0.0
        msg.latitude = 0.0
        msg.longitude = 0.0

        self.mount_pub.publish(msg)
        self.get_logger().debug(f"Camera tilt: {tilt_angle_deg:.1f}°")

    def joyCallback(self, data):
        """Process joystick input"""
        # Button mapping
        btn_arm = data.buttons[7]  # Start
        btn_disarm = data.buttons[6]  # Back
        btn_manual_mode = data.buttons[3]  # Y
        btn_automatic_mode = data.buttons[2]  # X
        btn_corrected_mode = data.buttons[0]  # A
        btn_camera_servo_up = data.buttons[4]  # LB
        btn_camera_servo_down = data.buttons[5]  # RB
        btn_camera_rest = data.buttons[9]  # R3

        btn_gripper_open = data.axes[2]  # LT
        btn_gripper_close = data.axes[5]  # RT
        btn_light = data.axes[6]  # D-Pad

        # Update held states
        self.lb_held = btn_camera_servo_up == 1
        self.rb_held = btn_camera_servo_down == 1
        self.dpad_left_held = btn_light == 1.0
        self.dpad_right_held = btn_light == -1.0

        # Arm/Disarm
        if btn_disarm == 1 and self.arming:
            self.arming = False
            self.armDisarm(False)
        if btn_arm == 1 and not self.arming:
            self.arming = True
            self.armDisarm(True)

        # Mode switching
        if btn_manual_mode and not self.set_mode[0]:
            self.set_mode = [True, False, False]
            self.get_logger().info("Mode: Manual")
        elif btn_automatic_mode and not self.set_mode[1]:
            self.set_mode = [False, True, False]
            self.get_logger().info("Mode: Automatic")
        elif btn_corrected_mode and not self.set_mode[2]:
            self.set_mode = [False, False, True]
            self.get_logger().info("Mode: Correction")

        # Camera reset
        if btn_camera_rest:
            self.tilt = 0.0
            self.send_camera_tilt_command(self.tilt)
            self.get_logger().info("Camera tilt reset")

        # Gripper control
        rt_pressed = btn_gripper_close < -0.5
        lt_pressed = btn_gripper_open > 0.5
        
        if rt_pressed and not self.rt_was_pressed and self.gripper < self.gripper_max:
            self.gripper = min(self.gripper + 430, self.gripper_max)
            self.send_servo_command(self.gripper_pin, self.gripper)
            self.get_logger().info(f"Gripper closing: {self.gripper}")
            
        if lt_pressed and not self.lt_was_pressed and self.gripper > self.gripper_min:
            self.gripper = max(self.gripper - 430, self.gripper_min)
            self.send_servo_command(self.gripper_pin, self.gripper)
            self.get_logger().info(f"Gripper opening: {self.gripper}")

        self.rt_was_pressed = rt_pressed
        self.lt_was_pressed = lt_pressed

        # === THRUSTER CONTROL ===
        if self.set_mode[0] and self.arming:  # Manual mode and armed
            # Axis mapping (adjust based on your controller)
            forward = self.mapValueScalSat(-data.axes[1])   # Left stick Y
            lateral = self.mapValueScalSat(-data.axes[0])   # Left stick X
            yaw = self.mapValueScalSat(-data.axes[3])       # Right stick X
            vertical = self.mapValueScalSat(data.axes[4])   # Right stick Y
            
            # Send thruster commands
            self.setOverrideRCIN(
                channel_pitch=1500,
                channel_roll=1500,
                channel_throttle=vertical,
                channel_yaw=yaw,
                channel_forward=forward,
                channel_lateral=lateral
            )
        elif not self.arming:
            # Send neutral commands when disarmed
            self.setOverrideRCIN(1500, 1500, 1500, 1500, 1500, 1500)

    def mapValueScalSat(self, value):
        """Map joystick -1 to 1 to PWM 1100-1900"""
        pulse_width = value * 400 + 1500
        return int(np.clip(pulse_width, 1100, 1700))

    def setOverrideRCIN(self, channel_pitch, channel_roll, channel_throttle,
                        channel_yaw, channel_forward, channel_lateral):
        """Override RC channels to control thrusters"""
        msg_override = OverrideRCIn()
        msg_override.channels[0] = np.uint16(channel_pitch)
        msg_override.channels[1] = np.uint16(channel_roll)
        msg_override.channels[2] = np.uint16(channel_throttle)
        msg_override.channels[3] = np.uint16(channel_yaw)
        msg_override.channels[4] = np.uint16(channel_forward)
        msg_override.channels[5] = np.uint16(channel_lateral)
        msg_override.channels[6] = 1500
        msg_override.channels[7] = 1500

        self.pub_msg_override.publish(msg_override)
        self.get_logger().info(f"published override = {msg_override}")


def main(args=None):
    rclpy.init(args=args)
    node = BlueROVJoystick()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

