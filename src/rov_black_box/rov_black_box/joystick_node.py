#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import MountControl, OverrideRCIn
from time import sleep, time
import numpy as np


class BlueROVJoystick(Node):
    def __init__(self):
        super().__init__('bluerov_joystick_node')
        
        self.get_logger().info('Starting BlueROV joystick with smooth control')
        
        # Publishers
        self.pub_msg_override = self.create_publisher(OverrideRCIn, "rc/override", 10)
        self.mount_pub = self.create_publisher(MountControl, 'mount_control/command', 10)
        
        # Subscribers
        self.subscription = self.create_subscription(Joy, 'joy', self.joyCallback, 10)

        # Smooth control parameters
        self.deadband = 0.08              # Joystick deadband
        self.sensitivity = 0.35           # Cubic response shaping (0-1)
        self.slew_rate_pwm_per_s = 600.0  # Max PWM change per second
        self.min_pwm = 1300               # Conservative range
        self.max_pwm = 1700
        self.abs_min = 1100               # Absolute limits
        self.abs_max = 1900
        self.failsafe_timeout = 0.3       # Seconds
        
        # Current and target PWM per axis
        self.current_pwm = {'forward': 1500, 'lateral': 1500, 'vertical': 1500, 'yaw': 1500}
        self.target_pwm = {'forward': 1500, 'lateral': 1500, 'vertical': 1500, 'yaw': 1500}
        
        # Failsafe tracking
        self.last_joy_time = time()
        
        # Control loop rate
        self.control_rate_hz = 50.0
        self.dt = 1.0 / self.control_rate_hz
        self.max_delta_per_tick = int(self.slew_rate_pwm_per_s * self.dt)
        
        # Timer for smooth control loop (50 Hz)
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        
        # Timer for continuous control (camera/lights) (20 Hz)
        self.timer = self.create_timer(0.05, self.continuous_control_callback)
        
        # Arm/disarm
        self.armDisarm(False)

        # Button state tracking
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

        self.get_logger().info(f'Smooth control: {self.slew_rate_pwm_per_s:.0f} PWM/s, '
                              f'{self.control_rate_hz:.0f} Hz, range {self.min_pwm}-{self.max_pwm}')

    def apply_deadband(self, x: float) -> float:
        """Apply deadband to joystick input"""
        if abs(x) < self.deadband:
            return 0.0
        # Rescale outside deadband
        return np.sign(x) * (abs(x) - self.deadband) / (1.0 - self.deadband)

    def shape_response(self, x: float) -> float:
        """Apply cubic shaping for fine control near center"""
        k = self.sensitivity
        return k * (x ** 3) + (1.0 - k) * x

    def map_to_pwm(self, x: float) -> int:
        """Map [-1,1] to PWM range"""
        span = float(max(self.max_pwm - 1500, 0))
        pwm = int(1500 + x * span)
        return int(np.clip(pwm, self.abs_min, self.abs_max))

    def ramp_pwm(self, current: int, target: int) -> int:
        """Gradually ramp from current to target PWM"""
        delta = target - current
        if abs(delta) <= self.max_delta_per_tick:
            return target
        return current + self.max_delta_per_tick * (1 if delta > 0 else -1)

    def control_loop(self):
        """High-rate control loop for smooth thruster ramping"""
        now = time()
        
        # Failsafe: neutralize if no joystick updates
        if (now - self.last_joy_time) > self.failsafe_timeout:
            for key in self.target_pwm.keys():
                self.target_pwm[key] = 1500

        # Ramp each axis toward target
        for key in self.current_pwm.keys():
            self.current_pwm[key] = self.ramp_pwm(self.current_pwm[key], self.target_pwm[key])

        # Publish RC override if armed and in manual mode
        if self.arming and self.set_mode[0]:
            self.setOverrideRCIN(
                channel_pitch=1500,
                channel_roll=1500,
                channel_throttle=self.current_pwm['vertical'],
                channel_yaw=self.current_pwm['yaw'],
                channel_forward=self.current_pwm['forward'],
                channel_lateral=self.current_pwm['lateral']
            )
        else:
            # Send neutral when disarmed
            self.setOverrideRCIN(1500, 1500, 1500, 1500, 1500, 1500)

    def continuous_control_callback(self):
        """Timer callback for continuous camera tilt and light control"""
        changed_tilt = False
        changed_light = False
        
        if self.lb_held:
            self.tilt = min(self.tilt + self.tilt_step, self.tilt_max)
            changed_tilt = True
        elif self.rb_held:
            self.tilt = max(self.tilt - self.tilt_step, self.tilt_min)
            changed_tilt = True
        
        if self.dpad_right_held:
            self.light = min(self.light + self.light_step, self.light_max)
            changed_light = True
        elif self.dpad_left_held:
            self.light = max(self.light - self.light_step, self.light_min)
            changed_light = True
        
        if changed_tilt:
            self.send_camera_tilt_command(self.tilt)
        if changed_light:
            self.send_servo_command(self.light_pin, self.light)

    def send_servo_command(self, pin_number, value):
        """Send servo command via MAVROS"""
        client = self.create_client(CommandLong, 'cmd/command')
        
        if not client.wait_for_service(timeout_sec=1.0):
            return
        
        request = CommandLong.Request()
        request.command = 183
        request.param1 = pin_number
        request.param2 = value
        request.param3 = 0.0
        request.param4 = 0.0

        client.call_async(request)

    def armDisarm(self, armed):
        """Arm or disarm vehicle"""
        cli = self.create_client(CommandLong, 'cmd/command')
        result = False
        while not result:
            result = cli.wait_for_service(timeout_sec=4.0)
        
        req = CommandLong.Request()
        req.broadcast = False
        req.command = 400
        req.confirmation = 0
        req.param1 = 1.0 if armed else 0.0
        req.param2 = 0.0
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0
        
        cli.call_async(req)
        self.get_logger().info(f"{'Armed' if armed else 'Disarmed'}")

    def send_camera_tilt_command(self, tilt_angle_deg):
        """Publish camera tilt angle"""
        msg = MountControl()
        msg.mode = 2
        msg.pitch = tilt_angle_deg
        msg.roll = 0.0
        msg.yaw = 0.0
        msg.altitude = 0.0
        msg.latitude = 0.0
        msg.longitude = 0.0

        self.mount_pub.publish(msg)

    def joyCallback(self, data):
        """Process joystick input"""
        # Update timestamp for failsafe
        self.last_joy_time = time()
        
        # Button mapping
        btn_arm = data.buttons[7]
        btn_disarm = data.buttons[6]
        btn_manual_mode = data.buttons[3]
        btn_automatic_mode = data.buttons[2]
        btn_corrected_mode = data.buttons[0]
        btn_camera_servo_up = data.buttons[4]
        btn_camera_servo_down = data.buttons[5]
        btn_camera_rest = data.buttons[9]

        btn_gripper_open = data.axes[2]
        btn_gripper_close = data.axes[5]
        btn_light = data.axes[6]

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

        # === SMOOTH THRUSTER CONTROL ===
        # Read joystick axes
        raw_forward = -float(data.axes[1])   # Left stick Y
        raw_lateral = -float(data.axes[0])   # Left stick X
        raw_yaw = -float(data.axes[3])       # Right stick X
        raw_vertical = float(data.axes[4])   # Right stick Y

        # Apply deadband and shaping
        def process_axis(val):
            return self.shape_response(self.apply_deadband(val))

        forward_shaped = process_axis(raw_forward)
        lateral_shaped = process_axis(raw_lateral)
        yaw_shaped = process_axis(raw_yaw)
        vertical_shaped = process_axis(raw_vertical)

        # Update target PWM (will be ramped in control_loop)
        self.target_pwm['forward'] = self.map_to_pwm(forward_shaped)
        self.target_pwm['lateral'] = self.map_to_pwm(lateral_shaped)
        self.target_pwm['yaw'] = self.map_to_pwm(yaw_shaped)
        self.target_pwm['vertical'] = self.map_to_pwm(vertical_shaped)

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


def main(args=None):
    rclpy.init(args=args)
    node = BlueROVJoystick()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

