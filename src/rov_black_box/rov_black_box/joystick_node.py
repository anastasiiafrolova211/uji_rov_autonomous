#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import OverrideRCIn, MountControl
import numpy as np

class BlueROVJoystick(Node):
    def __init__(self):
        super().__init__('bluerov_joystick_node')
        self.get_logger().info('Starting BlueROV joystick node (combined)')

        # Publishers
        self.override_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.mount_pub = self.create_publisher(MountControl, '/bluerov2/mount_control/command', 10)

        # QoS for subscriptions (joystick is best-effort)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribers
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, qos_profile)
        self.cmdvel_sub = self.create_subscription(Twist, 'cmd_vel', self.vel_callback, qos_profile)

        # Timer for continuous controls and mode handling (20 Hz)
        self.timer = self.create_timer(0.05, self.continuous_control_callback)

        # Button held states for continuous control
        self.lb_held = False
        self.rb_held = False
        self.dpad_left_held = False
        self.dpad_right_held = False

        # Servo + actuation params
        self.light_pin = 11.0
        self.gripper_pin = 13.0
        self.camera_servo_pin = 16.0

        self.light = 1100.0
        self.light_min = 1100.0
        self.light_max = 1900.0
        self.light_step = 15.0

        self.tilt = 0.0
        self.tilt_int = 0.0
        self.tilt_step = 2.0
        self.tilt_min = -60.0
        self.tilt_max = 60.0

        self.gripper = 1150.0
        self.gripper_min = 1150.0
        self.gripper_max = 1580.0
        self.gripper_step = 430.0

        self.rt_was_pressed = False
        self.lt_was_pressed = False

        # PWM configs and scaling for axes -> pwm mapping
        self.PWM_CENTER = 1500
        self.PWM_MIN = 1100
        self.PWM_MAX = 1900
        self.THRUSTER_SAFE_MIN = 1300
        self.THRUSTER_SAFE_MAX = 1700
        self.axis_deadzone = 0.05
        self.axis_scale = 0.4  # aggressive scaling

        # Modes: [manual, automatic, correction]
        self.set_mode = [True, False, False]
        self.arming = False
        self.correction_mode = False

        self.Correction_depth = self.PWM_CENTER
        self.Correction_yaw = self.PWM_CENTER
        self.Correction_surge = self.PWM_CENTER

        self.latest_cmd_vel = Twist()
        self.have_cmd_vel = False

    def joy_callback(self, data: Joy):
        btn_arm = data.buttons[7]
        btn_disarm = data.buttons[6]
        btn_manual_mode = data.buttons[3]
        btn_automatic_mode = data.buttons[2]
        btn_corrected_mode = data.buttons[0]
        btn_camera_servo_up = data.buttons[4]
        btn_camera_servo_down = data.buttons[5]
        btn_camera_rest = data.buttons[9]

        btn_gripper_open_axis = data.axes[2]
        btn_gripper_close_axis = data.axes[5]

        dpad_h = data.axes[6]

        self.lb_held = btn_camera_servo_up == 1
        self.rb_held = btn_camera_servo_down == 1
        self.dpad_left_held = dpad_h == 1.0
        self.dpad_right_held = dpad_h == -1.0

        if btn_disarm == 1 and self.arming:
            self.get_logger().info("Disarm requested.")
            self.arming = False
            self.arm_disarm(False)
        if btn_arm == 1 and not self.arming:
            self.get_logger().info("Arm requested.")
            self.arming = True
            self.arm_disarm(True)

        if btn_manual_mode and not self.set_mode[0]:
            self.set_mode = [True, False, False]
            self.get_logger().info("Switched to MANUAL mode.")
            self.correction_mode = False
        elif btn_automatic_mode and not self.set_mode[1]:
            self.set_mode = [False, True, False]
            self.get_logger().info("Switched to AUTOMATIC mode.")
            self.correction_mode = False
        elif btn_corrected_mode and not self.set_mode[2]:
            self.set_mode = [False, False, True]
            self.get_logger().info("Switched to CORRECTION mode.")
            self.correction_mode = False

        if btn_camera_rest == 1:
            self.tilt = 0.0
            self.send_camera_tilt_command(self.tilt)
            self.get_logger().info("Camera tilt reset.")

        rt_pressed = btn_gripper_close_axis < -0.5
        lt_pressed = btn_gripper_open_axis > 0.5
        if rt_pressed and not self.rt_was_pressed and self.gripper < self.gripper_max:
            self.gripper = min(self.gripper + self.gripper_step, self.gripper_max)
            self.send_servo_command(self.gripper_pin, self.gripper)
            self.get_logger().info(f"Gripper closing PWM: {self.gripper}")
        if lt_pressed and not self.lt_was_pressed and self.gripper > self.gripper_min:
            self.gripper = max(self.gripper - self.gripper_step, self.gripper_min)
            self.send_servo_command(self.gripper_pin, self.gripper)
            self.get_logger().info(f"Gripper opening PWM: {self.gripper}")

        self.rt_was_pressed = rt_pressed
        self.lt_was_pressed = lt_pressed

        if self.set_mode[0]:
            surge_pwm = self.mapValueScalSat(-data.axes[1])
            lateral_pwm = self.mapValueScalSat(-data.axes[0])
            heave_pwm = self.mapValueScalSat(-data.axes[4])
            yaw_pwm = self.mapValueScalSat(data.axes[3])
            pitch_pwm = self.PWM_CENTER
            roll_pwm = self.PWM_CENTER
            self.setOverrideRCIN(pitch_pwm, roll_pwm, heave_pwm, yaw_pwm, surge_pwm, lateral_pwm)

    def vel_callback(self, cmd_vel: Twist):
        self.latest_cmd_vel = cmd_vel
        self.have_cmd_vel = True

    def continuous_control_callback(self):
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

        if self.set_mode[0]:
            return

        if self.set_mode[1]:
            if not self.have_cmd_vel:
                self.setOverrideRCIN(self.PWM_CENTER, self.PWM_CENTER, self.PWM_CENTER,
                                     self.PWM_CENTER, self.PWM_CENTER, self.PWM_CENTER)
                return

            surge_pwm = self.mapValueScalSat(self.latest_cmd_vel.linear.x)
            lateral_pwm = self.mapValueScalSat(self.latest_cmd_vel.linear.y)
            heave_pwm = self.mapValueScalSat(self.latest_cmd_vel.linear.z)
            yaw_pwm = self.mapValueScalSat(self.latest_cmd_vel.angular.z)
            pitch_pwm = self.PWM_CENTER
            roll_pwm = self.PWM_CENTER
            self.setOverrideRCIN(pitch_pwm, roll_pwm, heave_pwm, yaw_pwm, surge_pwm, lateral_pwm)
            return

        if self.set_mode[2]:
            if not self.correction_mode:
                self.Correction_depth = self.PWM_CENTER
                self.Correction_yaw = self.PWM_CENTER
                self.Correction_surge = self.PWM_CENTER
                self.correction_mode = True
                self.get_logger().info("Correction mode initialized.")

            self.setOverrideRCIN(self.PWM_CENTER, self.PWM_CENTER,
                                 self.Correction_depth, self.Correction_yaw,
                                 self.Correction_surge, self.PWM_CENTER)
            return

    def send_servo_command(self, pin_number: float, value: float):
        client = self.create_client(CommandLong, '/mavros/cmd/command')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('MAVROS service not available!')
            return
        req = CommandLong.Request()
        req.command = 183
        req.param1 = float(pin_number)
        req.param2 = float(value)
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0
        client.call_async(req)
        self.get_logger().debug(f'Sent servo command pin {pin_number}, value {value}')

    def send_camera_tilt_command(self, tilt_angle_deg: float):
        msg = MountControl()
        msg.mode = 2
        msg.pitch = tilt_angle_deg
        msg.roll = 0.0
        msg.yaw = 0.0
        self.mount_pub.publish(msg)
        self.get_logger().debug(f"Camera tilt published: {tilt_angle_deg:.1f}°")

    def arm_disarm(self, armed: bool):
        cli = self.create_client(CommandLong, '/mavros/cmd/command')
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('MAVROS service not available!')
            return
        req = CommandLong.Request()
        req.command = 400
        req.param1 = 1.0 if armed else 0.0
        req.param2 = 0.0
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0
        fut = cli.call_async(req)
        fut.add_done_callback(lambda f: self.get_logger().info(f"{'Armed' if armed else 'Disarmed'} (service call returned)."))

    def setOverrideRCIN(self, channel_pitch, channel_roll, channel_throttle,
                        channel_yaw, channel_forward, channel_lateral):
        msg_override = OverrideRCIn()
        msg_override.channels = [int(1500)] * 18
        msg_override.channels[0] = int(self._clamp_pwm(channel_pitch))
        msg_override.channels[1] = int(self._clamp_pwm(channel_roll))
        msg_override.channels[2] = int(self._clamp_pwm(channel_throttle))
        msg_override.channels[3] = int(self._clamp_pwm(channel_yaw))
        msg_override.channels[4] = int(self._clamp_pwm(channel_forward))
        msg_override.channels[5] = int(self._clamp_pwm(channel_lateral))
        msg_override.channels[6] = int(1500)
        msg_override.channels[7] = int(1500)
        self.override_pub.publish(msg_override)

    def _clamp_pwm(self, pwm_value):
        try:
            pwm = int(pwm_value)
        except Exception:
            pwm = self.PWM_CENTER
        pwm = max(self.PWM_MIN, min(self.PWM_MAX, pwm))
        pwm = max(self.THRUSTER_SAFE_MIN, min(self.THRUSTER_SAFE_MAX, pwm))
        return pwm

    def mapValueScalSat(self, value: float):
        return self.axis_to_pwm(value, self.axis_scale)

    def axis_to_pwm(self, axis_val: float, scale: float):
        if abs(axis_val) < self.axis_deadzone:
            return self.PWM_CENTER
        pwm_signal = self.PWM_CENTER + int(axis_val * scale * (self.PWM_MAX - self.PWM_CENTER))
        pwm_signal = min(max(self.PWM_MIN, pwm_signal), self.PWM_MAX)
        return pwm_signal


def main(args=None):
    rclpy.init(args=args)
    node = BlueROVJoystick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

