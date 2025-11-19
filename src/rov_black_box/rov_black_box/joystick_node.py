#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from mavros_msgs.srv import CommandLong, SetMode
from mavros_msgs.msg import OverrideRCIn, MountControl, State


class BlueROVJoystick(Node):
    def __init__(self):
        super().__init__('bluerov_joystick_node')
        self.get_logger().info('Starting BlueROV joystick node (manual/auto + depth-hold + roll bumpers)')  # [web:388]

        # ----------------- Parameters -----------------
        self.declare_parameter('light_pin', 12.0)        # lights on pin 12 [web:388]
        self.declare_parameter('gripper_pin', 10.0)      # gripper on pin 10 [web:388]
        self.declare_parameter('camera_servo_pin', 16.0)

        self.light_pin = float(self.get_parameter('light_pin').value)
        self.gripper_pin = float(self.get_parameter('gripper_pin').value)
        self.camera_servo_pin = float(self.get_parameter('camera_servo_pin').value)

        # ----------------- Publishers -----------------
        self.override_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)  # [web:355]
        self.mount_pub = self.create_publisher(MountControl, '/mavros/mount_control/command', 10)  # [web:355]

        # ----------------- QoS & Subscribers -----------------
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )  # [web:706]

        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, qos_profile)  # [web:388]
        self.cmdvel_sub = self.create_subscription(Twist, 'cmd_vel', self.vel_callback, qos_profile)  # [web:706]
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)  # [web:355]

        # ----------------- Services -----------------
        self.cmd_client = self.create_client(CommandLong, '/mavros/cmd/command')  # [web:355]
        while not self.cmd_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for MAVROS CommandLong service...')  # [web:355]

        self.set_mode_cli = self.create_client(SetMode, '/mavros/set_mode')  # [web:355]
        while not self.set_mode_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for MAVROS SetMode service...')  # [web:355]

        # ----------------- Timers -----------------
        self.timer = self.create_timer(0.05, self.continuous_control_callback)  # [web:706]

        # ----------------- Button / control state -----------------
        self.dpad_left_held = False
        self.dpad_right_held = False
        self.dpad_up_held = False
        self.dpad_down_held = False

        # Lights PWM
        self.light = 1100.0
        self.light_min = 1100.0
        self.light_max = 1900.0
        self.light_step = 15.0

        # Camera tilt (deg)
        self.tilt = 0.0
        self.tilt_step = 2.0
        self.tilt_min = -60.0
        self.tilt_max = 60.0

        # Gripper PWM
        self.gripper = 1150.0
        self.gripper_min = 1150.0
        self.gripper_max = 1580.0
        self.gripper_step = 430.0

        self.rt_was_pressed = False
        self.lt_was_pressed = False

        # ----------------- PWM and scaling -----------------
        self.PWM_CENTER = 1500
        self.PWM_MIN = 1100
        self.PWM_MAX = 1900
        self.THRUSTER_SAFE_MIN = 1300
        self.THRUSTER_SAFE_MAX = 1700

        self.axis_deadzone = 0.15
        self.axis_expo = 0.5
        self.scale_surge = 0.40
        self.scale_lateral = 0.40
        self.scale_yaw = 0.30
        self.scale_heave = 0.30
        self.scale_roll = 0.35   # roll magnitude for bumper buttons [web:388]

        # Control modes: [manual, automatic]
        self.set_mode = [True, False]   # start in MANUAL [web:388]
        self.arming = False

        # Depth-hold state
        self.depth_hold = False
        self.prev_depth_hold_btn = 0
        self.depth_hold_settle_until = None
        self.depth_hold_settle_duration = 1.0  # seconds of neutral heave after ALT_HOLD [web:387]

        self.latest_cmd_vel = Twist()
        self.have_cmd_vel = False

        self.get_logger().info('BlueROV joystick initialized.')  # [web:387]

    # ----------------- MAVROS helpers -----------------

    def state_callback(self, msg: State):
        # For diagnostics if needed [web:355]
        pass

    def set_flight_mode(self, mode_str: str):
        if not self.set_mode_cli.service_is_ready():
            return  # [web:355]
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode_str
        self.set_mode_cli.call_async(req)  # [web:355]

    def send_neutral_override(self):
        msg_override = OverrideRCIn()
        msg_override.channels = [int(self.PWM_CENTER)] * 18
        self.override_pub.publish(msg_override)  # [web:355]

    # ----------------- Joystick callback -----------------

    def joy_callback(self, data: Joy):
        # Buttons (Xbox-style mapping) [web:388]
        btn_arm = data.buttons[7]
        btn_disarm = data.buttons[6]
        btn_manual_mode = data.buttons[3]
        btn_automatic_mode = data.buttons[2]
        btn_depth_hold_mode = data.buttons[0]
        btn_camera_rest = data.buttons[9]

        # New: bumpers for roll
        btn_roll_left = data.buttons[4]   # LB
        btn_roll_right = data.buttons[5]  # RB

        btn_gripper_open_axis = data.axes[2]   # LT [web:388]
        btn_gripper_close_axis = data.axes[5]  # RT [web:388]

        btn_light = data.axes[6]               # D-pad left/right [web:388]
        btn_camera_tilt = data.axes[7]         # D-pad up/down [web:388]

        self.dpad_left_held = btn_light == 1.0
        self.dpad_right_held = btn_light == -1.0
        self.dpad_up_held = btn_camera_tilt == 1.0
        self.dpad_down_held = btn_camera_tilt == -1.0

        # Arm / disarm
        if btn_disarm == 1 and self.arming:
            self.get_logger().info("Disarm requested.")
            self.arming = False
            self.arm_disarm(False)  # [web:387]
        if btn_arm == 1 and not self.arming:
            self.get_logger().info("Arm requested.")
            self.arming = True
            self.arm_disarm(True)   # [web:387]

        # Manual / automatic mapping
        if btn_manual_mode and not self.set_mode[0]:
            self.set_mode = [True, False]
            self.get_logger().info("Switched to MANUAL control (joystick → RC).")  # [web:388]
        elif btn_automatic_mode and not self.set_mode[1]:
            self.set_mode = [False, True]
            self.get_logger().info("Switched to AUTOMATIC control (cmd_vel → RC).")  # [web:366]

        # Depth-hold toggle (FCU MANUAL <-> ALT_HOLD)
        if btn_depth_hold_mode == 1 and self.prev_depth_hold_btn == 0:
            if not self.depth_hold:
                self.depth_hold = True
                self.set_flight_mode('ALT_HOLD')  # [web:387]
                self.depth_hold_settle_until = self.get_clock().now() + Duration(
                    seconds=self.depth_hold_settle_duration
                )
                self.get_logger().info("Depth Hold ON (ALT_HOLD), starting settle window.")  # [web:387]
            else:
                self.depth_hold = False
                self.set_flight_mode('MANUAL')  # [web:387]
                self.depth_hold_settle_until = None
                self.get_logger().info("Depth Hold OFF (MANUAL).")  # [web:387]
        self.prev_depth_hold_btn = btn_depth_hold_mode

        # Camera tilt reset
        if btn_camera_rest == 1:
            self.tilt = 0.0
            self.send_camera_tilt_command(self.tilt)
            self.get_logger().info("Camera tilt reset to 0 degrees")  # [web:388]

        # Gripper controls
        rt_pressed = btn_gripper_close_axis < -0.5
        lt_pressed = btn_gripper_open_axis > 0.5

        if rt_pressed and not self.rt_was_pressed and self.gripper < self.gripper_max:
            self.gripper = min(self.gripper + self.gripper_step, self.gripper_max)
            self.send_servo_command(self.gripper_pin, self.gripper)
            self.get_logger().info(f"Gripper closing PWM: {self.gripper}")  # [web:388]
        if lt_pressed and not self.lt_was_pressed and self.gripper > self.gripper_min:
            self.gripper = max(self.gripper - self.gripper_step, self.gripper_min)
            self.send_servo_command(self.gripper_pin, self.gripper)
            self.get_logger().info(f"Gripper opening PWM: {self.gripper}")  # [web:388]

        self.rt_was_pressed = rt_pressed
        self.lt_was_pressed = lt_pressed

        # MANUAL control: joystick → RC override
        if self.set_mode[0]:
            surge_pwm = self.mapValueScalSat(data.axes[1], self.scale_surge)
            lateral_pwm = self.mapValueScalSat(-data.axes[0], self.scale_lateral)
            heave_pwm = self._heave_pwm_from_axis(data.axes[4])

            # Roll: bumpers give digital ±1 "axis"
            roll_axis = 0.0
            if btn_roll_left:
                roll_axis += 1.0
            if btn_roll_right:
                roll_axis -= 1.0
            roll_pwm = self.axis_to_pwm(roll_axis, self.scale_roll, self.axis_expo)

            yaw_pwm = self.mapValueScalSat(-data.axes[3], self.scale_yaw)
            pitch_pwm = self.PWM_CENTER

            self.setOverrideRCIN(pitch_pwm, roll_pwm, heave_pwm, yaw_pwm, surge_pwm, lateral_pwm)

    # ----------------- cmd_vel + continuous control -----------------

    def vel_callback(self, cmd_vel: Twist):
        self.latest_cmd_vel = cmd_vel
        self.have_cmd_vel = True  # [web:706]

    def continuous_control_callback(self):
        # DPAD lights and camera tilt
        changed_tilt = False
        changed_light = False

        if self.dpad_up_held:
            self.tilt = min(self.tilt + self.tilt_step, self.tilt_max)
            changed_tilt = True
        elif self.dpad_down_held:
            self.tilt = max(self.tilt - self.tilt_step, self.tilt_min)
            changed_tilt = True

        if self.dpad_right_held:
            self.light = min(self.light + self.light_step, self.light_max)
            changed_light = True
        elif self.dpad_left_held:
            self.light = max(self.light - self.light_step, self.light_min)
            changed_light = True

        if changed_tilt:
            self.send_camera_tilt_command(self.tilt)  # [web:388]
        if changed_light:
            self.send_servo_command(self.light_pin, self.light)  # [web:388]

        # MANUAL handled in joy_callback
        if self.set_mode[0]:
            return

        # AUTOMATIC: cmd_vel → RC override (roll neutral)
        if self.set_mode[1]:
            if not self.have_cmd_vel:
                self.setOverrideRCIN(
                    self.PWM_CENTER, self.PWM_CENTER, self.PWM_CENTER,
                    self.PWM_CENTER, self.PWM_CENTER, self.PWM_CENTER
                )
                return

            surge_pwm = self.mapValueScalSat(
                self._clamp_unit(self.latest_cmd_vel.linear.x),
                self.scale_surge
            )
            lateral_pwm = self.mapValueScalSat(
                self._clamp_unit(self.latest_cmd_vel.linear.y),
                self.scale_lateral
            )
            heave_pwm = self._heave_pwm_from_axis(
                self._clamp_unit(self.latest_cmd_vel.linear.z)
            )
            yaw_pwm = self.mapValueScalSat(
                self._clamp_unit(self.latest_cmd_vel.angular.z),
                self.scale_yaw
            )

            pitch_pwm = self.PWM_CENTER
            roll_pwm = self.PWM_CENTER  # keep roll neutral in AUTO [web:387]
            self.setOverrideRCIN(pitch_pwm, roll_pwm, heave_pwm, yaw_pwm, surge_pwm, lateral_pwm)
            return

    # ----------------- Low-level helpers -----------------

    def _clamp_unit(self, v: float):
        return max(-1.0, min(1.0, float(v)))  # [web:707]

    def _heave_pwm_from_axis(self, axis_val: float):
        # Neutral heave during short settle window after entering ALT_HOLD [web:387]
        if self.depth_hold and self.depth_hold_settle_until is not None:
            now = self.get_clock().now()
            if now < self.depth_hold_settle_until:
                return self.PWM_CENTER
            else:
                self.depth_hold_settle_until = None
        return self.mapValueScalSat(axis_val, self.scale_heave)

    def send_servo_command(self, pin_number: float, value: float):
        # MAV_CMD_DO_SET_SERVO (183) [web:348]
        if not self.cmd_client.service_is_ready():
            return
        req = CommandLong.Request()
        req.broadcast = False
        req.command = 183
        req.confirmation = 0
        req.param1 = float(pin_number)
        req.param2 = float(value)
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0
        self.cmd_client.call_async(req)

    def send_camera_tilt_command(self, tilt_angle_deg: float):
        msg = MountControl()
        msg.mode = 2  # MAV_MOUNT_MODE_MAVLINK_TARGETING [web:348]
        msg.pitch = float(tilt_angle_deg)
        msg.roll = 0.0
        msg.yaw = 0.0
        msg.altitude = 0.0
        msg.latitude = 0.0
        msg.longitude = 0.0
        self.mount_pub.publish(msg)
        self.send_servo_command(self.camera_servo_pin, self.PWM_CENTER)

    def arm_disarm(self, armed: bool):
        # MAV_CMD_COMPONENT_ARM_DISARM (400) [web:348][web:614]
        if not self.cmd_client.service_is_ready():
            return
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
        self.cmd_client.call_async(req)

    def setOverrideRCIN(self, channel_pitch, channel_roll, channel_throttle,
                        channel_yaw, channel_forward, channel_lateral):
        msg_override = OverrideRCIn()
        msg_override.channels = [int(self.PWM_CENTER)] * 18
        msg_override.channels[0] = int(self._clamp_pwm(channel_pitch))
        msg_override.channels[1] = int(self._clamp_pwm(channel_roll))
        msg_override.channels[2] = int(self._clamp_pwm(channel_throttle))
        msg_override.channels[3] = int(self._clamp_pwm(channel_yaw))
        msg_override.channels[4] = int(self._clamp_pwm(channel_forward))
        msg_override.channels[5] = int(self._clamp_pwm(channel_lateral))
        msg_override.channels[6] = int(self.PWM_CENTER)
        msg_override.channels[7] = int(self.PWM_CENTER)
        self.override_pub.publish(msg_override)  # [web:355]

    def _clamp_pwm(self, pwm_value):
        try:
            pwm = int(pwm_value)
        except Exception:
            pwm = self.PWM_CENTER
        pwm = max(self.PWM_MIN, min(self.PWM_MAX, pwm))
        pwm = max(self.THRUSTER_SAFE_MIN, min(self.THRUSTER_SAFE_MAX, pwm))
        return pwm

    def mapValueScalSat(self, value: float, scale: float):
        return self.axis_to_pwm(value, scale, self.axis_expo)

    def axis_to_pwm(self, axis_val: float, scale: float, expo: float):
        if abs(axis_val) < self.axis_deadzone:
            return self.PWM_CENTER
        shaped = (1.0 - expo) * axis_val + expo * (axis_val ** 3)
        shaped = max(-1.0, min(1.0, shaped))
        pwm_signal = self.PWM_CENTER + int(shaped * scale * (self.PWM_MAX - self.PWM_CENTER))
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
        node.send_neutral_override()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

