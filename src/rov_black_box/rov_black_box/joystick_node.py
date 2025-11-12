#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import OverrideRCIn, MountControl, State

class BlueROVJoystick(Node):
    def __init__(self):
        super().__init__('bluerov_joystick_node')
        self.get_logger().info('Starting BlueROV joystick node with teleop twist support')

        # Publishers
        self.override_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.mount_pub = self.create_publisher(MountControl, '/mavros/mount_control/command', 10)

        # QoS for subscriptions
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribers
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, qos_profile)
        self.cmdvel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmdvel_callback, qos_profile)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)

        # Service client
        self.cmd_client = self.create_client(CommandLong, '/mavros/cmd/command')
        
        while not self.cmd_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for MAVROS CommandLong service...')
        
        # Timers
        self.timer = self.create_timer(0.05, self.continuous_control_callback)
        self.failsafe_timer = self.create_timer(0.1, self.failsafe_check)
        
        # Failsafe tracking
        self.last_joy_time = self.get_clock().now()
        self.last_state_time = self.get_clock().now()
        self.failsafe_timeout = 0.5
        self.mavros_timeout = 1.0
        self.failsafe_active = False
        self.mavros_connected = False

        # Button states
        self.dpad_left_held = False
        self.dpad_right_held = False
        self.dpad_up_held = False
        self.dpad_down_held = False

        # Camera servo pin
        self.camera_servo_pin = 12.0  # Confirmed working

        # Camera tilt params
        self.tilt = 0.0
        self.tilt_step = 2.0
        self.tilt_min = -60.0
        self.tilt_max = 60.0

        # Gripper state
        self.rt_was_pressed = False
        self.lt_was_pressed = False

        # PWM configs
        self.PWM_CENTER = 1500
        self.PWM_MIN = 1100
        self.PWM_MAX = 1900
        self.THRUSTER_SAFE_MIN = 1300
        self.THRUSTER_SAFE_MAX = 1700
        self.axis_deadzone = 0.05

        # Modes for manual/autonomous (manual mode sends RC override)
        self.set_mode = [True, False, False]  # manual, automatic, correction
        self.arming = False
        self.correction_mode = False

        # Correction variables (for correction mode)
        self.Correction_depth = self.PWM_CENTER
        self.Correction_yaw = self.PWM_CENTER
        self.Correction_surge = self.PWM_CENTER

        # Latest Twist from teleop_twist_joy (/cmd_vel)
        self.latest_cmd_vel = Twist()
        self.have_cmd_vel = False

        self.get_logger().info("BlueROV joystick initialized with teleop twist support")

    def state_callback(self, msg: State):
        self.last_state_time = self.get_clock().now()
        self.mavros_connected = msg.connected

    def failsafe_check(self):
        current_time = self.get_clock().now()
        time_since_joy = (current_time - self.last_joy_time).nanoseconds / 1e9
        time_since_state = (current_time - self.last_state_time).nanoseconds / 1e9
        
        joy_timeout = time_since_joy > self.failsafe_timeout
        mavros_timeout = time_since_state > self.mavros_timeout or not self.mavros_connected
        
        if joy_timeout or mavros_timeout:
            if not self.failsafe_active:
                if joy_timeout:
                    self.get_logger().error(f'FAILSAFE: No joystick input for {time_since_joy:.2f}s - STOPPING THRUSTERS')
                if mavros_timeout:
                    self.get_logger().error(f'FAILSAFE: MAVROS connection lost - STOPPING THRUSTERS')
                self.failsafe_active = True
            
            self.send_neutral_override()
        else:
            if self.failsafe_active:
                self.get_logger().info("FAILSAFE CLEARED: Joystick and MAVROS connection restored")
                self.failsafe_active = False

    def send_neutral_override(self):
        msg = OverrideRCIn()
        msg.channels = [int(1500)] * 18
        self.override_pub.publish(msg)
        
    def cmdvel_callback(self, cmd_vel: Twist):
        self.latest_cmd_vel = cmd_vel
        self.have_cmd_vel = True

    def joy_callback(self, data: Joy):
        self.last_joy_time = self.get_clock().now()

        # Buttons
        btn_arm = data.buttons[7]
        btn_disarm = data.buttons[6]
        btn_manual_mode = data.buttons[3]
        btn_automatic_mode = data.buttons[2]
        btn_corrected_mode = data.buttons[0]
        btn_reset_cam = data.buttons[9]

        btn_gripper_open_axis = data.axes[2]  # LT
        btn_gripper_close_axis = data.axes[5]  # RT

        btn_light_lr = data.axes[6]  # D-pad left/right
        btn_light_ud = data.axes[7]  # D-pad up/down for camera tilt

        self.dpad_left_held = btn_light_lr == 1.0
        self.dpad_right_held = btn_light_lr == -1.0
        self.dpad_up_held = btn_light_ud == 1.0
        self.dpad_down_held = btn_light_ud == -1.0

        # Arm/disarm handling
        if btn_disarm == 1 and self.arming:
            self.arming = False
            self.arm_disarm(False)
            self.get_logger().info("Disarm requested")
        if btn_arm == 1 and not self.arming:
            self.arming = True
            self.arm_disarm(True)
            self.get_logger().info("Arm requested")

        # Mode switching
        if btn_manual_mode and not self.set_mode[0]:
            self.set_mode = [True, False, False]
            self.correction_mode = False
            self.get_logger().info("Switched to MANUAL mode")
        elif btn_automatic_mode and not self.set_mode[1]:
            self.set_mode = [False, True, False]
            self.correction_mode = False
            self.get_logger().info("Switched to AUTOMATIC mode")
        elif btn_corrected_mode and not self.set_mode[2]:
            self.set_mode = [False, False, True]
            self.correction_mode = False
            self.get_logger().info("Switched to CORRECTION mode")

        # Camera reset tilt
        if btn_reset_cam == 1:
            self.tilt = 0.0
            self.send_camera_tilt_command(self.tilt)
            self.get_logger().info("Camera tilt reset to 0")

        # Gripper control on triggers (momentary)
        rt_pressed = btn_gripper_close_axis < -0.5
        lt_pressed = btn_gripper_open_axis > 0.5
        
        if rt_pressed and not self.rt_was_pressed:
            self.send_gripper_command(open_gripper=False)
        if lt_pressed and not self.lt_was_pressed:
            self.send_gripper_command(open_gripper=True)
            
        self.rt_was_pressed = rt_pressed
        self.lt_was_pressed = lt_pressed

        if self.failsafe_active:
            return

        if self.set_mode[0]:
            # Manual mode - send RC override
            surge_pwm = self.mapValueScalSat(data.axes[1])       # forward/back
            lateral_pwm = self.mapValueScalSat(-data.axes[0])    # left/right
            yaw_pwm = self.mapValueScalSat(-data.axes[3])        # rotation left/right
            
            # Use velocity input for heave if available (auto depth hold)
            if self.have_cmd_vel:
                heave_pwm = self.mapValueScalSat(self.latest_cmd_vel.linear.z)
            else:
                heave_pwm = self.mapValueScalSat(data.axes[4])  # fallback

            pitch_pwm = self.PWM_CENTER
            roll_pwm = self.PWM_CENTER

            self.setOverrideRCIN(pitch_pwm, roll_pwm, heave_pwm, yaw_pwm, surge_pwm, lateral_pwm)

    def continuous_control_callback(self):
        changed_tilt = False
        send_light_brighter = False
        send_light_dimmer = False

        if self.dpad_up_held:
            self.tilt = min(self.tilt + self.tilt_step, self.tilt_max)
            changed_tilt = True
        elif self.dpad_down_held:
            self.tilt = max(self.tilt - self.tilt_step, self.tilt_min)
            changed_tilt = True

        if self.dpad_right_held:
            send_light_brighter = True
        elif self.dpad_left_held:
            send_light_dimmer = True

        if changed_tilt:
            self.send_camera_tilt_command(self.tilt)
        if send_light_brighter:
            self.send_lights_command(brighter=True)
        if send_light_dimmer:
            self.send_lights_command(brighter=False)

    def send_lights_command(self, brighter=True):
        req = CommandLong.Request()
        req.command = 183
        req.param1 = 11.0  # Light pin (change if needed)
        req.param2 = 1900.0 if brighter else 1100.0
        self.cmd_client.call_async(req)
        self.get_logger().info(f'Lights command sent: {"brighter" if brighter else "dimmer"}')

    def send_gripper_command(self, open_gripper=True):
        req = CommandLong.Request()
        req.command = 183
        req.param1 = 13.0  # Gripper pin (change if needed)
        req.param2 = 1900.0 if open_gripper else 1100.0
        self.cmd_client.call_async(req)
        self.get_logger().info(f'Gripper command sent: {"open" if open_gripper else "close"}')

    def send_camera_tilt_command(self, tilt_angle_deg: float):
        msg = MountControl()
        msg.mode = 2
        msg.pitch = float(tilt_angle_deg)
        msg.roll = 0.0
        msg.yaw = 0.0
        msg.altitude = 0.0
        msg.latitude = 0.0
        msg.longitude = 0.0
        self.mount_pub.publish(msg)

        req = CommandLong.Request()
        req.command = 183
        req.param1 = self.camera_servo_pin
        req.param2 = 1500.0
        self.cmd_client.call_async(req)

    def arm_disarm(self, armed: bool):
        if not self.cmd_client.service_is_ready():
            return 
        req = CommandLong.Request()
        req.broadcast = False
        req.command = 400
        req.confirmation = 0
        req.param1 = 1.0 if armed else 0.0
        self.cmd_client.call_async(req)

    def setOverrideRCIN(self, pitch, roll, throttle, yaw, forward, lateral):
        msg = OverrideRCIn()
        msg.channels = [1500] * 18
        msg.channels[0] = int(self._clamp_pwm(pitch))
        msg.channels[1] = int(self._clamp_pwm(roll))
        msg.channels[2] = int(self._clamp_pwm(throttle))
        msg.channels[3] = int(self._clamp_pwm(yaw))
        msg.channels[4] = int(self._clamp_pwm(forward))
        msg.channels[5] = int(self._clamp_pwm(lateral))
        self.override_pub.publish(msg)

    def _clamp_pwm(self, pwm):
        try:
            pwm_i = int(pwm)
        except:
            pwm_i = self.PWM_CENTER
        pwm_i = max(self.PWM_MIN, min(self.PWM_MAX, pwm_i))
        pwm_i = max(self.THRUSTER_SAFE_MIN, min(self.THRUSTER_SAFE_MAX, pwm_i))
        return pwm_i

    def mapValueScalSat(self, value):
        if abs(value) < self.axis_deadzone:
            return self.PWM_CENTER
        pwm = int(self.PWM_CENTER + value * self.axis_scale * (self.PWM_MAX - self.PWM_CENTER))
        pwm = max(self.PWM_MIN, min(self.PWM_MAX, pwm))
        return pwm

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

