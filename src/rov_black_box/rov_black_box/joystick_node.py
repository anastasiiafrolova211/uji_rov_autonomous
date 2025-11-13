#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from mavros_msgs.msg import MountControl, State
from mavros_msgs.srv import CommandLong
from pymavlink import mavutil
import time

class BlueROVJoystickDepthHold(Node):
    def __init__(self):
        super().__init__('bluerov_joystick_depth_hold_node')
        self.get_logger().info('Starting BlueROV joystick node with depth hold and camera tilt support')

        # Subscribe to MAVROS state (optional, for connection state etc.)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)

        # Subscribe to joystick messages
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 50)  # 50Hz

        # Publisher for camera tilt (MountControl)
        self.mount_pub = self.create_publisher(MountControl, '/mavros/mount_control/command', 10)

        # Service client for servo commands
        self.cmd_client = self.create_client(CommandLong, '/mavros/cmd/command')
        while not self.cmd_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for MAVROS CommandLong service...')

        # MAVLink connection to send manual control and position target messages
        self.master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
        self.master.wait_heartbeat()
        self.get_logger().info('Connected to MAVLink')

        # Set mode to ALT_HOLD for depth hold
        self.set_mode('ALT_HOLD')

        # Variables for depth hold control
        self.current_depth_target = 0.0
        self.prev_time = time.time()
        self.axis_deadzone = 0.05

        # Camera tilt params
        self.camera_servo_pin = 12
        self.tilt = 0.0
        self.tilt_step = 2.0
        self.tilt_min = -60.0
        self.tilt_max = 60.0

    def state_callback(self, msg: State):
        pass  # Can be used for connection monitoring

    def set_mode(self, mode_name: str):
        mode_id = self.master.mode_mapping().get(mode_name)
        if mode_id is None:
            self.get_logger().error(f'Mode {mode_name} not found')
            return
        self.master.set_mode(mode_id)
        self.get_logger().info(f'Set mode to {mode_name}')

    def joy_callback(self, msg: Joy):
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        # Read joystick axes (adjust indices according to your joystick)
        roll_input = msg.axes[0]    # Roll left/right
        pitch_input = msg.axes[1]   # Pitch forward/backward
        heave_input = msg.axes[2]   # Vertical up/down
        yaw_input = msg.axes[3]     # Yaw rotation

        # D-pad vertical for camera tilt (axes[7])
        dpad_up = msg.axes[7] == 1.0
        dpad_down = msg.axes[7] == -1.0

        # Update current depth target with heave stick input (if outside deadzone)
        if abs(heave_input) > self.axis_deadzone:
            depth_rate = 0.5  # meters per second depth change speed
            self.current_depth_target += -heave_input * depth_rate * dt
            self.send_depth_target(self.current_depth_target)

        # Send manual control (roll, pitch, yaw), vertical thrust set to middle (500) for depth hold
        roll_cmd = int(roll_input * 1000)
        pitch_cmd = int(pitch_input * 1000)
        yaw_cmd = int(yaw_input * 1000)

        self.master.mav.manual_control_send(
            self.master.target_system,
            pitch_cmd,
            roll_cmd,
            500,  # Vertical thrust mid-point as autopilot controls depth
            yaw_cmd,
            0  # buttons
        )

        # Camera tilt control
        changed_tilt = False
        if dpad_up:
            self.tilt = min(self.tilt + self.tilt_step, self.tilt_max)
            changed_tilt = True
        elif dpad_down:
            self.tilt = max(self.tilt - self.tilt_step, self.tilt_min)
            changed_tilt = True

        if changed_tilt:
            self.send_camera_tilt_command(self.tilt)

    def send_depth_target(self, depth_m):
        time_boot_ms = int((time.time() - self.master.time_boot_ms / 1000.0) * 1000)
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        ) & ~mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE

        self.master.mav.set_position_target_global_int_send(
            time_boot_ms,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            type_mask,
            0, 0,  # lat_int, lon_int unused here
            int(depth_m * 1e7),  # Depth converted to global int format (check calibration)
            0, 0, 0,  # velocity ignored
            0, 0, 0,  # acceleration ignored
            0, 0  # yaw, yaw_rate ignored
        )
        self.get_logger().info(f'Set depth target to {depth_m:.2f} meters')

    def send_camera_tilt_command(self, tilt_deg):
        # Publish MountControl
        msg = MountControl()
        msg.mode = 2  # MAV_MOUNT_MODE_MAVLINK_TARGETING
        msg.pitch = float(tilt_deg)
        msg.roll = 0.0
        msg.yaw = 0.0
        self.mount_pub.publish(msg)

        # Send servo override
        req = CommandLong.Request()
        req.command = 183  # MAV_CMD_DO_SET_SERVO
        req.param1 = float(self.camera_servo_pin)
        req.param2 = float(self.tilt_angle_to_pwm(tilt_deg))

        self.cmd_client.call_async(req)
        self.get_logger().info(f'Camera tilt set to {tilt_deg:.1f} deg')

    def tilt_angle_to_pwm(self, angle_deg):
        pwm_min = 1100
        pwm_max = 1900
        angle_range = self.tilt_max - self.tilt_min
        normalized = (angle_deg - self.tilt_min) / angle_range
        pwm = pwm_min + normalized * (pwm_max - pwm_min)
        return int(pwm)

def main(args=None):
    rclpy.init(args=args)
    node = BlueROVJoystickDepthHold()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

