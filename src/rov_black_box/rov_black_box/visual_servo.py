#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.qos import QoSProfile
import time

class VisualServoApproachBox(Node):
    """
    APPROACHES forward until the bbox height reaches desired_bbox_h_stop.

    Expects PoseStamped where:
      pose.position.x -> bbox center x (px)
      pose.position.y -> bbox center y (px)
      pose.position.z -> bbox height (px)
    """

    def __init__(self):
        super().__init__('visual_servo_approach_box')

        # --- Parameters (tune these) ---
        self.declare_parameter('box_pose_topic', 'box_pose')
        self.declare_parameter('autonomy_topic', '/autonomy_toggle')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # Camera / image size (pixels)
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)

        # stopping condition: when bbox height (px) >= desired_bbox_h_stop we consider "close"
        self.declare_parameter('desired_bbox_h_stop', 220.0)
        # hysteresis: when >= stop, we won't re-approach until bbox_h drops below backoff_px
        self.declare_parameter('desired_bbox_h_backoff', 200.0)

        # gains 
        self.declare_parameter('kp_yaw', 1.0)
        self.declare_parameter('kp_forward', 0.45)
        self.declare_parameter('kp_depth', 0.6)   # correct vertical image offset -> linear.z
        self.declare_parameter('kp_lateral', 0.18) # small lateral correction (optional)

        # limits
        self.declare_parameter('max_lin', 0.30)   # m/s
        self.declare_parameter('max_ang', 0.5)    # rad/s

        # centering deadzone (normalized; 0.0..0.5)
        self.declare_parameter('center_deadzone', 0.04)  # ~4% of image dims

        # detection timeout
        self.declare_parameter('detection_timeout', 1.0)  # seconds

        # smoothing on bbox height to avoid jitter
        self.declare_parameter('h_alpha', 0.4)

        # enable per-loop log for tuning (set false if too chatty)
        self.declare_parameter('publish_log', True)

        # --- Read parameters ---
        p = self
        self.box_pose_topic = p.get_parameter('box_pose_topic').get_parameter_value().string_value
        self.autonomy_topic = p.get_parameter('autonomy_topic').get_parameter_value().string_value
        self.cmd_vel_topic = p.get_parameter('cmd_vel_topic').get_parameter_value().string_value

        self.image_width = float(p.get_parameter('image_width').get_parameter_value().integer_value)
        self.image_height = float(p.get_parameter('image_height').get_parameter_value().integer_value)

        self.desired_bbox_h_stop = float(p.get_parameter('desired_bbox_h_stop').get_parameter_value().double_value)
        self.desired_bbox_h_backoff = float(p.get_parameter('desired_bbox_h_backoff').get_parameter_value().double_value)

        self.kp_yaw = float(p.get_parameter('kp_yaw').get_parameter_value().double_value)
        self.kp_forward = float(p.get_parameter('kp_forward').get_parameter_value().double_value)
        self.kp_depth = float(p.get_parameter('kp_depth').get_parameter_value().double_value)
        self.kp_lateral = float(p.get_parameter('kp_lateral').get_parameter_value().double_value)

        self.max_lin = float(p.get_parameter('max_lin').get_parameter_value().double_value)
        self.max_ang = float(p.get_parameter('max_ang').get_parameter_value().double_value)

        self.center_deadzone = float(p.get_parameter('center_deadzone').get_parameter_value().double_value)
        self.detection_timeout = float(p.get_parameter('detection_timeout').get_parameter_value().double_value)

        self.h_alpha = float(p.get_parameter('h_alpha').get_parameter_value().double_value)
        self.publish_log = bool(p.get_parameter('publish_log').get_parameter_value().bool_value)

        # --- state ---
        self.autonomy_active = False
        self.last_detection = None       # tuple (cx_px, cy_px, bbox_h_px_smoothed)
        self._raw_last_h = None
        self.last_detection_time = 0.0
        self.in_stop_zone = False        # true when we've reached desired_bbox_h_stop

        # --- ROS interface ---
        qos = QoSProfile(depth=5)
        self.create_subscription(Bool, self.autonomy_topic, self.autonomy_cb, qos)
        self.create_subscription(PoseStamped, self.box_pose_topic, self.box_pose_cb, qos)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # timer
        self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.get_logger().info(f'VisualServoApproachBox started. Listening to "{self.box_pose_topic}".')

    # --- callbacks ---
    def autonomy_cb(self, msg: Bool):
        if msg.data and not self.autonomy_active:
            self.autonomy_active = True
            self.get_logger().info('Autonomy ENGAGED')
        elif not msg.data and self.autonomy_active:
            self.autonomy_active = False
            self.get_logger().info('Autonomy DISENGAGED -> publishing zero cmd')
            self.publish_zero()

    def box_pose_cb(self, msg: PoseStamped):
        try:
            cx = float(msg.pose.position.x)
            cy = float(msg.pose.position.y)
            raw_h = float(msg.pose.position.z)
            # exponential smoothing on bbox height
            if self._raw_last_h is None:
                h_s = raw_h
            else:
                h_s = self.h_alpha * raw_h + (1.0 - self.h_alpha) * self._raw_last_h
            self._raw_last_h = h_s
            self.last_detection = (cx, cy, h_s)
            self.last_detection_time = time.time()
        except Exception as e:
            self.get_logger().warn(f'Bad box_pose message: {e}')

    # --- control loop ---
    def control_loop(self):
        if not self.autonomy_active:
            return

        # detection timeout -> stop
        if self.last_detection is None or (time.time() - self.last_detection_time) > self.detection_timeout:
            self.get_logger().warn_once('No recent detection -> stopping')
            self.publish_zero()
            return

        cx_px, cy_px, bbox_h_px = self.last_detection

        # normalized coords 0..1
        cx_n = max(0.0, min(1.0, cx_px / max(1.0, self.image_width)))
        cy_n = max(0.0, min(1.0, cy_px / max(1.0, self.image_height)))

        # errors relative to image center (center = 0)
        ex = cx_n - 0.5   # horizontal: -left, +right
        ey = cy_n - 0.5   # vertical: -up, +down

        # check stop/hysteresis using bbox height
        if not self.in_stop_zone and bbox_h_px >= self.desired_bbox_h_stop:
            self.in_stop_zone = True
            if self.publish_log:
                self.get_logger().info(f'Reached stop zone: bbox_h={bbox_h_px:.1f} >= {self.desired_bbox_h_stop}')
        elif self.in_stop_zone and bbox_h_px < self.desired_bbox_h_backoff:
            self.in_stop_zone = False
            if self.publish_log:
                self.get_logger().info(f'Back off from stop zone: bbox_h={bbox_h_px:.1f} < {self.desired_bbox_h_backoff}')

        # YAW to reduce horizontal error
        yaw_cmd = - self.kp_yaw * ex

        # small lateral correction (optional) to help centering if your vehicle supports it
        lateral_cmd = - self.kp_lateral * ex

        # depth correction from vertical error (image y increases downwards)
        depth_cmd = self.kp_depth * ey

        # forward approach: only if roughly centered and not in stop zone
        center_ok = (abs(ex) < self.center_deadzone) and (abs(ey) < self.center_deadzone)
        if center_ok and not self.in_stop_zone:
            # forward command proportional to deficit in bbox height
            # use fraction of desired bbox height
            forward_frac = (self.desired_bbox_h_stop - bbox_h_px) / max(1.0, self.desired_bbox_h_stop)
            forward_cmd = self.kp_forward * forward_frac
            # clamp + ensure sign: positive = forward if bbox smaller than desired
            forward_cmd = max(-self.max_lin, min(self.max_lin, forward_cmd))
        else:
            forward_cmd = 0.0

        # if in stop zone, zero forward; optionally keep small centering corrections active
        if self.in_stop_zone:
            forward_cmd = 0.0

        # clamp other commands
        lateral_cmd = max(-self.max_lin, min(self.max_lin, lateral_cmd))
        depth_cmd = max(-self.max_lin, min(self.max_lin, depth_cmd))
        yaw_cmd = max(-self.max_ang, min(self.max_ang, yaw_cmd))

        # build and publish Twist (linear.x = forward, linear.y = lateral, linear.z = heave, angular.z = yaw)
        t = Twist()
        t.linear.x = float(forward_cmd)
        t.linear.y = float(lateral_cmd)
        t.linear.z = float(depth_cmd)
        t.angular.z = float(yaw_cmd)
        self.cmd_pub.publish(t)

        if self.publish_log:
            self.get_logger().debug(
                f'ex={ex:.4f} ey={ey:.4f} bbox_h={bbox_h_px:.1f} | forward={forward_cmd:.3f} lat={lateral_cmd:.3f} '
                f'depth={depth_cmd:.3f} yaw={yaw_cmd:.3f}'
            )

    def publish_zero(self):
        t = Twist()
        self.cmd_pub.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoApproachBox()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
