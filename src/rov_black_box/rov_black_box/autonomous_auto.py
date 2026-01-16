#!/usr/bin/env python3
"""
bluerov_smart_vision.py

Adds shift-left-click to set the aim crosshair (aim_x_offset).  Hold SHIFT and
left-click in the "Smart Vision Control" window — the cyan vertical line will
move to the clicked position and be used as the yaw target.

Other features:
 - target_depth = -4.54 m
 - deadman_mode = 'full' by default (hold RB to run; release pauses)
 - sway deadband + derivative filtering to reduce oscillation
 - YOLO guarded
 - HUD shows offset and deadman state
"""

import time
import math
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64, String
from cv_bridge import CvBridge

# YOLO guarded
try:
    from ultralytics import YOLO
except Exception:
    YOLO = None


class BlueROVSmartVision(Node):
    def __init__(self):
        super().__init__('bluerov_smart_vision')

        # --------------------------
        # Topics / model
        # --------------------------
        self.model_path = '/home/elex/uji_rov_autonomous/src/rov_black_box/detection_model/best.pt'
        self.camera_topic = '/bluerov2/camera/image_raw'

        # --------------------------
        # Deadman mode: 'full' or 'lunge'
        self.deadman_mode = 'full'        # default: hold RB to allow full mission
        self.deadman_button_idx = 5       # RB
        # --------------------------

        # --------------------------
        # Fixed mission parameters
        # --------------------------
        self.target_depth = -4.59          # fixed depth (m)

        # --------------------------
        # Tuning (conservative defaults)
        # --------------------------
        self.lunge_trigger_height = 340.0
        self.kp_surge = 0.002
        self.max_surge_speed = 0.15

        # Yaw PID
        self.kp_yaw = 0.00018
        self.kd_yaw = 0.0010
        self.yaw_deriv_alpha = 0.12
        self.yaw_deadband_center = 4.0
        self.yaw_fine_window = 15.0
        self.yaw_fine_scale = 0.35

        # Sway PID
        self.kp_sway = 0.0020
        self.ki_sway = 0.0003
        self.kd_sway = 0.006

        # Depth PID
        self.kp_depth_dive = 0.8
        self.kp_depth_hold = 0.25
        self.ki_depth = 0.02
        self.kd_depth = 0.5
        self.buoyancy_offset = -0.10

        self.lunge_ready_yaw_kp = 0.00022
        self.lunge_ready_yaw_kd = 0.0004

        
        self.required_stable_time = 1.0
        self.recovery_grace_period = 1.5
        self.recovery_duration = 2.0
        self.orbit_creep_speed = 0.05
        self.parallax_priority_thresh = 30.0
        self.orbit_gate_thresh = 30.0
        self.squaring_deadband = 15.0
        self.yaw_deadzone = 40
        self.holding_max_pwr = 0.05
        self.holding_noise_limit = 3.0
        self.min_sway_pwr = 0.08
        self.kick_deadband = 15.0

        self.lunge_duration = 2.5
        self.lunge_speed = 0.30

        self.smoothing_alpha_cmd = 0.25
        self.box_conf_thresh = 0.15

        self.ratio_min = 0.5
        self.ratio_max = 2.5
        self.tracking_radius = 150.0

        self.yaw_sign = -1.0
        self.sway_sign = 1.0
        self.heave_sign = 1.0
        self.surge_sign = 1.0

        # --------------------------
        # Sway damping to reduce oscillation
        # --------------------------
        self.sway_deadband = 8.0
        self.sway_hold_scale = 0.35
        self.sway_deriv_alpha = 0.25
        self.prev_sway_error = 0.0
        self.sway_deriv_filtered = 0.0

        # --------------------------
        # Aim offset (shift-click sets this)
        # --------------------------
        self.aim_x_offset = 0.0  # pixels; positive moves aim to the right

        # --------------------------
        # Internal state
        # --------------------------
        self.mission_state = "IDLE"
        self.auto_active = False

        self.deadman_pressed = False
        self._prev_deadman_pressed = False
        self.deadman_paused = False
        self._saved_mission_state = None

        self.current_depth = 0.0
        self.depth_vel = 0.0
        self.depth_integral = 0.0

        self.box_data = None
        self.handle_data = None
        self.last_box_center = None

        self.filt_box_cx = None
        self.filt_hdl_cx = None

        self.yaw_measurement_filtered = None
        self.prev_yaw_measurement = None

        self.prev_cmd = Twist()

        self.last_loop_time = time.time()
        self.last_depth_time = time.time()
        self.lunge_start_time = 0.0
        self.stable_start_time = None
        self.recovery_start_time = 0.0
        self.target_loss_time = None

        self.img_width = 640
        self.img_height = 480
        self.window_initialized = False

        # integrators
        self.sway_integral = 0.0

        # NEW: holding-phase flag (used to force yaw-only + prevent smoothing leak)
        self.in_holding_phase = False

        # --------------------------
        # Model load (guarded)
        # --------------------------
        self.get_logger().info(f"Attempting to load YOLO model from: {getattr(self, 'model_path', 'unset')}")
        if YOLO is not None:
            try:
                self.model = YOLO(self.model_path)
                self.get_logger().info("YOLO model loaded.")
            except Exception as e:
                self.get_logger().warning(f"Failed to load YOLO model: {e}")
                self.model = None
        else:
            self.get_logger().info("ultralytics.YOLO not available — running without detector.")
            self.model = None

        # cv bridge
        self.bridge = CvBridge()

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # subscriptions & publishers
        self.sub_img = self.create_subscription(Image, self.camera_topic, self.image_callback, 10)
        self.sub_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.sub_auto = self.create_subscription(Bool, '/rov/servo_mode_active', self.auto_callback, 10)
        self.sub_depth = self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.depth_callback, qos_profile)

        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/servoing/status', 10)
        self.pub_deadman = self.create_publisher(Bool, '/rov/deadman_pressed', 10)

        self.get_logger().info('Smart Vision Node ready (camera display restored).')

    # --------------------------
    # Deadman helpers
    # --------------------------
    def _on_deadman_release(self):
        self._saved_mission_state = self.mission_state
        self.mission_state = "PAUSED_BY_DEADMAN"
        self.deadman_paused = True
        try:
            self.pub_cmd.publish(self._zero_command())
        except Exception:
            pass
        self.get_logger().info("Deadman released: visual servoing paused. Hold RB to resume.")

    def _on_deadman_press(self):
        if self.deadman_paused:
            self.deadman_paused = False
            if self._saved_mission_state:
                self.mission_state = self._saved_mission_state
            else:
                self.mission_state = "IDLE"
            self._saved_mission_state = None
            self.get_logger().info("Deadman pressed: visual servoing resumed.")

    # --------------------------
    # Callbacks
    # --------------------------
    def auto_callback(self, msg: Bool):
        prev = self.auto_active
        self.auto_active = bool(msg.data)
        if self.auto_active and not prev:
            self.get_logger().info(f">>> AUTO ENGAGED (target depth locked at {self.target_depth:.2f} m) <<<")
            self.mission_state = "DIVING"
            self._reset_control_state()
        elif (not self.auto_active) and prev:
            self.get_logger().info("<<< MANUAL MODE >>>")
            self.mission_state = "IDLE"
            self._reset_control_state()
            try:
                self.pub_cmd.publish(self._zero_command())
            except Exception:
                pass

    def depth_callback(self, msg: Float64):
        now = time.time()
        raw_depth = float(msg.data)
        dt = now - getattr(self, 'last_depth_time', now)
        if dt > 1e-6:
            self.depth_vel = (raw_depth - self.current_depth) / dt
        self.current_depth = raw_depth
        self.last_depth_time = now

    def joy_callback(self, msg: Joy):
        try:
            cur = bool(msg.buttons[self.deadman_button_idx])
        except Exception:
            cur = False

        try:
            self.pub_deadman.publish(Bool(data=cur))
        except Exception:
            pass

        prev = getattr(self, '_prev_deadman_pressed', False)
        self._prev_deadman_pressed = cur
        self.deadman_pressed = cur

        if self.deadman_mode == 'full':
            if prev and (not cur):
                self._on_deadman_release()
            elif (not prev) and cur:
                self._on_deadman_press()

    # --------------------------
    # Mouse callback: shift-left-click sets aim_x_offset
    # --------------------------
    def mouse_callback(self, event, x, y, flags, param):
        # Only accept left-button down with SHIFT key held
        if event == cv2.EVENT_LBUTTONDOWN:
            # OpenCV uses flags bitmask for modifier keys
            if flags & cv2.EVENT_FLAG_SHIFTKEY:
                # compute offset relative to center
                center_x = float(self.img_width) / 2.0
                new_offset = float(x) - center_x
                self.aim_x_offset = new_offset
                self.get_logger().info(f"NEW AIM OFFSET SET: {self.aim_x_offset:.1f} px (shift-click)")

    # --------------------------
    # Helpers / controllers
    # --------------------------
    def _reset_control_state(self):
        self.filt_box_cx = None
        self.filt_hdl_cx = None
        self.last_box_center = None
        self.depth_integral = 0.0
        self.sway_integral = 0.0
        self.prev_sway_error = 0.0
        self.sway_deriv_filtered = 0.0
        self.yaw_measurement_filtered = None
        self.prev_yaw_measurement = None
        self.target_loss_time = None
        self.stable_start_time = None

    def _zero_command(self):
        return Twist()

    def apply_smoothing(self, raw_val, filt_val):
        if filt_val is None:
            return float(raw_val)
        return (self.smoothing_alpha_cmd * raw_val) + ((1.0 - self.smoothing_alpha_cmd) * filt_val)

    def compute_sway_pid(self, error, prev_error, dt):
        if abs(error) <= getattr(self, 'sway_deadband', 8.0):
            self.prev_sway_error = error
            self.sway_deriv_filtered = 0.0
            self.sway_integral *= 0.5
            return 0.0

        derivative_raw = (error - getattr(self, 'prev_sway_error', error)) / dt if dt > 0 else 0.0
        alpha = getattr(self, 'sway_deriv_alpha', 0.25)
        self.sway_deriv_filtered = alpha * derivative_raw + (1.0 - alpha) * getattr(self, 'sway_deriv_filtered', 0.0)
        self.prev_sway_error = error

        if abs(error) < 100.0 and dt > 0:
            self.sway_integral += error * dt
            limit = 0.15 / max(self.ki_sway, 1e-9)
            self.sway_integral = float(np.clip(self.sway_integral, -limit, limit))
        else:
            self.sway_integral = 0.0

        p_term = error * self.kp_sway
        i_term = self.sway_integral * self.ki_sway
        d_term = -self.sway_deriv_filtered * self.kd_sway

        out = p_term + i_term + d_term
        return float(out)

    def compute_yaw_pd_filtered(self, error, measurement, dt, kp_override=None, kd_override=None):
        kp = kp_override if kp_override is not None else self.kp_yaw
        kd = kd_override if kd_override is not None else self.kd_yaw

        if self.yaw_measurement_filtered is None:
            self.yaw_measurement_filtered = measurement
            self.prev_yaw_measurement = measurement

        self.yaw_measurement_filtered = (
            self.yaw_deriv_alpha * measurement +
            (1.0 - self.yaw_deriv_alpha) * self.yaw_measurement_filtered
        )

        measurement_rate = 0.0
        if dt > 1e-6:
            measurement_rate = (self.yaw_measurement_filtered - self.prev_yaw_measurement) / dt

        max_rate = 400.0
        measurement_rate = float(np.clip(measurement_rate, -max_rate, max_rate))
        self.prev_yaw_measurement = self.yaw_measurement_filtered

        p_term = error * kp
        d_term = -measurement_rate * kd
        return p_term + d_term

    def shape_yaw_for_fine_aim(self, err_yaw, raw_yaw_cmd):
        e = abs(err_yaw)
        if e <= self.yaw_deadband_center:
            return 0.0
        if e <= self.yaw_fine_window:
            span = self.yaw_fine_window - self.yaw_deadband_center
            frac = (e - self.yaw_deadband_center) / max(span, 1e-9)
            scale = self.yaw_fine_scale * frac
            return raw_yaw_cmd * scale
        return raw_yaw_cmd

    def apply_stiction_kick(self, raw_cmd, min_pwr, error_pixels):
        if abs(error_pixels) < self.kick_deadband:
            return raw_cmd
        if abs(raw_cmd) < 0.01:
            return 0.0
        if abs(raw_cmd) < min_pwr:
            return min_pwr if raw_cmd > 0 else -min_pwr
        return raw_cmd

    def get_depth_cmd(self, target, dt, use_integral=True, gentle=False):
        err = target - self.current_depth
        kp = self.kp_depth_hold if gentle else self.kp_depth_dive

        if use_integral and dt > 0 and abs(err) < 0.5:
            self.depth_integral += err * dt
            limit = 0.25 / max(self.ki_depth, 1e-9)
            self.depth_integral = float(np.clip(self.depth_integral, -limit, limit))
        else:
            self.depth_integral = 0.0

        p_term = err * kp
        i_term = self.depth_integral * self.ki_depth if use_integral else 0.0
        d_term = self.depth_vel * getattr(self, 'kd_depth', self.kd_depth)

        z_cmd = p_term + i_term - d_term
        if err <= 0:
            z_cmd += self.buoyancy_offset
        z_cmd *= self.heave_sign

        return float(np.clip(z_cmd, -0.6, 0.6))

    # --------------------------
    # Detections processing
    # --------------------------
    def process_detections(self, results):
        raw_boxes = []
        raw_handles = []
        if results and len(results) > 0 and getattr(results[0], 'boxes', None):
            for box in results[0].boxes:
                try:
                    coords = box.xyxy[0].cpu().numpy().astype(int)
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                except Exception:
                    continue

                if cls_id == 0 and conf < self.box_conf_thresh:
                    continue
                if cls_id == 1 and conf < 0.4:
                    continue

                cx = int((coords[0] + coords[2]) / 2)
                cy = int((coords[1] + coords[3]) / 2)
                width = coords[2] - coords[0]
                height = coords[3] - coords[1]

                det = {'coords': coords, 'center': (cx, cy),
                       'width': width, 'height': height, 'conf': conf}

                if cls_id == 0:
                    raw_boxes.append(det)
                elif cls_id == 1:
                    raw_handles.append(det)
        return raw_boxes, raw_handles

    def select_best_target(self, detections, img_center):
        if not detections:
            self.last_box_center = None
            return None

        valid = []
        for d in detections:
            h = max(1.0, float(d.get('height', 1)))
            w = float(d.get('width', 0))
            ratio = w / h
            if self.ratio_min < ratio < self.ratio_max:
                valid.append(d)

        if not valid:
            self.last_box_center = None
            return None

        if self.last_box_center is not None:
            lx, ly = self.last_box_center
            nearby = [d for d in valid if math.hypot(d['center'][0] - lx, d['center'][1] - ly) < self.tracking_radius]
            if nearby:
                best = max(nearby, key=lambda x: x.get('conf', 0.0))
                self.last_box_center = best['center']
                return best

        best = max(valid, key=lambda x: x.get('conf', 0.0))
        self.last_box_center = best['center']
        return best

    def select_best_handle(self, detections, box_center):
        if not detections or box_center is None:
            return None
        bx, by = box_center
        try:
            best = min(detections, key=lambda d: math.hypot(d['center'][0] - bx, d['center'][1] - by))
            return best
        except Exception:
            return None

    # --------------------------
    # Main image callback
    # --------------------------
    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        h, w, _ = frame.shape
        self.img_width = w
        self.img_height = h
        img_center = (w // 2, h // 2)
        # apply aim offset here:
        target_center_x = float(w // 2) + float(getattr(self, 'aim_x_offset', 0.0))

        if not self.window_initialized:
            try:
                cv2.namedWindow("Smart Vision Control", cv2.WINDOW_NORMAL)
                # set our bound mouse callback (mouse_callback uses self.img_width)
                cv2.setMouseCallback("Smart Vision Control", self.mouse_callback)
                self.window_initialized = True
            except Exception:
                pass

        # YOLO inference
        results = None
        if self.model is not None:
            try:
                results = self.model(frame, verbose=False)
            except Exception as e:
                self.get_logger().warning(f"YOLO inference error: {e}")
                results = None

        raw_boxes, raw_handles = self.process_detections(results)

        # select targets
        self.box_data = self.select_best_target(raw_boxes, img_center)
        box_center_ref = self.box_data['center'] if self.box_data else None
        self.handle_data = self.select_best_handle(raw_handles, box_center_ref)

        # filtering
        if self.box_data is not None:
            self.filt_box_cx = self.apply_smoothing(self.box_data['center'][0], self.filt_box_cx)
        else:
            self.filt_box_cx = None

        if self.handle_data is not None:
            self.filt_hdl_cx = self.apply_smoothing(self.handle_data['center'][0], self.filt_hdl_cx)
        else:
            self.filt_hdl_cx = None

        # timing
        current_time = time.time()
        dt_loop = max(0.001, current_time - self.last_loop_time)
        self.last_loop_time = current_time

        # reset holding flag each frame; states can set it True
        self.in_holding_phase = False

        # control
        cmd = self._zero_command()
        status_text = self.mission_state

        # pause when deadman released and mode == full
        if self.deadman_mode == 'full' and not self.deadman_pressed:
            if not self.deadman_paused:
                self._on_deadman_release()
            status_text = "DEADMAN RELEASED - HOLD BUTTON"
            try:
                self.pub_status.publish(String(data=status_text))
                self.pub_cmd.publish(self._zero_command())
            except Exception:
                pass
            self._render_visualization(frame, h, w, target_center_x, status_text, cmd)
            return

        # state machine
        if not self.auto_active:
            status_text = "MANUAL (Joy)"
        elif self.mission_state == "IDLE":
            pass
        elif self.mission_state == "DIVING":
            status_text = self.state_diving(cmd, target_center_x, dt_loop)
        elif self.mission_state == "SEARCH_HANDLE":
            status_text = self.state_search_handle(cmd, target_center_x, dt_loop)
        elif self.mission_state == "SQUARING":
            status_text = self.state_squaring(cmd, target_center_x, dt_loop)
        elif self.mission_state == "LUNGE_READY":
            status_text = self.state_lunge_ready(cmd, target_center_x, dt_loop, current_time)
        elif self.mission_state == "RECOVERY_BACKOFF":
            status_text = self.state_recovery_backoff(cmd, dt_loop, current_time)
        elif self.mission_state == "LUNGING":
            status_text = self.state_lunging(cmd, current_time)
        elif self.mission_state == "COMPLETE":
            status_text = "COMPLETE"

        # publish smoothed command
        if self.auto_active:
            alpha = float(getattr(self, 'smoothing_alpha_cmd', 0.25))
            smoothed = Twist()
            smoothed.linear.x = alpha * cmd.linear.x + (1.0 - alpha) * self.prev_cmd.linear.x
            smoothed.linear.y = alpha * cmd.linear.y + (1.0 - alpha) * self.prev_cmd.linear.y
            smoothed.linear.z = alpha * cmd.linear.z + (1.0 - alpha) * self.prev_cmd.linear.z
            smoothed.angular.x = alpha * cmd.angular.x + (1.0 - alpha) * self.prev_cmd.angular.x
            smoothed.angular.y = alpha * cmd.angular.y + (1.0 - alpha) * self.prev_cmd.angular.y
            smoothed.angular.z = alpha * cmd.angular.z + (1.0 - alpha) * self.prev_cmd.angular.z

            # NEW: If we're holding, enforce yaw-only and block smoothing "leak" of linear axes
            if getattr(self, 'in_holding_phase', False):
                smoothed.linear.x = 0.0
                smoothed.linear.y = 0.0
                smoothed.linear.z = 0.0  # (yaw-only means no depth hold during holding)

            # OPTIONAL (but requested behavior): during the lunge, force "straight on"
            # by preventing any leftover sway/yaw from smoothing.
            if self.mission_state == "LUNGING":
                smoothed.linear.y = 0.0
                smoothed.angular.z = 0.0

            try:
                self.pub_cmd.publish(smoothed)
                self.pub_status.publish(String(data=status_text))
                self.prev_cmd = smoothed
            except Exception:
                pass

        # visualization (this uses current aim_x_offset)
        self._render_visualization(frame, h, w, target_center_x, status_text, cmd)

    # --------------------------
    # State implementations
    # --------------------------
    def state_diving(self, cmd, target_center_x, dt_loop):
        depth_err = abs(self.target_depth - self.current_depth)
        cmd.linear.z = self.get_depth_cmd(self.target_depth, dt_loop, use_integral=False, gentle=False)
        if self.filt_box_cx is not None:
            err_x = target_center_x - self.filt_box_cx
            cmd.angular.z = float(np.clip(err_x * self.kp_yaw * self.yaw_sign, -0.4, 0.4))
            cmd.linear.x = self.surge_sign * min(self.max_surge_speed, 0.05)
        if depth_err < 0.08:
            self.mission_state = "SEARCH_HANDLE"
        return "DIVING"

    def state_search_handle(self, cmd, target_center_x, dt_loop):
        cmd.linear.z = self.get_depth_cmd(self.target_depth, dt_loop, use_integral=True, gentle=True)
        if self.handle_data is not None:
            self.mission_state = "SQUARING"
            self.stable_start_time = None
            return "SEARCH: FOUND HANDLE"
        elif self.box_data is not None:
            if self.filt_box_cx is not None:
                err_x = target_center_x - self.filt_box_cx
                cmd.angular.z = float(np.clip(err_x * self.kp_yaw * self.yaw_sign, -0.4, 0.4))
            cmd.linear.x = abs(0.12) * self.surge_sign
            return "SEARCH: CRAWLING..."
        else:
            return "SEARCH: LOST BOX"

    def state_squaring(self, cmd, target_center_x, dt_loop):
        if not (self.box_data is not None):
            cmd.linear.z = self.get_depth_cmd(self.target_depth, dt_loop, use_integral=False, gentle=True)
            return "LOST BOX"
        if not (self.handle_data is not None):
            self.mission_state = "SEARCH_HANDLE"
            return "LOST HANDLE"

        box_cx = self.filt_box_cx
        hdl_cx = self.filt_hdl_cx
        parallax_err = hdl_cx - box_cx
        is_squared = abs(parallax_err) < self.squaring_deadband
        err_yaw = target_center_x - hdl_cx

        suppress_yaw = abs(parallax_err) > self.parallax_priority_thresh

        raw_yaw = self.compute_yaw_pd_filtered(err_yaw, hdl_cx, dt_loop)
        if suppress_yaw:
            raw_yaw *= 0.3
        cmd.angular.z = float(np.clip(raw_yaw * self.yaw_sign, -0.20, 0.20))

        sway_target_err = err_yaw if is_squared else parallax_err
        raw_sway = self.compute_sway_pid(sway_target_err, 0.0, dt_loop)
        if is_squared:
            raw_sway *= getattr(self, 'sway_hold_scale', 0.35)
        cmd.linear.y = float(np.clip(raw_sway * self.sway_sign, -0.12, 0.12))

        status = "LOCKED" if is_squared else f"ORBITING (Err:{int(parallax_err)})"

        current_h = max(0.0, float(self.box_data.get('height', 0)))
        dist_err = self.lunge_trigger_height - current_h
        can_move_fwd = True

        if abs(parallax_err) > self.orbit_gate_thresh:
            can_move_fwd = False
            status += " [ALIGNING]"
            cmd.linear.x = self.orbit_creep_speed * self.surge_sign
        elif current_h > (self.lunge_trigger_height * 0.8) and not is_squared:
            can_move_fwd = False
            status += " [HALT]"

        if dist_err < 10:
            cmd.linear.x = 0.0
            self.mission_state = "LUNGE_READY"
            self.stable_start_time = None
            self.target_loss_time = None
        elif can_move_fwd:
            speed = dist_err * self.kp_surge
            cmd.linear.x = float(np.clip(speed, -0.10, self.max_surge_speed)) * self.surge_sign

        cmd.linear.z = self.get_depth_cmd(self.target_depth, dt_loop, use_integral=False, gentle=True)
        return status

    def state_lunge_ready(self, cmd, target_center_x, dt_loop, current_time):
        if not self._targets_present():
            if self.target_loss_time is None:
                self.target_loss_time = current_time
                status = "TARGETS LOST - GRACE PERIOD"
            else:
                grace_elapsed = current_time - self.target_loss_time
                if grace_elapsed >= self.recovery_grace_period:
                    self.mission_state = "RECOVERY_BACKOFF"
                    self.recovery_start_time = current_time
                    self.stable_start_time = None
                    self.target_loss_time = None
                    return "RECOVERY TRIGGERED"
                else:
                    status = f"GRACE: {self.recovery_grace_period - grace_elapsed:.2f}s"
            cmd.linear.z = self.get_depth_cmd(self.target_depth, dt_loop, use_integral=False, gentle=True)
            return status

        self.target_loss_time = None

        hdl_cx = self.filt_hdl_cx
        box_cx = self.filt_box_cx
        parallax_err = hdl_cx - box_cx
        err_yaw = target_center_x - hdl_cx

        raw_yaw = self.compute_yaw_pd_filtered(
            err_yaw, hdl_cx, dt_loop,
            kp_override=self.lunge_ready_yaw_kp,
            kd_override=self.lunge_ready_yaw_kd
        )
        yaw_cmd = float(np.clip(raw_yaw * self.yaw_sign, -0.15, 0.15))
        yaw_cmd = self.shape_yaw_for_fine_aim(err_yaw, yaw_cmd)
        cmd.angular.z = self.apply_stiction_kick(yaw_cmd, 0.06, err_yaw)

        raw_sway = self.compute_sway_pid(parallax_err, 0.0, dt_loop)
        cmd.linear.y = self.apply_stiction_kick(
            float(np.clip(raw_sway * self.sway_sign, -0.3, 0.3)),
            self.min_sway_pwr,
            parallax_err
        )

        dist_err = self.lunge_trigger_height - self.box_data['height']
        cmd.linear.x = float(np.clip(dist_err * self.kp_surge, -0.1, 0.1)) * self.surge_sign

        is_aligned = (abs(parallax_err) < self.squaring_deadband) and (abs(err_yaw) < self.yaw_deadzone)

        if is_aligned:
            if self.stable_start_time is None:
                self.stable_start_time = current_time
                status = "STABILIZING..."
            else:
                elapsed_stable = current_time - self.stable_start_time
                status = f"HOLDING... {elapsed_stable:.1f}s"

                # NEW: HOLDING = yaw-only (no surge/sway/heave at all)
                self.in_holding_phase = True

                # Optionally keep yaw gentle + deadbanded
                cmd.angular.z = float(np.clip(cmd.angular.z, -self.holding_max_pwr, self.holding_max_pwr))
                if abs(cmd.angular.z) < 0.01:
                    cmd.angular.z = 0.0

                # Force all linear axes to zero during holding
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.linear.z = 0.0

                if elapsed_stable >= self.required_stable_time:
                    if self.deadman_mode in ('lunge', 'hybrid') and not self.deadman_pressed:
                        status = "READY TO LUNGE - HOLD BUTTON"
                    else:
                        self.mission_state = "LUNGING"
                        self.lunge_start_time = current_time
        else:
            self.stable_start_time = None
            status = f"ALIGNING: P={int(parallax_err)} Y={int(err_yaw)}"

        # Depth hold only when NOT in holding (so holding is truly zero except yaw)
        if not self.in_holding_phase:
            cmd.linear.z = self.get_depth_cmd(self.target_depth, dt_loop, use_integral=False, gentle=True)

        return status

    def state_recovery_backoff(self, cmd, dt_loop, current_time):
        elapsed = current_time - self.recovery_start_time
        if elapsed < self.recovery_duration:
            cmd.linear.x = -0.15 * self.surge_sign
            cmd.linear.z = self.get_depth_cmd(self.target_depth, dt_loop, use_integral=False, gentle=True)
            return f"RECOVERING... ({elapsed:.1f}s)"
        else:
            self.mission_state = "SEARCH_HANDLE"
            self.target_loss_time = None
            return "RECOVERY COMPLETE"

    def state_lunging(self, cmd, current_time):
        elapsed = current_time - self.lunge_start_time
        if elapsed < self.lunge_duration:
            # NEW: lunge straight: surge only (no sway, no yaw)
            cmd.linear.x = self.lunge_speed * self.surge_sign
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0
            cmd.angular.z = 0.0
            return f"LUNGING {elapsed:.1f}s"
        else:
            self.mission_state = "COMPLETE"
            return "LUNGE COMPLETE"

    def _targets_present(self):
        return (self.box_data is not None) and (self.handle_data is not None)

    # --------------------------
    # Visualization HUD (uses aim_x_offset)
    # --------------------------
    def _render_visualization(self, frame, h, w, target_center_x, status_text, cmd):
        viz = frame.copy()
        # reference lines (image center and aim)
        cv2.line(viz, (w//2, 0), (w//2, h), (200, 200, 200), 1)  # image center
        aim_x_int = int(target_center_x)
        cv2.line(viz, (aim_x_int, 0), (aim_x_int, h), (255, 255, 0), 2)  # cyan/yellow aim line

        # draw detections
        try:
            if self.box_data:
                x1,y1,x2,y2 = self.box_data['coords']
                cv2.rectangle(viz, (x1,y1),(x2,y2),(0,128,255),2)
                cx,cy = self.box_data['center']
                cv2.circle(viz, (int(cx), int(cy)), 6, (0,128,255), -1)
                cv2.putText(viz, "BOX", (x1, y1-6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,128,255),2)
            if self.handle_data:
                x1,y1,x2,y2 = self.handle_data['coords']
                cv2.rectangle(viz, (x1,y1),(x2,y2),(0,220,0),2)
                cx,cy = self.handle_data['center']
                cv2.circle(viz, (int(cx), int(cy)), 6, (0,220,0), -1)
                cv2.putText(viz, "HDL", (x1, y1-6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,220,0),2)
        except Exception:
            pass

        # HUD
        hud_w = 360
        hud_h = 140
        hud_x = max(10, w - hud_w - 10)
        hud_y = 10
        cv2.rectangle(viz, (hud_x, hud_y), (hud_x + hud_w, hud_y + hud_h), (0,0,0), -1)

        cv2.putText(viz, f"State: {status_text}", (hud_x+10, hud_y+24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,220,255), 1)
        cv2.putText(viz, f"Depth: {self.current_depth:.2f} m (Tgt {self.target_depth:.2f} m)", (hud_x+10, hud_y+48), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200,220,255), 1)

        try:
            yaw_v = getattr(cmd, 'angular', None).z if getattr(cmd, 'angular', None) else 0.0
            sway_v = getattr(cmd, 'linear', None).y if getattr(cmd, 'linear', None) else 0.0
            surge_v = getattr(cmd, 'linear', None).x if getattr(cmd, 'linear', None) else 0.0
        except Exception:
            yaw_v = sway_v = surge_v = 0.0

        cv2.putText(viz, f"Yaw cmd: {yaw_v:+.3f}", (hud_x+10, hud_y+72), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200,220,255), 1)
        cv2.putText(viz, f"Sway cmd:{sway_v:+.3f}", (hud_x+10, hud_y+92), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200,220,255), 1)
        cv2.putText(viz, f"Surge cmd:{surge_v:+.3f}", (hud_x+10, hud_y+112), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200,220,255), 1)

        # display aim offset in px
        cv2.putText(viz, f"Aim offset: {int(self.aim_x_offset)} px", (hud_x+10, hud_y+132), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255,255,0), 1)

        dm_text = f"Deadman (RB#{self.deadman_button_idx}): {'HELD' if self.deadman_pressed else 'RELEASED'}"
        dm_col = (50, 220, 50) if self.deadman_pressed else (180,180,180)
        cv2.putText(viz, dm_text, (hud_x+180, hud_y+24), cv2.FONT_HERSHEY_SIMPLEX, 0.45, dm_col, 1)

        try:
            cv2.rectangle(viz, (0, self.img_height - 60), (self.img_width, self.img_height), (0,0,0), -1)
            cv2.putText(viz, f"{status_text}", (12, self.img_height - 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
            cv2.imshow("Smart Vision Control", viz)
            cv2.waitKey(1)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = BlueROVSmartVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == '__main__':
    main()

