#!/usr/bin/env python3
"""
bluerov_smart_vision.py
MANUAL LUNGE + STABILITY GATE + OSCILLATION FIXES + THRUSTER INFO:
- Smooth depth control during orbit (gentler gains, lower clamp).
- Blend sway objective smoothly (no hard switch at squared boundary).
- Ramped stiction kick (no step-like 0->0.08 jump).
- Continuous yaw scale during orbit (no hard suppress).
- Hysteresis on "aligned" condition (enter tight, exit loose).
- Real-time thruster info box (depth + surge/sway/heave/yaw with color coding).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64, String

from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from ultralytics import YOLO


class BlueROVSmartVision(Node):
    def __init__(self):
        super().__init__('bluerov_smart_vision')

        # --- CONFIGURATION ---
        self.model_path = '/home/elex/uji_rov_autonomous/src/rov_black_box/detection_model/best.pt'
        self.camera_topic = '/bluerov2/camera/image_raw'
        self.joy_topic = '/joy'
        self.window_name = "Smart Vision Control"

        # --- MANUAL TRIGGER CONFIG ---
        self.lunge_button_idx = 5  # 'A' commonly
        self.lunge_button_pressed = False
        self.prev_lunge_button_pressed = False

        # --- DASHBOARD BUTTON ---
        self.ui_lunge_pressed = False
        self.sub_ui_lunge = self.create_subscription(Bool, '/rov/ui_lunge', self.ui_lunge_cb, 10)

        # --- TUNING ---
        self.aim_x_offset = -50.0

        # 1. ORBIT PRIORITY
        self.parallax_priority_thresh = 30.0
        self.orbit_gate_thresh = 30.0
        self.orbit_creep_speed = 0.05

        # 2. ACTIVE APPROACH GAINS (Used when moving/orbiting)
        self.kp_sway_active = 0.0035
        self.ki_sway_active = 0.0005
        self.kd_sway_active = 0.006

        self.kp_yaw_active = 0.00035
        self.kd_yaw_active = 0.0008

        # 3. STATION KEEPING GAINS (for LUNGE_READY)
        self.kp_sway_station = 0.0015
        self.ki_sway_station = 0.0002
        self.kd_sway_station = 0.012

        self.kp_yaw_station = 0.00015
        self.kd_yaw_station = 0.002

        # KICK SETTINGS
        self.min_sway_pwr = 0.08

        # DEADBANDS
        self.kick_deadband_active = 8.0
        self.kick_deadband_station = 15.0
        self.ramp_px_stiction = 20.0  # ramp over this many pixels

        # DIRECTIONS
        self.yaw_sign = -1.0
        self.sway_sign = 1.0
        self.heave_sign = 1.0
        self.surge_sign = 1.0

        # --- MISSION PARAMETERS ---
        self.target_depth = -4.53
        self.buoyancy_offset = -0.10

        # DEPTH PID (for DIVING)
        self.kp_depth_dive = 0.8
        self.kp_depth_hold = 0.4
        self.ki_depth = 0.10
        self.kd_depth = 0.5
        self.i_max = 0.3

        # NEW: Orbit-specific depth gains (gentler, less overshoot)
        self.kp_depth_orbit = 0.25
        self.ki_depth_orbit = 0.04
        self.kd_depth_orbit = 0.35

        self.kp_yaw_search = 0.00045
        self.kp_surge = 0.002
        self.max_surge_speed = 0.15
        self.crawl_speed = 0.20
        self.dive_surge_speed = 0.10

        self.box_conf_thresh = 0.15
        self.depth_tol = 0.08
        self.yaw_deadzone = 40
        self.lunge_trigger_height = 280.0
        self.squaring_deadband = 15.0

        # MANUAL LUNGE
        self.lunge_speed = 0.45

        # IMPORTANT: hold still before allowing manual lunge
        self.required_stable_time = 2.0

        # smoothing for detections
        self.smoothing_alpha = 0.1

        # --- LOCKING PARAMETERS ---
        self.ratio_min = 0.5
        self.ratio_max = 2.5
        self.tracking_radius = 150.0

        # --- HOLDING STABILITY ---
        self.holding_max_pwr = 0.05
        self.holding_noise_limit = 3.0

        # --- HYSTERESIS for "aligned" condition ---
        self.aligned_enter_parallax = 12.0
        self.aligned_enter_yaw = 20.0
        self.aligned_exit_parallax = 25.0
        self.aligned_exit_yaw = 40.0
        self.is_currently_aligned = False

        # --- STATE ---
        self.mission_state = "IDLE"
        self.auto_active = False
        self.current_depth = 0.0

        self.depth_integral = 0.0
        self.sway_integral = 0.0

        self.box_data = None
        self.handle_data = None
        self.last_box_center = None

        self.filt_box_cx = None
        self.filt_hdl_cx = None

        self.prev_yaw_err = 0.0
        self.prev_sway_err = 0.0
        self.last_loop_time = time.time()

        self.stable_start_time = None
        self.lunge_armed = False

        self.recovery_start_time = 0.0

        self.last_depth_time = time.time()
        self.depth_vel = 0.0
        self.depth_filtered = None
        self.filt_depth_vel = 0.0

        self.img_width = 640
        self.window_initialized = False

        self.get_logger().info(f'Loading YOLO from {self.model_path}...')
        self.model = YOLO(self.model_path)
        self.bridge = CvBridge()

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub_img = self.create_subscription(Image, self.camera_topic, self.image_callback, 10)
        self.sub_auto = self.create_subscription(Bool, '/rov/servo_mode_active', self.auto_callback, 10)
        self.sub_depth = self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.depth_callback, qos_profile)
        self.sub_joy = self.create_subscription(Joy, self.joy_topic, self.joy_callback, 10)

        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/servoing/status', 10)

        self.get_logger().info('Smart Vision Node Ready.')

    def ui_lunge_cb(self, msg: Bool):
        self.ui_lunge_pressed = bool(msg.data)

    def joy_callback(self, msg: Joy):
        pressed = False
        try:
            if len(msg.buttons) > self.lunge_button_idx:
                pressed = bool(msg.buttons[self.lunge_button_idx])
        except Exception:
            pressed = False
        self.prev_lunge_button_pressed = self.lunge_button_pressed
        self.lunge_button_pressed = pressed

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if flags & cv2.EVENT_FLAG_SHIFTKEY:
                center_x = self.img_width / 2.0
                new_offset = x - center_x
                self.aim_x_offset = float(new_offset)
                self.get_logger().info(f"NEW AIM OFFSET SET: {self.aim_x_offset:.1f} pixels")

    def auto_callback(self, msg: Bool):
        self.auto_active = msg.data
        if msg.data:
            self.get_logger().info(">>> AUTO ENGAGED: Starting Dive <<<")
            self.mission_state = "DIVING"
            self.filt_box_cx = None
            self.filt_hdl_cx = None
            self.last_box_center = None
            self.prev_yaw_err = 0.0
            self.prev_sway_err = 0.0
            self.depth_integral = 0.0
            self.sway_integral = 0.0
            self.stable_start_time = None
            self.lunge_armed = False
            self.is_currently_aligned = False
        else:
            self.get_logger().info("<<< MANUAL MODE >>>")
            self.mission_state = "IDLE"
            self.depth_integral = 0.0
            self.sway_integral = 0.0
            self.stable_start_time = None
            self.lunge_armed = False
            self.is_currently_aligned = False
            self.pub_cmd.publish(Twist())

    def depth_callback(self, msg: Float64):
        now = time.time()
        raw_depth = msg.data
        if self.depth_filtered is None:
            self.depth_filtered = raw_depth
        dt = now - self.last_depth_time
        if dt > 0.001:
            raw_depth_vel = (raw_depth - self.current_depth) / dt
            # Low-pass filter depth_vel to reduce derivative action noise
            alpha_vel = 0.2
            self.filt_depth_vel = (alpha_vel * raw_depth_vel) + ((1.0 - alpha_vel) * self.filt_depth_vel)
            self.depth_vel = self.filt_depth_vel
            self.current_depth = raw_depth
            self.last_depth_time = now

    def apply_smoothing(self, raw_val, filt_val):
        if filt_val is None:
            return float(raw_val)
        return (self.smoothing_alpha * raw_val) + ((1.0 - self.smoothing_alpha) * filt_val)

    def compute_sway_pid(self, error, prev_error, dt, use_station_gains=False):
        derivative = (error - prev_error) / dt if dt > 0 else 0.0

        if use_station_gains:
            kp, ki, kd = self.kp_sway_station, self.ki_sway_station, self.kd_sway_station
        else:
            kp, ki, kd = self.kp_sway_active, self.ki_sway_active, self.kd_sway_active

        if abs(error) < 100.0:
            self.sway_integral += error * dt
            limit = 0.15 / ki if ki > 0 else 0.0
            self.sway_integral = float(np.clip(self.sway_integral, -limit, limit))
        else:
            self.sway_integral = 0.0

        p_term = error * kp
        i_term = self.sway_integral * ki
        d_term = derivative * kd
        return p_term + i_term + d_term

    def compute_yaw_pd(self, error, prev_error, dt, use_station_gains=False):
        derivative = (error - prev_error) / dt if dt > 0 else 0.0
        if use_station_gains:
            kp, kd = self.kp_yaw_station, self.kd_yaw_station
        else:
            kp, kd = self.kp_yaw_active, self.kd_yaw_active
        return (error * kp) + (derivative * kd)

    def select_best_target(self, detections, img_center):
        if not detections:
            self.last_box_center = None
            return None

        valid_candidates = []
        for d in detections:
            w, h = d['width'], d['height']
            ratio = w / float(h)
            if self.ratio_min < ratio < self.ratio_max:
                valid_candidates.append(d)

        if not valid_candidates:
            return None

        if self.last_box_center is not None:
            lx, ly = self.last_box_center
            nearby_candidates = []
            for d in valid_candidates:
                tx, ty = d['center']
                dist = np.hypot(tx - lx, ty - ly)
                if dist < self.tracking_radius:
                    nearby_candidates.append(d)
            if nearby_candidates:
                best = max(nearby_candidates, key=lambda x: x['conf'])
                self.last_box_center = best['center']
                return best

        best = max(valid_candidates, key=lambda x: x['conf'])
        self.last_box_center = best['center']
        return best

    def select_best_handle(self, detections, box_center):
        if not detections or not box_center:
            return None
        bx, by = box_center
        best = None
        min_dist = float('inf')
        for d in detections:
            tx, ty = d['center']
            dist = np.hypot(tx - bx, ty - by)
            if dist < min_dist:
                min_dist = dist
                best = d
        return best

    def get_depth_cmd(self, target, dt, use_integral=True, gentle=False, orbit=False):
        """
        Depth PID with orbit-specific gentler gains.
        orbit=True uses lower P/I/D (less overshoot during lateral motion).
        """
        err = target - self.current_depth

        if orbit:
            kp = self.kp_depth_orbit
            ki = self.ki_depth_orbit
            kd = self.kd_depth_orbit
        else:
            kp = self.kp_depth_hold if gentle else self.kp_depth_dive
            ki = self.ki_depth
            kd = self.kd_depth

        if use_integral and dt > 0 and abs(err) < 0.5:
            self.depth_integral += err * dt
            limit = 0.25 / ki if ki > 0 else 0.25
            self.depth_integral = float(np.clip(self.depth_integral, -limit, limit))
        else:
            self.depth_integral = 0.0

        p_term = err * kp
        i_term = self.depth_integral * ki if use_integral else 0.0
        d_term = self.depth_vel * kd

        z_cmd = p_term + i_term - d_term
        if err <= 0:
            z_cmd += self.buoyancy_offset
        z_cmd *= self.heave_sign

        return float(np.clip(z_cmd, -0.6, 0.6))

    def apply_stiction_kick_ramped(self, raw_cmd, min_pwr, error_pixels, deadband, ramp_px):
        """
        Ramped stiction kick: smoothly increases from 0 to min_pwr over ramp_px pixels
        beyond the deadband, instead of a step-like jump.
        """
        if abs(error_pixels) < deadband:
            return 0.0

        excess = abs(error_pixels) - deadband
        min_eff = min_pwr * np.clip(excess / ramp_px, 0.0, 1.0)

        if abs(raw_cmd) < 0.01:
            return 0.0
        if abs(raw_cmd) < min_eff:
            return min_eff if raw_cmd > 0 else -min_eff
        return raw_cmd

    def update_aligned_hysteresis(self, parallax_err, err_yaw):
        """
        Hysteresis logic: enters aligned at tight thresholds, exits at loose thresholds.
        Prevents rapid toggling when detections jitter near boundaries.
        """
        if self.is_currently_aligned:
            if abs(parallax_err) > self.aligned_exit_parallax or abs(err_yaw) > self.aligned_exit_yaw:
                self.is_currently_aligned = False
        else:
            if abs(parallax_err) < self.aligned_enter_parallax and abs(err_yaw) < self.aligned_enter_yaw:
                self.is_currently_aligned = True

        return self.is_currently_aligned

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        h, w, _ = frame.shape
        self.img_width = w
        img_center = (w // 2, h // 2)
        target_center_x = float(w // 2) + self.aim_x_offset

        if not self.window_initialized:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.setMouseCallback(self.window_name, self.mouse_callback)
            self.window_initialized = True

        results = self.model(frame, verbose=False)
        viz_base = results[0].plot()
        overlay = viz_base.copy()

        raw_boxes = []
        raw_handles = []

        if results and len(results) > 0 and results[0].boxes:
            for box in results[0].boxes:
                coords = box.xyxy[0].cpu().numpy().astype(int)
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])

                if cls_id == 0:
                    if conf < self.box_conf_thresh:
                        continue
                else:
                    if conf < 0.4:
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

        self.box_data = self.select_best_target(raw_boxes, img_center)
        box_center_ref = self.box_data['center'] if self.box_data else None
        self.handle_data = self.select_best_handle(raw_handles, box_center_ref)

        if self.box_data:
            self.filt_box_cx = self.apply_smoothing(self.box_data['center'][0], self.filt_box_cx)
        else:
            self.filt_box_cx = None

        if self.handle_data:
            self.filt_hdl_cx = self.apply_smoothing(self.handle_data['center'][0], self.filt_hdl_cx)
        else:
            self.filt_hdl_cx = None

        cmd = Twist()
        status_text = self.mission_state
        current_time = time.time()
        dt_loop = max(0.001, current_time - self.last_loop_time)
        self.last_loop_time = current_time

        if not self.auto_active:
            status_text = "MANUAL (Joy)"

        elif self.mission_state == "IDLE":
            pass

        # --- LOGIC ---
        elif self.mission_state == "DIVING":
            depth_err = abs(self.target_depth - self.current_depth)
            cmd.linear.z = self.get_depth_cmd(self.target_depth, dt_loop, use_integral=False, gentle=False)

            if self.filt_box_cx is not None:
                err_x = target_center_x - self.filt_box_cx
                cmd.angular.z = float(np.clip(err_x * self.kp_yaw_search * self.yaw_sign, -0.4, 0.4))
                cmd.linear.x = self.dive_surge_speed * self.surge_sign

            if depth_err < self.depth_tol:
                self.mission_state = "SEARCH_HANDLE"

        elif self.mission_state == "SEARCH_HANDLE":
            cmd.linear.z = self.get_depth_cmd(self.target_depth, dt_loop, use_integral=True, gentle=True, orbit=True)

            if self.handle_data:
                self.mission_state = "SQUARING"
                self.stable_start_time = None
                self.lunge_armed = False
                self.is_currently_aligned = False
            elif self.box_data:
                status_text = "SEARCH: CRAWLING..."
                err_x = target_center_x - self.filt_box_cx
                cmd.angular.z = float(np.clip(err_x * self.kp_yaw_search * self.yaw_sign, -0.4, 0.4))
                cmd.linear.x = abs(self.crawl_speed) * self.surge_sign
                cmd.linear.y = 0.0
            else:
                status_text = "SEARCH: LOST BOX"
                cmd.linear.x = 0.0

        elif self.mission_state == "SQUARING":
            if not self.box_data:
                status_text = "LOST BOX"
                cmd.linear.z = self.get_depth_cmd(self.target_depth, dt_loop, use_integral=True, gentle=True, orbit=True)

            elif not self.handle_data:
                self.mission_state = "SEARCH_HANDLE"

            else:
                box_cx = self.filt_box_cx
                hdl_cx = self.filt_hdl_cx
                parallax_err = hdl_cx - box_cx
                err_yaw = target_center_x - hdl_cx

                raw_yaw = self.compute_yaw_pd(err_yaw, self.prev_yaw_err, dt_loop, use_station_gains=False)
                yaw_scale = np.clip(1.0 - abs(parallax_err) / self.parallax_priority_thresh, 0.3, 1.0)
                cmd.angular.z = float(np.clip(raw_yaw * yaw_scale * self.yaw_sign, -0.20, 0.20))

                blend = np.clip(abs(parallax_err) / self.squaring_deadband, 0.0, 1.0)
                sway_target_err = blend * parallax_err + (1.0 - blend) * err_yaw

                raw_sway = self.compute_sway_pid(sway_target_err, self.prev_sway_err, dt_loop, use_station_gains=False)
                cmd.linear.y = float(np.clip(raw_sway * self.sway_sign, -0.3, 0.3))

                self.prev_yaw_err = err_yaw
                self.prev_sway_err = sway_target_err

                current_h = self.box_data['height']
                dist_err = self.lunge_trigger_height - current_h
                can_move_fwd = True

                if abs(parallax_err) > self.orbit_gate_thresh:
                    can_move_fwd = False
                    status_text = "ORBITING [ALIGNING]"
                    cmd.linear.x = self.orbit_creep_speed * self.surge_sign
                elif current_h > (self.lunge_trigger_height * 0.8):
                    can_move_fwd = False
                    status_text = "SQUARING [HALT]"

                if dist_err < 10:
                    cmd.linear.x = 0.0
                    self.mission_state = "LUNGE_READY"
                    self.stable_start_time = None
                    self.lunge_armed = False
                    self.is_currently_aligned = False
                elif can_move_fwd:
                    speed = dist_err * self.kp_surge
                    final_speed = float(np.clip(speed, -0.10, self.max_surge_speed))
                    cmd.linear.x = final_speed * self.surge_sign
                else:
                    status_text = f"ORBITING (P={int(parallax_err)} Y={int(err_yaw)})"

                cmd.linear.z = self.get_depth_cmd(self.target_depth, dt_loop, use_integral=True, gentle=True, orbit=True)

        elif self.mission_state == "LUNGE_READY":
            if not self.box_data or not self.handle_data:
                self.mission_state = "RECOVERY_BACKOFF"
                self.recovery_start_time = current_time
                self.stable_start_time = None
                self.lunge_armed = False
                self.is_currently_aligned = False
                status_text = "LOST TARGETS"
            else:
                box_cx = self.filt_box_cx
                hdl_cx = self.filt_hdl_cx
                parallax_err = hdl_cx - box_cx
                err_yaw = target_center_x - hdl_cx

                raw_yaw = self.compute_yaw_pd(err_yaw, self.prev_yaw_err, dt_loop, use_station_gains=True)
                cmd.angular.z = self.apply_stiction_kick_ramped(
                    float(np.clip(raw_yaw * self.yaw_sign, -0.15, 0.15)),
                    0.08, err_yaw, self.kick_deadband_station, self.ramp_px_stiction
                )

                raw_sway = self.compute_sway_pid(parallax_err, self.prev_sway_err, dt_loop, use_station_gains=True)
                cmd.linear.y = self.apply_stiction_kick_ramped(
                    float(np.clip(raw_sway * self.sway_sign, -0.2, 0.2)),
                    self.min_sway_pwr, parallax_err, self.kick_deadband_station, self.ramp_px_stiction
                )

                self.prev_yaw_err = err_yaw
                self.prev_sway_err = parallax_err

                cmd.linear.x = 0.0

                dist_err = self.lunge_trigger_height - self.box_data['height']
                is_aligned = self.update_aligned_hysteresis(parallax_err, err_yaw)

                if is_aligned:
                    if self.stable_start_time is None:
                        self.stable_start_time = current_time
                        self.lunge_armed = False

                    elapsed_stable = current_time - self.stable_start_time
                    status_text = f"HOLDING... {elapsed_stable:.1f}s"

                    cmd.angular.z = float(np.clip(cmd.angular.z, -self.holding_max_pwr, self.holding_max_pwr))
                    cmd.linear.y = float(np.clip(cmd.linear.y, -self.holding_max_pwr, self.holding_max_pwr))

                    if abs(err_yaw) < self.holding_noise_limit:
                        cmd.angular.z = 0.0
                    if abs(parallax_err) < self.holding_noise_limit:
                        cmd.linear.y = 0.0

                    if elapsed_stable >= self.required_stable_time:
                        self.lunge_armed = True
                else:
                    self.stable_start_time = None
                    self.lunge_armed = False
                    status_text = f"ALIGNING: P={int(parallax_err)} Y={int(err_yaw)}"

                if self.lunge_armed:
                    status_text = "READY (ARMED): HOLD 'A' TO LUNGE"
                    if self.lunge_button_pressed or self.ui_lunge_pressed:
                        status_text = ">>> MANUAL LUNGE! <<<"
                        cmd.linear.x = self.lunge_speed * self.surge_sign

            cmd.linear.z = self.get_depth_cmd(self.target_depth, dt_loop, use_integral=True, gentle=True, orbit=True)

        elif self.mission_state == "RECOVERY_BACKOFF":
            elapsed = current_time - self.recovery_start_time
            if elapsed < 2.0:
                status_text = f"RECOVERING... ({elapsed:.1f}s)"
                cmd.linear.x = -0.15 * self.surge_sign
                cmd.linear.z = self.get_depth_cmd(self.target_depth, dt_loop, use_integral=True, gentle=True, orbit=True)
                cmd.angular.z = 0.0
                cmd.linear.y = 0.0
            else:
                self.mission_state = "SEARCH_HANDLE"

        elif self.mission_state == "COMPLETE":
            status_text = "COMPLETE"
            cmd = Twist()

        if self.auto_active:
            self.pub_cmd.publish(cmd)
            self.pub_status.publish(String(data=status_text))

        # --- VISUALIZATION ---
        cv2.line(overlay, (int(self.img_width / 2), 0), (int(self.img_width / 2), h), (200, 200, 200), 1)
        aim_x_int = int(target_center_x)
        cv2.line(overlay, (aim_x_int, 0), (aim_x_int, h), (255, 255, 0), 2)
        alpha = 0.4
        cv2.addWeighted(overlay, alpha, viz_base, 1 - alpha, 0, viz_base)

        if self.last_box_center:
            lx, ly = self.last_box_center
            cv2.circle(viz_base, (int(lx), int(ly)), 15, (0, 0, 255), 2)

        if self.filt_box_cx is not None:
            cv2.circle(viz_base, (int(self.filt_box_cx), int(h / 2)), 8, (255, 100, 0), -1)
            cv2.putText(viz_base, "BOX", (int(self.filt_box_cx) + 10, int(h / 2)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 0), 1)
        if self.filt_hdl_cx is not None:
            cv2.circle(viz_base, (int(self.filt_hdl_cx), int(h / 2)), 8, (0, 255, 0), -1)
            cv2.putText(viz_base, "HDL", (int(self.filt_hdl_cx) + 10, int(h / 2)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # --- STATUS BAR ---
        cv2.rectangle(viz_base, (0, 0), (w, 60), (0, 0, 0), -1)
        cv2.putText(viz_base, f"State: {status_text}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(viz_base, f"Offset: {int(self.aim_x_offset)}", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)

        # --- THRUSTER INFO BOX ---
        info_box_x, info_box_y = 10, 75
        info_box_w, info_box_h = 220, 110
        
        overlay_info = viz_base.copy()
        cv2.rectangle(overlay_info, (info_box_x, info_box_y), 
                      (info_box_x + info_box_w, info_box_y + info_box_h), 
                      (0, 0, 0), -1)
        cv2.addWeighted(overlay_info, 0.6, viz_base, 0.4, 0, viz_base)
        
        cv2.rectangle(viz_base, (info_box_x, info_box_y), 
                      (info_box_x + info_box_w, info_box_y + info_box_h), 
                      (100, 100, 100), 1)
        
        depth_text = f"Depth: {self.current_depth:.2f}m"
        cv2.putText(viz_base, depth_text, (info_box_x + 8, info_box_y + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        def get_thruster_color(value):
            if abs(value) < 0.01:
                return (150, 150, 150)  # Gray = neutral
            elif value > 0:
                return (0, 255, 0)  # Green = forward/up
            else:
                return (0, 0, 255)  # Red = backward/down
        
        surge_color = get_thruster_color(cmd.linear.x)
        surge_text = f"Surge:  {cmd.linear.x:+.3f}"
        cv2.putText(viz_base, surge_text, (info_box_x + 8, info_box_y + 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, surge_color, 1)
        
        sway_color = get_thruster_color(cmd.linear.y)
        sway_text = f"Sway:   {cmd.linear.y:+.3f}"
        cv2.putText(viz_base, sway_text, (info_box_x + 8, info_box_y + 57),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, sway_color, 1)
        
        heave_color = get_thruster_color(cmd.linear.z)
        heave_text = f"Heave:  {cmd.linear.z:+.3f}"
        cv2.putText(viz_base, heave_text, (info_box_x + 8, info_box_y + 74),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, heave_color, 1)
        
        yaw_color = get_thruster_color(cmd.angular.z)
        yaw_text = f"Yaw:    {cmd.angular.z:+.3f}"
        cv2.putText(viz_base, yaw_text, (info_box_x + 8, info_box_y + 91),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, yaw_color, 1)

        cv2.imshow(self.window_name, viz_base)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = BlueROVSmartVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

