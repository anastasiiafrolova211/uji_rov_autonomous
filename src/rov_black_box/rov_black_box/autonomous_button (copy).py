#!/usr/bin/env python3
"""
bluerov_smart_vision.py
FIXED: DISTANCE CONTROL
1. Increased Safety Distance: Reduced target box height (280 -> 230 px).
   - ROV will stop further back.
2. Active Braking: If the ROV gets too close, it reverses instead of just stopping.
3. Smoothed Kick & Strong Hold logic preserved.
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
        self.was_lunging = False

        # --- DASHBOARD BUTTON ---
        self.ui_lunge_pressed = False
        self.sub_ui_lunge = self.create_subscription(Bool, '/rov/ui_lunge', self.ui_lunge_cb, 10)

        # --- TUNING ---
        self.aim_x_offset = -50.0

        # 1. ORBIT PRIORITY
        self.parallax_priority_thresh = 30.0
        self.orbit_gate_thresh = 30.0
        self.orbit_creep_speed = 0.05

        # 2. SWAY PID
        self.kp_sway = 0.0040  
        self.ki_sway = 0.0015  
        self.kd_sway = 0.008   

        # 3. YAW PID
        self.kp_yaw = 0.00040
        self.kd_yaw = 0.0012

        # KICK SETTINGS
        self.min_sway_pwr = 0.06
        self.kick_deadband = 12.0
        self.kick_ramp_width = 15.0

        # DIRECTIONS
        self.yaw_sign = -1.0
        self.sway_sign = 1.0
        self.heave_sign = 1.0
        self.surge_sign = 1.0

        # --- MISSION PARAMETERS ---
        self.target_depth = -4.56
        self.buoyancy_offset = -0.10

        # DEPTH PID
        self.kp_depth_dive = 0.8
        self.kp_depth_hold = 0.4
        self.ki_depth = 0.10
        self.kd_depth = 0.5
        
        self.kp_yaw_search = 0.00045
        self.kp_surge = 0.002
        self.max_surge_speed = 0.15
        self.crawl_speed = 0.20
        self.dive_surge_speed = 0.10

        self.box_conf_thresh = 0.15
        self.depth_tol = 0.08
        self.yaw_deadzone = 40
        
        # --- DISTANCE SAFETY UPDATE ---
        # Reduced from 280.0 to 230.0 so it stops further back.
        self.lunge_trigger_height = 230.0
        self.squaring_deadband = 15.0

        # MANUAL LUNGE SPEED
        self.lunge_speed = 0.45

        self.smoothing_alpha = 0.3

        # --- LOCKING PARAMETERS ---
        self.ratio_min = 0.5
        self.ratio_max = 2.5
        self.tracking_radius = 150.0

        # --- HOLDING STABILITY ---
        self.holding_max_pwr = 0.05
        self.holding_noise_limit = 3.0

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
        
        # RECOVERY & TIMEOUTS
        self.recovery_start_time = 0.0
        self.target_lost_time = None
        self.loss_timeout = 2.5

        self.last_depth_time = time.time()
        self.depth_vel = 0.0
        self.depth_filtered = None
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
            self.target_lost_time = None
            self.was_lunging = False
        else:
            self.get_logger().info("<<< MANUAL MODE >>>")
            self.mission_state = "IDLE"
            self.depth_integral = 0.0
            self.sway_integral = 0.0
            self.target_lost_time = None
            self.lunge_armed = False
            self.was_lunging = False
            self.pub_cmd.publish(Twist())

    def depth_callback(self, msg: Float64):
        now = time.time()
        raw_depth = msg.data
        if self.depth_filtered is None:
            self.depth_filtered = raw_depth
        dt = now - self.last_depth_time
        if dt > 0.001:
            self.depth_vel = (raw_depth - self.current_depth) / dt
            self.current_depth = raw_depth
            self.last_depth_time = now

    def apply_smoothing(self, raw_val, filt_val):
        if filt_val is None:
            return float(raw_val)
        return (self.smoothing_alpha * raw_val) + ((1.0 - self.smoothing_alpha) * filt_val)

    def compute_sway_pid(self, error, prev_error, dt):
        """Full PID for Sway (Fixes steady-state angle error)"""
        derivative = (error - prev_error) / dt if dt > 0 else 0.0
        
        if abs(error) < 100.0:
            self.sway_integral += error * dt
            limit = 0.20 / self.ki_sway 
            self.sway_integral = float(np.clip(self.sway_integral, -limit, limit))
        else:
            self.sway_integral = 0.0 
            
        p_term = error * self.kp_sway
        i_term = self.sway_integral * self.ki_sway
        d_term = derivative * self.kd_sway
        return p_term + i_term + d_term

    def compute_yaw_pd(self, error, prev_error, dt):
        """PD for Yaw"""
        derivative = (error - prev_error) / dt if dt > 0 else 0.0
        return (error * self.kp_yaw) + (derivative * self.kd_yaw)

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
        if not detections or not box_center: return None
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

    def get_depth_cmd(self, target, dt, use_integral=True, gentle=False):
        err = target - self.current_depth
        kp = self.kp_depth_hold if gentle else self.kp_depth_dive
        
        if use_integral and dt > 0 and abs(err) < 0.5:
            self.depth_integral += err * dt
            limit = 0.25 / self.ki_depth
            self.depth_integral = float(np.clip(self.depth_integral, -limit, limit))
        else:
            self.depth_integral = 0.0
            
        p_term = err * kp
        i_term = self.depth_integral * self.ki_depth if use_integral else 0.0
        d_term = self.depth_vel * self.kd_depth
        
        z_cmd = p_term + i_term - d_term
        if err <= 0: z_cmd += self.buoyancy_offset
        z_cmd *= self.heave_sign
        
        return float(np.clip(z_cmd, -0.6, 0.6))

    def apply_stiction_kick_ramped(self, raw_cmd, min_pwr, error_pixels):
        """
        Replaces harsh step-kick with a smooth ramp to prevent jitter.
        """
        err_abs = abs(error_pixels)
        if err_abs < self.kick_deadband:
            return raw_cmd
        
        ramp_factor = (err_abs - self.kick_deadband) / self.kick_ramp_width
        ramp_factor = np.clip(ramp_factor, 0.0, 1.0)
        
        needed_kick = min_pwr * ramp_factor
        
        if abs(raw_cmd) < needed_kick:
            return needed_kick if raw_cmd > 0 else -needed_kick
            
        return raw_cmd

    # --- CUSTOM DRAWING FUNCTION ---
    def draw_det(self, img, det, color, label):
        x1, y1, x2, y2 = [int(v) for v in det['coords']]
        cx, cy = det['center']
        conf = det.get('conf', 0.0)

        # Thin anti-aliased box
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2, lineType=cv2.LINE_AA)

        # Center dot
        cv2.circle(img, (int(cx), int(cy)), 4, color, -1, lineType=cv2.LINE_AA)

        # Label tag
        text = f"{label} {conf:.2f}"
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        pad = 3
        bx1, by1 = x1, max(0, y1 - th - 2 * pad)
        bx2, by2 = x1 + tw + 2 * pad, y1
        cv2.rectangle(img, (bx1, by1), (bx2, by2), (0, 0, 0), -1)
        cv2.putText(img, text, (x1 + pad, y1 - pad),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            return

        h, w, _ = frame.shape
        self.img_width = w
        img_center = (w // 2, h // 2)
        target_center_x = float(w // 2) + self.aim_x_offset

        if not self.window_initialized:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, w, h) 
            cv2.setMouseCallback(self.window_name, self.mouse_callback)
            self.window_initialized = True

        results = self.model(frame, verbose=False)
        
        raw_boxes = []
        raw_handles = []
        
        if results and len(results) > 0 and results[0].boxes:
            for box in results[0].boxes:
                coords = box.xyxy[0].cpu().numpy().astype(int)
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                if cls_id == 0:
                    if conf < self.box_conf_thresh: continue 
                else:
                    if conf < 0.4: continue
                cx = int((coords[0] + coords[2]) / 2)
                cy = int((coords[1] + coords[3]) / 2)
                width = coords[2] - coords[0]
                height = coords[3] - coords[1]
                det = {'coords': coords, 'center': (cx, cy), 
                       'width': width, 'height': height, 'conf': conf}
                if cls_id == 0: raw_boxes.append(det)
                elif cls_id == 1: raw_handles.append(det)

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
            cmd.linear.z = self.get_depth_cmd(self.target_depth, dt_loop, use_integral=True, gentle=True)
            if self.handle_data:
                self.mission_state = "SQUARING"
                self.stable_start_time = None 
                self.target_lost_time = None
                self.lunge_armed = False
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
                cmd.linear.z = self.get_depth_cmd(self.target_depth, dt_loop, use_integral=True, gentle=True)
            elif not self.handle_data:
                self.mission_state = "SEARCH_HANDLE"
            else:
                box_cx = self.filt_box_cx
                hdl_cx = self.filt_hdl_cx
                parallax_err = hdl_cx - box_cx 
                is_squared = abs(parallax_err) < self.squaring_deadband
                err_yaw = target_center_x - hdl_cx
                
                # Orbit Priority Logic
                suppress_yaw = abs(parallax_err) > self.parallax_priority_thresh
                
                raw_yaw = self.compute_yaw_pd(err_yaw, self.prev_yaw_err, dt_loop)
                if suppress_yaw: raw_yaw *= 0.2
                
                # CRASH GUARD: If super close (>70% screen), don't yaw
                box_width_pct = self.box_data['width'] / float(self.img_width)
                if box_width_pct > 0.70:
                    raw_yaw = 0.0 
                    status_text += " [PROXIMITY]"

                cmd.angular.z = float(np.clip(raw_yaw * self.yaw_sign, -0.20, 0.20))
                
                sway_target_err = err_yaw if is_squared else parallax_err
                raw_sway = self.compute_sway_pid(sway_target_err, self.prev_sway_err, dt_loop)
                cmd.linear.y = float(np.clip(raw_sway * self.sway_sign, -0.3, 0.3))

                if is_squared:
                    status_text = "LOCKED"
                else:
                    status_text = f"ORBITING (Err:{int(parallax_err)})"

                self.prev_yaw_err = err_yaw
                self.prev_sway_err = sway_target_err

                current_h = self.box_data['height']
                
                # --- NEW: PROXIMITY DAMPENING (STRONGER HOLD) ---
                is_close = current_h > 200
                dampener = 0.75 if is_close else 1.0 
                
                cmd.angular.z *= dampener
                cmd.linear.y *= dampener
                if is_close: status_text += " [STEADY]"

                # --- ORBIT GATE ---
                if abs(parallax_err) > self.orbit_gate_thresh:
                    cmd.linear.x = self.orbit_creep_speed * self.surge_sign
                    status_text += " [ALIGNING]"
                else:
                    dist_err = self.lunge_trigger_height - current_h
                    
                    # --- BACK-OFF LOGIC ---
                    if dist_err < -5.0: # Too close!
                        cmd.linear.x = -0.10 * self.surge_sign # Back up
                        status_text += " [BACKING]"
                    elif dist_err < 10:
                        cmd.linear.x = 0.0
                        self.mission_state = "LUNGE_READY"
                        self.stable_start_time = None
                        self.target_lost_time = None
                        self.lunge_armed = False
                    else:
                        speed = dist_err * self.kp_surge
                        final_speed = float(np.clip(speed, -0.10, self.max_surge_speed))
                        cmd.linear.x = final_speed * self.surge_sign
                        status_text = "APPROACHING"

                cmd.linear.z = self.get_depth_cmd(self.target_depth, dt_loop, use_integral=True, gentle=True)

        elif self.mission_state == "LUNGE_READY":
            if not self.box_data or not self.handle_data:
                if self.target_lost_time is None:
                    self.target_lost_time = current_time
                elapsed_lost = current_time - self.target_lost_time
                status_text = f"WAITING... ({elapsed_lost:.1f}s)"

                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0
                cmd.linear.z = self.get_depth_cmd(self.target_depth, dt_loop, use_integral=True, gentle=True)

                if elapsed_lost > self.loss_timeout:
                    self.mission_state = "RECOVERY_BACKOFF"
                    self.recovery_start_time = current_time
                    self.stable_start_time = None
                    self.target_lost_time = None 
                    self.lunge_armed = False
                    status_text = "LOST TARGETS - RECOVERING"
            else:
                self.target_lost_time = None
                box_cx = self.filt_box_cx
                hdl_cx = self.filt_hdl_cx
                parallax_err = hdl_cx - box_cx
                err_yaw = target_center_x - hdl_cx
                
                # --- FINE TUNING (STRONG HOLD) ---
                raw_yaw = self.compute_yaw_pd(err_yaw, self.prev_yaw_err, dt_loop)
                raw_sway = self.compute_sway_pid(parallax_err, self.prev_sway_err, dt_loop)
                
                # Keep high power (75%) to prevent drifting
                final_yaw = raw_yaw * self.yaw_sign * 0.75 
                final_sway = raw_sway * self.sway_sign * 0.75
                
                # Use RAMPED kick to prevent jitter
                cmd.angular.z = self.apply_stiction_kick_ramped(float(np.clip(final_yaw, -0.15, 0.15)), 0.08, err_yaw)
                cmd.linear.y = self.apply_stiction_kick_ramped(float(np.clip(final_sway, -0.2, 0.2)), self.min_sway_pwr, parallax_err)
                
                self.prev_yaw_err = err_yaw
                self.prev_sway_err = parallax_err
                
                # --- BACK-OFF IN FINE ALIGN ---
                current_h = self.box_data['height']
                if current_h > (self.lunge_trigger_height + 5.0):
                     cmd.linear.x = -0.08 * self.surge_sign # Gentle backoff
                     status_text = "FINE ALIGN [BACKING]"
                else:
                     cmd.linear.x = 0.0

                # --- INSTANT ARMING LOGIC ---
                in_fine_zone = abs(parallax_err) < 60.0
                self.lunge_armed = in_fine_zone

                # --- DEADMAN TRIGGER LOGIC ---
                user_pressing = self.lunge_button_pressed or self.ui_lunge_pressed
                
                if self.lunge_armed:
                    if user_pressing:
                        status_text = ">>> MANUAL LUNGE! <<<"
                        cmd.linear.x = self.lunge_speed * self.surge_sign
                        self.was_lunging = True 
                    else:
                        status_text = "ARMED (FINE): HOLD 'A'"
                        if self.was_lunging:
                            self.mission_state = "COMPLETE"
                else:
                    self.was_lunging = False

                cmd.linear.z = self.get_depth_cmd(self.target_depth, dt_loop, use_integral=True, gentle=True)

        elif self.mission_state == "RECOVERY_BACKOFF":
            elapsed = current_time - self.recovery_start_time
            if elapsed < 2.0:
                status_text = f"RECOVERING... ({elapsed:.1f}s)"
                cmd.linear.x = -0.15 * self.surge_sign 
                cmd.linear.z = self.get_depth_cmd(self.target_depth, dt_loop, use_integral=True, gentle=True)
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
        viz = frame.copy()
        
        cv2.line(viz, (int(w/2), 0), (int(w/2), h), (180, 180, 180), 1, cv2.LINE_AA)
        aim_x_int = int(target_center_x)
        cv2.line(viz, (aim_x_int, 0), (aim_x_int, h), (255, 255, 0), 2, cv2.LINE_AA)

        if self.box_data:
            self.draw_det(viz, self.box_data, (255, 140, 0), "BOX")
        if self.handle_data:
            self.draw_det(viz, self.handle_data, (0, 255, 0), "HDL")

        # Top Header
        cv2.rectangle(viz, (0,0), (w, 64), (0,0,0), -1)
        cv2.putText(viz, f"State: {status_text}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(viz, f"Offset: {int(self.aim_x_offset)}", (10, 55), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1, cv2.LINE_AA)

        # Info Box
        sub_alpha = 0.5
        info_overlay = viz.copy()
        cv2.rectangle(info_overlay, (10, 70), (220, 210), (20, 20, 20), -1)
        cv2.addWeighted(info_overlay, sub_alpha, viz, 1 - sub_alpha, 0, viz)
        cv2.rectangle(viz, (10, 70), (220, 210), (0, 0, 0), 1) 

        font_sc = 0.5
        col = (220, 220, 220)
        thick = 1
        
        cv2.putText(viz, f"SURGE (X): {cmd.linear.x:.2f}", (20, 95), cv2.FONT_HERSHEY_SIMPLEX, font_sc, col, thick, cv2.LINE_AA)
        cv2.putText(viz, f"SWAY  (Y): {cmd.linear.y:.2f}", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, font_sc, col, thick, cv2.LINE_AA)
        cv2.putText(viz, f"HEAVE (Z): {cmd.linear.z:.2f}", (20, 145), cv2.FONT_HERSHEY_SIMPLEX, font_sc, col, thick, cv2.LINE_AA)
        cv2.putText(viz, f"YAW   (R): {cmd.angular.z:.2f}", (20, 170), cv2.FONT_HERSHEY_SIMPLEX, font_sc, col, thick, cv2.LINE_AA)
        
        depth_col = (0, 255, 0) if abs(self.target_depth - self.current_depth) < 0.1 else (255, 255, 255)
        cv2.putText(viz, f"DEPTH: {self.current_depth:.2f}m", (20, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.6, depth_col, 2, cv2.LINE_AA)

        cv2.imshow(self.window_name, viz)
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
