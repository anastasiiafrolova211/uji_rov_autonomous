#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64, String

from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from ultralytics import YOLO


class BlueROVVisionNode(Node):
    def __init__(self):
        super().__init__('bluerov_vision_node')

        # --- CONFIGURATION ---
        self.model_path = '/home/elex/uji_rov_autonomous/src/rov_black_box/detection_model/best.pt'
        self.camera_topic = '/bluerov2/camera/image_raw'

        # --- MISSION PARAMETERS ---
        self.target_depth = -4.57
        self.buoyancy_offset = -0.10

        # --- DETECTION ---
        self.target_box_area = 0.04
        self.transition_threshold = 0.01
        self.smooth_alpha = 0.15

        # --- SQUARING "SAFE ZONE" ---
        self.squaring_deadband = 0.30

        # --- BOX RATIO EMA ---
        self.box_ratio_ema = None
        self.box_ratio_alpha = 0.35
        self.box_ratio_last_time = None
        self.box_ratio_rate = 0.0

        # --- GAINS ---
        self.kp_depth_up = 0.3
        self.kp_depth_down = 0.3

        # DIVING
        self.kp_yaw_diving = 1.0
        self.kp_lat_diving = 0.0

        # ORBIT GAINS
        self.kp_yaw_orbit = 0.8
        self.kp_sway_orbit = 1.2

        # --- FIXED: MISSING FALLBACK GAIN ADDED HERE ---
        self.kp_sway_fallback = 1.0

        # LOCK GAINS
        self.kp_sway_lock = 1.0
        self.kp_yaw_lock = 0.5

        # LUNGE READY
        self.kp_lat_hold = 1.0
        self.kp_yaw_hold = 0.8
        self.kp_dist_hold = 2.0

        # --- SPEEDS ---
        self.max_vert_speed = 0.30
        self.max_fwd_speed = 0.15
        self.min_fwd_crawl = 0.05
        self.min_back_crawl = 0.05

        # --- ADAPTIVE APPROACH ---
        self.max_approach_error_tol = 0.25

        # --- LUNGE / TIMERS ---
        self.enable_lunge = True
        self.lunge_duration = 2.5
        self.lunge_speed = 0.45
        self.handle_align_tol = 0.08
        # REMOVED COUNTDOWN - Instant fire
        self.lunge_countdown = 0.0

        # --- BRAKING TUNING ---
        self.area_deadband = 0.002
        self.area_overshoot_tol = 0.005
        self.max_backoff_speed = 0.08
        self.area_to_speed_scale = 3.0
        self.rate_penalty_scale = 0.8
        self.rate_threshold = 0.02

        # Reduced stability timer to just filter noise (0.2s)
        self.approach_stable_start_time = 0.0
        self.approach_stable_duration = 0.2

        # --- Filter & PD Setup ---
        self.depth_filtered = None
        self.last_depth_filtered = None
        self.last_depth_time = None
        self.depth_vel = 0.0
        self.alpha_depth_filter = 0.15
        self.kd_depth = 0.8
        self.bottom_buffer = 0.12
        self.deadband = 0.03
        self.bottom_lift = 0.06
        self.strict_tol = 0.06
        self.loose_tol = 0.30

        # YOLO
        self.get_logger().info(f'Loading YOLO model from {self.model_path}...')
        self.model = YOLO(self.model_path)

        # ROS Setup
        self.bridge = CvBridge()

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(Image, self.camera_topic, self.image_callback, 10)
        self.auto_sub = self.create_subscription(Bool, '/rov/servo_mode_active', self.auto_state_callback, 10)
        self.depth_sub = self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.depth_callback, qos_profile)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # ---------- NEW: NARRATOR-RELATED PUBLISHERS ----------
        # Mission state as a simple string, consumed by narrator.py
        self.state_pub = self.create_publisher(String, '/rov/mission_state', 10)
        # Black box detection confidence (0–1)
        self.box_conf_pub = self.create_publisher(Float64, '/rov/blackbox_confidence', 10)
        # Black box angle relative to center (deg, left negative, right positive)
        self.box_angle_pub = self.create_publisher(Float64, '/rov/blackbox_angle_deg', 10)
        # Handle confidence (0–1)
        self.handle_conf_pub = self.create_publisher(Float64, '/rov/handle_confidence', 10)
        # Handle distance estimate in meters (very rough, from area ratio)
        self.handle_dist_pub = self.create_publisher(Float64, '/rov/handle_distance_m', 10)
        # Handle visibility
        self.handle_visible_pub = self.create_publisher(Bool, '/rov/handle_visible', 10)
        # ------------------------------------------------------

        # State Variables
        self.current_depth = 0.0
        self.auto_active = False
        self.depth_achieved = False
        self.filt_target_x = None
        self.filt_err_x = 0.0
        self.mission_state = "IDLE"

        # Timers
        self.lunge_start_time = 0.0
        self.is_squared = False

        self.get_logger().info(
            'Vision Control Ready. Mode: Deadband Squaring + INSTANT LUNGE'
        )

    # ---------- Simple helper to publish mission state string ----------
    def publish_state(self):
        msg = String()
        msg.data = self.mission_state
        self.state_pub.publish(msg)

    def auto_state_callback(self, msg: Bool):
        self.auto_active = msg.data

        if msg.data:
            self.get_logger().info(">>> AUTO ENGAGED <<<")
            self.mission_state = "DIVING"
            self.depth_achieved = False
            self.lunge_start_time = 0.0
            self.approach_stable_start_time = 0.0
            self.box_ratio_ema = None
            self.box_ratio_last_time = None
            self.box_ratio_rate = 0.0
            self.is_squared = False
        else:
            self.get_logger().info("<<< MANUAL MODE - RESET <<<")
            self.depth_achieved = False
            self.mission_state = "IDLE"
            self.lunge_start_time = 0.0
            self.approach_stable_start_time = 0.0
            self.box_ratio_ema = None
            self.box_ratio_last_time = None
            self.box_ratio_rate = 0.0
            self.is_squared = False
            self.cmd_pub.publish(Twist())

        # publish mission state after any change
        self.publish_state()

    def depth_callback(self, msg: Float64):
        self.current_depth = msg.data

    def select_best_target(self, detections, image_center):
        if not detections:
            return None

        best_target = None
        min_distance = float('inf')
        cx, cy = image_center

        for det in detections:
            tx, ty = det['center']
            dist = np.hypot(tx - cx, ty - cy)
            if dist < min_distance:
                min_distance = dist
                best_target = det

        return best_target

    def update_box_ratio_ema(self, box_ratio, now):
        if self.box_ratio_ema is None:
            self.box_ratio_ema = float(box_ratio)
            self.box_ratio_last_time = now
            self.box_ratio_rate = 0.0
            return

        dt = max(1e-3, now - (self.box_ratio_last_time or now))
        prev = self.box_ratio_ema
        alpha = self.box_ratio_alpha

        self.box_ratio_ema = alpha * float(box_ratio) + (1.0 - alpha) * prev
        self.box_ratio_rate = (self.box_ratio_ema - prev) / dt
        self.box_ratio_last_time = now

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w, _ = frame.shape
            img_area = float(h * w)
            rov_cx, rov_cy = w // 2, h // 2

            if self.filt_target_x is None:
                self.filt_target_x = float(rov_cx)

            # --- 1. DETECTION ---
            try:
                results = self.model(frame, verbose=False)
            except Exception as e:
                self.get_logger().error(f"YOLO inference error: {e}")
                results = None

            raw_boxes = []
            raw_handles = []

            if (
                results
                and len(results) > 0
                and hasattr(results[0], 'boxes')
                and results[0].boxes is not None
            ):
                for box in results[0].boxes:
                    try:
                        coords = box.xyxy[0].cpu().numpy()
                        cls_id = int(box.cls[0].cpu().numpy()) if hasattr(box.cls[0], 'cpu') else int(box.cls[0])
                        conf = float(box.conf[0].cpu().numpy()) if hasattr(box.conf[0], 'cpu') else float(box.conf[0])
                    except Exception:
                        coords = np.array(box.xyxy[0])
                        cls_id = int(box.cls[0])
                        conf = float(box.conf[0])

                    if conf < 0.25:
                        continue

                    cx = int((coords[0] + coords[2]) / 2)
                    cy = int((coords[1] + coords[3]) / 2)
                    width = float(coords[2] - coords[0])
                    height = float(coords[3] - coords[1])
                    area = width * height

                    det_data = {
                        'coords': coords,
                        'center': (cx, cy),
                        'conf': conf,
                        'area': area
                    }

                    if cls_id == 0:
                        raw_boxes.append(det_data)
                    elif cls_id == 1:
                        raw_handles.append(det_data)

            box_target = self.select_best_target(raw_boxes, (rov_cx, rov_cy))
            handle_target = self.select_best_target(raw_handles, (rov_cx, rov_cy))

            box_ratio = 0.0
            if box_target:
                box_ratio = box_target['area'] / img_area

            current_time = time.time()
            self.update_box_ratio_ema(box_ratio, current_time)
            box_ratio_ema = float(self.box_ratio_ema or box_ratio)

            # ---------- NEW: PUBLISH DETECTION INFO FOR NARRATOR ----------
            # Box confidence and relative angle
            if box_target:
                box_conf_msg = Float64()
                box_conf_msg.data = float(box_target['conf'])
                self.box_conf_pub.publish(box_conf_msg)

                bx, _ = box_target['center']
                # relative horizontal offset -> approximate angle in degrees (simple linear mapping)
                # normalized offset in [-1, 1]
                norm_off = (bx - rov_cx) / float(max(1.0, w / 2))
                angle_deg = norm_off * 30.0  # "roughly +/-30 deg" span
                angle_msg = Float64()
                angle_msg.data = float(angle_deg)
                self.box_angle_pub.publish(angle_msg)
            else:
                # publish zero/NaN if you prefer; here we just send 0
                box_conf_msg = Float64()
                box_conf_msg.data = 0.0
                self.box_conf_pub.publish(box_conf_msg)

            # Handle confidence/visibility and rough distance (from box area)
            handle_visible_msg = Bool()
            if handle_target:
                handle_visible_msg.data = True
                self.handle_visible_pub.publish(handle_visible_msg)

                h_conf_msg = Float64()
                h_conf_msg.data = float(handle_target['conf'])
                self.handle_conf_pub.publish(h_conf_msg)

                # crude distance ~ inverse sqrt(area_ratio)
                if box_ratio_ema > 1e-6:
                    approx_dist = 1.0 / max(1e-3, np.sqrt(box_ratio_ema))
                else:
                    approx_dist = 0.0
                dist_msg = Float64()
                dist_msg.data = float(approx_dist)
                self.handle_dist_pub.publish(dist_msg)
            else:
                handle_visible_msg.data = False
                self.handle_visible_pub.publish(handle_visible_msg)

                h_conf_msg = Float64()
                h_conf_msg.data = 0.0
                self.handle_conf_pub.publish(h_conf_msg)

                dist_msg = Float64()
                dist_msg.data = 0.0
                self.handle_dist_pub.publish(dist_msg)
            # ---------------------------------------------------------------

            # --- VISUALIZATION SETUP ---
            active_target = box_target
            display_color = (255, 100, 0)

            if handle_target and self.mission_state == "APPROACHING":
                active_target = handle_target
                display_color = (0, 255, 255)

            # --- SMOOTHING ---
            target_x = float(rov_cx)
            if box_target:
                target_x, _ = box_target['center']
                target_x = float(target_x)

            self.filt_target_x = (self.smooth_alpha * target_x) + ((1.0 - self.smooth_alpha) * self.filt_target_x)

            cmd = Twist()

            # --- ERROR CALCULATIONS ---
            err_box_img = (self.filt_target_x - float(rov_cx)) / float(max(1.0, (w / 2)))
            err_box_img = float(np.clip(err_box_img, -1.0, 1.0))

            self.filt_err_x = 0.3 * err_box_img + 0.7 * self.filt_err_x
            err_box_img_stable = self.filt_err_x

            err_handle_img = 0.0
            err_handle_box = 0.0

            if box_target and handle_target:
                hx, _ = handle_target['center']
                bx, _ = box_target['center']

                err_handle_img = (hx - rov_cx) / float(max(1.0, w / 2))
                err_handle_img = float(np.clip(err_handle_img, -1.0, 1.0))

                err_handle_box = (hx - bx) / float(max(1.0, w / 2))
                err_handle_box = float(np.clip(err_handle_box, -1.0, 1.0))

            # --- DEPTH PD ---
            now = current_time

            if self.depth_filtered is None:
                self.depth_filtered = float(self.current_depth)
                self.last_depth_filtered = self.depth_filtered
                self.last_depth_time = now

            dt = max(1e-3, now - self.last_depth_time) if self.last_depth_time is not None else 1e-3

            self.depth_filtered = (self.alpha_depth_filter * float(self.current_depth)) + \
                                  ((1.0 - self.alpha_depth_filter) * self.depth_filtered)

            self.depth_vel = (self.depth_filtered - self.last_depth_filtered) / dt
            self.last_depth_filtered = self.depth_filtered
            self.last_depth_time = now

            depth_error = float(self.target_depth) - float(self.depth_filtered)

            if not self.depth_achieved:
                if abs(depth_error) < self.strict_tol:
                    self.depth_achieved = True
            else:
                if abs(depth_error) > self.loose_tol:
                    self.depth_achieved = False

            if self.mission_state in ["LUNGE_READY"]:
                kp = 0.15
                local_deadband = 0.08
                kd = 0.4
            else:
                kp = self.kp_depth_up if depth_error > 0 else self.kp_depth_down
                local_deadband = self.deadband
                kd = self.kd_depth

            z_cmd = (depth_error * kp) - (kd * self.depth_vel)

            if depth_error <= 0:
                z_cmd += self.buoyancy_offset

            if abs(depth_error) < local_deadband:
                z_cmd = self.bottom_lift
                self.depth_achieved = True

            if self.depth_filtered < (self.target_depth - self.bottom_buffer):
                if z_cmd < 0:
                    z_cmd = 0.0

            z_cmd = max(z_cmd, self.bottom_lift)
            cmd.linear.z = float(np.clip(z_cmd, -self.max_vert_speed, self.max_vert_speed))

            # --- STATE TRANSITIONS ---
            if self.mission_state == "DIVING" and self.depth_achieved and box_target:
                self.mission_state = "APPROACHING"
                self.approach_stable_start_time = 0.0
                self.is_squared = False
                self.publish_state()

            # --- CONTROL LOGIC ---
            if self.mission_state == "DIVING":
                if box_target:
                    cmd.angular.z = float(np.clip(err_box_img_stable * self.kp_yaw_diving, -0.45, 0.45))
                    cmd.linear.y = 0.0
                else:
                    cmd.angular.z = 0.0
                    cmd.linear.y = 0.0

            elif self.mission_state == "APPROACHING":
                if box_target and handle_target:
                    # 1. CHECK SQUARENESS
                    if abs(err_handle_box) < self.squaring_deadband:
                        self.is_squared = True
                    else:
                        self.is_squared = False

                    if self.is_squared:
                        # MODE A: HANDLE LOCK
                        cmd.angular.z = float(np.clip(err_handle_img * self.kp_yaw_lock, -0.45, 0.45))
                        cmd.linear.y = float(np.clip(err_handle_img * self.kp_sway_lock, -0.45, 0.45))
                    else:
                        # MODE B: ORBIT
                        cmd.angular.z = float(np.clip(err_handle_img * self.kp_yaw_orbit, -0.45, 0.45))
                        cmd.linear.y = float(np.clip(err_handle_box * self.kp_sway_orbit, -0.45, 0.45))
                elif box_target:
                    # Fallback
                    cmd.linear.y = float(np.clip(err_box_img_stable * self.kp_sway_fallback, -0.45, 0.45))
                    cmd.angular.z = 0.0
                else:
                    cmd.linear.y = 0.0
                    cmd.angular.z = 0.0

            # --- FORWARD / DISTANCE CONTROL ---
            area_error = float(self.target_box_area) - box_ratio_ema
            can_move_forward = True

            if self.mission_state == "APPROACHING" and handle_target and not self.is_squared:
                can_move_forward = False

            if self.mission_state == "DIVING":
                cmd.linear.x = 0.0

            elif self.mission_state == "APPROACHING":
                if area_error > self.area_deadband:
                    if can_move_forward:
                        speed = (self.area_to_speed_scale * area_error)
                        speed = float(np.clip(speed, 0.0, self.max_fwd_speed))
                        if speed < self.min_fwd_crawl:
                            speed = self.min_fwd_crawl
                        cmd.linear.x = speed
                    else:
                        cmd.linear.x = 0.0
                        self.approach_stable_start_time = 0.0

                elif abs(area_error) <= self.area_deadband:
                    # In Zone
                    cmd.linear.x = 0.0
                    if self.approach_stable_start_time == 0.0:
                        self.approach_stable_start_time = current_time

                    # Short stability check (0.2s) just to filter noise
                    if (current_time - self.approach_stable_start_time) >= self.approach_stable_duration:
                        # === INSTANT LUNGE LOGIC ===
                        if self.is_squared:
                            # We are close AND squared. FIRE IMMEDIATELY.
                            self.mission_state = "LUNGING"
                            self.lunge_start_time = current_time
                            self.publish_state()
                        else:
                            # We are close but NOT squared. Go to ready to fix angle.
                            self.mission_state = "LUNGE_READY"
                            self.approach_stable_start_time = 0.0
                            self.publish_state()
                else:
                    # Back Up
                    overshoot = -area_error
                    backoff = min(
                        self.max_backoff_speed,
                        (overshoot / max(1e-6, self.area_overshoot_tol)) * self.max_backoff_speed
                    )
                    backoff = float(np.clip(backoff, 0.01, self.max_backoff_speed))
                    if backoff < self.min_back_crawl:
                        backoff = self.min_back_crawl
                    cmd.linear.x = -backoff
                    self.approach_stable_start_time = 0.0

            # --- LUNGE_READY (FIXING ALIGNMENT THEN FIRING) ---
            if self.mission_state == "LUNGE_READY":
                # 1. DISTANCE HOLDING
                if area_error > self.area_deadband:
                    dist_speed = float(np.clip(area_error * self.kp_dist_hold, 0.0, 0.08))
                    if dist_speed < self.min_fwd_crawl:
                        dist_speed = self.min_fwd_crawl
                    cmd.linear.x = dist_speed
                elif area_error < -self.area_deadband:
                    dist_speed = float(np.clip(area_error * self.kp_dist_hold, -0.1, 0.0))
                    if abs(dist_speed) < self.min_back_crawl:
                        dist_speed = -self.min_back_crawl
                    cmd.linear.x = dist_speed
                else:
                    cmd.linear.x = 0.0

                ready_to_fire = False

                if box_target and handle_target:
                    hx, _ = handle_target['center']
                    bx, _ = box_target['center']

                    err_handle_img = (hx - rov_cx) / float(max(1.0, w / 2))
                    err_handle_img = float(np.clip(err_handle_img, -1.0, 1.0))

                    err_handle_box = (hx - bx) / float(max(1.0, w / 2))
                    err_handle_box = float(np.clip(err_handle_box, -1.0, 1.0))

                    cmd.angular.z = float(np.clip(err_handle_img * self.kp_yaw_hold, -0.2, 0.2))
                    cmd.linear.y = float(np.clip(err_handle_box * self.kp_lat_hold, -0.45, 0.45))

                    is_distance_good = abs(area_error) <= (self.area_deadband * 2.0)
                    is_aligned = abs(err_handle_box) <= self.handle_align_tol

                    if is_aligned and is_distance_good:
                        ready_to_fire = True
                else:
                    cmd.angular.z = 0.0
                    cmd.linear.y = 0.0
                    ready_to_fire = False

                # INSTANT FIRE - NO COUNTDOWN
                if ready_to_fire:
                    self.mission_state = "LUNGING"
                    self.lunge_start_time = current_time
                    self.publish_state()

            # --- LUNGING ---
            if self.mission_state == "LUNGING":
                cmd.linear.x = float(self.lunge_speed)
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0

                if (current_time - self.lunge_start_time) > self.lunge_duration:
                    self.mission_state = "COMPLETE"
                    self.publish_state()

            elif self.mission_state == "COMPLETE":
                cmd = Twist()

            elif not self.depth_achieved and self.mission_state not in ["IDLE", "DIVING"]:
                self.mission_state = "DIVING"
                cmd.linear.x = 0.0
                self.approach_stable_start_time = 0.0
                self.publish_state()

            # --- publish cmd_vel ---
            if self.auto_active:
                if self.mission_state == "COMPLETE":
                    self.cmd_pub.publish(Twist())
                else:
                    self.cmd_pub.publish(cmd)
            else:
                self.cmd_pub.publish(Twist())

            # --- VISUALIZATION ---
            if active_target:
                try:
                    if active_target.get('coords') is not None:
                        ax1, ay1, ax2, ay2 = map(int, active_target['coords'])
                        cv2.rectangle(frame, (ax1, ay1), (ax2, ay2), display_color, 2)
                except Exception:
                    pass

            if box_target:
                bx, by = box_target['center']
                cv2.circle(frame, (bx, by), 5, (0, 0, 255), -1)

            if handle_target:
                hx, hy = handle_target['center']
                cv2.circle(frame, (hx, hy), 8, (0, 255, 255), 2)
                cv2.putText(frame, "HANDLE", (hx + 10, hy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                if box_target:
                    cv2.line(frame, (bx, by), (hx, hy), (0, 255, 0), 2)

            s_color = (0, 255, 0)
            if self.mission_state == "LUNGING":
                s_color = (0, 0, 255)
            if self.mission_state == "COMPLETE":
                s_color = (255, 255, 0)

            cv2.putText(frame, f"STATE: {self.mission_state}",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, s_color, 2)

            if not can_move_forward and self.mission_state == "APPROACHING":
                cv2.putText(frame, "HALTED: SQUARING UP", (20, 190),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            if self.is_squared and self.mission_state == "APPROACHING":
                cv2.putText(frame, "LOCKED: HANDLE TARGETING", (20, 190),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            d_color = (0, 255, 0) if self.depth_achieved else (0, 0, 255)
            cv2.putText(frame, f"Depth: {self.current_depth:.2f}m",
                        (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, d_color, 2)

            cv2.putText(frame,
                        f"BoxR: {box_ratio:.4f} EMA:{box_ratio_ema:.4f} rate:{self.box_ratio_rate:.4f}",
                        (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)

            if self.auto_active:
                cmd_str = f"Lat:{cmd.linear.y:.2f} Yaw:{cmd.angular.z:.2f} Fwd:{cmd.linear.x:.2f}"
                cv2.putText(frame, cmd_str, (20, 160),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            cv2.imshow("BlueROV Vision Debug", frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error in callback: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = BlueROVVisionNode()
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

