#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from ultralytics import YOLO

class BlueROVVisionNode(Node):
    def __init__(self):
        super().__init__('bluerov_vision_node')
        
        # --- CONFIGURATION ---
        self.model_path = '/home/elex/mainproj/v8_seg/runs/segment/bluerov_handle_12802/weights/best.pt'
        self.camera_topic = '/bluerov2/camera/image_raw'
        
        # --- TOOL OFFSET (ADJUSTED) ---
        # Was -0.15 (Left), Now -0.08 (Slightly more Right/Center)
        self.tool_offset_x = -0.08  
        
        # --- MISSION PARAMETERS ---
        self.target_depth = -4.65        
        self.buoyancy_offset = -0.10     
        
        # --- DETECTION ---
        # 1. STOPPING POINT (Runway Start)
        self.target_box_area = 0.04      
        
        # 2. TRANSITION POINT (The Fix)
        # Start tracking handle much earlier (when box is smaller/further)
        # This gives the robot more distance to align laterally.
        self.transition_threshold = 0.01
        
        self.smooth_alpha = 0.15         
        
        # --- GAINS ---
        self.kp_depth_up = 1.0      
        self.kp_depth_down = 0.3    
        
        # Strong Alignment Gains
        self.kp_yaw = 0.5    
        self.kp_lat = 0.65   
        
        # --- SPEEDS ---
        self.max_vert_speed = 0.30 
        self.max_fwd_speed  = 0.15 
        self.min_dive_thrust = -0.15 
        
        # --- ADAPTIVE APPROACH ---
        self.max_approach_error_tol = 0.25 
        
        # --- LUNGE SETTINGS ---
        self.enable_lunge = True         
        self.lunge_duration = 2.5        
        self.lunge_speed = 0.45          
        self.stability_required_time = 4.0 
        self.alignment_tolerance = 0.10  
        
        # --- Filter & PD Setup ---
        self.depth_filtered = None
        self.last_depth_filtered = None
        self.last_depth_time = None
        self.depth_vel = 0.0
        self.alpha_depth_filter = 0.25   
        self.kd_depth = 0.8              
        self.bottom_buffer = 0.12        
        self.deadband = 0.03             
        self.bottom_lift = 0.06          
        self.strict_tol = 0.06
        self.loose_tol = 0.30

        # Load YOLO
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
        
        # State Variables
        self.current_depth = 0.0
        self.auto_active = False
        self.depth_achieved = False 
        self.last_handle_time = 0.0
        self.last_handle_pos = None 
        self.locked_mode = "BOX" 
        self.filt_target_x = 0
        
        # Mission State
        self.mission_state = "IDLE"
        self.stable_start_time = 0.0
        self.lunge_start_time = 0.0
        
        self.get_logger().info(f'Vision Control Ready. Start Detect: {self.transition_threshold} | Offset: {self.tool_offset_x}')

    def auto_state_callback(self, msg):
        self.auto_active = msg.data
        if msg.data:
            self.get_logger().info(">>> AUTO ENGAGED <<<")
        else:
            self.get_logger().info("<<< MANUAL MODE - RESET <<<")
            self.depth_achieved = False 
            self.mission_state = "IDLE"
            self.stable_start_time = 0.0
            self.lunge_start_time = 0.0

    def depth_callback(self, msg):
        self.current_depth = msg.data

    def select_best_target(self, detections, image_center):
        if not detections: return None
        best_target = None
        min_distance = float('inf')
        cx, cy = image_center
        for det in detections:
            tx, ty = det['center']
            dist = np.sqrt((tx - cx)**2 + (ty - cy)**2)
            if dist < min_distance:
                min_distance = dist
                best_target = det
        return best_target

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w, _ = frame.shape
            img_area = h * w
            
            rov_cx, rov_cy = w // 2, h // 2
            tool_cx = int(rov_cx + (self.tool_offset_x * (w / 2)))

            if self.filt_target_x == 0: self.filt_target_x = tool_cx

            # --- 1. DETECTION ---
            results = self.model(frame, verbose=False)
            raw_boxes = []
            raw_handles = []
            if results[0].boxes:
                for box in results[0].boxes:
                    coords = box.xyxy[0].cpu().numpy()
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    cx = int((coords[0] + coords[2]) / 2)
                    cy = int((coords[1] + coords[3]) / 2)
                    width = coords[2] - coords[0]
                    height = coords[3] - coords[1]
                    area = width * height
                    det_data = {'coords': coords, 'center': (cx, cy), 'conf': conf, 'area': area}
                    if conf < 0.25: continue 
                    if cls_id == 1: raw_handles.append(det_data)
                    elif cls_id == 0: raw_boxes.append(det_data)

            box_target = self.select_best_target(raw_boxes, (rov_cx, rov_cy))
            handle_target = self.select_best_target(raw_handles, (rov_cx, rov_cy))

            # --- 2. LOGIC ---
            box_ratio = 0.0
            if box_target: box_ratio = box_target['area'] / img_area

            active_target = None
            close_enough = box_ratio > self.transition_threshold

            current_time = time.time()
            if close_enough and handle_target:
                self.last_handle_time = current_time
                self.last_handle_pos = handle_target['center']
                self.locked_mode = "HANDLE"
                active_target = handle_target
                display_color = (0, 255, 0)
            elif close_enough and (current_time - self.last_handle_time) < 1.0 and self.last_handle_pos is not None:
                self.locked_mode = "HANDLE_GHOST"
                active_target = {'center': self.last_handle_pos, 'coords': None, 'area': 0} 
                display_color = (0, 255, 255)
            else:
                self.locked_mode = "BOX"
                active_target = box_target
                display_color = (255, 100, 0)

            # --- 3. SMOOTHING ---
            target_x = tool_cx 
            if active_target: target_x, _ = active_target['center']
            
            self.filt_target_x = (self.smooth_alpha * target_x) + ((1 - self.smooth_alpha) * self.filt_target_x)
            
            # ============================================
            # STEP 4: CONTROL
            # ============================================
            cmd = Twist()
            err_x = (self.filt_target_x - tool_cx) / (w / 2) 
            
            # --- DEPTH PD ---
            now = current_time
            if self.depth_filtered is None:
                self.depth_filtered = self.current_depth
                self.last_depth_filtered = self.depth_filtered
                self.last_depth_time = now

            dt = max(1e-3, now - self.last_depth_time) if self.last_depth_time is not None else 1e-3
            self.depth_filtered = (self.alpha_depth_filter * self.current_depth) + ((1.0 - self.alpha_depth_filter) * self.depth_filtered)
            self.depth_vel = (self.depth_filtered - self.last_depth_filtered) / dt
            self.last_depth_filtered = self.depth_filtered
            self.last_depth_time = now

            depth_error = self.target_depth - self.depth_filtered

            if not self.depth_achieved:
                if abs(depth_error) < self.strict_tol: self.depth_achieved = True
            else:
                if abs(depth_error) > self.loose_tol: self.depth_achieved = False

            if depth_error > 0: kp = self.kp_depth_up
            else: kp = self.kp_depth_down

            z_cmd = (depth_error * kp) - (self.kd_depth * self.depth_vel)
            if depth_error <= 0: z_cmd += self.buoyancy_offset

            if abs(depth_error) < self.deadband:
                z_cmd = self.bottom_lift
                self.depth_achieved = True

            if self.depth_filtered < (self.target_depth - self.bottom_buffer):
                if z_cmd < 0: z_cmd = 0.0
                z_cmd = max(z_cmd, self.bottom_lift)

            cmd.linear.z = float(np.clip(z_cmd, -self.max_vert_speed, self.max_vert_speed))

            # --- ALIGNMENT ---
            if active_target:
                cmd.angular.z = float(np.clip(err_x * self.kp_yaw, -0.45, 0.45))
                cmd.linear.y = float(np.clip(err_x * self.kp_lat, -0.45, 0.45))

            # --- STATE MACHINE ---
            if self.mission_state == "LUNGING":
                cmd.linear.x = self.lunge_speed 
                cmd.linear.y = 0.0 
                cmd.angular.z = 0.0 
                if (current_time - self.lunge_start_time) > self.lunge_duration:
                    self.mission_state = "COMPLETE"

            elif self.mission_state == "COMPLETE":
                self.cmd_pub.publish(Twist())

            elif not self.depth_achieved and self.locked_mode == "BOX":
                self.mission_state = "DIVING"
                cmd.linear.x = 0.0
                self.stable_start_time = 0.0

            else:
                if box_ratio < self.target_box_area:
                    self.mission_state = "APPROACHING"
                    self.stable_start_time = 0.0
                    
                    # ADAPTIVE APPROACH SPEED
                    abs_err = abs(err_x)
                    if abs_err < self.max_approach_error_tol:
                        speed_factor = 1.0 - (abs_err / self.max_approach_error_tol)
                        fwd_cmd = self.max_fwd_speed * speed_factor
                        if fwd_cmd < 0.05: fwd_cmd = 0.0 
                        cmd.linear.x = float(fwd_cmd)
                    else:
                        cmd.linear.x = 0.0
                else:
                    self.mission_state = "STATION_KEEPING"
                    cmd.linear.x = 0.0 
                    
                    is_aligned = abs(err_x) < self.alignment_tolerance
                    if is_aligned:
                        if self.stable_start_time == 0.0:
                            self.stable_start_time = current_time
                        elif (current_time - self.stable_start_time) > self.stability_required_time:
                            if self.enable_lunge:
                                self.mission_state = "LUNGING"
                                self.lunge_start_time = current_time
                                self.get_logger().info("!!! LUNGING !!!")
                    else:
                        self.stable_start_time = 0.0

            if self.auto_active and self.mission_state != "COMPLETE":
                self.cmd_pub.publish(cmd)
            elif self.auto_active and self.mission_state == "COMPLETE":
                self.cmd_pub.publish(Twist())
            else:
                self.cmd_pub.publish(Twist())

            # ============================================
            # VISUALIZATION
            # ============================================
            if active_target:
                try:
                    if active_target.get('coords') is not None:
                        ax1, ay1, ax2, ay2 = map(int, active_target['coords'])
                        cv2.rectangle(frame, (ax1, ay1), (ax2, ay2), display_color, 2)
                except: pass

                draw_x = int(self.filt_target_x)
                cv2.line(frame, (rov_cx, 0), (rov_cx, h), (100, 100, 100), 1)
                cv2.line(frame, (tool_cx, 0), (tool_cx, h), (255, 0, 0), 2)
                cv2.circle(frame, (draw_x, h//2), 5, (0, 0, 255), -1)

                s_color = (0,255,0)
                if self.mission_state == "LUNGING": s_color = (0,0,255) 
                if self.mission_state == "COMPLETE": s_color = (255,255,0) 

                cv2.putText(frame, f"STATE: {self.mission_state}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, s_color, 2)
                
                if self.mission_state == "STATION_KEEPING":
                     if self.stable_start_time > 0:
                         time_left = self.stability_required_time - (current_time - self.stable_start_time)
                         time_text = f"LUNGE IN: {time_left:.1f}s"
                         col = (0,0,255)
                     else:
                         time_text = "ALIGNING..."
                         col = (0,255,255) 
                     cv2.putText(frame, time_text, (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)

                d_color = (0, 255, 0) if self.depth_achieved else (0, 0, 255)
                cv2.putText(frame, f"Depth: {self.current_depth:.2f}m", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, d_color, 2)
                
                if self.auto_active:
                     cmd_str = f"Lat:{cmd.linear.y:.2f} Fwd:{cmd.linear.x:.2f}"
                     cv2.putText(frame, cmd_str, (20, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)

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
