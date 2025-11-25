#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import math

# ==============================================================================
# SCENARIO & CALIBRATION 
SCENARIO_MODE = "VERTICAL"  # "HORIZONTAL" or "VERTICAL"

# --- OFFSETS (Pixels from Center) ---
GRIPPER_OFFSET_X = 0    
GRIPPER_OFFSET_Y = 80   

# --- VERTICAL MODE TILT SETTINGS ---
TILT_SPEED = 0.2        # Positive = Pitch Nose Down

# When nose is down + thrusting down = backward drift
# So we apply FORWARD thrust to compensate
TILT_COUNTER_SURGE = 0.15  # Positive = Forward compensation

# --- VELOCITY LIMITS ---
MAX_LINEAR_VEL = 0.5    # m/s
MAX_ANGULAR_VEL = 0.5   # rad/s

# --- DETECTION TIMEOUT ---
DETECTION_TIMEOUT = 2.0  # seconds
# ==============================================================================

class FinalServoNode(Node):
    def __init__(self):
        super().__init__('visual_servo_node')
        self.get_logger().info(f"ARMED: {SCENARIO_MODE} | Counter-Surge: +{TILT_COUNTER_SURGE}")

        self.MODEL_PATH = '/home/elex/mainproj/v8_seg/runs/segment/bluerov_handle_12802/weights/best.pt'
        
        # Control Gains
        self.KX = 0.002       
        self.KY = 0.0025      
        self.KZ = 0.000005    
        self.KW = 0.01        

        # State Machine Thresholds
        self.AREA_COMMITMENT = 45000   
        self.ALIGN_TOLERANCE = 40      
        self.LUNGE_TIME = 4.0          
        self.LUNGE_POWER = 0.5         
        
        # Vertical Mode Aiming
        self.VERTICAL_AIM_OFFSET = 120 

        # ROS Setup
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(Bool, '/rov/servo_mode_active', self.mode_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Detection Watchdog
        self.last_detection_time = 0.0
        self.last_valid_cmd = Twist()
        self.detection_lost = False

        # State Machine
        self.active = False
        self.state = "IDLE"
        self.lunge_start = 0.0

        # Vision
        self.bridge = CvBridge()
        self.model = YOLO(self.MODEL_PATH)

    def mode_callback(self, msg: Bool):
        if msg.data and not self.active:
            self.active = True
            self.state = "APPROACH"
            self.last_detection_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info("✅ AUTO ENGAGED")
        elif not msg.data and self.active:
            self.active = False
            self.state = "IDLE"
            self.cmd_pub.publish(Twist())
            self.get_logger().info("⏸️ AUTO DISENGAGED")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        H, W = cv_image.shape[:2]
        results = self.model(cv_image, verbose=False, conf=0.5)
        box, handle = self._parse_results(results[0], cv_image)

        cmd = Twist()
        if self.active:
            cmd = self.control_loop(box, handle, W, H)
            self.cmd_pub.publish(cmd)

        self.draw_hud(cv_image, box, handle)
        cv2.imshow("ROV View", cv_image)
        cv2.waitKey(1)
    
    def control_loop(self, box, handle, W, H):
        cmd = Twist()
        now = self.get_clock().now().nanoseconds / 1e9
        
        CX, CY = W/2, H/2
        TargetX = CX + GRIPPER_OFFSET_X
        TargetY = CY + GRIPPER_OFFSET_Y
        
        if SCENARIO_MODE == "VERTICAL":
            TargetY += self.VERTICAL_AIM_OFFSET

        # ==================== PHASE 1: ALIGN & APPROACH ====================
        if self.state == "APPROACH":
            target = handle if handle else box 
            
            # Detection Loss Handler
            if not target:
                if (now - self.last_detection_time) > DETECTION_TIMEOUT:
                    if not self.detection_lost:
                        self.get_logger().warn("🔴 DETECTION LOST - HOLDING POSITION")
                        self.detection_lost = True
                    return Twist()  # Full stop
                else:
                    # Brief loss - hold last command
                    return self.last_valid_cmd
            
            # Detection Active
            self.last_detection_time = now
            self.detection_lost = False

            tx, ty, tw, th = target['bbox']
            area = tw * th
            angle = target.get('angle', 0.0)

            # Proportional Control
            cmd.linear.y = -self.KX * (tx - TargetX)  # Sway
            cmd.linear.z = -self.KY * (ty - TargetY)  # Heave
            cmd.angular.z = self.KW * (-angle)        # Yaw alignment
            
            err_dist = self.AREA_COMMITMENT - area
            cmd.linear.x = self.KZ * err_dist         # Surge (approach)

            # Velocity Saturation
            cmd = self._saturate_velocities(cmd)

            # Lock Check
            aligned_x = abs(tx - TargetX) < self.ALIGN_TOLERANCE
            aligned_y = abs(ty - TargetY) < self.ALIGN_TOLERANCE
            close_enough = area >= self.AREA_COMMITMENT

            if aligned_x and aligned_y and close_enough:
                self.state = "LUNGE"
                self.lunge_start = now
                self.get_logger().info("🎯 LOCKED - LUNGING!")

            self.last_valid_cmd = cmd

        # ==================== PHASE 2: LUNGE ====================
        elif self.state == "LUNGE":
            if (now - self.lunge_start) > self.LUNGE_TIME:
                self.state = "DONE"
                self.get_logger().info("✅ LUNGE COMPLETE")
                return Twist()

            if SCENARIO_MODE == "HORIZONTAL":
                cmd.linear.x = self.LUNGE_POWER 

            elif SCENARIO_MODE == "VERTICAL":
                # 1. DROP DOWN
                cmd.linear.z = -self.LUNGE_POWER 
                
                # 2. TILT NOSE DOWN
                cmd.angular.y = TILT_SPEED 

                # 3. COMPENSATE FORWARD (FIXED!)
                # Pitched down + heave down = backward drift
                # Apply forward thrust to keep drop vertical
                cmd.linear.x = TILT_COUNTER_SURGE  # POSITIVE (forward)

            cmd = self._saturate_velocities(cmd)
            self.last_valid_cmd = cmd

        elif self.state == "DONE":
            cmd = Twist()  # Hold position (zero velocity)

        return cmd

    def _saturate_velocities(self, cmd):
        """Apply velocity limits for safety"""
        cmd.linear.x = np.clip(cmd.linear.x, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        cmd.linear.y = np.clip(cmd.linear.y, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        cmd.linear.z = np.clip(cmd.linear.z, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        cmd.angular.y = np.clip(cmd.angular.y, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
        cmd.angular.z = np.clip(cmd.angular.z, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
        return cmd

    def _parse_results(self, result, img):
        """Extract box and handle detections from YOLO results"""
        box, handle = None, None
        if result.boxes:
            for i, cls in enumerate(result.boxes.cls):
                xywh = result.boxes.xywh[i].cpu().numpy()
                mask = None
                if result.masks:
                    m = result.masks.data[i].cpu().numpy()
                    m = cv2.resize(m, (img.shape[1], img.shape[0]))
                    mask = (m * 255).astype(np.uint8)
                
                data = {'bbox': xywh, 'mask': mask, 'angle': 0.0}
                
                if int(cls) == 0:  # Box
                    box = data
                elif int(cls) == 1:  # Handle
                    if mask is not None:
                        data['angle'], _ = self.get_orientation(mask)
                    handle = data
        return box, handle

    def get_orientation(self, mask):
        """Compute handle orientation using PCA"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: 
            return 0.0, (0, 0)
        
        pts = max(contours, key=cv2.contourArea).reshape(-1, 2).astype(np.float64)
        if len(pts) < 2: 
            return 0.0, (0, 0)
        
        mean, evecs, _ = cv2.PCACompute2(pts, mean=None)
        angle = math.degrees(math.atan2(evecs[0][1], evecs[0][0]))
        
        # Normalize to [-90, 90]
        if angle < -90: 
            angle += 180
        elif angle > 90: 
            angle -= 180
        
        return angle, (0, 0)

    def draw_hud(self, img, box, handle):
        """Draw visual debugging overlay"""
        H, W = img.shape[:2]
        CX, CY = W//2, H//2
        AimX = int(CX + GRIPPER_OFFSET_X)
        AimY = int(CY + GRIPPER_OFFSET_Y)
        
        if SCENARIO_MODE == "VERTICAL": 
            AimY += self.VERTICAL_AIM_OFFSET

        # Center crosshair (green)
        cv2.circle(img, (CX, CY), 5, (0, 255, 0), 1)
        
        # Target reticle (magenta)
        cv2.circle(img, (AimX, AimY), 10, (255, 0, 255), 2)
        cv2.line(img, (AimX-20, AimY), (AimX+20, AimY), (255, 0, 255), 2)
        cv2.line(img, (AimX, AimY-20), (AimX, AimY+20), (255, 0, 255), 2)

        # Box detection (blue)
        if box:
            x, y, w, h = map(int, box['bbox'])
            cv2.rectangle(img, (x-w//2, y-h//2), (x+w//2, y+h//2), (255, 0, 0), 2)
            cv2.putText(img, "BOX", (x-w//2, y-h//2-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
        # Handle detection (green)
        if handle:
            x, y, w, h = map(int, handle['bbox'])
            cv2.rectangle(img, (x-w//2, y-h//2), (x+w//2, y+h//2), (0, 255, 0), 2)
            cv2.line(img, (x, y), (AimX, AimY), (0, 255, 255), 1)  # Tracking line
            
            angle = handle.get('angle', 0.0)
            cv2.putText(img, f"HANDLE {angle:.1f}°", (x-w//2, y-h//2-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # State & Status Display
        status_color = (0, 255, 0) if self.active else (128, 128, 128)
        cv2.putText(img, f"STATE: {self.state}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        cv2.putText(img, f"MODE: {SCENARIO_MODE}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        if self.detection_lost:
            cv2.putText(img, "⚠️ DETECTION LOST", (W//2-150, H-30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

def main(args=None):
    rclpy.init(args=args)
    node = FinalServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("🛑 Shutting down...")
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

