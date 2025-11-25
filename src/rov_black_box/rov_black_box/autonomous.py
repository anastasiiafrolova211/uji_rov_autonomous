#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import math
import os

# ==============================================================================
# ⚠️ SCENARIO & CALIBRATION ⚠️
SCENARIO_MODE = "VERTICAL" 

GRIPPER_OFFSET_X = 0    
GRIPPER_OFFSET_Y = 80   

TILT_SPEED = 0.2        
TILT_COUNTER_SURGE = 0.15 

MAX_LINEAR_VEL = 0.3    
MAX_ANGULAR_VEL = 0.5   
DETECTION_TIMEOUT = 1.0 
# ==============================================================================

class FinalServoNode(Node):
    def __init__(self):
        super().__init__('visual_servo_node')
        self.get_logger().info(f"🚀 ARMED: {SCENARIO_MODE} | Waiting for Auto...")

        self.MODEL_PATH = '/home/elex/mainproj/v8_seg/runs/segment/bluerov_handle_12802/weights/best.pt'
        
        # Gains & Thresholds
        self.KX = 0.002       
        self.KY = 0.0025      
        self.KZ = 0.000005    
        self.KW = 0.01        
        self.AREA_COMMITMENT = 45000   
        self.ALIGN_TOLERANCE = 40      
        self.LUNGE_TIME = 4.0          
        self.LUNGE_POWER = 0.5         
        self.VERTICAL_AIM_OFFSET = 120 

        # --- RETRY SETTINGS ---
        self.BACKOFF_TIME = 3.0       # Seconds to reverse before trying again
        self.BACKOFF_POWER = -0.3     # Reverse speed

        # ROS Setup
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(Bool, '/rov/servo_mode_active', self.mode_callback, 10)
        
        # NEW: Confirmation Subscriber (True = Success, False = Retry)
        self.create_subscription(Bool, '/rov/mission_confirmation', self.confirm_callback, 10)
        
        # NEW: Status Publisher (To trigger your Dashboard Popup)
        self.status_pub = self.create_publisher(String, '/rov/mission_status', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.last_detection_time = 0.0
        self.last_valid_cmd = Twist()
        self.detection_lost = False

        self.active = False
        self.state = "IDLE"
        self.timer_start = 0.0
        
        # Confirmation variables
        self.waiting_for_user = False

        self.bridge = CvBridge()
        if not os.path.exists(self.MODEL_PATH): self.get_logger().error("Model missing")
        self.model = YOLO(self.MODEL_PATH)

    def mode_callback(self, msg: Bool):
        """Master Auto Switch"""
        if msg.data and not self.active:
            self.active = True
            self.state = "APPROACH"
            self.last_detection_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info("✅ AUTO ENGAGED")
            self.status_pub.publish(String(data="APPROACHING"))
        elif not msg.data and self.active:
            self.active = False
            self.state = "IDLE"
            self.cmd_pub.publish(Twist())
            self.get_logger().info("⏸️ AUTO DISENGAGED")
            self.status_pub.publish(String(data="MANUAL"))

    def confirm_callback(self, msg: Bool):
        """Receives YES/NO from Dashboard"""
        if self.state == "WAIT_FOR_USER":
            if msg.data is True:
                # USER PRESSED YES
                self.get_logger().info("🎉 CONFIRMED SUCCESS!")
                self.state = "DONE"
                self.status_pub.publish(String(data="SUCCESS"))
            else:
                # USER PRESSED NO
                self.get_logger().info("🔄 RETRY REQUESTED - BACKING OFF")
                self.state = "BACKOFF"
                self.timer_start = self.get_clock().now().nanoseconds / 1e9
                self.status_pub.publish(String(data="RETRYING"))

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

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
        if SCENARIO_MODE == "VERTICAL": TargetY += self.VERTICAL_AIM_OFFSET

        # ==================== PHASE 1: ALIGN & APPROACH ====================
        if self.state == "APPROACH":
            target = handle if handle else box 
            if not target:
                if (now - self.last_detection_time) > DETECTION_TIMEOUT:
                    return Twist() 
                return self.last_valid_cmd
            
            self.last_detection_time = now
            tx, ty, tw, th = target['bbox']
            area = tw * th
            angle = target.get('angle', 0.0)

            cmd.linear.y = -self.KX * (tx - TargetX) 
            cmd.linear.z = -self.KY * (ty - TargetY) 
            cmd.angular.z = self.KW * (-angle)       
            cmd.linear.x = self.KZ * (self.AREA_COMMITMENT - area)         
            
            cmd = self._saturate_velocities(cmd)

            if abs(tx - TargetX) < self.ALIGN_TOLERANCE and \
               abs(ty - TargetY) < self.ALIGN_TOLERANCE and \
               area >= self.AREA_COMMITMENT:
                
                self.state = "LUNGE"
                self.timer_start = now
                self.get_logger().info("🎯 LOCKED - LUNGING!")
                self.status_pub.publish(String(data="LUNGING"))

            self.last_valid_cmd = cmd

        # ==================== PHASE 2: LUNGE ====================
        elif self.state == "LUNGE":
            if (now - self.timer_start) > self.LUNGE_TIME:
                # LUNGE FINISHED -> ASK USER
                self.state = "WAIT_FOR_USER"
                self.get_logger().info("❓ LUNGE DONE - WAITING FOR CONFIRMATION")
                self.status_pub.publish(String(data="CONFIRM_ATTACH")) # <--- TRIGGER POPUP
                return Twist()

            if SCENARIO_MODE == "HORIZONTAL":
                cmd.linear.x = self.LUNGE_POWER 
            elif SCENARIO_MODE == "VERTICAL":
                cmd.linear.z = -self.LUNGE_POWER 
                cmd.angular.y = TILT_SPEED 
                cmd.linear.x = TILT_COUNTER_SURGE 

            self.last_valid_cmd = cmd

        # ==================== PHASE 3: WAIT FOR USER ====================
        elif self.state == "WAIT_FOR_USER":
            # Hover in place (Zero velocity) while waiting for dashboard button
            cmd = Twist() 

        # ==================== PHASE 4: BACKOFF (RETRY) ====================
        elif self.state == "BACKOFF":
            if (now - self.timer_start) > self.BACKOFF_TIME:
                self.state = "APPROACH"
                self.get_logger().info("🔄 BACKOFF COMPLETE - RESTARTING APPROACH")
                self.status_pub.publish(String(data="APPROACHING"))
                return Twist()
            
            # Move Backwards to reset view
            cmd.linear.x = self.BACKOFF_POWER # -0.3 m/s
            
            # If we were in Vertical mode, maybe go Up a little too?
            if SCENARIO_MODE == "VERTICAL":
                cmd.linear.z = 0.2 # Go Up

        # ==================== PHASE 5: DONE ====================
        elif self.state == "DONE":
            cmd = Twist() 

        return cmd

    def _saturate_velocities(self, cmd):
        cmd.linear.x = np.clip(cmd.linear.x, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        cmd.linear.y = np.clip(cmd.linear.y, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        cmd.linear.z = np.clip(cmd.linear.z, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        cmd.angular.y = np.clip(cmd.angular.y, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
        cmd.angular.z = np.clip(cmd.angular.z, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
        return cmd

    def _parse_results(self, result, img):
        # ... (Same as previous code) ...
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
                if int(cls) == 0: box = data
                elif int(cls) == 1: 
                    if mask is not None: data['angle'] = self.get_orientation(mask)
                    handle = data
        return box, handle

    def get_orientation(self, mask):
        # ... (Same as previous code) ...
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return 0.0
        pts = max(contours, key=cv2.contourArea).reshape(-1, 2).astype(np.float64)
        if len(pts) < 2: return 0.0
        mean, evecs, _ = cv2.PCACompute2(pts, mean=None)
        angle = math.degrees(math.atan2(evecs[0][1], evecs[0][0]))
        if angle < -90: angle += 180
        elif angle > 90: angle -= 180
        return angle

    def draw_hud(self, img, box, handle):
        # ... (Same as previous code) ...
        H, W = img.shape[:2]
        CX, CY = W//2, H//2
        TargetX = int(CX + GRIPPER_OFFSET_X)
        TargetY = int(CY + GRIPPER_OFFSET_Y)
        if SCENARIO_MODE == "VERTICAL": TargetY += self.VERTICAL_AIM_OFFSET

        cv2.circle(img, (CX, CY), 5, (0,255,0), 1)
        cv2.circle(img, (TargetX, TargetY), 10, (255,0,255), 2)
        
        # Display State clearly
        cv2.putText(img, f"STATE: {self.state}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
        
        # Display Popup Prompt on Video Feed if Waiting
        if self.state == "WAIT_FOR_USER":
            cv2.putText(img, "DID WE ATTACH?", (W//2-150, H//2), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,0,255), 3)
            cv2.putText(img, "CHECK DASHBOARD", (W//2-160, H//2+50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)

        if box:
            x,y,w,h = map(int, box['bbox'])
            cv2.rectangle(img, (x-w//2, y-h//2), (x+w//2, y+h//2), (255,0,0), 2)
        if handle:
            x,y,w,h = map(int, handle['bbox'])
            cv2.rectangle(img, (x-w//2, y-h//2), (x+w//2, y+h//2), (0,255,0), 2)
            cv2.line(img, (x, y), (TargetX, TargetY), (0,255,255), 1)

def main(args=None):
    rclpy.init(args=args)
    node = FinalServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
