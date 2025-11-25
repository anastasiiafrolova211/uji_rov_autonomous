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
import os


def get_orientation(mask_points):
    """
    Calculates the orientation and major axis of the object using PCA 
    (Principal Component Analysis) on the mask points.
    Returns: angle in degrees, major axis length.
    """
    # Needs at least 5 points for PCA to be stable
    if len(mask_points) < 5:
        return 0.0, 0
    
    # Convert points to float32 for PCA and ensure shape is (N, 2)
    points = np.array(mask_points, dtype=np.float32).reshape(-1, 2)
    
    # Calculate the principal axes using PCA
    # mean: The centroid of the points (automatically calculated if None)
    # eigenvectors: The principal components (axes of maximum variance)
    # We only need the first two components for 2D orientation
    mean, eigenvectors = cv2.PCACompute(points, mean=None, maxComponents=2)
    
    # Calculate the angle of the main axis (first eigenvector) from the horizontal (X-axis)
    # np.arctan2(y, x) returns angle in radians
    angle = np.arctan2(eigenvectors[0, 1], eigenvectors[0, 0])
    angle_deg = np.degrees(angle)
    
    # Ensure the angle is constrained to the meaningful range for yaw control (-90 to 90)
    if angle_deg < -90:
        angle_deg += 180
    elif angle_deg > 90:
        angle_deg -= 180
    
    # Estimate the length of the major axis by projecting points onto the main axis
    projected_points = np.dot(points - mean, eigenvectors[0])
    major_axis_length = np.ptp(projected_points) # Peak-to-peak (max - min)
    
    return angle_deg, major_axis_length


def draw_hud(img, center, bbox, state, angle_deg, color=(0, 255, 255)):
    """Draws visual feedback (center, error, state) on the image frame."""
    H, W, _ = img.shape
    C_img = (W // 2, H // 2)
    
    # Draw image center (Target)
    cv2.circle(img, C_img, 5, (255, 0, 0), -1)
    
    # Draw target center and error line
    if center is not None:
        center_int = (int(center[0]), int(center[1]))
        cv2.circle(img, center_int, 5, color, -1)
        # Draw line showing pixel error vector
        cv2.line(img, C_img, center_int, color, 2)
        
        E_X = center[0] - C_img[0]
        E_Y = center[1] - C_img[1]
        
        cv2.putText(img, f"Ex: {E_X:.0f}, Ey: {E_Y:.0f}", (10, H - 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    # Draw state and orientation
    cv2.putText(img, f"STATE: {state}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(img, f"YAW ERR: {angle_deg:.1f} deg", (10, 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)

    # Draw the bounding box
    if bbox is not None:
        x, y, w, h = map(int, bbox)
        x1, y1 = int(x - w / 2), int(y - h / 2)
        x2, y2 = int(x + w / 2), int(y + h / 2)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

# -----------------------------------------------------------------------------
# --- VISUAL SERVO NODE CLASS ---
# -----------------------------------------------------------------------------

class VisualServo(Node):
    def __init__(self):
        super().__init__('visual_servo_node')

        # --- ⚠️ CONFIGURATION: UPDATE MODEL PATH ⚠️ ---
        # *** REPLACE THIS WITH YOUR ABSOLUTE PATH TO best.pt ***
        self.MODEL_PATH = '/home/elex/mainproj/v8_seg/runs/segment/bluerov_handle_12802/weights/best.pt'
        self.CMD_TOPIC = '/cmd_vel'
        
        # --- Servoing Gains (PROPORTIONAL CONTROLLERS - TUNE THESE!) ---
        self.KX, self.KY = 0.003, 0.003      # Lateral (Y) and Vertical (Z) centering gains
        self.KZ = 0.000008                   # Forward (X) distance gain (Area Error)
        self.KW = 0.005                      # Yaw (W) rotation gain (Angle Error)
        
        # --- Servoing Targets and Tolerances ---
        self.TARGET_AREA = 160000            # Optimal distance area (e.g., 400x400 pixels)
        self.CENTER_TOLERANCE = 15           # Max allowed pixel error for centering handle
        self.AREA_TOLERANCE = 10000          # Max allowed area error for approach phase
        self.YAW_TOLERANCE = 5.0             # Max allowed angle error for yaw correction
        self.CONF_THRESHOLD = 0.5            # Min confidence for detection

        # --- ROS Publishers/Subscribers ---
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.mode_sub = self.create_subscription(Bool, '/rov/servo_mode_active', self.mode_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, self.CMD_TOPIC, 10)
        
        # State Variables
        self.servo_active = False           
        self.target_state = "SEARCH"        
        
        # Load YOLO Model
        if not os.path.exists(self.MODEL_PATH):
             self.get_logger().error(f"FATAL: Model not found at: {self.MODEL_PATH}")
             rclpy.shutdown()
             return

        self.bridge = CvBridge()
        self.model = YOLO(self.MODEL_PATH)
        self.get_logger().info("Visual Servo Node Initialized. Waiting for AUTOMATIC mode...")


    def mode_callback(self, msg: Bool):
        """Receives the signal from the joystick node to enable/disable servoing."""
        self.servo_active = msg.data
        if self.servo_active:
             self.get_logger().info("Visual Servoing Enabled. Starting state: SEARCH.")
             self.target_state = "SEARCH" 
        else:
             self.cmd_pub.publish(Twist()) # Stop motion immediately
             self.get_logger().info("Visual Servoing Disabled. Joystick taking control.")


    def image_callback(self, msg: Image):
        if not self.servo_active:
            return 

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return

        H, W, _ = cv_image.shape
        
        # 1. YOLO Inference
        results = self.model(cv_image, verbose=False, conf=self.CONF_THRESHOLD)
        result = results[0]
        
        box_data, handle_data = self._parse_results(result)
        
        # 2. State Machine and Control Calculation
        cmd_vel, debug_center, debug_bbox, angle_deg = self.run_servoing_state_machine(
            cv_image, box_data, handle_data
        )
        
        # 3. Publish Command
        self.cmd_pub.publish(cmd_vel)

        # 4. Debug Visualization
        annotated_frame = result.plot()
        draw_hud(annotated_frame, debug_center, debug_bbox, 
                 self.target_state if self.servo_active else "MANUAL", 
                 angle_deg)
        cv2.imshow("Visual Servo Debug View", annotated_frame)
        cv2.waitKey(1) 


    def _parse_results(self, result):
        """Extracts bounding box and segmentation data for the box (cls 0) and handle (cls 1)."""
        box_data, handle_data = None, None
        
        for i, cls_id in enumerate(result.boxes.cls):
            bbox = result.boxes.xywh[i].cpu().numpy().flatten()
            mask_points = None
            if result.masks and result.masks.xy:
                mask_points = result.masks.xy[i].cpu().numpy().astype(np.int32)
            
            data = {'bbox': bbox, 'mask': mask_points}

            if int(cls_id) == 0:
                box_data = data
            elif int(cls_id) == 1:
                handle_data = data
        
        return box_data, handle_data


    def run_servoing_state_machine(self, frame, box_data, handle_data):
        """Determines the current ROV state and calculates the Twist command."""
        cmd_vel = Twist()
        H, W, _ = frame.shape
        C_img = (W / 2, H / 2)
        
        debug_center, debug_bbox, angle_deg = C_img, None, 0.0

        if self.target_state == "SEARCH":
            cmd_vel.angular.z = 0.05 # Slow yaw to search
            if box_data is not None:
                self.get_logger().info("✅ Box found. Starting approach.")
                self.target_state = "APPROACH_BOX"
            return cmd_vel, C_img, None, angle_deg


        elif self.target_state == "APPROACH_BOX":
            if box_data is None:
                self.get_logger().warn("Box lost during approach. Reverting to SEARCH.")
                self.target_state = "SEARCH"
                return cmd_vel, C_img, None, angle_deg

            box_bbox = box_data['bbox']
            debug_bbox = box_bbox
            
            # Center of the object in pixels
            C_box = (box_bbox[0], box_bbox[1])
            debug_center = C_box

            # 1. Distance Control (Linear X)
            W_box, H_box = box_bbox[2], box_bbox[3]
            current_area = W_box * H_box
            E_Z = self.TARGET_AREA - current_area
            cmd_vel.linear.x = self.KZ * E_Z
            
            # 2. Transition Check
            if abs(E_Z) < self.AREA_TOLERANCE:
                self.get_logger().info("✅ Distance reached. Centering handle.")
                self.target_state = "CENTER_HANDLE"
                
            # 3. Coarse Centering (Linear Y and Z)
            E_X = C_box[0] - C_img[0]
            E_Y = C_box[1] - C_img[1]
            cmd_vel.linear.y = -self.KY * E_X
            cmd_vel.linear.z = -self.KX * E_Y

        
        elif self.target_state == "CENTER_HANDLE":
            if handle_data is None:
                self.get_logger().warn("Handle lost. Reverting to approach box (Phase 2).")
                self.target_state = "APPROACH_BOX"
                return cmd_vel, C_img, None, angle_deg
            
            handle_bbox = handle_data['bbox']
            handle_mask = handle_data['mask']
            debug_bbox = handle_bbox

            C_handle = (handle_bbox[0], handle_bbox[1])
            debug_center = C_handle

            # 1. Centering (Linear Y and Z)
            E_X = C_handle[0] - C_img[0]
            E_Y = C_handle[1] - C_img[1]
            cmd_vel.linear.y = -self.KY * E_X 
            cmd_vel.linear.z = -self.KX * E_Y 
            
            # 2. Orientation Control (Angular Z - Yaw) using PCA
            if handle_mask is not None:
                angle_deg, major_axis_length = get_orientation(handle_mask)
                E_W = -angle_deg # Yaw error
                cmd_vel.angular.z = self.KW * E_W
            
            # 3. Transition Check (Alignment Completion)
            if (abs(E_X) < self.CENTER_TOLERANCE and 
                abs(E_Y) < self.CENTER_TOLERANCE and
                abs(E_W) < self.YAW_TOLERANCE):
                
                self.get_logger().info("🎯 Handle perfectly aligned and locked. Holding.")
                self.target_state = "HOLD"
        
        
        elif self.target_state == "HOLD":
            # Publish zero Twist to stop all motion
            pass 
        
        return cmd_vel, debug_center, debug_bbox, angle_deg


def main(args=None):
    rclpy.init(args=args)
    node = VisualServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
