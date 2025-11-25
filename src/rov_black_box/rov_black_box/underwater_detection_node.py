import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import math
import os

class HandleDetector(Node):
    def __init__(self):
        super().__init__('handle_detector')

        # --- ⚠️ CONFIGURATION: UPDATE THESE TWO PATHS ⚠️ ---
        # 1. Path to your 'best.pt' file (uses the folder generated during training)
        self.model_path = '/home/elex/mainproj/v8_seg/runs/segment/bluerov_handle_12802/weights/best.pt' 
        # 2. The ROS topic publishing the image feed (e.g., from your camera or the video publisher)
        self.camera_topic = '/camera/image_raw' 
        # ----------------------------------------------------

        self.conf_threshold = 0.5               # Minimum confidence to accept a detection
        
        # ROS Setup
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10)
        self.bridge = CvBridge()
        
        # Publishers: Angle for Control Loop, Debug image for visualization
        self.angle_pub = self.create_publisher(Float32, '/handle/angle', 10)
        self.debug_pub = self.create_publisher(Image, '/handle/debug', 10)

        # Load YOLO Model
        if not os.path.exists(self.model_path):
             self.get_logger().error(f"FATAL: Model not found at: {self.model_path}")
             rclpy.shutdown()
             return

        self.get_logger().info("Loading YOLOv8 Model...")
        self.model = YOLO(self.model_path)
        self.get_logger().info("✅ Model Loaded! Waiting for image feed...")

    def image_callback(self, msg):
        try:
            # Convert ROS Image -> OpenCV Image (NumPy array)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Img conversion failed: {e}")
            return

        # 1. Run Inference on the GPU
        results = self.model(cv_image, verbose=False, conf=self.conf_threshold)
        result = results[0]

        annotated_frame = result.plot() 
        
        target_angle = 0.0
        target_conf = 0.0
        found_handle = False

        # 2. Find the Target Handle and Process
        if result.masks is not None:
            # Iterate through detections to find the 'handle' (assuming class 1)
            for i, cls_id in enumerate(result.boxes.cls):
                if int(cls_id) == 1: # Class 1 (Handle)
                    
                    conf = float(result.boxes.conf[i])
                    
                    # Extract and resize mask data
                    mask = result.masks.data[i].cpu().numpy() 
                    mask = cv2.resize(mask, (cv_image.shape[1], cv_image.shape[0]))
                    mask = (mask * 255).astype(np.uint8)

                    # 3. Calculate Angle using PCA
                    angle_deg, center, axis_end = self.get_orientation(mask)
                    
                    target_angle = angle_deg
                    target_conf = conf
                    found_handle = True

                    # --- VISUALIZATION: Draw the PCA Axis ---
                    cv2.circle(annotated_frame, center, 8, (0, 0, 255), -1) # Center point
                    cv2.line(annotated_frame, center, axis_end, (0, 255, 0), 4, cv2.LINE_AA) # Green Axis Line
                    
                    # Publish the angle to the control loop
                    msg = Float32()
                    msg.data = angle_deg
                    self.angle_pub.publish(msg)
                    
                    break # Lock onto the first detected handle

        # 4. Draw HUD and Display Window
        self.draw_hud(annotated_frame, found_handle, target_conf, target_angle)
        cv2.imshow("BlueROV Vision Feed", annotated_frame)
        cv2.waitKey(1) 

        # Publish debug topic
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8"))

    def get_orientation(self, mask):
        """Calculates the angle of the object in the mask using PCA."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return 0.0, (0,0), (0,0)

        c = max(contours, key=cv2.contourArea)
        pts = c.reshape(-1, 2).astype(np.float64)
        
        if len(pts) < 2: return 0.0, (0,0), (0,0)

        # PCA Computation
        mean, eigenvectors, _ = cv2.PCACompute2(pts, mean=None)
        cntr = (int(mean[0][0]), int(mean[0][1]))
        
        vx, vy = eigenvectors[0][0], eigenvectors[0][1]
        angle_rad = math.atan2(vy, vx)
        angle_deg = math.degrees(angle_rad)
        
        # Calculate end point for the line drawing
        length = 150
        p2 = (int(cntr[0] + vx * length), int(cntr[1] + vy * length))
        
        return angle_deg, cntr, p2

    def draw_hud(self, img, found, conf, angle):
        """Draws the Heads Up Display text box."""
        overlay = img.copy()
        cv2.rectangle(overlay, (10, 10), (350, 130), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, img, 0.4, 0, img)

        font = cv2.FONT_HERSHEY_SIMPLEX
        white = (255, 255, 255)
        
        if found:
            color = (0, 255, 0) # Green
            status = "LOCKED"
            conf_text = f"Conf: {conf*100:.1f}%"
            angle_text = f"Roll: {angle:.1f} deg"
        else:
            color = (0, 0, 255) # Red
            status = "SEARCHING"
            conf_text = "Conf: --"
            angle_text = "Roll: --"

        cv2.putText(img, f"STATUS: {status}", (25, 50), font, 0.8, color, 2)
        cv2.putText(img, conf_text, (25, 85), font, 0.7, white, 2)
        cv2.putText(img, angle_text, (25, 115), font, 0.7, white, 2)

def main(args=None):
    rclpy.init(args=args)
    node = HandleDetector()
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
