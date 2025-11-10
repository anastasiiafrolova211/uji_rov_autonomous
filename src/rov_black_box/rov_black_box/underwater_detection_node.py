#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO


class UnderwaterDetectionNode(Node):
    def __init__(self):
        super().__init__('underwater_detection')
        
        self.get_logger().info('Starting Underwater Detection Node...')
        
        # Parameters
        self.declare_parameter('camera_topic', 'camera/image_raw')
        self.declare_parameter('yolo_model_path', '/home/elex/uji_rov_autonomous/best_seg.pt')
        self.declare_parameter('enable_aruco', True)
        self.declare_parameter('enable_yolo', True)
        self.declare_parameter('show_visualization', True)
        self.declare_parameter('confidence_threshold', 0.5)
        
        # Get parameters
        camera_topic = self.get_parameter('camera_topic').value
        model_path = self.get_parameter('yolo_model_path').value
        self.enable_aruco = self.get_parameter('enable_aruco').value
        self.enable_yolo = self.get_parameter('enable_yolo').value
        self.show_viz = self.get_parameter('show_visualization').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize YOLO model
        self.yolo_model = None
        if self.enable_yolo:
            try:
                self.yolo_model = YOLO(model_path)
                self.get_logger().info(f'YOLO model loaded: {model_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to load YOLO model: {e}')
                self.enable_yolo = False
        
        # Initialize ArUco detector
        self.aruco_dict = None
        self.aruco_params = None
        if self.enable_aruco:
            try:
                self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
                self.aruco_params = cv2.aruco.DetectorParameters()
                self.get_logger().info('ArUco detector initialized')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize ArUco: {e}')
                self.enable_aruco = False
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        
        # Publishers
        self.image_pub = self.create_publisher(Image, 'detection/image', 10)
        self.box_pose_pub = self.create_publisher(PoseStamped, 'box_pose', 10)
        self.handle_pose_pub = self.create_publisher(PoseStamped, 'handle_pose', 10)
        
        # Visualization
        if self.show_viz:
            cv2.namedWindow('Underwater Detection', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Underwater Detection', 800, 600)
            
            # Show test frame
            test_img = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(test_img, 'Waiting for camera...', (150, 240),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.imshow('Underwater Detection', test_img)
            cv2.waitKey(1)
        
        # FPS tracking
        self.last_viz_time = self.get_clock().now()
        self.frame_count = 0
        
        self.get_logger().info(f'Subscribed to: {camera_topic}')
        self.get_logger().info('Underwater Detection Node ready!')
    
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            self.frame_count += 1
            
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Create visualization image
            viz_image = cv_image.copy()
            
            # ArUco marker detection
            if self.enable_aruco:
                self.detect_aruco(cv_image, viz_image)
            
            # YOLO detection
            if self.enable_yolo and self.yolo_model is not None:
                self.detect_yolo(cv_image, viz_image)
            
            # Add FPS counter
            current_time = self.get_clock().now()
            dt = (current_time - self.last_viz_time).nanoseconds / 1e9
            fps = 1.0 / dt if dt > 0 else 0
            
            # Add info overlay
            cv2.putText(viz_image, f'FPS: {fps:.1f}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(viz_image, f'Frame: {self.frame_count}', (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            self.last_viz_time = current_time
            
            # Show visualization
            if self.show_viz:
                cv2.imshow('Underwater Detection', viz_image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.get_logger().info('Quit requested')
                    rclpy.shutdown()
            
            # Publish annotated image
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(viz_image, encoding='bgr8')
                annotated_msg.header = msg.header
                self.image_pub.publish(annotated_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish image: {e}')
                
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')
    
    def detect_aruco(self, cv_image, viz_image):
        """Detect and visualize ArUco markers"""
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, 
            self.aruco_dict, 
            parameters=self.aruco_params
        )
        
        if ids is not None:
            # Draw markers
            cv2.aruco.drawDetectedMarkers(viz_image, corners, ids)
            
            # Process each marker
            for i, marker_id in enumerate(ids.flatten()):
                corner = corners[i][0]
                center = corner.mean(axis=0).astype(int)
                
                # Draw center point
                cv2.circle(viz_image, tuple(center), 5, (0, 0, 255), -1)
                
                # Add label
                cv2.putText(viz_image, f'ArUco {marker_id}', 
                           (center[0] - 40, center[1] - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                self.get_logger().debug(f'ArUco marker {marker_id} at {center}')
    
    def detect_yolo(self, cv_image, viz_image):
        """Detect and visualize objects using YOLO"""
        results = self.yolo_model(cv_image, verbose=False, conf=self.conf_threshold)
        
        for result in results:
            # Process bounding boxes
            if result.boxes is not None:
                boxes = result.boxes
                
                for box in boxes:
                    # Get box info
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    class_name = self.yolo_model.names[cls]
                    
                    # Color based on class
                    if 'box' in class_name.lower():
                        color = (255, 0, 0)  # Blue for box
                    elif 'handle' in class_name.lower():
                        color = (0, 255, 255)  # Yellow for handle
                    else:
                        color = (0, 255, 0)  # Green for others
                    
                    # Draw bounding box
                    cv2.rectangle(viz_image, (x1, y1), (x2, y2), color, 2)
                    
                    # Draw label background
                    label = f'{class_name} {conf:.2f}'
                    label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                    cv2.rectangle(viz_image, (x1, y1 - label_size[1] - 10), 
                                 (x1 + label_size[0], y1), color, -1)
                    
                    # Draw label text
                    cv2.putText(viz_image, label, (x1, y1 - 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    
                    # Calculate and draw center
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2
                    cv2.circle(viz_image, (cx, cy), 5, color, -1)
                    
                    # Publish pose
                    self.publish_pose(cx, cy, class_name)
                    
                    self.get_logger().debug(f'{class_name} at ({cx}, {cy}) conf={conf:.2f}')
            
            # Process segmentation masks
            if result.masks is not None:
                masks = result.masks.data.cpu().numpy()
                classes = result.boxes.cls.cpu().numpy().astype(int)
                
                for mask, cls in zip(masks, classes):
                    class_name = self.yolo_model.names[cls]
                    
                    # Resize mask to image size
                    mask_resized = cv2.resize(mask, (cv_image.shape[1], cv_image.shape[0]))
                    mask_binary = (mask_resized > 0.5).astype(np.uint8)
                    
                    # Create colored overlay
                    colored_mask = np.zeros_like(cv_image)
                    if 'box' in class_name.lower():
                        colored_mask[:, :, 0] = mask_binary * 255  # Blue
                    elif 'handle' in class_name.lower():
                        colored_mask[:, :, 1] = mask_binary * 255  # Green
                        colored_mask[:, :, 2] = mask_binary * 255  # Red (Green + Red = Yellow)
                    else:
                        colored_mask[:, :, 1] = mask_binary * 255  # Green
                    
                    # Blend with visualization image
                    viz_image[:] = cv2.addWeighted(viz_image, 1.0, colored_mask, 0.3, 0)
    
    def publish_pose(self, x, y, class_name):
        """Publish detected object pose"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'camera_frame'
        
        # Image coordinates
        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        pose_msg.pose.position.z = 0.0
        
        # Orientation (identity quaternion)
        pose_msg.pose.orientation.w = 1.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        
        # Publish to appropriate topic
        if 'box' in class_name.lower():
            self.box_pose_pub.publish(pose_msg)
        elif 'handle' in class_name.lower():
            self.handle_pose_pub.publish(pose_msg)
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.show_viz:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UnderwaterDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

