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
        # Set default to your namespaced topic if desired, or override via launch
        self.declare_parameter('camera_topic', '/bluerov2/camera/image_raw')
        self.declare_parameter('yolo_model_path', '/home/elex/uji_rov_autonomous/v8m_1280.pt')
        self.declare_parameter('enable_aruco', True)
        self.declare_parameter('enable_yolo', True)
        self.declare_parameter('show_visualization', True)

        # Class-specific confidences (tune for your case)
        self.declare_parameter('box_conf_threshold', 0.5)
        self.declare_parameter('handle_conf_threshold', 0.3)

        # Get parameters
        camera_topic = self.get_parameter('camera_topic').value
        model_path = self.get_parameter('yolo_model_path').value
        self.enable_aruco = bool(self.get_parameter('enable_aruco').value)
        self.enable_yolo = bool(self.get_parameter('enable_yolo').value)
        self.show_viz = bool(self.get_parameter('show_visualization').value)
        self.box_conf = float(self.get_parameter('box_conf_threshold').value)
        self.handle_conf = float(self.get_parameter('handle_conf_threshold').value)

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
                if hasattr(cv2.aruco, "DetectorParameters"):
                    self.aruco_params = cv2.aruco.DetectorParameters()
                else:
                    self.aruco_params = cv2.aruco.DetectorParameters_create()
                self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
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
            try:
                cv2.namedWindow('Underwater Detection', cv2.WINDOW_NORMAL)
                cv2.resizeWindow('Underwater Detection', 800, 600)
                test_img = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(test_img, 'Waiting for camera...', (150, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.imshow('Underwater Detection', test_img)
                cv2.waitKey(1)
            except cv2.error as e:
                self.get_logger().warn(f"OpenCV GUI not available: {e}")
                self.show_viz = False

        # FPS tracking
        self.last_viz_time = self.get_clock().now()
        self.frame_count = 0

        # Temporal smoothing + last centers
        self.last_box_pixel = None
        self.last_handle_pixel = None
        self.smoothing_alpha = 0.3

        self.get_logger().info(f'Subscribed to image topic: {camera_topic}')
        self.get_logger().info('Underwater Detection Node ready!')

    # ------------- Callbacks -------------

    def image_callback(self, msg: Image):
        """Process incoming camera images."""
        try:
            self.frame_count += 1

            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self.frame_count == 1:
                h, w = cv_image.shape[:2]
                self.get_logger().info(f"First image received: {w}x{h}")

            viz_image = cv_image.copy()

            if self.enable_aruco and self.aruco_dict is not None:
                try:
                    self.detect_aruco(cv_image, viz_image)
                except Exception as e:
                    self.get_logger().error(f'ArUco detection error: {e}')
                    self.enable_aruco = False

            if self.enable_yolo and self.yolo_model is not None:
                try:
                    self.detect_yolo(cv_image, viz_image, msg.header)
                except Exception as e:
                    self.get_logger().error(f'YOLO detection error: {e}')
                    self.enable_yolo = False

            current_time = self.get_clock().now()
            dt = (current_time - self.last_viz_time).nanoseconds / 1e9
            fps = 1.0 / dt if dt > 0 else 0.0
            self.last_viz_time = current_time

            cv2.putText(viz_image, f'FPS: {fps:.1f}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(viz_image, f'Frame: {self.frame_count}', (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            if self.show_viz:
                try:
                    cv2.imshow('Underwater Detection', viz_image)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        self.get_logger().info('Quit requested')
                        rclpy.shutdown()
                except cv2.error as e:
                    self.get_logger().warn(f"OpenCV imshow error: {e}")
                    self.show_viz = False

            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(viz_image, encoding='bgr8')
                annotated_msg.header = msg.header
                self.image_pub.publish(annotated_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish image: {e}')

        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    # ------------- Detection helpers -------------

    def detect_aruco(self, cv_image, viz_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(viz_image, corners, ids)
            for i, marker_id in enumerate(ids.flatten()):
                corner = corners[i][0]
                center = corner.mean(axis=0).astype(int)
                cv2.circle(viz_image, tuple(center), 5, (0, 0, 255), -1)
                cv2.putText(viz_image, f'ArUco {marker_id}',
                            (center[0] - 40, center[1] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    def detect_yolo(self, cv_image, viz_image, header):
        """Detect and visualize objects; publish one stabilized box + handle per frame."""
        results = self.yolo_model(cv_image, verbose=False)

        best_box = None
        best_box_conf = 0.0

        handles = []  # collect all handle candidates

        for result in results:
            if result.boxes is None:
                continue
            boxes = result.boxes

            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                class_name = self.yolo_model.names[cls]

                # Draw all detections
                if 'box' in class_name.lower():
                    color = (255, 0, 0)
                elif 'handle' in class_name.lower():
                    color = (0, 255, 255)
                else:
                    color = (0, 255, 0)

                cv2.rectangle(viz_image, (x1, y1), (x2, y2), color, 2)
                label = f'{class_name} {conf:.2f}'
                label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                cv2.rectangle(
                    viz_image,
                    (x1, y1 - label_size[1] - 10),
                    (x1 + label_size[0], y1),
                    color,
                    -1
                )
                cv2.putText(viz_image, label, (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                cv2.circle(viz_image, (cx, cy), 4, color, -1)

                # Select best box by confidence (simple)
                if 'box' in class_name.lower():
                    if conf < self.box_conf:
                        continue
                    if conf > best_box_conf:
                        best_box_conf = conf
                        best_box = (x1, y1, x2, y2, cx, cy)

                # Collect all handle candidates for tracking-based selection
                elif 'handle' in class_name.lower():
                    if conf < self.handle_conf:
                        continue
                    area = max(1, (x2 - x1) * (y2 - y1))
                    handles.append((x1, y1, x2, y2, cx, cy, conf, area))

        # Publish single BOX pose
        if best_box is not None:
            x1, y1, x2, y2, cx, cy = best_box
            cx_s, cy_s = self.smooth_pixel(self.last_box_pixel, (cx, cy))
            self.last_box_pixel = (cx_s, cy_s)
            self.publish_pose(cx_s, cy_s, x2 - x1, y2 - y1, 'box', header)

        # Publish single HANDLE pose using "closest to last center" tracking
        if handles:
            if self.last_handle_pixel is not None:
                lx, ly = self.last_handle_pixel
                # pick the handle whose center is closest to last center
                best = min(
                    handles,
                    key=lambda h: (h[4] - lx) ** 2 + (h[5] - ly) ** 2
                )
            else:
                # first acquisition: pick largest-area handle
                best = max(handles, key=lambda h: h[7])

            x1, y1, x2, y2, cx, cy, conf, area = best
            cx_s, cy_s = self.smooth_pixel(self.last_handle_pixel, (cx, cy))
            self.last_handle_pixel = (cx_s, cy_s)
            self.publish_pose(cx_s, cy_s, x2 - x1, y2 - y1, 'handle', header)

    # ------------- Utility + publishing -------------

    def smooth_pixel(self, last, current):
        if last is None:
            return current
        lx, ly = last
        cx, cy = current
        a = self.smoothing_alpha
        sx = a * cx + (1.0 - a) * lx
        sy = a * cy + (1.0 - a) * ly
        return sx, sy

    def publish_pose(self, x, y, w, h, class_key, header):
        pose_msg = PoseStamped()
        pose_msg.header = header
        if not pose_msg.header.frame_id:
            pose_msg.header.frame_id = 'camera_frame'

        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        pose_msg.pose.position.z = float(h)

        pose_msg.pose.orientation.w = 1.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0

        if class_key == 'box':
            self.box_pose_pub.publish(pose_msg)
        elif class_key == 'handle':
            self.handle_pose_pub.publish(pose_msg)

    def destroy_node(self):
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

