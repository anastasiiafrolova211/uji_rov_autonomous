#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import torch
import time


class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # Parameters
        self.declare_parameter('model_path', 'best.pt')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('camera_frame', 'camera_link')

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        conf_thres = self.get_parameter('conf_threshold').get_parameter_value().double_value

        # Load YOLO model
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model = YOLO(model_path)
        self.model.to(self.device)
        self.model.conf = conf_thres

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.image_pub = self.create_publisher(Image, '/detector/image_out', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/detector/handle_pose', 10)

        self.get_logger().info(f'YOLO detector node started. Using {self.device}. Model: {model_path}')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLO segmentation
        results = self.model.predict(source=frame, verbose=False)

        annotated = frame.copy()
        handle_pose = PoseStamped()
        handle_found = False

        for r in results:
            if not r.masks:
                continue
            for mask, box, cls_id, conf in zip(r.masks.xy, r.boxes.xyxy, r.boxes.cls, r.boxes.conf):
                class_name = self.model.names[int(cls_id)]
                x1, y1, x2, y2 = map(int, box)
                color = (0, 255, 0) if class_name == 'handle' else (0, 128, 255)

                # Draw box
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                label = f"{class_name} {conf:.2f}"
                cv2.putText(annotated, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

                # Compute centroid for handle
                if class_name == 'handle':
                    mask_np = np.zeros(frame.shape[:2], dtype=np.uint8)
                    pts = np.array(mask, np.int32)
                    cv2.fillPoly(mask_np, [pts], 255)
                    M = cv2.moments(mask_np)
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        cv2.circle(annotated, (cx, cy), 5, (255, 0, 0), -1)

                        handle_pose.header.stamp = self.get_clock().now().to_msg()
                        handle_pose.header.frame_id = self.get_parameter('camera_frame').get_parameter_value().string_value
                        handle_pose.pose.position.x = float(cx)
                        handle_pose.pose.position.y = float(cy)
                        handle_pose.pose.position.z = 0.0
                        handle_found = True

        if handle_found:
            self.pose_pub.publish(handle_pose)

        # Publish annotated image
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        self.image_pub.publish(annotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

