#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import gi
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from std_msgs.msg import String, Int32MultiArray
from cv_bridge import CvBridge
from cv2 import aruco
from ultralytics import YOLO
import os

gi.require_version('Gst', '1.0')
from gi.repository import Gst

class UnderwaterDetectionNode(Node):

    def __init__(self):
        super().__init__("underwater_detection_node")
        
        # Parameters
        self.declare_parameter("port", 5600)
        self.declare_parameter("enable_detection", True)
        
        # ArUco parameters
        self.declare_parameter("aruco_dict", "DICT_4X4_50")
        self.declare_parameter("floor_marker_ids", [10, 20])
        self.declare_parameter("box_marker_id", 42)
        self.declare_parameter("marker_size", 0.10)
        
        # YOLO parameters
        self.declare_parameter("yolo_model_path", "best_seg.pt")
        self.declare_parameter("yolo_confidence", 0.5)
        self.declare_parameter("enable_yolo_detection", True)
        
        self.port = self.get_parameter("port").value
        self.enable_detection = self.get_parameter("enable_detection").value
        self.aruco_dict_name = self.get_parameter("aruco_dict").value
        self.floor_marker_ids = self.get_parameter("floor_marker_ids").value
        self.box_marker_id = self.get_parameter("box_marker_id").value
        self.marker_size = self.get_parameter("marker_size").value
        
        # YOLO setup
        self.yolo_model_path = self.get_parameter("yolo_model_path").value
        self.yolo_confidence = self.get_parameter("yolo_confidence").value
        self.enable_yolo_detection = self.get_parameter("enable_yolo_detection").value
        
        # Load YOLO model
        self.yolo_model = None
        if self.enable_yolo_detection:
            if os.path.exists(self.yolo_model_path):
                try:
                    self.yolo_model = YOLO(self.yolo_model_path)
                    self.get_logger().info(f"✓ YOLO model loaded from: {self.yolo_model_path}")
                except Exception as e:
                    self.get_logger().error(f"✗ Failed to load YOLO model: {e}")
                    self.enable_yolo_detection = False
            else:
                self.get_logger().error(f"✗ YOLO model not found at: {self.yolo_model_path}")
                self.enable_yolo_detection = False
        
        # Camera calibration
        self.camera_matrix = np.array([
            [576.9, 0.0, 409.7],
            [0.0, 576.7, 268.7],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)

        self.dist_coeffs = np.array([
            0.004002, 0.009106, -0.002852, 0.000090, 0.015693,
        ], dtype=np.float32).reshape(5, 1)
        
        # ArUco setup
        try:
            self.aruco_dict = aruco.getPredefinedDictionary(getattr(aruco, self.aruco_dict_name))
            self.get_logger().info(f"✓ ArUco dictionary: {self.aruco_dict_name}")
        except AttributeError:
            self.get_logger().error(f"✗ Dictionary {self.aruco_dict_name} not found!")
            self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # GStreamer setup
        self._frame = None
        self.video_source = 'udpsrc port={}'.format(self.port)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        self.video_decode = '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        self.video_sink_conf = '! appsink emit-signals=true sync=false max-buffers=2 drop=true'
        
        self.video_pipe = None
        self.video_sink = None
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        
        Gst.init()
        self.bridge = CvBridge()
        
        # Publishers
        self.image_publisher = self.create_publisher(Image, 'camera/image_annotated', 10)
        self.floor_markers_publisher = self.create_publisher(PoseArray, 'markers/floor_poses', 10)
        self.box_marker_publisher = self.create_publisher(PoseStamped, 'box/marker_pose', 10)
        self.box_pose_publisher = self.create_publisher(PoseStamped, 'box/pose', 10)
        self.handle_pose_publisher = self.create_publisher(PoseStamped, 'box/handle_pose', 10)
        self.detection_status_publisher = self.create_publisher(String, 'detection/status', 10)
        self.detected_marker_ids_publisher = self.create_publisher(Int32MultiArray, 'markers/detected_ids', 10)
        
        # Start video
        self.run()
        self.create_timer(0.033, self.update)
        
        self.get_logger().info('='*60)
        self.get_logger().info('Underwater Detection Node Started')
        self.get_logger().info(f'ArUco: {self.aruco_dict_name} | Size: {self.marker_size}m')
        self.get_logger().info(f'Floor markers: {self.floor_marker_ids}')
        self.get_logger().info(f'Box marker: {self.box_marker_id}')
        self.get_logger().info(f'YOLO: {"ENABLED" if self.enable_yolo_detection else "DISABLED"}')
        self.get_logger().info('='*60)

    def start_gst(self, config=None):
        if not config:
            config = [
                'videotestsrc ! decodebin',
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]
        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        buf = sample.get_buffer()
        caps = sample.get_caps()
        array = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array

    def frame(self):
        return self._frame

    def frame_available(self):
        return type(self._frame) != type(None)

    def run(self):
        self.start_gst([
            self.video_source,
            self.video_codec,
            self.video_decode,
            self.video_sink_conf
        ])
        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        try:
            sample = sink.emit('pull-sample')
            new_frame = self.gst_to_opencv(sample)
            self._frame = new_frame
        except Exception as e:
            self.get_logger().error(f"Error in callback: {e}")
        return Gst.FlowReturn.OK
    
    def detect_with_yolo(self, img):
        """
        Use YOLO to detect box and handle
        Returns: (box_bbox, handle_bbox, results)
        """
        try:
            results = self.yolo_model(img, conf=self.yolo_confidence, verbose=False)
            
            box_bbox = None
            handle_bbox = None
            
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    
                    # Assuming class 0 = box, class 1 = handle
                    if cls == 0:  # Box
                        box_bbox = (int(x1), int(y1), int(x2), int(y2), conf)
                    elif cls == 1:  # Handle
                        handle_bbox = (int(x1), int(y1), int(x2), int(y2), conf)
            
            return box_bbox, handle_bbox, results[0] if results else None
        
        except Exception as e:
            self.get_logger().error(f"YOLO detection error: {e}")
            return None, None, None
    
    def update(self):
        try:
            if not self.frame_available():
                return

            frame = self.frame()
            width = int(1920/2)
            height = int(1080/2)
            img = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
            
            detection_status = []
            detected_ids = []
            
            if self.enable_detection:
                # STEP 1: Detect ArUco markers
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = self.aruco_detector.detectMarkers(gray)
                
                floor_poses = PoseArray()
                floor_poses.header.stamp = self.get_clock().now().to_msg()
                floor_poses.header.frame_id = "camera"
                
                if ids is not None:
                    aruco.drawDetectedMarkers(img, corners, ids)
                    
                    for i, marker_id in enumerate(ids):
                        mid = int(marker_id[0])
                        detected_ids.append(mid)
                        
                        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                            [corners[i]], self.marker_size,
                            self.camera_matrix, self.dist_coeffs
                        )
                        
                        rvec, tvec = rvecs[0], tvecs[0]
                        cv2.drawFrameAxes(img, self.camera_matrix, self.dist_coeffs,
                                         rvec, tvec, 0.03, 2)
                        
                        distance = np.linalg.norm(tvec)
                        
                        if mid in self.floor_marker_ids:
                            cv2.putText(img, f"FLOOR {mid}", (int(corners[i][0][0][0]), int(corners[i][0][0][1]-10)),
                                       self.font, 0.7, (0, 255, 255), 2)
                            
                            pose = Pose()
                            pose.position.x = float(tvec[0][0])
                            pose.position.y = float(tvec[0][1])
                            pose.position.z = float(tvec[0][2])
                            floor_poses.poses.append(pose)
                            
                            detection_status.append(f"Floor {mid} @ {distance:.2f}m")
                        
                        elif mid == self.box_marker_id:
                            cv2.putText(img, f"BOX {mid}", (int(corners[i][0][0][0]), int(corners[i][0][0][1]-10)),
                                       self.font, 0.7, (255, 255, 0), 2)
                            
                            box_marker_pose = PoseStamped()
                            box_marker_pose.header.stamp = self.get_clock().now().to_msg()
                            box_marker_pose.header.frame_id = "camera"
                            box_marker_pose.pose.position.x = float(tvec[0][0])
                            box_marker_pose.pose.position.y = float(tvec[0][1])
                            box_marker_pose.pose.position.z = float(tvec[0][2])
                            self.box_marker_publisher.publish(box_marker_pose)
                            
                            detection_status.append(f"Box marker @ {distance:.2f}m")
                
                if len(floor_poses.poses) > 0:
                    self.floor_markers_publisher.publish(floor_poses)
                
                ids_msg = Int32MultiArray()
                ids_msg.data = detected_ids
                self.detected_marker_ids_publisher.publish(ids_msg)
                
                # STEP 2: YOLO Detection for Box + Handle
                if self.enable_yolo_detection and self.yolo_model is not None:
                    box_bbox, handle_bbox, yolo_result = self.detect_with_yolo(img)
                    
                    if box_bbox is not None:
                        x1, y1, x2, y2, conf = box_bbox
                        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 3)
                        cv2.putText(img, f"BOX {conf:.2f}", (x1, y1-10),
                                   self.font, 0.7, (0, 255, 0), 2)
                        detection_status.append(f"YOLO Box ({conf:.2f})")
                    
                    if handle_bbox is not None:
                        x1, y1, x2, y2, conf = handle_bbox
                        cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
                        cv2.putText(img, f"HANDLE {conf:.2f}", (x1, y1-10),
                                   self.font, 0.7, (255, 0, 255), 2)
                        detection_status.append(f"YOLO Handle ({conf:.2f})")
                
                # Display status
                status_text = " | ".join(detection_status) if detection_status else "No detections"
                cv2.putText(img, status_text, (10, 30), self.font, 0.6, (255, 255, 255), 2)
                cv2.putText(img, f"Markers: {len(detected_ids)}", (10, height-10),
                           self.font, 0.5, (255, 255, 255), 1)
            
            if detection_status:
                status_msg = String()
                status_msg.data = " | ".join(detection_status)
                self.detection_status_publisher.publish(status_msg)
            
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            self.image_publisher.publish(img_msg)
            
            cv2.imshow('Underwater Detection (YOLO + ArUco)', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.destroy_node()
                
        except Exception as e:
            self.get_logger().error(f"Error in update: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = UnderwaterDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

