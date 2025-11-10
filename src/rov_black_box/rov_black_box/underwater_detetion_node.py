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

gi.require_version('Gst', '1.0')
from gi.repository import Gst

class UnderwaterDetectionNode(Node):
    """
    Detects:
    1. Two ArUco markers on pool floor (for localization)
    2. Red box (target object)
    3. Handle on box (optional)
    """

    def __init__(self):
        super().__init__("underwater_detection_node")
        
        # Parameters
        self.declare_parameter("port", 5600)
        self.declare_parameter("enable_detection", True)
        
        # ArUco parameters - CORRECTED FOR 4X4
        self.declare_parameter("aruco_dict", "DICT_4X4_50")
        self.declare_parameter("floor_marker_ids", [10, 20])
        self.declare_parameter("box_marker_id", 42)
        self.declare_parameter("marker_size", 0.10)  # 10cm = 0.10m
        
        # Box detection parameters
        self.declare_parameter("enable_box_detection", True)
        self.declare_parameter("enable_handle_detection", True)
        
        self.port = self.get_parameter("port").value
        self.enable_detection = self.get_parameter("enable_detection").value
        self.aruco_dict_name = self.get_parameter("aruco_dict").value
        self.floor_marker_ids = self.get_parameter("floor_marker_ids").value
        self.box_marker_id = self.get_parameter("box_marker_id").value
        self.marker_size = self.get_parameter("marker_size").value
        self.enable_box_detection = self.get_parameter("enable_box_detection").value
        self.enable_handle_detection = self.get_parameter("enable_handle_detection").value
        
        # Camera calibration (your calibrated values)
        self.camera_matrix = np.array([
            [576.9, 0.0, 409.7],
            [0.0, 576.7, 268.7],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)

        self.dist_coeffs = np.array([
            0.004002, 0.009106, -0.002852, 0.000090, 0.015693,
        ], dtype=np.float32).reshape(5, 1)
        
        # ArUco setup - CORRECTED
        try:
            self.aruco_dict = aruco.getPredefinedDictionary(getattr(aruco, self.aruco_dict_name))
            self.get_logger().info(f"✓ Using ArUco dictionary: {self.aruco_dict_name}")
        except AttributeError:
            self.get_logger().error(f"✗ Dictionary {self.aruco_dict_name} not found! Using DICT_4X4_50 as fallback")
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
        
        # Start video stream
        self.run()
        self.create_timer(0.033, self.update)  # ~30 Hz
        
        self.get_logger().info('='*60)
        self.get_logger().info('Underwater Detection Node Started')
        self.get_logger().info(f'ArUco Dictionary: {self.aruco_dict_name}')
        self.get_logger().info(f'Marker Size: {self.marker_size}m')
        self.get_logger().info(f'Floor Markers: {self.floor_marker_ids}')
        self.get_logger().info(f'Box Marker: {self.box_marker_id}')
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
    
    def detect_red_box(self, img):
        """Detect red/orange box using color detection"""
        try:
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            
            # Red color ranges (includes orange)
            lower_red1 = np.array([0, 80, 80], dtype=np.uint8)
            upper_red1 = np.array([15, 255, 255], dtype=np.uint8)
            lower_red2 = np.array([165, 80, 80], dtype=np.uint8)
            upper_red2 = np.array([180, 255, 255], dtype=np.uint8)
            
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            red_mask = mask1 | mask2
            
            # Morphological operations
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel, iterations=3)
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=1)
            
            contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) > 0:
                box_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(box_contour)
                
                if area > 1000:
                    x, y, w, h = cv2.boundingRect(box_contour)
                    center = (x + w // 2, y + h // 2)
                    return box_contour, center, area
            
            return None, None, 0
        except Exception as e:
            self.get_logger().error(f"Error in box detection: {e}")
            return None, None, 0
    
    def detect_handle(self, img, box_contour):
        """Detect black handle within box region"""
        try:
            x, y, w, h = cv2.boundingRect(box_contour)
            box_region = img[y:y+h, x:x+w]
            
            hsv = cv2.cvtColor(box_region, cv2.COLOR_BGR2HSV)
            lower_black = np.array([0, 0, 0], dtype=np.uint8)
            upper_black = np.array([180, 100, 60], dtype=np.uint8)
            
            black_mask = cv2.inRange(hsv, lower_black, upper_black)
            
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 5))
            black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
            black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel, iterations=1)
            
            contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            best_handle = None
            best_score = 0
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < 200:
                    continue
                
                bx, by, bw, bh = cv2.boundingRect(contour)
                aspect = bw / (bh + 1e-6)
                
                if aspect > 4 and bw > 40:
                    score = aspect * area
                    if score > best_score:
                        best_score = score
                        best_handle = contour
            
            if best_handle is not None:
                bx, by, bw, bh = cv2.boundingRect(best_handle)
                handle_contour = best_handle.copy()
                handle_contour[:, :, 0] += x
                handle_contour[:, :, 1] += y
                handle_center = (x + bx + bw // 2, y + by + bh // 2)
                return handle_contour, handle_center
            
            return None, None
        except Exception as e:
            self.get_logger().error(f"Error in handle detection: {e}")
            return None, None
    
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
                # STEP 1: Detect ALL ArUco markers
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
                        
                        # Estimate pose
                        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                            [corners[i]], self.marker_size,
                            self.camera_matrix, self.dist_coeffs
                        )
                        
                        rvec, tvec = rvecs[0], tvecs[0]
                        
                        # Draw coordinate frame
                        cv2.drawFrameAxes(img, self.camera_matrix, self.dist_coeffs,
                                         rvec, tvec, 0.03, 2)
                        
                        distance = np.linalg.norm(tvec)
                        
                        # Check if floor marker
                        if mid in self.floor_marker_ids:
                            cv2.putText(img, f"FLOOR {mid}", (int(corners[i][0][0][0]), int(corners[i][0][0][1]-10)),
                                       self.font, 0.7, (0, 255, 255), 2)
                            
                            pose = Pose()
                            pose.position.x = float(tvec[0][0])
                            pose.position.y = float(tvec[0][1])
                            pose.position.z = float(tvec[0][2])
                            floor_poses.poses.append(pose)
                            
                            detection_status.append(f"Floor {mid} @ {distance:.2f}m")
                        
                        # Check if box marker
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
                
                # Publish floor markers
                if len(floor_poses.poses) > 0:
                    self.floor_markers_publisher.publish(floor_poses)
                
                # Publish detected IDs
                ids_msg = Int32MultiArray()
                ids_msg.data = detected_ids
                self.detected_marker_ids_publisher.publish(ids_msg)
                
                # STEP 2: Detect red box
                if self.enable_box_detection:
                    box_contour, box_center, box_area = self.detect_red_box(img)
                    
                    if box_contour is not None:
                        x, y, w, h = cv2.boundingRect(box_contour)
                        cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 3)
                        cv2.circle(img, box_center, 5, (0, 255, 0), -1)
                        cv2.putText(img, f"RED BOX", (x, y-10),
                                   self.font, 0.7, (0, 255, 0), 2)
                        
                        detection_status.append(f"Red box")
                        
                        # STEP 3: Detect handle
                        if self.enable_handle_detection:
                            handle_contour, handle_center = self.detect_handle(img, box_contour)
                            
                            if handle_contour is not None:
                                cv2.drawContours(img, [handle_contour], 0, (255, 0, 255), 3)
                                cv2.circle(img, handle_center, 5, (255, 0, 255), -1)
                                cv2.putText(img, "HANDLE", (handle_center[0]+10, handle_center[1]-10),
                                           self.font, 0.6, (255, 0, 255), 2)
                                detection_status.append("Handle")
                
                # Display status
                status_text = " | ".join(detection_status) if detection_status else "No detections"
                cv2.putText(img, status_text, (10, 30), self.font, 0.6, (255, 255, 255), 2)
                cv2.putText(img, f"Markers: {len(detected_ids)}", (10, height-10),
                           self.font, 0.5, (255, 255, 255), 1)
            
            # Publish status
            if detection_status:
                status_msg = String()
                status_msg.data = " | ".join(detection_status)
                self.detection_status_publisher.publish(status_msg)
            
            # Publish image
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            self.image_publisher.publish(img_msg)
            
            cv2.imshow('Underwater Detection', img)
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

