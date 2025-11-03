import rclpy
from rclpy.node import Node
import cv2
import gi
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from cv2 import aruco

gi.require_version('Gst', '1.0')
from gi.repository import Gst

class VideoNode(Node):
    """BlueRov video capture with ArUco-based box localization"""

    def __init__(self):
        super().__init__("video_node")

        self.declare_parameter("port", 5600) 
        self.declare_parameter("enable_detection", True)
        self.declare_parameter("box_marker_id_1", 10)
        self.declare_parameter("box_marker_id_2", 11)
        self.declare_parameter("marker_size_1", 0.05)  
        self.declare_parameter("marker_size_2", 0.08)  
        
        # Box geometry
        self.declare_parameter("box_length", 0.30)
        self.declare_parameter("box_width", 0.16)
        self.declare_parameter("box_height", 0.14)
        
        # Marker positions relative to box center
        self.declare_parameter("marker1_offset_x", 0.15)
        self.declare_parameter("marker1_offset_y", 0.0)
        self.declare_parameter("marker1_offset_z", 0.0)
        self.declare_parameter("marker2_offset_x", -0.15)
        self.declare_parameter("marker2_offset_y", 0.0)
        self.declare_parameter("marker2_offset_z", 0.0)
        
        # Handle position relative to box center
        self.declare_parameter("handle_offset_x", 0.0)
        self.declare_parameter("handle_offset_y", 0.0)
        self.declare_parameter("handle_offset_z", 0.07)

        self.port = self.get_parameter("port").value
        self.enable_detection = self.get_parameter("enable_detection").value
        self.box_marker_1 = self.get_parameter("box_marker_id_1").value
        self.box_marker_2 = self.get_parameter("box_marker_id_2").value
        self.marker_size_1 = self.get_parameter("marker_size_1").value
        self.marker_size_2 = self.get_parameter("marker_size_2").value
        
        # Box dimensions
        self.box_dims = np.array([
            self.get_parameter("box_length").value,
            self.get_parameter("box_width").value,
            self.get_parameter("box_height").value
        ])
        
        # Marker sizes dictionary
        self.marker_sizes = {
            self.box_marker_1: self.marker_size_1,
            self.box_marker_2: self.marker_size_2
        }
        
        # Marker offsets from box center
        self.marker_offsets = {
            self.box_marker_1: np.array([
                self.get_parameter("marker1_offset_x").value,
                self.get_parameter("marker1_offset_y").value,
                self.get_parameter("marker1_offset_z").value
            ]),
            self.box_marker_2: np.array([
                self.get_parameter("marker2_offset_x").value,
                self.get_parameter("marker2_offset_y").value,
                self.get_parameter("marker2_offset_z").value
            ])
        }
        
        # Handle offset
        self.handle_offset = np.array([
            self.get_parameter("handle_offset_x").value,
            self.get_parameter("handle_offset_y").value,
            self.get_parameter("handle_offset_z").value
        ])
        
        self._frame = None
        self.video_source = 'udpsrc port={}'.format(self.port)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        self.video_decode = '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        self.video_sink_conf = '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        # Camera calibration - remember to REPLACE
        self.camera_matrix = np.array([
            [800.0, 0.0, 480.0],
            [0.0, 800.0, 270.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        
        # ArUco setup
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        Gst.init() 
        self.bridge = CvBridge()

        # Publishers
        self.image_publisher = self.create_publisher(Image, 'camera/image', 10)
        self.detection_publisher = self.create_publisher(String, 'object_detection/status', 10)
        self.box_pose_publisher = self.create_publisher(PoseStamped, 'object_detection/box_pose', 10)
        self.handle_pose_publisher = self.create_publisher(PoseStamped, 'object_detection/handle_pose', 10)
        self.detected_marker_id_publisher = self.create_publisher(Int32, 'object_detection/visible_marker', 10)

        self.run()
        self.create_timer(0.033, self.update)
        
        self.get_logger().info(f'Video node started')
        self.get_logger().info(f'Marker {self.box_marker_1}: {self.marker_size_1}m')
        self.get_logger().info(f'Marker {self.box_marker_2}: {self.marker_size_2}m')

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
        sample = sink.emit('pull-sample')
        new_frame = self.gst_to_opencv(sample)
        self._frame = new_frame
        return Gst.FlowReturn.OK
    
    def estimate_box_pose_from_single_marker(self, marker_id, marker_corners):
        """
        Calculate box and handle pose using correct marker size
        """
        if marker_id not in self.marker_sizes:
            return None, None
        
        # Get correct marker size
        marker_size = self.marker_sizes[marker_id]
        
        # Estimate pose with correct size
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            [marker_corners],
            marker_size,
            self.camera_matrix,
            self.dist_coeffs
        )
        
        marker_rvec = rvecs[0]
        marker_tvec = tvecs[0]
        
        if marker_id not in self.marker_offsets:
            return None, None
        
        # Convert to rotation matrix
        marker_rotation, _ = cv2.Rodrigues(marker_rvec)
        
        # Calculate box center
        marker_offset_box = self.marker_offsets[marker_id]
        marker_offset_camera = marker_rotation @ marker_offset_box
        box_tvec = marker_tvec - marker_offset_camera.reshape(3, 1)
        
        box_rvec = marker_rvec
        box_rotation = marker_rotation
        
        # Calculate handle position
        handle_offset_camera = box_rotation @ self.handle_offset
        handle_tvec = box_tvec + handle_offset_camera.reshape(3, 1)
        
        return (box_rvec, box_tvec, box_rotation), (box_rvec, handle_tvec)
    
    def project_3d_point_to_2d(self, point_3d):
        point_2d, _ = cv2.projectPoints(
            point_3d.reshape(1, 1, 3),
            np.zeros(3),
            np.zeros(3),
            self.camera_matrix,
            self.dist_coeffs
        )
        return tuple(point_2d[0][0].astype(int))
    
    def draw_box_outline(self, img, box_rvec, box_tvec):
        """Draw 3D bounding box"""
        half_dims = self.box_dims / 2
        corners_3d = np.array([
            [-half_dims[0], -half_dims[1], -half_dims[2]],
            [ half_dims[0], -half_dims[1], -half_dims[2]],
            [ half_dims[0],  half_dims[1], -half_dims[2]],
            [-half_dims[0],  half_dims[1], -half_dims[2]],
            [-half_dims[0], -half_dims[1],  half_dims[2]],
            [ half_dims[0], -half_dims[1],  half_dims[2]],
            [ half_dims[0],  half_dims[1],  half_dims[2]],
            [-half_dims[0],  half_dims[1],  half_dims[2]]
        ], dtype=np.float32)
        
        corners_2d, _ = cv2.projectPoints(
            corners_3d,
            box_rvec,
            box_tvec,
            self.camera_matrix,
            self.dist_coeffs
        )
        
        corners_2d = corners_2d.reshape(-1, 2).astype(int)
        
        # Draw edges
        for i in range(4):
            cv2.line(img, tuple(corners_2d[i]), tuple(corners_2d[(i+1)%4]), (0, 255, 0), 2)
        for i in range(4, 8):
            cv2.line(img, tuple(corners_2d[i]), tuple(corners_2d[4+(i+1)%4]), (0, 255, 0), 2)
        for i in range(4):
            cv2.line(img, tuple(corners_2d[i]), tuple(corners_2d[i+4]), (0, 255, 0), 2)
    
    def update(self):        
        if not self.frame_available():
            return

        frame = self.frame()
        width = int(1920/2)
        height = int(1080/2)
        dim = (width, height)
        img = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
        
        detection_status = []
        
        if self.enable_detection:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
            
            if ids is not None:
                aruco.drawDetectedMarkers(img, corners, ids)
            
            box_detected = False
            detected_marker_id = None
            
            if ids is not None:
                for i, marker_id in enumerate(ids):
                    if marker_id[0] == self.box_marker_1 or marker_id[0] == self.box_marker_2:
                        detected_marker_id = int(marker_id[0])
                        
                        box_pose, handle_pose = self.estimate_box_pose_from_single_marker(
                            detected_marker_id, corners[i]
                        )
                        
                        if box_pose is not None:
                            box_detected = True
                            box_rvec, box_tvec, box_rotation = box_pose
                            handle_rvec, handle_tvec = handle_pose
                            
                            # Draw coordinate frame
                            cv2.drawFrameAxes(img, self.camera_matrix, self.dist_coeffs, 
                                             box_rvec, box_tvec, 0.1, 3)
                            
                            # Draw 3D box
                            self.draw_box_outline(img, box_rvec, box_tvec)
                            
                            # Draw box center
                            box_center_2d = self.project_3d_point_to_2d(box_tvec[0])
                            cv2.circle(img, box_center_2d, 8, (0, 255, 0), -1)
                            cv2.putText(img, "Box Center", (box_center_2d[0] + 10, box_center_2d[1]), 
                                       self.font, 0.6, (0, 255, 0), 2)
                            
                            # Draw handle
                            handle_center_2d = self.project_3d_point_to_2d(handle_tvec[0])
                            cv2.circle(img, handle_center_2d, 8, (255, 0, 255), -1)
                            cv2.putText(img, "Handle", (handle_center_2d[0] + 10, handle_center_2d[1]), 
                                       self.font, 0.6, (255, 0, 255), 2)
                            
                            cv2.line(img, box_center_2d, handle_center_2d, (255, 255, 0), 2)
                            
                            # Distance
                            distance = np.linalg.norm(box_tvec[0])
                            marker_size = self.marker_sizes[detected_marker_id]
                            cv2.putText(img, f"Distance: {distance:.2f}m", (10, 30), 
                                       self.font, 0.7, (255, 255, 255), 2)
                            cv2.putText(img, f"Marker {detected_marker_id} ({marker_size*100:.1f}cm)", (10, 60), 
                                       self.font, 0.6, (255, 255, 0), 2)
                            
                            # Publish poses
                            box_pose_msg = PoseStamped()
                            box_pose_msg.header.stamp = self.get_clock().now().to_msg()
                            box_pose_msg.header.frame_id = "camera"
                            box_pose_msg.pose.position.x = float(box_tvec[0][0])
                            box_pose_msg.pose.position.y = float(box_tvec[0][1])
                            box_pose_msg.pose.position.z = float(box_tvec[0][2])
                            self.box_pose_publisher.publish(box_pose_msg)
                            
                            handle_pose_msg = PoseStamped()
                            handle_pose_msg.header.stamp = self.get_clock().now().to_msg()
                            handle_pose_msg.header.frame_id = "camera"
                            handle_pose_msg.pose.position.x = float(handle_tvec[0][0])
                            handle_pose_msg.pose.position.y = float(handle_tvec[0][1])
                            handle_pose_msg.pose.position.z = float(handle_tvec[0][2])
                            self.handle_pose_publisher.publish(handle_pose_msg)
                            
                            marker_msg = Int32()
                            marker_msg.data = detected_marker_id
                            self.detected_marker_id_publisher.publish(marker_msg)
                            
                            detection_status.append(f"Box at {distance:.2f}m (Marker {detected_marker_id})")
                        
                        break
            
            if not box_detected:
                cv2.putText(img, "Box not detected", (10, 30), 
                           self.font, 0.7, (0, 0, 255), 2)
        
        if detection_status:
            status_msg = String()
            status_msg.data = " | ".join(detection_status)
            self.detection_publisher.publish(status_msg)
        
        # Crosshair
        cv2.drawMarker(img, (width//2, height//2), (0, 255, 255), 
                      cv2.MARKER_CROSS, 20, 2)
        
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.image_publisher.publish(img_msg)

        cv2.imshow('BlueROV2 Camera - Box Localization', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()


def main(args=None):
    rclpy.init(args=args)    
    node = VideoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

