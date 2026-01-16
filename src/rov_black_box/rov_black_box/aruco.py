#!/usr/bin/env python3
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PoseArray, Pose
from tf2_ros import TransformBroadcaster

from cv_bridge import CvBridge


def rotmat_to_quat(R: np.ndarray):
    """3x3 rotation matrix -> quaternion (x,y,z,w)."""
    tr = float(R[0, 0] + R[1, 1] + R[2, 2])
    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    return float(qx), float(qy), float(qz), float(qw)


def estimate_pose_solvepnp_square(corners_4x2, marker_length_m, K, D):
    """
    corners_4x2: (4,2) image points for the marker corners (pixel coords).
    Returns (rvec, tvec) where pose is marker->camera, as OpenCV convention for solvePnP.
    """
    L = float(marker_length_m)
    half = L / 2.0

    # Required object-point order for SOLVEPNP_IPPE_SQUARE:
    # 0: (-L/2, +L/2, 0)
    # 1: (+L/2, +L/2, 0)
    # 2: (+L/2, -L/2, 0)
    # 3: (-L/2, -L/2, 0)
    obj_pts = np.array(
        [
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )

    img_pts = np.asarray(corners_4x2, dtype=np.float64).reshape(4, 2)

    ok, rvec, tvec = cv2.solvePnP(
        obj_pts, img_pts, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE
    )
    if not ok:
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return None, None
    return rvec, tvec


class ArucoVisionDetector(Node):
    def __init__(self):
        super().__init__("aruco_vision_detector")

        # --- Topics / frames
        self.declare_parameter("image_topic", "/bluerov2/camera/image_raw")
        self.declare_parameter("use_camera_info", False)
        self.declare_parameter("camera_info_topic", "/bluerov2/camera/camera_info")
        self.declare_parameter("camera_frame", "bluerov2/camera_optical_frame")

        # --- ArUco
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("marker_length_m", 0.30)

        # --- Calibration defaults (your values)
        self.declare_parameter(
            "camera_matrix",
            [576.9, 0.0, 409.7,
             0.0, 576.7, 268.7,
             0.0, 0.0, 1.0]
        )
        self.declare_parameter(
            "dist_coeffs",
            [0.004002, 0.009106, -0.002852, 0.000090, 0.015693]
        )

        # --- Outputs
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("tf_child_prefix", "aruco_obs_")
        self.declare_parameter("publish_pose_array", True)
        self.declare_parameter("pose_array_topic", "/aruco/poses")

        # --- Debug view (headless switch)
        self.declare_parameter("show_debug_view", True)
        self.declare_parameter("debug_window_name", "Aruco Detection (annotated)")
        self.declare_parameter("draw_axes", True)

        # Read params
        self.image_topic = self.get_parameter("image_topic").value
        self.use_camera_info = bool(self.get_parameter("use_camera_info").value)
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.camera_frame = self.get_parameter("camera_frame").value

        self.dict_name = self.get_parameter("dictionary").value
        self.marker_len = float(self.get_parameter("marker_length_m").value)

        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.child_prefix = self.get_parameter("tf_child_prefix").value
        self.publish_pose_array = bool(self.get_parameter("publish_pose_array").value)
        self.pose_array_topic = self.get_parameter("pose_array_topic").value

        self.show_debug_view = bool(self.get_parameter("show_debug_view").value)
        self.window_name = self.get_parameter("debug_window_name").value
        self.draw_axes = bool(self.get_parameter("draw_axes").value)

        # ArUco detector (OpenCV 4.10 API)
        if not hasattr(cv2.aruco, self.dict_name):
            raise RuntimeError(f"Unknown ArUco dictionary: {self.dict_name}")
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.dict_name))
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Calib
        self.K = None
        self.D = None
        self._load_calib_from_params()

        if self.use_camera_info:
            self.create_subscription(CameraInfo, self.camera_info_topic, self._cam_info_cb, 10)

        # ROS IO
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseArray, self.pose_array_topic, 10)

        self.create_subscription(Image, self.image_topic, self._image_cb, 10)

        if self.show_debug_view:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        self.get_logger().info(f"image_topic: {self.image_topic}")
        self.get_logger().info(f"camera_frame: {self.camera_frame}")
        self.get_logger().info(f"use_camera_info: {self.use_camera_info}")
        self.get_logger().info(f"show_debug_view: {self.show_debug_view}")

    def _load_calib_from_params(self):
        Kflat = list(self.get_parameter("camera_matrix").value)
        Dflat = list(self.get_parameter("dist_coeffs").value)

        if len(Kflat) != 9:
            raise RuntimeError("camera_matrix must be length 9")
        self.K = np.array(Kflat, dtype=np.float64).reshape(3, 3)

        # D can be 4/5/8/… depending on model; keep as-is
        self.D = np.array(Dflat, dtype=np.float64)

    def _cam_info_cb(self, msg: CameraInfo):
        # If you later provide camera_info, it overrides K/D
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64)
        if msg.header.frame_id:
            self.camera_frame = msg.header.frame_id

    def _to_bgr(self, img_msg: Image):
        # Best-effort conversion
        try:
            return self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception:
            cv = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
            if cv is None:
                return None
            if len(cv.shape) == 2:
                return cv2.cvtColor(cv, cv2.COLOR_GRAY2BGR)
            if len(cv.shape) == 3 and cv.shape[2] == 3:
                return cv
            if len(cv.shape) == 3 and cv.shape[2] == 4:
                return cv2.cvtColor(cv, cv2.COLOR_BGRA2BGR)
            return None

    def _image_cb(self, msg: Image):
        img = self._to_bgr(msg)
        if img is None:
            return

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = self.detector.detectMarkers(gray)

        if ids is None or len(ids) == 0:
            if self.show_debug_view:
                cv2.imshow(self.window_name, img)
                cv2.waitKey(1)
            return

        # Draw detected markers
        cv2.aruco.drawDetectedMarkers(img, corners, ids)

        now = self.get_clock().now().to_msg()

        if self.publish_pose_array:
            pa = PoseArray()
            pa.header.stamp = now
            pa.header.frame_id = self.camera_frame

        for i in range(len(ids)):
            mid = int(ids[i][0])

            # corners[i] is usually shape (1,4,2); reshape to (4,2)
            c = corners[i].reshape(4, 2)

            rvec, tvec = estimate_pose_solvepnp_square(c, self.marker_len, self.K, self.D)
            if rvec is None:
                continue

            # Optional axes overlay
            if self.draw_axes:
                try:
                    cv2.drawFrameAxes(img, self.K, self.D, rvec, tvec, self.marker_len * 0.5)
                except Exception:
                    pass

            R, _ = cv2.Rodrigues(rvec)
            qx, qy, qz, qw = rotmat_to_quat(R)

            if self.publish_tf:
                t = TransformStamped()
                t.header.stamp = now
                t.header.frame_id = self.camera_frame
                t.child_frame_id = f"{self.child_prefix}{mid}"

                t.transform.translation.x = float(tvec[0])
                t.transform.translation.y = float(tvec[1])
                t.transform.translation.z = float(tvec[2])

                t.transform.rotation.x = qx
                t.transform.rotation.y = qy
                t.transform.rotation.z = qz
                t.transform.rotation.w = qw

                self.tf_broadcaster.sendTransform(t)

            if self.publish_pose_array:
                p = Pose()
                p.position.x = float(tvec[0])
                p.position.y = float(tvec[1])
                p.position.z = float(tvec[2])
                p.orientation.x = qx
                p.orientation.y = qy
                p.orientation.z = qz
                p.orientation.w = qw
                pa.poses.append(p)

        if self.publish_pose_array:
            self.pose_pub.publish(pa)

        if self.show_debug_view:
            cv2.imshow(self.window_name, img)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.show_debug_view:
            try:
                cv2.destroyWindow(self.window_name)
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = ArucoVisionDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

