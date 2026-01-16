#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from visualization_msgs.msg import Marker, MarkerArray


class ArucoDetector(Node):
    """
    RViz visualization helper:
    - ArUco map:
        TF: parent_frame -> aruco_<id> (static)
        MarkerArray: /aruco_map_markers (republished periodically)
    - ROV placeholder:
        TF: parent_frame -> rov/base_link (dynamic, optional)
        Marker: /rov_marker (periodic)
      driven by PoseStamped on /rov/pose (pose is assumed expressed in parent_frame).
    """

    def __init__(self):
        super().__init__("aruco_detector")

        # ---- Basic params (with defaults) ----
        self.declare_parameter("parent_frame", "map")
        self.declare_parameter("child_prefix", "aruco_")

        self.declare_parameter("publish_aruco_tfs", True)
        self.declare_parameter("publish_aruco_markers", True)
        self.declare_parameter("aruco_marker_topic", "/aruco_map_markers")

        # ---- IMPORTANT: declare array params with explicit types (avoid [] => BYTE_ARRAY) ----
        # rclpy allows declaring parameter type via rclpy.Parameter.Type.* [web:301]
        self.declare_parameter("aruco_ids", rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter("aruco_x", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("aruco_y", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("aruco_z", rclpy.Parameter.Type.DOUBLE_ARRAY)

        # ---- ROV placeholder ----
        self.declare_parameter("publish_rov_tf", True)
        self.declare_parameter("publish_rov_marker", True)
        self.declare_parameter("rov_frame", "rov/base_link")
        self.declare_parameter("rov_pose_topic", "/rov/pose")
        self.declare_parameter("rov_marker_topic", "/rov_marker")

        # ---- Marker sizes + rates ----
        self.declare_parameter("aruco_cube_xy", 0.10)
        self.declare_parameter("aruco_cube_z", 0.02)

        self.declare_parameter("rov_arrow_len", 0.6)
        self.declare_parameter("rov_arrow_w", 0.15)
        self.declare_parameter("rov_arrow_h", 0.15)

        self.declare_parameter("aruco_marker_rate_hz", 1.0)
        self.declare_parameter("rov_publish_rate_hz", 20.0)

        # ---- Read params ----
        self.parent_frame = str(self.get_parameter("parent_frame").value)
        self.child_prefix = str(self.get_parameter("child_prefix").value)

        self.publish_aruco_tfs = bool(self.get_parameter("publish_aruco_tfs").value)
        self.publish_aruco_markers = bool(self.get_parameter("publish_aruco_markers").value)
        self.aruco_marker_topic = str(self.get_parameter("aruco_marker_topic").value)

        self.publish_rov_tf = bool(self.get_parameter("publish_rov_tf").value)
        self.publish_rov_marker = bool(self.get_parameter("publish_rov_marker").value)
        self.rov_frame = str(self.get_parameter("rov_frame").value)
        self.rov_pose_topic = str(self.get_parameter("rov_pose_topic").value)
        self.rov_marker_topic = str(self.get_parameter("rov_marker_topic").value)

        self.aruco_cube_xy = float(self.get_parameter("aruco_cube_xy").value)
        self.aruco_cube_z = float(self.get_parameter("aruco_cube_z").value)

        self.rov_arrow_len = float(self.get_parameter("rov_arrow_len").value)
        self.rov_arrow_w = float(self.get_parameter("rov_arrow_w").value)
        self.rov_arrow_h = float(self.get_parameter("rov_arrow_h").value)

        self.aruco_marker_rate_hz = float(self.get_parameter("aruco_marker_rate_hz").value)
        self.rov_publish_rate_hz = float(self.get_parameter("rov_publish_rate_hz").value)

        # ---- Read arrays (may be None if not set in YAML) ----
        self.aruco_ids = self._as_list(self.get_parameter("aruco_ids").value)
        self.aruco_x = self._as_list(self.get_parameter("aruco_x").value)
        self.aruco_y = self._as_list(self.get_parameter("aruco_y").value)
        self.aruco_z = self._as_list(self.get_parameter("aruco_z").value)

        if not (len(self.aruco_ids) == len(self.aruco_x) == len(self.aruco_y) == len(self.aruco_z)):
            self.get_logger().error(
                f"Aruco arrays must have same length. "
                f"ids={len(self.aruco_ids)} x={len(self.aruco_x)} y={len(self.aruco_y)} z={len(self.aruco_z)}"
            )
            self.aruco_ids = []
            self.aruco_x = []
            self.aruco_y = []
            self.aruco_z = []

        # ---- TF + publishers ----
        self.tf_static = StaticTransformBroadcaster(self)  # static TF broadcaster [web:1]
        self.tf_dyn = TransformBroadcaster(self)           # dynamic TF broadcaster [web:235]

        self.aruco_marker_pub = self.create_publisher(MarkerArray, self.aruco_marker_topic, 10)
        self.rov_marker_pub = self.create_publisher(Marker, self.rov_marker_topic, 10)

        self.last_pose_msg: PoseStamped | None = None
        self.create_subscription(PoseStamped, self.rov_pose_topic, self._pose_cb, 10)

        # Publish static TFs once
        if self.publish_aruco_tfs and self.aruco_ids:
            self._publish_aruco_static_tfs()

        # Timers (republish markers so RViz will see them reliably) [web:21]
        if self.publish_aruco_markers:
            period = 1.0 / max(self.aruco_marker_rate_hz, 0.01)
            self.create_timer(period, self._publish_aruco_marker_array)

        rov_period = 1.0 / max(self.rov_publish_rate_hz, 0.01)
        self.create_timer(rov_period, self._publish_rov_outputs)

        self.get_logger().info(f"parent_frame: {self.parent_frame}")
        self.get_logger().info(f"ArUco count: {len(self.aruco_ids)} topic: {self.aruco_marker_topic}")
        self.get_logger().info(f"ROV pose: {self.rov_pose_topic} -> rov frame: {self.rov_frame}")

    @staticmethod
    def _as_list(v):
        if v is None:
            return []
        try:
            return list(v)
        except TypeError:
            return []

    def _pose_cb(self, msg: PoseStamped):
        self.last_pose_msg = msg

    def _publish_aruco_static_tfs(self):
        now = self.get_clock().now().to_msg()
        tfs = []

        for mid, x, y, z in zip(self.aruco_ids, self.aruco_x, self.aruco_y, self.aruco_z):
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.parent_frame
            t.child_frame_id = f"{self.child_prefix}{int(mid)}"

            t.transform.translation.x = float(x)
            t.transform.translation.y = float(y)
            t.transform.translation.z = float(z)

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            tfs.append(t)

        self.tf_static.sendTransform(tfs)

    def _publish_aruco_marker_array(self):
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()

        for mid, x, y, z in zip(self.aruco_ids, self.aruco_x, self.aruco_y, self.aruco_z):
            mk = Marker()
            mk.header.stamp = now
            mk.header.frame_id = self.parent_frame
            mk.ns = "aruco_map"
            mk.id = int(mid)

            mk.type = Marker.CUBE
            mk.action = Marker.ADD

            mk.pose.position.x = float(x)
            mk.pose.position.y = float(y)
            mk.pose.position.z = float(z)
            mk.pose.orientation.w = 1.0

            mk.scale.x = self.aruco_cube_xy
            mk.scale.y = self.aruco_cube_xy
            mk.scale.z = self.aruco_cube_z

            mk.color.r = 1.0
            mk.color.g = 0.4
            mk.color.b = 0.0
            mk.color.a = 0.9

            arr.markers.append(mk)

        self.aruco_marker_pub.publish(arr)

    def _publish_rov_outputs(self):
        now = self.get_clock().now().to_msg()

        if self.last_pose_msg is None:
            px = py = pz = 0.0
            qx = qy = qz = 0.0
            qw = 1.0
        else:
            p = self.last_pose_msg.pose.position
            q = self.last_pose_msg.pose.orientation
            px, py, pz = p.x, p.y, p.z
            qx, qy, qz, qw = q.x, q.y, q.z, q.w

        if self.publish_rov_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.parent_frame
            t.child_frame_id = self.rov_frame

            t.transform.translation.x = float(px)
            t.transform.translation.y = float(py)
            t.transform.translation.z = float(pz)

            t.transform.rotation.x = float(qx)
            t.transform.rotation.y = float(qy)
            t.transform.rotation.z = float(qz)
            t.transform.rotation.w = float(qw)

            self.tf_dyn.sendTransform(t)

        if self.publish_rov_marker:
            mk = Marker()
            mk.header.stamp = now
            mk.header.frame_id = self.parent_frame
            mk.ns = "rov"
            mk.id = 0

            mk.type = Marker.ARROW
            mk.action = Marker.ADD

            mk.pose.position.x = float(px)
            mk.pose.position.y = float(py)
            mk.pose.position.z = float(pz)
            mk.pose.orientation.x = float(qx)
            mk.pose.orientation.y = float(qy)
            mk.pose.orientation.z = float(qz)
            mk.pose.orientation.w = float(qw)

            mk.scale.x = self.rov_arrow_len
            mk.scale.y = self.rov_arrow_w
            mk.scale.z = self.rov_arrow_h

            mk.color.r = 0.1
            mk.color.g = 0.3
            mk.color.b = 1.0
            mk.color.a = 0.9

            self.rov_marker_pub.publish(mk)


def main():
    rclpy.init()
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

