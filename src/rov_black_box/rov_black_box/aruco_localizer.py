#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


def q_mult(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ], dtype=np.float64)


def q_conj(q):
    x, y, z, w = q
    return np.array([-x, -y, -z, w], dtype=np.float64)


def q_norm(q):
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return q / n


def rotate(q, v):
    # v' = q * [v,0] * q_conj
    q = q_norm(q)
    vx, vy, vz = v
    qv = np.array([vx, vy, vz, 0.0], dtype=np.float64)
    return q_mult(q_mult(q, qv), q_conj(q))[:3]


def tf_to_tq(tf: TransformStamped):
    t = np.array([tf.transform.translation.x,
                  tf.transform.translation.y,
                  tf.transform.translation.z], dtype=np.float64)
    q = np.array([tf.transform.rotation.x,
                  tf.transform.rotation.y,
                  tf.transform.rotation.z,
                  tf.transform.rotation.w], dtype=np.float64)
    return t, q_norm(q)


def tq_to_tf(parent, child, stamp, t, q):
    out = TransformStamped()
    out.header.stamp = stamp
    out.header.frame_id = parent
    out.child_frame_id = child
    out.transform.translation.x = float(t[0])
    out.transform.translation.y = float(t[1])
    out.transform.translation.z = float(t[2])
    out.transform.rotation.x = float(q[0])
    out.transform.rotation.y = float(q[1])
    out.transform.rotation.z = float(q[2])
    out.transform.rotation.w = float(q[3])
    return out


def compose(t1, q1, t2, q2):
    # T = T1 * T2
    q = q_norm(q_mult(q1, q2))
    t = t1 + rotate(q1, t2)
    return t, q


def inverse(t, q):
    q_inv = q_norm(q_conj(q))
    t_inv = rotate(q_inv, -t)
    return t_inv, q_inv


class ArucoLocalizer(Node):
    def __init__(self):
        super().__init__("aruco_localizer")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("rov_frame", "rov/base_link")
        self.declare_parameter("camera_frame", "bluerov2/camera_optical_frame")
        self.declare_parameter("marker_prefix_map", "aruco_")
        self.declare_parameter("marker_prefix_obs", "aruco_obs_")
        self.declare_parameter("preferred_ids", [4])  # try this first; add more later
        self.declare_parameter("publish_rate_hz", 20.0)

        self.map_frame = self.get_parameter("map_frame").value
        self.rov_frame = self.get_parameter("rov_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.marker_prefix_map = self.get_parameter("marker_prefix_map").value
        self.marker_prefix_obs = self.get_parameter("marker_prefix_obs").value
        self.preferred_ids = list(self.get_parameter("preferred_ids").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)  # standard tf2 listener pattern [web:17]
        self.tf_broadcaster = TransformBroadcaster(self)           # standard tf2 broadcaster pattern [web:235]

        self.create_timer(1.0 / max(self.publish_rate_hz, 0.01), self._tick)

        self.get_logger().info(f"map_frame={self.map_frame} rov_frame={self.rov_frame} camera_frame={self.camera_frame}")
        self.get_logger().info(f"preferred_ids={self.preferred_ids}")

    def _tick(self):
        now = rclpy.time.Time()

        for mid in self.preferred_ids:
            map_marker = f"{self.marker_prefix_map}{int(mid)}"
            obs_marker = f"{self.marker_prefix_obs}{int(mid)}"

            try:
                # map -> aruco_<id>
                T_map_marker = self.tf_buffer.lookup_transform(self.map_frame, map_marker, now)
                # camera -> aruco_obs_<id>
                T_cam_marker = self.tf_buffer.lookup_transform(self.camera_frame, obs_marker, now)
                # camera -> rov/base_link  (tf2 can invert the static rov->camera for us if present)
                T_cam_rov = self.tf_buffer.lookup_transform(self.camera_frame, self.rov_frame, now)
            except (LookupException, ConnectivityException, ExtrapolationException):
                continue

            t_mm, q_mm = tf_to_tq(T_map_marker)
            t_cm, q_cm = tf_to_tq(T_cam_marker)
            t_cr, q_cr = tf_to_tq(T_cam_rov)

            # map->camera = map->marker * inv(camera->marker)
            t_mc, q_mc = compose(t_mm, q_mm, *inverse(t_cm, q_cm))

            # map->rov = map->camera * camera->rov
            t_mr, q_mr = compose(t_mc, q_mc, t_cr, q_cr)

            out = tq_to_tf(self.map_frame, self.rov_frame, self.get_clock().now().to_msg(), t_mr, q_mr)
            self.tf_broadcaster.sendTransform(out)
            return  # published using the first marker that worked


def main():
    rclpy.init()
    node = ArucoLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

