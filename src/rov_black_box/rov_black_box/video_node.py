#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node

import cv2
import gi
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

gi.require_version('Gst', '1.0')
from gi.repository import Gst


class VideoNode(Node):
    def __init__(self):
        super().__init__("video_node")

        # Parameters
        self.declare_parameter("port", 5600)
        self.declare_parameter("show_window", True)
        self.port = int(self.get_parameter("port").value)
        self.show_window = bool(self.get_parameter("show_window").value)

        self._frame = None

        Gst.init(None)

        self.pipeline_str = (
            f"udpsrc port={self.port} "
            "! application/x-rtp, payload=96 "
            "! rtpjitterbuffer drop-on-latency=true "
            "! rtph264depay ! h264parse ! avdec_h264 "
            "! videoconvert ! video/x-raw,format=BGR "
            "! appsink name=appsink0 emit-signals=true sync=false max-buffers=1 drop=true"
        )

        self.video_pipe = None
        self.video_sink = None

        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, 'camera/image_raw', 10)

        self.window_name = 'BlueROV2 Camera Preview'
        if self.show_window and self._gui_available():
            try:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                placeholder = np.zeros((540, 960, 3), dtype=np.uint8)
                cv2.putText(placeholder, 'Waiting for video...', (40, 280),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (180, 180, 180), 2)
                cv2.imshow(self.window_name, placeholder)
                cv2.waitKey(1)
            except cv2.error as e:
                self.get_logger().warn(f"OpenCV GUI not available: {e}; running headless")
                self.show_window = False

        self.start_gst()
        self.create_timer(0.033, self.update)

        self.get_logger().info(f"Video node started on UDP port {self.port}")
        self.get_logger().info("Publishing camera/image_raw for teleop + autonomy")

    def start_gst(self):
        try:
            self.video_pipe = Gst.parse_launch(self.pipeline_str)
        except Exception as e:
            self.get_logger().error(f"Failed to create GStreamer pipeline: {e}")
            return

        bus = self.video_pipe.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self.on_bus_message)

        self.video_sink = self.video_pipe.get_by_name('appsink0')
        if not self.video_sink:
            self.get_logger().error("Could not find appsink0 in pipeline")
            return

        self.video_sink.connect('new-sample', self.callback)

        ret = self.video_pipe.set_state(Gst.State.PLAYING)
        if ret not in (Gst.StateChangeReturn.SUCCESS, Gst.StateChangeReturn.ASYNC):
            self.get_logger().error(f"Failed to set pipeline to PLAYING, ret={ret}")

    def on_bus_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            self.get_logger().error(f"GStreamer ERROR: {err} | {debug}")
        elif t == Gst.MessageType.WARNING:
            warn, debug = message.parse_warning()
            self.get_logger().warn(f"GStreamer WARNING: {warn} | {debug}")
        elif t == Gst.MessageType.EOS:
            self.get_logger().warn("GStreamer EOS received")

    @staticmethod
    def gst_to_opencv(sample):
        buf = sample.get_buffer()
        caps = sample.get_caps()
        s = caps.get_structure(0)
        width = s.get_value('width')
        height = s.get_value('height')
        data = buf.extract_dup(0, buf.get_size())
        arr = np.ndarray((height, width, 3), dtype=np.uint8, buffer=data)
        return arr

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.ERROR
        try:
            self._frame = self.gst_to_opencv(sample)
        except Exception as e:
            self.get_logger().error(f"GStreamer callback error: {e}")
            return Gst.FlowReturn.ERROR
        return Gst.FlowReturn.OK

    def frame_available(self):
        return self._frame is not None

    def frame(self):
        return self._frame

    def update(self):
        if not self.frame_available():
            if self.show_window and self._gui_available():
                cv2.waitKey(1)
            return

        frame = self.frame()

        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_frame'
            self.image_publisher.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish image: {e}")

        if self.show_window and self._gui_available():
            try:
                h, w = frame.shape[:2]
                display_img = cv2.resize(frame, (w, h), interpolation=cv2.INTER_AREA)
                cv2.imshow(self.window_name, display_img)
                cv2.waitKey(1)
            except cv2.error as e:
                self.get_logger().warn(f"OpenCV preview error: {e}")

    def _gui_available(self):
        return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))

    def destroy_node(self):
        if self.video_pipe:
            self.video_pipe.set_state(Gst.State.NULL)
        try:
            if self.show_window:
                cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


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

