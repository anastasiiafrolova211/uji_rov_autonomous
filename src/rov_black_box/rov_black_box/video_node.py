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

        # --- CONFIGURATION ---
        self.declare_parameter("port", 5600)
        # We default to 640x360 (16:9 aspect ratio)
        # This is PLENTY for YOLO and makes the system 5x faster than 1080p
        self.declare_parameter("width", 1920)   
        self.declare_parameter("height", 1080)  
        
        self.port = int(self.get_parameter("port").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)

        self._frame = None

        Gst.init(None)

        # --- OPTIMIZED PIPELINE ---
        # 1. 'videoscale' resizes the image efficiently in C++ before Python sees it.
        # 2. We use a queue to decouple decoding from publishing.
        self.pipeline_str = (
            f"udpsrc port={self.port} "
            "! application/x-rtp, payload=96 "
            "! rtpjitterbuffer drop-on-latency=true "
            "! rtph264depay ! h264parse ! avdec_h264 "
            "! queue max-size-buffers=1 leaky=downstream " # Drop frames if we get backed up
            "! videoconvert "
            f"! videoscale ! video/x-raw,format=BGR,width={self.width},height={self.height} "
            "! appsink name=appsink0 emit-signals=true sync=false max-buffers=1 drop=true"
        )

        self.video_pipe = None
        self.video_sink = None

        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, 'camera/image_raw', 10)

        # START
        self.start_gst()
        
        # Reduced timer to 30 FPS (0.033) to match camera, prevents spinning too fast
        self.create_timer(0.033, self.update)

        self.get_logger().info(f"Video Node Started on Port {self.port}")
        self.get_logger().info(f"OUTPUT Resolution: {self.width}x{self.height} (Optimized for Detection)")
        self.get_logger().info("NOTE: Local visualization window disabled to save CPU.")

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
        # Buffer is already resized to 640x360 by GStreamer
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

    def update(self):
        if self._frame is None:
            return

        # Publish the frame
        try:
            img_msg = self.bridge.cv2_to_imgmsg(self._frame, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_frame'
            self.image_publisher.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish image: {e}")

    def destroy_node(self):
        if self.video_pipe:
            self.video_pipe.set_state(Gst.State.NULL)
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
