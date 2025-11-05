#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import gi
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

gi.require_version('Gst', '1.0')
from gi.repository import Gst


class BlueROVCameraNode(Node):
    """Simple GStreamer video capture node for BlueROV camera"""

    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('port', 5600)
        self.port = self.get_parameter('port').get_parameter_value().integer_value

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)

        Gst.init()
        self.get_logger().info(f"Starting video stream from UDP port {self.port}...")

        self.pipeline = Gst.parse_launch(
            f"udpsrc port={self.port} ! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! appsink emit-signals=true sync=false"
        )
        self.appsink = self.pipeline.get_by_name('appsink0')
        if not self.appsink:
            # Fix naming mismatch for appsink element
            self.appsink = [el for el in self.pipeline.iterate_elements()][-1]
        self.appsink.connect('new-sample', self.on_new_sample)
        self.pipeline.set_state(Gst.State.PLAYING)

        self.get_logger().info("Camera node started and publishing /camera/image_raw")

    def on_new_sample(self, sink):
        try:
            sample = sink.emit('pull-sample')
            buf = sample.get_buffer()
            caps = sample.get_caps()
            arr = np.ndarray(
                (caps.get_structure(0).get_value('height'),
                 caps.get_structure(0).get_value('width'),
                 3),
                buffer=buf.extract_dup(0, buf.get_size()),
                dtype=np.uint8
            )

            img_msg = self.bridge.cv2_to_imgmsg(arr, encoding='bgr8')
            self.image_pub.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f"Error in video stream: {e}")
        return Gst.FlowReturn.OK


def main(args=None):
    rclpy.init(args=args)
    node = BlueROVCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

