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
    """BlueRov video capture class constructor"""

    def __init__(self):
        super().__init__("video_node")

        self.declare_parameter("port", 5600) 

        self.port               = self.get_parameter("port").value
        self._frame             = None
        self.video_source       = 'udpsrc port={}'.format(self.port)
        self.video_codec        = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        self.video_decode       = '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        self.video_sink_conf    = '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe         = None
        self.video_sink         = None
        self.font               = cv2.FONT_HERSHEY_PLAIN

        Gst.init() 

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Create publisher for the image (namespace-relative)
        self.image_publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        # Will become /bluerov2/camera/image_raw with namespace

        self.run()

        # Start update loop at 30 Hz
        self.create_timer(0.033, self.update)

        self.get_logger().info(f"Video node started on port {self.port}")
        self.get_logger().info("Publishing to camera/image_raw")

    def start_gst(self, config=None):
        """Start gstreamer pipeline and sink"""
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
        """Transform byte array into np array"""
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
        """Get Frame"""
        return self._frame

    def frame_available(self):
        """Check if frame is available"""
        return self._frame is not None

    def run(self):
        """Get frame to update _frame"""
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
    
    def update(self):        
        if not self.frame_available():
            return

        frame = self.frame()
        width = int(1920/2)
        height = int(1080/2)
        dim = (width, height)
        img = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)   

        # Publish image with timestamp
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_frame'
        
        self.image_publisher.publish(img_msg)

        # Show window
        cv2.imshow('BlueROV2 Camera', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Quit requested")
            self.destroy_node()
            rclpy.shutdown()

    def destroy_node(self):
        """Cleanup"""
        if self.video_pipe:
            self.video_pipe.set_state(Gst.State.NULL)
        cv2.destroyAllWindows()
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

