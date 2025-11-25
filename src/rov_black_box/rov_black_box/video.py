import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        
        # --- CONFIGURATION ---
        # ⚠️ UPDATE THIS PATH to your video file location ⚠️
        self.video_path = '/home/elex/rosbags/19_11/rosbag2_2025_11_19-12_45_09/output.mp4' 
        self.camera_topic = '/camera/image_raw' 
        # ---------------------

        # Open the video file
        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"FATAL: Could not open video file at {self.video_path}")
            # Exit the program if the video fails to open
            time.sleep(1) # Wait for log to print
            rclpy.shutdown() 
            return

        # Get video properties to set the timer interval
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        # Timer period: 1.0 / FPS. Default to ~30 FPS (0.033s) if FPS is 0 or unreadable.
        self.timer_period = 1.0 / self.fps if self.fps > 0 else 0.033  

        # Initialize ROS tools
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, self.camera_topic, 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.get_logger().info(f"✅ Publishing video frames from {self.video_path} at {self.fps:.2f} FPS...")

    def timer_callback(self):
        # 1. Read a frame from the video
        success, frame = self.cap.read()
        
        if success:
            # 2. Convert frame to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            
            # 3. Publish to the topic
            self.image_pub.publish(ros_image)
        else:
            # Video end reached or failure
            self.get_logger().info("Video finished. Looping to start...")
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0) # Loop back to start
            
def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Release the video object and destroy the node upon exit
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
