import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Configuration
bag_path = 'box_bag'  # Path to folder containing ROS 2 bag
image_topic = '/camera/image'  # Update this to your actual topic
output_video = 'output_video.mp4'
fps = 30  # Desired frames per second
max_frames = None  # Set to an integer to limit frames

def main():
    rclpy.init()
    bridge = CvBridge()

    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Prepare video writer
    writer = None
    frame_count = 0

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == image_topic:
            msg = deserialize_message(data, Image)
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if writer is None:
                height, width, _ = cv_image.shape
                writer = cv2.VideoWriter(output_video, cv2.VideoWriter_fourcc(*'XVID'), fps, (width, height))

            writer.write(cv_image)
            frame_count += 1
            print(f'Wrote frame {frame_count}')

            if max_frames and frame_count >= max_frames:
                break

    if writer:
        writer.release()
        print(f'Video saved to {output_video}')
    else:
        print('No frames were written.')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
