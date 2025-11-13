#!/usr/bin/env python3
import os
import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def rosbag_to_video(bag_path, output_file, fps=30):
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', 'cdr')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    bridge = CvBridge()
    video_writer = None
    frame_count = 0
    topic_name = None

    # Try to find the first Image topic
    try:
        topics_and_types = reader.get_all_topics_and_types()
        image_topics = [t.name for t in topics_and_types if "sensor_msgs/msg/Image" in t.type]
        if not image_topics:
            print(f"No Image topics found in {bag_path}")
            return
        topic_name = image_topics[0]
        print(f"🎥 Found image topic: {topic_name}")
    except Exception:
        print(f"Could not read topics for {bag_path}")
        return

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic != topic_name:
            continue

        try:
            img_msg = deserialize_message(data, Image)
            cv_img = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as e:
            print(f"Failed to read image: {e}")
            continue

        if video_writer is None:
            height, width = cv_img.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            video_writer = cv2.VideoWriter(output_file, fourcc, float(fps), (width, height))
            if not video_writer.isOpened():
                print(f"ERROR: Cannot open writer for {output_file}")
                return

        if len(cv_img.shape) == 2:
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
        video_writer.write(cv_img)
        frame_count += 1

    if video_writer:
        video_writer.release()
        print(f"Saved {frame_count} frames → {output_file}")
    else:
        print(f"No frames found for topic {topic_name}")

def convert_all_bags(base_dir, fps=30):
    print(f"Searching for .db3 files in {base_dir}...")
    for root, _, files in os.walk(base_dir):
        for f in files:
            if f.endswith(".db3"):
                bag_path = os.path.join(root, f)
                output_file = os.path.join(root, "output.mp4")
                print(f"Converting: {bag_path}")
                rosbag_to_video(bag_path, output_file, fps=fps)

if __name__ == "__main__":
    rclpy.init()
    base_dir = os.path.expanduser("~/uji_rov_autonomous")
    convert_all_bags(base_dir, fps=30)
    rclpy.shutdown()

