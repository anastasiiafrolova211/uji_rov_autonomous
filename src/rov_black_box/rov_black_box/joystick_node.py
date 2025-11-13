import os
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from rosbag2_py import serialization

FOLDER_PATH = '.'
TOPIC_NAME = '/bluerov2/camera/image_raw'
OUTPUT_FPS = 30

bridge = CvBridge()

def db3_to_images(db3_file, topic_name):
    storage_options = rosbag2_py.StorageOptions(uri=db3_file, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    topic_types = reader.get_all_topics_and_types()
    topic_type_dict = {t.name: t.type for t in topic_types}
    
    if topic_name not in topic_type_dict:
        print(f"Topic {topic_name} not found in {db3_file}")
        return []
    
    msg_type = get_message(topic_type_dict[topic_name])
    images = []

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == topic_name:
            # Proper deserialization
            msg = serialization.deserialize_message(data, msg_type)
            try:
                cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                images.append(cv_img)
            except Exception as e:
                print(f"Failed to convert image: {e}")
    
    return images

def main():
    for file in os.listdir(FOLDER_PATH):
        if file.endswith('.db3'):
            print(f"Processing {file}...")
            images = db3_to_images(os.path.join(FOLDER_PATH, file), TOPIC_NAME)
            
            if not images:
                print(f"No images found in {file} for topic {TOPIC_NAME}.")
                continue
            
            height, width, _ = images[0].shape
            output_file = os.path.splitext(file)[0] + '.mp4'
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            out = cv2.VideoWriter(output_file, fourcc, OUTPUT_FPS, (width, height))
            
            for img in images:
                out.write(img)
            out.release()
            print(f"Saved video: {output_file}")

if __name__ == '__main__':
    main()

