import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist
import csv
import os

# even if teleoperating ? still do the cognitive map thingy ?
 
class DataProcessing(Node):
    def __init__(self):
        super().__init__('post_mission_node')
        self.visited_markers = set()
        self.current_landmark = None
        self.previous_landmark = None
        self.weight = 0
        self.cmd_vel = None
        self.depth = None
        self.detected_target = None
        self.confidence = 0.0

        # Landmarks dictionary from report for cognitive class
        # on other way how are we even gonna distinguish then a? 
        # TODO use here ArUco only !
        self.landmarks_dict = {
            1: 'Pool Edge 1', 2: 'Pool Edge 2', 3: 'Pool Edge 3', 4: 'Pool Edge 4',
            5: 'Pool Edge 5', 6: 'Pool Edge 6', 7: 'Pool Edge 7', 8: 'Pool Edge 8',
            9: 'Corner 1', 10: 'Corner 2', 11: 'Corner 3', 12: 'Corner 4',
            13: 'Corner 5', 14: 'Corner 6', 15: 'Corner 7', 16: 'Corner 8',
            17: 'Aruco 1', 18: 'Aruco 2', 19: 'Aruco 3', 20: 'Aruco 4',
            21: 'Aruco 5', 22: 'Aruco 6', 23: 'Aruco 7', 24: 'Aruco 8',
            25: 'Aruco 9', 26: 'Window', 27: 'Black Box'
        }

        self.log_file = 'mission_log.csv'
        self.start_time = self.get_clock().now()

        self.subscriber()

        if not os.path.exists(self.log_file):
            with open(self.log_file, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(['Time (in sec)', 
                                 'CurrentPosition', 'NextPosition', 'Weight', 
                                 'CmdVelLinearX', 'CmdVelLinearZ', 'CmdVelAngularZ', 
                                 'Depth', 'DetectedTarget', 'Confidence', 
                                 'IsPathChosen'])

        self.timer = self.create_timer(1.0, self.log_data) # log every 1.0 - better safe than sorry

        self.is_path_chosen = False

    def subscriber(self):
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        
        # TODO cnahge all topic names here
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, qos_profile=qos_profile)
        self.depth_sub = self.create_subscription(Float32, 'global_position/rel_alt', self.depth_callback, qos_profile=qos_profile)
        
        # from underwater_detection node 
        self.detected_target_sub = self.create_subscription(Int32, 'detected_target', self.detected_target_callback, qos_profile=qos_profile)
        self.confidence_sub = self.create_subscription(Float32, 'confidence', self.confidence_callback, qos_profile=qos_profile)
        self.aruco_sub = self.create_subscription(Int32, 'aruco_id', self.aruco_callback, qos_profile=qos_profile)


    def aruco_callback(self, msg):
        self.previous_landmark = self.current_landmark
        self.current_landmark = msg.data
        if self.previous_landmark is not None:
            self.weight = self.calculate_weight(self.previous_landmark, self.current_landmark)
        else:
            self.weight = 0
        self.visited_markers.add(self.current_landmark)

# TODO
    def calculate_weight(self, from_landmark, to_landmark):
        # will use graph theory weight calculation Dijkstra algo (?)
        # for now just fixed
        return 1

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg

    def depth_callback(self, msg):
        self.depth = msg.data

    def detected_target_callback(self, msg):
        self.detected_target = msg.data

    def confidence_callback(self, msg):
        self.confidence = msg.data

    def log_data(self):
        current_time = round( (self.get_clock().now() - self.start_time).nanoseconds / 1e9, 2)

        current_position = self.landmarks_dict.get(self.previous_landmark, 'Unknown') if self.previous_landmark else '-'
        next_position = self.landmarks_dict.get(self.current_landmark, 'Unknown') if self.current_landmark else '-'

        linear_x = self.cmd_vel.linear.x if self.cmd_vel else 0.0
        linear_z = self.cmd_vel.linear.z if self.cmd_vel else 0.0
        angular_z = self.cmd_vel.angular.z if self.cmd_vel else 0.0
        depth = self.depth if self.depth else 0.0
        
        detected_target = self.detected_target if self.detected_target is not None else '-'
        confidence = self.confidence if self.confidence else 0.0

        is_path_chosen = self.is_path_chosen # still need to modify

        with open(self.log_file, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([current_time, current_position, next_position, self.weight,
                             linear_x, linear_z, angular_z, depth,
                             detected_target, confidence, is_path_chosen])


# TODO full analysis after the mission
# def analyze_log(filename='mission_log.csv'):
#     visited_all = set()
#     times = []
#     if not os.path.exists(filename):
#         print(f"Log file {filename} does not exist.")
#         return
#     with open(filename, 'r') as f:
#         reader = csv.DictReader(f)
#         for row in reader:
#             markers = row['VisitedAruco'].split(';') if row['VisitedAruco'] else []
#             visited_all.update(marker.strip() for marker in markers if marker)
#             times.append(float(row['Time']))
#     duration = times[-1] if times else 0.0
#     print(f"Mission Duration: {duration:.2f} seconds")
#     print(f"Total unique visited Arucos: {len(visited_all)}")
#     print(f"Visited Aruco: {visited_all}")


def main(args=None):
    rclpy.init(args=args)
    node = DataProcessing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
