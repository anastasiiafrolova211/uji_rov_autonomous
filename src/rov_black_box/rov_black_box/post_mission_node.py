import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import csv
import os

class DataProcessing(Node):
    def __init__(self):
        super().__init__('data_processing_node')
        self.visited_markers = set()
        self.cmd_vel = None
        self.depth = None
        self.log_file = 'mission_log.csv'
        self.start_time = self.get_clock().now()

        self.subscriber()

        if not os.path.exists(self.log_file):
            with open(self.log_file, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(['Time', 'VisitedMarkers', 'CmdVelLinearX', 'CmdVelAngularZ', 'Depth'])

        self.timer = self.create_timer(1.0, self.log_data)  # log at 1 Hz

    def aruco_callback(self, msg):
        self.visited_markers.add(msg.data)

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg

    def depth_callback(self, msg):
        self.depth = msg.data

    def log_data(self):
        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        linear_x = self.cmd_vel.linear.x if self.cmd_vel else 0.0
        angular_z = self.cmd_vel.angular.z if self.cmd_vel else 0.0

        depth = self.depth if self.depth else 0.0
        
        visited_str = ';'.join(str(m) for m in sorted(self.visited_markers))

        with open(self.log_file, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([current_time, visited_str, linear_x, angular_z, depth])
        self.get_logger().info(f'Logged data at time {current_time:.2f}s')

def analyze_log(filename='mission_log.csv'):
    visited_all = set()
    times = []
    if not os.path.exists(filename):
        print(f"Log file {filename} does not exist.")
        return
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            markers = row['VisitedMarkers'].split(';') if row['VisitedMarkers'] else []
            visited_all.update(int(m) for m in markers if m)
            times.append(float(row['Time']))
    duration = times[-1] if times else 0.0
    print(f"Mission Duration: {duration:.2f} seconds")
    print(f"Total unique visited Aruco markers: {len(visited_all)}")
    print(f"Visited marker IDs: {visited_all}")


def subscriber(self):
    qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

    self.aruco = self.create_subscription(Int32, 'aruco_id', self.aruco_callback, qos_profile=qos_profile)
    self.aruco
    self.cmd = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, qos_profile=qos_profile)
    self.cmd
    self.depth = self.create_subscription(Float32, 'depth', self.depth_callback, qos_profile=qos_profile)
    self.depth
    self.imu = self.create_subscription(Imu, "imu/data", self.odo_callback, qos_profile=qos_profile)


def main(args=None):
    import sys
    if args and 'analyze' in args:
        analyze_log()
    else:
        rclpy.init(args=args)
        data_processing = DataProcessing()
        rclpy.spin(data_processing)
        data_processing.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(sys.argv)
