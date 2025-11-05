#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import gi
import numpy as np

gi.require_version('Gst', '1.0')
from gi.repository import Gst

class ArUcoDetector(Node):
    def __init__(self):
        super().__init__("aruco_detector")
        self.get_logger().info("ArUco detector with size measurement started")

        self.declare_parameter("port", 5600)
        self.port = self.get_parameter("port").value
        
        self._frame = None
        self.video_source = 'udpsrc port={}'.format(self.port)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        self.video_decode = '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        self.video_sink_conf = '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        # All ArUco dictionaries to try
        self.dictionaries = [
            "DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250", "DICT_4X4_1000",
            "DICT_5X5_50", "DICT_5X5_100", "DICT_5X5_250", "DICT_5X5_1000",
            "DICT_6X6_50", "DICT_6X6_100", "DICT_6X6_250", "DICT_6X6_1000",
            "DICT_7X7_50", "DICT_7X7_100", "DICT_7X7_250", "DICT_7X7_1000",
            "DICT_ARUCO_ORIGINAL", "DICT_APRILTAG_16h5", "DICT_APRILTAG_25h9",
            "DICT_APRILTAG_36h10", "DICT_APRILTAG_36h11"
        ]

        Gst.init()
        self.run()
        
        # Timer for real-time detection
        self.create_timer(0.1, self.detect_markers)

    def start_gst(self, config=None):
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

    def run(self):
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

    def frame_available(self):
        return type(self._frame) != type(None)

    def measure_marker_size_pixels(self, corners):
        """
        Measure marker size in pixels (average of all 4 sides)
        """
        corner_points = corners[0]
        
        # Calculate distances between consecutive corners
        distances = []
        for i in range(4):
            p1 = corner_points[i]
            p2 = corner_points[(i + 1) % 4]
            dist = np.linalg.norm(p2 - p1)
            distances.append(dist)
        
        avg_size = np.mean(distances)
        return avg_size

    def detect_markers(self):
        if not self.frame_available():
            return

        frame = self._frame.copy()
        
        # Resize for display
        width = int(1920/2)
        height = int(1080/2)
        dim = (width, height)
        display_frame = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)

        detected = False
        detected_dict = None
        all_corners = None
        all_ids = None

        # Try all dictionaries
        for dict_name in self.dictionaries:
            try:
                aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
                parameters = cv2.aruco.DetectorParameters()
                corners, ids, _ = cv2.aruco.detectMarkers(display_frame, aruco_dict, parameters=parameters)

                if ids is not None and len(ids) > 0:
                    detected = True
                    detected_dict = dict_name
                    all_corners = corners
                    all_ids = ids
                    break
            except AttributeError:
                continue

        if detected:
            # Draw detected markers
            cv2.aruco.drawDetectedMarkers(display_frame, all_corners, all_ids)
            
            # Display information for each marker
            self.get_logger().info(f"Dictionary: {detected_dict}")
            
            for i, (corner, marker_id) in enumerate(zip(all_corners, all_ids)):
                marker_id = marker_id[0]
                
                # Measure size in pixels
                size_pixels = self.measure_marker_size_pixels(corner)
                
                # Get center of marker
                center = corner[0].mean(axis=0).astype(int)
                
                # Draw info on frame
                cv2.putText(display_frame, f"ID: {marker_id}", 
                           (center[0] - 40, center[1] - 40),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(display_frame, f"Size: {size_pixels:.1f}px", 
                           (center[0] - 40, center[1] - 15),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                
                # Log the information
                self.get_logger().info(
                    f"Marker ID {marker_id}: {size_pixels:.1f} pixels"
                )
            
            # Display dictionary name
            cv2.putText(display_frame, f"Dict: {detected_dict}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(display_frame, "Press 'S' to save measurements", (10, height - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        else:
            cv2.putText(display_frame, "No ArUco markers detected", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("ArUco Detection & Measurement", display_frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("Quitting...")
            self.destroy_node()
            rclpy.shutdown()
        elif key == ord('s') and detected:
            # Save frame with measurements
            timestamp = self.get_clock().now().to_msg()
            filename = f"aruco_measurement_{timestamp.sec}.png"
            cv2.imwrite(filename, display_frame)
            self.get_logger().info(f"Saved measurement to {filename}")
            
            # Print measurement summary
            self.get_logger().info("\n===== MEASUREMENT SUMMARY =====")
            self.get_logger().info(f"Dictionary: {detected_dict}")
            for i, (corner, marker_id) in enumerate(zip(all_corners, all_ids)):
                size_px = self.measure_marker_size_pixels(corner)
                self.get_logger().info(f"Marker {marker_id[0]}: {size_px:.1f} pixels")
            self.get_logger().info("===============================\n")


def main(args=None):
    rclpy.init(args=args)
    node = ArUcoDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

