#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import gi
import numpy as np
import pickle
import os

gi.require_version('Gst', '1.0')
from gi.repository import Gst

class CameraCalibration(Node):
    def __init__(self):
        super().__init__("camera_calibration")
        
        self.declare_parameter("port", 5600)
        self.declare_parameter("checkerboard_rows", 6)
        self.declare_parameter("checkerboard_cols", 9)
        self.declare_parameter("square_size", 0.02)  # Size in meters (2cm)
        
        self.port = self.get_parameter("port").value
        self.rows = self.get_parameter("checkerboard_rows").value
        self.cols = self.get_parameter("checkerboard_cols").value
        self.square_size = self.get_parameter("square_size").value
        
        self._frame = None
        self.video_source = 'udpsrc port={}'.format(self.port)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        self.video_decode = '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        self.video_sink_conf = '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None
        
        # Calibration data
        self.objpoints = []  # 3D points in real world space
        self.imgpoints = []  # 2D points in image plane
        self.frame_count = 0
        self.calibration_frames_needed = 20
        
        # Termination criteria for corner refinement
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
        # Prepare object points (0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)
        self.objp = np.zeros((self.rows * self.cols, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2)
        self.objp *= self.square_size
        
        Gst.init()
        self.run()
        
        # Run calibration loop at ~30 Hz to match camera
        self.create_timer(0.033, self.calibrate)
        
        self.get_logger().info(f"Camera Calibration Node Started")
        self.get_logger().info(f"Need {self.calibration_frames_needed} frames with visible checkerboard")
        self.get_logger().info(f"Checkerboard: {self.cols}x{self.rows}, Square size: {self.square_size}m")
        self.get_logger().info(f"Controls: SPACE=capture, C=calibrate, Q=quit")

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

    def frame_available(self):
        return type(self._frame) != type(None)

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

    def calibrate(self):
        if not self.frame_available():
            return

        frame = self._frame.copy()
        height, width = frame.shape[:2]
        
        # Downsample for faster detection (4x speedup)
        scale_factor = 0.5  # Process at half resolution
        small_frame = cv2.resize(frame, None, fx=scale_factor, fy=scale_factor, 
                                 interpolation=cv2.INTER_LINEAR)
        gray_small = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
        
        # Find checkerboard corners on smaller image (much faster)
        ret, corners = cv2.findChessboardCorners(gray_small, (self.cols, self.rows), None)
        
        display_frame = frame.copy()
        
        if ret:
            # Scale corners back to full resolution
            corners_full = corners * (1.0 / scale_factor)
            
            # Refine corners at full resolution for accuracy
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners2 = cv2.cornerSubPix(gray, corners_full, (11, 11), (-1, -1), self.criteria)
            
            # Draw checkerboard
            cv2.drawChessboardCorners(display_frame, (self.cols, self.rows), corners2, ret)
            
            cv2.putText(display_frame, f"Checkerboard DETECTED ({len(self.objpoints)}/{self.calibration_frames_needed})", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display_frame, "Press SPACE to capture", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        else:
            corners2 = None
            cv2.putText(display_frame, "Checkerboard NOT DETECTED", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(display_frame, "Point camera at checkerboard", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        cv2.putText(display_frame, f"Captured frames: {len(self.objpoints)}/{self.calibration_frames_needed}", 
                   (10, height-30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(display_frame, "C=Calibrate, Q=Quit", (10, height-60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        cv2.imshow("Camera Calibration", display_frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("Quitting...")
            cv2.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()
        elif key == ord(' ') and ret:  # Space to capture
            self.objpoints.append(self.objp)
            self.imgpoints.append(corners2)
            self.get_logger().info(f"Captured frame {len(self.objpoints)}/{self.calibration_frames_needed}")
        elif key == ord('c') and len(self.objpoints) >= 3:  # C to calibrate
            self.perform_calibration((width, height))

    def perform_calibration(self, image_size):
        """Perform camera calibration"""
        self.get_logger().info("Performing calibration...")
        
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.objpoints, 
            self.imgpoints, 
            image_size,
            None, 
            None
        )
        
        if ret:
            # Calculate mean reprojection error for quality check
            mean_error = 0
            for i in range(len(self.objpoints)):
                imgpoints2, _ = cv2.projectPoints(self.objpoints[i], rvecs[i], 
                                                 tvecs[i], camera_matrix, dist_coeffs)
                error = cv2.norm(self.imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
                mean_error += error
            
            mean_error = mean_error / len(self.objpoints)
            
            self.get_logger().info("Calibration successful!")
            self.get_logger().info(f"Mean reprojection error: {mean_error:.4f} pixels")
            
            # Warn if error is too high
            if mean_error > 1.0:
                self.get_logger().warn("Reprojection error > 1 pixel - consider recalibrating!")
            
            # Save calibration
            calibration_data = {
                'camera_matrix': camera_matrix,
                'dist_coeffs': dist_coeffs,
                'reprojection_error': mean_error
            }
            
            # Save to file
            calib_file = os.path.expanduser('~/uji_rov_autonomous/camera_calibration.pkl')
            os.makedirs(os.path.dirname(calib_file), exist_ok=True)
            with open(calib_file, 'wb') as f:
                pickle.dump(calibration_data, f)
            
            self.get_logger().info(f"Calibration saved to {calib_file}")
            
            # Print values
            print("\n" + "="*60)
            print("CAMERA CALIBRATION RESULTS")
            print("="*60)
            print(f"\nMean Reprojection Error: {mean_error:.4f} pixels")
            print("\nCamera Matrix:")
            print(camera_matrix)
            print("\nDistortion Coefficients:")
            print(dist_coeffs.flatten())
            print("\nPython code for your video_node.py:")
            print("="*60)
            print("self.camera_matrix = np.array([")
            print(f"    [{camera_matrix[0,0]:.1f}, {camera_matrix[0,1]:.1f}, {camera_matrix[0,2]:.1f}],")
            print(f"    [{camera_matrix[1,0]:.1f}, {camera_matrix[1,1]:.1f}, {camera_matrix[1,2]:.1f}],")
            print(f"    [{camera_matrix[2,0]:.1f}, {camera_matrix[2,1]:.1f}, {camera_matrix[2,2]:.1f}]")
            print("], dtype=np.float32)")
            print("\nself.dist_coeffs = np.array([", end="")
            dist_list = []
            for coeff in dist_coeffs.flatten():
                dist_list.append(f"{coeff:.6f}")
            print(", ".join(dist_list), end="")
            print("], dtype=np.float32).reshape(5, 1)")
            print("="*60 + "\n")
            
            cv2.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()
        else:
            self.get_logger().error("Calibration failed!")

    def destroy_node(self):
        """Cleanup"""
        if self.video_pipe:
            self.video_pipe.set_state(Gst.State.NULL)
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibration()
    
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

