import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import Image
import numpy as np
import cv2
import os
from cv_bridge import CvBridge

# camera parameters - calibrated to our Bellatrix/Barbi/Bibble
u0 = 409.7
v0 = 268.7
lx = 576.6
ly = 576.6

kud =0.00683 
kdu = -0.01424     

# bunch of thresholds
threshold = 0.4  # confidence
ratio_threshold = 0.75 # Lowe’s ratio test
min_good_matches = 10 # minimum matches before homography - optimal is 8-10 but can be 6-12
ransac_threshold = 4.0 # reprojection threshold in pixels - whatever it is 

# color filtering can be an option if i take 'mean' color across whole reference
target_color_lower = np.array([60, 50, 150])
target_color_upper = np.array([160, 160, 180])
use_color_filter = False 
# this shit doesnt help a lot lets say so False for now

# grouping in order for sides template checking
template_groups = {
    "side_logo_view": ["side_logo_view", "side_stripe1_view", "side_stripe2_view", "top_view"],
    "side_aruco_view": ["side_aruco_view", "side_stripe1_view", "side_stripe2_view", "top_view"],
    "side_stripe1_view": ["side_stripe1_view", "side_logo_view", "side_aruco_view", "top_view"],
    "side_stripe2_view": ["side_stripe2_view", "side_logo_view", "side_aruco_view", "top_view"],
    "top_view": ["top_view", "side_logo_view", "side_stripe1_view", "side_stripe2_view", "side_aruco_view"]
}

    
# convert a pixel coordinate to meters given linear calibration parameters
def convert2meter(pt,u0,v0,lx,ly):
    return (float(pt[0])-u0)/lx, (float(pt[1])-v0)/ly

# convert a pixel coordinate to meters using defaut calibration parameters
def convertOnePoint2meter(pt):
    global u0,v0,lx, ly
    return (float(pt[0])-u0)/lx, (float(pt[1])-v0)/ly

# convert a list of pixels coordinates to meters using defaut calibration parameters
def convertListPoint2meter (points):
    global u0,v0,lx, ly
    
    if(np.shape(points)[0] > 1):
        n = int(np.shape(points)[0]/2)
        point_reshaped = (np.array(points).reshape(n,2))
        point_meter = []
        for pt in point_reshaped:
            pt_meter = convert2meter(pt,u0,v0,lx,ly)
            point_meter.append(pt_meter)
        point_meter = np.array(point_meter).reshape(-1)
        return point_meter

def overlay_points(image, pt, r, g, b, text="", scale=1, offsetx=5, offsety=5):
    cv2.circle(image, (int(pt[0]), int(pt[1])), int(4*scale+1), (b, g, r), -1)
    position = (int(pt[0]) + offsetx, int(pt[1]) + offsety)
    cv2.putText(image, text, position, cv2.FONT_HERSHEY_SIMPLEX, scale, (b, g, r, 255), 1)




# ------------- i think i will delete later
def mean_color(template_bgr):
    return np.mean(template_bgr.reshape(-1, 3), axis=0)

def color_in_range(color_bgr, lower, upper):
    return np.all(color_bgr >= lower) and np.all(color_bgr <= upper)
# ------------------------------------

# to track the 'consistency' of detection = avoid jumping
def polygon_area(pts):
    x = pts[:, 0, 0]
    y = pts[:, 0, 1]
    return 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))





class TrackBoxNode(Node):
    def __init__(self):
        super().__init__('track_box_node')
        
        self.pub_tracked_point = self.create_publisher(Float64MultiArray, '/tracked_point', 10)
        self.pub_tracked_area = self.create_publisher(Float64MultiArray, '/tracked_area', 10)

        self.subscription = self.create_subscription(
            Image,                    # Message type
            'camera/image',           # Topic (assumed topic name)
            self.cameracallback,      # Callback function
            1                         # Queue size (adjust if necessary)
        )
        self.subscription  # prevent unused variable warning

        self.bridge = CvBridge()  # CvBridge for converting ROS images to OpenCV format


        # SIFT + Matcher
        self.feature_detector = cv2.SIFT_create()
        self.matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)

        # Load templates once
        self.template_dir = "box_templates"
        # here should be a full folder path depends on the laptop it is used on
        self.templates = self.load_templates()

        # Last state memory
        self.last_template_name = list(self.templates.keys())[0] if self.templates else None
        self.last_state = {"area": None, "center": None, "score": 0}


        self.get_logger().info(f"Box tracking node started. {len(self.templates)} templates loaded")


    def load_templates(self):
        templates = {}
        for fname in os.listdir(self.template_dir):
            path = os.path.join(self.template_dir, fname)

            name = os.path.splitext(fname)[0]
            img_bgr = cv2.imread(path)

            gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
            kp, des = self.feature_detector.detectAndCompute(gray, None)

            templates[name] = {
                "gray": gray,
                "kp": kp,
                "des": des,
                "mean_color": mean_color(img_bgr)
            }
            #  might get rid of mean_color

        return templates
    

    def cameracallback(self, image_data):
        # Initial try - feature extraction
        
        # Get image data
        frame = self.bridge.imgmsg_to_cv2(image_data, "bgr8")

        frame, self.last_template_name, self.last_state = self.process_frame(
            frame, self.templates, self.last_template_name, self.last_state
        )
        cv2.imshow("Black box tracking", cv2.resize(frame, (960, 540)))
        
        cv2.waitKey(2)


    def process_frame(self, frame, templates, last_template_name, last_state):
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        kp_frame, des_frame = self.feature_detector.detectAndCompute(gray_frame, None)
        
        if des_frame is None or kp_frame is None:
            return frame, last_template_name, last_state

        # --- Adaptive template prioritization --- to switch between sides
        decay_factor = 0.95
        last_score = last_state.get("score", 0)
        if last_score > threshold * 0.9 and last_template_name in template_groups:
            # keep searching locally if last detection is strong
            group = template_groups[last_template_name]
            template_order = [last_template_name] + [t for t in group if t != last_template_name]
        else:
            # explore all if weak or lost
            template_order = templates.keys()
        # --------

        best_score = 0
        best_result = None
        best_name = last_template_name

        for name in template_order:
            t = templates[name]
            template_gray, kp_template, des_template, mean_col = (
                t["gray"], t["kp"], t["des"], t["mean_color"]
            )

            if (use_color_filter == True and not color_in_range(mean_col, target_color_lower, target_color_upper)):
                continue

            knn_matches = self.matcher.knnMatch(des_template, des_frame, k=2)
            good_matches = []
            for m, n in knn_matches:
                if m.distance < ratio_threshold * n.distance:
                    good_matches.append(m)

            if len(good_matches) < min_good_matches:
                continue

            src_pts = np.float32([kp_template[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp_frame[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

            H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, ransac_threshold)
            if H is None:
                continue

            matches_mask = mask.ravel().tolist()
            inliers = sum(matches_mask)
            ratio_inliers = inliers / len(good_matches)
            score = 0.7 * ratio_inliers + 0.3 * min(len(good_matches) / 100.0, 1.0)

            if score > best_score:
                best_score = score
                best_result = (H, good_matches, name, kp_template, kp_frame)
                best_name = name


        if best_result and best_score > threshold:
            H, good_matches, name, kp_t, kp_frame = best_result
            h, w = templates[name]["gray"].shape
            corners = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
            projected = cv2.perspectiveTransform(corners, H)
            area = polygon_area(projected)
            cx, cy = np.mean(projected[:, 0, 0]), np.mean(projected[:, 0, 1])

            # spatial consistency filter - to avoid jumping across the camera frame
            if last_state["area"] is not None:
                area_change = abs(area - last_state["area"]) / last_state["area"]
                dist = np.linalg.norm(np.array([cx, cy]) - np.array(last_state["center"]))
                if area_change > 0.7 or dist > 500: # 70% and center jumps in pixels
                    return frame, last_template_name, last_state

            frame = cv2.polylines(frame, [np.int32(projected)], True, (0, 255, 0), 3)
            overlay_points(frame, [cx, cy], 0, 0, 255, f"{name} | score={best_score:.2f}", scale=0.6)

            # Convert to meters + publish later

            # cx_m, cy_m = convertOnePoint2meter((cx, cy))
            # msg_point = Float64MultiArray()
            # msg_point.data = [cx_m, cy_m]
            # msg_area = Float64MultiArray()
            # msg_area.data = [float(area)]
            # self.pub_tracked_point.publish(msg_point)
            # self.pub_tracked_area.publish(msg_area)

            last_state.update({"area": area, "center": np.array([cx, cy]), "score": best_score})
            last_template_name = name
        else:
            last_state["score"] *= decay_factor
            if last_state["score"] < threshold * 0.5:
                last_template_name = None

        return frame, last_template_name, last_state
    


def main(args=None):
    rclpy.init(args=args)
    node = TrackBoxNode()

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
