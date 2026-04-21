#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs  # registers PointStamped transform
import cv2
import numpy as np

# Try to import YOLO
try:
    from ultralytics import YOLO
except ImportError:
    print("Please install ultralytics: pip3 install ultralytics")
    YOLO = None

# Import our SORT implementation from the same directory
try:
    from sort import Sort
except ImportError as e:
    print(f"Failed to import sort.py: {e}")
    Sort = None

def iou(bb_test, bb_gt):
    """
    Computes IOU between two bounding boxes [x1, y1, x2, y2]
    """
    xx1 = np.maximum(bb_test[0], bb_gt[0])
    yy1 = np.maximum(bb_test[1], bb_gt[1])
    xx2 = np.minimum(bb_test[2], bb_gt[2])
    yy2 = np.minimum(bb_test[3], bb_gt[3])
    w = np.maximum(0., xx2 - xx1)
    h = np.maximum(0., yy2 - yy1)
    wh = w * h
    o = wh / ((bb_test[2] - bb_test[0]) * (bb_test[3] - bb_test[1])
        + (bb_gt[2] - bb_gt[0]) * (bb_gt[3] - bb_gt[1]) - wh)
    return o

class YoloSortTracker(Node):
    def __init__(self):
        super().__init__('yolo_sort_tracker')
        
        self.declare_parameter('camera_topic', '/camera/image_raw')
        camera_topic = self.get_parameter('camera_topic').value
        
        self.bridge = CvBridge()
        
        # ── TF2 Listener (for pixel_to_map) ──────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # ── Camera intrinsics (must match URDF sensor definition) ────────
        #    URDF: 640x480, horizontal_fov = 1.089 rad
        self.img_w = 640
        self.img_h = 480
        hfov = 1.089  # radians
        self.fx = self.img_w / (2.0 * np.tan(hfov / 2.0))  # ~548 px
        self.fy = self.fx  # square pixels
        self.cx = self.img_w / 2.0   # 320
        self.cy = self.img_h / 2.0   # 240
        
        # Initialize YOLO model
        if YOLO is not None:
            self.get_logger().info("Loading YOLO26n model...")
            self.model = YOLO('yolo26n.pt')
        else:
            self.get_logger().error("Ultralytics YOLO not found!")
            self.model = None
            
        # Initialize SORT tracker
        if Sort is not None:
            self.mot_tracker = Sort(max_age=30, min_hits=3, iou_threshold=0.3)
        else:
            self.mot_tracker = None
            
        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        self.get_logger().info(f"Subscribed to {camera_topic}. Waiting for images...")
        self.frame_count = 0

    # ─────────────────────────────────────────────────────────────────────
    #  pixel_to_map:  (u, v, depth)  →  (map_x, map_y)
    # ─────────────────────────────────────────────────────────────────────
    def pixel_to_map(self, u: float, v: float, depth: float,
                     camera_frame: str = 'camera_depth_optical_frame',
                     target_frame: str = 'map') -> tuple:
        """
        Convert a pixel coordinate (u, v) + depth value into (x, y)
        in the global 'map' frame.

        Steps:
          1. Back-project (u, v, depth) → 3-D point in the camera
             optical frame using pinhole camera intrinsics.
          2. Pack the 3-D point into a PointStamped message.
          3. Use tf2_ros to transform the point from the camera
             optical frame into the 'map' frame.

        Args:
            u     : horizontal pixel coordinate
            v     : vertical pixel coordinate
            depth : depth in metres at (u, v)
            camera_frame : TF frame of the camera optical link
            target_frame : TF frame to transform into (default 'map')

        Returns:
            (map_x, map_y) as floats, or None on failure.
        """
        # 1. Back-project pixel → 3-D point in camera optical frame
        #    In the optical frame convention: Z forward, X right, Y down
        x_cam = (u - self.cx) * depth / self.fx
        y_cam = (v - self.cy) * depth / self.fy
        z_cam = depth

        # 2. Create a PointStamped in the camera optical frame
        point_cam = PointStamped()
        point_cam.header.stamp = self.get_clock().now().to_msg()
        point_cam.header.frame_id = camera_frame
        point_cam.point.x = float(z_cam)   # optical Z → ROS X (forward)
        point_cam.point.y = float(-x_cam)  # optical -X → ROS Y (left)
        point_cam.point.z = float(-y_cam)  # optical -Y → ROS Z (up)

        # 3. Transform into the target frame via tf2
        try:
            point_map = self.tf_buffer.transform(
                point_cam, target_frame,
                timeout=Duration(seconds=0.5)
            )
            return (point_map.point.x, point_map.point.y)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return None

    def image_callback(self, msg):
        if self.model is None or self.mot_tracker is None:
            return
            
        self.frame_count += 1
        
        # Convert ROS Image to OpenCV Frame
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
            
        # 1. Run YOLO Object Detection
        results = self.model(cv_image, verbose=False)[0]
        
        # 2. Extract bounding boxes and scores for SORT
        # SORT expects: [[x1, y1, x2, y2, score], [x1, y1, x2, y2, score], ...]
        detections = []
        class_names = results.names
        
        yolo_boxes = [] # Keep track of original boxes to map classes back
        
        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            score = box.conf[0].cpu().numpy()
            cls_id = int(box.cls[0].cpu().numpy())
            
            detections.append([x1, y1, x2, y2, score])
            yolo_boxes.append({
                'box': [x1, y1, x2, y2],
                'class_name': class_names[cls_id]
            })
            
        dets_array = np.array(detections) if len(detections) > 0 else np.empty((0, 5))
        
        # 3. Update SORT Tracker
        tracked_objects = self.mot_tracker.update(dets_array)
        
        # 4. Map tracked IDs back to classes and Log the output
        if len(tracked_objects) > 0:
            self.get_logger().info(f"--- Frame {self.frame_count} ---")
            
            for track in tracked_objects:
                # SORT output is [x1, y1, x2, y2, obj_id]
                x1, y1, x2, y2, obj_id = track
                obj_id = int(obj_id)
                
                # Find the YOLO class that best matches this tracked box via IOU
                best_class = "Object"
                best_iou = 0
                for yb in yolo_boxes:
                    overlap = iou([x1, y1, x2, y2], yb['box'])
                    if overlap > best_iou and overlap > 0.1:
                        best_iou = overlap
                        best_class = yb['class_name']
                        
                # Log string output explicitly requested by user
                log_str = f"Detected: {best_class}_{obj_id} at [{int(x1)}, {int(y1)}, {int(x2)}, {int(y2)}]"
                self.get_logger().info(log_str)
                
                # Draw the tracked bounding box and label
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(cv_image, f"{best_class}_{obj_id}", (int(x1), int(y1) - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                            
        # Optionally display the image using CV2
        cv2.imshow("YOLO + SORT Tracking", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloSortTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
