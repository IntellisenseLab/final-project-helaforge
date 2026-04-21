"""
YoloSortTracker – standalone YOLO + SORT tracking node with TF pixel-to-map.
"""
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

try:
    from ultralytics import YOLO
except ImportError:
    print("Please install ultralytics: pip3 install ultralytics")
    YOLO = None

try:
    from diff_drive_robot.sort import Sort
except ImportError as e:
    print(f"Failed to import sort: {e}")
    Sort = None

def iou(bb_test, bb_gt):
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

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.img_w = 640
        self.img_h = 480
        hfov = 1.089
        self.fx = self.img_w / (2.0 * np.tan(hfov / 2.0))
        self.fy = self.fx
        self.cx = self.img_w / 2.0
        self.cy = self.img_h / 2.0

        if YOLO is not None:
            self.get_logger().info("Loading YOLO26n model...")
            self.model = YOLO('yolo26n.pt')
        else:
            self.get_logger().error("Ultralytics YOLO not found!")
            self.model = None

        if Sort is not None:
            self.mot_tracker = Sort(max_age=30, min_hits=3, iou_threshold=0.3)
        else:
            self.mot_tracker = None

        self.subscription = self.create_subscription(
            Image, camera_topic, self.image_callback, 10)
        self.get_logger().info(f"Subscribed to {camera_topic}. Waiting for images...")
        self.frame_count = 0

    def pixel_to_map(self, u, v, depth,
                     camera_frame='camera_depth_optical_frame',
                     target_frame='map'):
        x_cam = (u - self.cx) * depth / self.fx
        y_cam = (v - self.cy) * depth / self.fy
        z_cam = depth

        point_cam = PointStamped()
        point_cam.header.stamp = self.get_clock().now().to_msg()
        point_cam.header.frame_id = camera_frame
        point_cam.point.x = float(z_cam)
        point_cam.point.y = float(-x_cam)
        point_cam.point.z = float(-y_cam)

        try:
            point_map = self.tf_buffer.transform(
                point_cam, target_frame, timeout=Duration(seconds=0.5))
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

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        results = self.model(cv_image, verbose=False)[0]
        detections = []
        class_names = results.names
        yolo_boxes = []

        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            score = box.conf[0].cpu().numpy()
            cls_id = int(box.cls[0].cpu().numpy())
            detections.append([x1, y1, x2, y2, score])
            yolo_boxes.append({'box': [x1, y1, x2, y2], 'class_name': class_names[cls_id]})

        dets_array = np.array(detections) if len(detections) > 0 else np.empty((0, 5))
        tracked_objects = self.mot_tracker.update(dets_array)

        if len(tracked_objects) > 0:
            self.get_logger().info(f"--- Frame {self.frame_count} ---")
            for track in tracked_objects:
                x1, y1, x2, y2, obj_id = track
                obj_id = int(obj_id)
                best_class = "Object"
                best_iou = 0
                for yb in yolo_boxes:
                    overlap = iou([x1, y1, x2, y2], yb['box'])
                    if overlap > best_iou and overlap > 0.1:
                        best_iou = overlap
                        best_class = yb['class_name']
                self.get_logger().info(
                    f"Detected: {best_class}_{obj_id} at [{int(x1)}, {int(y1)}, {int(x2)}, {int(y2)}]")
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(cv_image, f"{best_class}_{obj_id}", (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

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
