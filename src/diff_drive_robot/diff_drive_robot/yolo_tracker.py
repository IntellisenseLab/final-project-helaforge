"""
yolo_tracker.py  (BoT-SORT edition)
=====================================
Standalone YOLO + BoT-SORT tracking node.

Replaces the previous SORT-based tracker with the Ultralytics built-in
BoT-SORT tracker for robust, re-identification-aware tracking during
robot movement.

Key changes vs. the SORT edition
---------------------------------
• Uses model.track(image, tracker=cfg, persist=True) — no external SORT code
• `persist=True` maintains tracker state between callback invocations
• Tracking IDs come from results[0].boxes.id (None when no tracks)
• Falls back to bytetrack.yaml via ROS parameter `tracker_cfg`

Subscribes:
  /camera/image_raw  (sensor_msgs/Image, bgr8)

Publishes:
  (visual debug only; actual semantic mapping is done in semantic_navigator)

Parameters:
  tracker_cfg  (string, default "botsort.yaml")
               Set to "bytetrack.yaml" on hardware-constrained platforms.
  every_n      (int, default 2)
               Process every N-th frame to reduce CPU load.

Usage:
  ros2 run diff_drive_robot yolo_tracker
  ros2 run diff_drive_robot yolo_tracker --ros-args -p tracker_cfg:=bytetrack.yaml
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    from ultralytics import YOLO
except ImportError:
    print('[yolo_tracker] Please install ultralytics: '
          'pip3 install ultralytics --break-system-packages')
    YOLO = None


class YoloBotsortTracker(Node):
    def __init__(self):
        super().__init__('yolo_botsort_tracker')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('tracker_cfg',  'botsort.yaml')   # or bytetrack.yaml
        self.declare_parameter('every_n',      2)                 # process every N frames

        camera_topic  = self.get_parameter('camera_topic').value
        self.tracker_cfg = self.get_parameter('tracker_cfg').value
        self.every_n  = self.get_parameter('every_n').value

        self.bridge      = CvBridge()
        self.frame_count = 0

        # ── YOLO model ────────────────────────────────────────────────────────
        if YOLO is not None:
            self.get_logger().info(
                f'Loading YOLO model with {self.tracker_cfg} tracker …')
            self.model = YOLO('yolov8n.pt')
            # Warm-up pass (pre-JIT the graph so first real frame is fast)
            self.model.track(
                np.zeros((480, 640, 3), dtype=np.uint8),
                tracker=self.tracker_cfg,
                persist=True,
                verbose=False)
            self.get_logger().info(
                f'YOLO + {self.tracker_cfg} warmed up ✓\n'
                f'  Switch tracker: --ros-args -p tracker_cfg:=bytetrack.yaml')
        else:
            self.model = None

        # ── Subscription ──────────────────────────────────────────────────────
        self.sub = self.create_subscription(
            Image, camera_topic, self._image_cb, 10)
        self.get_logger().info(
            f'Subscribed to {camera_topic}. Tracking with {self.tracker_cfg}.')

    # ── Image callback ─────────────────────────────────────────────────────────
    def _image_cb(self, msg: Image):
        if self.model is None:
            return

        self.frame_count += 1
        if self.frame_count % self.every_n != 0:
            return

        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        # ── BoT-SORT tracking ─────────────────────────────────────────────────
        # persist=True: tracker state survives across callback invocations.
        # This is what enables consistent track IDs even when objects
        # briefly leave the frame.
        results = self.model.track(
            img,
            tracker=self.tracker_cfg,
            persist=True,
            verbose=False)

        if not results or results[0].boxes is None:
            return

        boxes = results[0].boxes

        # .id is None when BoT-SORT has not yet assigned IDs (first frame)
        if boxes.id is None:
            return

        ids      = boxes.id.cpu().numpy().astype(int)
        xyxy     = boxes.xyxy.cpu().numpy()
        classes  = boxes.cls.cpu().numpy().astype(int)
        names    = results[0].names

        self.get_logger().info(
            f'[frame {self.frame_count}] {len(ids)} track(s):',
            throttle_duration_sec=1.0)

        for tid, box, cls_id in zip(ids, xyxy, classes):
            x1, y1, x2, y2 = box.astype(int)
            label = f'{names[cls_id]}_{tid}'

            # Visual debug
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 200, 50), 2)
            cv2.putText(img, label, (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 50), 2)

            self.get_logger().info(
                f'  {label}: [{x1},{y1}→{x2},{y2}]',
                throttle_duration_sec=1.0)

        cv2.imshow('YOLO + BoT-SORT Tracking', img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloBotsortTracker()
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
