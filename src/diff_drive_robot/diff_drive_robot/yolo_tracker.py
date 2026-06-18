"""
yolo_tracker.py
===============
Standalone YOLO + BoT-SORT tracking test node.

This node is for checking RGB detection only. The production object mapping is
done in semantic_navigator.py, but the same Raspberry Pi friendly rules are used
here:

* load the YOLO model once
* subscribe with sensor-data QoS, queue depth 1
* keep only the newest RGB frame and drop old frames
* run timer-based inference so callbacks stay light
* prevent parallel inference
* keep preview, video saving, and annotated image publishing disabled by default
"""

import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

try:
    from ultralytics import YOLO
except ImportError:
    print('[yolo_tracker] Please install ultralytics: '
          'pip3 install ultralytics --break-system-packages')
    YOLO = None


class YoloBotsortTracker(Node):
    def __init__(self):
        super().__init__('yolo_botsort_tracker')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_topic', '')
        self.declare_parameter('tracker_cfg', 'botsort.yaml')
        self.declare_parameter('yolo_model', 'yolo26n_ncnn_model')
        self.declare_parameter('yolo_imgsz', 640)
        self.declare_parameter('yolo_conf', 0.40)
        self.declare_parameter('detection_rate', 3.0)
        self.declare_parameter('detection_enabled', True)
        self.declare_parameter('preview_enabled', False)
        self.declare_parameter('publish_annotated_image', False)
        self.declare_parameter('save_video', False)
        self.declare_parameter('every_n', 1)

        image_topic = str(self.get_parameter('image_topic').value)
        legacy_topic = str(self.get_parameter('camera_topic').value)
        self.image_topic = legacy_topic if legacy_topic else image_topic
        self.tracker_cfg = str(self.get_parameter('tracker_cfg').value)
        yolo_model = str(self.get_parameter('yolo_model').value)
        self.yolo_imgsz = int(self.get_parameter('yolo_imgsz').value)
        self.yolo_conf = float(self.get_parameter('yolo_conf').value)
        self.detection_rate = max(
            0.1, float(self.get_parameter('detection_rate').value))
        self.detection_enabled = bool(
            self.get_parameter('detection_enabled').value)
        self.preview_enabled = bool(
            self.get_parameter('preview_enabled').value)
        self.publish_annotated_image = bool(
            self.get_parameter('publish_annotated_image').value)
        self.save_video = bool(self.get_parameter('save_video').value)

        self.bridge = CvBridge()
        self.frame_count = 0
        self._latest_msg: Image | None = None
        self._frame_lock = threading.Lock()
        self._inference_lock = threading.Lock()
        self._last_inference_time = 0.0

        self.model = None
        if YOLO is not None:
            try:
                self.get_logger().info(
                    f'Loading YOLO model {yolo_model} with '
                    f'{self.tracker_cfg} tracker ...')
                self.model = YOLO(yolo_model)
                self.model.track(
                    np.zeros((480, 640, 3), dtype=np.uint8),
                    tracker=self.tracker_cfg,
                    persist=True,
                    imgsz=self.yolo_imgsz,
                    conf=self.yolo_conf,
                    show=False,
                    save=False,
                    save_txt=False,
                    verbose=False)
                self.get_logger().info(
                    f'YOLO warmed up: model={yolo_model}, '
                    f'imgsz={self.yolo_imgsz}, conf={self.yolo_conf:.2f}, '
                    f'rate={self.detection_rate:.1f} fps')
            except Exception as exc:
                self.get_logger().warn(
                    f'YOLO could not start ({exc}); detection disabled.')
                self.model = None

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(Image, self.image_topic, self._image_cb, qos)
        self.create_timer(0.05, self._detection_timer_cb)

        self.get_logger().info(
            f'Subscribed to {self.image_topic}. '
            f'Preview enabled: {self.preview_enabled}.')

    def _image_cb(self, msg: Image):
        with self._frame_lock:
            self._latest_msg = msg

    def _detection_timer_cb(self):
        if not self.detection_enabled or self.model is None:
            return

        now = time.monotonic()
        if now - self._last_inference_time < (1.0 / self.detection_rate):
            return

        if not self._inference_lock.acquire(blocking=False):
            return

        try:
            with self._frame_lock:
                msg = self._latest_msg
                self._latest_msg = None
            if msg is None:
                return

            self._last_inference_time = now
            self._run_detection(msg)
        finally:
            self._inference_lock.release()

    def _run_detection(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(f'Image conversion failed: {exc}')
            return

        self.frame_count += 1
        results = self.model.track(
            img,
            tracker=self.tracker_cfg,
            persist=True,
            imgsz=self.yolo_imgsz,
            conf=self.yolo_conf,
            show=False,
            save=False,
            save_txt=False,
            verbose=False)

        if not results or results[0].boxes is None:
            return

        boxes = results[0].boxes
        if boxes.id is None:
            return

        ids = boxes.id.cpu().numpy().astype(int)
        xyxy = boxes.xyxy.cpu().numpy()
        classes = boxes.cls.cpu().numpy().astype(int)
        names = results[0].names

        self.get_logger().info(
            f'[frame {self.frame_count}] {len(ids)} track(s)',
            throttle_duration_sec=1.0)

        for tid, box, cls_id in zip(ids, xyxy, classes):
            x1, y1, x2, y2 = box.astype(int)
            label = f'{names[cls_id]}_{tid}'
            self.get_logger().info(
                f'  {label}: [{x1},{y1}->{x2},{y2}]',
                throttle_duration_sec=1.0)

            if self.preview_enabled:
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 200, 50), 2)
                cv2.putText(
                    img,
                    label,
                    (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 200, 50),
                    2)

        if self.preview_enabled:
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
