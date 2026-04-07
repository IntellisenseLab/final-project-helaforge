import time

import cv2
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from torchvision import transforms
from torchvision.models import MobileNet_V3_Small_Weights, mobilenet_v3_small


class FeatureExtractor(Node):
    def __init__(self):
        super().__init__('feature_extractor')
        self.declare_parameter('input_topic', '/camera/color/image_raw')
        self.declare_parameter('output_topic', '/image_features')
        self.declare_parameter('process_interval_sec', 1.0)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.process_interval_sec = (
            self.get_parameter('process_interval_sec').get_parameter_value().double_value
        )
        self.last_processed_time = 0.0

        self.bridge = CvBridge()
        self.device = torch.device('cpu')

        try:
            weights = MobileNet_V3_Small_Weights.DEFAULT
            self.model = mobilenet_v3_small(weights=weights)
            self.preprocess = weights.transforms()
            self.get_logger().info('Loaded MobileNetV3-Small pretrained weights.')
        except Exception as exc:
            self.get_logger().warning(
                f'Could not load pretrained weights ({exc}). Falling back to default transforms.'
            )
            self.model = mobilenet_v3_small(weights=None)
            self.preprocess = transforms.Compose(
                [
                    transforms.ToPILImage(),
                    transforms.Resize((224, 224)),
                    transforms.ToTensor(),
                    transforms.Normalize(
                        mean=[0.485, 0.456, 0.406],
                        std=[0.229, 0.224, 0.225],
                    ),
                ]
            )

        self.model = self.model.to(self.device).eval()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.publisher_ = self.create_publisher(Float32MultiArray, output_topic, qos_profile)
        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.on_image,
            qos_profile,
        )

        self.get_logger().info(
            f'FeatureExtractor started. input_topic={input_topic}, output_topic={output_topic}, '
            f'process_interval_sec={self.process_interval_sec}'
        )

    @torch.no_grad()
    def on_image(self, msg):
        now = time.monotonic()
        if (now - self.last_processed_time) < self.process_interval_sec:
            return
        self.last_processed_time = now

        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            input_tensor = self.preprocess(frame_rgb).unsqueeze(0).to(self.device)

            feature_map = self.model.features(input_tensor)
            pooled = torch.nn.functional.adaptive_avg_pool2d(feature_map, (1, 1))
            feature_vector = pooled.flatten(start_dim=1).squeeze(0).cpu().tolist()

            out = Float32MultiArray()
            out.data = [float(v) for v in feature_vector]
            self.publisher_.publish(out)
            self.get_logger().info(f'Published image feature length: {len(out.data)}')
        except Exception as exc:
            self.get_logger().error(f'Feature extraction failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = FeatureExtractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
