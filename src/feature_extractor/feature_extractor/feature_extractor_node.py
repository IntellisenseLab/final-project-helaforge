import time
import sys
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

TORCH_IMPORT_ERROR = None
try:
    import torch
    from torchvision import transforms
    from torchvision.models import MobileNet_V3_Small_Weights, mobilenet_v3_small
    HAS_TORCH = True
except Exception as exc:
    torch = None
    transforms = None
    MobileNet_V3_Small_Weights = None
    mobilenet_v3_small = None
    HAS_TORCH = False
    TORCH_IMPORT_ERROR = str(exc)

class FeatureExtractor(Node):
    def __init__(self):
        super().__init__('feature_extractor')
        self.declare_parameter('input_topic', '/camera/color/image_raw')
        self.declare_parameter('output_topic', '/image_features')
        self.declare_parameter('process_interval_sec', 1.0)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.process_interval_sec = self.get_parameter('process_interval_sec').get_parameter_value().double_value
        
        self.last_processed_time = 0.0
        self.bridge = CvBridge()
        self.use_torch_pipeline = False
        self.device = None
        self.model = None
        self.preprocess = None

        # CHAIR IDENTIFICATION SETUP (Commented for now)
        # MobileNet ImageNet Indices for chairs: 500 (folding chair), 827 (studio couch), 423 (barber chair)
        # self.chair_indices = [500, 827, 423, 765, 831] 

        if HAS_TORCH:
            self.device = torch.device('cpu')
            try:
                weights = MobileNet_V3_Small_Weights.DEFAULT
                self.model = mobilenet_v3_small(weights=weights)
                self.preprocess = weights.transforms()
                self.model = self.model.to(self.device).eval()
                self.use_torch_pipeline = True
                self.get_logger().info('Loaded MobileNetV3-Small pretrained weights.')
            except Exception as exc:
                self.get_logger().warning(f'Could not initialize MobileNetV3 ({exc}). Using OpenCV fallback.')
        else:
            self.get_logger().warning(
                f'PyTorch/torchvision unavailable in {sys.executable} ({TORCH_IMPORT_ERROR}). '
                'Using OpenCV fallback.'
            )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.publisher_ = self.create_publisher(Float32MultiArray, output_topic, qos_profile)
        self.subscription = self.create_subscription(Image, input_topic, self.on_image, qos_profile)

        self.get_logger().info(f'FeatureExtractor started. input_topic={input_topic}')

    def on_image(self, msg):
        now = time.monotonic()
        if (now - self.last_processed_time) < self.process_interval_sec:
            return
        self.last_processed_time = now

        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if self.use_torch_pipeline:
                # 1. Image Preprocessing
                frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                pil_img = transforms.ToPILImage()(frame_rgb) # Convert for transform
                input_tensor = self.preprocess(pil_img).unsqueeze(0).to(self.device)
                
                with torch.no_grad():
                    # --- OPTION A: RAW FEATURES (What you currently use) ---
                    feature_map = self.model.features(input_tensor)
                    pooled = torch.nn.functional.adaptive_avg_pool2d(feature_map, (1, 1))
                    feature_vector = pooled.flatten(start_dim=1).squeeze(0).cpu().tolist()

                    # --- OPTION B: CHAIR IDENTIFICATION LOGIC (Commented) ---
                    output = self.model(input_tensor) # Run full model including classifier
                    probabilities = torch.nn.functional.softmax(output[0], dim=0)
                    top_prob, top_catid = torch.topk(probabilities, 1)
                    
                    #Check if the top ID matches common "chair" indices
                    is_chair = any(idx == top_catid.item() for idx in [500, 827, 423,559])
                    if is_chair:
                        self.get_logger().info(f'MATCH FOUND: Chair identified with {top_prob.item():.2f} confidence!')
                    else:
                        self.get_logger().info(f'Scanning... detected ID: {top_catid.item()}')
            
            else:
                # Fallback Histogram Logic
                hist_b = cv2.calcHist([frame_bgr], [0], None, [32], [0, 256]).flatten()
                hist_g = cv2.calcHist([frame_bgr], [1], None, [32], [0, 256]).flatten()
                hist_r = cv2.calcHist([frame_bgr], [2], None, [32], [0, 256]).flatten() # Fixed
                feature_vector = np.concatenate([hist_b, hist_g, hist_r]).astype(np.float32)
                norm = np.linalg.norm(feature_vector) + 1e-8
                feature_vector = (feature_vector / norm).tolist()

            out = Float32MultiArray()
            out.data = [float(v) for v in feature_vector]
            self.publisher_.publish(out)
            # self.get_logger().info(f'Published features. Length: {len(out.data)}')
            
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