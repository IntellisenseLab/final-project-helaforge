import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import vosk
import sounddevice as sd
import queue
import json
import os
import re
import time
from ament_index_python.packages import get_package_share_directory


class VoiceInterpreter(Node):
    def __init__(self):
        super().__init__('voice_interpreter')
        self.command_map = {
            'forward': 'move_forward',
            'backward': 'move_backward',
            'left': 'turn_left',
            'right': 'turn_right',
            'stop': 'stop',
        }
        self.command_aliases = {
            'stop': {'stop', 'halt', 'freeze', 'wait'},
            'forward': {'forward', 'ahead', 'straight'},
            'backward': {'backward', 'back', 'reverse'},
            'left': {'left'},
            'right': {'right', 'write'},
        }
        self.last_published = ''
        self.last_publish_time = 0.0

        self.declare_parameter('topic_name', '/recognized_speech')
        # Resolve model path from installed package location
        try:
            package_share_dir = get_package_share_directory('voice_control_logic')
            default_model = os.path.join(package_share_dir, 'models/vosk-model-small-en-us-0.15')
        except Exception:
            # Fallback for development: look in src directory
            default_model = os.path.join(os.path.dirname(__file__), '../../models/vosk-model-small-en-us-0.15')
        self.declare_parameter('model_path', default_model)
        self.declare_parameter('samplerate', 16000)
        # 1600 samples at 16 kHz ~= 100 ms chunks, much lower latency than 8000.
        self.declare_parameter('blocksize', 1600)
        self.declare_parameter('audio_device', '')

        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.samplerate = self.get_parameter('samplerate').get_parameter_value().integer_value
        self.blocksize = self.get_parameter('blocksize').get_parameter_value().integer_value
        audio_device = self.get_parameter('audio_device').get_parameter_value().string_value

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.publisher_ = self.create_publisher(String, topic_name, qos_profile)

        self.q = queue.Queue()

        if not os.path.isdir(model_path):
            raise FileNotFoundError(
                f'Vosk model folder not found: {model_path}. '
                'Pass a valid path using --ros-args -p model_path:=/path/to/model'
            )

        self.model = vosk.Model(model_path)
        self.rec = vosk.KaldiRecognizer(self.model, self.samplerate)

        try:
            default_input_idx = sd.default.device[0]
            input_name = sd.query_devices(default_input_idx)['name']
            self.get_logger().info(f'Default input device: {default_input_idx} ({input_name})')
        except Exception as exc:
            self.get_logger().warning(f'Could not query default input device: {exc}')

        self.get_logger().info(
            f'Voice Interpreter started. Topic={topic_name}, model={model_path}, '
            f'samplerate={self.samplerate}, blocksize={self.blocksize}'
        )

        stream_kwargs = {
            'samplerate': self.samplerate,
            'blocksize': self.blocksize,
            'dtype': 'int16',
            'channels': 1,
            'latency': 'low',
            'callback': self.audio_callback,
        }
        if audio_device:
            stream_kwargs['device'] = audio_device
        self.stream = sd.RawInputStream(**stream_kwargs)

    def audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warning(f'Audio stream status: {status}')
        self.q.put(bytes(indata))

    def extract_command(self, recognized_text):
        cleaned = re.sub(r'[^a-z\s]', ' ', recognized_text.lower()).strip()
        tokens = [token for token in cleaned.split() if token]
        if not tokens:
            return None

        # Safety first: stop has priority whenever detected in a phrase.
        for alias in self.command_aliases['stop']:
            if alias in tokens:
                return 'stop'

        for command in ('forward', 'backward', 'left', 'right'):
            for alias in self.command_aliases[command]:
                if alias in tokens:
                    return command

        return self.command_map.get(cleaned) and cleaned

    def publish_command(self, command_key, recognized_text):
        mapped = self.command_map.get(command_key)
        if mapped is None:
            self.get_logger().info(f'Unrecognized: "{recognized_text}"')
            return

        now = time.monotonic()
        if mapped == self.last_published and (now - self.last_publish_time) < 0.3:
            return

        msg = String()
        msg.data = mapped
        self.publisher_.publish(msg)
        self.last_published = mapped
        self.last_publish_time = now
        self.get_logger().info(f'Published: "{msg.data}"')

    def run_recognition(self):
        with self.stream:
            while rclpy.ok():
                data = self.q.get()
                if self.rec.AcceptWaveform(data):
                    result = json.loads(self.rec.Result())
                    text = result.get('text', '').strip().lower()
                    if text:
                        command_key = self.extract_command(text)
                        self.publish_command(command_key, text)
                else:
                    # Use partial hypotheses for snappier command response.
                    partial = json.loads(self.rec.PartialResult()).get('partial', '').strip().lower()
                    if partial:
                        command_key = self.extract_command(partial)
                        if command_key is not None:
                            self.publish_command(command_key, partial)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VoiceInterpreter()
        node.run_recognition()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f'Failed to start voice_interpreter: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        # Avoid RCLError when shutdown was already triggered by SIGINT.
        if rclpy.ok():
            rclpy.shutdown()