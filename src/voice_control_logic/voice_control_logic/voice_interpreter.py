import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import vosk
import sounddevice as sd
import queue
import json
import os


class VoiceInterpreter(Node):
    def __init__(self):
        super().__init__('voice_interpreter')
        self.declare_parameter('topic_name', '/recognized_speech')
        self.declare_parameter('model_path', os.path.expanduser('~/Downloads/vosk-model-en-us-0.22'))
        self.declare_parameter('samplerate', 16000)
        self.declare_parameter('blocksize', 8000)
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

        self.get_logger().info(
            f'Voice Interpreter started. Topic={topic_name}, model={model_path}, '
            f'samplerate={self.samplerate}, blocksize={self.blocksize}'
        )

        stream_kwargs = {
            'samplerate': self.samplerate,
            'blocksize': self.blocksize,
            'dtype': 'int16',
            'channels': 1,
            'callback': self.audio_callback,
        }
        if audio_device:
            stream_kwargs['device'] = audio_device
        self.stream = sd.RawInputStream(**stream_kwargs)

    def audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warning(f'Audio stream status: {status}')
        self.q.put(bytes(indata))

    def run_recognition(self):
        with self.stream:
            while rclpy.ok():
                data = self.q.get()
                if self.rec.AcceptWaveform(data):
                    result = json.loads(self.rec.Result())
                    text = result.get('text', '')
                    if text:
                        msg = String()
                        msg.data = text
                        self.publisher_.publish(msg)
                        self.get_logger().info(f'Published: "{text}"')


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