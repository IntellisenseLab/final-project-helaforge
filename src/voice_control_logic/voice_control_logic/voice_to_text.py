import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import vosk
import sys
import sounddevice as sd
import queue
import json
import os

MODEL_DIRNAME = 'vosk-model-small-en-us-0.15'


def resolve_model_path(current_dir):
    env_model_path = os.getenv('VOSK_MODEL_PATH')
    if env_model_path and os.path.isdir(env_model_path):
        return env_model_path

    candidates = [
        os.path.join(current_dir, '..', 'models', MODEL_DIRNAME),
        os.path.join(current_dir, '..', '..', 'models', MODEL_DIRNAME),
    ]

    for parent in [current_dir] + list(path for path in _iter_parents(current_dir)):
        candidates.append(os.path.join(parent, 'models', MODEL_DIRNAME))
        candidates.append(os.path.join(parent, 'src', 'voice_control_logic', 'models', MODEL_DIRNAME))

    for candidate in candidates:
        candidate = os.path.abspath(candidate)
        if os.path.isdir(candidate):
            return candidate

    raise FileNotFoundError(
        f"Could not find Vosk model folder '{MODEL_DIRNAME}'. "
        "Set VOSK_MODEL_PATH or place the model under models/."
    )


def _iter_parents(path):
    current = os.path.abspath(path)
    while True:
        parent = os.path.dirname(current)
        if parent == current:
            break
        yield parent
        current = parent


class VoiceToTextNode(Node):
    def __init__(self):
        super().__init__('voice_to_text_node')
        self.publisher_ = self.create_publisher(String, '/recognized_speech', 10)
        
        # Audio Setup
        self.q = queue.Queue()
        #self.model = vosk.Model("../models/vosk-model-small-en-us-0.15") # Path to your extracted model folder
        
        
        # Resolve model path from env var or common workspace locations.
        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = resolve_model_path(current_dir)
        self.get_logger().info(f'Using Vosk model path: {model_path}')
        self.model = vosk.Model(model_path)
        
        self.rec = vosk.KaldiRecognizer(self.model, 16000)
        # Start Microphone Stream
        self.get_logger().info('Vosk is listening... speak now!')
        with sd.RawInputStream(samplerate=16000, blocksize=8000, dtype='int16',
                               channels=1, callback=self.callback):
            while rclpy.ok():
                data = self.q.get()
                if self.rec.AcceptWaveform(data):
                    result = json.loads(self.rec.Result())
                    text = result.get('text', '')
                    if text:
                        self.get_logger().info(f'Heard: {text}')
                        msg = String()
                        msg.data = text
                        self.publisher_.publish(msg)

    def callback(self, indata, frames, time, status):
        self.q.put(bytes(indata))

def main(args=None):
    rclpy.init(args=args)
    node = VoiceToTextNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()