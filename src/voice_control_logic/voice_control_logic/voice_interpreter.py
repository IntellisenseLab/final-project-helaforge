import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import vosk
import sys
import sounddevice as sd
import queue
import json
import os

class VoiceInterpreter(Node):
    def __init__(self):
        super().__init__('voice_interpreter')
        self.publisher_ = self.create_publisher(String, '/recognized_speech', 10)
        
        # Audio setup
        self.q = queue.Queue()
        
        # Point to your model - update this path if necessary!
        model_path = os.path.expanduser('~/Downloads/vosk-model-small-en-us-0.15')
        self.model = vosk.Model(model_path)
        self.rec = vosk.KaldiRecognizer(self.model, 16000)

        self.get_logger().info('Voice Interpreter Node started. Listening...')

        # Start stream in a non-blocking way
        self.stream = sd.RawInputStream(samplerate=16000, blocksize=8000, dtype='int16',
                                       channels=1, callback=self.audio_callback)
        
    def audio_callback(self, indata, frames, time, status):
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
    node = VoiceInterpreter()
    try:
        node.run_recognition()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()