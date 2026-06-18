import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import vosk
import sys
import sounddevice as sd
import queue
import json
import os

class VoiceToTextNode(Node):
    def __init__(self):
        super().__init__('voice_to_text_node')
        self.publisher_ = self.create_publisher(String, '/recognized_speech', 10)
        
        # Audio Setup
        self.q = queue.Queue()
        #self.model = vosk.Model("../models/vosk-model-small-en-us-0.15") # Path to your extracted model folder
        
        
        model_path = os.path.expanduser('~/qbot_ws/src/voice_control_logic/models/vosk-model-small-en-us-0.15')
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