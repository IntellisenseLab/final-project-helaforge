"""
Voice Commander for SemanticNavigator – Vosk offline speech recognition.
"""
import os
import sys
import json
import queue
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import sounddevice as sd
except ImportError:
    print("ERROR: pip3 install sounddevice --break-system-packages")
    sys.exit(1)

try:
    from vosk import Model, KaldiRecognizer
except ImportError:
    print("ERROR: pip3 install vosk --break-system-packages")
    sys.exit(1)


audio_q: queue.Queue = queue.Queue()

def audio_callback(indata, frames, time_info, status):
    audio_q.put(bytes(indata))


class VoiceCommander(Node):

    def __init__(self, model_path: str):
        super().__init__('voice_commander')
        self.pub = self.create_publisher(String, '/semantic_nav/command', 10)

        if not os.path.isdir(model_path):
            self.get_logger().error(
                f'Vosk model not found at: {model_path}\n'
                f'Download from https://alphacephei.com/vosk/models\n'
                f'Extract to ~/vosk-model or set VOSK_MODEL_PATH')
            sys.exit(1)

        self.get_logger().info(f'Loading Vosk model from {model_path} …')
        self.model = Model(model_path)
        self.recognizer = KaldiRecognizer(self.model, 16000)
        self.get_logger().info('Vosk model loaded')

        self.get_logger().info(
            '\n'
            '╔══════════════════════════════════════╗\n'
            '║      VOICE COMMANDER READY           ║\n'
            '║                                      ║\n'
            '║  Say:                                 ║\n'
            '║    "scan environment" - start scan    ║\n'
            '║    "scan stop"        - stop/go home  ║\n'
            '║    "go to chair"      - navigate obj  ║\n'
            '║    "return home"   - go back to start ║\n'
            '║    "list"          - show objects      ║\n'
            '║    "quit" / "exit" - stop this node   ║\n'
            '╚══════════════════════════════════════╝')

    def process_text(self, text: str):
        text = text.strip().lower()
        if not text:
            return

        self.get_logger().info(f'Heard: "{text}"')

        cmd = None

        if 'scan' in text and ('stop' in text or 'end' in text or 'finish' in text):
            cmd = 'scan stop'
        elif 'scan environment' in text or 'scan the environment' in text:
            cmd = 'scan'
        elif 'scan' in text:
            cmd = 'scan'
        elif 'start' in text and 'scan' in text:
            cmd = 'scan'
        elif 'return' in text and 'home' in text:
            cmd = 'return home'
        elif 'go home' in text or 'come home' in text or 'back home' in text:
            cmd = 'return home'
        elif 'go to' in text:
            idx = text.index('go to') + 5
            obj_name = text[idx:].strip()
            if obj_name:
                cmd = obj_name
            else:
                self.get_logger().warn('  "go to" what? Say "go to chair_5"')
        elif 'navigate to' in text:
            idx = text.index('navigate to') + 11
            obj_name = text[idx:].strip()
            if obj_name:
                cmd = obj_name
        elif 'list' in text or 'show' in text or 'objects' in text:
            cmd = 'list'
        elif 'quit' in text or 'exit' in text:
            self.get_logger().info('Voice Commander shutting down …')
            raise SystemExit()

        if cmd:
            msg = String()
            msg.data = cmd
            self.pub.publish(msg)
            self.get_logger().info(f'  Published: "{cmd}"')
        else:
            self.get_logger().info(f'  (not a recognized command)')


def main(args=None):
    rclpy.init(args=args)

    model_path = os.environ.get(
        'VOSK_MODEL_PATH',
        os.path.expanduser('~/vosk-model'))

    node = VoiceCommander(model_path)

    samplerate = 16000

    try:
        with sd.RawInputStream(
                samplerate=samplerate,
                blocksize=8000,
                dtype='int16',
                channels=1,
                callback=audio_callback):

            node.get_logger().info(
                f'Microphone open (16 kHz). Speak your commands!')

            while rclpy.ok():
                data = audio_q.get()
                if node.recognizer.AcceptWaveform(data):
                    result = json.loads(node.recognizer.Result())
                    text = result.get('text', '')
                    if text:
                        node.process_text(text)

    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    except Exception as e:
        node.get_logger().error(f'Audio error: {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
