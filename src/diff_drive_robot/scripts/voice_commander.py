#!/usr/bin/env python3
"""
Voice Commander for SemanticNavigator
======================================
Uses Vosk offline speech recognition to issue commands via voice.

Recognized phrases:
  "scan"             → starts scanning
  "stop scan"        → stops scanning, robot returns home
  "go to <object>"   → navigate to detected object
  "return home"      → robot returns to start position
  "list"             → print detected objects

Requires:
  pip3 install vosk sounddevice
  Download model: https://alphacephei.com/vosk/models
  Place in ~/vosk-model  (or set VOSK_MODEL_PATH env var)
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


# ── Audio queue ──────────────────────────────────────────────────────
audio_q: queue.Queue = queue.Queue()

def audio_callback(indata, frames, time_info, status):
    """Called by sounddevice for each audio block."""
    audio_q.put(bytes(indata))


class VoiceCommander(Node):

    def __init__(self, model_path: str):
        super().__init__('voice_commander')
        self.pub = self.create_publisher(String, '/semantic_nav/command', 10)

        # ── Load Vosk model ──────────────────────────────────────────
        if not os.path.isdir(model_path):
            self.get_logger().error(
                f'Vosk model not found at: {model_path}\n'
                f'Download from https://alphacephei.com/vosk/models\n'
                f'Extract to ~/vosk-model or set VOSK_MODEL_PATH')
            sys.exit(1)

        self.get_logger().info(f'Loading Vosk model from {model_path} …')
        self.model = Model(model_path)
        self.recognizer = KaldiRecognizer(self.model, 16000)
        self.get_logger().info('Vosk model loaded ✓')

        # ── Known object list (populated when user says "list") ──────
        self.get_logger().info(
            '\n'
            '╔══════════════════════════════════════╗\n'
            '║   🎤  VOICE COMMANDER READY  🎤      ║\n'
            '║                                      ║\n'
            '║  Say:                                 ║\n'
            '║    "scan"          - start scanning   ║\n'
            '║    "stop scan"     - stop & go home   ║\n'
            '║    "go to chair"   - navigate to obj  ║\n'
            '║    "return home"   - go back to start ║\n'
            '║    "list"          - show objects      ║\n'
            '║    "quit" / "exit" - stop this node   ║\n'
            '╚══════════════════════════════════════╝')

    def process_text(self, text: str):
        """Parse recognized text and publish the appropriate command."""
        text = text.strip().lower()
        if not text:
            return

        self.get_logger().info(f'Heard: "{text}"')

        cmd = None

        # ── Match commands ───────────────────────────────────────────
        if 'scan' in text and ('stop' in text or 'end' in text or 'finish' in text):
            cmd = 'scan stop'
        elif 'scan' in text and 'start' not in text:
            # Just "scan" alone
            if text.strip() in ('scan', 'begin scan', 'start scan'):
                cmd = 'scan'
            else:
                cmd = 'scan'
        elif 'start' in text and 'scan' in text:
            cmd = 'scan'
        elif 'return' in text and 'home' in text:
            cmd = 'return home'
        elif 'go home' in text or 'come home' in text or 'back home' in text:
            cmd = 'return home'
        elif 'go to' in text:
            # Extract object name after "go to"
            idx = text.index('go to') + 5
            obj_name = text[idx:].strip()
            if obj_name:
                cmd = obj_name
            else:
                self.get_logger().warn('  → "go to" what? Say "go to chair_5"')
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
            self.get_logger().info(f'  → Published: "{cmd}"')
        else:
            self.get_logger().info(f'  → (not a recognized command)')


def main(args=None):
    rclpy.init(args=args)

    # ── Find Vosk model ──────────────────────────────────────────────
    model_path = os.environ.get(
        'VOSK_MODEL_PATH',
        os.path.expanduser('~/vosk-model'))

    node = VoiceCommander(model_path)

    # ── Open microphone ──────────────────────────────────────────────
    device_info = sd.query_devices(kind='input')
    samplerate = int(device_info['default_samplerate'])
    # Vosk expects 16000 Hz — we'll resample if needed but try native
    samplerate = 16000

    try:
        with sd.RawInputStream(
                samplerate=samplerate,
                blocksize=8000,
                dtype='int16',
                channels=1,
                callback=audio_callback):

            node.get_logger().info(
                f'🎤 Microphone open (16 kHz). Speak your commands!')

            while rclpy.ok():
                data = audio_q.get()
                if node.recognizer.AcceptWaveform(data):
                    result = json.loads(node.recognizer.Result())
                    text = result.get('text', '')
                    if text:
                        node.process_text(text)
                # Partial results (optional live feedback)
                # partial = json.loads(node.recognizer.PartialResult())

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
