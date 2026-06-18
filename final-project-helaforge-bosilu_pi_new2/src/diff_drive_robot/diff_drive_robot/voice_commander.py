"""
Voice Commander for SemanticNavigator – Vosk offline speech recognition.
"""
import os
import sys
import json
import queue
import re
from difflib import SequenceMatcher
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


START_ALIASES = {
    'start',
    'scan',
    'start scan',
    'start scanning',
    'scan environment',
    'scan the environment',
    'scan room',
    'scan the room',
    'scan area',
    'map',
    'map room',
    'map the room',
    'map environment',
    'map the environment',
    'start map',
    'start mapping',
    'start mapping environment',
    'start mapping the environment',
    'start environment mapping',
    'begin',
    'begin scan',
    'begin scanning',
    'begin mapping',
    'begin mapping environment',
    'begin mapping the environment',
    'begin environment mapping',
    'create map',
    'create the map',
    'explore',
    'start exploration',
}

STOP_ALIASES = {
    'stop',
    'scan stop',
    'stop scan',
    'stop scanning',
    'stop mapping',
    'stop map',
    'finish',
    'finish scan',
    'finish scanning',
    'finish mapping',
    'end',
    'end scan',
    'end scanning',
    'end mapping',
    'complete scan',
    'done',
    'mapping done',
    'scanning done',
}

HOME_ALIASES = {
    'return home',
    'go home',
    'go to home',
    'come home',
    'back home',
    'go back home',
    'go back to home',
    'return to home',
    'return to start',
    'go to start',
    'go back to start',
    'back to start',
    'home',
}

LIST_ALIASES = {
    'list',
    'objects',
    'list objects',
    'show objects',
    'show object list',
    'show me objects',
    'what objects',
    'what objects did you see',
    'detected objects',
    'tell me objects',
}

GO_TO_PREFIXES = (
    'go to ',
    'go two ',
    'go too ',
    'go the ',
    'go ',
    'navigate to ',
    'navigate ',
    'move to ',
    'move ',
    'drive to ',
    'drive ',
    'take me to ',
    'find ',
)

COMMON_OBJECT_NAMES = (
    'person', 'chair', 'table', 'dining table', 'bottle', 'cup', 'book',
    'laptop', 'keyboard', 'mouse', 'cell phone', 'phone', 'remote', 'tv',
    'monitor', 'backpack', 'handbag', 'suitcase', 'umbrella', 'bed', 'couch',
    'sofa', 'door', 'box', 'bag', 'plant', 'clock', 'vase', 'scissors',
    'teddy bear', 'toothbrush', 'microwave', 'oven', 'sink', 'refrigerator',
    'fridge', 'toilet', 'car', 'bicycle', 'motorcycle', 'bus', 'truck',
    'traffic light', 'bench', 'cat', 'dog', 'banana', 'apple', 'orange',
    'sandwich', 'plate', 'fork', 'knife', 'spoon', 'bowl',
)

NUMBER_WORDS = {
    'zero': '0',
    'one': '1',
    'two': '2',
    'too': '2',
    'to': '2',
    'three': '3',
    'four': '4',
    'for': '4',
    'five': '5',
    'six': '6',
    'seven': '7',
    'eight': '8',
    'ate': '8',
    'nine': '9',
    'ten': '10',
    'eleven': '11',
    'twelve': '12',
    'thirteen': '13',
    'fourteen': '14',
    'fifteen': '15',
    'sixteen': '16',
    'seventeen': '17',
    'eighteen': '18',
    'nineteen': '19',
    'twenty': '20',
    'thirty': '30',
    'forty': '40',
    'fifty': '50',
    'sixty': '60',
    'seventy': '70',
    'eighty': '80',
    'ninety': '90',
}

DIGIT_WORDS = {
    'zero': '0',
    'one': '1',
    'two': '2',
    'too': '2',
    'to': '2',
    'three': '3',
    'four': '4',
    'for': '4',
    'five': '5',
    'six': '6',
    'seven': '7',
    'eight': '8',
    'ate': '8',
    'nine': '9',
}

SMALL_NUMBER_VALUES = {
    'zero': 0,
    'one': 1,
    'two': 2,
    'too': 2,
    'to': 2,
    'three': 3,
    'four': 4,
    'for': 4,
    'five': 5,
    'six': 6,
    'seven': 7,
    'eight': 8,
    'ate': 8,
    'nine': 9,
    'ten': 10,
    'eleven': 11,
    'twelve': 12,
    'thirteen': 13,
    'fourteen': 14,
    'fifteen': 15,
    'sixteen': 16,
    'seventeen': 17,
    'eighteen': 18,
    'nineteen': 19,
}

TENS_NUMBER_VALUES = {
    'twenty': 20,
    'thirty': 30,
    'forty': 40,
    'fifty': 50,
    'sixty': 60,
    'seventy': 70,
    'eighty': 80,
    'ninety': 90,
}

NUMBER_FILLER_WORDS = {'number', 'no'}

NUMBER_GRAMMAR_ALIASES = {
    1: ('one',),
    2: ('two',),
    3: ('three',),
    4: ('four',),
    5: ('five',),
    6: ('six',),
    7: ('seven',),
    8: ('eight',),
    9: ('nine',),
    10: ('ten',),
    11: ('eleven', 'one one'),
    12: ('twelve', 'one two'),
    13: ('thirteen', 'one three'),
    14: ('fourteen', 'one four'),
    15: ('fifteen', 'one five'),
    16: ('sixteen', 'one six'),
    17: ('seventeen', 'one seven'),
    18: ('eighteen', 'one eight'),
    19: ('nineteen', 'one nine'),
    20: ('twenty', 'two zero'),
    21: ('twenty one', 'two one'),
    22: ('twenty two', 'two two'),
    23: ('twenty three', 'two three'),
    24: ('twenty four', 'two four'),
    25: ('twenty five', 'two five'),
    26: ('twenty six', 'two six'),
    27: ('twenty seven', 'two seven'),
    28: ('twenty eight', 'two eight'),
    29: ('twenty nine', 'two nine'),
    30: ('thirty', 'three zero'),
}

COMMAND_ALIASES = START_ALIASES | STOP_ALIASES | HOME_ALIASES | LIST_ALIASES


def normalize_text(text: str) -> str:
    text = text.strip().lower().replace('_', ' ')
    text = re.sub(r'[^a-z0-9 ]+', ' ', text)
    return ' '.join(text.split())


def env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def best_match(text: str, choices, cutoff: float = 0.78) -> str | None:
    best = None
    best_score = 0.0
    for choice in choices:
        score = SequenceMatcher(None, text, choice).ratio()
        if score > best_score:
            best = choice
            best_score = score
    return best if best is not None and best_score >= cutoff else None


def spoken_number_to_digits(words: list[str]) -> str | None:
    words = [w for w in words if w not in NUMBER_FILLER_WORDS]
    if not words:
        return None

    if all(w.isdigit() for w in words):
        return ''.join(words)

    if len(words) > 1 and all(w in DIGIT_WORDS for w in words):
        return ''.join(DIGIT_WORDS[w] for w in words)

    total = 0
    current = 0
    used = False
    for word in words:
        if word in SMALL_NUMBER_VALUES:
            current += SMALL_NUMBER_VALUES[word]
            used = True
        elif word in TENS_NUMBER_VALUES:
            current += TENS_NUMBER_VALUES[word]
            used = True
        elif word == 'hundred' and used:
            current *= 100
        else:
            return None

    total += current
    return str(total) if used else None


def object_words_to_label(obj_name: str) -> str:
    words = obj_name.split()
    if not words:
        return obj_name

    # Convert suffixes like "chair thirteen", "chair one three",
    # and "chair number thirteen" to "chair 13".
    max_suffix_words = min(4, len(words) - 1)
    for suffix_len in range(max_suffix_words, 0, -1):
        split = len(words) - suffix_len
        number = spoken_number_to_digits(words[split:])
        if number is not None:
            return ' '.join(words[:split] + [number])

    return ' '.join(words)


def build_command_grammar() -> list[str]:
    phrases = set(COMMAND_ALIASES)
    for prefix in GO_TO_PREFIXES:
        clean_prefix = prefix.strip()
        for obj in COMMON_OBJECT_NAMES:
            phrases.add(f'{clean_prefix} {obj}')
    numbered_prefixes = ('go to', 'navigate to', 'drive to', 'move to', 'find')
    for prefix in numbered_prefixes:
        for obj in COMMON_OBJECT_NAMES:
            for number_aliases in NUMBER_GRAMMAR_ALIASES.values():
                for number_text in number_aliases:
                    phrases.add(f'{prefix} {obj} {number_text}')
                    phrases.add(f'{prefix} {obj} number {number_text}')
    phrases.update({'quit', 'exit', 'close', 'shutdown voice'})
    # [unk] lets Vosk keep object names that are not in the fixed command list.
    phrases.add('[unk]')
    return sorted(phrases)


def audio_callback(indata, frames, time_info, status):
    if status:
        # Keep stdout quiet during normal operation, but this helps catch
        # microphone underrun/overflow when debugging poor recognition.
        print(status, file=sys.stderr)
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
        self.use_grammar = env_bool('VOICE_COMMAND_GRAMMAR', True)
        if self.use_grammar:
            grammar = build_command_grammar()
            self.recognizer = KaldiRecognizer(
                self.model, 16000, json.dumps(grammar))
            self.get_logger().info(
                f'Command grammar enabled ({len(grammar)} phrases + [unk]).')
        else:
            self.recognizer = KaldiRecognizer(self.model, 16000)
            self.get_logger().info('Command grammar disabled.')
        self.recognizer.SetWords(True)
        self.get_logger().info('Vosk model loaded')

        self.get_logger().info(
            '\n'
            '╔══════════════════════════════════════╗\n'
            '║      VOICE COMMANDER READY           ║\n'
            '║                                      ║\n'
            '║  Say:                                 ║\n'
            '║    "start mapping" - start scan       ║\n'
            '║    "stop mapping"  - stop/go home     ║\n'
            '║    "go to chair"   - navigate object  ║\n'
            '║    "return home"   - go back to start ║\n'
            '║    "list"          - show objects      ║\n'
            '║    "quit" / "exit" - stop this node   ║\n'
            '╚══════════════════════════════════════╝')

    def process_text(self, text: str):
        text = normalize_text(text)
        if not text:
            return

        self.get_logger().info(f'Heard: "{text}"')

        cmd = None

        if text in {'quit', 'exit', 'close', 'shutdown voice'}:
            self.get_logger().info('Voice Commander shutting down …')
            raise SystemExit()

        fuzzy = best_match(text, COMMAND_ALIASES)
        if fuzzy and fuzzy != text:
            self.get_logger().info(f'  Interpreting as: "{fuzzy}"')
            text = fuzzy

        if text in STOP_ALIASES:
            cmd = 'scan stop'
        elif text in START_ALIASES:
            cmd = 'scan'
        elif text in HOME_ALIASES:
            cmd = 'return home'
        elif text in LIST_ALIASES:
            cmd = 'list'
        else:
            for prefix in GO_TO_PREFIXES:
                if text.startswith(prefix):
                    obj_name = text[len(prefix):].strip()
                    if obj_name:
                        cmd = f'go to {object_words_to_label(obj_name)}'
                    else:
                        self.get_logger().warn(
                            '  "go to" what? Say "go to chair_5"')
                    break

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

    samplerate = int(os.environ.get('VOICE_SAMPLE_RATE', '16000'))
    blocksize = int(os.environ.get('VOICE_BLOCKSIZE', '4000'))
    device = os.environ.get('VOICE_INPUT_DEVICE')
    if device is not None:
        try:
            device = int(device)
        except ValueError:
            pass

    try:
        with sd.RawInputStream(
                samplerate=samplerate,
                blocksize=blocksize,
                dtype='int16',
                channels=1,
                device=device,
                callback=audio_callback):

            node.get_logger().info(
                f'Microphone open ({samplerate} Hz, block={blocksize}, '
                f'device={device if device is not None else "default"}). '
                'Speak your commands!')

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
