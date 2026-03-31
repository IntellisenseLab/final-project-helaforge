import sounddevice as sd
import vosk
import sys
import queue
import json

# Setup audio queue
q = queue.Queue()

def callback(indata, frames, time, status):
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))

# Load the model you just downloaded
model = vosk.Model("../models/vosk-model-small-en-us-0.15")
rec = vosk.KaldiRecognizer(model, 16000)

print("Speak now! (Ctrl+C to stop)")
with sd.RawInputStream(device=8,samplerate=16000, blocksize=8000, dtype='int16',
                       channels=1, callback=callback):
    while True:
        data = q.get()
        if rec.AcceptWaveform(data):
            result = json.loads(rec.Result())
            print(f"Recognized: {result['text']}")