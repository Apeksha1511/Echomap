# speech_input.py
# Offline Vosk speech recognition for EchoMap
# ----------------------------------------------------------
# Two modes:
# 1. Landmark naming (free-form speech)
# 2. Mapping-mode movement commands (grammar-based recognition)
# ----------------------------------------------------------

import os
import json
import time

MODEL_PATH = os.environ.get(
    "ECHOMAP_VOSK_MODEL",
    os.path.join(os.path.dirname(__file__), "model", "vosk")
)

MIC_NAME = "Jabra EVOLVE 20 MS"


def _get_mic_index(pa):
    """
    Automatically detect the Jabra USB microphone.
    Falls back to the default microphone if not found.
    """
    for i in range(pa.get_device_count()):
        info = pa.get_device_info_by_index(i)

        if (
            MIC_NAME.lower() in info["name"].lower()
            and info["maxInputChannels"] > 0
        ):
            print(f"[SPEECH] Using microphone: {info['name']} (index {i})")
            return i

    print("[SPEECH] Jabra microphone not found. Using default microphone.")
    return None


def _listen_raw(timeout=6, grammar=None):
    try:
        from vosk import Model, KaldiRecognizer
        import pyaudio
        import json, time

        model = Model(MODEL_PATH)

        if grammar:
            rec = KaldiRecognizer(model, 16000, json.dumps(grammar))
        else:
            rec = KaldiRecognizer(model, 16000)

        pa = pyaudio.PyAudio()
        mic_index = _get_mic_index(pa)

        stream = pa.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            input_device_index=mic_index,
            frames_per_buffer=4000,
        )

        stream.start_stream()

        # Warm up microphone
        for _ in range(3):
            stream.read(4000, exception_on_overflow=False)

        print("[VOICE] Listening...")

        end = time.time() + timeout

        while time.time() < end:
            data = stream.read(4000, exception_on_overflow=False)

            if rec.AcceptWaveform(data):
                result = json.loads(rec.Result())
                text = result.get("text", "").strip()
                if text:
                    print("[VOICE] Heard:", text)
                    stream.stop_stream()
                    stream.close()
                    pa.terminate()
                    return text

            else:
                partial = json.loads(rec.PartialResult()).get("partial", "")
                if partial:
                    print("[VOICE] Partial:", partial)

        # Return whatever Vosk has buffered
        result = json.loads(rec.FinalResult())
        text = result.get("text", "").strip()

        stream.stop_stream()
        stream.close()
        pa.terminate()

        if text:
            print("[VOICE] Final:", text)
            return text

        return None

    except Exception as e:
        print("[SPEECH ERROR]", e)
        return None

# ------------------------------------------------------------------
# LANDMARK NAMING
# ------------------------------------------------------------------

def listen_for_name(timeout=6):
    """
    Free-form landmark naming using offline Vosk.
    Returns spoken landmark name like 'computer lab' or 'kitchen'.
    """
    text = _listen_raw(timeout=timeout, grammar=None)

    if text:
        text = text.lower().strip()

        # Clean up common filler words
        fillers = {"the", "a", "an"}
        words = [w for w in text.split() if w not in fillers]
        text = " ".join(words)

        print("[VOICE] Landmark recognized:", text)
        return text

    print("[VOICE] Landmark not detected.")
    return None


# ------------------------------------------------------------------
# MOVEMENT COMMANDS
# ------------------------------------------------------------------

GRAMMAR_COMMANDS = [
    "left",
    "right",
    "forward",
    "backward",
    "walk forward",
    "walk backward",
    "move forward",
    "move backward",
    "turn left",
    "turn right",
    "move left",
    "move right",
    "stop",
    "finish",

    # Step commands
    "walk one step",
    "walk two steps",
    "walk three steps",
    "walk four steps",
    "walk five steps",
    "walk six steps",
    "walk seven steps",
    "walk eight steps",
    "walk nine steps",
    "walk ten steps",

    "move one step",
    "move two steps",
    "move three steps",
    "move four steps",
    "move five steps",
    "move six steps",
    "move seven steps",
    "move eight steps",
    "move nine steps",
    "move ten steps",
]

COMMAND_MAP = {
    "left": "turn_left",
    "turn left": "turn_left",

    "right": "turn_right",
    "turn right": "turn_right",

    "move left": "move_left",
    "move right": "move_right",

    "forward": "forward",
    "walk forward": "forward",
    "move forward": "forward",

    "backward": "backward",
    "walk backward": "backward",
    "move backward": "backward",

    "walk one step": "forward_1",
    "walk two steps": "forward_2",
    "walk three steps": "forward_3",
    "walk four steps": "forward_4",
    "walk five steps": "forward_5",
    "walk six steps": "forward_6",
    "walk seven steps": "forward_7",
    "walk eight steps": "forward_8",
    "walk nine steps": "forward_9",
    "walk ten steps": "forward_10",

    "move one step": "forward_1",
    "move two steps": "forward_2",
    "move three steps": "forward_3",
    "move four steps": "forward_4",
    "move five steps": "forward_5",
    "move six steps": "forward_6",
    "move seven steps": "forward_7",
    "move eight steps": "forward_8",
    "move nine steps": "forward_9",
    "move ten steps": "forward_10",

    "stop": "stop",
    "finish": "finish",
}
# ==========================================================
# Mapping Mode Commands (Ultrasonic SLAM)
# ==========================================================

GRAMMAR_MAPPING = [
    "turn left",
    "turn right"
]

MAPPING_COMMAND_MAP = {
    "turn left": "turn_left",
    "turn right": "turn_right"
}


def listen_for_mapping_command(timeout=5):
    """
    Used ONLY in Mapping Mode.
    Accepts only 'turn left' or 'turn right'.
    """
    text = _listen_raw(timeout=timeout, grammar=GRAMMAR_MAPPING)

    if not text:
        print("[VOICE] Mapping command not detected.")
        return None

    text = text.lower().strip()
    print(f"[VOICE] Mapping command: {text}")

    return MAPPING_COMMAND_MAP.get(text)


def listen_for_command(timeout=6):
    """
    Used during Mapping Mode when Button 2 is pressed.

    Returns commands such as:
        turn_left
        turn_right
        move_left
        move_right
        forward
        backward
        forward_5
        forward_6
        stop
        finish
    """

    text = _listen_raw(timeout=timeout, grammar=GRAMMAR_COMMANDS)

    if not text:
        print("[VOICE] Command not detected.")
        return None

    text = text.lower().strip()

    print(f"[VOICE] Recognized command: {text}")

    if text in COMMAND_MAP:
        return COMMAND_MAP[text]

    print("[VOICE] Unknown command.")
    return None

# ------------------------------------------------------------------
# TEST MODE
# ------------------------------------------------------------------

if __name__ == "__main__":
    print("\n--- EchoMap Speech Test ---")

    while True:
        print("\n1. Landmark Name")
        print("2. Movement Command")
        print("3. Exit")

        choice = input("Choice: ")

        if choice == "1":
            name = listen_for_name()
            print("Landmark:", name)

        elif choice == "2":
            cmd = listen_for_command()
            print("Command:", cmd)

        elif choice == "3":
            break