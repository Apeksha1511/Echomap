# speech_input.py — optional offline Vosk landmark naming
import os, json, subprocess, tempfile
MODEL_PATH=os.environ.get('ECHOMAP_VOSK_MODEL', os.path.join(os.path.dirname(__file__),'model','vosk'))

def listen_for_name(timeout=4):
    try:
        from vosk import Model, KaldiRecognizer
        import pyaudio
        if not os.path.isdir(MODEL_PATH): raise RuntimeError('Vosk model directory not found')
        rec=KaldiRecognizer(Model(MODEL_PATH),16000); pa=pyaudio.PyAudio()
        stream=pa.open(format=pyaudio.paInt16,channels=1,rate=16000,input=True,frames_per_buffer=4000); stream.start_stream()
        import time; end=time.time()+timeout; text=''
        while time.time()<end:
            data=stream.read(4000,exception_on_overflow=False)
            if rec.AcceptWaveform(data):
                text=json.loads(rec.Result()).get('text','').strip()
                if text: break
        stream.stop_stream(); stream.close(); pa.terminate(); return text or None
    except Exception as e:
        print(f'[SPEECH] Offline recognizer unavailable: {e}')
        return None
