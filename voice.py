# voice.py — offline TTS with automatic ALSA device fallback
import subprocess, tempfile, os, threading
from config import AUDIO_DEVICE, ESPEAK_VOICE, ESPEAK_SPEED, ESPEAK_VOL
_lock=threading.Lock()

def _play(wav=None):
    cmd=['aplay']
    if AUDIO_DEVICE and AUDIO_DEVICE != 'default': cmd += ['-D',AUDIO_DEVICE]
    if wav: cmd.append(wav)
    return subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False).returncode

def speak(text):
    text=str(text).strip()
    if not text: return
    print(f'[VOICE] {text}')
    with _lock:
        try:
            # Prefer eSpeak because it is present in the project's offline setup.
            p=subprocess.Popen(['espeak','-v',ESPEAK_VOICE,'-s',str(ESPEAK_SPEED),'-a',str(ESPEAK_VOL),text,'--stdout'],stdout=subprocess.PIPE,stderr=subprocess.DEVNULL)
            rc=_play_pipe(p.stdout)
            p.wait()
            if rc != 0 and AUDIO_DEVICE != 'default':
                p=subprocess.Popen(['espeak','-v',ESPEAK_VOICE,'-s',str(ESPEAK_SPEED),'-a',str(ESPEAK_VOL),text,'--stdout'],stdout=subprocess.PIPE,stderr=subprocess.DEVNULL)
                _play_pipe(p.stdout, force_default=True); p.wait()
        except Exception as e: print(f'[VOICE ERROR] {e}')

def _play_pipe(pipe,force_default=False):
    cmd=['aplay'] if force_default or AUDIO_DEVICE=='default' else ['aplay','-D',AUDIO_DEVICE]
    return subprocess.run(cmd,stdin=pipe,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,check=False).returncode
