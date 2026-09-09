# calibration.py — step length calibration helper
from voice import speak

def calibrate_step_length(default=65.0):
    speak(f'Step calibration. Current default is {int(default)} centimeters.')
    print(f'[CALIBRATION] Enter measured step length in cm (Enter={int(default)}): ',end='',flush=True)
    try: value=float(input().strip() or default)
    except Exception: value=default
    value=max(40.0,min(100.0,value)); speak(f'Step length set to {int(value)} centimeters.'); return value
