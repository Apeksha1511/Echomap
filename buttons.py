# buttons.py — Simple reliable button handler

import RPi.GPIO as GPIO
import time
import os
from config import BTN_TAG, BTN_CYCLE, BTN_NAVIGATE, LONG_PRESS_SECS

DEBOUNCE = 0.03

def setup_buttons():
    GPIO.setup(BTN_TAG,      GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(BTN_CYCLE,    GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(BTN_NAVIGATE, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    print("[BUTTONS] Ready.")

def is_pressed(pin):
    return GPIO.input(pin) == GPIO.LOW

def check_btn(pin):
    """
    Non-blocking. Returns 'short', 'long', or None.
    """
    if not is_pressed(pin):
        return None
    time.sleep(DEBOUNCE)
    if not is_pressed(pin):
        return None

    press_start = time.time()
    long_fired  = False

    while is_pressed(pin):
        if time.time() - press_start >= LONG_PRESS_SECS:
            long_fired = True
            break
        time.sleep(0.02)

    # Wait for release
    t = time.time()
    while is_pressed(pin):
        if time.time() - t > 0.5:
            break
        time.sleep(0.02)

    return 'long' if long_fired else 'short'

def beep(times=1):
    for _ in range(times):
        print("\a", end="", flush=True)
        time.sleep(0.1)

def shutdown(speak):
    speak("Shutting down. Goodbye.")
    time.sleep(1.5)
    GPIO.cleanup()
    os.system("sudo shutdown -h now")
