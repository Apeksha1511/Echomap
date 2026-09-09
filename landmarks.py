# landmarks.py — Landmark save and select using buttons

import time
import json
from buttons import check_btn, beep, shutdown, BTN_TAG, BTN_CYCLE, BTN_NAVIGATE
from database import save_landmark, get_landmark, list_landmarks
from mapping  import OccupancyGrid

_counter = [0]

def save_current_location(speak, grid):
    """Button 1 — save current position with auto name."""
    _counter[0] += 1
    name = f"location{_counter[0]}"
    save_landmark(name, grid.pos_x, grid.pos_y, grid)
    beep(2)
    speak(f"Saved. {name}.")
    print(f"[SAVE] '{name}' at ({grid.pos_x},{grid.pos_y})")

def ask_current_location(speak, grid):
    """
    Startup: ask user where they are.
    Returns location name string or None.
    """
    lms = list_landmarks()
    if not lms:
        speak("No saved locations yet. Walk around first then press Button 1 to save locations.")
        return None

    speak(f"Where are you now? I have {len(lms)} saved locations.")
    speak("Press Button 2 to cycle. Press Button 3 to confirm.")
    time.sleep(0.3)

    idx = 0
    speak(lms[0][0])

    while True:
        p2 = check_btn(BTN_CYCLE)
        p3 = check_btn(BTN_NAVIGATE)

        if p2 == 'short':
            beep(1)
            idx = (idx + 1) % len(lms)
            speak(lms[idx][0])

        if p3 == 'short':
            beep(2)
            name, gx, gy = lms[idx]
            row = get_landmark(name)
            if row:
                _, _, map_json = row
                grid.from_dict(json.loads(map_json))
            else:
                grid.pos_x, grid.pos_y = gx, gy
            speak(f"Starting from {name}.")
            print(f"[START] Position = '{name}' ({gx},{gy})")
            return name

        if p3 == 'long':
            shutdown(speak)

        time.sleep(0.05)

def ask_destination(speak, current_loc):
    """
    Ask user where they want to go.
    Returns destination name string or None.
    """
    lms   = list_landmarks()
    dests = [(n, gx, gy) for n, gx, gy in lms if n != current_loc]

    if not dests:
        speak("No other saved locations to navigate to.")
        return None

    speak(f"Where to go? {len(dests)} destinations available.")
    speak("Press Button 2 to cycle. Press Button 3 to confirm.")
    time.sleep(0.3)

    idx = 0
    speak(dests[0][0])

    while True:
        p2 = check_btn(BTN_CYCLE)
        p3 = check_btn(BTN_NAVIGATE)

        if p2 == 'short':
            beep(1)
            idx = (idx + 1) % len(dests)
            speak(dests[idx][0])

        if p3 == 'short':
            beep(2)
            name = dests[idx][0]
            speak(f"Going to {name}.")
            return name

        if p3 == 'long':
            shutdown(speak)

        time.sleep(0.05)
