# landmarks.py — Landmark save and select using buttons

import time
import json
from buttons import check_btn, beep, shutdown, BTN_TAG, BTN_CYCLE, BTN_NAVIGATE
from database import save_landmark, get_landmark, list_landmarks
from mapping  import OccupancyGrid
from speech_input import listen_for_name

_counter = [0]

def save_current_location(speak, grid):
    """Auto-named save, kept for backward compatibility — not used by the
    current Mapping Mode flow, which uses register_landmark() instead."""
    _counter[0] += 1
    name = f"location{_counter[0]}"
    save_landmark(name, grid.pos_x, grid.pos_y, grid)
    beep(2)
    speak(f"Saved. {name}.")
    print(f"[SAVE] '{name}' at ({grid.pos_x},{grid.pos_y})")

def register_landmark(speak, grid):
    """Button 1 short press in Mapping Mode — voice-named landmark save.
    'Say landmark name.' -> STT -> save current grid position/heading -> confirm."""
    speak("Say landmark name.")
    name = listen_for_name(timeout=5)
    if not name:
        speak("Landmark name not detected.")
        return
    # Save landmark with Ultrasonic SLAM pose and scan signature
    save_landmark(name, grid.pos_x, grid.pos_y, grid)

    beep(2)

# Voice confirmation
    speak(f"{name} landmark mapped.")

# Console log for debugging / IEEE demo
    print(
        f"[SLAM] Landmark '{name}' mapped at "
        f"({grid.pos_x}, {grid.pos_y}) | "
        f"Heading={grid.heading:.0f}° | "
        f"Confidence={grid.pose_confidence:.2f}"
    )

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
    speak(lms[idx][0])

    while True:
        p2 = check_btn(BTN_CYCLE)
        p3 = check_btn(BTN_NAVIGATE)

        # Cycle through saved landmarks
        if p2 == "short":
            beep(1)
            idx = (idx + 1) % len(lms)
            speak(lms[idx][0])
            time.sleep(0.25)

        # Confirm current location
        elif p3 == "short":
            beep(2)

            name, gx, gy = lms[idx]
            row = get_landmark(name)

            if row:
                grid_x, grid_y, heading, confidence, scan_signature, map_json = row

                # Restore saved map and SLAM pose
                grid.from_dict(json.loads(map_json))
                grid.pos_x = grid_x
                grid.pos_y = grid_y
                grid.heading = heading
                grid.pose_confidence = confidence

            else:
                grid.pos_x = gx
                grid.pos_y = gy
                grid.heading = 0

            speak(f"Starting from {name}.")
            print(f"[START] Position = '{name}' ({grid.pos_x}, {grid.pos_y})")
            time.sleep(0.25)
            return name

        elif p3 == "long":
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
            time.sleep(0.25)

        if p3 == 'short':
            beep(2)
            name = dests[idx][0]
            speak(f"Going to {name}.")
            time.sleep(0.25)
            return name

        if p3 == 'long':
            shutdown(speak)

        time.sleep(0.05)
