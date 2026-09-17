import time
from datetime import datetime
from collections import deque
import json
import RPi.GPIO as GPIO

from sensors import setup_sensors, read_all_sensors, is_stuck, reset_timeout_streak
from buttons import setup_buttons, check_btn, beep
from voice import speak
from navigation import get_nav_instruction, reset_dynamic_history
from mapping import OccupancyGrid
from database import init_db, get_landmark, log_session, get_recent_sessions,save_landmark
from landmarks import (
    register_landmark,
    ask_current_location,
    ask_destination,
)
from speech_input import (
    listen_for_name,
    listen_for_command,
    listen_for_mapping_command
)
from path_planner import (
    astar,
    path_to_instructions,
    instruction_to_speech,
    route_metrics,
)
from progression import level_for_confidence
from confidence_model import ConfidenceModel
from config import (
    STOP_REPLAN_SECS, TURN_WAIT_SECONDS, MAX_TURN_ATTEMPTS,
    DYNAMIC_REANNOUNCE_SECS, PROLONGED_BLOCK_SECS,
)

# -------------------------------------------------------
# Modes
# -------------------------------------------------------

DISCOVERY = "DISCOVERY"
MAPPING = "MAPPING"
DESTINATION_SELECT = "DESTINATION_SELECT"
NAVIGATION = "NAVIGATION"

mode = DISCOVERY

grid = OccupancyGrid()
confidence_model = ConfidenceModel()

last_voice = ""
last_voice_time = 0

VOICE_INTERVAL = 2

# Faster re-announce interval used ONLY for STOP / PATH_BLOCKED, so the
# person gets frequent audio confirmation while stopped instead of waiting
# up to VOICE_INTERVAL seconds between repeats.
STOP_VOICE_INTERVAL = 0.75
_last_stop_voice = ""
_last_stop_voice_time = 0

# How many consecutive matching readings are required AFTER the turn-wait
# window ends before the system commits to a decision (turn again, slow,
# or clear). A single reading taken right at the deadline can still reflect
# a turn that isn't fully finished yet — requiring two agreeing readings in
# a row avoids reacting to one stale/mid-turn sample.
POST_TURN_CONFIRMATIONS = 2
# -------- Button Debounce --------
from buttons import (
    check_btn,
    beep,
    shutdown,
    BTN_TAG,
    BTN_CYCLE,
    BTN_NAVIGATE
)
last_press_time = {
    BTN_TAG: 0,
    BTN_CYCLE: 0,
    BTN_NAVIGATE: 0
}

DEBOUNCE_TIME = 0.35 
STOP_DIST = 30  # seconds
# -------------------------------------------------------
# Speak without repeating continuously
# -------------------------------------------------------


def speak_once(message):
    """Standard throttle (VOICE_INTERVAL) — used for all normal
    announcements: CLEAR, SLOW, TURN_LEFT/RIGHT, MOVING_CENTER, etc."""
    global last_voice, last_voice_time

    if not message:
        return

    now = time.time()

    if message != last_voice or now - last_voice_time > VOICE_INTERVAL:
        speak(message)
        last_voice = message
        last_voice_time = now


def speak_stop(message):
    """Faster throttle (STOP_VOICE_INTERVAL) — used ONLY for STOP and
    PATH_BLOCKED messages, so the person hears frequent confirmation that
    the system is still tracking them while they're stopped/blocked."""
    global _last_stop_voice, _last_stop_voice_time

    if not message:
        return

    now = time.time()

    if message != _last_stop_voice or now - _last_stop_voice_time > STOP_VOICE_INTERVAL:
        speak(message)
        _last_stop_voice = message
        _last_stop_voice_time = now


# -------------------------------------------------------
# Discovery Mode — turn / wait / rescan state machine
# -------------------------------------------------------
#
# Hard safety rules encoded here (see EchoMap spec §9-§20):
#  - Never say "Path clear. Walk." immediately after a turn instruction.
#  - Never assume the person actually turned or actually walked.
#  - STOP always preempts the turn-wait timer, every tick.
#  - Bounded retries: after MAX_TURN_ATTEMPTS failed turns in a row,
#    declare the path blocked instead of oscillating forever.
#  - While waiting for a person to complete a physical turn, sensor noise
#    from the turn itself (misread as MOVING_CENTER, or brief side-sensor
#    echo timeouts) must not cancel the wait — see reset_dynamic_history()
#    and reset_timeout_streak() in _issue_turn().
#  - After the wait window ends, require POST_TURN_CONFIRMATIONS consecutive
#    matching readings before acting — a single sample right at the deadline
#    can still reflect an unfinished turn.


class DiscoveryController:
    def __init__(self):
        self.awaiting_rescan = False
        self.rescan_deadline = 0.0
        self.turn_history = deque(maxlen=MAX_TURN_ATTEMPTS)
        self.in_dynamic_wait = False
        self.dynamic_wait_deadline = 0.0
        self.blocked_since = None
        self.confirming = False
        self.confirm_action = None
        self.confirm_votes = 0

    def reset(self):
        self.awaiting_rescan = False
        self.turn_history.clear()
        self.in_dynamic_wait = False
        self.blocked_since = None
        self.confirming = False
        self.confirm_action = None
        self.confirm_votes = 0

    def tick(self, left, center, right):
        if is_stuck('CENTER') or is_stuck('LEFT') or is_stuck('RIGHT'):
             speak_once('Sensor unavailable. Please check EchoMap.')
             return
        action, message = get_nav_instruction(left, center, right)
        print(f'[DISCOVERY] L={left:.1f} C={center:.1f} R={right:.1f} -> {action}')
        now = time.time()

        if action == 'STOP':
            self.reset()
            speak_stop('Stop.Take a step back.')
            return

        if action == 'PATH_BLOCKED':
          if self.blocked_since is None:
            self.blocked_since = now
            speak_stop('Stop. Path blocked.')
          elif now - self.blocked_since > PROLONGED_BLOCK_SECS:
            speak_stop('No safe path detected. Please turn around and try another direction.')
          else:
            speak_stop('Stop. Path blocked.')
          self.awaiting_rescan = False
          self.confirming = False
          self.turn_history.clear()
          return

        # Shield the turn-wait window: while we're still giving the person
        # time to physically complete a turn, don't let sensor noise from
        # the turn itself (misread as MOVING_CENTER) cancel the wait.
        # STOP/PATH_BLOCKED above still preempt immediately every tick.
        if self.awaiting_rescan and now < self.rescan_deadline:
            return

        if action == 'MOVING_CENTER':
            # Cancel any pending turn-wait/confirmation — a moving obstacle
            # takes priority.
            self.awaiting_rescan = False
            self.confirming = False
            self.turn_history.clear()
            if not self.in_dynamic_wait:
                # First detection: announce once, then go quiet.
                self.in_dynamic_wait = True
                speak('Moving obstacle ahead. Please wait.')
                self.dynamic_wait_deadline = now + DYNAMIC_REANNOUNCE_SECS
            elif now >= self.dynamic_wait_deadline:
                # Still blocking after the wait window — re-announce once.
                speak('Moving obstacle ahead. Please wait.')
                self.dynamic_wait_deadline = now + DYNAMIC_REANNOUNCE_SECS
            # else: silently keep waiting, no repeat.
            return

        # Obstacle is gone (classifier no longer reports MOVING_CENTER).
        self.in_dynamic_wait = False

        if self.awaiting_rescan:
            # Deadline has passed (guaranteed by the shield above). Don't
            # act on this single reading yet — require POST_TURN_CONFIRMATIONS
            # consecutive ticks agreeing on the same action first.
            if not self.confirming:
                self.confirming = True
                self.confirm_action = action
                self.confirm_votes = 1
                return

            if action == self.confirm_action:
                self.confirm_votes += 1
            else:
                # Reading changed since the last check — restart the count
                # against the new action rather than trusting a flip.
                self.confirm_action = action
                self.confirm_votes = 1

            if self.confirm_votes < POST_TURN_CONFIRMATIONS:
                return

            # Two consecutive agreeing readings — commit to this decision.
            self.awaiting_rescan = False
            self.confirming = False
            if action in ('TURN_LEFT', 'TURN_RIGHT'):
                self._issue_turn(action, message)
            elif action == 'SLOW':
                self.turn_history.clear()
                speak_once('Slow down.')
            else:  # CLEAR
                self.turn_history.clear()
                speak_once('Path clear. Walk.')
            return

        if action in ('TURN_LEFT', 'TURN_RIGHT'):
            self._issue_turn(action, message)
        elif action == 'SLOW':
            speak_once('Slow down.')
        elif action == 'CLEAR':
            self.turn_history.clear()
            speak_once('Path clear. Walk.')

    def _issue_turn(self, action, message):
        self.turn_history.append('left' if action == 'TURN_LEFT' else 'right')
        if len(self.turn_history) >= self.turn_history.maxlen:
            self.reset()
            speak_once('Path blocked. Please wait.')
            return
        speak_once(message)
        # Don't carry pre-turn motion/timeout history into the rescan window —
        # the person is about to physically rotate, which naturally causes
        # fast sensor changes and occasional echo timeouts that must not be
        # misread as a moving obstacle or a faulty sensor.
        reset_dynamic_history()
        reset_timeout_streak()
        self.awaiting_rescan = True
        self.confirming = False
        self.confirm_action = None
        self.confirm_votes = 0
        self.rescan_deadline = time.time() + TURN_WAIT_SECONDS
        print(f'[DISCOVERY] Turn issued ({action}); waiting {TURN_WAIT_SECONDS}s before rescan.')


discovery_controller = DiscoveryController()


def discovery_loop():
    left, center, right = read_all_sensors()
    grid.update(left, center, right)
    discovery_controller.tick(left, center, right)


# -------------------------------------------------------
# Mapping Mode
# -------------------------------------------------------
# Mapping is for a sighted person; they don't need turn-by-turn guidance,
# only safety-relevant announcements. Position/heading here change ONLY
# from explicit STT commands (see mapping.apply_mapping_command) — never
# from this ambient obstacle scan.

_MAPPING_SAFETY_ACTIONS = {'STOP', 'PATH_BLOCKED', 'MOVING_CENTER'}
_MAPPING_MESSAGES = {
    'STOP': 'Stop.',
    'PATH_BLOCKED': 'Stop. Path blocked.',
    'MOVING_CENTER': 'Moving obstacle ahead. Please wait.',
}


def mapping_loop():
    """
    Mapping Mode with Ultrasonic SLAM.
    Runs continuously while Mapping Mode is active.
    """

    # Read all three ultrasonic sensors
    left, center, right = read_all_sensors()

    # ---------------- Occupancy Grid Mapping ----------------
    grid.update(left, center, right)

    # ---------------- Ultrasonic SLAM ----------------
    grid.update_slam(left, center, right)

    # ---------------- Loop Closure ----------------
    matched = grid.loop_closure()

    if matched:
        speak(f"Landmark recognized. {matched}. Position corrected.")

    # Optional debug output
    grid.print_map()

    time.sleep(0.05)
    action, _ = get_nav_instruction(left, center, right)
    if action in _MAPPING_SAFETY_ACTIONS:
        speak_once(_MAPPING_MESSAGES[action])

    # -------- Button 1 Short → Landmark --------
    press1 = check_btn(16)

    if press1 == "short":
        register_landmark(speak, grid)

    # -------- Button 1 Long → Exit Mapping -----
    if press1 == "long":
        speak("Discovery mode.")
        discovery_controller.reset()
        mode = DISCOVERY
        return

    # -------- Button 2 Short → Listen for Command -----
    press2 = check_btn(20)

    if press2 == "short":
        speak("Say movement command.")
        command = listen_for_command(timeout=5)
        if command:
            response = grid.apply_mapping_command(command)
            speak(response)
        else:
            speak("Command not detected.")


# -------------------------------------------------------
# Navigation Mode
# -------------------------------------------------------


def _plan_route(goal_x, goal_y):
    """
    Plan A* path and convert it into simple navigation instructions.
    Returns:
    path, speech_steps
    """

    start = (grid.pos_x, grid.pos_y)
    goal = (goal_x, goal_y)

    print(f"[A*] Planning from {start} to {goal}")

    path = astar(grid.grid, start, goal)

    if path is None or len(path) < 2:
        return None, None

    speech_steps = []
    current_heading = float(grid.heading)

    direction = {
        (1, 0): 90,     # right/east
        (-1, 0): 270,   # left/west
        (0, -1): 0,     # forward/north
        (0, 1): 180     # backward/south
    }

    for i in range(1, len(path)):
        x1, y1 = path[i - 1]
        x2, y2 = path[i]

        dx = x2 - x1
        dy = y2 - y1

        if (dx, dy) not in direction:
            continue

        target_heading = direction[(dx, dy)]

        diff = (target_heading - current_heading) % 360

        if diff == 90:
            speech_steps.append("Turn right.")
        elif diff == 270:
            speech_steps.append("Turn left.")
        elif diff == 180:
            speech_steps.append("Turn around.")

    # Every A* cell represents one forward movement.
        speech_steps.append("Walk forward one step.")

        current_heading = target_heading

    print(f"[A*] Path cells = {len(path)}")
    print(f"[A*] Speech instructions = {len(speech_steps)}")

    return path, speech_steps


def _predict_confidence(errors, walk_steps, complexity, dynamic_obstacles):
    """Feed recent session history + this session into the confidence model.
    This is a heuristic recurrent scorer, not a trained/validated LSTM —
    see confidence_model.py."""
    recent = get_recent_sessions(5)
    seq = [
        {
            'error_rate': r[7] or 0,
            'reaction_time': r[6] or 0,
            'route_complexity': r[8] or 0,
            'success': r[4] or 0,
            'dynamic_obstacles': r[10] or 0,
        }
        for r in reversed(recent)
    ]
    seq.append({
        'error_rate': errors / max(walk_steps, 1),
        'reaction_time': 0,  # not yet measured on-device
        'route_complexity': complexity,
        'success': 1,
        'dynamic_obstacles': dynamic_obstacles,
    })
    return confidence_model.predict(seq)


def navigation_loop(source_name, destination_name):
    """
    Navigate from a selected source landmark to a destination landmark
    using A* + ultrasonic safety sensing.

    Navigation pose is controlled by the planned A* route.
    Ultrasonic sensors are used for real-time obstacle detection.
    """

    start_time = datetime.now().isoformat()

    # ==========================================================
    # Load SOURCE landmark
    # ==========================================================
    source_row = get_landmark(source_name)

    if source_row is None:
        speak("Source location not found.")
        return

    sx, sy, sheading, sconf, sscan, smap = source_row

    # Restore map and pose from source landmark
    try:
        grid.from_dict(json.loads(smap))
    except Exception as e:
        print(f"[NAV] Failed to restore map: {e}")
        speak("Unable to load source map.")
        return

    grid.pos_x = sx
    grid.pos_y = sy
    grid.heading = sheading
    grid.pose_confidence = sconf

    # ==========================================================
    # Load DESTINATION landmark
    # ==========================================================
    dest_row = get_landmark(destination_name)

    if dest_row is None:
        speak("Destination not found.")
        return

    goal_x, goal_y, goal_heading, goal_conf, goal_scan, goal_map = dest_row

    print(
        f"[START] ({grid.pos_x}, {grid.pos_y}) "
        f"Heading={grid.heading}"
    )

    print(
        f"[GOAL] ({goal_x}, {goal_y}) "
        f"Heading={goal_heading}"
    )

    # ==========================================================
    # A* ROUTE PLANNING
    # ==========================================================
    instructions, speech_steps = _plan_route(
        goal_x,
        goal_y
    )

    if instructions is None or speech_steps is None:
        speak("No route found.")
        return

    print(f"[A*] Path cells = {len(instructions)}")
    print(f"[A*] Speech instructions = {len(speech_steps)}")

    # ==========================================================
    # ROUTE METRICS
    # ==========================================================
    walk_steps = sum(
        1
        for step in speech_steps
        if step == "Walk forward one step."
    )

    turns = sum(
        1
        for step in speech_steps
        if step.startswith("Turn")
    )

    complexity = round(
        turns * 0.5 + walk_steps / 20.0,
        2
    )

    # ==========================================================
    # NAVIGATION METRICS
    # ==========================================================
    dynamic_obstacles = 0
    errors = 0

    waiting_dynamic = False
    stop_since = None

    step_idx = 0

    # ==========================================================
    # EXECUTE ROUTE
    # ==========================================================
    while step_idx < len(speech_steps):

        current_step = speech_steps[step_idx]

        print(
            f"[NAV] Step {step_idx + 1}/"
            f"{len(speech_steps)}: {current_step}"
        )

        finished = False

        while not finished:

            # --------------------------------------------------
            # Read ultrasonic sensors
            # --------------------------------------------------
            left, center, right = read_all_sensors()

            print(
                f"[SENSORS] "
                f"L={left} cm "
                f"C={center} cm "
                f"R={right} cm"
            )

            # --------------------------------------------------
            # Update occupancy grid ONLY
            #
            # Do NOT call update_slam() here.
            # Do NOT call loop_closure() here.
            #
            # Navigation pose is controlled by the A* route.
            # --------------------------------------------------
            

            # --------------------------------------------------
            # Real-time ultrasonic safety
            # --------------------------------------------------
            grid.update(left,center,right)
            action, message = get_nav_instruction(
                left,
                center,
                right
            )

            # ==================================================
            # MOVING OBSTACLE
            # ==================================================
            if action == "MOVING_CENTER":

                if not waiting_dynamic:
                    dynamic_obstacles += 1
                    waiting_dynamic = True

                    print(
                        "[NAV] Dynamic obstacle detected."
                    )

                speak_once(message)

            # ==================================================
            # STOP / BLOCKED PATH
            # ==================================================
            elif action in ("STOP", "PATH_BLOCKED"):

                if stop_since is None:
                    stop_since = time.time()
                    errors += 1

                speak_once(message)

                print(
                    f"[NAV] Path blocked. "
                    f"Waiting {STOP_REPLAN_SECS}s before replanning."
                )

                # ------------------------------------------------
                # Replan after obstacle remains for too long
                # ------------------------------------------------
                if time.time() - stop_since > STOP_REPLAN_SECS:

                    speak("Replanning route.")

                    print(
                        f"[A*] Replanning from "
                        f"({grid.pos_x}, {grid.pos_y}) "
                        f"to ({goal_x}, {goal_y})"
                    )

                    new_instructions, new_speech_steps = _plan_route(
                        goal_x,
                        goal_y
                    )

                    if (
                        new_instructions is None
                        or new_speech_steps is None
                    ):
                        speak("No alternate route found.")

                        stop_since = time.time()

                    else:
                        # ----------------------------------------
                        # Replace current route
                        # ----------------------------------------
                        instructions = new_instructions
                        speech_steps = new_speech_steps

                        # Recalculate metrics
                        walk_steps = sum(
                            1
                            for step in speech_steps
                            if step == "Walk forward one step."
                        )

                        turns = sum(
                            1
                            for step in speech_steps
                            if step.startswith("Turn")
                        )

                        complexity = round(
                            turns * 0.5 + walk_steps / 20.0,
                            2
                        )

                        # Start the new route from the beginning
                        step_idx = -1

                        stop_since = None
                        waiting_dynamic = False

                        print(
                            "[NAV] New route accepted."
                        )

                        finished = True

            # ==================================================
            # PATH CLEAR / SAFE
            # ==================================================
            else:
                if "walk" in current_step.lower() and center < STOP_DIST:
                    speak_once("Stop. Obstacle ahead.")
                    finished = False
                    time.sleep(0.05)
                    continue

                speak(current_step)
                if waiting_dynamic:

                    speak_once(
                        "Path clear. Continuing."
                    )

                    waiting_dynamic = False

                stop_since = None

                # ==================================================
                # UPDATE NAVIGATION POSE
                #
                # Only update the pose AFTER the ultrasonic
                # safety check says it is safe to continue.
                # ==================================================

                step = current_step.lower()

                if "turn around" in step:

                    grid.turn(180)

                    print(
                        f"[NAV] Turn around -> "
                        f"Heading={grid.heading}"
                    )

                elif "turn left" in step:

                    grid.turn(-90)

                    print(
                        f"[NAV] Turn left -> "
                        f"Heading={grid.heading}"
                    )

                elif "turn right" in step:

                    grid.turn(90)

                    print(
                        f"[NAV] Turn right -> "
                        f"Heading={grid.heading}"
                    )

                elif "walk" in step:

                    old_x = grid.pos_x
                    old_y = grid.pos_y

                    grid.advance_steps(1)

                    print(
                        f"[NAV] Walk -> "
                        f"({old_x}, {old_y}) "
                        f"to "
                        f"({grid.pos_x}, {grid.pos_y})"
                    )

                print(
                    f"[POSE] "
                    f"X={grid.pos_x}, "
                    f"Y={grid.pos_y}, "
                    f"Heading={grid.heading:.0f}°"
                )

                finished = True

            time.sleep(0.05)

        # Move to next instruction
        step_idx += 1

    # ==========================================================
    # VERIFY DESTINATION
    # ==========================================================
    current_pose = (
        grid.pos_x,
        grid.pos_y
    )

    goal_pose = (
        goal_x,
        goal_y
    )

    print(
        f"[NAV] Final pose = {current_pose} "
        f"Goal = {goal_pose} "
        f"Heading={grid.heading}"
    )

    # ==========================================================
    # DESTINATION CONFIRMED
    # ==========================================================
    if current_pose == goal_pose:

        speak("Destination reached.")

        print(
            "[NAV] Destination confirmed."
        )

        success = True

    # ==========================================================
    # DESTINATION NOT CONFIRMED
    # ==========================================================
    else:

        print(
            f"[NAV] Destination NOT confirmed. "
            f"Current={current_pose}, "
            f"Goal={goal_pose}"
        )

        speak(
            "Route completed. "
            "Destination not confirmed."
        )

        success = False

    # ==========================================================
    # CONFIDENCE CALCULATION
    # ==========================================================
    confidence = _predict_confidence(
        errors,
        walk_steps,
        complexity,
        dynamic_obstacles
    )

    # ==========================================================
    # LOG NAVIGATION SESSION
    # ==========================================================
    confidence = log_session(
        start_time=start_time,
        destination=destination_name,
        steps=walk_steps,
        errors=errors,
        success=success,
        route_complexity=complexity,
        route_steps=walk_steps,
        turns=turns,
        dynamic_obstacles=dynamic_obstacles,
        level=level_for_confidence(confidence),
        confidence=confidence
    )

    # ==========================================================
    # FINAL CONFIDENCE ANNOUNCEMENT
    # ==========================================================
    speak(
        f"Confidence score "
        f"{int(confidence * 100)} percent."
    )

    print(
        f"[NAV] Navigation complete. "
        f"Success={success}, "
        f"Confidence={confidence:.2f}"
    )
# -------------------------------------------------------
# Destination Selection
# -------------------------------------------------------

def destination_selection():
    """
    Discovery Mode destination workflow:
    1. Ask source location.
    2. Ask destination.
    """

    # -------- Source Selection --------
    speak("Choose source location.")

    current_location = ask_current_location(speak, grid)

    if current_location is None:
        speak("No source selected.")
        return None, None

    # -------- Destination Selection --------
    speak("Choose destination.")

    destination = ask_destination(speak, current_location)

    if destination is None:
        speak("No destination selected.")
        return None, None

    speak(f"Navigating from {current_location} to {destination}.")
    return current_location, destination
# -------------------------------------------------------
# Startup
# -------------------------------------------------------


def startup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    setup_sensors()
    setup_buttons()
    init_db()

    speak("Echo Map started.")
    speak("Discovery mode ready.")


# -------------------------------------------------------
# Main Loop
# -------------------------------------------------------
def button_pressed(button_pin, press_type):
    """
    Returns True only once for each physical button press.
    Prevents repeated triggers while the button is held.
    """
    global last_press_time

    press = check_btn(button_pin)

    if press != press_type:
        return False

    now = time.time()

    if now - last_press_time[button_pin] < DEBOUNCE_TIME:
        return False

    last_press_time[button_pin] = now
    return True

def main():
    global mode

    startup()

    try:
        while True:

            # Read buttons once every loop
            press1 = check_btn(BTN_TAG)
            press2 = check_btn(BTN_CYCLE)

            # ==========================================
            # Button 1 Long Press → Toggle Modes
            # ==========================================
            if press1 == "long":
                beep(2)

                if mode == DISCOVERY:
                    speak("Mapping mode activated.")
                    discovery_controller.reset()
                    mode = MAPPING
                else:
                    speak("Discovery mode activated.")
                    discovery_controller.reset()
                    mode = DISCOVERY

                time.sleep(0.3)
                continue

            # ==========================================
            # DISCOVERY MODE
            # ==========================================
            if mode == DISCOVERY:

                # Button 2 → Source then Destination selection
                if press2 == "short":
                    print("[BUTTON] Button 2 detected.")
                    beep()
                    mode = DESTINATION_SELECT
                    continue

                discovery_loop()

            # ==========================================
            # DESTINATION SELECTION MODE
            # ==========================================
            elif mode == DESTINATION_SELECT:

                source, destination = destination_selection()

                if source is not None and destination is not None:

        # Store the selected source and destination
                    discovery_controller.current_location = source
                    discovery_controller.destination = destination

                    print(f"[ROUTE] Source: {source}  Destination: {destination}")
                    speak(f"Route ready from {source} to {destination}.")
                    navigation_loop(source,destination)

                discovery_controller.reset()
                speak("Discovery mode.")
                mode = DISCOVERY

            # ==========================================
            # MAPPING MODE
            # ==========================================
            elif mode == MAPPING:

                mapping_loop()

                # Button 1 Short → Save landmark
                if press1 == "short":
                    beep()
                    register_landmark(speak, grid)
                    time.sleep(0.25)

                # Button 2 Short → Turn command
                elif press2 == "short":
                    beep()
                    speak("Say command.")

                    command = listen_for_mapping_command(timeout=5)

                    if command:
                        response = grid.apply_mapping_command(command)
                        speak(response)
                    else:
                        speak("Turn left or turn right only.")

                    time.sleep(0.25)

            time.sleep(0.05)

    except KeyboardInterrupt:
        pass

    finally:
        GPIO.cleanup()


if __name__ == "__main__":
    main()