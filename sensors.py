# sensors.py — filtered, cross-talk-aware HC-SR04 scanning
import time
import statistics
import RPi.GPIO as GPIO
from config import (
    SENSORS, MAX_SENSOR_CM, SENSOR_GAP_SEC, SENSOR_TIMEOUT_SEC, FILTER_WINDOW,
    DEBUG_SENSOR_LOG, SENSOR_LOG_INTERVAL,
)

_history = {name: [] for name in SENSORS}
_last_debug_time = 0.0

# Consecutive *genuine* hardware timeouts (no echo pulse at all) before we
# stop trusting a sensor. This is separate from "999.0 = nothing within
# range", which is a perfectly valid reading and must never count here.
_timeout_streak = {name: 0 for name in SENSORS}
TIMEOUT_STUCK_THRESHOLD = 8

# Wall-clock time each sensor last returned a genuinely valid reading (not
# a None timeout). Unlike _timeout_streak, this is NEVER cleared by
# reset_timeout_streak() — it's a backstop that catches a sensor stuck for
# too long in real time, even across several turn-issuances that keep
# wiping the streak counter. Without this, repeated turns in a tight area
# could reset the streak counter faster than it ever reaches
# TIMEOUT_STUCK_THRESHOLD, letting a genuinely failing sensor silently keep
# returning a stale cached distance forever with no warning.
_last_success_time = {name: time.monotonic() for name in SENSORS}
STUCK_TIME_SECS = 3.0


def setup_sensors():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for pins in SENSORS.values():
        GPIO.setup(pins['trig'], GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(pins['echo'], GPIO.IN)
    time.sleep(0.2)
    print('[SENSORS] All 3 sensors ready.')


def get_distance(trig, echo):
    """Returns:
    - a float distance in cm for a successful measurement (999.0 if the
      object is beyond MAX_SENSOR_CM — this is a VALID 'nothing nearby'
      reading, common and expected in open spaces)
    - 999.0 if the echo pulse never started at all (no object close enough
      to reflect back before the timeout) — also valid, not a fault
    - None ONLY if an echo pulse genuinely started and then got stuck —
      this is the real hardware-glitch signal.
    """
    GPIO.output(trig, GPIO.LOW)
    time.sleep(0.0002)
    GPIO.output(trig, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig, GPIO.LOW)

    deadline = time.monotonic() + SENSOR_TIMEOUT_SEC
    while GPIO.input(echo) == GPIO.LOW:
        if time.monotonic() >= deadline:
            # Echo pulse never started — indistinguishable from "nothing in
            # range," which is the normal case for open corridors/sides.
            # Do NOT treat this as a fault.
            return 999.0
    pulse_start = time.monotonic()

    deadline = pulse_start + SENSOR_TIMEOUT_SEC
    while GPIO.input(echo) == GPIO.HIGH:
        if time.monotonic() >= deadline:
            # Echo pulse DID start (something was detected), but the pin
            # never returned low. This is a genuine sensor glitch.
            return None
    pulse_end = time.monotonic()

    distance = (pulse_end - pulse_start) * 17150.0
    if distance <= 1 or distance > MAX_SENSOR_CM:
        return 999.0
    return round(distance, 1)

def _filtered(name, value):
    if value is None:
        # Genuine echo-stuck timeout.
        _timeout_streak[name] += 1

        # Do not add a bad reading to the distance history.
        # Return 999 only after repeated failures.
        if _timeout_streak[name] >= TIMEOUT_STUCK_THRESHOLD:
            return 999.0

        # Keep the previous valid value temporarily during a
        # very short transient timeout.
        h = _history[name]
        if h:
            return round(statistics.median(h), 1)
        return 999.0

    # Successful reading
    _timeout_streak[name] = 0
    _last_success_time[name] = time.monotonic()

    # IMPORTANT:
    # 999 means no obstacle within measurable range.
    # It must clear stale obstacle history instead of leaving
    # the previous close distance alive.
    if value >= MAX_SENSOR_CM:
        _history[name].clear()
        return 999.0

    _history[name].append(value)
    _history[name] = _history[name][-FILTER_WINDOW:]

    return round(statistics.median(_history[name]), 1)


def is_stuck(name):
    if _timeout_streak[name] >= TIMEOUT_STUCK_THRESHOLD:
        return True
    # Time-based backstop: catches a sensor that's genuinely failing for
    # too long even if reset_timeout_streak() keeps wiping the streak
    # counter (e.g. during a run of frequent turn-issuances).
    return (time.monotonic() - _last_success_time[name]) > STUCK_TIME_SECS


def reset_timeout_streak(name=None):
    """Clear the SHORT-TERM stuck-sensor counter for one sensor or all
    three. Called by main.py when a turn is issued: brief echo timeouts on
    the side (and sometimes center) sensors while the person is physically
    rotating are expected and shouldn't immediately trip the streak-based
    check. This does NOT reset _last_success_time, so it can no longer mask
    a sensor that's genuinely failing for several real seconds — that's
    still caught by the time-based check in is_stuck()."""
    names = ('LEFT', 'CENTER', 'RIGHT') if name is None else (name,)
    for n in names:
        _timeout_streak[n] = 0


def read_all_sensors():
    values = []
    for name in ('LEFT', 'CENTER', 'RIGHT'):
        pins = SENSORS[name]
        raw = get_distance(pins['trig'], pins['echo'])
        values.append(_filtered(name, raw))
        time.sleep(SENSOR_GAP_SEC)

    return tuple(values)