# sensors.py — filtered, cross-talk-aware HC-SR04 scanning
import time
import statistics
import RPi.GPIO as GPIO
from config import SENSORS, MAX_SENSOR_CM, SENSOR_GAP_SEC, SENSOR_TIMEOUT_SEC, FILTER_WINDOW

_history = {name: [] for name in SENSORS}


def setup_sensors():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for pins in SENSORS.values():
        GPIO.setup(pins['trig'], GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(pins['echo'], GPIO.IN)
    time.sleep(0.2)
    print('[SENSORS] All 3 sensors ready.')


def get_distance(trig, echo):
    # Ensure a clean low state before each acoustic burst.
    GPIO.output(trig, GPIO.LOW)
    time.sleep(0.0002)
    GPIO.output(trig, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig, GPIO.LOW)

    deadline = time.monotonic() + SENSOR_TIMEOUT_SEC
    while GPIO.input(echo) == GPIO.LOW:
        if time.monotonic() >= deadline:
            return 999.0
    pulse_start = time.monotonic()

    deadline = pulse_start + SENSOR_TIMEOUT_SEC
    while GPIO.input(echo) == GPIO.HIGH:
        if time.monotonic() >= deadline:
            return 999.0
    pulse_end = time.monotonic()

    distance = (pulse_end - pulse_start) * 17150.0
    if distance <= 1 or distance > MAX_SENSOR_CM:
        return 999.0
    return round(distance, 1)


def _filtered(name, value):
    # Time-domain median filter removes single-frame 800–1200 cm spikes.
    if value < MAX_SENSOR_CM:
        _history[name].append(value)
        _history[name] = _history[name][-FILTER_WINDOW:]
    h = _history[name]
    if not h:
        return 999.0
    return round(statistics.median(h), 1)


def read_all_sensors():
    values = []
    for name in ('LEFT', 'CENTER', 'RIGHT'):
        pins = SENSORS[name]
        raw = get_distance(pins['trig'], pins['echo'])
        values.append(_filtered(name, raw))
        time.sleep(SENSOR_GAP_SEC)
    return tuple(values)
