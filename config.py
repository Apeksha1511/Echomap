# EchoMap configuration
import os

# --- Sensor GPIO mapping -------------------------------------------------
# CRITICAL FIX: the previous config had these three entries rotated relative
# to the physically tested wiring (LEFT/CENTER/RIGHT pins were correct but
# attached to the wrong compass label, which silently mirrored obstacle
# avoidance decisions). Do not change these values without re-verifying
# against the physical harness.
SENSORS = {
    'LEFT':   {'trig': 22, 'echo': 25},
    'CENTER': {'trig': 17, 'echo': 27},
    'RIGHT':  {'trig': 23, 'echo': 24},
}

BTN_TAG = 16       # Button 1
BTN_CYCLE = 20     # Button 2
BTN_NAVIGATE = 21  # Button 3

GRID_SIZE = 100
CELL_CM = 10
UNKNOWN = 0.5
FREE = 0.0
OCCUPIED = 1.0

STOP_DIST = 30
WARN_DIST = 100
SLOW_DIST = 160
MAX_SENSOR_CM = 400

# Minimum side-sensor clearance required before a direction is considered
# safe to turn into. Slightly above STOP_DIST so a turn never aims the user
# at a near-critical obstacle on the new heading.
TURN_MIN_CLEARANCE = STOP_DIST + 5

# How long to wait after announcing a turn before re-scanning to confirm the
# person's new forward direction (rules 6/14/15). Configurable, not hard-coded.
TURN_WAIT_SECONDS = 2.5

# If this many consecutive turn attempts fail to reach a clear path, stop
# retrying and declare the path blocked instead of oscillating (rule 13/16).
MAX_TURN_ATTEMPTS = 4

# HC-SR04 needs acoustic separation between pings. 0.015s was too tight for
# 3 sensors firing in sequence and caused cross-talk echoes to be read as a
# real short CENTER distance (spurious repeated "Stop."). Widened to give
# each sensor's echo time to fully settle before the next one fires.
SENSOR_GAP_SEC = 0.10
SENSOR_TIMEOUT_SEC = 0.1
FILTER_WINDOW = 3

# Navigation mode: how long a STOP must persist before triggering an A* replan.
STOP_REPLAN_SECS = 4.0

AUDIO_DEVICE = os.environ.get('ECHOMAP_AUDIO_DEVICE', 'default')
ESPEAK_VOICE = 'en-gb'
ESPEAK_SPEED = 130
ESPEAK_VOL = 200
SPEAK_COOLDOWN = 2.5

LONG_PRESS_SECS = 3.0
MAP_UPDATE_SECS = 0.10

# Dynamic obstacle classifier
DYNAMIC_HISTORY = 5
DYNAMIC_MOVE_CM = 18
DYNAMIC_CONFIRMATIONS = 2

# Navigation / personalization
GUIDANCE_HIGH_CONF = 0.78
GUIDANCE_MED_CONF = 0.55
LEVEL_THRESHOLDS = (0.55, 0.68, 0.80)
CONFIDENCE_MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'model', 'confidence_lstm.npz')

# Dashboard (optional — see main.py startup, Flask is never required)
DASHBOARD_HOST = '0.0.0.0'
DASHBOARD_PORT = 5000

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DB_PATH = os.path.join(BASE_DIR, 'echomap.db')

# NOTE: No MPU6050 / IMU constants here. The system does not use, import, or
# require an IMU anywhere in the runtime.

# --- Debugging ---
DEBUG_SENSOR_LOG = True       # print LEFT/CENTER/RIGHT cm readings
SENSOR_LOG_INTERVAL = 1.0     # seconds between debug prints (avoid flooding console)

# --- Decision-level debounce ---
# Require this many consecutive ticks of "both sides too close" before
# actually declaring PATH_BLOCKED, instead of trusting one noisy sample.
# While unconfirmed, the system still says "Stop." (never "Clear").
PATH_BLOCKED_CONFIRMATIONS = 3

# How long to stay silent after announcing a moving obstacle before
# re-announcing (if it's still blocking). Prevents repeating every 2s.
DYNAMIC_REANNOUNCE_SECS = 10.0
# config.py
PROLONGED_BLOCK_SECS = 8.0  # how long PATH_BLOCKED persists before advising a manual U-turn