# EchoMap configuration
import os

SENSORS = {
    'LEFT':   {'trig': 17, 'echo': 27},
    'CENTER': {'trig': 23, 'echo': 24},
    'RIGHT':  {'trig': 22, 'echo': 25},
}

MPU6050_ADDRESS = 0x68
MPU6050_PWR = 0x6B

BTN_TAG = 16
BTN_CYCLE = 20
BTN_NAVIGATE = 21

GRID_SIZE = 100
CELL_CM = 10
UNKNOWN = 0.5
FREE = 0.0
OCCUPIED = 1.0

STOP_DIST = 30
WARN_DIST = 100
SLOW_DIST = 160
SIDE_WARN = 30
MAX_SENSOR_CM = 400

# HC-SR04 needs acoustic separation between pings. The effective scan rate is
# kept near the synopsis target while avoiding cross-talk.
SENSOR_GAP_SEC = 0.015
SENSOR_TIMEOUT_SEC = 0.025
FILTER_WINDOW = 3

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

# Dashboard
DASHBOARD_HOST = '0.0.0.0'
DASHBOARD_PORT = 5000

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DB_PATH = os.path.join(BASE_DIR, 'echomap.db')
