# navigation.py — three-direction obstacle decisions + temporal dynamic classification
from collections import deque
from config import STOP_DIST, WARN_DIST, SLOW_DIST, SIDE_WARN, DYNAMIC_HISTORY, DYNAMIC_MOVE_CM, DYNAMIC_CONFIRMATIONS

_history = {'LEFT': deque(maxlen=DYNAMIC_HISTORY), 'CENTER': deque(maxlen=DYNAMIC_HISTORY), 'RIGHT': deque(maxlen=DYNAMIC_HISTORY)}
_dynamic_votes = {'LEFT': 0, 'CENTER': 0, 'RIGHT': 0}


def _update_dynamic(name, distance):
    h = _history[name]
    if distance >= 400:
        h.append(None)
        return False
    h.append(distance)
    valid = [x for x in h if x is not None]
    if len(valid) < 3:
        return False
    movement = max(valid) - min(valid)
    # A persistent obstacle has a stable range; a person crossing the beam
    # creates a consistent temporal change rather than a one-frame spike.
    moving = movement >= DYNAMIC_MOVE_CM and valid[-1] < WARN_DIST
    if moving:
        _dynamic_votes[name] = min(DYNAMIC_CONFIRMATIONS, _dynamic_votes[name] + 1)
    else:
        _dynamic_votes[name] = max(0, _dynamic_votes[name] - 1)
    return _dynamic_votes[name] >= DYNAMIC_CONFIRMATIONS


def classify_dynamic(left, center, right):
    return {
        'LEFT': _update_dynamic('LEFT', left),
        'CENTER': _update_dynamic('CENTER', center),
        'RIGHT': _update_dynamic('RIGHT', right),
    }


def get_nav_instruction(left, center, right):
    dynamic = classify_dynamic(left, center, right)
    if dynamic['CENTER'] and center < WARN_DIST:
        return 'MOVING_CENTER', 'Moving obstacle ahead. Please wait.'

    if center < STOP_DIST:
        return 'STOP', 'Stop.'
    if center < WARN_DIST:
        l = left if left < 400 else 500
        r = right if right < 400 else 500
        if l >= r:
            return 'TURN_LEFT', 'Obstacle ahead. Turn left.'
        return 'TURN_RIGHT', 'Obstacle ahead. Turn right.'
    if center < SLOW_DIST:
        return 'SLOW', 'Slow down.'
    if left < SIDE_WARN:
        return 'MOVE_RIGHT', 'Move right.'
    if right < SIDE_WARN:
        return 'MOVE_LEFT', 'Move left.'
    return 'CLEAR', ''
