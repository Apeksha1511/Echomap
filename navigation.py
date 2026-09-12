# navigation.py — three-direction obstacle decisions + temporal dynamic classification
#
# IMPORTANT: this module only classifies a single fresh sensor reading. It
# NEVER issues lateral-movement instructions (MOVE_LEFT/MOVE_RIGHT) because
# the three forward-facing sensors cannot fully observe a side path — see
# EchoMap spec §8/§13. All turn-wait/rescan sequencing lives in main.py's
# DiscoveryController, which is the only place allowed to decide that a
# turn has been "confirmed" by a fresh post-turn scan.
from collections import deque
from config import (
    STOP_DIST, WARN_DIST, SLOW_DIST, TURN_MIN_CLEARANCE,
    DYNAMIC_HISTORY, DYNAMIC_MOVE_CM, DYNAMIC_CONFIRMATIONS,
    PATH_BLOCKED_CONFIRMATIONS,
)

_history = {'LEFT': deque(maxlen=DYNAMIC_HISTORY), 'CENTER': deque(maxlen=DYNAMIC_HISTORY), 'RIGHT': deque(maxlen=DYNAMIC_HISTORY)}
_dynamic_votes = {'LEFT': 0, 'CENTER': 0, 'RIGHT': 0}

# Tracks whether each sensor was "in range" (< WARN_DIST) on the previous
# tick. When an obstacle first enters range, its history is cleared so the
# filter settling onto the true distance isn't misread as movement.
_prev_in_range = {'LEFT': False, 'CENTER': False, 'RIGHT': False}

# Votes toward confirming "both sides too close" — requires multiple
# consecutive ticks before actually declaring PATH_BLOCKED, so a single
# noisy sample can't produce a false "Path blocked" (spec rule 16 / test 9).
_blocked_votes = 0


def reset_dynamic_history(name=None):
    """Clear dynamic-obstacle tracking state for one sensor or all three.
    Called by main.py right when a turn is issued: the CENTER/side readings
    are about to change fast because the *person* is rotating, not because
    something is moving — so stale/in-progress history shouldn't be trusted
    once the person starts turning."""
    names = ('LEFT', 'CENTER', 'RIGHT') if name is None else (name,)
    for n in names:
        _history[n].clear()
        _dynamic_votes[n] = 0
        _prev_in_range[n] = False


def _update_dynamic(name, distance):
    now_in_range = distance < WARN_DIST
    if now_in_range and not _prev_in_range[name]:
        # Obstacle just entered range this tick — wipe stale/irrelevant
        # history so the classifier can't confuse "filter still settling
        # onto the true distance" with genuine movement.
        _history[name].clear()
        _dynamic_votes[name] = 0
    _prev_in_range[name] = now_in_range

    h = _history[name]
    if distance >= 400:
        h.append(None)
        return False
    h.append(distance)
    valid = [x for x in h if x is not None]
    if len(valid) < DYNAMIC_HISTORY:
        # Require the full history window (not just 3) before trusting a
        # movement judgment — this is what was actually causing turns to be
        # misread as moving obstacles.
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
    global _blocked_votes
    dynamic = classify_dynamic(left, center, right)

    if dynamic['CENTER'] and center < WARN_DIST:
        _blocked_votes = 0
        return 'MOVING_CENTER', 'Moving obstacle ahead. Please wait.'

    if center < STOP_DIST:
        l = left if left < 400 else 999.0
        r = right if right < 400 else 999.0
        l_safe = l >= TURN_MIN_CLEARANCE
        r_safe = r >= TURN_MIN_CLEARANCE

        if not l_safe and not r_safe:
            _blocked_votes = min(PATH_BLOCKED_CONFIRMATIONS, _blocked_votes + 1)
            if _blocked_votes >= PATH_BLOCKED_CONFIRMATIONS:
                return 'PATH_BLOCKED', 'Stop. Path blocked.'
            return 'STOP', 'Stop.'

        _blocked_votes = 0
        return 'STOP', 'Stop.'

    if center < WARN_DIST:
        l = left if left < 400 else 999.0
        r = right if right < 400 else 999.0
        l_safe = l >= TURN_MIN_CLEARANCE
        r_safe = r >= TURN_MIN_CLEARANCE

        if not l_safe and not r_safe:
            _blocked_votes = min(PATH_BLOCKED_CONFIRMATIONS, _blocked_votes + 1)
            if _blocked_votes >= PATH_BLOCKED_CONFIRMATIONS:
                return 'PATH_BLOCKED', 'Stop. Path blocked.'
            return 'STOP', 'Stop.'

        _blocked_votes = 0
        if l_safe and not r_safe:
            return 'TURN_LEFT', 'Obstacle ahead. Turn left.'
        if r_safe and not l_safe:
            return 'TURN_RIGHT', 'Obstacle ahead. Turn right.'
        if l >= r:
            return 'TURN_LEFT', 'Obstacle ahead. Turn left.'
        return 'TURN_RIGHT', 'Obstacle ahead. Turn right.'

    _blocked_votes = 0
    if center < SLOW_DIST:
        return 'SLOW', 'Slow down.'

    return 'CLEAR', 'Path clear. Walk.'