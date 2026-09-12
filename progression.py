# progression.py — confidence-driven guidance and gamified levels
from config import LEVEL_THRESHOLDS

def level_for_confidence(conf):
    c=float(conf)
    if c < LEVEL_THRESHOLDS[0]: return 1
    if c < LEVEL_THRESHOLDS[1]: return 2
    if c < LEVEL_THRESHOLDS[2]: return 3
    return 4

def guidance_interval(conf):
    c=float(conf)
    if c < .55: return 2
    if c < .78: return 4
    return 7

def route_allowed(conf, complexity):
    # Higher confidence permits more route turns/complexity.
    return complexity <= max(2.0, 2.0 + 12.0*float(conf))

def level_message(level):
    names={1:'straight corridors',2:'simple turns',3:'multiple turns',4:'complex rooms and multiple doors'}
    return f'Level {level}. {names[level]}.'
