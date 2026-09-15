# path_planner.py — IEEE EchoMap Version
# A* path planner with compressed navigation instructions

import heapq
import math
from config import GRID_SIZE, OCCUPIED, UNKNOWN, CELL_CM


# ==========================================================
# A* PATH PLANNER
# ==========================================================

def astar(grid, start, goal, unknown_cost=1.5):
    """
    Compute shortest path using A* search.
    Unknown cells receive a small traversal penalty.
    """

    if not (0 <= start[0] < GRID_SIZE and 0 <= start[1] < GRID_SIZE):
        return None

    if not (0 <= goal[0] < GRID_SIZE and 0 <= goal[1] < GRID_SIZE):
        return None

    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    open_set = [(heuristic(start, goal), 0, start)]
    came_from = {}
    g_score = {start: 0}
    closed = set()

    while open_set:

        _, current_cost, current = heapq.heappop(open_set)

        if current in closed:
            continue

        closed.add(current)

        if current == goal:

            path = [current]

            while current in came_from:
                current = came_from[current]
                path.append(current)

            return path[::-1]

        for dx, dy in ((1,0), (-1,0), (0,1), (0,-1)):

            nxt = (current[0] + dx, current[1] + dy)

            if not (0 <= nxt[0] < GRID_SIZE and 0 <= nxt[1] < GRID_SIZE):
                continue

            if grid[nxt[0]][nxt[1]] >= OCCUPIED:
                continue

            cost = 1.0

            if grid[nxt[0]][nxt[1]] == UNKNOWN:
                cost += unknown_cost - 1

            new_cost = current_cost + cost

            if new_cost < g_score.get(nxt, float("inf")):

                g_score[nxt] = new_cost
                came_from[nxt] = current

                priority = new_cost + heuristic(nxt, goal)

                heapq.heappush(open_set, (priority, new_cost, nxt))

    return None

# ==========================================================
# CONVERT PATH TO COMPRESSED INSTRUCTIONS
# ==========================================================

def path_to_instructions(path, start_heading=0, step_cm=65):
    """
    Convert A* path into navigation instructions.

    Output format:
        ("turn_left", 90)
        ("walk", 5)
        ("turn_right", 90)
        ("walk", 2)
        ("destination", 0)
    """

    if not path or len(path) < 2:
        return []

    heading_map = {
        (0, -1): 0,      # North
        (1, 0): 90,      # East
        (0, 1): 180,     # South
        (-1, 0): 270     # West
    }

    instructions = []

    current_heading = start_heading
    forward_cells = 0

    def flush_forward():
        nonlocal forward_cells

        if forward_cells == 0:
            return

        steps = max(
            1,
            round(forward_cells * CELL_CM / step_cm)
        )

        instructions.append(("walk", steps))
        forward_cells = 0

    for current, nxt in zip(path, path[1:]):

        dx = nxt[0] - current[0]
        dy = nxt[1] - current[1]

        required_heading = heading_map[(dx, dy)]

        turn = (required_heading - current_heading + 360) % 360

        if turn == 270:
            turn = -90

        if turn == 90:
            flush_forward()
            instructions.append(("turn_right", 90))
            current_heading = required_heading

        elif turn == -90:
            flush_forward()
            instructions.append(("turn_left", -90))
            current_heading = required_heading

        elif turn == 180:
            flush_forward()
            instructions.append(("turn_back", 180))
            current_heading = required_heading

        forward_cells += 1

    flush_forward()

    instructions.append(("destination", 0))

    return instructions


# ==========================================================
# SPEECH SENTENCES
# ==========================================================

def instruction_to_speech(instructions):
    """
    Convert structured instructions into spoken English.
    """

    speech = []

    for command, value in instructions:

        if command == "turn_left":
            speech.append("Turn left.")

        elif command == "turn_right":
            speech.append("Turn right.")

        elif command == "turn_back":
            speech.append("Turn around.")

        elif command == "walk":

            if value == 1:
                speech.append("Walk forward one step.")
            else:
                speech.append(f"Walk forward {value} steps.")

        elif command == "destination":
            speech.append("Destination reached.")

    return speech


# ==========================================================
# ROUTE METRICS
# ==========================================================

def route_metrics(instructions):
    """
    Metrics for confidence prediction.
    """

    walk_steps = sum(
        value for cmd, value in instructions
        if cmd == "walk"
    )

    turns = sum(
        1 for cmd, _ in instructions
        if cmd in ("turn_left", "turn_right", "turn_back")
    )

    complexity = round(turns * 0.5 + walk_steps / 20.0, 2)

    return walk_steps, turns, complexity