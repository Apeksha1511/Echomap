# mapping.py — lightweight real-time 2D occupancy grid
import numpy as np
import math
import json                      # NEW
from database import get_all_landmarks   # NEW
from config import GRID_SIZE, CELL_CM, UNKNOWN, FREE, OCCUPIED
class OccupancyGrid:
    def __init__(self):
        self.grid = np.full((GRID_SIZE, GRID_SIZE), UNKNOWN, dtype=float)
        self.pos_x = GRID_SIZE // 2
        self.pos_y = GRID_SIZE // 2
        self.heading = 0.0
        self.step_cm = 40.0
        self.path = [(self.pos_x, self.pos_y)]
        # ---------- Ultrasonic SLAM State ----------
        self.prev_scan = None              # Previous ultrasonic readings
        self.pose_confidence = 1.0         # Localization confidence
        self.scan_history = []             # Recent scans (for loop closure later)
        self.auto_localization = True      # Enable SLAM localization

    def _safe(self, x, y):
        return 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE

    def update(self, left_cm, center_cm, right_cm):
        x, y = self.pos_x, self.pos_y
        if self._safe(x, y):
            self.grid[x, y] = FREE

        def mark(dist, angle_offset):
            if dist >= 400:
                return
            rad = math.radians(self.heading + angle_offset)
            cells = max(1, int(dist / CELL_CM))
            for i in range(1, cells):
                fx = x + int(round(i * math.sin(rad)))
                fy = y - int(round(i * math.cos(rad)))
                if self._safe(fx, fy):
                    self.grid[fx, fy] = FREE
            ox = x + int(round(cells * math.sin(rad)))
            oy = y - int(round(cells * math.cos(rad)))
            if self._safe(ox, oy):
                self.grid[ox, oy] = OCCUPIED
        mark(center_cm, 0)
        mark(left_cm, -45)
        mark(right_cm, 45)

    def update_slam(self, left_cm, center_cm, right_cm):
      """
        Lightweight Ultrasonic SLAM localization.
        Runs only during Mapping Mode.
      """

      current_scan = (left_cm, center_cm, right_cm)

    # First scan: initialize history.
      if self.prev_scan is None:
        self.prev_scan = current_scan
        self.scan_history.append(current_scan)
        return

      prev_left, prev_center, prev_right = self.prev_scan

    # ---------- Estimate Forward Movement ----------
      delta_front = prev_center - center_cm

    # Wall approaching -> assume user moved forward one cell.
      if self.auto_localization and 8 <= delta_front <= 30:
        self.advance_steps(1)

    # ---------- Estimate Turn ----------
      left_change = left_cm - prev_left
      right_change = right_cm - prev_right

      if left_change > 80 and right_change < -80:
        self.turn(90)

      elif right_change > 80 and left_change < -80:
        self.turn(-90)

    # ---------- Pose Confidence ----------
      confidence = 1.0

      if center_cm > 300:
        confidence -= 0.15

      if abs(delta_front) > 50:
        confidence -= 0.15

      self.pose_confidence = max(0.5, min(1.0, confidence))

    # ---------- Save History ----------
      self.scan_history.append(current_scan)

      if len(self.scan_history) > 10:
        self.scan_history.pop(0)

      self.prev_scan = current_scan

    def loop_closure(self):
       """
       Ultrasonic SLAM Loop Closure.

       Compares the current ultrasonic scan with saved landmark
       scan signatures. If a match is found, correct the current pose.
    """

       if self.prev_scan is None:
        return None

       current_left, current_center, current_right = self.prev_scan

       landmarks = get_all_landmarks()

       for name, x, y, heading, confidence, signature in landmarks:

        if signature is None:
            continue

        try:
            saved = json.loads(signature)
        except Exception:
            continue

        # Difference between current scan and saved scan
        dl = abs(current_left - saved["left"])
        dc = abs(current_center - saved["center"])
        dr = abs(current_right - saved["right"])

        # Match threshold (15 cm tolerance)
        if dl <= 15 and dc <= 15 and dr <= 15:

            # Correct pose
            self.pos_x = int(x)
            self.pos_y = int(y)
            self.heading = float(heading)

            # Increase confidence
            self.pose_confidence = max(self.pose_confidence, float(confidence))

            print(f"[SLAM] Loop closure at landmark '{name}'")

            return name

       return None

    def advance_steps(self, steps=1):
        # Position changes only when an actual/commanded walking step is known.
        for _ in range(max(0, int(steps))):
            rad = math.radians(self.heading)
            nx = self.pos_x + int(round(math.sin(rad)))
            ny = self.pos_y - int(round(math.cos(rad)))
            if self._safe(nx, ny):
                self.pos_x, self.pos_y = nx, ny
                self.grid[nx, ny] = FREE
                self.path.append((nx, ny))

    def move_forward(self):
        # Backward-compatible alias; callers should use advance_steps().
        self.advance_steps(1)

    def turn(self, degrees):
        self.heading = (self.heading + degrees) % 360

    def _strafe(self, angle_offset):
        # Sideways correction step relative to current heading — does NOT
        # change self.heading, per spec: "move left/right" is a movement
        # correction, not an intentional 90-degree turn.
        rad = math.radians(self.heading + angle_offset)
        nx = self.pos_x + int(round(math.sin(rad)))
        ny = self.pos_y - int(round(math.cos(rad)))
        if self._safe(nx, ny):
            self.pos_x, self.pos_y = nx, ny
            self.grid[nx, ny] = FREE
            self.path.append((nx, ny))

    def apply_mapping_command(self, command):
        """Intentional mapper commands from STT in Mapping Mode. Distinct from
        the obstacle-avoidance TURN_LEFT/TURN_RIGHT/MOVE_LEFT/MOVE_RIGHT that
        navigation.py returns — those must never call this."""
        command = (command or '').strip().lower()
        if command == 'turn_left':
            self.turn(-90)
            return 'Turn left recorded.'
        if command == 'turn_right':
            self.turn(90)
            return 'Turn right recorded.'
        if command == 'move_left':
            self._strafe(-90)
            return 'Move left recorded.'
        if command == 'move_right':
            self._strafe(90)
            return 'Move right recorded.'
        return 'Command not recognized.'

    def set_heading(self, degrees):
        self.heading = float(degrees) % 360

    def set_step_length(self, cm):
        self.step_cm = max(40.0, min(100.0, float(cm)))

    def print_map(self, size=24):
        cx, cy = self.pos_x, self.pos_y
        half = size // 2
        print('\n[MAP] # wall . free ? unknown @ you')
        for y in range(cy-half, cy+half):
            row=''
            for x in range(cx-half, cx+half):
                if x == cx and y == cy: row += '@'
                elif not self._safe(x,y): row += ' '
                elif self.grid[x,y] == OCCUPIED: row += '#'
                elif self.grid[x,y] == FREE: row += '.'
                else: row += '?'
            print(row)
        print(f"Pos: ({cx}, {cy})")
        print(f"Heading: {self.heading:.0f}°")
        print(f"SLAM Confidence: {self.pose_confidence:.2f}")

    def to_dict(self):
        return {'grid': self.grid.tolist(), 'pos_x': self.pos_x, 'pos_y': self.pos_y,
                'heading': self.heading, 'step_cm': self.step_cm, 'path': self.path[-500:]}

    def from_dict(self, d):
        self.grid = np.array(d['grid'], dtype=float)
        self.pos_x = int(d['pos_x']); self.pos_y = int(d['pos_y'])
        self.heading = float(d.get('heading', 0.0))
        self.step_cm = float(d.get('step_cm', 65.0))
        self.path = [tuple(p) for p in d.get('path', [(self.pos_x,self.pos_y)])]
