# mapping.py — lightweight real-time 2D occupancy grid
import numpy as np
import math
from config import GRID_SIZE, CELL_CM, UNKNOWN, FREE, OCCUPIED

class OccupancyGrid:
    def __init__(self):
        self.grid = np.full((GRID_SIZE, GRID_SIZE), UNKNOWN, dtype=float)
        self.pos_x = GRID_SIZE // 2
        self.pos_y = GRID_SIZE // 2
        self.heading = 0.0
        self.step_cm = 65.0
        self.path = [(self.pos_x, self.pos_y)]

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
        print(f'Pos:({cx},{cy}) Heading:{self.heading:.0f}°')

    def to_dict(self):
        return {'grid': self.grid.tolist(), 'pos_x': self.pos_x, 'pos_y': self.pos_y,
                'heading': self.heading, 'step_cm': self.step_cm, 'path': self.path[-500:]}

    def from_dict(self, d):
        self.grid = np.array(d['grid'], dtype=float)
        self.pos_x = int(d['pos_x']); self.pos_y = int(d['pos_y'])
        self.heading = float(d.get('heading', 0.0))
        self.step_cm = float(d.get('step_cm', 65.0))
        self.path = [tuple(p) for p in d.get('path', [(self.pos_x,self.pos_y)])]
