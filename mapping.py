# mapping.py — lightweight real-time 2D occupancy grid
import numpy as np
import math
import json
import time
from database import get_all_landmarks
from config import GRID_SIZE, CELL_CM, UNKNOWN, FREE, OCCUPIED


class OccupancyGrid:
    def __init__(self):
        self.grid = np.full(
            (GRID_SIZE, GRID_SIZE),
            UNKNOWN,
            dtype=float
        )

        self.pos_x = GRID_SIZE // 2
        self.pos_y = GRID_SIZE // 2

        # Heading is controlled ONLY by explicit mapper turn commands.
        # 0° = forward / north
        # 90° = right
        # 180° = backward
        # 270° = left
        self.heading = 0.0

        self.step_cm = 40.0
        self.path = [(self.pos_x, self.pos_y)]

        # ---------------------------------------------------------
        # Ultrasonic SLAM / Motion Estimation State
        # ---------------------------------------------------------

        self.prev_scan = None

        # Localization confidence
        self.pose_confidence = 1.0

        # Recent scans
        self.scan_history = []

        # Enable conservative sensor-based motion estimation
        self.auto_localization = True

        # ---------------------------------------------------------
        # Motion estimation parameters
        # ---------------------------------------------------------

        # Number of consecutive scans that must support movement
        # before one grid-cell movement is accepted.
        self.motion_confirmations = 0

        # Required confirmations before moving one grid cell.
        self.required_motion_confirmations = 2
        self.last_motion_time = 0.0

# Accumulated estimated walking distance.
        self.accumulated_motion_cm = 0.0

# Approximate physical distance represented by one grid cell.
        self.grid_step_cm = 40.0

        self.min_forward_delta = 4.0
        self.max_forward_delta = 80.0

# Prevent the same sensor movement from being counted repeatedly.
        self.motion_cooldown = 0.15

    # ---------------------------------------------------------
    # Basic grid safety
    # ---------------------------------------------------------

    def _safe(self, x, y):
        return 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE

    # ---------------------------------------------------------
    # Occupancy Grid Update
    # ---------------------------------------------------------

    def update(self, left_cm, center_cm, right_cm):
        x, y = self.pos_x, self.pos_y

        if self._safe(x, y):
            self.grid[x, y] = FREE

        def mark(dist, angle_offset):
            # Invalid / out-of-range reading.
            if dist is None:
                return

            if dist >= 400:
                return

            if dist <= 0:
                return

            rad = math.radians(
                self.heading + angle_offset
            )

            cells = max(
                1,
                int(dist / CELL_CM)
            )

            # Free cells between user and obstacle
            for i in range(1, cells):
                fx = x + int(
                    round(i * math.sin(rad))
                )

                fy = y - int(
                    round(i * math.cos(rad))
                )

                if self._safe(fx, fy):
                    self.grid[fx, fy] = FREE

            # Obstacle cell
            ox = x + int(
                round(cells * math.sin(rad))
            )

            oy = y - int(
                round(cells * math.cos(rad))
            )

            if self._safe(ox, oy):
                self.grid[ox, oy] = OCCUPIED

        # Center sensor = forward
        mark(center_cm, 0)

        # Left sensor
        mark(left_cm, -45)

        # Right sensor
        mark(right_cm, 45)

    # ---------------------------------------------------------
    # Lightweight Ultrasonic SLAM
    # ---------------------------------------------------------

    def update_slam(self, left_cm, center_cm, right_cm):
        """
        Estimate mapper movement from consecutive ultrasonic scans.

        Sensor observations estimate forward movement.
        Heading is changed ONLY by explicit turn commands.
        """

        print(
            f"[SLAM SENSOR] L={left_cm:.1f} "
            f"C={center_cm:.1f} R={right_cm:.1f}"
        )

        current_scan = (
            left_cm,
            center_cm,
            right_cm
        )

        # First scan establishes the reference.
        if self.prev_scan is None:
            self.prev_scan = current_scan
            self.scan_history.append(current_scan)
            self.motion_confirmations = 0
            self.pose_confidence = 0.70
            return

        previous_scan = self.prev_scan

        current_values = [
            left_cm,
            center_cm,
            right_cm
        ]

        previous_values = [
            previous_scan[0],
            previous_scan[1],
            previous_scan[2]
        ]

        # -----------------------------------------------------
        # Compare valid sensor readings
        # -----------------------------------------------------

        changes = []

        for current, previous in zip(
            current_values,
            previous_values
        ):
            if (
                current is not None
                and previous is not None
                and 0 < current < 400
                and 0 < previous < 400
            ):
                changes.append(
                    abs(current - previous)
                )

        # Need at least two sensors to support movement.
        enough_sensors = len(changes) >= 2

        # At least two sensors should change.
        changed_sensors = sum(
            1 for change in changes
            if change >= self.min_forward_delta
        )

        average_change = (
            sum(changes) / len(changes)
            if changes
            else 0.0
        )

        movement_detected = (
            self.auto_localization
            and enough_sensors
            and changed_sensors >= 2
            and self.min_forward_delta
            <= average_change
            <= self.max_forward_delta
        )

        # -----------------------------------------------------
        # Accumulate movement
        # -----------------------------------------------------

        if movement_detected:
            self.motion_confirmations += 1

            # The sensor-distance change is treated as
            # approximate physical movement.
            self.accumulated_motion_cm += average_change

            print(
                f"[SLAM] Movement accumulation: "
                f"{self.accumulated_motion_cm:.1f} cm"
            )

        else:
            self.motion_confirmations = max(
                0,
                self.motion_confirmations - 1
            )

        # -----------------------------------------------------
        # Convert accumulated movement into grid cells
        # -----------------------------------------------------

        if (
            self.auto_localization
            and self.motion_confirmations
            >= self.required_motion_confirmations
        ):

            cells_to_move = int(
                self.accumulated_motion_cm
                / self.grid_step_cm
            )

            if cells_to_move >= 1:

                old_position = (
                    self.pos_x,
                    self.pos_y
                )

                self.advance_steps(cells_to_move)

                new_position = (
                    self.pos_x,
                    self.pos_y
                )

                if new_position != old_position:

                    used_distance = (
                        cells_to_move
                        * self.grid_step_cm
                    )

                    self.accumulated_motion_cm = max(
                        0.0,
                        self.accumulated_motion_cm
                        - used_distance
                    )

                    self.last_motion_time = (
                        time.monotonic()
                    )

                    self.motion_confirmations = 0

                    self.pose_confidence = min(
                        1.0,
                        self.pose_confidence + 0.05
                    )

                    print(
                        f"[SLAM] Moved {cells_to_move} "
                        f"grid cell(s) -> "
                        f"Pos: ({self.pos_x}, "
                        f"{self.pos_y}) "
                        f"Heading: "
                        f"{self.heading:.0f}°"
                    )

        # -----------------------------------------------------
        # Heading is NEVER changed here.
        # -----------------------------------------------------
        #
        # Only explicit:
        #   turn_left
        #   turn_right
        #
        # commands can change heading.
        # -----------------------------------------------------

        confidence = 0.85

        if len(changes) < 2:
            confidence -= 0.15

        if average_change > self.max_forward_delta:
            confidence -= 0.15

        self.pose_confidence = max(
            0.50,
            min(1.0, confidence)
        )

        # Save scan.
        self.scan_history.append(
            current_scan
        )

        if len(self.scan_history) > 10:
            self.scan_history.pop(0)

        self.prev_scan = current_scan
    # ---------------------------------------------------------
    # Reset SLAM tracking state
    # ---------------------------------------------------------

    def reset_slam(self):
        """
        Reset only temporary scan/motion estimation state.

        The occupancy map, position and heading are preserved.
        """

        self.prev_scan = None
        self.motion_confirmations = 0
        self.last_motion_time = 0.0
        self.scan_history = []

    # ---------------------------------------------------------
    # Loop Closure
    # ---------------------------------------------------------

    def loop_closure(self):
        """
        Ultrasonic SLAM loop closure.

        Compares the current ultrasonic scan with saved landmark
        scan signatures.

        If a matching landmark is found, correct the position.

        IMPORTANT:
        The saved landmark heading is NOT automatically applied.
        Heading is controlled only by explicit turn commands.
        """

        if self.prev_scan is None:
            return None

        current_left, current_center, current_right = (
            self.prev_scan
        )

        landmarks = get_all_landmarks()

        for (
            name,
            x,
            y,
            heading,
            confidence,
            signature
        ) in landmarks:

            if signature is None:
                continue

            try:
                saved = json.loads(signature)

            except Exception:
                continue

            # Validate saved signature
            if not all(
                key in saved
                for key in (
                    "left",
                    "center",
                    "right"
                )
            ):
                continue

            try:
                dl = abs(
                    current_left
                    - saved["left"]
                )

                dc = abs(
                    current_center
                    - saved["center"]
                )

                dr = abs(
                    current_right
                    - saved["right"]
                )

            except Exception:
                continue

            # Match threshold = 15 cm
            if (
                dl <= 15
                and dc <= 15
                and dr <= 15
            ):

                # Correct position only.
                self.pos_x = int(x)
                self.pos_y = int(y)

                # DO NOT change heading here.
                #
                # The mapper's heading is controlled only by:
                # turn_left / turn_right voice commands.

                self.pose_confidence = max(
                    self.pose_confidence,
                    float(confidence)
                )

                # Reset temporary motion estimator because
                # localization has just been corrected.
                self.motion_confirmations = 0

                print(
                    f"[SLAM] Loop closure at "
                    f"landmark '{name}'"
                )

                return name

        return None

    # ---------------------------------------------------------
    # Position Movement
    # ---------------------------------------------------------

    def advance_steps(self, steps=1):
        """
        Advance the tracked position.

        Existing coordinate convention is preserved:

        Heading 0°:
            x stays the same
            y decreases

        Heading 90°:
            x increases
            y stays the same

        Heading 180°:
            x stays the same
            y increases

        Heading 270°:
            x decreases
            y stays the same
        """

        for _ in range(
            max(0, int(steps))
        ):

            rad = math.radians(
                self.heading
            )

            nx = self.pos_x + int(
                round(math.sin(rad))
            )

            ny = self.pos_y - int(
                round(math.cos(rad))
            )

            if self._safe(nx, ny):

                self.pos_x = nx
                self.pos_y = ny

                self.grid[nx, ny] = FREE

                self.path.append(
                    (nx, ny)
                )

    # ---------------------------------------------------------
    # Backward-compatible movement
    # ---------------------------------------------------------

    def move_forward(self):
        """
        Backward-compatible alias.
        """

        self.advance_steps(1)

    # ---------------------------------------------------------
    # Intentional Turn
    # ---------------------------------------------------------

    def turn(self, degrees):
        """
        Change heading.

        This function is intentionally NOT called by ultrasonic
        scan processing.

        It is used only by explicit mapper commands.
        """

        self.heading = (
            self.heading + degrees
        ) % 360

    # ---------------------------------------------------------
    # Sideways Movement
    # ---------------------------------------------------------

    def _strafe(self, angle_offset):
        """
        Move one grid cell relative to current heading.

        This does NOT change heading.
        """

        rad = math.radians(
            self.heading + angle_offset
        )

        nx = self.pos_x + int(
            round(math.sin(rad))
        )

        ny = self.pos_y - int(
            round(math.cos(rad))
        )

        if self._safe(nx, ny):

            self.pos_x = nx
            self.pos_y = ny

            self.grid[nx, ny] = FREE

            self.path.append(
                (nx, ny)
            )

    # ---------------------------------------------------------
    # Mapping Voice Commands
    # ---------------------------------------------------------

    def apply_mapping_command(self, command):
        """
        Process intentional mapper commands.

        Only explicit turn commands modify heading.

        Obstacle avoidance actions from navigation.py must
        never call this function.
        """

        command = (
            command or ""
        ).strip().lower()

        # -----------------------------------------------------
        # Explicit intentional turns
        # -----------------------------------------------------

        if command == "turn_left":

            self.turn(-90)

            # Reset motion estimator after a turn.
            self.motion_confirmations = 0
            self.prev_scan = None

            return "Turn left recorded."

        if command == "turn_right":

            self.turn(90)

            # Reset motion estimator after a turn.
            self.motion_confirmations = 0
            self.prev_scan = None

            return "Turn right recorded."

        # -----------------------------------------------------
        # Optional lateral movement commands
        # -----------------------------------------------------

        if command == "move_left":

            self._strafe(-90)

            self.motion_confirmations = 0

            return "Move left recorded."

        if command == "move_right":

            self._strafe(90)

            self.motion_confirmations = 0

            return "Move right recorded."

        # -----------------------------------------------------
        # Forward/backward voice commands are deliberately
        # not used for automatic Mapping Mode movement.
        # Movement is estimated from sensor observations.
        # -----------------------------------------------------

        if command in (
            "forward",
            "backward",
            "forward_1",
            "forward_2",
            "forward_3",
            "forward_4",
            "forward_5",
            "forward_6",
            "forward_7",
            "forward_8",
            "forward_9",
            "forward_10",
            "backward_1",
            "backward_2",
            "backward_3",
            "backward_4",
            "backward_5",
            "backward_6",
            "backward_7",
            "backward_8",
            "backward_9",
            "backward_10",
        ):

            return (
                "Movement is tracked automatically."
            )

        return "Command not recognized."

    # ---------------------------------------------------------
    # Heading Setter
    # ---------------------------------------------------------

    def set_heading(self, degrees):
        """
        Explicitly set heading.

        Kept for compatibility with existing code.
        """

        self.heading = (
            float(degrees) % 360
        )

        self.motion_confirmations = 0
        self.prev_scan = None

    # ---------------------------------------------------------
    # Step Length
    # ---------------------------------------------------------

    def set_step_length(self, cm):
        self.step_cm = max(
            40.0,
            min(
                100.0,
                float(cm)
            )
        )

    # ---------------------------------------------------------
    # Print Local Map
    # ---------------------------------------------------------

    def print_map(self, size=24):

        cx = self.pos_x
        cy = self.pos_y

        half = size // 2

        print(
            "\n[MAP] # wall . free ? unknown @ you"
        )

        for y in range(
            cy - half,
            cy + half
        ):

            row = ""

            for x in range(
                cx - half,
                cx + half
            ):

                if (
                    x == cx
                    and y == cy
                ):
                    row += "@"

                elif not self._safe(x, y):
                    row += " "

                elif (
                    self.grid[x, y]
                    == OCCUPIED
                ):
                    row += "#"

                elif (
                    self.grid[x, y]
                    == FREE
                ):
                    row += "."

                else:
                    row += "?"

            print(row)

        print(
            f"Pos: ({cx}, {cy})"
        )

        print(
            f"Heading: {self.heading:.0f}°"
        )

        print(
            f"SLAM Confidence: "
            f"{self.pose_confidence:.2f}"
        )

    # ---------------------------------------------------------
    # Save Grid State
    # ---------------------------------------------------------

    def to_dict(self):

        return {
            "grid": self.grid.tolist(),
            "pos_x": self.pos_x,
            "pos_y": self.pos_y,
            "heading": self.heading,
            "step_cm": self.step_cm,
            "path": self.path[-500:]
        }

    # ---------------------------------------------------------
    # Restore Grid State
    # ---------------------------------------------------------

    def from_dict(self, d):

        self.grid = np.array(
            d["grid"],
            dtype=float
        )

        self.pos_x = int(
            d["pos_x"]
        )

        self.pos_y = int(
            d["pos_y"]
        )

        self.heading = float(
            d.get(
                "heading",
                0.0
            )
        )

        self.step_cm = float(
            d.get(
                "step_cm",
                65.0
            )
        )

        self.path = [
            tuple(p)
            for p in d.get(
                "path",
                [
                    (
                        self.pos_x,
                        self.pos_y
                    )
                ]
            )
        ]

        # Temporary SLAM state should start fresh
        # after loading a map.
        self.reset_slam()