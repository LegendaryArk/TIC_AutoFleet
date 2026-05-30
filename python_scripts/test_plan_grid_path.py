import sys
import os
import unittest
import importlib
import importlib.util
import types

sys.path.insert(0, os.path.dirname(__file__))

# Stub out the gui module so central-arbiter imports cleanly without a display
gui_stub = types.ModuleType("gui")
class _FakeGUI:
    def __init__(self, **kw): pass
    def update_robot(self, *a, **kw): pass
    def run(self): pass
gui_stub.TelemetryGUI = _FakeGUI
sys.modules.setdefault("gui", gui_stub)

# central-arbiter has a hyphen so we load it manually
spec = importlib.util.spec_from_file_location(
    "central_arbiter",
    os.path.join(os.path.dirname(__file__), "central-arbiter.py"),
)
central_arbiter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(central_arbiter)

plan_grid_path = central_arbiter.plan_grid_path
GRID_DIM_CELLS = central_arbiter.GRID_DIM_CELLS


# ------------------------------------------------------------------
# Matplotlib visualization
# ------------------------------------------------------------------
def plot_path(
    start: tuple[int, int],
    goal: tuple[int, int],
    blocked: set[tuple[int, int]],
    path: list[tuple[int, int]] | None,
    *,
    label: str = "",
    grid_size: int = GRID_DIM_CELLS,
    ax=None,
    crop: bool = True,
) -> None:
    import numpy as np
    import matplotlib.pyplot as plt

    standalone = ax is None
    if standalone:
        fig, ax = plt.subplots(figsize=(7, 7))

    path_set = set(path) if path else set()

    # Determine the bounding box to display
    if crop:
        all_cells = path_set | {start, goal} | blocked
        rows = [r for r, c in all_cells if 0 <= r < grid_size and 0 <= c < grid_size]
        cols = [c for r, c in all_cells if 0 <= r < grid_size and 0 <= c < grid_size]
        pad = 1
        r_min = max(0, min(rows) - pad)
        r_max = min(grid_size - 1, max(rows) + pad)
        c_min = max(0, min(cols) - pad)
        c_max = min(grid_size - 1, max(cols) + pad)
    else:
        r_min, r_max = 0, grid_size - 1
        c_min, c_max = 0, grid_size - 1

    height = r_max - r_min + 1
    width  = c_max - c_min + 1

    # Build an RGB image: one pixel per cell
    img = np.full((height, width, 3), 0.93)  # default: light gray

    COLORS = {
        "empty":   [0.93, 0.93, 0.93],
        "path":    [0.45, 0.70, 1.00],
        "blocked": [0.30, 0.30, 0.30],
        "start":   [0.20, 0.78, 0.35],
        "goal":    [0.95, 0.35, 0.35],
    }

    for r in range(r_min, r_max + 1):
        for c in range(c_min, c_max + 1):
            ri, ci = r - r_min, c - c_min
            cell = (r, c)
            if cell == start:
                img[ri, ci] = COLORS["start"]
            elif cell == goal:
                img[ri, ci] = COLORS["goal"]
            elif cell in blocked:
                img[ri, ci] = COLORS["blocked"]
            elif cell in path_set:
                img[ri, ci] = COLORS["path"]
            else:
                img[ri, ci] = COLORS["empty"]

    ax.imshow(img, origin="upper", aspect="equal",
              extent=[c_min - 0.5, c_max + 0.5, r_max + 0.5, r_min - 0.5])

    # Grid lines
    for c in range(c_min, c_max + 2):
        ax.axvline(c - 0.5, color="white", linewidth=0.4, zorder=1)
    for r in range(r_min, r_max + 2):
        ax.axhline(r - 0.5, color="white", linewidth=0.4, zorder=1)

    # Path arrows
    if path and len(path) > 1:
        for (r0, c0), (r1, c1) in zip(path, path[1:]):
            ax.annotate(
                "",
                xy=(c1, r1), xytext=(c0, r0),
                arrowprops=dict(arrowstyle="->", color="navy", lw=1.3),
                zorder=2,
            )

    # S / G labels
    sr, sc = start
    gr, gc = goal
    ax.text(sc, sr, "S", ha="center", va="center",
            fontsize=9, fontweight="bold", color="white", zorder=3)
    ax.text(gc, gr, "G", ha="center", va="center",
            fontsize=9, fontweight="bold", color="white", zorder=3)

    # Axis ticks at actual grid coordinates
    ax.set_xticks(range(c_min, c_max + 1))
    ax.set_yticks(range(r_min, r_max + 1))
    ax.tick_params(labelsize=7)

    # y-axis increases downward to match (row, col) convention
    ax.invert_yaxis()

    suffix = f"  [{len(path)} cells]" if path else "  [no path]"
    ax.set_title(label + suffix, fontsize=10)

    if standalone:
        plt.tight_layout()
        plt.show()


# ------------------------------------------------------------------
# Unit tests  (no plotting — keep them fast and CI-safe)
# ------------------------------------------------------------------
class TestPlanGridPath(unittest.TestCase):

    def assert_valid_path(self, path, start, goal):
        self.assertIsNotNone(path)
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)
        for r, c in path:
            self.assertGreaterEqual(r, 0)
            self.assertGreaterEqual(c, 0)
            self.assertLess(r, GRID_DIM_CELLS)
            self.assertLess(c, GRID_DIM_CELLS)
        # Each step moves at most 1 cell in any direction (8-directional)
        for (r0, c0), (r1, c1) in zip(path, path[1:]):
            self.assertLessEqual(
                max(abs(r1 - r0), abs(c1 - c0)), 1, "non-adjacent step in path"
            )

    # ------------------------------------------------------------------
    # Basic cases
    # ------------------------------------------------------------------
    def test_start_equals_goal(self):
        path = plan_grid_path((5, 5), (5, 5), set())
        self.assertEqual(path, [(5, 5)])

    def test_adjacent_goal_no_obstacles(self):
        path = plan_grid_path((0, 0), (0, 1), set())
        self.assert_valid_path(path, (0, 0), (0, 1))

    def test_straight_horizontal(self):
        path = plan_grid_path((0, 0), (0, 5), set())
        self.assert_valid_path(path, (0, 0), (0, 5))
        self.assertEqual(len(path), 6)  # no diagonal shortcut on same row

    def test_straight_vertical(self):
        path = plan_grid_path((0, 0), (5, 0), set())
        self.assert_valid_path(path, (0, 0), (5, 0))
        self.assertEqual(len(path), 6)  # no diagonal shortcut on same col

    def test_diagonal_chebyshev(self):
        # With 8-directional movement optimal length = Chebyshev distance + 1
        start, goal = (0, 0), (3, 4)
        path = plan_grid_path(start, goal, set())
        self.assert_valid_path(path, start, goal)
        self.assertEqual(len(path), max(3, 4) + 1)  # = 5

    # ------------------------------------------------------------------
    # Grid corners / edges
    # ------------------------------------------------------------------
    def test_top_left_to_bottom_right(self):
        start = (0, 0)
        goal = (GRID_DIM_CELLS - 1, GRID_DIM_CELLS - 1)
        path = plan_grid_path(start, goal, set())
        self.assert_valid_path(path, start, goal)
        self.assertEqual(len(path), GRID_DIM_CELLS)  # pure diagonal

    def test_same_row_edge(self):
        path = plan_grid_path((0, 0), (0, GRID_DIM_CELLS - 1), set())
        self.assert_valid_path(path, (0, 0), (0, GRID_DIM_CELLS - 1))

    # ------------------------------------------------------------------
    # Obstacle avoidance
    # ------------------------------------------------------------------
    def test_single_obstacle_detour(self):
        start, goal = (0, 0), (0, 2)
        blocked = {(0, 1)}
        path = plan_grid_path(start, goal, blocked)
        self.assert_valid_path(path, start, goal)
        for cell in path:
            self.assertNotIn(cell, blocked)

    def test_wall_with_gap(self):
        start, goal = (0, 0), (0, 5)
        blocked = {(r, 2) for r in range(5)}
        path = plan_grid_path(start, goal, blocked)
        self.assert_valid_path(path, start, goal)
        for cell in path:
            self.assertNotIn(cell, blocked)

    def test_goal_in_blocked_is_still_reachable(self):
        goal = (2, 2)
        path = plan_grid_path((0, 0), goal, {goal})
        self.assert_valid_path(path, (0, 0), goal)

    def test_no_path_when_completely_surrounded(self):
        blocked = {(0, 1), (1, 0), (1, 2), (2, 1), (1, 1)}
        # With 8 directions we also need to block diagonals
        blocked |= {(0, 0), (0, 2), (2, 0), (2, 2)}
        path = plan_grid_path((3, 3), (1, 1), blocked)
        self.assertIsNone(path)

    def test_no_path_full_column_wall(self):
        blocked = {(r, 5) for r in range(GRID_DIM_CELLS)}
        path = plan_grid_path((0, 0), (0, 10), blocked)
        self.assertIsNone(path)

    # ------------------------------------------------------------------
    # Other-robot blocking (typical caller usage)
    # ------------------------------------------------------------------
    def test_other_robot_as_obstacle(self):
        other = (5, 5)
        path = plan_grid_path((5, 0), (5, 10), {other})
        self.assert_valid_path(path, (5, 0), (5, 10))
        self.assertNotIn(other, path)

    def test_start_not_blocked(self):
        start = (3, 3)
        path = plan_grid_path(start, (3, 6), {start})
        self.assert_valid_path(path, start, (3, 6))

    # ------------------------------------------------------------------
    # Path quality
    # ------------------------------------------------------------------
    def test_optimal_length_no_obstacles(self):
        start, goal = (1, 1), (4, 7)
        path = plan_grid_path(start, goal, set())
        expected = max(abs(4 - 1), abs(7 - 1)) + 1  # Chebyshev + 1
        self.assertEqual(len(path), expected)

    def test_no_duplicate_cells(self):
        path = plan_grid_path((0, 0), (5, 5), set())
        self.assertIsNotNone(path)
        self.assertEqual(len(path), len(set(path)), "path contains duplicate cells")


# ------------------------------------------------------------------
# Demo  (python3 test_plan_grid_path.py --demo)
# ------------------------------------------------------------------
def run_demo():
    import matplotlib.pyplot as plt

    cases = [
        ("Open field",            (2, 2),  (8, 9),  set()),
        ("Around a wall",         (0, 0),  (0, 8),  {(r, 4) for r in range(7)}),
        ("Other robot blocking",  (3, 0),  (3, 10), {(3, 5)}),
        ("No path (surrounded)",  (3, 3),  (1, 1),
            {(0, 0), (0, 1), (0, 2), (1, 0), (1, 2), (2, 0), (2, 1), (2, 2), (1, 1)}),
    ]

    fig, axes = plt.subplots(2, 2, figsize=(13, 11))
    fig.suptitle("plan_grid_path  —  8-directional BFS", fontsize=14, fontweight="bold")

    for ax, (label, start, goal, blocked) in zip(axes.flat, cases):
        path = plan_grid_path(start, goal, blocked)
        plot_path(start, goal, blocked, path, label=label, ax=ax)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    if "--demo" in sys.argv:
        sys.argv.remove("--demo")
        run_demo()
    else:
        unittest.main(verbosity=2)
