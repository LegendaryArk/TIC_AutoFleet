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
plan_grid_path_spacetime = central_arbiter.plan_grid_path_spacetime
chebyshev = central_arbiter.chebyshev
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
# Space-time A* tests
# ------------------------------------------------------------------
class TestPlanGridPathSpacetime(unittest.TestCase):

    def assert_valid_st_path(self, path, start, goal):
        """Connected (8-dir + stay-in-place), in-bounds, starts at start, ends at goal."""
        self.assertIsNotNone(path)
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)
        for r, c in path:
            self.assertGreaterEqual(r, 0)
            self.assertGreaterEqual(c, 0)
            self.assertLess(r, GRID_DIM_CELLS)
            self.assertLess(c, GRID_DIM_CELLS)
        for (r0, c0), (r1, c1) in zip(path, path[1:]):
            self.assertLessEqual(
                max(abs(r1 - r0), abs(c1 - c0)), 1,
                "non-adjacent / non-wait step in path",
            )

    def assert_clearance_maintained(self, path_two, path_one, clearance):
        """Chebyshev separation > clearance at every timestep (goal cell exempt)."""
        last = len(path_one) - 1
        goal = path_two[-1]
        for t, cell_two in enumerate(path_two):
            if cell_two == goal:
                break
            cell_one = path_one[min(t, last)]
            dist = chebyshev(cell_two, cell_one)
            self.assertGreater(
                dist, clearance,
                f"Clearance violated at t={t}: robot2={cell_two} robot1={cell_one} dist={dist}",
            )

    # ------------------------------------------------------------------
    # Trivial / degenerate cases
    # ------------------------------------------------------------------
    def test_st_start_equals_goal(self):
        path = plan_grid_path_spacetime((5, 5), (5, 5), [(0, 0)], clearance=1)
        self.assertEqual(path, [(5, 5)])

    def test_st_empty_other_path_falls_back(self):
        # No robot 1 path — should still find a valid path via the fallback.
        path = plan_grid_path_spacetime((0, 0), (0, 5), [], clearance=1)
        self.assert_valid_st_path(path, (0, 0), (0, 5))

    # ------------------------------------------------------------------
    # No-conflict cases
    # ------------------------------------------------------------------
    def test_st_no_conflict_direct_path(self):
        # Robot 1 travels row 0; robot 2 is far away in row 10 — no overlap.
        path_one = [(0, c) for c in range(6)]
        path_two = plan_grid_path_spacetime((10, 0), (10, 5), path_one, clearance=1)
        self.assert_valid_st_path(path_two, (10, 0), (10, 5))
        # No conflict means robot 2 takes the direct Chebyshev-optimal path.
        self.assertEqual(len(path_two), 6)

    def test_st_parallel_paths_no_wait(self):
        # Both robots travel in the same direction but separate rows (gap > clearance).
        path_one = [(5, c) for c in range(8)]   # row 5
        start_two, goal_two = (0, 0), (0, 7)    # row 0 — 5 rows away
        path_two = plan_grid_path_spacetime(start_two, goal_two, path_one, clearance=2)
        self.assert_valid_st_path(path_two, start_two, goal_two)
        self.assert_clearance_maintained(path_two, path_one, clearance=2)
        self.assertEqual(len(path_two), 8)  # direct, no wait steps needed

    # ------------------------------------------------------------------
    # Clearance enforcement
    # ------------------------------------------------------------------
    def test_st_clearance_1_crossing_paths(self):
        # Robot 1 moves horizontally through row 5; robot 2 crosses vertically
        # through column 3 — their naïve paths share cell (5, 3).
        path_one = [(5, c) for c in range(8)]   # (5,0)..(5,7)
        start_two, goal_two = (2, 3), (8, 3)
        path_two = plan_grid_path_spacetime(start_two, goal_two, path_one, clearance=1)
        self.assert_valid_st_path(path_two, start_two, goal_two)
        self.assert_clearance_maintained(path_two, path_one, clearance=1)

    def test_st_clearance_2_crossing_paths(self):
        # Wider 2-cell buffer — robot 2 must stay at least 3 cells from robot 1.
        path_one = [(5, c) for c in range(8)]
        start_two, goal_two = (0, 3), (9, 3)
        path_two = plan_grid_path_spacetime(start_two, goal_two, path_one, clearance=2)
        self.assert_valid_st_path(path_two, start_two, goal_two)
        self.assert_clearance_maintained(path_two, path_one, clearance=2)

    def test_st_stationary_robot1_acts_as_permanent_obstacle(self):
        # Robot 1 is a single-cell path — it never moves.
        # Robot 2 must route around the clearance zone for the entire journey.
        path_one = [(5, 5)]
        start_two, goal_two = (5, 0), (5, 10)
        path_two = plan_grid_path_spacetime(start_two, goal_two, path_one, clearance=2)
        self.assert_valid_st_path(path_two, start_two, goal_two)
        self.assert_clearance_maintained(path_two, path_one, clearance=2)
        # No cell in the path (except goal) may be within 2 of (5,5).
        for cell in path_two[:-1]:
            self.assertGreater(chebyshev(cell, (5, 5)), 2)

    # ------------------------------------------------------------------
    # Wait-step behaviour
    # ------------------------------------------------------------------
    def test_st_conflict_at_crossing_is_resolved(self):
        # Robot 1 moves rightward from (5,3); robot 2's direct path south through
        # col 3 conflicts at (5,3).  8-connectivity allows a same-length diagonal
        # detour, so we assert the path is valid and clearance is maintained.
        path_one = [(5, c) for c in range(3, 10)]
        start_two, goal_two = (3, 3), (7, 3)
        path_two = plan_grid_path_spacetime(start_two, goal_two, path_one, clearance=1)
        self.assert_valid_st_path(path_two, start_two, goal_two)
        self.assert_clearance_maintained(path_two, path_one, clearance=1)

    def test_st_head_on_route_blocked_finds_alternate(self):
        # Robot 1 sweeps row 5 left-to-right.  Robot 2 starts 5 rows north of
        # robot 1's start (distance=5 > clearance=1, so no initial conflict) and
        # heads south along col 5 — the direct path hits (5,5) at t=5, exactly
        # when robot 1 is there.  The planner must find a collision-free route.
        path_one = [(5, c) for c in range(10)]
        start_two, goal_two = (0, 5), (9, 5)
        path_two = plan_grid_path_spacetime(start_two, goal_two, path_one, clearance=1)
        self.assert_valid_st_path(path_two, start_two, goal_two)
        self.assert_clearance_maintained(path_two, path_one, clearance=1)

    # ------------------------------------------------------------------
    # Bounds and robustness
    # ------------------------------------------------------------------
    def test_st_path_stays_in_bounds(self):
        # Robot 1 occupies column 0 — robot 2 must travel down column 2 near the edge.
        path_one = [(r, 0) for r in range(GRID_DIM_CELLS)]
        start_two, goal_two = (0, 2), (GRID_DIM_CELLS - 1, 2)
        path_two = plan_grid_path_spacetime(start_two, goal_two, path_one, clearance=1)
        self.assert_valid_st_path(path_two, start_two, goal_two)
        for r, c in path_two:
            self.assertGreaterEqual(r, 0)
            self.assertGreaterEqual(c, 0)
            self.assertLess(r, GRID_DIM_CELLS)
            self.assertLess(c, GRID_DIM_CELLS)

    def test_st_no_path_within_max_t(self):
        # max_t=1 with a long journey guarantees no solution is found in time.
        path_one = [(0, 0)]
        path_two = plan_grid_path_spacetime((0, 5), (0, 30), path_one, clearance=0, max_t=1)
        self.assertIsNone(path_two)


# ------------------------------------------------------------------
# Demo  (python3 test_plan_grid_path.py --demo)
# ------------------------------------------------------------------
def plot_spacetime_paths(
    path_one: list[tuple[int, int]],
    path_two: list[tuple[int, int]],
    *,
    label: str = "",
    clearance: int = 1,
    ax=None,
) -> None:
    """Visualise two robot paths on the same grid, colour-coded."""
    import numpy as np
    import matplotlib.pyplot as plt

    standalone = ax is None
    if standalone:
        fig, ax = plt.subplots(figsize=(8, 8))

    all_cells = set(path_one) | set(path_two)
    if not all_cells:
        return

    rows = [r for r, c in all_cells]
    cols = [c for r, c in all_cells]
    pad = clearance + 1
    r_min = max(0, min(rows) - pad)
    r_max = min(GRID_DIM_CELLS - 1, max(rows) + pad)
    c_min = max(0, min(cols) - pad)
    c_max = min(GRID_DIM_CELLS - 1, max(cols) + pad)

    height = r_max - r_min + 1
    width  = c_max - c_min + 1

    img = np.full((height, width, 3), 0.93)

    set_one = set(path_one)
    set_two = set(path_two)

    for r in range(r_min, r_max + 1):
        for c in range(c_min, c_max + 1):
            ri, ci = r - r_min, c - c_min
            cell = (r, c)
            in_one = cell in set_one
            in_two = cell in set_two
            if in_one and in_two:
                img[ri, ci] = [0.85, 0.45, 0.85]  # purple — shared cell
            elif in_one:
                img[ri, ci] = [0.35, 0.65, 1.00]  # blue — robot 1
            elif in_two:
                img[ri, ci] = [1.00, 0.65, 0.20]  # orange — robot 2

    ax.imshow(img, origin="upper", aspect="equal",
              extent=[c_min - 0.5, c_max + 0.5, r_max + 0.5, r_min - 0.5])

    for c in range(c_min, c_max + 2):
        ax.axvline(c - 0.5, color="white", linewidth=0.4, zorder=1)
    for r in range(r_min, r_max + 2):
        ax.axhline(r - 0.5, color="white", linewidth=0.4, zorder=1)

    def draw_path(path, color, label_char):
        if len(path) > 1:
            for (r0, c0), (r1, c1) in zip(path, path[1:]):
                if (r0, c0) != (r1, c1):  # skip wait arrows
                    ax.annotate("", xy=(c1, r1), xytext=(c0, r0),
                                arrowprops=dict(arrowstyle="->", color=color, lw=1.2), zorder=2)
        sr, sc = path[0]
        gr, gc = path[-1]
        ax.text(sc, sr, f"{label_char}S", ha="center", va="center",
                fontsize=8, fontweight="bold", color="white", zorder=3)
        ax.text(gc, gr, f"{label_char}G", ha="center", va="center",
                fontsize=8, fontweight="bold", color="white", zorder=3)

    draw_path(path_one, "navy",   "1")
    draw_path(path_two, "saddlebrown", "2")

    ax.set_xticks(range(c_min, c_max + 1))
    ax.set_yticks(range(r_min, r_max + 1))
    ax.tick_params(labelsize=7)
    ax.invert_yaxis()
    ax.set_title(f"{label}  [R1:{len(path_one)} cells  R2:{len(path_two)} cells]", fontsize=10)

    if standalone:
        plt.tight_layout()
        plt.show()


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
    fig.suptitle("plan_grid_path  —  8-directional A*", fontsize=14, fontweight="bold")

    for ax, (label, start, goal, blocked) in zip(axes.flat, cases):
        path = plan_grid_path(start, goal, blocked)
        plot_path(start, goal, blocked, path, label=label, ax=ax)

    plt.tight_layout()
    plt.show()

    # Space-time demo
    st_cases = [
        (
            "Crossing paths (clearance=1)",
            [(5, c) for c in range(8)], (2, 3), (8, 3), 1,
        ),
        (
            "Head-on, wider clearance=2",
            [(5, c) for c in range(8)], (0, 3), (9, 3), 2,
        ),
        (
            "Stationary obstacle (clearance=2)",
            [(5, 5)], (5, 0), (5, 10), 2,
        ),
        (
            "Robot 2 must wait (clearance=1)",
            [(5, c) for c in range(3, 10)], (3, 3), (7, 3), 1,
        ),
    ]

    fig2, axes2 = plt.subplots(2, 2, figsize=(14, 12))
    fig2.suptitle("plan_grid_path_spacetime  —  space-time A* with N-cell clearance",
                  fontsize=13, fontweight="bold")

    for ax, (label, path_one, start_two, goal_two, cl) in zip(axes2.flat, st_cases):
        path_two = plan_grid_path_spacetime(start_two, goal_two, path_one, clearance=cl)
        plot_spacetime_paths(path_one, path_two or [], label=label, clearance=cl, ax=ax)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    if "--demo" in sys.argv:
        sys.argv.remove("--demo")
        run_demo()
    else:
        unittest.main(verbosity=2)
