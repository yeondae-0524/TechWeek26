"""Bounded exploration recovery after the controller's obstacle wait.

Replan the same goal, then temporarily exclude its neighbourhood. This module
never clears occupancy or issues motor commands. No reverse/spin escape stage.
"""
import math
from collections import deque

from interfaces import FREE


def reachable_cells(grid, start):
    """Known-free 4-connected component used to reject impossible retry goals."""
    height, width = len(grid), len(grid[0]) if grid else 0
    row, col = start
    if not (0 <= row < height and 0 <= col < width) or grid[row][col] != FREE:
        return set()
    reached = {start}
    pending = deque([start])
    while pending:
        row, col = pending.popleft()
        for cell in ((row - 1, col), (row + 1, col), (row, col - 1), (row, col + 1)):
            rr, cc = cell
            if (0 <= rr < height and 0 <= cc < width
                    and cell not in reached and grid[rr][cc] == FREE):
                reached.add(cell)
                pending.append(cell)
    return reached


class RecoveryLadder:
    def __init__(self, *, replan_delay, blacklist_duration, blacklist_radius, max_replans=2):
        if any(not math.isfinite(v) or v <= 0 for v in
               (replan_delay, blacklist_duration, blacklist_radius)):
            raise ValueError("recovery durations and radius must be positive")
        if not isinstance(max_replans, int) or max_replans < 0:
            raise ValueError("max_replans must be a nonnegative integer")
        self.replan_delay = replan_delay
        self.blacklist_duration = blacklist_duration
        self.blacklist_radius = blacklist_radius
        self.max_replans = max_replans
        self.blacklist = []
        self.reset()

    def reset(self):
        """Cancel the current goal, retaining temporary exclusions."""
        self.goal = None
        self.attempts = 0
        self.ready_at = 0.0
        self.stage = "IDLE"

    def begin(self, goal):
        self.goal = tuple(goal)
        self.attempts = 0
        self.stage = "FOLLOW"

    def fail(self, now):
        if self.goal is None:
            return
        if self.attempts < self.max_replans:
            self.attempts += 1
            self.ready_at = now + self.replan_delay
            self.stage = "WAIT_REPLAN"
        else:
            self.exclude(self.goal, now)
            self.reset()
            self.ready_at = now + self.replan_delay
            self.stage = "CHOOSE_GOAL"

    def planned(self):
        # A new path is not evidence of physical progress. Keep the retry count.
        self.stage = "FOLLOW"

    def exclude(self, goal, now):
        self.blacklist.append((tuple(goal), now + self.blacklist_duration))

    def excluded(self, goal, now):
        self.blacklist[:] = [(xy, expiry) for xy, expiry in self.blacklist if expiry > now]
        return any(math.dist(goal, xy) <= self.blacklist_radius for xy, _ in self.blacklist)
