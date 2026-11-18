from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Dict, Iterable, List, Optional, Tuple


@dataclass(frozen=True)
class Observation:
    """One detection in the world frame."""

    color: str
    x: float
    y: float
    z: float
    yaw: float = 0.0


@dataclass
class Track:
    """An object followed over several camera frames."""

    track_id: str
    color: str
    x: float
    y: float
    z: float
    yaw: float
    hits: int = 1       # how often it was seen
    misses: int = 0     # consecutive frames it was NOT seen

    def update(self, obs: Observation, alpha: float) -> None:
        # Smooth the position (exponential moving average) to reduce noise
        self.x += alpha * (obs.x - self.x)
        self.y += alpha * (obs.y - self.y)
        self.z += alpha * (obs.z - self.z)
        self.yaw = obs.yaw
        self.hits += 1
        self.misses = 0


@dataclass(frozen=True)
class Bin:
    color: str
    x: float
    y: float
    size: float = 0.13   # inner width [m]

    def contains(self, x: float, y: float) -> bool:
        half = self.size / 2.0
        return abs(x - self.x) <= half and abs(y - self.y) <= half

    def slot(self, index: int, spacing: float = 0.055, cols: int = 2,
             rows: int = 2) -> Tuple[float, float]:
        """Position for the index-th cube: a 2x2 grid, so cubes never touch.

        spacing must exceed the cube size + placement error (40 mm + ~10 mm).
        """
        row, col = divmod(index % (cols * rows), cols)
        ox = (col - (cols - 1) / 2.0) * spacing
        oy = (row - (rows - 1) / 2.0) * spacing
        return self.x + ox, self.y + oy


@dataclass(frozen=True)
class Workspace:
    """Rectangle on the table the robot may pick from (world frame)."""

    x_min: float = 0.25
    x_max: float = 0.75
    y_min: float = -0.40
    y_max: float = 0.22

    def contains(self, x: float, y: float) -> bool:
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max


@dataclass
class ObjectTracker:
    """Nearest-neighbour tracking with hit / miss counting."""

    match_radius: float = 0.03   # max movement between frames to count as "same object"
    min_hits: int = 3            # sightings needed before a track is trusted
    max_misses: int = 5          # consecutive misses before a track is dropped
    alpha: float = 0.5           # smoothing factor
    tracks: Dict[str, Track] = field(default_factory=dict)
    _next_id: Dict[str, int] = field(default_factory=dict)

    def update(self, observations: Iterable[Observation]) -> None:
        unmatched = set(self.tracks)
        for obs in observations:
            # Find the closest existing track of the same colour
            best, best_d = None, self.match_radius
            for tid in unmatched:
                t = self.tracks[tid]
                if t.color != obs.color:
                    continue
                d = math.hypot(t.x - obs.x, t.y - obs.y)
                if d <= best_d:
                    best, best_d = tid, d
            if best is not None:
                self.tracks[best].update(obs, self.alpha)
                unmatched.discard(best)
            else:
                # A new object: give it a new stable ID
                n = self._next_id.get(obs.color, 0)
                self._next_id[obs.color] = n + 1
                tid = f'{obs.color}_{n}'
                self.tracks[tid] = Track(tid, obs.color, obs.x, obs.y, obs.z, obs.yaw)
        # Tracks not seen this frame
        for tid in unmatched:
            self.tracks[tid].misses += 1
            if self.tracks[tid].misses > self.max_misses:
                del self.tracks[tid]

    def stable(self) -> List[Track]:
        return [t for t in self.tracks.values() if t.hits >= self.min_hits]

    def remove(self, track_id: str) -> None:
        self.tracks.pop(track_id, None)


def select_next_target(
    tracks: Iterable[Track],
    bins: Dict[str, Bin],
    workspace: Workspace,
    blacklist: Iterable[str] = (),
    robot_xy: Tuple[float, float] = (0.0, 0.0),
) -> Optional[Track]:
