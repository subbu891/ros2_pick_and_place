import pytest

from pnp_task.gz_grasp_bridge import nearest
from pnp_task.sorting_logic import (Bin, ObjectTracker, Observation, pick_was_verified,
                                    select_next_target, Workspace)

# Same bins as in pnp_world.sdf
BINS = {
    'red': Bin('red', 0.30, 0.35),
    'green': Bin('green', 0.45, 0.35),
    'blue': Bin('blue', 0.60, 0.35),
}


def feed(tracker, obs, n):
    for _ in range(n):
        tracker.update(obs)


def test_track_becomes_stable_after_min_hits():
    tr = ObjectTracker(min_hits=3)
    obs = [Observation('red', 0.5, 0.0, 0.42)]
    feed(tr, obs, 2)
    assert tr.stable() == []
    feed(tr, obs, 1)
    assert [t.track_id for t in tr.stable()] == ['red_0']


def test_association_keeps_id_under_noise():
    tr = ObjectTracker(match_radius=0.03)
    tr.update([Observation('blue', 0.50, 0.00, 0.42)])
    tr.update([Observation('blue', 0.51, 0.005, 0.42)])
    assert list(tr.tracks) == ['blue_0']
    assert tr.tracks['blue_0'].hits == 2


def test_colours_are_never_merged():
    tr = ObjectTracker()
    tr.update([Observation('red', 0.5, 0.0, 0.42), Observation('green', 0.5, 0.0, 0.42)])
    assert sorted(tr.tracks) == ['green_0', 'red_0']


def test_track_is_dropped_after_max_misses():
    tr = ObjectTracker(max_misses=2)
    tr.update([Observation('red', 0.5, 0.0, 0.42)])
    feed(tr, [], 3)
    assert tr.tracks == {}


def test_smoothing_moves_towards_measurement():
    tr = ObjectTracker(alpha=0.5)
    tr.update([Observation('red', 0.50, 0.0, 0.42)])
    tr.update([Observation('red', 0.52, 0.0, 0.42)])
    assert tr.tracks['red_0'].x == pytest.approx(0.51)


def test_select_prefers_closest_object_to_robot():
    tr = ObjectTracker(min_hits=1)
    tr.update([Observation('red', 0.70, 0.0, 0.42), Observation('blue', 0.35, -0.1, 0.42)])
    assert select_next_target(tr.stable(), BINS, Workspace()).color == 'blue'


def test_select_skips_sorted_and_blacklisted():
    tr = ObjectTracker(min_hits=1)
    tr.update([
        Observation('red', 0.30, 0.35, 0.42),     # already in the red bin
        Observation('green', 0.40, 0.0, 0.42),    # blacklisted below
        Observation('blue', 0.60, 0.0, 0.42),
    ])
    t = select_next_target(tr.stable(), BINS, Workspace(), blacklist={'green_0'})
    assert t.track_id == 'blue_0'


def test_select_ignores_unreachable_and_unknown_colours():
    tr = ObjectTracker(min_hits=1)
    tr.update([Observation('yellow', 0.5, 0.0, 0.42), Observation('red', 1.5, 0.0, 0.42)])
    assert select_next_target(tr.stable(), BINS, Workspace()) is None


def test_bin_slots_are_distinct_inside_and_cubes_do_not_touch():
    b = BINS['green']
    slots = [b.slot(i) for i in range(4)]
    assert len(set(slots)) == 4
    half = 0.02   # half a cube
    assert all(b.contains(x + sx * half, y + sy * half)
               for x, y in slots for sx in (-1, 1) for sy in (-1, 1))
    for i, (x1, y1) in enumerate(slots):
        for x2, y2 in slots[i + 1:]:
            assert max(abs(x1 - x2), abs(y1 - y2)) > 2 * half + 0.01


def test_verification_fails_if_object_still_visible():
    tr = ObjectTracker(min_hits=1)
    obs = [Observation('green', 0.35, 0.10, 0.42)]
    tr.update(obs)
    tr.update(obs)   # still there after a "successful" pick
    assert not pick_was_verified(tr, 'green_0')


def test_verification_passes_once_object_disappears():
    tr = ObjectTracker(min_hits=1, max_misses=1)
    tr.update([Observation('green', 0.35, 0.10, 0.42)])
    tr.update([])
    assert pick_was_verified(tr, 'green_0')
    tr.update([])    # dropped entirely
    assert pick_was_verified(tr, 'green_0')


def test_nearest_cube_for_grasp_bridge():
    poses = {'cube_a': (0.5, 0.0, 0.42), 'cube_b': (0.6, 0.1, 0.42)}
    name, d = nearest(poses, 0.59, 0.1, 0.43)
    assert name == 'cube_b'
    assert d == pytest.approx(0.01414, abs=1e-4)
    assert nearest({}, 0, 0, 0)[0] is None
