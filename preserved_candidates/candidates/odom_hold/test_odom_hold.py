"""Pure tests of odom_hold.StandingHold (no ROS, no sleeps: time is the sample stamp). Run: python3 -m pytest -q"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import odom_hold as oh  # noqa: E402

HZ = 150.0
DT = 1.0 / HZ
STAND = (4, 'STAND_DEFAULT', 100, 1)


class Sim:
    """Feeds raw leg samples; raw pose = start + integral of a velocity function, stamps = clock."""

    def __init__(self, hold=None, state=STAND):
        self.h = hold or oh.StandingHold()
        self.t, self.x, self.y, self.yaw = 100.0, 5.0, -3.0, 0.4
        self.out = []                                   # (t, raw x, raw y, pub x, pub y, held)
        self.state = state
        self.next_state = self.t

    def run(self, secs, vx=0.0, vy=0.0, wz=0.0, state=None, stale=False):
        """MC state messages arrive every 0.1 s (stale=True: none arrive)."""
        if state is not None:
            self.state, self.next_state = state, self.t
        for _ in range(int(round(secs * HZ))):
            self.t += DT
            if self.state and not stale and self.t >= self.next_state:
                self.h.set_state(self.t, *self.state)
                self.next_state = self.t + 0.1
            self.x, self.y, self.yaw = self.x + vx * DT, self.y + vy * DT, self.yaw + wz * DT
            px, py, held = self.h.update(self.t, self.t, self.x, self.y, self.yaw)
            self.out.append((self.t, self.x, self.y, px, py, held))
        return self.out[-1]


DRIFT = 1.7e-4          # m/s body-left in the data; here +y


def test_is_standing_truth_table():
    assert oh.is_standing(4, 'STAND_DEFAULT', 100, 1)
    assert not oh.is_standing(5, 'LOCOMOTION_DEFAULT', 100, 1)        # walking, also walking with zero command
    assert not oh.is_standing(5, 'STAND_DEFAULT', 100, 1)             # stopping: fsm still 5
    assert not oh.is_standing(4, 'LOCOMOTION_DEFAULT', 100, 1)        # starting: action first
    assert not oh.is_standing(4, 'STAND_DEFAULT', 200, 1)             # TRANSITION
    assert not oh.is_standing(4, 'STAND_DEFAULT', 0, 1)
    assert not oh.is_standing(4, 'STAND_DEFAULT', 100, 2)             # not body STAND


def test_standing_drift_is_removed_and_offset_absorbs_it():
    s = Sim()
    s.run(60.0, vy=DRIFT)
    t, rx, ry, px, py, held = s.out[-1]
    assert held
    assert ry - -3.0 > 0.0099                                         # raw crept ~1 cm in the minute
    assert abs(py - s.out[int(oh.WINDOW_S * HZ) + 5][4]) < 1e-12      # published frozen after the window filled
    assert abs(px - s.out[0][3]) < 1e-12
    assert abs(s.h.offset() - math.hypot(rx - px, ry - py)) < 1e-12


def test_no_hold_without_state_stale_state_or_other_state():
    for state in (None, (5, 'LOCOMOTION_DEFAULT', 100, 1), (4, 'STAND_DEFAULT', 200, 1)):
        s = Sim(state=state)
        s.run(5.0, vy=DRIFT)
        assert not any(o[5] for o in s.out)
        assert all(o[1] == o[3] and o[2] == o[4] for o in s.out)      # bit-identical pass-through
    s = Sim()
    s.run(2.0)
    assert s.out[-1][5]
    s.run(oh.STATE_MAX_AGE_S + 0.5, stale=True)                        # no new state message
    assert not s.out[-1][5]


def test_pose_gate_beats_standing_state():
    """State says standing but the legs walk (state lags / push): never held."""
    s = Sim()
    s.run(2.0)
    s.run(3.0, vx=0.3)                                                 # state unchanged
    assert not s.out[-1][5]
    moving = [o for o in s.out if o[0] > 102.0 + 0.1]
    assert not any(o[5] for o in moving)


def test_hold_exit_is_continuous_and_increments_are_raw_afterwards():
    s = Sim()
    s.run(10.0, vy=DRIFT)
    n = len(s.out)
    s.run(1.0, vx=0.3, state=(4, 'LOCOMOTION_DEFAULT', 100, 1))
    out = s.out
    release = next(i for i in range(n, len(out)) if not out[i][5])
    assert release - n <= 3                                            # released within a few samples of the start
    for a, b in zip(out[n - 1:], out[n:]):                              # step of the published pose <= raw step
        assert math.hypot(b[3] - a[3], b[4] - a[4]) <= math.hypot(b[1] - a[1], b[2] - a[2]) + 1e-15
    off = {(round(o[1] - o[3], 12), round(o[2] - o[4], 12)) for o in out[release:]}
    assert len(off) == 1                                               # constant offset: raw increments, no jump
    assert s.h.counts['exits'] == 1


def test_hold_entry_is_continuous_after_walking():
    s = Sim(state=(5, 'LOCOMOTION_DEFAULT', 100, 1))
    s.run(2.0, vx=0.3)
    s.run(0.2, state=STAND)
    s.run(3.0, vy=DRIFT)
    out = s.out
    entry = next(i for i, o in enumerate(out) if o[5])
    assert entry > 0 and not out[entry - 1][5]
    assert abs(out[entry][3] - out[entry - 1][3]) <= abs(out[entry][1] - out[entry - 1][1]) + 1e-15   # no jump
    assert out[entry][0] - 102.0 >= oh.WINDOW_S - DT                    # window had to fill first


def test_push_while_standing_releases_and_yaw_change_releases():
    s = Sim()
    s.run(3.0)
    assert s.out[-1][5]
    s.run(0.3, vy=0.1)                                                 # external shove, no state change
    assert not s.out[-1][5]
    s.run(3.0)
    assert s.out[-1][5]
    s.run(0.5, wz=0.5)                                                 # turning while MC still says STAND
    assert not s.out[-1][5]


def test_threshold_boundaries_follow_the_constants():
    for excursion, expect_held in ((0.9 * oh.D_POS_M, True), (1.1 * oh.D_POS_M, False)):
        s = Sim()
        s.run(2.0)
        s.t += DT
        px, py, held = s.h.update(s.t, s.t, s.x, s.y + excursion, s.yaw)      # one-sample step
        assert held is expect_held


def test_yaw_is_never_modified_and_z_not_touched():
    s = Sim()
    s.run(3.0, wz=0.2e-4)                                              # below d_yaw per window: translation held
    assert s.out[-1][5]
    # the hold API returns no yaw at all: the caller publishes the raw MC yaw


def test_gap_and_stamp_back_restart_the_window_but_keep_the_offset():
    s = Sim()
    s.run(5.0, vy=DRIFT)
    off = (s.h.ox, s.h.oy)
    s.t += 0.5                                                         # stream gap > MAX_GAP_S
    s.run(0.1)
    assert not s.out[-1][5] and (s.h.ox, s.h.oy) == off
    s.run(oh.WINDOW_S + 0.1)
    assert s.out[-1][5]
    px, py, held = s.h.update(s.t - 1.0, s.t, s.x, s.y, s.yaw)          # stamp went back
    assert not held
    assert s.h.update(s.t - 0.99, s.t, s.x, s.y, s.yaw)[2] is False


def test_zero_offset_is_bit_identical():
    s = Sim(state=None)
    s.run(1.0, vx=0.2, vy=0.1)
    assert all(o[1] == o[3] and o[2] == o[4] for o in s.out)


def budget(secs):
    return oh.DRIFT_RATE_MAX_MPS * secs + oh.DRIFT_SLACK_M


def hidden(s):
    o = s.out[-1]
    return math.hypot(o[1] - o[3], o[2] - o[4])


def test_real_drift_is_held_for_the_whole_episode():
    s = Sim()
    s.run(2.0)
    s.run(300.0, vy=DRIFT)                      # real drift 0.17 mm/s for 5 min: always inside the budget
    assert s.out[-1][5] and s.h.counts.get('cap_releases', 0) == 0
    assert hidden(s) < budget(302.0)


def test_slow_ramp_released_and_latched_for_the_episode():
    for rate in (0.4e-3, 0.55e-3):             # both pass the 0.5 s window gate (< 0.9 mm/s incl. the 0.17 mm/s drift)
        s = Sim()
        s.run(2.0)
        t0 = s.h.t_ep
        for _ in range(600):                    # 60 s in 0.1 s steps; stop at the release
            s.run(0.1, vy=rate + DRIFT)
            if s.h.capped:
                break
        t_rel = s.out[-1][0] - t0
        h_rel = hidden(s)
        assert s.h.counts['cap_releases'] == 1 and t_rel < 20.0
        assert h_rel <= budget(t_rel) + 1e-3 * 0.2e-3          # hidden at release = the budget (+ one sample of ramp)
        s.run(60.0 - t_rel, vy=rate + DRIFT)
        assert abs(hidden(s) - h_rel) < 1e-12                            # frozen: nothing more is absorbed
        assert s.h.capped and not s.h.held
        assert s.h.counts['entries'] == s.h.counts['exits'] == 1      # no re-entry after the release
        assert s.out[-1][4] - s.out[0][4] > 0.9 * rate * 40           # the ramp is visible afterwards


def test_budget_is_cumulative_over_hold_reentries():
    """Ramp that repeatedly drops below the window gate (stop/go): every hold run adds to the same budget."""
    s = Sim()
    s.run(2.0)
    for _ in range(200):
        s.run(2.5, vy=0.85e-3)                  # absorbed while quiet (hold); 0.85 mm/s is just inside the window gate
        s.run(0.3, vy=3e-3)                     # short jerk breaks the quiet window: hold exits, motion passes through
        if s.h.capped:
            break
    assert s.h.counts['entries'] >= 2 and s.h.counts['cap_releases'] == 1
    entries = s.h.counts['entries']
    s.run(5.0)                                  # fully quiet afterwards: latched, no re-entry in this episode
    assert s.h.counts['entries'] == entries and not s.h.held
    assert hidden(s) <= budget(s.out[-1][0] - s.h.t_ep)


def test_new_episode_after_leaving_standing_resets_latch_keeps_offset():
    s = Sim()
    s.run(2.0)
    s.run(20.0, vy=0.8e-3)
    assert s.h.capped
    off = (s.h.ox, s.h.oy)
    s.run(1.0, state=(5, 'LOCOMOTION_DEFAULT', 100, 1))       # leaves standing: episode over
    assert not s.h.capped and not s.h.in_episode
    assert (s.h.ox, s.h.oy) == off
    s.run(3.0, state=STAND)                                   # back: new episode, hold works again, budget restarts
    assert s.h.in_episode and s.h.held and not s.h.capped
    assert math.hypot(s.h.ox - s.h.ox_ep, s.h.oy - s.h.oy_ep) <= budget(3.0)


def test_stale_state_ends_the_episode():
    s = Sim()
    s.run(2.0)
    s.run(20.0, vy=0.8e-3)
    assert s.h.capped
    s.run(2.0, stale=True)                                    # no state messages for > 1 s
    assert not s.h.capped and not s.h.in_episode and not s.h.held
    s.run(3.0)                                                # state resumes
    assert s.h.held and not s.h.capped


def test_release_is_jump_free_and_yaw_untouched():
    s = Sim()
    s.run(2.0)
    s.run(20.0, vy=0.8e-3, wz=3e-4)
    assert s.h.capped
    steps = [(b[3] - a[3], b[4] - a[4], b[1] - a[1], b[2] - a[2]) for a, b in zip(s.out, s.out[1:]) if not b[5]]
    assert all(abs(p - r) < 1e-12 and abs(q - w) < 1e-12 for p, q, r, w in steps)
