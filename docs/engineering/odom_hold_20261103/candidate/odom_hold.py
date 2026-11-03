"""Standing hold for the native (MC leg odometry) odom -- pure logic, no ROS. Used by native_odom_bridge.py.

Problem (reports/20261103-d04b-drift.md): while the robot stands (MC fsm 4, STAND_DEFAULT) the vendor leg-odometry
pose creeps body-left at a constant ~1.0 cm/min (1.7e-4 m/s); it is passed 1:1 to odom->base_footprint, so every
odom-fixed costmap mark creeps relative to the robot.

Rule. The published translation is FROZEN (held) while BOTH hold:
  1. MC says standing: fsm_state == STAND_FSM (4), action_desc == 'STAND_DEFAULT', action status == RUNNING (100),
     body_status == BODY_STAND (1), and the state message is not older than STATE_MAX_AGE_S.
     (In the 11-02 bags every locomotion segment, incl. Nav2 goals and RC walking, has fsm 5 and/or
     LOCOMOTION_DEFAULT and/or status TRANSITION 200; walking with zero command stays in 5/LOCOMOTION_DEFAULT.)
  2. The raw leg pose itself is quiet: over the trailing WINDOW_S the position moved <= D_POS_M and the yaw
     <= D_YAW_RAD from every sample of the window. The state alone is NOT enough: (a) after a walk the state returns
     to 4/STAND_DEFAULT/100 while the legs still settle for ~1 s, (b) an external push of the standing robot
     (nav_bag_183504, 25 mm / 46 mrad in 1 s) changes no MC state field at all. The pose gate also releases the hold
     within ~10-30 ms of a locomotion start, ahead of the 10 Hz state message.
D_POS_M / D_YAW_RAD are the MAXIMA of the 0.5 s window excursion measured in the quiet standing data of the 11-02
bags (see reports/20261103-odom-standing-hold.md), not tuned values; a noise outlier above them only releases the hold
(pass-through = today's behaviour), it can never create motion.

Hold = no accumulation of translation: the published position is p_hold while held; `offset = raw - published` is
updated every held sample (it absorbs the vendor drift) and kept constant otherwise, so published = raw - offset is
continuous at hold entry and exit (no jump) and has exactly the raw increments whenever it is not held. Yaw is never
held: it is the raw MC yaw always. While held the twist is reported as zero.
"""
import math
from collections import deque

#: MC /aima/mc/common/state values (the numbering observed on the robot, docs/MC_BALANCE_NAV2_CONTRACT.md: 4 = STABLE).
STAND_FSM = 4
STAND_ACTION = 'STAND_DEFAULT'
ACTION_RUNNING = 100               # McActionStatus RUNNING (200 = TRANSITION, 0 = IDLE)
BODY_STAND = 1
#: MC state stream: 10 Hz, observed max gap 0.50 s over 4 h of bags (p99.9 0.39 s); twice that = stale.
STATE_MAX_AGE_S = 1.0
#: Quiet window and excursion bounds: maxima over 1.09 M quiet standing samples (11-02), window 0.5 s:
#: position 0.450 mm, yaw 199 urad (p99: 0.29 mm / 9 urad); 13 bags, 1.09 M samples, 4 h.
WINDOW_S = 0.5
D_POS_M = 4.5e-4
D_YAW_RAD = 2.0e-4
#: Cumulative-offset cap (verifier F2): a steady ramp < D_POS_M/WINDOW_S = 0.9 mm/s passes the window gate forever.
#: The hold is released when the offset absorbed since hold entry exceeds DRIFT_RATE_MAX_MPS * held_time + DRIFT_SLACK_M.
#: DRIFT_RATE_MAX_MPS = 2 x 1.9e-4 m/s (max measured held drift, 0.19 mm/s, nav_bag_183504, 155.7 s run) = 0.38 mm/s;
#: after a cap release the hold stays off (latched) until the stand/quiet condition drops once;
#: slack 1 mm covers window noise (<= 0.45 mm) at entry. Hidden motion is thereby bounded to ~2x the real drift rate.
DRIFT_RATE_MAX_MPS = 3.8e-4
DRIFT_SLACK_M = 1.0e-3
SAMPLE_SPACING_S = 0.01            # window storage thinning (leg odometry runs at ~150 Hz)
MAX_GAP_S = 0.2                    # source stream gap: restart the window (native_odom.NATIVE_MAX_AGE)


def is_standing(fsm, action, status, body):
    """Explicit MC standing signal (fields of /aima/mc/common/state)."""
    return fsm == STAND_FSM and action == STAND_ACTION and status == ACTION_RUNNING and body == BODY_STAND


class StandingHold:
    """set_state(now, ...) on each MC state message, update(stamp, now, x, y, yaw) on each accepted leg sample
    (raw x, y in leg_odom, unwrapped yaw). update returns (x_pub, y_pub, held)."""

    def __init__(self, window=WINDOW_S, d_pos=D_POS_M, d_yaw=D_YAW_RAD, state_max_age=STATE_MAX_AGE_S,
                 spacing=SAMPLE_SPACING_S):
        self.window, self.d_pos, self.d_yaw, self.state_max_age, self.spacing = window, d_pos, d_yaw, state_max_age, spacing
        self.ox = self.oy = 0.0            # raw - published
        self.ring = deque()                # (stamp, x, y, yaw) raw, thinned
        self.standing, self.state_t = False, float('-inf')
        self.held, self.hx, self.hy = False, 0.0, 0.0
        self.capped = False
        self.t_enter = self.ox_enter = self.oy_enter = 0.0
        self.counts = dict(held_samples=0, entries=0, exits=0)

    def set_state(self, now, fsm, action, status, body):
        self.standing, self.state_t = is_standing(fsm, action, status, body), now

    def clear_window(self):
        """Stream discontinuity (stamp back, long gap): the quiet window restarts; the offset is kept."""
        self.ring.clear()

    def quiet(self, stamp, x, y, yaw):
        """True when the raw pose stayed within (d_pos, d_yaw) of every window sample, the window being full."""
        ring = self.ring
        if len(ring) < 2 or stamp - ring[0][0] < self.window:
            return False
        return all(math.hypot(x - rx, y - ry) <= self.d_pos and abs(yaw - ra) <= self.d_yaw
                   for _, rx, ry, ra in ring)

    def _gap(self, stamp):
        ring = self.ring
        if ring and (stamp <= ring[-1][0] or stamp - ring[-1][0] > MAX_GAP_S):
            ring.clear()

    def _push(self, stamp, x, y, yaw):
        ring = self.ring
        if not ring or stamp - ring[-1][0] >= self.spacing:
            ring.append((stamp, x, y, yaw))
        while len(ring) > 1 and stamp - ring[1][0] >= self.window:
            ring.popleft()

    def update(self, stamp, now, x, y, yaw):
        self._gap(stamp)
        hold = self.standing and now - self.state_t <= self.state_max_age and self.quiet(stamp, x, y, yaw)
        if not hold:
            self.capped = False             # latch ends when the stand/quiet condition itself ends
        elif self.capped:
            hold = False                    # cap-released: stay pass-through until standing/quiet is re-established
        self._push(stamp, x, y, yaw)
        if hold and self.held:
            absorbed = math.hypot(x - self.hx - self.ox_enter, y - self.hy - self.oy_enter)
            if absorbed > DRIFT_RATE_MAX_MPS * (stamp - self.t_enter) + DRIFT_SLACK_M:
                hold = False
                self.capped = True
                self.counts['cap_releases'] = self.counts.get('cap_releases', 0) + 1
        if hold:
            if not self.held:
                self.held, self.hx, self.hy = True, x - self.ox, y - self.oy      # freeze where it is published now
                self.t_enter, self.ox_enter, self.oy_enter = stamp, self.ox, self.oy
                self.counts['entries'] += 1
            self.ox, self.oy = x - self.hx, y - self.hy
            self.counts['held_samples'] += 1
        elif self.held:
            self.held = False
            self.counts['exits'] += 1
        return x - self.ox, y - self.oy, self.held

    def offset(self):
        return math.hypot(self.ox, self.oy)
