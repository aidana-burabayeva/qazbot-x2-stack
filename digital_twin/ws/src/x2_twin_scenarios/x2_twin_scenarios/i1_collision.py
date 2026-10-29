"""Scenario I1: false 'RPP detected collision ahead' from stale local-costmap marks (physical-X2 Goal01) and its regression checks.

Every state is proven along the whole chain  command -> plant -> LaserScan -> ObstacleLayer -> costmap -> Nav2/RPP:
  * a command is published ONCE with a sequence number; the plant acknowledges it with its applied state (no re-sending: a missing
    acknowledgement is a delivery failure and fails the run);
  * the scan is read back (finite / +inf / NaN at the bearings of interest);
  * the costmap is read at the expected SURFACE of the obstacle (the arc facing the sensor), not at its geometric centre;
  * Nav2's result, the motion and the non-zero MC commands are recorded.
Runs against a stack with ``localization:=truth`` (AMCL/Trust are not the subject), a start pose with an OPEN sector in the scan FOV
(no return within range_max: only there can the bridge publish +inf and a stale mark persist) and a plant scan sector wider than the
observed-FOV mask (scan_fov_deg 130) so that returns beyond the mask edge exist. Phases:
  A real obstacle      detected in scan and costmap surface; a goal through it must NOT be driven;
  C scan loss          removed while the scan is absent, then 'empty' (no returns at all): the mark must stay;
  B removed in FOV     scans back: the FOV bearing reads observed-free; candidate: the region is cleared, legacy: stale (the bug);
  D outside the FOV    an obstacle beyond the mask edge, then removed: reads unknown (NaN) and its mark must stay;
  E free route         a goal along the open bearing: no false collision -> moves (candidate) / refused with zero motion (legacy).
Phase wall-clock windows are written to results.json for per-phase counting of 'detected collision ahead' in the controller log.
``--mode baseline`` records the legacy behaviour (soft checks), ``--mode candidate`` hard-checks the fix.

  ros2 run x2_twin_scenarios i1_collision --name NAME --mode baseline|candidate --start x,y,yaw
"""
import argparse
import json
import math
import pathlib
import sys
import time

import numpy as np
import rclpy
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from x2_twin_scenarios.harness import Harness, SUCCEEDED
from x2_twin_sim.level_scan import FOV_DEG
from x2_twin_sim.occupancy import OccupancyMap

ROBOT_R, GHOST_R = 0.454, 0.15


def open_run(scan_msg, lo_deg: float, hi_deg: float, min_bins: int):
    """Widest run of bins WITHOUT a return (not finite) between lo_deg and hi_deg of the live scan -> (centre_deg, width_deg) or None.
    Read from the scan the stack really receives, not from the map: the spawn pose differs from the nominal one by a few cm/deg."""
    r = np.array(scan_msg.ranges)
    ang = np.degrees(scan_msg.angle_min + scan_msg.angle_increment * (np.arange(len(r)) + 0.5))
    ok = ~np.isfinite(r) & (ang >= lo_deg) & (ang <= hi_deg)
    best, i = (0, 0), 0
    while i < len(r):
        if ok[i]:
            j = i
            while j + 1 < len(r) and ok[j + 1]:
                j += 1
            best = max(best, (j - i + 1, (i + j) // 2))
            i = j + 1
        else:
            i += 1
    if best[0] < min_bins:
        return None
    return float(ang[best[1]]), best[0] * math.degrees(scan_msg.angle_increment)


class Cost:
    """Local costmap in odom == map here: the full grid once, then the deltas of ``costmap_updates`` (what Nav2 really publishes)."""

    def __init__(self, h: Harness):
        self.info, self.data = None, None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        h.create_subscription(OccupancyGrid, 'nav2/local_costmap/costmap', self._full, qos)
        h.create_subscription(OccupancyGridUpdate, 'nav2/local_costmap/costmap_updates', self._delta, 10)

    @property
    def grid(self):
        return self.info

    def _full(self, m: OccupancyGrid) -> None:
        self.info = m.info
        self.data = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)

    def _delta(self, u: OccupancyGridUpdate) -> None:
        if self.data is None or (u.x + u.width > self.data.shape[1]) or (u.y + u.height > self.data.shape[0]):
            return
        self.data[u.y:u.y + u.height, u.x:u.x + u.width] = np.array(u.data, dtype=np.int16).reshape(u.height, u.width)

    def _cell(self, x: float, y: float):
        g = self.info
        ix, iy = int((x - g.origin.position.x) / g.resolution), int((y - g.origin.position.y) / g.resolution)
        return (ix, iy) if (1 <= ix < g.width - 1 and 1 <= iy < g.height - 1) else None

    def surface_lethal(self, sensor, centre, radius: float, half_arc_deg: float = 40.0, n: int = 5) -> dict:
        """Lethal cells on the EXPECTED SURFACE of a disc: n points on the arc facing the sensor (+-half_arc_deg), each checked in its
        3x3 neighbourhood (cell quantisation). Returns {'hit': points with a lethal cell, 'of': n} or {'hit': -1} if outside the window."""
        if self.info is None:
            return dict(hit=-1, of=n)
        toward = math.atan2(sensor[1] - centre[1], sensor[0] - centre[0])         # from the disc centre to the sensor
        hit = 0
        for k in np.linspace(-half_arc_deg, half_arc_deg, n):
            a = toward + math.radians(k)
            c = self._cell(centre[0] + radius * math.cos(a), centre[1] + radius * math.sin(a))
            if c is None:
                return dict(hit=-1, of=n)
            hit += int((self.data[c[1] - 1:c[1] + 2, c[0] - 1:c[0] + 2] == 100).any())
        return dict(hit=hit, of=n)

    def region_lethal(self, centre, reach: float = 0.35) -> int:
        """Any lethal cell within ``reach`` of the obstacle centre (stale marks anywhere around it); -1 outside the window."""
        if self.info is None:
            return -1
        res = self.info.resolution
        c = self._cell(*centre)
        k = int(round(reach / res))
        if c is None or not (k <= c[0] < self.info.width - k and k <= c[1] < self.info.height - k):
            return -1
        return int((self.data[c[1] - k:c[1] + k + 1, c[0] - k:c[0] + k + 1] == 100).sum())


class Scan:
    """Latest LaserScan as the ObstacleLayer receives it; classifies the bins around a bearing as return / observed-free / unknown."""

    def __init__(self, h: Harness):
        self.msg, self.t = None, 0.0
        self._h = h
        h.create_subscription(LaserScan, '/qazbot_lio/scan', self._on, QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))

    def _on(self, m: LaserScan) -> None:
        self.msg, self.t = m, self._h.now()

    def after(self, t0: float):
        """Wait for a scan that arrived after t0 (a command's acknowledgement): only such a scan can show the commanded state."""
        self._h.wait(lambda: self.t > t0, 15, 'a scan published after the command')

    def kind(self, bearing_deg: float, half_deg: float = 0.5) -> dict:
        m = self.msg
        if m is None:
            return dict(finite=0, inf=0, nan=0, rmin=None)
        r = np.array(m.ranges)
        ang = np.degrees(m.angle_min + m.angle_increment * (np.arange(len(r)) + 0.5))
        k = np.abs(ang - bearing_deg) <= half_deg
        fin = np.isfinite(r[k])
        return dict(finite=int(fin.sum()), inf=int(np.isposinf(r[k]).sum()), nan=int(np.isnan(r[k]).sum()),
                    rmin=round(float(r[k][fin].min()), 3) if fin.any() else None)


def run(args) -> int:
    rclpy.init()
    h = Harness()
    cost, scan = Cost(h), Scan(h)
    cmd = h.create_publisher(String, '/x2_twin/scenario_cmd', 10)
    plant = {}
    h.create_subscription(String, '/x2_twin/scenario_state', lambda m: plant.update(json.loads(m.data)),
                          QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    start = tuple(float(v) for v in args.start.split(','))
    cand = args.mode == 'candidate'
    seq, phases, nz, cmds = [0], {}, [], []
    from aimdk_msgs.msg import McLocomotionVelocity
    h.create_subscription(McLocomotionVelocity, '/aima/mc/locomotion/velocity',
                          lambda m: (cmds.append((round(h.now(), 3), m.forward_velocity, m.angular_velocity)),
                                     nz.append(h.now()) if (abs(m.forward_velocity) > 1e-3 or abs(m.angular_velocity) > 1e-3) else None),
                          QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
    rc = 1

    def say(**kw):
        """ONE publication, then proof of delivery: the plant echoes the sequence number with the state it applied. No re-sending."""
        h.wait(lambda: cmd.get_subscription_count() > 0, 30, 'plant subscribed to scenario_cmd')
        seq[0] += 1
        cmd.publish(String(data=json.dumps(dict(kw, seq=seq[0]))))
        h.wait(lambda: plant.get('seq') == seq[0], 10, f'plant acknowledgement of command {seq[0]} (delivery failure, not retried)')
        return h.now()

    def phase(name):
        phases[name] = [time.time(), None]
        return name

    def end(name):
        phases[name][1] = time.time()

    try:
        occ = OccupancyMap.from_yaml(args.map_yaml)

        def at(rng, brg):
            """World point at range/bearing from the robot's CURRENT truth pose (it may have turned a little in a goal attempt)."""
            x, y, yaw = h.pose
            return x + rng * math.cos(yaw + brg), y + rng * math.sin(yaw + brg)

        # --- readiness (truth localisation: the trust verdict comes from the truth localiser)
        h.wait(lambda: h.pose is not None and h.state['adapter'].get('live') is True, 120, 'plant and live adapter')
        h.wait(lambda: h.state['trust'].get('source') == 'truth_localizer' and h.state['trust'].get('trusted') is True, 60, 'truth trust')
        r = h.trigger('motion_adapter/register')
        h.check('registered', r.success, r.message)
        h.wait(lambda: h.state['adapter'].get('registration') == 'registered', 20, 'registration')
        h.wait(lambda: h.state['coord'].get('ready', False), 180, 'coordinator ready')
        h.wait(lambda: cost.grid is not None and scan.msg is not None, 60, 'local costmap and scan')
        h.wait(lambda: 'seq' in plant, 30, 'plant scenario state')
        scan.after(h.now())
        half_sector = (args.scan_fov_deg or 103.0) / 2.0                     # bearings the plant sensor can observe at all
        mid = open_run(scan.msg, FOV_DEG[0] + 3, FOV_DEG[1] - 3, 24)
        h.check('open_sector_inside_fov', mid is not None, mid)
        edge = None
        if half_sector > abs(FOV_DEG[0]) + 3 + 8:                          # room for an observable obstacle beyond the mask edge
            edge = open_run(scan.msg, -half_sector + 1, FOV_DEG[0] - 3, 32)
            h.check('open_sector_beyond_fov_edge', edge is not None, edge)
        else:
            h.check('D_not_applicable_sector_ends_at_the_mask_edge', True, dict(half_sector=half_sector, fov_mask=FOV_DEG))
        bearing = math.radians(mid[0])
        g = at(1.0, bearing)
        h.check('baseline_state_clean', cost.region_lethal(g) == 0 and plant.get('ghosts') == [] and plant.get('scan') == 'normal',
                dict(region_lethal=cost.region_lethal(g), plant=dict(plant)))
        h.check('plant_scan_format', (plant.get('scan_format') == 'i1') if cand else True, plant.get('scan_format'))   # ablations vary it
        brg_deg = math.degrees(bearing)

        # --- A: a real obstacle. command -> plant -> scan -> costmap surface -> RPP/Nav2
        a = phase('A_real_obstacle')
        t_ack = say(ghosts=[dict(x=g[0], y=g[1], r=GHOST_R)])
        h.check('A1_plant_applied_obstacle', plant.get('ghosts') == [dict(x=g[0], y=g[1], r=GHOST_R)] or len(plant.get('ghosts', [])) == 1,
                plant.get('ghosts'))
        scan.after(t_ack)
        k = scan.kind(brg_deg)
        h.check('A2_scan_has_the_return', k['finite'] > 0 and abs(k['rmin'] - (1.0 - GHOST_R)) < 0.06, k)
        h.wait(lambda: cost.surface_lethal(h.pose[:2], g, GHOST_R)['hit'] >= 3, 30, 'obstacle surface lethal in the costmap')
        h.check('A3_costmap_surface_lethal', True, cost.surface_lethal(h.pose[:2], g, GHOST_R))
        n0, p0 = len(nz), h.pose
        goal = at(1.5, bearing)
        h.check('A4_goal_has_room_for_the_robot', float(occ.dist_at(np.array([goal]))[0]) >= 0.8, dict(goal=goal))
        hdl = h.send_goal(*goal, h.pose[2] + bearing)
        status = h.finish(hdl, 60.0) if hdl.accepted else None
        h.spin(1.0)
        dmin = math.hypot(h.pose[0] - g[0], h.pose[1] - g[1])
        moved = math.hypot(h.pose[0] - p0[0], h.pose[1] - p0[1])
        h.check('A4_real_obstacle_blocks_motion', status != SUCCEEDED and dmin >= ROBOT_R + GHOST_R - 0.05,
                dict(status=status, accepted=hdl.accepted, min_dist_to_obstacle=round(dmin, 3), moved=round(moved, 3),
                     nonzero_cmds=len(nz) - n0), hard=cand)
        end(a)

        # --- C: scan loss. The obstacle disappears while nothing proves it: the mark must stay
        c = phase('C_scan_loss')
        say(scan='none', ghosts=[])
        h.spin(8.0)
        h.check('C1_no_scan_mark_stays', cost.region_lethal(g) > 0, dict(region_lethal=cost.region_lethal(g)), hard=cand)
        scan.after(say(scan='empty'))
        k = scan.kind(brg_deg)
        h.check('C2_empty_scan_is_not_free_space', (k['inf'] == 0) if cand else True, k, hard=cand)
        h.spin(8.0)
        h.check('C3_empty_scan_mark_stays', cost.region_lethal(g) > 0, dict(region_lethal=cost.region_lethal(g)), hard=cand)
        # part of the sensor blocked: that sector delivers no point at all
        scan.after(say(scan=f'blind:{brg_deg - 10:.1f}:{brg_deg + 10:.1f}'))
        k = scan.kind(brg_deg)
        h.check('C4_blocked_sector_is_not_free_space', (k['inf'] == 0 and k['nan'] > 0) if cand else True, k, hard=cand)
        h.spin(8.0)
        h.check('C5_blocked_sector_mark_stays', cost.region_lethal(g) > 0, dict(region_lethal=cost.region_lethal(g)), hard=cand)
        end(c)

        # --- B: scans are back and show the FOV bearing free: the stale mark must go (candidate) / is the bug (legacy)
        b = phase('B_removed_in_fov')
        scan.after(say(scan='normal'))
        k = scan.kind(brg_deg)
        h.check('B1_scan_reads_free_at_the_obstacle_bearing', k['finite'] == 0 and k['inf'] > 0 and k['nan'] == 0, k, hard=cand)
        h.spin(10.0)
        left = cost.region_lethal(g)
        h.check('B2_stale_mark_cleared' if cand else 'B2_bug_reproduced_stale_mark_persists', (left == 0) if cand else (left > 0),
                dict(region_lethal=left, surface=cost.surface_lethal(h.pose[:2], g, GHOST_R)), hard=cand)
        end(b)

        if edge is not None:
            # --- D: beyond the observed FOV mask: unknown is not free
            out_brg = edge[0]
            d = phase('D_outside_fov')
            og = at(1.5, math.radians(out_brg))
            scan.after(say(ghosts=[dict(x=og[0], y=og[1], r=0.10)]))
            k = scan.kind(out_brg, 1.0)
            h.check('D1_scan_has_the_return', k['finite'] > 0 and abs(k['rmin'] - 1.4) < 0.1, k)
            h.wait(lambda: cost.surface_lethal(h.pose[:2], og, 0.10, 30.0, 3)['hit'] >= 2, 30, 'outside-FOV surface lethal')
            h.check('D2_costmap_surface_lethal', True, cost.surface_lethal(h.pose[:2], og, 0.10, 30.0, 3))
            scan.after(say(ghosts=[]))
            k = scan.kind(out_brg, 1.0)
            unknown = k['finite'] == 0 and k['nan'] > 0 and k['inf'] == 0
            legacy_inf = k['finite'] == 0 and k['inf'] > 0
            h.check('D3_scan_reads_unknown_outside_the_fov', unknown if cand else legacy_inf, k, hard=cand)
            h.spin(10.0)
            h.check('D4_outside_fov_mark_not_erased', cost.region_lethal(og, 0.2) > 0,
                    dict(region_lethal=cost.region_lethal(og, 0.2), surface=cost.surface_lethal(h.pose[:2], og, 0.10, 30.0, 3)), hard=cand)
            end(d)

        # --- E: free route
        e = phase('E_free_route')
        n0, p0 = len(nz), h.pose
        goal = at(1.5, bearing)
        h.check('E_goal_has_room_for_the_robot', float(occ.dist_at(np.array([goal]))[0]) >= 0.8, dict(goal=goal))
        hdl = h.send_goal(*goal, h.pose[2] + bearing)
        status = h.finish(hdl, 90.0) if hdl.accepted else None
        h.spin(1.0)
        moved = math.hypot(h.pose[0] - p0[0], h.pose[1] - p0[1])
        h.check('E1_free_route_moves' if cand else 'E1_bug_reproduced_goal_refused_without_motion',
                (moved > 0.5) if cand else (moved < 0.15 and status != SUCCEEDED),
                dict(status=status, accepted=hdl.accepted, moved=round(moved, 3), nonzero_cmds=len(nz) - n0), hard=cand)
        end(e)
        rc = 0
    except Exception as exc:                                              # noqa: BLE001 - recorded in results
        import traceback
        traceback.print_exc()
        print(f'SCENARIO ERROR: {exc!r}', file=sys.stderr)
        h.results.append(dict(name='scenario_error', ok=False, detail=repr(exc), hard=True))
    finally:
        o = pathlib.Path(args.results_dir) / args.name
        o.mkdir(parents=True, exist_ok=False)
        (o / 'trace.csv').write_text('t,x,y,yaw,v,w\n' + ''.join(','.join(map(str, r)) + '\n' for r in h.trace))
        (o / 'cmd_trace.csv').write_text('t,cmd_v,cmd_w\n' + ''.join(','.join(map(str, r)) + '\n' for r in cmds))
        (o / 'results.json').write_text(json.dumps(dict(
            mode=args.mode, start=start, phases=phases, checks=h.results,
            failed=[r['name'] for r in h.results if not r['ok']], hard_failed=[r['name'] for r in h.results if not r['ok'] and r['hard']]),
            indent=2, default=str))
        h.destroy_node()
        rclpy.try_shutdown()
    return rc


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--name', required=True)
    ap.add_argument('--mode', choices=('baseline', 'candidate'), required=True)
    ap.add_argument('--start', required=True)
    ap.add_argument('--scan-fov-deg', type=float, default=0.0, help='plant scan sector in degrees (0 = twin.yaml: 103)')
    ap.add_argument('--results-dir', default='/results')
    ap.add_argument('--map-yaml', default='/opt/x2twin/stack/maps/first_map_nav2_v3/map.yaml')
    sys.exit(run(ap.parse_args()))
