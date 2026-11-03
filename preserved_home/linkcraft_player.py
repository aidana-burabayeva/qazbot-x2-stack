#!/usr/bin/env python3
"""
linkcraft_player.py - play LinkCraft actions on N AimDK X2 robots at the same
instant. It is the on-robot player behind linkcraft_app.py, and it can also
run a synchronised loop on its own (no laptop needed).

Unlike robot_dialogue.py (robots take turns), here every robot performs the
same action simultaneously. One robot is the leader: it keeps a warm
"follower" process alive on each of the other robots and, each cycle, tells
them all to fire at one agreed wall-clock instant.

  # on any one of the robots (list every robot, including this one):
  python3 linkcraft_player.py run 192.168.0.28 192.168.0.54 --action "My action"

  # other action / other gap
  python3 linkcraft_player.py run 192.168.0.28 192.168.0.54 \
      --action "My action" --gap 120 --duration 33

Modes
-----
  run     Leader. Drives every IP you pass. Ctrl+C stops everything.
  follow  Warm player, started automatically by the leader over SSH.
  list    Print this robot's LinkCraft resources (name / key / version).
  push    From your laptop: scp this script to every robot you list.

Why a warm follower
-------------------
Bringing up rclpy and the aimdk services takes seconds and that latency is
not constant, so "ssh robot python3 play.py" cannot be synchronised. The
follower pays that cost once, then sits polling a cue file. A cue is a ~200 ms
SSH write, so a 1.5 s lead is plenty and all robots move together.

Clock skew
----------
Robots rarely share an NTP source. At startup the leader measures each peer's
clock offset (3 samples, best round-trip wins) and expresses every cue in that
peer's own clock. Offsets are re-measured every --resync cycles.

Requirements: passwordless SSH from the leader to each of the other robots
(one direction only - this is not the baton dialogue), this script at the same
path on all of them, and Develop_MC motion mode everywhere.
"""

import argparse
import json
import os
import re
import shlex
import signal
import socket
import subprocess
import sys
import threading
import time

STATE_DIR = "/tmp/linkcraft_player"
CUE = f"{STATE_DIR}/cue"
READY = f"{STATE_DIR}/follower.ready"
STOP = f"{STATE_DIR}/stop"
PIDFILE = f"{STATE_DIR}/follower.pid"
LAST = f"{STATE_DIR}/last.json"
LOG = f"{STATE_DIR}/follower.log"

DEFAULT_ROS_SETUP = ["/opt/ros/humble/setup.bash", "~/aimdk/install/setup.bash"]

# ExecuteActionResource "accepted": 1 for recorded (arm) actions, 200 for the
# whole-body linkcraft_resource_onnx_* actions. -1 is our own "the call threw".
OK_STATUSES = (1, 200)


def play_ok(status, message):
    """Did ExecuteActionResource start the action? AimDK 1.1 docs: the status
    is always FAILURE(2) in that firmware and only the message is reliable
    ("Play success"); AimDK 1.0 robots return 1/200 with the same message."""
    return str(message).strip() == "Play success" or status in OK_STATUSES


# --------------------------------------------------------------------------- #
# ROS 2 / AimDK layer  (same service calls robot_dialogue.py uses)             #
# --------------------------------------------------------------------------- #

def make_ros_client(dry_run=False):
    if dry_run:
        return _DryRunClient()

    import rclpy
    from rclpy.node import Node
    from aimdk_msgs.srv import GetRobotResources, ExecuteActionResource

    class AimdkClient(Node):
        def __init__(self):
            super().__init__("linkcraft_player_client")
            self.get_resource_client = self.create_client(
                GetRobotResources, "/aimdk_5Fmsgs/srv/GetRobotResources")
            self.play_resource_client = self.create_client(
                ExecuteActionResource, "/aimdk_5Fmsgs/srv/ExecuteActionResource")

        def list_resources(self, timeout_sec=30.0):
            if not self.get_resource_client.wait_for_service(timeout_sec=timeout_sec):
                raise RuntimeError("GetRobotResources service unavailable")
            request = GetRobotResources.Request()
            future = self.get_resource_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            if future.result() is None:
                raise RuntimeError("GetRobotResources call failed or timed out")
            out = []
            for res in future.result().robot_resources:
                ver = res.current_version
                out.append({"name": ver.name or "<no name>",
                            "resource_key": res.resource_key,
                            "version": ver.version})
            return out

        def warm_play_service(self, timeout_sec=30.0):
            """Block until ExecuteActionResource is discoverable, so the first
            cue does not pay for service discovery."""
            return self.play_resource_client.wait_for_service(timeout_sec=timeout_sec)

        def play(self, resource_key, version, meta_type=None, attempts=4,
                 timeout_sec=10.0, slaves=None):
            """slaves = list of IPs: AimDK 1.1 group control - this robot is
            the master and starts the action on itself and every slave."""
            if not self.play_resource_client.wait_for_service(timeout_sec=timeout_sec):
                raise RuntimeError("ExecuteActionResource service unavailable")
            request = ExecuteActionResource.Request()
            request.resource_key = resource_key
            request.resource_version = version
            if meta_type is None:
                meta_type = "BODY_MONTION" if "onnx" in resource_key else "ARM_MONTION"
            meta = {"resource_type": meta_type}
            if slaves:
                from aimdk_msgs.msg import SlaveDevice
                for ip in slaves:
                    dev = SlaveDevice()
                    dev.ip = ip
                    request.slaves.append(dev)
                meta = {"control_type": "GROUP_CONTROL", **meta}
                # never re-send a group start: a late answer would otherwise
                # make every robot play the action twice
                attempts, timeout_sec = 1, max(timeout_sec, 20.0)
            request.meta = json.dumps(meta)
            for i in range(attempts):
                future = self.play_resource_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
                if future.done() and future.result() is not None:
                    resp = future.result()
                    return resp.header.status.value, resp.header.message
                self.get_logger().warning(f"ExecuteActionResource retry [{i}]")
            raise RuntimeError("ExecuteActionResource failed after retries")

        def group_time_sync(self, slaves, timeout_sec=30.0):
            """AimDK 1.1: sync clocks between this (master) robot and the
            slaves; required before group playback. Returns (code, message),
            code 0 = ok."""
            try:
                from aimdk_msgs.msg import SlaveDevice
                from aimdk_msgs.srv import GroupControlTimeSync
            except ImportError:
                raise RuntimeError("this robot's AimDK has no GroupControlTimeSync "
                                   "(group control needs firmware/AimDK 1.1)")
            if getattr(self, "time_sync_client", None) is None:
                self.time_sync_client = self.create_client(
                    GroupControlTimeSync, "/aimdk_5Fmsgs/srv/GroupControlTimeSync")
            if not self.time_sync_client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError("GroupControlTimeSync service not running on this "
                                   "robot (group control needs firmware/AimDK 1.1)")
            request = GroupControlTimeSync.Request()
            for ip in slaves:
                dev = SlaveDevice()
                dev.ip = ip
                request.slaves.append(dev)
            future = self.time_sync_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            if not future.done() or future.result() is None:
                raise RuntimeError("GroupControlTimeSync timed out")
            resp = future.result()
            return int(resp.error_code), resp.error_msg

        def close(self):
            self.destroy_node()

    rclpy.init()
    return AimdkClient()


class _DryRunClient:
    def list_resources(self, timeout_sec=30.0):
        menu = [
            {"name": "Kazakh Greeting 1",
             "resource_key": "linkcraft_resource_DRY001", "version": "0.0.1"},
            {"name": "Greeting",
             "resource_key": "linkcraft_resource_DRY002", "version": "0.0.1"},
            {"name": "Dialogue 1",
             "resource_key": "linkcraft_resource_DRY003", "version": "0.0.1"},
        ]
        # real robots list their resources in their own order - rotate the fake
        # menu per host so a dry run cannot accidentally depend on the index
        shift = sum(bytearray(socket.gethostname().encode())) % len(menu)
        return menu[shift:] + menu[:shift]

    def warm_play_service(self, timeout_sec=30.0):
        return True

    def play(self, resource_key, version, meta_type=None, slaves=None, **kwargs):
        if slaves:
            return 2, "Play success"          # what AimDK 1.1 really answers
        return 1, f"dry-run {resource_key}"

    def group_time_sync(self, slaves, **kwargs):
        return 0, f"dry-run time sync with {', '.join(slaves)}"

    def close(self):
        pass


def shutdown_ros(client):
    try:
        client.close()
    except Exception:
        pass
    try:
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


# --------------------------------------------------------------------------- #
# Start-delay measurement: when did the joints actually start to move?         #
# --------------------------------------------------------------------------- #
#
# ExecuteActionResource returning "accepted" does not mean the robot is moving.
# Between the two there is a delay (often 100-200 ms) that differs per robot,
# so robots that are *called* at the same instant still visibly start apart.
# The follower measures that delay on every play from the robot's own joint
# encoders, and the app uses the history to fire slow robots earlier.

JOINT_TOPICS = [f"/aima/hal/joint/{part}/state"
                for part in ("arm", "leg", "waist", "head")]


class OnsetDetector:
    """Pure logic, no ROS - fed (time, joint name, position) samples.

    1. begin(): the cue has arrived. Until the fire instant every joint is
       expected to stand still; we record each joint's min/max position.
       That range is its "resting band" (sensor noise + balance sway).
    2. fire(t): the service is being called. Bands are frozen. A joint whose
       band is wider than WOBBLE was already moving - the measurement would
       be meaningless, so it is rejected ("not-resting").
    3. Afterwards, the first joint to leave its band by more than
       max(MIN_MOVE, JITTER_K x its own noise) has started the action. The
       onset is NOT the moment it crossed that threshold (that would be late
       by however long the joint needed to travel MIN_MOVE) but the last
       moment that joint was still inside its band.
    """

    MIN_MOVE = 0.015     # rad - smallest movement that counts as "moving"
    JITTER_K = 4.0       # threshold = max(MIN_MOVE, JITTER_K x band width)
    WOBBLE = 0.1         # rad - band wider than this: robot was not at rest
    MIN_SAMPLES = 3      # a joint needs this many baseline samples to count
    EARLY_OK = 0.05      # s - onset may sit this far before fire (msg jitter)

    def __init__(self):
        self.reset()

    def reset(self):
        self.phase = "idle"      # idle -> baseline -> armed -> done
        self.base = {}           # name -> [lo, hi, n, last_t]
        self.band = {}           # name -> (lo, hi, thr)
        self.last_in = {}        # name -> last time inside its band
        self.t_fire = None
        self.onset = None
        self.joint = None
        self.reason = None

    def begin(self):
        self.reset()
        self.phase = "baseline"

    def sample(self, t, name, pos):
        if self.phase == "baseline":
            b = self.base.get(name)
            if b is None:
                self.base[name] = [pos, pos, 1, t]
            else:
                if pos < b[0]:
                    b[0] = pos
                if pos > b[1]:
                    b[1] = pos
                b[2] += 1
                b[3] = t
        elif self.phase == "armed":
            band = self.band.get(name)
            if band is None:
                return
            lo, hi, thr = band
            if lo <= pos <= hi:
                self.last_in[name] = t
            elif pos < lo - thr or pos > hi + thr:
                onset = self.last_in.get(name, self.t_fire)
                if onset < self.t_fire - self.EARLY_OK:
                    self._finish("early", None, name)
                else:
                    self._finish("ok", max(onset, self.t_fire), name)

    def fire(self, t):
        if self.phase != "baseline":
            return
        self.t_fire = t
        for name, (lo, hi, n, last_t) in self.base.items():
            if n < self.MIN_SAMPLES:
                continue
            width = hi - lo
            if width > self.WOBBLE:
                self._finish("not-resting", None, name)
                return
            self.band[name] = (lo, hi, max(self.MIN_MOVE, self.JITTER_K * width))
            self.last_in[name] = last_t
        if not self.band:
            self._finish("no-data", None, None)
            return
        self.phase = "armed"

    def _finish(self, reason, onset, joint):
        self.phase = "done"
        self.reason, self.onset, self.joint = reason, onset, joint

    def result(self):
        """{'watch': reason, 'onset': t or None, 'joint': name or None}"""
        if self.phase == "armed":
            return {"watch": "no-motion", "onset": None, "joint": None}
        if self.phase in ("idle", "baseline"):
            return {"watch": "skipped", "onset": None, "joint": None}
        return {"watch": self.reason, "onset": self.onset, "joint": self.joint}


class MotionWatch:
    """Feeds an OnsetDetector from the four joint-state topics (~500 Hz each),
    on its own node and executor thread, so it keeps receiving while the main
    thread is blocked in the ExecuteActionResource call.

    Kept cheap on purpose - the robot is busiest exactly when it starts an
    action (onnx actions load for 0.6-2.5 s), and a Python process eating a
    core then makes that start later and less predictable:
    * between plays the executor does not spin at all, so ~2000 msgs/s are
      left to DDS and never reach Python;
    * messages arrive as raw bytes and only every STRIDE-th one per topic is
      deserialized (250 Hz instead of 500 Hz - 4 ms resolution)."""

    WINDOW = 4.0         # s after fire to wait for the first movement
    STRIDE = 2           # deserialize every 2nd message of each topic

    def __init__(self, log):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.serialization import deserialize_message
        from aimdk_msgs.msg import JointStateArray

        self.det = OnsetDetector()
        self.lock = threading.Lock()
        self.done = threading.Event()
        self.active = threading.Event()      # spin only while measuring
        self.seen = 0
        self._count = {}
        self._decode = lambda raw: deserialize_message(raw, JointStateArray)
        self.node = Node("linkcraft_motion_watch")
        for topic in JOINT_TOPICS:
            # sensor-data QoS (best effort) matches both reliable and
            # best-effort publishers; raw=True: bytes, decoded on demand
            self.node.create_subscription(
                JointStateArray, topic,
                lambda raw, topic=topic: self._on_raw(topic, raw),
                qos_profile_sensor_data, raw=True)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._rclpy = rclpy
        threading.Thread(target=self._spin, daemon=True).start()
        log(f"motion watch on {', '.join(JOINT_TOPICS)} "
            f"(idle between plays, every {self.STRIDE}. message)")

    def _spin(self):
        try:
            while self._rclpy.ok():
                if not self.active.wait(0.2):
                    continue
                self.executor.spin_once(timeout_sec=0.05)
        except Exception:        # context shut down under us: just stop
            pass

    def _on_raw(self, topic, raw):
        if not self.active.is_set():
            return               # stale message queued while idle
        n = self._count.get(topic, 0) + 1
        self._count[topic] = n
        if n % self.STRIDE:
            return
        t = time.time()          # receive time, robot clock (same as `fired`)
        msg = self._decode(raw)
        self.seen += 1
        with self.lock:
            if self.det.phase not in ("baseline", "armed"):
                return
            for j in msg.joints:
                self.det.sample(t, j.name, j.position)
            if self.det.phase == "done":
                self.done.set()

    def begin(self):
        with self.lock:
            self.done.clear()
            self.det.begin()
        self.active.set()

    def fire(self, t):
        with self.lock:
            self.det.fire(t)
            if self.det.phase == "done":
                self.done.set()

    def wait(self, deadline):
        self.done.wait(max(0.0, deadline - time.time()))
        self.active.clear()
        with self.lock:
            res = self.det.result()
            self.det.reset()
        if res["watch"] == "no-data" and self.seen == 0:
            res["watch"] = "no-topics"     # nothing ever arrived on the topics
        return res

    def cancel(self):
        self.active.clear()
        with self.lock:
            self.det.reset()

    def close(self):
        self.active.clear()
        try:
            self.executor.shutdown(timeout_sec=1.0)
            self.node.destroy_node()
        except Exception:
            pass


class _SimulatedWatch:
    """--dry-run: no joints to read. Reports a made-up but stable start delay
    per robot (40-180 ms from the hostname, +-8 ms noise) so the app's
    calibration and compensation can be exercised without moving anything."""

    WINDOW = 0.0

    def __init__(self, log):
        import random
        self.rng = random.Random()
        self.base = 0.040 + (sum(bytearray(socket.gethostname().encode())) % 15) * 0.010
        self.t_fire = None
        log(f"motion watch simulated (dry run): ~{self.base * 1000:.0f} ms start delay")

    def begin(self):
        self.t_fire = None

    def fire(self, t):
        self.t_fire = t

    def wait(self, deadline):
        if self.t_fire is None:
            return {"watch": "skipped", "onset": None, "joint": None}
        onset = self.t_fire + max(0.0, self.base + self.rng.gauss(0, 0.008))
        return {"watch": "simulated", "onset": onset, "joint": "dry_run"}

    def cancel(self):
        self.t_fire = None

    def close(self):
        pass


class _NoWatch:
    WINDOW = 0.0

    def begin(self):
        pass

    def fire(self, t):
        pass

    def wait(self, deadline):
        return {"watch": "off", "onset": None, "joint": None}

    def cancel(self):
        pass

    def close(self):
        pass


# --------------------------------------------------------------------------- #
# Readiness check ("all or nothing")                                           #
# --------------------------------------------------------------------------- #
#
# Firmware/AimDK 1.1 publishes what ExecuteActionResource itself looks at
# before refusing to play ("not in ready posture", "faulty state", "is
# playing"): /aima/mc/common/state (10 Hz), /aima/hds/alert_code_list (20 Hz)
# and /aima/hal/pmu/state (1 Hz). The follower keeps the latest of each and
# answers "can you dance right now?" in a few milliseconds.

POSES = {0: "unknown", 1: "standing", 2: "lying face up", 3: "lying face down",
         4: "sitting", 5: "squatting", 6: "FALLING"}
FSM = {0: "unknown", 1: "starting", 2: "stable", 3: "moving", 4: "safe",
       5: "special", 6: "test"}
PLAYER = {0: "idle", 1: "pre-playing", 2: "playing", 3: "interrupting", 4: "error"}
ACTION_STATUS = {0: "idle", 100: "running", 200: "switching mode",
                 300: "switching mode (uninterruptible)"}
# motion-control modes in which a LinkCraft action can't start
UNREADY_MODES = {
    "PASSIVE_DEFAULT": "passive mode (motors off)",
    "SOFT_EMERGENCY_STOP": "SOFT EMERGENCY STOP",
    "DAMPING_DEFAULT": "damping mode",
    "ZERO_TORQUE_DEFAULT": "zero-torque mode",
    "SIT_DOWN_DEFAULT": "sitting down",
    "CROUCH_DOWN_DEFAULT": "crouching",
    "LIE_DOWN_DEFAULT": "lying down",
    "STAND_UP_DEFAULT": "still standing up",
}
BATTERY_MIN = 15          # % - below this the robot is not allowed to dance
BATTERY_WARN = 25         # % - below this: warning only
STATE_MAX_AGE = 2.0       # s - older motion-control state counts as missing


def evaluate_state(st):
    """st: snapshot from RobotState.snapshot(). Returns (problems, warnings);
    any problem = this robot must not start. Only positive evidence blocks:
    a robot whose firmware doesn't publish a topic gets a warning."""
    problems, warnings = [], []
    mc = st.get("mc")
    if mc is None:
        warnings.append(st.get("mc_missing") or "motion-control state not available")
    elif mc["age"] > STATE_MAX_AGE:
        problems.append(f"motion-control state is {mc['age']:.1f}s old "
                        f"(motion control not responding?)")
    else:
        if mc["pose"] != 1:
            problems.append(f"not standing (posture: {POSES.get(mc['pose'], mc['pose'])})")
        if mc["mode"] in UNREADY_MODES:
            problems.append(f"motion control in {UNREADY_MODES[mc['mode']]} ({mc['mode']})")
        # fsm_state is NOT checked: on real X2 Ultra robots (firmware 1.1)
        # "safe" is the normal standing state, and they play from it
        if mc["action_status"] in (200, 300):
            problems.append(f"motion control is {ACTION_STATUS[mc['action_status']]}")
        if mc["player"] != 0:
            what = f" '{mc['motion']}'" if mc.get("motion") else ""
            problems.append(f"already busy: player {PLAYER.get(mc['player'], mc['player'])}{what}")
    alerts = st.get("alerts")
    if alerts is None:
        warnings.append(st.get("alerts_missing") or "fault alerts not available")
    elif alerts["codes"]:
        # only a warning: robots play with standing alerts (seen on a real
        # X2 Ultra), and a fault serious enough to matter switches motion
        # control to damping mode / disables the arms, which IS checked above
        warnings.append("fault alert(s): " + ", ".join(alerts["codes"][:6]))
    bat = st.get("battery")
    if bat is None:
        warnings.append(st.get("battery_missing") or "battery level not available")
    elif bat["percent"] < BATTERY_MIN:
        problems.append(f"battery {bat['percent']}% (minimum {BATTERY_MIN}%)")
    elif bat["percent"] < BATTERY_WARN:
        warnings.append(f"battery low: {bat['percent']}%")
    return problems, warnings


class RobotState:
    """Latest motion-control state, fault alerts and battery level, on its own
    node and executor thread (low-rate topics: ~31 msgs/s together)."""

    TOPICS = {"mc": "/aima/mc/common/state",
              "alerts": "/aima/hds/alert_code_list",
              "battery": "/aima/hal/pmu/state"}

    def __init__(self, log):
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        import importlib

        self.lock = threading.Lock()
        self.latest = {}          # kind -> (receive time, plain dict)
        self.missing = {}         # kind -> why it can't be subscribed
        self.node = Node("linkcraft_state_watch")
        types = {"mc": "McCommonState", "alerts": "AlertCodeArray",
                 "battery": "PmuState"}
        msgs = importlib.import_module("aimdk_msgs.msg")
        for kind, type_name in types.items():
            msg_type = getattr(msgs, type_name, None)
            if msg_type is None:
                self.missing[kind] = (f"{self.TOPICS[kind]} not in this robot's AimDK "
                                      f"(firmware/AimDK < 1.1?)")
                continue
            # best effort: matches the BEST_EFFORT publishers
            self.node.create_subscription(
                msg_type, self.TOPICS[kind],
                lambda msg, kind=kind: self._on(kind, msg), qos_profile_sensor_data)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        import rclpy
        self._rclpy = rclpy
        self.started = time.time()
        threading.Thread(target=self._spin, daemon=True).start()
        log("state watch on " + ", ".join(self.TOPICS[k] for k in self.TOPICS
                                         if k not in self.missing))

    def _spin(self):
        try:
            while self._rclpy.ok():
                self.executor.spin_once(timeout_sec=0.2)
        except Exception:
            pass

    def _on(self, kind, msg):
        try:
            if kind == "mc":
                plain = {"pose": int(msg.body_status.value),
                         "fsm": int(msg.fsm_state.current_state),
                         "mode": str(msg.action_info.action_desc),
                         "action_status": int(msg.action_info.status.value),
                         "player": int(msg.motion_status.player_state.value),
                         "motion": str(msg.motion_status.motion)}
            elif kind == "alerts":
                plain = {"codes": [str(a.code) for a in msg.list]}
            else:
                plain = {"percent": int(msg.battery_remaining_capacity_percentage)}
        except AttributeError as exc:       # message layout differs
            self.missing[kind] = f"{self.TOPICS[kind]}: unexpected message ({exc})"
            return
        with self.lock:
            self.latest[kind] = (time.time(), plain)

    def snapshot(self):
        now = time.time()
        out = {}
        with self.lock:
            latest = dict(self.latest)
        for kind in self.TOPICS:
            if kind in latest:
                t, plain = latest[kind]
                out[kind] = {**plain, "age": round(now - t, 2)}
            else:
                waited = now - self.started
                out[kind + "_missing"] = self.missing.get(kind) or (
                    f"nothing received on {self.TOPICS[kind]} in {waited:.0f}s")
        return out

    def close(self):
        try:
            self.executor.shutdown(timeout_sec=1.0)
            self.node.destroy_node()
        except Exception:
            pass


class _SimulatedState:
    """--dry-run: a ready robot, unless /tmp/linkcraft_player/fake_state.json
    says otherwise (same layout as RobotState.snapshot())."""

    def snapshot(self):
        try:
            with open(os.path.join(STATE_DIR, "fake_state.json")) as fh:
                return json.loads(fh.read())
        except (OSError, ValueError):
            return {"mc": {"pose": 1, "fsm": 2, "mode": "STAND_DEFAULT",
                           "action_status": 100, "player": 0, "motion": "",
                           "age": 0.05},
                    "alerts": {"codes": [], "age": 0.02},
                    "battery": {"percent": 80, "age": 0.4}}

    def close(self):
        pass


def make_robot_state(args, log):
    if args.dry_run:
        return _SimulatedState()
    try:
        return RobotState(log)
    except Exception as exc:
        why = str(exc)
        log(f"! state watch unavailable ({why}) - readiness checks only warn")

        class _None:
            def snapshot(self):
                return {"mc_missing": f"state watch failed: {why}"}

            def close(self):
                pass
        return _None()


def make_motion_watch(args, log):
    """Never fatal: a robot without the topics or message type still plays,
    it just reports no start delay."""
    if getattr(args, "no_watch", False):
        return _NoWatch()
    if args.dry_run:
        return _SimulatedWatch(log)
    try:
        return MotionWatch(log)
    except Exception as exc:
        log(f"! motion watch unavailable ({exc}) - start delays will not be measured")
        return _NoWatch()


# --------------------------------------------------------------------------- #
# Resource lookup by name                                                      #
# --------------------------------------------------------------------------- #

def _norm(text):
    return re.sub(r"[^a-z0-9]+", " ", (text or "").lower()).strip()


def resolve_action(resources, wanted):
    """Find one resource on THIS robot.

    Menu order is per-robot and means nothing here - nothing is ever addressed
    by index. `wanted` is either a resource key (used as-is) or a name: exact
    match after normalising case and punctuation, then a unique substring.
    Ambiguity is an error - never guess which greeting."""
    if wanted.startswith("linkcraft_resource_"):
        hit = [r for r in resources if r["resource_key"] == wanted]
        if hit:
            return hit[0]
        raise RuntimeError(f"no resource with key {wanted} on this robot")
    target = _norm(wanted)
    exact = [r for r in resources if _norm(r["name"]) == target]
    if len(exact) == 1:
        return exact[0]
    if len(exact) > 1:
        raise RuntimeError(f"{len(exact)} resources are named '{wanted}' - "
                           f"rename one, or address this robot by key with "
                           f"--action-for")
    partial = [r for r in resources if target and target in _norm(r["name"])]
    if len(partial) == 1:
        print(f"   (matched '{partial[0]['name']}')")
        return partial[0]
    if len(partial) > 1:
        names = ", ".join(repr(r["name"]) for r in partial)
        raise RuntimeError(f"'{wanted}' matches several resources: {names}")
    names = ", ".join(repr(r["name"]) for r in resources) or "(none)"
    raise RuntimeError(f"no resource named '{wanted}' on this robot. Available: "
                       f"{names}. If it is called something else here, pass "
                       f"--action-for <ip>=<name or key>.")


def parse_overrides(pairs):
    """--action-for 192.168.0.54="My action v2"  ->  {ip: name-or-key}"""
    out = {}
    for raw in pairs or []:
        if "=" not in raw:
            raise SystemExit(f"--action-for wants IP=NAME, got {raw!r}")
        host, value = raw.split("=", 1)
        out[host.strip()] = value.strip()
    return out


def extract_json_array(text):
    """Pull the resource array out of an SSH reply - a login shell prints a
    MOTD and ROS chatter around it. Only accept an array of resource records:
    a ROS log line like [1772526701.308] is itself valid JSON."""
    decoder = json.JSONDecoder()
    empty = False
    for idx, ch in enumerate(text):
        if ch != "[":
            continue
        try:
            value, _ = decoder.raw_decode(text[idx:])
        except json.JSONDecodeError:
            continue
        if not isinstance(value, list):
            continue
        if value and all(isinstance(v, dict) and "resource_key" in v for v in value):
            return value
        if not value:
            empty = True
    if empty:
        return []
    raise RuntimeError("no resource list in the reply - is the script path right "
                       "and ROS sourced there? Raw output:\n"
                       + (text.strip()[:400] or "(empty)"))


# --------------------------------------------------------------------------- #
# SSH plumbing                                                                 #
# --------------------------------------------------------------------------- #

def ssh_argv(host, user, key=None, port=None):
    # ControlMaster matters for synchronisation, not just speed: without it every
    # cue pays a fresh TCP + auth handshake (200-800 ms, and variable), which eats
    # the lead time unevenly across robots. With it, cue delivery is ~20-50 ms and
    # roughly equal for all of them.
    argv = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
            "-o", "StrictHostKeyChecking=accept-new",
            "-o", "ServerAliveInterval=5",
            "-o", "ControlMaster=auto",
            "-o", f"ControlPath={STATE_DIR}/ssh-%r@%h:%p",
            "-o", "ControlPersist=600"]
    if key:
        argv += ["-i", os.path.expanduser(key)]
    if port:
        argv += ["-p", str(port)]
    argv.append(f"{user}@{host}")
    return argv


class Peer:
    def __init__(self, host, args, action):
        self.host = host
        self.action = action
        self.user = args.user
        self.key = args.ssh_key
        self.port = args.ssh_port
        self.script = args.remote_script
        self.ros_setup = args.ros_setup
        self.offset = 0.0          # peer_clock - my_clock, seconds
        self.rtt = 0.0
        self.last_send = 0.0
        self.alive = True
        self.watch = getattr(args, "watch", True)   # joint watch in followers

    # -- raw ssh -------------------------------------------------------- #
    def ssh(self, command, timeout=20, check=True, login_shell=False):
        if login_shell:
            command = f"bash -lc {shlex.quote(command)}"
        proc = subprocess.run(
            ssh_argv(self.host, self.user, self.key, self.port) + [command],
            capture_output=True, text=True, timeout=timeout)
        if check and proc.returncode != 0:
            raise RuntimeError(proc.stderr.strip() or f"ssh exited {proc.returncode}")
        return proc

    def prelude(self):
        return "".join(f"source {s} >/dev/null 2>&1; " for s in self.ros_setup)

    # -- clock ---------------------------------------------------------- #
    def measure_offset(self, samples=3):
        best = None
        for _ in range(samples):
            t0 = time.time()
            proc = self.ssh("date +%s.%N", timeout=10)
            t1 = time.time()
            try:
                remote = float(proc.stdout.strip().splitlines()[-1])
            except (ValueError, IndexError):
                continue
            rtt = t1 - t0
            if best is None or rtt < best[0]:
                best = (rtt, remote - (t0 + t1) / 2.0)
        if best is None:
            raise RuntimeError("could not read the peer's clock")
        self.rtt, self.offset = best
        return self.offset

    # -- follower lifecycle --------------------------------------------- #
    #
    # Never `pkill -f 'linkcraft_player.py follow'` in the same command that also
    # *launches* the follower: that pattern matches the remote shell's own
    # command line, so the shell SIGTERMs itself before it starts anything.
    # Killing goes in its own SSH call, by pid file first, with a pattern
    # assembled from two string literals as the fallback.
    KILL_CMD = (
        f"mkdir -p {STATE_DIR}; "
        f"if [ -f {PIDFILE} ]; then kill \"$(cat {PIDFILE})\" 2>/dev/null; fi; "
        f"PAT=\"linkcraft_player\"\"[.]py follow\"; pkill -f \"$PAT\" 2>/dev/null; "
        f"sleep 0.4; "
        # a follower that ignored SIGTERM (old player versions: rclpy swallows
        # it) would keep stealing cues from its successor - make sure it's gone
        f"pkill -9 -f \"$PAT\" 2>/dev/null; sleep 0.1; true"
    )

    def kill_follower(self):
        self.ssh(self.KILL_CMD, timeout=20, check=False)

    def follower_alive(self):
        proc = self.ssh(f"kill -0 \"$(cat {PIDFILE} 2>/dev/null)\" 2>/dev/null",
                        timeout=10, check=False)
        return proc.returncode == 0

    def start_follower(self, action, dry_run, wait=90):
        self.kill_follower()
        self.ssh(f"rm -f {READY} {CUE} {STOP} {PIDFILE} {LAST}", timeout=15,
                 check=False)

        launch = (
            f"{self.prelude()}"
            f"mkdir -p {STATE_DIR}; "
            f"nohup python3 {shlex.quote(self.script)} follow "
            f"--action {shlex.quote(action)}"
            f"{' --dry-run' if dry_run else ''}"
            f"{'' if self.watch else ' --no-watch'} "
            f"> {LOG} 2>&1 < /dev/null & echo $! > {PIDFILE}; disown; echo started"
        )
        self.ssh(launch, timeout=30, login_shell=True)

        started = time.time()
        deadline = started + wait
        while time.time() < deadline:
            proc = self.ssh(f"test -f {READY} && cat {READY}", timeout=10, check=False)
            if proc.returncode == 0 and proc.stdout.strip():
                return proc.stdout.strip()
            # give the pid file a moment to appear before believing it is dead
            if time.time() - started > 3.0 and not self.follower_alive():
                tail = self.ssh(f"tail -n 20 {LOG}", timeout=10, check=False).stdout
                raise RuntimeError(f"follower died on startup:\n{tail.strip()}")
            time.sleep(1.0)
        raise RuntimeError(f"follower did not come up within {wait}s "
                           f"(check {LOG} on {self.host})")

    def list_resources_remote(self, dry_run=False, timeout=120):
        inner = (f"{self.prelude()}python3 {shlex.quote(self.script)} list --json"
                 f"{' --dry-run' if dry_run else ''}")
        proc = self.ssh(inner, timeout=timeout, login_shell=True)
        return extract_json_array(proc.stdout)

    def cue(self, at_my_clock, seq, extra=None):
        payload = json.dumps({"at": at_my_clock + self.offset, "seq": seq,
                              **(extra or {})})
        cmd = (f"mkdir -p {STATE_DIR} && printf '%s' {shlex.quote(payload)} > {CUE}.part "
               f"&& mv {CUE}.part {CUE}")
        t0 = time.time()
        self.ssh(cmd, timeout=15)
        self.last_send = time.time() - t0

    def last_record(self, seq):
        """The follower's raw record for cue `seq`, or None (not there yet)."""
        proc = self.ssh(f"cat {LAST} 2>/dev/null", timeout=10, check=False)
        if proc.returncode != 0 or not proc.stdout.strip():
            return None
        try:
            rec = json.loads(proc.stdout.strip().splitlines()[-1])
        except json.JSONDecodeError:
            return None
        return rec if rec.get("seq") == seq else None

    def last_play(self, seq):
        """What that robot actually did, translated into the leader's clock."""
        rec = self.last_record(seq)
        if rec is None or "fired" not in rec:
            return None
        startup = rec.get("startup")
        status = rec.get("status")
        return {"fired": rec["fired"] - self.offset,
                "call_ms": (rec["returned"] - rec["fired"]) * 1000.0,
                "status": status,
                "ok": rec.get("ok", status in OK_STATUSES),
                "message": rec.get("message", ""),
                "mode": rec.get("mode", "play"),
                # seconds from the service call to the first joint movement,
                # or None (see "watch" for why)
                "startup": None if startup is None else float(startup),
                "watch": rec.get("watch", "old-player"),
                "joint": rec.get("joint")}

    def close_master(self):
        """Drop the persistent ssh connection instead of leaving it for 10 min."""
        argv = ssh_argv(self.host, self.user, self.key, self.port)
        subprocess.run(argv[:-1] + ["-O", "exit", argv[-1]],
                       capture_output=True, timeout=10)

    def stop_follower(self):
        try:
            self.ssh(f"mkdir -p {STATE_DIR}; printf stop > {STOP}; sleep 0.6; true",
                     timeout=20, check=False)
            self.kill_follower()
        except Exception as exc:
            print(f"  ! could not stop the follower on {self.host}: {exc}")


# --------------------------------------------------------------------------- #
# Local address detection                                                      #
# --------------------------------------------------------------------------- #

def local_addresses():
    addrs = {"127.0.0.1", "localhost", "::1"}
    try:
        addrs.add(socket.gethostbyname(socket.gethostname()))
    except Exception:
        pass
    for argv in (["ip", "-4", "-o", "addr", "show"], ["ifconfig"]):
        try:
            out = subprocess.run(argv, capture_output=True, text=True,
                                 timeout=5).stdout
        except Exception:
            continue
        addrs.update(re.findall(r"inet (?:addr:)?(\d+\.\d+\.\d+\.\d+)", out))
        if out:
            break
    return addrs


# --------------------------------------------------------------------------- #
# follow                                                                       #
# --------------------------------------------------------------------------- #

def write_last(record):
    with open(LAST + ".part", "w") as fh:
        fh.write(json.dumps(record))
    os.replace(LAST + ".part", LAST)


def _own_pidfile():
    try:
        with open(PIDFILE) as fh:
            return fh.read().strip() == str(os.getpid())
    except OSError:
        return False


def _exit_on_sigterm():
    """rclpy (Humble) catches SIGTERM and only shuts ROS down - the process
    then kept running and stole cues from the next follower. Exit instead.
    Installed after rclpy.init() so it replaces rclpy's handler."""
    def handler(signum, frame):
        raise SystemExit(0)
    signal.signal(signal.SIGTERM, handler)


def cmd_follow(args):
    os.makedirs(STATE_DIR, exist_ok=True)
    with open(PIDFILE, "w") as fh:
        fh.write(str(os.getpid()))
    for path in (READY, CUE, STOP):
        try:
            os.remove(path)
        except FileNotFoundError:
            pass

    def log(msg):
        now = time.time()
        stamp = time.strftime('%H:%M:%S', time.localtime(now))
        print(f"[follow {stamp}.{int(now % 1 * 1000):03d}] {msg}", flush=True)

    client = make_ros_client(dry_run=args.dry_run)
    _exit_on_sigterm()
    watch = _NoWatch()
    state = None
    try:
        watch = make_motion_watch(args, log)
        time.sleep(0.6)           # AimDK: max ~2 new ROS nodes per second
        state = make_robot_state(args, log)
        # --action "" = generic player: every cue names its own resource
        if args.action:
            step = resolve_action(client.list_resources(), args.action)
            log(f"action '{step['name']}' -> {step['resource_key']} v{step['version']}")
        else:
            step = {"name": None, "resource_key": None, "version": None}
            log("generic player: the action comes with each cue")
        client.warm_play_service()
        with open(READY + ".part", "w") as fh:
            fh.write(json.dumps({"name": step["name"],
                                 "resource_key": step["resource_key"],
                                 "version": step["version"],
                                 "pid": os.getpid()}))
        os.replace(READY + ".part", READY)
        log("ready, waiting for cues")

        last_check = 0.0
        while True:
            if os.path.exists(STOP):
                os.remove(STOP)
                log("stop received")
                break
            now = time.time()
            if now - last_check > 0.5:
                last_check = now
                # a newer follower took over, or ROS is gone: never linger
                if not _own_pidfile():
                    log("another follower owns the pid file - exiting")
                    break
                if not args.dry_run:
                    import rclpy
                    if not rclpy.ok():
                        log("ROS context is down - exiting")
                        break
            if not os.path.exists(CUE):
                time.sleep(0.02)
                continue
            try:
                with open(CUE) as fh:
                    payload = json.loads(fh.read())
            except (OSError, json.JSONDecodeError):
                time.sleep(0.02)
                continue
            os.remove(CUE)

            at = float(payload.get("at", 0))
            seq = payload.get("seq", "?")
            mode = payload.get("mode", "play")
            slaves = payload.get("slaves") or []

            if mode == "check":
                snap = state.snapshot()
                problems, warnings = evaluate_state(snap)
                write_last({"seq": seq, "mode": mode, "ok": not problems,
                            "problems": problems, "warnings": warnings,
                            "state": snap, "fired": time.time(),
                            "returned": time.time()})
                if problems:
                    log(f"cue {seq} readiness check: NOT ready - {'; '.join(problems)}")
                continue

            if mode == "timesync":
                # AimDK 1.1 group control: master syncs clocks with slaves
                t0 = time.time()
                try:
                    code, text = client.group_time_sync(slaves)
                except Exception as exc:
                    code, text = -1, str(exc)
                write_last({"seq": seq, "mode": mode, "error_code": code,
                            "error_msg": text, "fired": t0, "returned": time.time()})
                log(f"cue {seq} time sync with {', '.join(slaves)}: "
                    f"error_code={code} {text}")
                continue

            key = payload.get("resource_key") or step["resource_key"]
            ver = payload.get("version") or step["version"]
            if mode != "watch" and not key:
                log(f"! cue {seq} names no action - ignored")
                continue
            # from here to `at` the robot should be standing still: that
            # stretch is the motion watch's baseline
            watch.begin()
            late = time.time() - at
            if late > 0.25:
                log(f"! cue {seq} arrived {late:.2f}s late - firing now")
            while time.time() < at:
                time.sleep(min(0.005, max(0.0, at - time.time())))
            fired = time.time()
            watch.fire(fired)
            if mode == "watch":
                # group slave: the master starts us; we only time our motion
                # from the instant the master was told to call
                status, message, ok = None, "watch only (group slave)", True
            else:
                try:
                    status, message = client.play(key, ver, slaves=slaves or None)
                except Exception as exc:          # keep the player alive
                    status, message = -1, str(exc)
                ok = play_ok(status, message)
            returned = time.time()
            if ok:
                seen = watch.wait(fired + watch.WINDOW)
            else:
                watch.cancel()
                seen = {"watch": "not-played", "onset": None, "joint": None}
            startup = (None if seen["onset"] is None
                       else round(seen["onset"] - fired, 4))
            write_last({"seq": seq, "mode": mode, "scheduled": at, "fired": fired,
                        "returned": returned, "status": status, "ok": ok,
                        "message": str(message)[:500],
                        "startup": startup, "watch": seen["watch"],
                        "joint": seen["joint"]})
            moved = (f"moved +{startup * 1000:.0f}ms ({seen['joint']})"
                     if startup is not None else f"start delay: {seen['watch']}")
            what = {"group": f"group master (+{len(slaves)} slave(s))",
                    "watch": "group slave, watch only"}.get(mode, "play")
            log(f"cue {seq} [{key or '-'}] {what}: fired {(fired - at) * 1000:+.0f}ms "
                f"off schedule, service call {(returned - fired) * 1000:.0f}ms, "
                f"status={status} message={message}, {moved}")
    except KeyboardInterrupt:
        log("interrupted")
    except SystemExit:
        log("terminated")
    finally:
        # a successor may already own these files: leave them alone then
        if _own_pidfile():
            for path in (READY, CUE, PIDFILE):  # LAST is left for inspection
                try:
                    os.remove(path)
                except FileNotFoundError:
                    pass
        watch.close()
        if state is not None:
            state.close()
        shutdown_ros(client)
    return 0


# --------------------------------------------------------------------------- #
# list                                                                         #
# --------------------------------------------------------------------------- #

def cmd_list(args):
    client = make_ros_client(dry_run=args.dry_run)
    try:
        resources = client.list_resources()
    finally:
        shutdown_ros(client)
    if args.json:
        print(json.dumps(resources))
        return 0
    print("=== LinkCraft actions on this robot ===")
    for idx, res in enumerate(resources):
        print(f"  [{idx}] {res['name']}")
        print(f"        key:     {res['resource_key']}")
        print(f"        version: {res['version']}")
    return 0


# --------------------------------------------------------------------------- #
# push                                                                         #
# --------------------------------------------------------------------------- #

def cmd_push(args):
    me = os.path.abspath(__file__)
    rc = 0
    for host in args.robots:
        dest = f"{args.user}@{host}:{args.remote_script}"
        print(f"  {me} -> {dest}")
        argv = ["scp", "-o", "ConnectTimeout=5"]
        if args.ssh_key:
            argv += ["-i", os.path.expanduser(args.ssh_key)]
        if args.ssh_port:
            argv += ["-P", str(args.ssh_port)]
        argv += [me, dest]
        if subprocess.run(argv).returncode != 0:
            print(f"  FAIL {host}")
            rc = 1
    return rc


# --------------------------------------------------------------------------- #
# check                                                                        #
# --------------------------------------------------------------------------- #

def cmd_check(args):
    """Ask every robot what it would play, without moving anything.


    Menu indices differ from robot to robot and are never used - this shows the
    name each robot matched and the key it resolved to, so a robot that calls
    the action something else shows up here instead of mid-demo."""
    os.makedirs(STATE_DIR, exist_ok=True)   # ssh control sockets live here
    overrides = parse_overrides(args.action_for)
    mine = local_addresses()
    rows, bad = [], 0

    for host in args.robots:
        wanted = overrides.get(host, args.action)
        try:
            if host in mine and not args.no_local:
                client = make_ros_client(dry_run=args.dry_run)
                try:
                    resources = client.list_resources()
                finally:
                    shutdown_ros(client)
            else:
                resources = Peer(host, args, wanted).list_resources_remote(
                    dry_run=args.dry_run)
            step = resolve_action(resources, wanted)
            rows.append((host, "ok", step["name"], step["resource_key"],
                         step["version"], len(resources)))
        except Exception as exc:
            rows.append((host, "FAIL", str(exc), "", "", 0))
            bad += 1

    print(f"=== looking for {args.action!r} ===")
    for host, state, a, b, c, n in rows:
        if state == "ok":
            print(f"  {host:<16} {a!r} -> {b} v{c}   ({n} resources on this robot)")
        else:
            head, _, rest = a.partition("\n")
            print(f"  {host:<16} FAIL: {head}")
            for line in rest.strip().splitlines()[-6:]:
                print(f"  {'':<16}       {line}")
    if bad:
        print(f"\n{bad} robot(s) cannot play this. Give them their own name or key:")
        print(f"  --action-for <ip>=\"<name or linkcraft_resource_...>\"")
        return 1
    keys = {r[3] for r in rows if r[1] == "ok"}
    if len(keys) > 1:
        print("\nNote: the robots resolved to different resource keys. That is "
              "fine - each one plays its own copy.")
    print("\nAll robots can play it.")
    return 0


# --------------------------------------------------------------------------- #
# run (leader)                                                                 #
# --------------------------------------------------------------------------- #

def cmd_run(args):
    os.makedirs(STATE_DIR, exist_ok=True)   # holds the ssh control sockets too
    if getattr(args, "once", False):
        args.max_cycles = 1
    mine = local_addresses()
    remote_hosts, self_hosts = [], []
    for host in args.robots:
        (self_hosts if host in mine else remote_hosts).append(host)

    play_local = bool(self_hosts) and not args.no_local
    if not play_local and not remote_hosts:
        print("Nothing to drive: no remote IPs and this robot is not playing.",
              file=sys.stderr)
        return 1

    print("=== linkcraft_player ===")
    print(f"  action   : {args.action!r}")
    print(f"  robots   : {', '.join(args.robots)}")
    print(f"  this one : {'plays locally (' + ', '.join(self_hosts) + ')' if play_local else 'drives only, does not play'}")
    if args.max_cycles == 1:
        print("  timing   : fire together, once, then exit")
    else:
        print(f"  timing   : fire together, hold {args.duration:g}s, "
              f"then {args.gap:g}s gap, repeat"
              f"{'' if not args.max_cycles else f' (max {args.max_cycles} cycles)'}")
    if args.duration == 0 and args.max_cycles != 1:
        print("             note: --duration is 0, so the gap is measured from the")
        print("             trigger. Pass the action's real length for a true gap.")

    overrides = parse_overrides(args.action_for)
    unknown = set(overrides) - set(args.robots)
    if unknown:
        print(f"--action-for names robots that are not in the list: "
              f"{', '.join(sorted(unknown))}", file=sys.stderr)
        return 1
    peers = [Peer(h, args, overrides.get(h, args.action)) for h in remote_hosts]
    client = None
    step = None

    try:
        if play_local:
            print("\n-- local robot --")
            client = make_ros_client(dry_run=args.dry_run)
            local_action = overrides.get(self_hosts[0], args.action)
            if local_action != args.action:
                print(f"   override: {local_action!r}")
            step = resolve_action(client.list_resources(), local_action)
            print(f"   {step['name']} -> {step['resource_key']} v{step['version']}")
            client.warm_play_service()
            print("   ready")

        for peer in peers:
            print(f"\n-- {peer.host} --")
            if peer.action != args.action:
                print(f"   override: {peer.action!r}")
            info = peer.start_follower(peer.action, args.dry_run)
            try:
                info = json.loads(info)
                print(f"   {info['name']} -> {info['resource_key']} v{info['version']}")
            except (json.JSONDecodeError, KeyError, TypeError):
                pass
            peer.measure_offset()
            print(f"   ready  (clock offset {peer.offset:+.3f}s, rtt {peer.rtt*1000:.0f}ms)")

        print(f"\nRunning. Ctrl+C to stop.\n")
        cycle = 0
        while True:
            cycle += 1
            if args.resync and cycle > 1 and (cycle - 1) % args.resync == 0:
                for peer in [p for p in peers if p.alive]:
                    try:
                        peer.measure_offset()
                    except Exception as exc:
                        print(f"  ! clock resync failed on {peer.host}: {exc}")

            fire_at = time.time() + args.lead
            errors = {}

            def send(peer):
                try:
                    peer.cue(fire_at, cycle)
                except Exception as exc:
                    errors[peer.host] = exc

            threads = [threading.Thread(target=send, args=(p,))
                       for p in peers if p.alive]
            for t in threads:
                t.start()

            status = message = None
            local_fired = local_call_ms = None
            if play_local:
                while time.time() < fire_at:
                    time.sleep(min(0.005, max(0.0, fire_at - time.time())))
                try:
                    local_fired = time.time()
                    status, message = client.play(step["resource_key"],
                                                  step["version"])
                    local_call_ms = (time.time() - local_fired) * 1000.0
                except Exception as exc:
                    # one bad service call should not end a 40-minute demo
                    errors[self_hosts[0]] = exc
            for t in threads:
                t.join()
            # even when this robot only drives, hold until the agreed instant so
            # the gap below is measured from the play, not from the cue
            while time.time() < fire_at:
                time.sleep(min(0.02, max(0.0, fire_at - time.time())))

            stamp = time.strftime("%H:%M:%S", time.localtime(fire_at))
            fired = [h for h in remote_hosts if h not in errors]
            if play_local and self_hosts[0] not in errors:
                fired = self_hosts[:1] + fired
            print(f"[cycle {cycle}] {stamp} fired on {', '.join(fired) or '(nobody)'}"
                  + (f" | local status={status} message={message}"
                     if status is not None else ""))
            for host, exc in errors.items():
                print(f"[cycle {cycle}] ! {host}: {exc}")

            slow = [p for p in peers if p.alive and p.last_send > args.lead * 0.6]
            if slow:
                worst = max(p.last_send for p in slow)
                print(f"[cycle {cycle}] ! cue delivery took up to {worst*1000:.0f}ms "
                      f"of the {args.lead*1000:.0f}ms lead - raise --lead")

            if args.verify:
                parts, spread = [], []
                if local_fired is not None:
                    parts.append(f"{self_hosts[0]} +0ms ({local_call_ms:.0f}ms call)")
                    spread.append(local_fired)
                for peer in peers:
                    if peer.host in errors:
                        continue
                    try:
                        rec = peer.last_play(cycle)
                    except Exception:
                        rec = None
                    if rec is None:
                        parts.append(f"{peer.host} no report")
                        continue
                    base = local_fired if local_fired is not None else fire_at
                    parts.append(f"{peer.host} {(rec['fired'] - base)*1000:+.0f}ms "
                                 f"({rec['call_ms']:.0f}ms call)")
                    spread.append(rec["fired"])
                tail = (f"  spread {max((max(spread) - min(spread)) * 1000, 0):.0f}ms"
                        if len(spread) > 1 else "")
                print(f"[cycle {cycle}] sync: " + " | ".join(parts) + tail)
            if errors and args.stop_on_error:
                print("stopping (--stop-on-error)")
                break

            if args.max_cycles and cycle >= args.max_cycles:
                # Let the last action finish before the followers are torn down.
                # Even with duration 0, hold a couple of seconds: the service
                # call may still be in flight, and killing the follower
                # mid-call would abort it. Playback itself lives on the robot,
                # not in the follower, so the motion survives the teardown.
                time.sleep(max(args.duration, 2.0))
                print("played once." if args.max_cycles == 1
                      else f"reached {args.max_cycles} cycles.")
                break

            # fire-to-fire period is duration + gap; the cue lead comes out of
            # the tail of the gap, not on top of it
            wait = max(0.0, args.duration + args.gap - args.lead)
            next_at = time.strftime("%H:%M:%S",
                                    time.localtime(time.time() + wait + args.lead))
            print(f"           next at {next_at} "
                  f"({args.duration + args.gap:g}s cycle)")
            time.sleep(wait)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        print("Stopping followers...")
        for peer in peers:
            peer.stop_follower()
            try:
                peer.close_master()
            except Exception:
                pass
        if client is not None:
            shutdown_ros(client)
        print("Done.")
    return 0


# --------------------------------------------------------------------------- #

def add_ssh_args(parser):
    parser.add_argument("--user", default=os.environ.get("ROBOT_USER", "agi"),
                        help="ssh user on the robots (default: agi)")
    parser.add_argument("--ssh-key", default=None, help="ssh private key path")
    parser.add_argument("--ssh-port", default=None, help="ssh port")
    parser.add_argument("--remote-script", default="/home/agi/linkcraft_player.py",
                        help="path of this script on the other robots")


def main():
    parser = argparse.ArgumentParser(
        description="Play one LinkCraft action on several X2 robots at once, on a loop.")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_run = sub.add_parser("run", help="drive every robot you list (leader)")
    p_run.add_argument("robots", nargs="+", metavar="IP",
                       help="IPs of every robot that should play, including this one")
    p_run.add_argument("--action", required=True,
                       help=f"LinkCraft action name, or a resource key, used on "
                            f"every robot. Resolved "
                            f"per robot - menu order and keys may differ.")
    p_run.add_argument("--action-for", action="append", default=[], metavar="IP=NAME",
                       help="override for one robot that calls it something else, "
                            "e.g. --action-for 192.168.0.54=\"My action v2\" "
                            "(a linkcraft_resource_... key also works). Repeatable.")
    p_run.add_argument("--gap", type=float, default=120.0,
                       help="idle seconds between plays (default: 120)")
    p_run.add_argument("--duration", type=float, default=0.0,
                       help="how long the action takes; added before the gap")
    p_run.add_argument("--lead", type=float, default=1.5,
                       help="seconds between issuing a cue and the agreed fire "
                            "instant (default: 1.5)")
    p_run.add_argument("--resync", type=int, default=20,
                       help="re-measure peer clocks every N cycles (0 = never)")
    p_run.add_argument("--max-cycles", type=int, default=0,
                       help="stop after N plays (0 = forever)")
    p_run.add_argument("--once", action="store_true",
                       help="play it a single time and exit (same as --max-cycles 1)")
    p_run.add_argument("--no-local", action="store_true",
                       help="drive the listed robots but do not play here")
    p_run.add_argument("--no-verify", dest="verify", action="store_false",
                       default=True,
                       help="skip the after-the-fact sync report (one extra SSH "
                            "per robot per cycle, during the gap)")
    p_run.add_argument("--stop-on-error", action="store_true",
                       help="abort if any robot misses a cue")
    p_run.add_argument("--ros-setup", action="append", default=None,
                       help="setup.bash to source on the peers (repeatable)")
    p_run.add_argument("--dry-run", action="store_true",
                       help="skip ROS everywhere; log what would be played")
    add_ssh_args(p_run)
    p_run.set_defaults(func=cmd_run)

    p_follow = sub.add_parser("follow", help="warm player (started by the leader)")
    p_follow.add_argument("--action", default="",
                          help="empty = generic player, action comes with each cue")
    p_follow.add_argument("--dry-run", action="store_true")
    p_follow.add_argument("--no-watch", action="store_true",
                          help="do not watch the joints / measure start delays")
    p_follow.set_defaults(func=cmd_follow)

    p_list = sub.add_parser("list", help="print this robot's LinkCraft resources")
    p_list.add_argument("--json", action="store_true")
    p_list.add_argument("--dry-run", action="store_true")
    p_list.set_defaults(func=cmd_list)

    p_check = sub.add_parser("check",
                             help="ask every robot what it would play (no motion)")
    p_check.add_argument("robots", nargs="+", metavar="IP")
    p_check.add_argument("--action", required=True)
    p_check.add_argument("--action-for", action="append", default=[], metavar="IP=NAME")
    p_check.add_argument("--no-local", action="store_true",
                         help="treat every IP as remote, even this robot's own")
    p_check.add_argument("--ros-setup", action="append", default=None)
    p_check.add_argument("--dry-run", action="store_true")
    add_ssh_args(p_check)
    p_check.set_defaults(func=cmd_check)

    p_push = sub.add_parser("push", help="scp this script to the robots you list")
    p_push.add_argument("robots", nargs="+", metavar="IP")
    add_ssh_args(p_push)
    p_push.set_defaults(func=cmd_push)

    args = parser.parse_args()
    if getattr(args, "ros_setup", None) is None and hasattr(args, "ros_setup"):
        args.ros_setup = DEFAULT_ROS_SETUP
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
