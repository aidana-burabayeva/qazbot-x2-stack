"""Last CONFIRMED localization pose, persisted across localization restarts (pure Python, no ROS; tested without a robot).

Writer: localization_trust_node (param pose_store) saves at the READY rising edge (operator confirm_anchor + clean gate)
and at a clean shutdown while READY with clean diagnostics. Never on plain AMCL output.
Reader: seed_pose.py --from-file publishes the record ONCE as initialpose with its saved covariance. The seeded pose
is only a hypothesis: the trust node treats it like any initialpose (READY cleared until confirm_anchor + gate).
Missing / corrupt / other-map files are refused (load returns a reason); the operator then poses as before.
"""
import hashlib
import json
import math
import os
import time

FORMAT = 1
DEFAULT_PATH = os.path.join(os.environ.get('ROS_HOME', os.path.expanduser('~/.ros')), 'qazbot_x2_last_confirmed_pose.json')


def map_image_sha256(map_yaml):
    """sha256 of the image a Nav2 map yaml points to (same identity as the trust node's map_sha256)."""
    import yaml
    with open(os.path.abspath(map_yaml)) as f:
        image = os.path.join(os.path.dirname(os.path.abspath(map_yaml)), yaml.safe_load(f)['image'])
    with open(image, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


def _check(pose, cov):
    """'' when pose (x, y, yaw) and the row-major 6x6 covariance can seed AMCL (it reads xy 2x2 and yaw-yaw), else a reason."""
    if len(pose) != 3 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in pose):
        return 'pose'
    if len(cov) != 36 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in cov):
        return 'covariance'
    cxx, cxy, cyx, cyy, caa = cov[0], cov[1], cov[6], cov[7], cov[35]
    if cxy != cyx or cxx <= 0 or cyy <= 0 or caa <= 0 or cxx * cyy - cxy * cxy <= 0:
        return 'covariance'
    return ''


def save(path, pose, cov, map_sha256, source, **extra):
    """Atomic write (tmp + fsync + os.replace): a crash leaves the old file or the new one, never a partial one."""
    pose, cov = [float(v) for v in pose], [float(v) for v in cov]
    bad = _check(pose, cov)
    if bad or not map_sha256:
        raise ValueError('not saved: ' + (bad or 'map_sha256'))
    rec = dict(format=FORMAT, x=pose[0], y=pose[1], yaw=pose[2], covariance=cov, map_sha256=map_sha256,
               source=source, saved_unix=time.time(), **extra)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(rec, f)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)
    return rec


def load(path, map_sha256):
    """(record, '') when the file is usable on the map with image hash `map_sha256`, else (None, reason). Never raises."""
    try:
        with open(path) as f:
            rec = json.load(f)
    except FileNotFoundError:
        return None, 'no_file'
    except (OSError, ValueError) as e:
        return None, 'unreadable: %s' % e
    if not isinstance(rec, dict) or rec.get('format') != FORMAT:
        return None, 'format'
    if rec.get('map_sha256') != map_sha256:
        return None, 'map_sha256_mismatch'
    bad = _check([rec.get(k) for k in ('x', 'y', 'yaw')], rec.get('covariance') or [])
    return (None, bad) if bad else (rec, '')


class ConfirmedPoseSaver:
    """Saves the trust node's CONFIRMED pose: on the READY rising edge, and at shutdown only while READY with clean
    checks. Disabled when path is empty. on_verdict / on_shutdown return the saved record or None."""
    SHUTDOWN_IGNORE = ('T4_no_baseline',)      # baseline is optional for READY, so its absence is not a pose doubt

    def __init__(self, path, map_sha256, log=print):
        self.path, self.map_sha256, self.log = path, map_sha256, log
        self.was_ready = False

    def _save(self, source, pose, cov, map_ok):
        if not self.path:
            return None
        if map_ok != '' or pose is None or cov is None:
            self.log('confirmed pose NOT saved (%s): map_ok=%r pose=%r' % (source, map_ok, pose))
            return None
        try:
            rec = save(self.path, pose, cov, self.map_sha256, source)
        except (OSError, ValueError) as e:
            self.log('confirmed pose NOT saved (%s): %s' % (source, e))
            return None
        self.log('confirmed pose saved (%s) to %s: (%.3f, %.3f, %.3f)' % (source, self.path, rec['x'], rec['y'], rec['yaw']))
        return rec

    def on_verdict(self, verdict, pose, cov, map_ok):
        """Call after every evaluation; saves once per READY rising edge."""
        rising = bool(verdict['ready']) and not self.was_ready
        self.was_ready = bool(verdict['ready'])
        return self._save('ready', pose, cov, map_ok) if rising else None

    def on_shutdown(self, verdict, pose, cov, map_ok):
        """Clean stop: refresh the file with the current pose only if the latest evaluation was READY, MOTION_OK and
        had no diagnostic (T2/T3/T4/T6/T7) doubt; otherwise keep the last confirmed record."""
        doubt = [r for r in verdict.get('diagnostics', []) if r not in self.SHUTDOWN_IGNORE]
        if verdict.get('ready') and verdict.get('motion_ok') and not doubt:
            return self._save('shutdown', pose, cov, map_ok)
        if self.path:
            self.log('confirmed pose not refreshed at shutdown: ready=%s motion_ok=%s diagnostics=%s'
                     % (verdict.get('ready'), verdict.get('motion_ok'), doubt))
        return None
