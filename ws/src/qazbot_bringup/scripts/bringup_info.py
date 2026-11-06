#!/usr/bin/env python3
"""Bringup guard + one-line summary for x2_navigation.launch.xml.

Called by the launch through $(command ...): prints the summary that the launch logs at INFO (workspace, git SHA
from the installed manifest, mode, profile, odom source, LiDAR source, TF owner, control channel) and exits non-zero
(message on stderr, launch aborts) when the requested mode does not match the environment:
  lab         ROS_DOMAIN_ID must be set and != 0, and the graph must be host-local (ROS_LOCALHOST_ONLY=1 or a
              FastDDS profile whitelisting only 127.0.0.1); the motion adapter runs dry (no /aima/* publisher).
  production  ROS_DOMAIN_ID unset or 0 (the AgiBot MC graph); the motion adapter is live; only from a release:
              <release>/manifest.yaml (or QAZBOT_RELEASE_MANIFEST) with a 40-hex sha and dirty: false. A dev
              build_manifest.yaml is never accepted.
Pure functions below are unit-tested (test/test_bringup_info.py).
"""
import argparse
import os
import re
import sys

MODES = ('lab', 'production')
ODOM_SOURCES = ('lio',)
NS = 'qazbot_d1_map'


def install_root(here=None):
    """<install> from <install>/qazbot_bringup/lib/qazbot_bringup/bringup_info.py (or QAZBOT_INSTALL)."""
    if os.environ.get('QAZBOT_INSTALL'):
        return os.environ['QAZBOT_INSTALL']
    here = here or os.path.dirname(os.path.realpath(__file__))
    return os.path.abspath(os.path.join(here, '..', '..', '..'))


def read_manifest(paths):
    """First existing manifest among paths -> (path, {key: value}) with flat 'key: value' lines."""
    for p in paths:
        if p and os.path.isfile(p):
            out = {}
            with open(p) as f:
                for line in f:
                    m = re.match(r'^([A-Za-z0-9_]+):\s*(.*?)\s*$', line)
                    if m and not line.startswith(' '):
                        out[m.group(1)] = m.group(2).strip('"\'')
            return p, out
    return None, {}


def manifest_candidates(root, env=os.environ):
    return [env.get('QAZBOT_RELEASE_MANIFEST'),
            os.path.join(os.path.dirname(root), 'manifest.yaml'),          # <release>/manifest.yaml
            os.path.join(root, 'qazbot_bringup', 'share', 'qazbot_bringup', 'build_manifest.yaml')]


def release_manifest_candidates(root, env=os.environ):
    return [env.get('QAZBOT_RELEASE_MANIFEST'), os.path.join(os.path.dirname(root), 'manifest.yaml')]


def release_errors(root, env=os.environ):
    """Reasons why root is not a clean release (production gate)."""
    path, m = read_manifest(release_manifest_candidates(root, env))
    if not path:
        return ['mode production runs only from a release: no <release>/manifest.yaml and no QAZBOT_RELEASE_MANIFEST '
                '(build with docker/deploy/build_release.sh)']
    errs = []
    if m.get('kind') != 'release':
        errs.append(f'manifest {path} is not kind: release (kind={m.get("kind", "missing")}); only docker/deploy/build_release.sh makes a release')
    if not re.fullmatch(r'[0-9a-f]{40}', m.get('sha', '')):
        errs.append(f'release manifest {path} has no 40-hex sha')
    if m.get('dirty', '').lower() != 'false':
        errs.append(f'release manifest {path} is not dirty: false (dirty={m.get("dirty", "missing")})')
    return errs


def loopback_only_profile(path):
    """True when the FastDDS profile whitelists only 127.0.0.1 (no other interface address)."""
    if not path or not os.path.isfile(path):
        return False
    with open(path) as f:
        addrs = re.findall(r'<address>\s*([^<\s]+)\s*</address>', f.read())
    return bool(addrs) and all(a == '127.0.0.1' for a in addrs)


def check_environment(mode, odom_source, env=os.environ, root=None):
    """List of reasons why this mode must not start here (empty = ok)."""
    errors = []
    if mode not in MODES:
        errors.append(f'mode must be one of {MODES}, got {mode!r}')
    if odom_source not in ODOM_SOURCES:
        errors.append(f'odom_source must be one of {ODOM_SOURCES}, got {odom_source!r} '
                      '(native_imu was removed 2026-11-05, tag pre-consolidation-fb18dc1)')
    domain = env.get('ROS_DOMAIN_ID', '').strip()
    if mode == 'lab':
        if domain in ('', '0'):
            errors.append('mode lab needs an isolated ROS_DOMAIN_ID (set and != 0, e.g. 88); refusing domain 0, '
                          'the robot graph with MC')
        if env.get('ROS_LOCALHOST_ONLY') != '1' and not loopback_only_profile(env.get('FASTRTPS_DEFAULT_PROFILES_FILE')):
            errors.append('mode lab needs ROS_LOCALHOST_ONLY=1 or a loopback-only FastDDS profile '
                          '(qazbot_bringup config/dds_loopback.xml)')
    if mode == 'production' and domain not in ('', '0'):
        errors.append(f'mode production runs on the robot graph (ROS_DOMAIN_ID unset or 0), got {domain!r}')
    if mode == 'production':
        errors += release_errors(root if root is not None else install_root(), env)
    return errors


def lidar_topic(x2_yaml):
    try:
        with open(x2_yaml) as f:
            m = re.search(r'lid_topic:\s*"?([^"\s]+)"?', f.read())
        return m.group(1) if m else 'unknown'
    except OSError:
        return 'unknown'


def summary(mode, profile, odom_source, root, manifest_path, manifest, lidar):
    sha = manifest.get('sha', 'unknown')
    dirty = manifest.get('dirty', 'unknown')
    control = (f'/{NS}/cmd_vel_authorized -> x2_motion_adapter LIVE (SDK source, priority 30, ENABLE) -> '
               '/aima/mc/locomotion/velocity' if mode == 'production' else
               f'/{NS}/cmd_vel_authorized -> x2_motion_adapter DRY RUN (~/dry_run_cmd, no /aima/* publisher)')
    return (f'QAZBOT BRINGUP mode={mode} profile={profile} workspace={root} git_sha={sha} dirty={dirty} '
            f'manifest={manifest_path or "none"} odom_source={odom_source} lidar_source={lidar} -> FAST-LIO '
            f'tf_owner=[odom->base_footprint: qazbot_level_bridge (C++ qazbot_lidar_projector); map->odom: amcl; '
            f'tree /{NS}/tf] control_channel=[{control}] domain={os.environ.get("ROS_DOMAIN_ID", "0")}')


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', required=True)
    ap.add_argument('--profile', required=True)
    ap.add_argument('--odom-source', required=True)
    ap.add_argument('--profile-dir', default='')
    a = ap.parse_args(argv)
    root = install_root()
    errors = check_environment(a.mode, a.odom_source, root=root)
    if a.profile_dir and not os.path.isdir(a.profile_dir):
        errors.append(f'profile {a.profile!r} has no directory {a.profile_dir}')
    if errors:
        sys.stderr.write('[ERROR] QAZBOT BRINGUP REFUSED: ' + '; '.join(errors) + '\n')
        return 1
    path, manifest = read_manifest(manifest_candidates(root))
    lidar = lidar_topic(os.path.join(root, 'qazbot_localization', 'share', 'qazbot_localization', 'config', 'x2.yaml'))
    sys.stdout.write(summary(a.mode, a.profile, a.odom_source, root, path, manifest, lidar))
    return 0


if __name__ == '__main__':
    sys.exit(main())
