#!/usr/bin/env python3
"""Bringup guard + one-line summary for x2_navigation.launch.xml.

Called by the launch through $(command ...): prints the summary that the launch logs at INFO (workspace, git SHA
from the installed manifest, mode, profile, odom source, LiDAR source, TF owner, control channel) and exits non-zero
(message on stderr, launch aborts) when the requested mode does not match the environment:
  lab         ROS_DOMAIN_ID must be set and != 0, and the graph must be host-local (ROS_LOCALHOST_ONLY=1 or a
              FastDDS profile whitelisting only 127.0.0.1); the motion adapter runs dry (no /aima/* publisher).
  production  ROS_DOMAIN_ID unset or 0 (the AgiBot MC graph); the motion adapter is live. Only a RELEASE build:
              the running install is <repo>/ws/install of a git checkout, <install>/qazbot_build_manifest.yaml
              (ws/tools/build.sh) has sha == `git rev-parse HEAD`, dirty: false and workspace == <repo>, the working
              tree is clean, HEAD carries an annotated x2-release-YYYYMMDD-N tag; env.sh was sourced with
              QAZBOT_MODE=production from this install and no candidate overlay (QAZBOT_SMAC_PREFIX,
              QAZBOT_LIFECYCLE_PREFIX) is set.
  --release-check  prints only the build/git part of the production gate for this install (ws/tools/status.sh).
Pure functions below are unit-tested (test/test_bringup_info.py).
"""
import argparse
import os
import re
import subprocess
import sys

MODES = ('lab', 'production')
ODOM_SOURCES = ('lio',)
NS = 'qazbot_d1_map'
MANIFEST = 'qazbot_build_manifest.yaml'          # written by ws/tools/build.sh into <repo>/ws/install
RELEASE_TAG = re.compile(r'x2-release-[0-9]{8}-[0-9]+')
OVERLAYS = ('QAZBOT_SMAC_PREFIX', 'QAZBOT_LIFECYCLE_PREFIX')


def install_root(here=None):
    """<install> from <install>/qazbot_bringup/lib/qazbot_bringup/bringup_info.py (the running script itself)."""
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


def manifest_candidates(root):
    """Workspace manifest (ws/tools/build.sh), else the per-package one of a plain colcon build (summary only)."""
    return [os.path.join(root, MANIFEST),
            os.path.join(root, 'qazbot_bringup', 'share', 'qazbot_bringup', 'build_manifest.yaml')]


def _git(repo, *args):
    r = subprocess.run(['git', '-C', repo, *args], capture_output=True, text=True)
    return r.stdout.strip() if r.returncode == 0 else None


def release_tags(repo):
    """Annotated x2-release-YYYYMMDD-N tags on HEAD of repo."""
    out = _git(repo, 'tag', '--points-at', 'HEAD', '--list', 'x2-release-*') or ''
    return [t for t in out.split() if RELEASE_TAG.fullmatch(t) and _git(repo, 'cat-file', '-t', t) == 'tag']


def release_errors(root):
    """Reasons why the install root is not a release build (build/git part of the production gate)."""
    root = os.path.realpath(root)
    ws, path = os.path.dirname(root), os.path.join(root, MANIFEST)
    repo = os.path.dirname(ws)
    top = _git(repo, 'rev-parse', '--show-toplevel')
    if os.path.basename(root) != 'install' or os.path.basename(ws) != 'ws' or not top or os.path.realpath(top) != repo:
        return [f'install {root} is not <repo>/ws/install of a git checkout (production runs only from the '
                'in-place workspace built by ws/tools/build.sh)']
    _, m = read_manifest([path])
    if not m:
        return [f'no build manifest {path} (build with ws/tools/build.sh)']
    errs = []
    head = _git(repo, 'rev-parse', 'HEAD')
    if m.get('sha') != head:
        errs.append(f'manifest sha {m.get("sha", "missing")} != HEAD {head} of {repo} (rebuild: ws/tools/build.sh)')
    if m.get('dirty', '').lower() != 'false':
        errs.append(f'manifest {path} is not dirty: false (dirty={m.get("dirty", "missing")})')
    if not m.get('workspace') or os.path.realpath(m['workspace']) != repo:
        errs.append(f'manifest workspace {m.get("workspace", "missing")!r} != {repo} (built elsewhere; rebuild here)')
    if _git(repo, 'status', '--porcelain') != '':
        errs.append(f'working tree {repo} is not clean (git status)')
    if not release_tags(repo):
        errs.append(f'HEAD {head} of {repo} has no annotated x2-release-YYYYMMDD-N tag')
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
        root = os.path.realpath(root if root is not None else install_root())
        if env.get('QAZBOT_MODE') != 'production':
            errors.append('mode production needs env.sh sourced with QAZBOT_MODE=production '
                          f'(robot DDS profile), got QAZBOT_MODE={env.get("QAZBOT_MODE", "unset")!r}')
        if env.get('QAZBOT_INSTALL') and os.path.realpath(env['QAZBOT_INSTALL']) != root:
            errors.append(f'env.sh of another install was sourced (QAZBOT_INSTALL={env["QAZBOT_INSTALL"]}, running {root})')
        errors += [f'candidate overlay {k}={env[k]} is not allowed in production' for k in OVERLAYS if env.get(k)]
        errors += release_errors(root)
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
    tag = manifest.get('release_tag') if manifest.get('release_tag') not in (None, '', 'null') else 'none'
    control = (f'/{NS}/cmd_vel_authorized -> x2_motion_adapter LIVE (SDK source, priority 30, ENABLE) -> '
               '/aima/mc/locomotion/velocity' if mode == 'production' else
               f'/{NS}/cmd_vel_authorized -> x2_motion_adapter DRY RUN (~/dry_run_cmd, no /aima/* publisher)')
    return (f'QAZBOT BRINGUP mode={mode} profile={profile} workspace={root} git_sha={sha} dirty={dirty} release_tag={tag} '
            f'manifest={manifest_path or "none"} odom_source={odom_source} lidar_source={lidar} -> FAST-LIO '
            f'tf_owner=[odom->base_footprint: qazbot_level_bridge (C++ qazbot_lidar_projector); map->odom: amcl; '
            f'tree /{NS}/tf] control_channel=[{control}] domain={os.environ.get("ROS_DOMAIN_ID", "0")}')


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--release-check', action='store_true', help='only the build/git part of the production gate')
    ap.add_argument('--mode')
    ap.add_argument('--profile')
    ap.add_argument('--odom-source')
    ap.add_argument('--profile-dir', default='')
    a = ap.parse_args(argv)
    root = install_root()
    if a.release_check:
        errors = release_errors(root)
        sys.stdout.write('release: ' + ('yes' if not errors else 'NO: ' + '; '.join(errors)) + '\n')
        return 1 if errors else 0
    if not (a.mode and a.profile and a.odom_source):
        ap.error('--mode, --profile and --odom-source are required')
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
