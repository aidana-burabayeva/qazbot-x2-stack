#!/usr/bin/env python3
"""Import the pinned third-party sources into ws/src/third_party (vcs-style, idempotent) and apply ws/third_party/patches.

    python3 ws/third_party/import_sources.py [--git-cache DIR]

For every entry of ws/third_party.repos (parents before nested paths, file order), DEST = ws/src/third_party/<path>:
  * missing/empty DEST -> git init + shallow fetch of exactly the pinned commit (from --git-cache / $QAZBOT_TP_GIT_CACHE
    when it has the commit, else from the url) + detached checkout;
  * HEAD must equal the pin, otherwise REFUSED;
  * patch state (ws/third_party/patches/<dir>/*.patch, PATCH_DIRS below), compared as git tree ids of the working
    tree: == pin + patches -> ok (nothing done); == pin (pristine) -> patches applied; anything else -> REFUSED
    (the checkout was modified unexpectedly; inspect it, then delete DEST to re-import).
Nothing is ever reset, cleaned or force-checked-out over existing content. Exit status != 0 on any failure.
"""
import argparse
import os
import shutil
import subprocess
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)
DEST = os.path.join(WS, 'src', 'third_party')
PATCH_DIRS = {'FAST_LIO': 'fast_lio', 'navigation2': 'nav2_smac_planner'}
# abort stalled HTTPS transfers (< 1 kB/s for 60 s) instead of hanging; retried
NET = ['-c', 'http.lowSpeedLimit=1000', '-c', 'http.lowSpeedTime=60']


def git(d, *a, env=None, check=True):
    r = subprocess.run(['git', '-C', d, *a], capture_output=True, text=True, env=env)
    if check and r.returncode != 0:
        raise SystemExit(f'git {" ".join(a)} in {d} failed: {r.stderr.strip()}')
    return r.stdout.strip()


def patches(path):
    sub = PATCH_DIRS.get(path)
    if not sub:
        return []
    d = os.path.join(HERE, 'patches', sub)
    return [os.path.join(d, f) for f in sorted(os.listdir(d)) if f.endswith('.patch')]


def tree_of(d, ps=None):
    """Tree id of the working tree (ps None) or of HEAD + patches ps, via a temporary index (real index untouched)."""
    idx = os.path.join(git(d, 'rev-parse', '--absolute-git-dir'), 'qazbot_verify.index')
    env = dict(os.environ, GIT_INDEX_FILE=idx)
    try:
        if ps is None:
            real = os.path.join(git(d, 'rev-parse', '--absolute-git-dir'), 'index')
            if os.path.isfile(real):
                shutil.copyfile(real, idx)          # stat cache: fast on big trees
            else:
                git(d, 'read-tree', 'HEAD', env=env)
            git(d, 'add', '-A', '.', env=env)
        else:
            git(d, 'read-tree', 'HEAD', env=env)
            for p in ps:
                git(d, 'apply', '--cached', p, env=env)
        return git(d, 'write-tree', env=env)
    finally:
        if os.path.exists(idx):
            os.remove(idx)


def fetch(d, url, version, cache):
    os.makedirs(d, exist_ok=True)
    git(d, 'init', '-q')
    git(d, 'remote', 'add', 'origin', url)
    c = os.path.join(cache, os.path.relpath(d, DEST)) if cache else ''
    if c and os.path.isdir(os.path.join(c, '.git')) and subprocess.run(
            ['git', '-C', c, 'cat-file', '-e', version + '^{commit}'], capture_output=True).returncode == 0:
        git(d, 'fetch', '-q', '--depth', '1', c, version)
    else:
        for _ in range(3):
            if subprocess.run(['git', *NET, '-C', d, 'fetch', '-q', '--depth', '1', 'origin', version]).returncode == 0:
                break
        else:
            raise SystemExit(f'{d}: fetch of {version} from {url} failed')
    git(d, 'checkout', '-q', '--detach', version)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--git-cache', default=os.environ.get('QAZBOT_TP_GIT_CACHE', ''))
    a = ap.parse_args(argv)
    repos = yaml.safe_load(open(os.path.join(WS, 'third_party.repos')))['repositories']
    for path, r in repos.items():
        d = os.path.join(DEST, path)
        if not os.path.isdir(d) or not os.listdir(d):
            fetch(d, r['url'], r['version'], a.git_cache)
            print(f'imported {path} @ {r["version"]}')
        elif git(d, 'rev-parse', '--show-toplevel', check=False) != os.path.realpath(d):
            raise SystemExit(f'REFUSED: {d} exists but is not a git checkout; delete it to re-import')
        head = git(d, 'rev-parse', 'HEAD')
        if head != r['version']:
            raise SystemExit(f'REFUSED: {d} HEAD {head} != pinned {r["version"]} (ws/third_party.repos); '
                             f'delete {d} to re-import')
        ps = patches(path)
        want, have, pristine = tree_of(d, ps), tree_of(d), tree_of(d, [])
        if have == want:
            state = 'pinned + patched' if ps else 'pinned'
        elif have == pristine:
            for p in ps:
                git(d, 'apply', p)
            if tree_of(d) != want:
                raise SystemExit(f'REFUSED: {d}: applying {ps} did not give the expected tree')
            state = 'pinned, patches applied: ' + ' '.join(os.path.basename(p) for p in ps)
        else:
            changed = git(d, 'status', '--porcelain')
            raise SystemExit(f'REFUSED: {d} is modified unexpectedly (neither pristine pin nor pin + patches):\n'
                             f'{changed}\ninspect it, then delete {d} to re-import')
        print(f'{path} @ {head[:12]}: {state}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
