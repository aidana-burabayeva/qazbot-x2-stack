#!/usr/bin/env python3
"""Audit of every remote branch against the two permanent branches (read-only except the non-destructive archive tags).

Classes: ACTIVE | INTEGRATED | SUPERSEDED | NEEDS_REVIEW | ARCHIVED. Nothing is deleted, force-pushed or reset; history stays reachable by branch AND by an
`archive/<branch>` tag when requested. usage: branch_audit.py [--repo .] [--tag] [--json out.json] [--md out.md]
"""
import argparse
import json
import subprocess
import time

STABLE = ('robot/stable', 'lab/stable')
PERMANENT_NOTE = {'robot/stable': 'permanent: installed X2 state', 'lab/stable': 'permanent: SIM_VERIFIED lab state'}
KEEP_UNTOUCHED = {'main': 'untouched by operator decision (not the installed X2 state, not the lab state)'}
RECENT_S = 48 * 3600


def git(repo, *a):
    r = subprocess.run(['git', '-C', repo, *a], capture_output=True, text=True)
    return r.returncode, r.stdout.strip()


def audit(repo='.'):
    rc, out = git(repo, 'for-each-ref', '--format=%(refname:short)|%(objectname)|%(committerdate:unix)|%(subject)', 'refs/remotes/origin')
    branches = {}
    for ln in out.splitlines():
        name, sha, ts, subj = ln.split('|', 3)
        name = name[len('origin/'):]
        if name in ('HEAD', '') or name.startswith('archive/'):
            continue
        branches[name] = dict(name=name, sha=sha, ts=int(ts), subject=subj)
    tips = {n: b['sha'] for n, b in branches.items()}
    stable_tips = {s: tips[s] for s in STABLE if s in tips}
    now = time.time()
    rows = []
    for n, b in sorted(branches.items()):
        sha = b['sha']
        merged_in = [s for s, t in stable_tips.items() if s != n and git(repo, 'merge-base', '--is-ancestor', sha, t)[0] == 0]
        later = [m for m, t in tips.items() if m != n and m not in STABLE and t != sha and git(repo, 'merge-base', '--is-ancestor', sha, t)[0] == 0]
        ahead = {s: git(repo, 'rev-list', '--count', f'{t}..{sha}')[1] for s, t in stable_tips.items() if s != n}
        if n in STABLE:
            cls, why = 'ACTIVE', PERMANENT_NOTE[n]
        elif n in KEEP_UNTOUCHED:
            cls, why = 'ARCHIVED', KEEP_UNTOUCHED[n]
        elif merged_in:
            cls, why = 'INTEGRATED', f'tip is contained in {", ".join(merged_in)}'
        elif later:
            cls, why = 'SUPERSEDED', f'tip is an ancestor of {", ".join(later[:3])}'
        elif now - b['ts'] < RECENT_S:
            cls, why = 'ACTIVE', 'recent work not yet in a stable branch'
        else:
            cls, why = 'NEEDS_REVIEW', 'not contained in a stable branch, no newer branch contains it, not recent: a human/agent must decide'
        rows.append(dict(branch=n, sha=sha[:12], date=time.strftime('%Y-%m-%d %H:%M', time.localtime(b['ts'])), cls=cls, why=why, ahead_of=ahead, subject=b['subject'][:80]))
    return rows


def render_md(rows):
    L = ['# Branch migration (audit)', '', 'Two permanent branches: **`robot/stable`** (installed X2 state) and **`lab/stable`** (SIM_VERIFIED lab state). Nothing was deleted, reset or force-pushed; every tip below is still reachable and tagged `archive/<branch>`.', '',
         '| branch | tip | date | class | reason | commits ahead of robot/stable, lab/stable |', '|---|---|---|---|---|---|']
    for r in rows:
        L.append(f"| `{r['branch']}` | `{r['sha']}` | {r['date']} | **{r['cls']}** | {r['why']} | {r['ahead_of'].get('robot/stable', '-')}, {r['ahead_of'].get('lab/stable', '-')} |")
    counts = {c: sum(1 for r in rows if r['cls'] == c) for c in ('ACTIVE', 'INTEGRATED', 'SUPERSEDED', 'NEEDS_REVIEW', 'ARCHIVED')}
    L += ['', 'Counts: ' + ', '.join(f'{k}={v}' for k, v in counts.items()) + '.', '']
    return '\n'.join(L)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--repo', default='.'); ap.add_argument('--tag', action='store_true'); ap.add_argument('--json'); ap.add_argument('--md')
    a = ap.parse_args()
    rows = audit(a.repo)
    if a.tag:
        for r in rows:
            if r['cls'] != 'ACTIVE' or r['branch'] not in STABLE:
                full = git(a.repo, 'rev-parse', f"origin/{r['branch']}")[1]
                git(a.repo, 'tag', f"archive/{r['branch'].replace('/', '-')}", full)   # fails silently if it exists: tags are never moved
    if a.json:
        open(a.json, 'w').write(json.dumps(rows, indent=1))
    md = render_md(rows)
    if a.md:
        open(a.md, 'w').write(md)
    print(md)


if __name__ == '__main__':
    main()
