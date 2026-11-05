#!/usr/bin/env python3
"""QazBot Operator Skills package checker (stdlib only). One Git repo is the single source of truth:
skills live in .claude/skills/ of every checkout (LAB laptop and X2 workspace) and travel by `git fetch` + ff-only merge.

  opskills.py validate [--role lab|robot|auto] [--json]   package integrity, operator-only flag, drift, name conflicts, contract
  opskills.py hashes --update                              re-pin SKILL.md sha256 after a deliberate edit (bump version first)
  opskills.py sync [--ref origin/<branch>]                 is a newer compatible package available? (read-only, never pulls)
  opskills.py role                                         LAB | ROBOT | UNKNOWN (env QAZBOT_ROLE or ~/.config/qazbot/role)

Never contacts the robot, never pulls/merges/resets by itself.
"""
import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(os.environ.get('OPSKILLS_ROOT', Path(__file__).resolve().parents[2]))
SKILLS = ROOT / '.claude' / 'skills'
MANIFEST = SKILLS / 'OPERATOR_SKILLS.json'
HANDOFF = ROOT / 'docs' / 'hardware' / 'LAB_TO_ROBOT_HANDOFF.md'
FEEDBACK = ROOT / 'docs' / 'hardware' / 'ROBOT_TO_LAB_FEEDBACK.md'
CONTRACT_RE = re.compile(r'<!-- QAZBOT-CONTRACT (\{.*?\}) -->')
ROLES = ('LAB', 'ROBOT')


def detect_role():
    r = os.environ.get('QAZBOT_ROLE')
    if not r:
        f = Path(os.environ.get('QAZBOT_ROLE_FILE', Path.home() / '.config' / 'qazbot' / 'role'))
        r = f.read_text().strip() if f.exists() else ''
    r = r.upper()
    return r if r in ROLES else 'UNKNOWN'


def semver(v):
    m = re.fullmatch(r'(\d+)\.(\d+)\.(\d+)', v or '')
    return tuple(int(x) for x in m.groups()) if m else None


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def frontmatter(p):
    t = Path(p).read_text()
    m = re.match(r'---\n(.*?)\n---\n', t, re.S)
    fm = {}
    if m:
        for ln in m.group(1).splitlines():
            if ':' in ln and not ln.startswith(' '):
                k, v = ln.split(':', 1)
                fm[k.strip()] = v.strip()
    return fm


def contract_block(path):
    if not Path(path).exists():
        return None
    m = CONTRACT_RE.search(Path(path).read_text())
    return json.loads(m.group(1)) if m else None


def load_manifest(path=MANIFEST):
    return json.loads(Path(path).read_text())


def external_names():
    home = Path.home() / '.claude'
    names = set()
    for d in (home / 'skills', home / 'commands'):
        if d.exists():
            names |= {p.stem if p.is_file() else p.name for p in d.iterdir()}
    return names


def validate(role='auto'):
    errs, warns, info = [], [], {}
    if not MANIFEST.exists():
        return ['manifest missing: ' + str(MANIFEST)], [], {}
    m = load_manifest()
    if not semver(m.get('version')):
        errs.append('package version is not semver')
    if not isinstance(m.get('contract_version'), int):
        errs.append('contract_version must be int')
    managed = m.get('skills', {})
    unmanaged = set(m.get('unmanaged', []))
    dirs = {p.name for p in SKILLS.iterdir() if (p / 'SKILL.md').exists()} if SKILLS.exists() else set()
    for name in sorted(dirs - set(managed) - unmanaged):
        errs.append(f'skill {name} is neither managed nor listed as unmanaged')
    ext = external_names()
    r = detect_role() if role == 'auto' else role.upper()
    for name, spec in managed.items():
        p = SKILLS / name / 'SKILL.md'
        if not p.exists():
            if spec.get('role') != r or r == 'UNKNOWN':  # role-subset install: the other role's skills need not be present
                info.setdefault('not_installed', []).append(name)
                if r == 'UNKNOWN':
                    errs.append(f'{name}: SKILL.md missing (role unknown, full package expected)')
            else:
                errs.append(f'{name}: SKILL.md missing')
            continue
        if spec.get('role') not in ROLES:
            errs.append(f'{name}: role must be LAB or ROBOT')
        fm = frontmatter(p)
        if fm.get('name') != name:
            errs.append(f'{name}: frontmatter name is {fm.get("name")!r}')
        if not fm.get('description'):
            errs.append(f'{name}: description missing')
        if fm.get('disable-model-invocation', '').lower() != 'true':
            errs.append(f'{name}: control skill must be operator-only (disable-model-invocation: true)')
        if not semver(spec.get('version')):
            errs.append(f'{name}: version is not semver')
        if spec.get('sha256') != sha(p):
            errs.append(f'{name}: SKILL.md changed since pinned (bump version, then `opskills.py hashes --update`)')
        if name in ext:
            errs.append(f'{name}: name conflicts with a user-level skill/command in ~/.claude')
    info['role'] = r
    info['active_for_role'] = sorted(n for n, s in managed.items() if s.get('role') == r and n not in info.get('not_installed', []))
    info['inactive_for_role'] = sorted(n for n, s in managed.items() if s.get('role') != r)
    if r == 'UNKNOWN':
        warns.append('role unknown: set QAZBOT_ROLE=LAB|ROBOT; control skills must refuse hardware actions')
    hb = contract_block(HANDOFF)
    if HANDOFF.exists():
        if not hb:
            warns.append('LAB_TO_ROBOT_HANDOFF.md has no QAZBOT-CONTRACT block (pre-contract document)')
        elif hb.get('contract_version') != m.get('contract_version'):
            errs.append(f'handoff contract_version {hb.get("contract_version")} != package {m.get("contract_version")}')
    fb = contract_block(FEEDBACK)
    if FEEDBACK.exists() and fb and fb.get('contract_version') != m.get('contract_version'):
        errs.append('feedback contract_version differs from package')
    info['package_version'] = m.get('version')
    return errs, warns, info


def git(*a):
    r = subprocess.run(['git', '-C', str(ROOT), *a], capture_output=True, text=True)
    return r.returncode, r.stdout.strip()


def sync(ref=None):
    rc, br = git('rev-parse', '--abbrev-ref', 'HEAD')
    ref = ref or (git('rev-parse', '--abbrev-ref', '@{u}')[1] or '')
    out = dict(branch=br, ref=ref, local=load_manifest()['version'] if MANIFEST.exists() else None)
    if not ref:
        out['verdict'] = 'NO_UPSTREAM'; return out
    rc, txt = git('show', f'{ref}:.claude/skills/OPERATOR_SKILLS.json')
    if rc != 0:
        out['verdict'] = 'REMOTE_HAS_NO_PACKAGE'; return out
    rm = json.loads(txt)
    out['remote'] = rm['version']
    l, r = semver(out['local']), semver(rm['version'])
    a, b = (git('rev-list', '--left-right', '--count', f'HEAD...{ref}')[1].split() + ['?', '?'])[:2]
    out.update(ahead=a, behind=b, dirty=bool(git('status', '--porcelain', '--untracked-files=no')[1]))
    if l is None or r is None:
        out['verdict'] = 'INVALID_VERSION'
    elif r[0] != l[0] or rm.get('contract_version') != load_manifest().get('contract_version'):
        out['verdict'] = 'INCOMPATIBLE_MAJOR'
    elif r > l:
        out['verdict'] = 'UPDATE_AVAILABLE'
        if out['dirty'] or (a not in ('0', '?')):
            out['note'] = 'dirty or diverged: do not pull blindly; integrate in a clean worktree'
        else:
            out['note'] = f'git merge --ff-only {ref}'
    else:
        out['verdict'] = 'UP_TO_DATE'
    return out


def update_hashes():
    m = load_manifest()
    for n, s in m['skills'].items():
        s['sha256'] = sha(SKILLS / n / 'SKILL.md')
    MANIFEST.write_text(json.dumps(m, indent=1) + '\n')


def main():
    ap = argparse.ArgumentParser()
    sp = ap.add_subparsers(dest='c', required=True)
    v = sp.add_parser('validate'); v.add_argument('--role', default='auto'); v.add_argument('--json', action='store_true')
    h = sp.add_parser('hashes'); h.add_argument('--update', action='store_true')
    s = sp.add_parser('sync'); s.add_argument('--ref')
    sp.add_parser('role')
    a = ap.parse_args()
    if a.c == 'role':
        print(detect_role())
    elif a.c == 'hashes':
        update_hashes() if a.update else print(json.dumps({n: sha(SKILLS / n / 'SKILL.md') for n in load_manifest()['skills']}, indent=1))
    elif a.c == 'sync':
        print(json.dumps(sync(a.ref), indent=1))
    else:
        e, w, i = validate(a.role)
        if a.json:
            print(json.dumps(dict(errors=e, warnings=w, **i), indent=1))
        else:
            print(f"qazbot-operator-skills {i.get('package_version')} role={i.get('role')} active={i.get('active_for_role')}")
            for x in e:
                print('ERROR  ', x)
            for x in w:
                print('WARN   ', x)
            print('OK' if not e else 'FAIL')
        sys.exit(1 if e else 0)


if __name__ == '__main__':
    main()
