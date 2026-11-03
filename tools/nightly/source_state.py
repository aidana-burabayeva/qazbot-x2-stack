"""Read-only source accounting. Observed evidence is never an import receipt or HW baseline."""
import os
import subprocess

HARDWARE_REF = 'origin/robot/stable'
PRESERVATION_REF = os.environ.get('NIGHTLY_PRESERVATION_REF', 'origin/robot/preservation-20261103')


def hardware_ref(ref):
    """Allow the installed branch and legacy captured baselines, never candidates/preservation."""
    return ref == HARDWARE_REF or ref.startswith('origin/baseline/x2-robot-')


def preservation_status(repo, integration, ref=PRESERVATION_REF):
    def git(*args):
        p = subprocess.run(['git', '-C', str(repo), *args], capture_output=True, text=True)
        if p.returncode:
            raise ValueError(p.stderr.strip())
        return p.stdout.strip()

    try:
        tip = git('rev-parse', '--verify', ref + '^{commit}')
    except ValueError:
        return dict(ref=ref, sha=None, status='UNAVAILABLE')
    try:
        installed = git('rev-parse', '--verify', HARDWARE_REF + '^{commit}')
        # Inspect production additions/changes relative to hardware, even if its SHA was already processed.
        paths = git('diff', '--name-only', '--no-renames', '--diff-filter=AM', installed, tip, '--', 'ws/src/').splitlines()
        matching, pending, intentional = [], [], []
        for path in paths:
            source = git('rev-parse', f'{tip}:{path}')
            try:
                imported = git('rev-parse', '--verify', f'{integration}:{path}')
            except ValueError:
                imported = None
            if source != imported and path == 'ws/src/maps/d1_qazbot_20261029/manifest.json':
                folder = path.rsplit('/', 1)[0]
                # A provenance descriptor may differ only when the actual map data match.
                if all(git('rev-parse', '--verify', f'{tip}:{folder}/{f}') ==
                       git('rev-parse', '--verify', f'{integration}:{folder}/{f}')
                       for f in ('map.pgm', 'map.yaml')):
                    intentional.append(path)
                    continue
            (matching if source == imported else pending).append(path)
        return dict(ref=ref, sha=tip, hardware_sha=installed,
                    status='IMPORT_PENDING' if pending else 'SOURCE_MATCH',
                    matching=matching, pending=pending, intentional_metadata=intentional,
                    qualification='BASELINE_SYNC_ONLY; no SIM_VERIFIED or HW_VERIFIED implied')
    except ValueError as exc:
        return dict(ref=ref, sha=tip, status='BLOCKED', reason=str(exc))
