"""Read-only source accounting. Observed evidence is never an import receipt or HW baseline."""
import os
from pathlib import PurePosixPath
import subprocess

HARDWARE_REF = 'origin/robot/stable'
PRESERVATION_REF = os.environ.get('NIGHTLY_PRESERVATION_REF', 'origin/robot/preservation-20261103')


def hardware_ref(ref):
    """Allow the installed branch and legacy captured baselines, never candidates/preservation."""
    return ref == HARDWARE_REF or ref.startswith('origin/baseline/x2-robot-')


def production_path(path):
    """Historical backups/candidates and package tests are evidence, not runtime source."""
    parts = PurePosixPath(path).parts
    name = parts[-1]
    return (not set(parts) & {'test', 'tests', 'backups', 'candidates', '__pycache__'}
            and '.bak' not in name and '_candidate' not in name and '_rejected' not in name
            and not name.endswith('.evidence_copy'))


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
        excluded = [p for p in paths if not production_path(p)]
        matching, pending, intentional = [], [], []
        for path in paths:
            if not production_path(path):
                continue
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
                    matching=matching, pending=pending, intentional_metadata=intentional, excluded_nonproduction=excluded,
                    qualification='BASELINE_SYNC_ONLY; no SIM_VERIFIED or HW_VERIFIED implied')
    except ValueError as exc:
        return dict(ref=ref, sha=tip, status='BLOCKED', reason=str(exc))
