#!/usr/bin/env python3
"""Verify source/map parity with the captured physical X2; no ROS or network."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--stack-dir', type=Path,
                        help='Verify an extracted ws/src tree, e.g. /opt/x2twin/stack')
    args = parser.parse_args()
    manifest = args.root / 'docs/evidence/x2_twin_20261030/robot_baseline_manifest.json'
    data = json.loads(manifest.read_text())
    failures = []
    for item in data['files']:
        relative = Path(item['path'])
        path = (args.stack_dir / relative.relative_to('ws/src')
                if args.stack_dir else args.root / relative)
        if not path.is_file():
            failures.append(f'MISSING {path}')
        elif hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256']:
            failures.append(f'DIFFERENT {path}')
    for failure in failures:
        print(failure)
    print(f"{len(data['files']) - len(failures)}/{len(data['files'])} files match captured X2")
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())
