# Minimal read-only offline host follow-up

This procedure is prepared, **not executed here on another host**. It is limited
to already authorized local backup storage on the operator PC. No SSH, network
mount, robot directory, ROS CLI, daemon, installation, extraction or replay.
Do not weaken sandbox permissions. If a path is denied, retain the exact error
and leave that check BLOCKED.

All four private archives were readable in this attempt, so there is no current
archive-permission blocker to resolve. If access differs for a later verifier,
run these read-only checks on the authorized host with the existing manifests:

```bash
cd /home/adn/QazBot/x2_preservation_20261103
sha256sum -c SHA256SUMS
python3 - <<'PY'
import tarfile
from pathlib import PurePosixPath
for name in ('robot_code_runtime_private.tar.gz', 'historical_preservation.tar'):
    count = 0
    with tarfile.open(name, 'r|*') as archive:
        for member in archive:
            path = PurePosixPath(member.name)
            assert not path.is_absolute() and '..' not in path.parts, member.name
            if member.isfile():
                with archive.extractfile(member) as stream:
                    while stream.read(1024 * 1024):
                        pass
            count += 1
    print(name, 'readable members:', count)
PY
```

For the three D04 bags, identify two **existing** offline-copy roots first. Each
root must contain the three registered `d04_static_bag_*` directories and their
metadata.yaml files. Do not copy 16.44 GB of bags to this workspace. Set the
following environment variable to the actual authorized roots, separated by
`:`; the placeholders below are illustrative and are not storage locations
verified by this attempt:

```bash
export QAZBOT_D04_COPY_ROOTS=/AUTHORIZED_OFFLINE_COPY_A:/AUTHORIZED_OFFLINE_COPY_B
python3 - <<'PY'
import importlib.util
import json
from pathlib import Path
root = Path('/home/adn/QazBot/control_backend_p20_twin02/run/nightly/attempts/x2_p0_preservation_restore.1.847fd9')
spec = importlib.util.spec_from_file_location('restore', root / 'tools/verify_preservation_restore.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
for bag in module.bags(module.SOURCE):
    print(json.dumps(bag, indent=2))
PY
```

This reads existing BAGS.md, stats and hashes MCAP files, parses metadata.yaml,
and reports real paths, device/inode identities and errors. Two hardlinks or
symlinks to one file count as one copy. Distinct filesystems are recorded
separately; two files on one filesystem do not establish media redundancy.
Each MCAP must match the exact expected size/SHA256 and be named in readable
metadata before counting as a verified copy. No playback is required.

Return the raw output as evidence. If no second copy exists, keep the exact bag
BLOCKED; copying or publishing new large data requires a separate storage plan.
This follow-up does not resolve broken FAST-LIO/runtime symlinks, missing system
images, host SDK import, physical safety or the historical partial archive.
