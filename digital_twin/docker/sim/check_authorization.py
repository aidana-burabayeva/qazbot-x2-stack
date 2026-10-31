"""No official MC process starts without a bounded host-issued worker capsule."""
import json
import os
from pathlib import Path
import time

p=Path(os.environ.get('NIGHTLY_AUTHORIZATION_FILE','/run/qazbot/worker.json'))
try:
    d=json.loads(p.read_text())
    if (d.get('mode') not in ('L3_QUALIFICATION','FULL_NIGHTLY') or d.get('expires',0)<=time.time() or
            d.get('l3_workers')!=1 or d.get('scope')!='stationary_official_mc' or d.get('mc')!='official' or
            d.get('map')!='d1_qazbot_20261029' or os.environ.get('TWIN_MC','official')!='official' or
            os.environ.get('TWIN_MAP')!='d1_qazbot_20261029' or not d.get('source_sha') or not d.get('campaign_sha')):
        raise ValueError('invalid or expired worker capsule')
except (OSError,ValueError,TypeError) as exc:
    raise SystemExit('official MC refused: '+str(exc))
