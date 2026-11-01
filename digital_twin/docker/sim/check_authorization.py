"""No official MC process starts without a bounded host-issued worker capsule.

Two disjoint capsule kinds exist:
  * L3_QUALIFICATION / FULL_NIGHTLY  - issued by the Nightly queue (tools/nightly/campaign.py), scope stationary_official_mc;
  * OPERATOR_SIM                     - issued by tools/twin/operator_sim.py after the operator's explicit confirmation,
    scope interactive_operator_mc. It never qualifies anything (qualifies=false), is capped to 8 h and is a different
    scope, so it cannot be used to run a Nightly campaign, and a Nightly capsule cannot be used for an interactive session.
Both take the same official-mc.lock (sim_supervisor.sh), so the two kinds never run an MC at the same time.
"""
import json
import os
from pathlib import Path
import time

MAP = 'd1_qazbot_20261029'
OPERATOR_MAX_S = 8 * 3600


def validate(d, env, now=None):
    now = time.time() if now is None else now
    if not isinstance(d, dict):
        raise ValueError('invalid or expired worker capsule')
    common = (d.get('expires', 0) > now and d.get('l3_workers') == 1 and d.get('mc') == 'official' and d.get('map') == MAP
              and env.get('TWIN_MC', 'official') == 'official' and env.get('TWIN_MAP') == MAP and bool(d.get('source_sha')))
    mode = d.get('mode')
    if mode in ('L3_QUALIFICATION', 'FULL_NIGHTLY'):
        ok = common and d.get('scope') == 'stationary_official_mc' and bool(d.get('campaign_sha'))
    elif mode == 'OPERATOR_SIM':
        ok = (common and d.get('scope') == 'interactive_operator_mc' and d.get('qualifies') is False
              and d.get('issued_by') == 'operator_sim' and bool(d.get('session_id'))
              and isinstance(d.get('issued'), (int, float)) and 0 < d['expires'] - d['issued'] <= OPERATOR_MAX_S)
    else:
        ok = False
    if not ok:
        raise ValueError('invalid or expired worker capsule')
    return mode


def main():
    p = Path(os.environ.get('NIGHTLY_AUTHORIZATION_FILE', '/run/qazbot/worker.json'))
    try:
        validate(json.loads(p.read_text()), os.environ)
    except (OSError, ValueError, TypeError) as exc:
        raise SystemExit('official MC refused: ' + str(exc))


if __name__ == '__main__':
    main()
