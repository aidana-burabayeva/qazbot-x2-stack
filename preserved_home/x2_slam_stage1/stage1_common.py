"""Shared pure-python helpers for Stage 1 (no ROS imports; unit-testable).

Interfaces referenced (official):
- /integrated_command std_msgs/String, payloads 'start_mapping' / 'stop_mapping:<map_name>'
  https://x2-aimdk.agibot.com/en/latest/Interface/perception/SLAM.html
  local example: /home/agi/aimdk/src/py_examples/py_examples/slam.py
- /aimdk_5Fmsgs/srv/GetStoredMapByName (aimdk_msgs/srv/GetStoredMapByName)
  local example: /home/agi/aimdk/src/py_examples/py_examples/get_map.py
  local srv: /home/agi/aimdk/src/aimdk_msgs/prebuilt_aarch64/share/aimdk_msgs/srv/GetStoredMapByName.srv
"""
import json
import os

INTEGRATED_COMMAND_TOPIC = '/integrated_command'  # SLAM.html, slam.py
GET_STORED_MAP_BY_NAME_SRV = '/aimdk_5Fmsgs/srv/GetStoredMapByName'  # SLAM.html, get_map.py
START_PAYLOAD = 'start_mapping'  # slam.py
SAVE_TIMEOUT_S = 120.0  # SLAM.html: save timeout default 120 s

DEFAULT_JSON = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'output', 'stage1_map_check.json')


def build_stop_payload(map_name):
    """Payload format from slam.py: f'stop_mapping:{map_name}'."""
    return f'stop_mapping:{map_name}'


def validate_map_name(map_name):
    return isinstance(map_name, str) and map_name.strip() != ''


def build_map_check(map_name, resp):
    """Build the JSON record from a GetStoredMapByName response.

    Field names (GetStoredMapByName.srv): code, map_info{resolution,width,height},
    map_version, map_path, data[], map_id.
    A field missing from the response is written as null.
    """
    def g(obj, name):
        return getattr(obj, name, None) if obj is not None else None

    info = g(resp, 'map_info')
    data = g(resp, 'data')

    def num(v):
        return None if v is None else (float(v) if isinstance(v, float) else int(v))

    return {
        'map_name': map_name,
        'map_id': num(g(resp, 'map_id')),
        'map_path': g(resp, 'map_path'),
        'map_version': num(g(resp, 'map_version')),
        'resolution': None if g(info, 'resolution') is None else float(info.resolution),
        'width': num(g(info, 'width')),
        'height': num(g(info, 'height')),
        'grid_data_present': bool(data is not None and len(data) > 0),
        'rviz_visible': False,  # set later by the operator
    }


def write_map_check(path, record):
    """Write/merge record into JSON file. Existing rviz_visible is preserved."""
    merged = {}
    if os.path.exists(path):
        try:
            with open(path) as f:
                merged = json.load(f)
        except (OSError, ValueError):
            merged = {}
    keep_rviz = merged.get('rviz_visible') if merged.get('map_name') == record.get('map_name') else None
    merged.update(record)
    if keep_rviz is not None:
        merged['rviz_visible'] = keep_rviz
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w') as f:
        json.dump(merged, f, indent=2, ensure_ascii=False)
        f.write('\n')
    return merged


def wait_for_subscribers(get_count, timeout_s, sleep, now, poll_s=0.5):
    """Bounded wait until get_count() >= 1. Returns final count."""
    deadline = now() + timeout_s
    while True:
        n = get_count()
        if n >= 1 or now() >= deadline:
            return n
        sleep(poll_s)


def poll_saved_map(call, timeout_s, sleep, now, log, interval_s=2.0):
    """Poll call() -> response|None until response.code == 0 or timeout.

    Returns (ok, last_response). Success only when code == 0.
    """
    deadline = now() + timeout_s
    last = None
    while True:
        resp = call()
        if resp is not None:
            last = resp
            if resp.code == 0:
                return True, resp
            log(f'GetStoredMapByName code={resp.code} (not 0), retrying')
        else:
            log('GetStoredMapByName: no response / service unavailable, retrying')
        if now() >= deadline:
            return False, last
        sleep(interval_s)
