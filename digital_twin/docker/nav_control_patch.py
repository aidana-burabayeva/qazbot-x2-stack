"""The two exact SIM-only changes to the copied robot launch file."""
from pathlib import Path

TURN_GUARD = ("    if experimental_turn:\n        if any(os.environ.get(k) != v for k,v in "
              "{'QAZBOT_SIM':'1','ROS_DOMAIN_ID':'87','ROS_LOCALHOST_ONLY':'1'}.items()):")
INF_LITERAL = "'obstacle_layer.scan.inf_is_valid': False,"


def transform(source):
    if source.count(TURN_GUARD) != 1 or source.count(INF_LITERAL) != 1:
        raise ValueError('expected exactly one experimental-turn guard and one inf_is_valid literal')
    source = source.replace(TURN_GUARD, TURN_GUARD.replace("'ROS_LOCALHOST_ONLY':'1'", "'TWIN_SANDBOX':'1'"))
    return source.replace(INF_LITERAL, "'obstacle_layer.scan.inf_is_valid': os.environ.get('QAZBOT_I1_INF_VALID') == '1',")


if __name__ == '__main__':
    import sys
    path = Path(sys.argv[1])
    path.write_text(transform(path.read_text()))
