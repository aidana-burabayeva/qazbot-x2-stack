#!/usr/bin/env python3
"""Missing lifecycle server at startup: isolated standalone manager, no hardware."""
import json, os, signal, subprocess, sys, time, re
from pathlib import Path
if os.environ.get('ROS_DOMAIN_ID') != '87' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise RuntimeError('requires isolated domain 87')
root=Path.home()/'control_backend'
out=root/'tests/integration_results'/sys.argv[1]
out.mkdir()
log=out/'manager.log'
with log.open('w') as stream:
    proc=subprocess.Popen(
        ['ros2','run','nav2_lifecycle_manager','lifecycle_manager','--ros-args',
         '-r','__ns:=/qazbot_missing_test','-p','node_names:=[missing_controller]',
         '-p','autostart:=true'],
        stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        deadline=time.monotonic()+15
        while 'Configuring missing_controller' not in log.read_text():
            if proc.poll() is not None:raise RuntimeError('manager exited before test')
            if time.monotonic()>deadline:raise RuntimeError('observation deadline: not in discovery')
            time.sleep(.05)
        start=time.monotonic()
        os.killpg(proc.pid,signal.SIGINT)
        code=proc.wait(timeout=12)
        result=dict(exit_code=code,shutdown_observed_s=time.monotonic()-start,
                    fatal_signal=bool(re.search(r'SIGABRT|SIGSEGV|terminate called',log.read_text())))
        (out/'result.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result))
        if code!=0 or result['fatal_signal']:raise RuntimeError('unclean missing-server shutdown')
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            proc.wait(timeout=12)
