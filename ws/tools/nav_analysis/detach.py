#!/usr/bin/env python3
"""Run a command detached with a log file and a PID file; stop it with SIGINT.

A background job started by a non-interactive shell inherits SIGINT as
ignored, so ros2 launch cannot shut its nodes down and leaves them orphaned.
Usage: detach.py NAME CMD...   (log: logs/NAME.log, pid: logs/NAME.pid)
"""
import os
import signal
import subprocess
import sys

name, cmd = sys.argv[1], sys.argv[2:]
here = os.path.dirname(os.path.abspath(__file__))
# control_backend/ is the nearest ancestor holding runtime/ (parent of navigation/, or two levels above it)
backend = next((p for p in (os.path.dirname(here), os.path.dirname(os.path.dirname(os.path.dirname(here))))
                if os.path.isdir(os.path.join(p, 'runtime'))), os.path.dirname(here))
logs = os.path.join(backend, 'logs')
os.makedirs(logs, exist_ok=True)
with open(os.path.join(logs, name + '.log'), 'ab') as log:
    proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True,
                            preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL))
with open(os.path.join(logs, name + '.pid'), 'w') as f:
    f.write(str(proc.pid))
print(name, proc.pid)
