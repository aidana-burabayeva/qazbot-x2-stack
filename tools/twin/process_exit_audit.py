"""Audit native children as well as ros2-launch wrapper exit codes."""
import re
from pathlib import Path

DIED=re.compile(r'\[([^\]\n]+)\]:\s*process has died.*exit code\s+(-?\d+)')

def audit_process_exits(directory,wrapper_codes):
    failures=[]
    for log in sorted(Path(directory).glob('*.log')):
        for line in log.read_text(errors='replace').splitlines():
            match=DIED.search(line)
            if match and int(match[2])!=0:
                failures.append(dict(log=log.name,process=match[1],exit_code=int(match[2])))
    return dict(clean_shutdown=all(code==0 for code in wrapper_codes.values()) and not failures,
                native_process_failures=failures,wrapper_codes=wrapper_codes)
