"""Protocol-level model of the MC input-source registry (AimDK 'MC Control Signal Configuration'). No ROS imports.

Implemented: ADD / ENABLE / DISABLE / DELETE / MODIFY with the documented required fields; system-provided names are refused;
commands from unregistered or disabled sources are dropped. NOT implemented (the official mc does these): priority arbitration,
the 1000 ms source timeout counted from the last non-zero command, the 200 ms command zeroing, return to stand after 2000 ms.
"""
from dataclasses import dataclass
from typing import Dict

ADD, MODIFY, DELETE, ENABLE, DISABLE = 1001, 1002, 1003, 2001, 2002
SYSTEM_SOURCES = frozenset({'rc', 'vr', 'app_proxy', 'interaction', 'pnc', 'linkswarm', 'remote_teleop_pc'})
OK, REFUSED = 0, 1


@dataclass
class Source:
    name: str
    priority: int
    timeout_ms: int
    enabled: bool = True


class InputSourceRegistry:
    def __init__(self) -> None:
        self.sources: Dict[str, Source] = {}
        self.counters = {'registrations': 0, 'refused': 0, 'accepted': 0, 'rejected': 0}

    def apply(self, action: int, name: str, priority: int = 0, timeout_ms: int = 0) -> int:
        code = self._apply(action, name, priority, timeout_ms)
        self.counters['registrations' if code == OK else 'refused'] += 1
        return code

    def _apply(self, action: int, name: str, priority: int, timeout_ms: int) -> int:
        if not name or name in SYSTEM_SOURCES:
            return REFUSED
        if action == ADD:
            if name in self.sources or not 0 <= priority <= 100 or timeout_ms <= 0:
                return REFUSED
            self.sources[name] = Source(name, priority, timeout_ms)
            return OK
        src = self.sources.get(name)
        if src is None:
            return REFUSED
        if action == ENABLE:
            src.enabled = True
        elif action == DISABLE:
            src.enabled = False
        elif action == DELETE:
            del self.sources[name]
        elif action == MODIFY:
            if not 0 <= priority <= 100 or timeout_ms <= 0:
                return REFUSED
            src.priority, src.timeout_ms = priority, timeout_ms
        else:
            return REFUSED
        return OK

    def accepts(self, name: str) -> bool:
        """True for a registered, enabled source; counts the decision."""
        src = self.sources.get(name)
        ok = bool(src and src.enabled)
        self.counters['accepted' if ok else 'rejected'] += 1
        return ok
