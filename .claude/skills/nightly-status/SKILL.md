---
name: nightly-status
description: Показать состояние лаборатории nightly (read-only). Только CLI tools/nightly/lab.py, агент не нужен.
---

Запусти и покажи вывод целиком (SIM ONLY, робот не трогать):

```bash
python3 tools/nightly/lab.py status
```

Правила: не reset/clean/stash/force-push; не ssh и не команды на X2; при `STOPPED_PUBLISH_PENDING` или `STOP_UNVERIFIED` скажи об этом прямо.
Оператор может запускать то же без Claude: `python3 tools/nightly/lab.py status`.

