---
name: nightly-status
description: Показать состояние лаборатории nightly (read-only). Только CLI tools/nightly/lab.py, агент не нужен.
disable-model-invocation: true
---

Роль LAB. Сначала `python3 tools/operator_skills/opskills.py validate --role lab`; при FAIL или роли ROBOT/UNKNOWN остановись и скажи оператору.

Запусти и покажи вывод целиком (SIM ONLY, робот не трогать):

```bash
python3 tools/nightly/lab.py status
```

Правила: не reset/clean/stash/force-push; не ssh и не команды на X2; при `STOPPED_PUBLISH_PENDING` или `STOP_UNVERIFIED` скажи об этом прямо.
Оператор может запускать то же без Claude: `python3 tools/nightly/lab.py status`.

