---
name: nightly-stop
description: Остановить и опубликовать лабораторию nightly (FREEZE→…→VERIFY), подготовить передачу физическому X2. Только CLI tools/nightly/lab.py, агент не нужен.
---

Запусти и покажи вывод целиком (SIM ONLY, робот не трогать):

```bash
python3 tools/nightly/lab.py stop
```

Правила: не reset/clean/stash/force-push; не ssh и не команды на X2; при `STOPPED_PUBLISH_PENDING` или `STOP_UNVERIFIED` скажи об этом прямо.
Оператор может запускать то же без Claude: `python3 tools/nightly/lab.py stop`.
Операторский STOP главнее /goal, /loop и Stop hook: после него не возобновляй работу без /nightly-resume. Session-only /loop cron и /goal открытых сессий отсюда не снять — сообщи, что оператору нужно сделать /goal clear и CronDelete.
