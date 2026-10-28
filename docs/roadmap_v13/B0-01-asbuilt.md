# B0-01 — as-built freeze, без ROS/DDS и без рестартов
Map: D1 first_map_nav2_v3. Host: agi / aarch64, прежняя .66, текущая .72.
Цель: перед дальнейшими правками отделить повреждения диска от работающего runtime.
Verdict: PARTIAL; restart/deployment gate ещё не закрыт.

## Сохранено
evidence/B0_20261027_1030Z/:
- disk_manifest.json: 5583 файлов с size/mode/mtime/inode/hash и ELF bounds;
- process_manifest.json: 12 процессов, argv, cwd, affinity, parameter files, selected environment;
- mapped_files.json: 704 mapped paths и сравнение backing inode с диском;
- asbuilt.tar.gz: 97348710 bytes; ws/src, candidates, tests, tools/stack/diagnostics, без dereference symlinks;
- environment.json, snapshot_sha256.json; damage_summary.json — классификация.
Идентификатор каталога служит ID, фактический UTC момент записан в environment.json.
Архив сохраняет повреждённые bytes как есть, это forensic snapshot, не готовый release.

## До/после
Runtime не менялся. FAST-LIO PID86463 affinity [5,6]; localisation launch86415.
level_bridge86488, AMCL86545, Trust86641, dry-run adapter86794, Nav292489, coordinator92646.
По 31 mapped файлу проекта disk inode соответствует mapped inode; loaded Python bytecode этим НЕ установлен.

## Повреждения
270 нулевых code/config файлов (кроме __init__.py) в candidates; в ws/src и tests нулевых code/config по этой проверке нет.
14 ELF имеют таблицы за EOF: 11 тестовых executables nav2_smac_bounds/build и 3 библиотеки в явно отложенном install_corrupt_restore_20261027.
Это не равно 14 повреждённым активным библиотекам: файлы карантинной копии не принимать за runtime.
Критичные loaded libraries и полную пригодность исходников ещё требуется сопоставить с эталонными hashes.
Source attribution массового copy не установлен: одинаковые mtime/262144-byte truncation — свидетельства повреждения, не имя виновного процесса.

## Погрешности снимка
5 read errors: один чужой /proc PID с Permission denied, четыре краткоживущих процесса исчезли.
vendor/MC процессы не attach/debugged; их protected state не изменялось.
Symlinks сохраняются как ссылки, внешние targets не выдаются за архивированные bytes.

## Rollback
Отсутствует: runtime правок нет. Перед любым восстановлением — hash precondition и резервная копия конкретного файла.
Никакого автоматического unpack архива поверх живого дерева.

## Next ONE step
Прогнать существующие native_imu/Trust/adapter tests на frozen tree в изолированном окружении, установить точный release subset и список недостающих эталонов.

