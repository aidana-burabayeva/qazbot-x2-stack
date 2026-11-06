# ws/tools — сборка и статус рабочего пространства, host-юниты, лабораторные и операторские инструменты (COLCON_IGNORE)

Перенесены как есть из `tools/` и офлайн-скриптов `ws/src/navigation/` (консолидация 2026-11-05). Многие скрипты
ссылаются на раскладку до консолидации (`ws/src/navigation`, `ws/src/lio`, `/home/agi/control_backend/...`,
`candidates/...`): они описывают/обслуживают **legacy production** и перед использованием с релизом должны быть
портированы. Новых путей `/home/agi` здесь не добавлять.

| Каталог | Что |
|---|---|
| `build.sh` | **единственная** сборка рабочего пространства на месте (`ws/build`, `ws/install`, third-party, runtime debs, манифест; README) |
| `status.sh` | read-only: HEAD/тег/dirty/манифест/production-гейт и откуда запущен каждый ROS-процесс (README) |
| `task_register.sh`, `task_cleanup.sh`, `cleanup_candidates.sh`, `test_task_cleanup.sh` | реестр временных путей задач и безопасная очистка (AGENTS.md DoD 6); `task_cleanup.sh` удаляет путь, только если он в `QAZBOT_CLEANUP_ROOTS`, не защищён (`QAZBOT_PROTECTED`: рабочее пространство робота, legacy, worktrees, aimdk, архив), не используется процессами и не содержит уникальной git-работы |
| `systemd/` | host user-юниты production (`qazbot-vendor-trim`), `install_user_units.sh` |
| `stack/` | legacy запуск/остановка/проверка стека (`stack_up.sh`, `check.sh`, `odom_owner_probe.py`, `detach.py`); заменён `qazbot_bringup x2_navigation.launch.xml` |
| `nav_analysis/` | офлайн-анализ bag/валидации (бывш. `ws/src/navigation/*.py`: analyze_validation, audit_control_frame, compare_*, inspect_*, matched_player, record_validation, replay_lio, walk_check, local_tf_relay, control_geometry + test) |
| `twin/` | инструменты цифрового двойника; `twin/mc_turn_controller` — плагин Nav2 только для SIM (собирается в twin-контейнере `docker/twin/build_turn_controller.sh`, в production не входит) |
| `rviz/` | RViz на ПК оператора |
| `operator_skills/`, `d04/`, `d05/`, `d07/`, `lio_calib/`, `roadmap_v13/` | инструменты отдельных расследований |
| `derive_nav2_map.py`, `record_motion.sh`, `stop_motion.sh`, ... | вспомогательные |
