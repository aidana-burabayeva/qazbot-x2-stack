# QazBot X2 Digital Twin — карта проекта

Всё живёт внутри `control_backend/` по существующим конвенциям (одна копия, без параллельных папок).

| Что | Где |
|---|---|
| Скрипты twin (реплей, зонды, анализ, Virtual MC, окружение SIM) | `tools/twin/` |
| Контейнер официального MC + MuJoCo (Dockerfile, запуск, обёртки), сквозная цепочка с реальным стеком | `tools/twin/docker/` (`run_official_chain.sh`) |
| Тесты twin (parity AMCL, live-цепочка) | `tests/twin/` |
| Результаты прогонов | `tests/integration_results/` (`twin_*`, `live_chain_*`, `real_amcl_*`, `replay_*`) |
| Логи шаговых прогонов sim | `logs/twin/` |
| Официальные архивы AgiBot, снимок PC1, калибровка | `vendor/` (см. `vendor/README.md`) |
| x86-сборка (aimdk_msgs, Smac) и данные контейнера | `runtime_x86/` |
| Исходники стека (общие с роботом, без форка) | `ws/src/`, `candidates/` |
| Bag с робота | `logs/`, `logs/bags/` |

Быстрый старт (хост Ubuntu 22.04 x86, ROS Humble):
1. `tools/twin/build_x86.sh` — aimdk_msgs + патченый Smac (один раз; нужна сеть для `git clone`).
2. `source tools/twin/env_sim.sh` — изолированное окружение (домен 87, localhost, `QAZBOT_SIM=1`).
3. Регрессия реального стека: `tools/twin/run_model_x86.sh NAME`; live-адаптер против Virtual MC: `tools/twin/run_live_chain.sh NAME`; с реальным AMCL: `QAZBOT_AMCL=real tools/twin/run_live_chain.sh NAME`.
4. Реплей AMCL на записях: `python3 tools/twin/replay_amcl.py logs/bags/<bag> NAME`.
5. Официальный MC + MuJoCo: `docker build -t qazbot-twin-sim:humble tools/twin/docker`, `tools/twin/docker/run_container.sh` (сети нет), далее `docs/digital_twin/official_sim_results.md`.

Документы: `official_mc_chain_20261026.md` (главный: реальный стек + официальный MC), `leg_vs_lio_20261026.md`, `architecture_audit.md`, `phase1_results.md`, `official_docs_digest.md`, `official_admonitions.md`, `official_sim_results.md`, `friday_data_review.md`, `20261023-twin-handoff.md`, `20261023-twin-mcparam-diff.md`, `sim_steps_20261023.md`.
Предел поворота: 0,5 рад/с (`QAZBOT_MAX_ANGULAR`), пока оператор не одобрит иное.

## Приближение к .66 (26.10.2026)

[Управление углом и параллельные испытания MC](mc_turn_control_20261026.md): два изолированных worker, финальная серия 12/12, четыре поворотные цели Nav2; ограничения AMCL и линейного подхода сохранены.

[Изменения, измерения и границы достоверности](robot66_fidelity_20261026.md): отдельный профиль наблюдений груди, вариант инерций по архивной URDF .66, сравнение с реальными body trajectories и поиск актуального MC. Физическое соответствие пока не подтверждено; новый вариант массы не выбран default.

## Продолжение 27.10.2026

[Ходьба MC: все прогоны, включая неудачные](mc_walk_control_20261026.md). [Go2: controller, smoother, teleop и robot_driver](go2_braking_audit_20261027.md). Перенесённый стенд работает из digital_twin/research; новые worker создаются без удаления прежних контейнеров.

[Разбор raicom2026-agibot-X2: полезные идеи и подтверждённые ограничения](raicom2026_repository_review_20261027.md).
