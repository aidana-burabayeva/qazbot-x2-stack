# Документация control_backend

> **Консолидация 2026-11-05.** Исторические документы ниже ссылаются на раскладку до консолидации. Соответствие путей:
> `ws/src/navigation/*.launch.py` → `ws/src/qazbot_localization/launch/localization.launch.py`, `ws/src/qazbot_navigation/launch/nav_*.launch.py`;
> `ws/src/navigation/{localization_trust_node,loc_trust,pose_store,seed_pose,source_stamp,launch_stagger}.py` → `qazbot_localization/scripts/`;
> `goal_coordinator.py`, `goal_alignment_watch.py`, `bt/`, `rviz/` → `qazbot_navigation/`; `x2_motion_adapter.py` → `qazbot_motion_control/scripts/`;
> `env.sh`, `aimdk_env.sh`, `run_*.sh`, `dds_*.xml` → `qazbot_bringup/`; `ws/src/lio/x2.yaml` → `qazbot_localization/config/x2.yaml`;
> `ws/src/maps/*` → `qazbot_maps/maps/*`; `tools/*` → `ws/tools/*`; `tests/*` → `ws/tests/*` (юнит-тесты → `ws/src/<pkg>/test/`).
> Удалены: `ws/src/lio/{level_bridge,native_odom*,raw_scan_bridge}.py` (тег `pre-consolidation-fb18dc1`). Доказательства
> (`docs/evidence`, результаты) — `/home/agi/qazbot_archive/repo_tree_fb18dc1/`. Релизы и откат: [RELEASE.md](RELEASE.md).

# Актуальный статус

См. INTEGRATION_STATUS.md и ../README.md. Некоторые исторические разделы ниже
описывают предыдущий этап; они не являются актом приёмки движения.

# control_backend: навигация D1 и X2 на общей карте

Состояние на 2026-10-21. Здесь собрано всё, что установлено по навигации X2 (.66) на карте D1, с доказательствами:
что проверено, что нет, какие файлы менялись. **Источник истины только этот каталог на роботе (`~/control_backend`).** Код правится прямо здесь, документация
ведётся здесь же; локальных рабочих копий нет.

## Документы

| Файл | О чём |
|---|---|
| [digital_twin/README.md](digital_twin/README.md) | Цифровой двойник X2 (SIM/REPLAY на x86 без робота): карта проекта, запуск, результаты, официальные доки AimDK |
| [DECISIONS.md](DECISIONS.md) | Принятая архитектура и правила, которые не нарушаем |
| [X2_AIMDK_VERSIONS.md](X2_AIMDK_VERSIONS.md) | Фактические версии AimDK/прошивки на .66, почему `ros2 pkg prefix aimdk_msgs` не находит пакет, сверка определений |
| [X2_MOTION_ADAPTER.md](X2_MOTION_ADAPTER.md) | Разбор `cmd_to_aimavel.py` (Nova-Valley) против документации и живого графа, дизайн адаптера, результаты тестов |
| [X2_FIRST_PHYSICAL_TEST_PLAN.md](X2_FIRST_PHYSICAL_TEST_PLAN.md) | Поэтапный план безопасной проверки с оператором: F0 (локализация), F1 (динамический габарит), S0–S5, N1–N2, ступенчатая проба реальной цели |
| [X2_NAV2_CONTROLLER.md](X2_NAV2_CONTROLLER.md) | Полная цепочка NavigateToPose: архитектура, топики, TF, параметры (кандидаты), рекаверы, тесты, запуск/остановка, дефекты |
| [X2_LOCALIZATION_TRUST.md](X2_LOCALIZATION_TRUST.md) | Критерий доверия локализации, защёлка/возобновление, единственный владелец движения и его пределы, заметка про 3D NDT |
| [RVIZ_LOCAL.md](RVIZ_LOCAL.md) | RViz на ПК: 2D Pose Estimate, 2D Goal Pose, шаги оператора |
| [INTEGRATION_STATUS.md](INTEGRATION_STATUS.md) | Текущее состояние интеграции (раздел «Текущее состояние» вверху) и хронологический журнал |
| [X2_FOOTPRINT.md](X2_FOOTPRINT.md) | Габарит X2: вендорский footprint, URDF, суставы, запись при ходьбе |
| [X2_NAV_STACK_RUNBOOK.md](X2_NAV_STACK_RUNBOOK.md) | Как запускать и останавливать стек, причина проблемы со сканом, локализация, планирование |
| [SOURCES.md](SOURCES.md) | Внешние источники, что из них взято и что нет |

Более ранние материалы: `STATUS.md` (перенос карты и локализация 20.10), `COMMON_BACKEND_2026-10-20.md` (слои бэкенда).

## Раскладка

```
control_backend/                 (X2: ~/control_backend)
  docs/                          эти документы
  runtime/                       FAST-LIO, Nav2 (user-space overlay), bt_navigator; в git не хранится
  ws/src/navigation/             launch, env.sh, detach.py, адаптер, walk_check, aimdk_env.sh, launch_stagger.py
  ws/src/lio/                    level_bridge.py, x2.yaml
  ws/src/maps/                   first_map (новая карта D1, 5 файлов, без home.yaml), first_map_nav2_v2 (производная для Nav2, tools/derive_nav2_map.py)
  logs/                          журналы запусков
  candidates/                    непроверенные кандидаты (body_frame_v1), на робота не выкладываются
```

Прочее: `tools/rviz/` (RViz2 для ноутбука, см. runbook), `tests/` (тесты и записи измерений), `docs/evidence/`
(сохранённые измерения прежних этапов: диагностика локализации, экстринсики, URDF перцепции).

## Тесты

| Что | Команда |
|---|---|
| Логика адаптера, 43 теста (core 33, early_takeover 1, new_faults 3, dry_run_shadow 6), без ROS и без робота | `cd tests/motion_adapter && python3 -m unittest -v test_adapter_core test_early_takeover test_new_faults test_dry_run_shadow` |
| Достоверность локализации, 46 тестов (16 + 13 + 17) | `cd tests/loc_trust && python3 -m unittest -v test_loc_trust test_source_failures test_trust_gaps` |
| Критерий остановки, 7 тестов | `cd tests/nav_e2e && python3 -m unittest -v test_stop_criterion` |
| Контроллер на модели MC, 19 проверок (домен 87) | `bash tests/controller_dry/run_dry.sh` |
| Сквозной сухой прогон реальной цепочки (адаптер live=false, trust, Nav2, координатор, модель; домен 87; последний полный 160/160) | `bash tests/nav_e2e/run_model.sh NEW_UNIQUE_RESULT_NAME` (легаси 73: `--legacy NAME`) |
| Производная карта Nav2 из карты D1 | `python3 -I tools/derive_nav2_map.py ws/src/maps/first_map ws/src/maps/first_map_nav2_v2 "ИСТОЧНИК" --extra` |
| Темп запуска нод (не больше 2 в секунду), нужен ROS Humble и `QAZBOT_SMAC_PREFIX` (5 тестов) | `python3 -m unittest -v tests/launch_stagger/test_launch_stagger.py` |
| Регрессия зазоров Smac на ПК (на карте v2 базовой линии нет, не запускалась; широкий `pkill` не запускать на роботе с живым стеком) | `tests/planner_clearance/run_variant.sh true OUT.json` |
| Огибающая в покое внутри вендорского footprint | `X2_URDF_DIR=/agibot/software/aimdk/extra/mc-rl/x2_urdf/X2_URDF-v1.3.0 python3 -m unittest -v tests/footprint/test_envelope_rest.py` |
| Интерфейсы AimDK на роботе | `bash tests/aimdk/check_aimdk_interfaces.sh` (на X2) |
| Дымовой тест адаптера в `dry_run` на живом графе | `tests/motion_adapter/ros_smoke_dry_run.py` |

## Сводка состояния

| Область | Статус |
|---|---|
| Версии AimDK/MC подтверждены | да (прошивка v1.1.5, интерфейс v1.1.0.4), версия прошивки самой платы MC напрямую не читается |
| `aimdk_msgs` на .66 | работает через аддитивный `aimdk_env.sh`, пересборка не понадобилась |
| FAST-LIO, level_bridge, map_server, AMCL, planner | работают; сухой стек запущен на аппаратном DDS (автозапуска нет, запускается вручную, см. `X2_NAV2_CONTROLLER.md`, раздел 7) |
| Исправление зазоров Smac | сохранено и проверено (0 нарушений против 4 из 19) |
| Адаптер движения | проверен в `dry_run` (теневая машина состояний), в MC не регистрируется; единственный реальный ADD был этапом S1 |
| Габарит X2 | в покое измерен (0,34 м), при ходьбе записи ещё нет |
| Контроллер Nav2 на X2 | установлены в user-space overlay; полный контур на модели: 160/160; на аппаратном DDS запущен сухой стек без движения |
| Первое физическое испытание | не проводилось, план (F0, F1, ступенчатая проба) в `X2_FIRST_PHYSICAL_TEST_PLAN.md`; открытые дефекты: `X2_NAV2_CONTROLLER.md`, раздел 8 |

## Добавлено 2026-10-21 (вечер)
- `X2_LOCALIZATION_TRUST.md`: критерий достоверности локализации, владелец движения (тесты `tests/loc_trust`).
- `X2_NAV2_CONTROLLER.md`: Controller 1.1.18, сухой замкнутый тест (`tests/controller_dry`).
- `X2_FOOTPRINT.md`: поправка радиуса вокруг навигационной точки (многоугольник 0,387; описанный прямоугольник 0,454).
