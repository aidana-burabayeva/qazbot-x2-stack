# X2 Digital Twin

Запускает **неизменённый** навигационный стек (Nav2, AMCL, Localization Trust, Goal Coordinator, RPP, Motion Adapter `live`)
против симулированного X2 — без физического робота. Один контейнер несёт **официальный** контроллер движения AgiBot (`mc`) и
MuJoCo, другой — навигационный стек; между ними внутренняя сеть без выхода наружу.

```
 ┌────────────────────── docker network `twin` (internal: no LAN, no internet, no published ports) ─────────────────────┐
 │  sim  (x2-twin-sim)                          stack  (x2-twin-stack)                                          │
 │  official mc v1.0.0  ⇄  MuJoCo lx2501_3_t2d5     hal_plant ──► /qazbot_lio/{scan,odom_level}, /qazbot_d1_map/tf      │
 │  /aima/hal/odom/state ───────────────────────────►  AMCL · Trust · Coordinator · Nav2 · x2_motion_adapter(live)      │
 │  /aima/mc/locomotion/velocity ◄───────────────────────────────────────────────────────────────────────────────────── │
 └──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
          scenario (profile test): operator session client → results/<run>/results.json
```
Топики плант-узла совпадают с боевым `level_bridge` (имена, частоты 10/5 Гц, QoS), поэтому стек запускается **без ремапов и правок**.

## Быстрый старт
```bash
cd digital_twin
make doctor                 # хост: docker/compose/buildx, vendor, диск
make vendor                 # один раз: распаковать архивы AgiBot (`make vendor ZIPS=<каталог с zip>`, проверка sha256)
cp .env.example .env
make build                  # образы sim (1,8 ГБ) + stack (1,76 ГБ); первая сборка ~10 мин, правка кода ~40 с
make test                   # юнит-тесты внутри образа
make up                     # sim (стойка) + stack, ждёт health      (~1 мин)
make scenario               # сессия оператора → results/<RUN_NAME>/results.json
make down                   # остановить; make clean — ещё и тома
make lite-up lite-scenario  # лёгкий режим без MuJoCo (виртуальный MC + кинематика), ~35 с
```

## Структура
```
digital_twin/
├── Makefile  .env.example  .gitignore  .flake8
├── docker/
│   ├── Dockerfile            стек: rosdep-manifests → deps → build → test → stack (мультистадийный, digest-pin, non-root)
│   ├── sim.Dockerfile        рантайм официального mc + MuJoCo (бинарники монтируются, в образ не кладутся)
│   ├── compose.yaml          sim-init · sim · stack · scenario (test) · stack-lite/scenario-lite (lite)
│   ├── entrypoints/          ros_entrypoint.sh · stack_health.sh
│   ├── sim/                  sim_supervisor.sh (запуск mc → sim → SD → LD) · twin-sim-health
│   └── dds/udp_only.xml      Fast DDS: только UDP (SHM не пересекает контейнеры)
├── ws/src/                   colcon-workspace, пакеты ament_python
│   ├── x2_twin_sim/        plant: hal_plant (официальный MuJoCo), kinematic_plant (lite), virtual_mc (lite); geometry, occupancy
│   ├── x2_twin_bringup/    launch (twin.launch.py, amcl.launch.py) + config/twin.yaml + тесты паритета с боевым AMCL
│   └── x2_twin_scenarios/  live_chain (сценарий-клиент) + analysis: leg_vs_lio, replay_amcl, summarize
├── scripts/                  doctor.sh · fetch_vendor.sh
├── research/                 исследовательский контур (стенды, измерения, результаты Codex): см. research/README.md
├── vendor/                   внешние артефакты AgiBot (не в git; см. vendor/README.md)
├── results/                  результаты прогонов (не в git)
└── docs/                     архитектура, эксплуатация, достоверность модели
```
Код навигационного стека (`control_backend/ws/src/{navigation,lio,maps}`) **не копируется и не форкается**: образ берёт его из `control_backend/ws/src`
(без `*.bak*`), для разработки монтируется поверх `/opt/x2twin/stack`.

## Принципы
- **Один исходный код стека**, различие REAL/SIM только в источнике сенсоров и MC. Twin публикует боевые имена топиков.
- **Изоляция**: сеть `internal`, домен ROS 87, маркер `TWIN_SANDBOX=1` (ноды плант-пакета отказываются стартовать иначе), non-root,
  `cap_drop: ALL`, `no-new-privileges`. Адаптер регистрируется в MC только явным вызовом `~/register`.
- **Воспроизводимость**: базовый образ закреплён по digest; navigation2 — по коммиту; артефакты AgiBot — по sha256; зависимости
  ROS — через `rosdep` из `package.xml`.
- **Лицензия**: бинарники AgiBot не входят в образы и не публикуются; образы с вендорскими исходниками (aimdk_msgs) — только в приватный реестр.
- **Честные границы** (см. `docs/model_fidelity.md`): sim ≠ робот.

## Режимы
| | `official` (по умолчанию) | `lite` |
|---|---|---|
| MC | официальный mc v1.0.0 + MuJoCo | виртуальный MC (регистрация/состояние) + кинематика |
| Скорость | старт ~1,5 мин | ~35 с |
| Для чего | поведение походки (минимальная скорость, срыв разворота), сравнение вариантов | быстрая регрессия логики стека |
| Достоверность | качественно воспроизводит Блокеры 1–2 | идеализированная модель, не калибрована |

## Параметры (`.env`)
`MAX_ANGULAR` (0.5 — предел поворота, меняется только решением оператора), `SIM_CPUS`/`STACK_CPUS`, `TWIN_GUI=1` (окно MuJoCo на
хосте: `xhost +local:`), `TWIN_SIM_RES`, `LP_NUM_THREADS`, `TWIN_WORLD` (`map` — физические стены из карты, `flat` — пустой пол),
`TWIN_MAP`/`TWIN_START` (карта и стартовая поза, общие для sim и стека).

## Диагностика
`make ps` · `make logs` · `docker compose -f docker/compose.yaml logs sim` · логи mc/sim — том `sim_logs` (`/var/log/twin`).
