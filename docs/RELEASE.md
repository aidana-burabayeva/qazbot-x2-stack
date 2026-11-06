# Релизы, деплой, откат (control_backend)

Скрипты: `docker/deploy/`. Префикс: `--prefix DIR` или явно экспортированный `$QAZBOT_PROD_PREFIX` — ОБЯЗАТЕЛЕН для `build_release.sh`, `deploy.sh`, `rollback.sh` (без него отказ, умолчания нет); только `status.sh` (чтение) по умолчанию смотрит в `$HOME/prod`.

```
<prefix>/releases/<sha>/install/        colcon install: ws/src + fast_lio + nav2_smac_planner (патч), read-only
<prefix>/releases/<sha>/deps/           pinned runtime debs (ws/third_party/runtime_debs.lock), раскладка как legacy runtime/
<prefix>/releases/<sha>/manifest.yaml   sha, date_utc, profile, builder, arch, third_party pins + patch sha256,
                                        third_party_builds (sha256 бинарников), runtime_debs_lock_sha256, dpkg_ros_humble
<prefix>/current  -> releases/<sha>     то, что запускается
<prefix>/previous -> releases/<sha>     цель отката
```

| Шаг | Команда | Что делает |
|---|---|---|
| Сборка | `build_release.sh --prefix DIR --sha <sha> [--deb-cache DIR] [--jobs 2]` | свежий clone, checkout ровно `<sha>`; отказ, если исходный рабочий каталог грязный, если релиз уже есть, если сборка испачкала checkout; staging → атомарный `mv`; `chmod a-w` |
| Деплой | `deploy.sh --prefix DIR --sha <sha>` | только симлинки: `previous` ← старый `current`, `current` ← `<sha>` (атомарный rename). Процессы не трогает |
| Запуск | `source <prefix>/current/install/qazbot_bringup/lib/qazbot_bringup/env.sh; ros2 launch qazbot_bringup x2_navigation.launch.xml mode:=production` | оператор; env.sh сам берёт `<release>/deps` |
| Статус | `status.sh` | SHA в `current`/`previous` (manifest) и SHA по путям запущенных процессов (`release <sha>` / `legacy`) |
| Откат | `rollback.sh --prefix DIR` | меняет местами `current` и `previous`; повторный вызов возвращает обратно; затем перезапуск |

Ничего из этого не запускает, не останавливает и не сигналит процессы; перезапуск стека на роботе — только оператор.

## Legacy (состояние до консолидации)
Текущее production (2026-11-05) работает из `/home/agi/control_backend` (не релиз; == fb18dc1 + rviz d01f779, проектор
9417bed, FAST-LIO/Smac из `candidates/`, Nav2 из `runtime/`). Это цель отката «legacy-fb18dc1»: остановить стек релиза
и запустить прежние команды (`/home/agi/qazbot_archive/20261105_live_snapshot_fb18dc1/cmdlines/*.txt`, порядок:
localization.launch.py → localization_trust_node.py → nav_full.launch.py → x2_motion_adapter.py).
`status.sh` показывает такие процессы как `legacy`. Каталог не удалять до подтверждённой работы релиза.

Адаптер берёт lock `~/control_backend/run/x2_motion_owner.lock` (параметр `lock_path`, умолчание не менялось), так что
legacy и релиз не могут одновременно владеть движением.
