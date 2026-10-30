# Контракт развёртывания: источник -> сборка -> проверенное развёртывание

Назначение: чтобы состояние X2 всегда сводилось к записанному Git SHA и проверяемым SHA256. Статус: **предложение**; контракт ещё не применялся. Фактическая структура: `REPOSITORY_LAYOUT.md`.

## 1. Правила

1. `git fetch`/`git pull`/смена ветки в checkout **никогда не меняют робота**. Checkout и развёртывание независимы.
2. Развёртывание только явным шагом, одобренным оператором, копированием файлов из **чистого** checkout (`git status --porcelain` пуст) на **записанном SHA**.
3. В развёртывании не делается `git init` и не заменяется `runtime/` клоном.
4. Runtime-файлы (`run/*.lock`, `logs/`, bags, `__pycache__`, `*.bak*`) в Git не коммитятся и в манифест кода не входят; `run/x2_motion_owner.lock` в Git сейчас отслеживается ошибочно (устарел) и должен быть убран из индекса отдельным коммитом.
5. Каждое развёртывание сопровождается манифестом. Нет манифеста: нет права утверждать, что развёрнуто.

## 2. Что фиксирует запись о развёртывании

- Исходный Git SHA (полный) и ветка; подтверждение чистого дерева.
- Версии зависимостей: пакеты ROS 2 Humble (`dpkg -l ros-humble-*`, ключевые), SHA256 архивов/префиксов `runtime/` и `candidates/*/install`, версия AIMDK/SDK (`docs/X2_AIMDK_VERSIONS.md`), версия Nav2.
- Изменённые пакеты и конфиги (список путей и SHA256 до и после).
- SHA256 **каждого** установленного файла.
- Пройденные регрессионные ворота: unit-тесты; E2E-прогон на модели; пороги T1/T5 без изменений (T5 0.10 м / 0.10 рад; Trust `jump_xy_max`/`jump_yaw_max` как заданы в скриптах запуска); collision detection, реальный footprint и freshness без ослабления.
- Время (UTC и локальное), оператор, путь каталога резервной копии.

## 3. Формат манифеста (JSON, пример схемы)

```json
{
  "schema": 1,
  "deployed_at": "2026-10-31T09:00:00Z",
  "operator": "имя оператора",
  "source": {
    "repo": "github.com/aidana-burabayeva/qazbot-x2-stack",
    "branch": "nav/first-goals-20261028",
    "sha": "982f61802035ab5b033e26ce9df881dab2e27468",
    "tree_clean": true
  },
  "dependencies": {
    "ros_distro": "humble",
    "ros_packages": {"ros-humble-rclpy": "x.y.z"},
    "runtime_overlays": {"runtime/qazbot_nav_deps": "sha256:...", "runtime/nav2_bt_deps": "sha256:...", "runtime/user_pkgs": "sha256:...", "candidates/fast_lio_portable": "sha256:..."},
    "aimdk_sdk": "версия из docs/X2_AIMDK_VERSIONS.md",
    "nav2": "версия"
  },
  "changed": [
    {"path": "ws/src/navigation/x2_motion_adapter.py", "before": "sha256:71db8ebd...", "after": "sha256:9c05bb34..."}
  ],
  "files": [
    {"path": "ws/src/navigation/localization.launch.py", "sha256": "..."}
  ],
  "gates": {
    "unit_tests": {"passed": true, "ref": "путь/результат"},
    "e2e_model_run": {"passed": true, "ref": "..."},
    "t1_t5_thresholds_unchanged": true,
    "collision_footprint_freshness_unchanged": true
  },
  "backup_dir": "/home/agi/control_backend/backups/deploy_<UTC>",
  "previous_manifest": "deploy_manifests/<предыдущий>.json"
}
```

Манифесты хранятся под корнем развёртывания (например `deploy_manifests/`), а копия публикуется в Git под `docs/evidence/` с SHA256 (правило AGENTS.md: секреты и сырой runtime в Git не загружать).

## 4. Процедура развёртывания

1. Оператор одобряет шаг; подтверждено, что стек не запущен либо остановлены только свои процессы.
2. Checkout на записанном SHA, дерево чистое; прогнаны unit-тесты и E2E на модели.
3. Резервная копия заменяемых файлов в `backups/deploy_<UTC>/` с SHA256.
4. Копирование файлов; пересчёт SHA256 установленных файлов и сверка с манифестом.
5. Запись манифеста; ссылка на предыдущий.

## 5. Откат

1. Взять предыдущий манифест и каталог резервной копии.
2. Остановить свои процессы (по одобрению оператора).
3. Восстановить файлы из `backup_dir`.
4. Пересчитать SHA256 восстановленных файлов и сверить с предыдущим манифестом; расхождение означает, что откат не завершён.
5. Записать откат отдельным манифестом.

## 6. Известные пробелы (на 2026-10-30)

- **Манифеста всего дерева нет.** Есть только манифесты отдельных сессий (`docs/evidence/nav_session_2026102{8,9}/manifest.json`, карты `ws/src/maps/first_map_nav2_v*/manifest.json`); они фиксируют файлы сессии, а не развёртывание.
- Версии зависимостей `runtime/` и `candidates/` нигде не записаны хэшами; префиксы не сравнивались с чем-либо.
- Висячие симлинки `runtime/fast_lio` (79) и `fast_lio` в `AMENT_PREFIX_PATH`: нужно удалить из overlay либо задокументировать.
- DDS-профиль и скрипты запуска живут в `docs/evidence/nav_20261029_1425/`, вне `ws/src`; в манифест нужно включать явно.
- Пороги T5 заданы аргументами скриптов запуска, а не в `ws/src`.
- `env.sh` с относительным `runtime/` не работает из checkout (см. `MIGRATION_PLAN.md`).
