# control_backend: правила для агентов и Definition of Done

## Порядок источников (абсолютное правило)
1. Безопасность робота. 2. Явное указание оператора. 3. **Официальная документация AgiBot** — сначала:
   https://x2-aimdk.agibot.com (версия = SDK робота), локально `~/aimdk/docs/cn/_build/html`, `~/aimdk/README.en.md`,
   `~/aimdk/topics_and_services/`, `~/aimdk/src/*/README.md`. Рекомендованные интерфейсы, топики, сервисы, QoS, режимы
   управления; не изобретать альтернатив. В отчёте указывать раздел документации или явно писать «документация молчит».
4. Henki ROS 2 best practices (`~/.claude/standards/henki_ros2_best_practices.md`): пакеты с `package.xml`, параметры в
   `config/*.yaml`, launch в `launch/` (XML предпочтителен), README пакета (обзор, запуск, API, параметры), тесты.
   Легаси-код — в существующем стиле; стандарт применяется к новому и рефакторингу.
5. **Высокочастотный код (≳ 50 Гц, облака, изображения, контуры управления) — только C++**, composable nodes +
   intra-process для больших данных. Python — инструменты, оркестрация, тесты, низкая частота.
   Кандидаты на перенос в C++: `qazbot_motion_control/x2_motion_adapter.py` (50 Гц MC, HIGH),
   `qazbot_navigation/goal_coordinator.py` (20 Гц шлюз cmd_vel, MEDIUM).

## Физический робот
- Один владелец робота. Каждую сессию начинать с read-only проверки процессов, TF, топиков; старые PID не доказательство.
- Не запускать дубликаты FAST-LIO/AMCL/trust/проектора. Не останавливать чужие процессы.
- Без явного разрешения оператора: не менять production, не запускать адаптер `live:=true`, не регистрировать источник
  MC, не публиковать в `/aima/*`, не отправлять goals/cmd_vel. Движение — только при операторе и рабочем RC/останове.
- LAB на роботе: `ROS_DOMAIN_ID=88`, `ROS_LOCALHOST_ONLY=1` (или контейнер `network_mode: none`), `nice -n 19`,
  `--parallel-workers 2`; никогда домен 0. `ros2 launch qazbot_bringup x2_navigation.launch.xml` в режиме lab сам это проверяет.
- AimDK «二次开发边界与声明»: не более 2 новых узлов/с при стойке/ходьбе (launch_stagger 0.6 с), не трогать PC1 и
  вендорские DDS/системные настройки.
- Production — только релизы `<prefix>/releases/<sha>` через `docker/deploy/` (README, docs/RELEASE.md).

## Что никогда не попадает в репозиторий
Каталоги candidates/, scratch/, backups/, preserved_*, результаты прогонов, bags (`*.bag`, `*.db3`, `*.mcap`), логи,
`*.bak*`, сборки (`build/ install/ log/`), бинарники сторонних пакетов. Доказательства — в `/home/agi/qazbot_archive/<sha>/`
с SHA256 и INDEX.md; в `docs/` — только лёгкие сводки `.md`. (Прежнее правило «bags в docs/evidence» отменено решением
консолидации P1S-7, 2026-11-05.) Сторонний код — только `ws/third_party.repos` (точный коммит) + `ws/third_party/patches`.
Дубликаты реализаций запрещены: одна реализация на функцию; эксперименты — в ветке, не копией файла.

## Definition of Done (каждая задача)
1. Документация AgiBot проверена и процитирована (или «молчит»); Henki и правило C++ соблюдены, отклонения записаны.
2. Сборка `colcon build` вне дерева и `colcon test` (домен ≠ 0) зелёные для затронутых пакетов; launch `--show-args` работает.
3. Имена узлов, namespaces, топики, frame ids и значения параметров production не изменены без явного решения
   (проверка эквивалентности параметров, если затронуты launch/config).
4. Независимая верификация (другой агент/человек) до пометки «готово».
5. Коммиты логичными шагами, push в отдельную ветку; без force-push и переписывания истории; ветки не удалять.
6. **Чистое рабочее дерево**: `git status` пуст; каждый временный путь зарегистрирован
   (`docker/tools/task_register.sh <task> <path>`) и удалён (`docker/tools/task_cleanup.sh <task>`); нерегистрированный
   мусор = задача не выполнена. Проверка: `docker/tools/cleanup_candidates.sh`.
7. Отчёт: что сделано, доказательства, отклонения, риски; HANDOFF/TEST_STATUS обновлены при смене состояния.

## Где что
README.md (структура, сборка, запуск, SHA на роботе, LAB→PRODUCTION, откат), README пакетов в `ws/src/*/`,
`docker/README.md` (хост vs контейнер, изоляция LAB), `ws/third_party/README.md` (происхождение сторонних бинарников),
`docs/HANDOFF.md`, `docs/TEST_STATUS.md` (исторические пути до консолидации: таблица в `docs/README.md`).
