# X2 .66: версии AimDK, прошивки и интерфейсов (Задачи A и B)

Проверено 2026-10-21 чтением файлов и вызовами только чтения. Ничего не устанавливалось и не заменялось.

## A. Фактические версии

| Что | Значение | Источник на роботе |
|---|---|---|
| Платформа / прошивка | **v1.1.5** | `~/.aima/env/bashrc` (`AGIBOT_SOFTWARE_VERSION`), `/agibot/software/metadata.yaml` (`version`) |
| Изделие | X2 Ultra, `lx2501_3_t2d5`, hardware `t2d5` | `bashrc`, `metadata.yaml` (`product`) |
| Среда исполнения aimrte | v0.9.2.14 | `metadata.yaml` (`platform_version.content`) |
| **Версия интерфейса `aimdk_msgs`** | **v1.1.0.4** | `/agibot/software/common/version.json`, `metadata.yaml` |
| Пакет AimDK в прошивке | v1.1.0-rc10 | `metadata.yaml` (soc1, пакет `aimdk`) |
| Дерево исходников SDK | `v0.0.0-g14fe2e8` (коммит g14fe2e8, без тега) | `/agibot/software/aimdk/version` и `~/aimdk/version` совпадают |
| Старый SDK | `v1.0.0-gf8a92e8` | `~/aimdk.bak/version`, не используется |
| Локальная документация | «AimDK_X2 1.1.0» | `~/aimdk/docs/cn/_build/html` |
| Навигация вендора (пакет `nav`) | X2_ULTRA-v1.1-r10-hotfix-5, сборка 2609031015 | `metadata.yaml` |
| Версия MC | отдельно прочитать нельзя | см. ниже |

Предположение «1.1.5» подтверждено: это версия **платформы/прошивки**. Версия **интерфейса** v1.1.0.4, и она
соответствует локальной документации 1.1.0. Документацию 1.1.0 нельзя автоматически приписать всей прошивке 1.1.5, поэтому
совместимость проверена отдельно (раздел ниже).

**MC** выполняется на другой плате (soc0, PC1 `10.0.1.40`, отвечает на ping за 0,4 мс), прошивка которой с .66
не читается. Косвенное подтверждение совместимости: типы живых сервисов и топиков MC совпадают с определениями в
v1.1.0.4, значения в ответах совпадают с таблицами документации 1.1.0 (ниже). На плате MC программы разработчика
запускать нельзя (требование документации), и мы этого не делаем: всё работает на .66 (`10.0.1.41`).

## Живая проверка MC (только чтение)

| Что | Ответ робота | Документация 1.1.0 |
|---|---|---|
| `GetMcLocomotionSpeedStatus`, режим | LOW (1), вперёд −1,0…+0,8, вбок ±1,0, поворот ±1,0 | совпадает |
| `GetCurrentInputSource` | `rc`, приоритет 80, таймаут 1000 мс | совпадает (пульт, 80, 1000) |
| `/aima/mc/common/state` | 10 Гц, BEST_EFFORT, издатель TRANSIENT_LOCAL, `action_desc=STAND_DEFAULT`, статус 100 | совпадает |
| Сервисы | `/aimdk_5Fmsgs/srv/{SetMcInputSource,GetCurrentInputSource,GetMcLocomotionSpeedStatus,SetMcAction,...}` | совпадает |
| Издатели `/aima/mc/locomotion/velocity` | вендорские `aimrt_planning`, `rc`, `app_proxy`, `interaction` (4) | пресеты: pnc, rc, app_proxy, interaction |

## Сверка с онлайн-документацией

2026-10-21 загружены `x2-aimdk.agibot.com/zh-cn/dev/Interface/control_mod/{locomotion,MC_control}.html`. Страницы озаглавлены
«AimDK_X2 1.1.0», то есть новее локальной копии на момент загрузки ничего нет. Совпали с локальной: топик
`/aima/mc/locomotion/velocity` (BEST_EFFORT + VOLATILE, рекомендуемые 50 Гц), обнуление через 200 мс и возврат в стойку через
2000 мс, пороги страгивания (вперёд ~0,1; вбок ~0,5…0,7; поворот ~0,03…0,05, «по реальному роботу»), границы режимов
LOW/MEDIUM/HIGH/STEP, встроенные источники и приоритеты (linkswarm 90, rc 80, vr 70, app_proxy 60, interaction 50, pnc 40, таймаут
1000 мс), правила арбитража, префикс `sdk_` и диапазон 20…39 для SDK, ограничение «не более 2 нод в секунду». Онлайн-страница
динамическая; при смене версии сверку повторить.

## B. Почему `ros2 pkg prefix aimdk_msgs` говорит «Package not found»

Причина: пакет лежит в отдельном префиксе `/agibot/software/common` (там есть свой ament index), а в обычной оболочке
`AMENT_PREFIX_PATH` содержит только `/opt/ros/humble`. Граф при этом видит типы, потому что вендорские процессы используют
их из своего префикса, а DDS передаёт имена типов. Пакет, определения и библиотеки на месте, ничего не потеряно.

Исправление: аддитивный overlay `ws/src/navigation/aimdk_env.sh`, который только добавляет префикс в начало переменных
процесса (после `setup.bash` ROS). Пересборка и замена системных библиотек не потребовались.

Ловушка: при установке `PYTHONPATH` нельзя перезаписывать значение. Без записей ROS пропадает `rosidl_parser`, и импорт
сообщений падает (`No module named 'rosidl_parser'`). В `aimdk_env.sh` значение дополняется.

`ros2 interface show aimdk_msgs/msg/McActionInfo` падает с `InvalidResourceName: //` из-за комментария в определении (ошибка
разбора в инструменте, не в рантайме): живой `ros2 topic echo /aima/mc/common/state` и Python-импорт работают.

## Сверка определений (установленные и исходники SDK)

Скрипт `tests/aimdk/check_aimdk_interfaces.sh` (запуск на X2, выход 0 при успехе). Результат 2026-10-21, все **SAME**:
`McLocomotionVelocity`, `McInputSource`, `McCommonState`, `McLocomotionSpeedStatus`, `JointStateArray`, `JointState`,
`SetMcInputSource`, `GetCurrentInputSource`, `GetMcLocomotionSpeedStatus`. Определения из репозитория Nova-Valley
(`src/aimdk_msgs`) на роботе не используются и не сравнивались, нужные нам типы берутся только из установленного
префикса.

## Версии и обновления

Любое обновление прошивки (FOTA) нужно сопровождать повтором `check_aimdk_interfaces.sh`. Если определения
`McLocomotionVelocity`, `SetMcInputSource` или `McCommonState` изменятся, адаптер отказывается считаться проверенным.
