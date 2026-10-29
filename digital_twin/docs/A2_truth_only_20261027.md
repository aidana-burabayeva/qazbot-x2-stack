# A2 truth-only Nav2 (roadmap v1.3, поток A) — 2026-10-27

Цель: отделить контроллер и MC от локализации. Единственный источник `map→odom` и вердикта доверия — истина симулятора; AMCL и Localization Trust выключены. Только twin.

## Как включается
`make scenario-truth` (или `TWIN_LOCALIZATION=truth make up scenario`). Launch: `localization:=truth` → `truth_loc.launch.py` (map_server + `truth_localizer`).
`truth_localizer` публикует `map→odom` = единица (plant уже выдаёт `odom→base_footprint` в карте), `localization_trust/{trusted,status}` и сервисы `arm_check`, `confirm_anchor` (координатор готов только пока `arm_check` существует). Сценарий: `--localization truth`.
Режим по умолчанию (`amcl`) не изменён. Юнит-тесты: 38 passed, 2 skipped; flake8 чист.

## Результат (`results/a2-truth2/results.json`, официальный MC v1.0.0)
| | AMCL (`a0-baseline`) | truth (`a2-truth2`) |
|---|---|---|
| hard_failed | нет | нет |
| «туда» | ABORTED(6), xy 0.152 м, yaw 0.899 рад | ABORTED(6), xy 0.253 м, yaw 0.940 рад |
| «обратно» | ABORTED(6), xy 1.653 м, yaw 0.952 рад | ABORTED(6), xy 1.738 м, yaw 0.973 рад |
| command_limits | n=2459, vmax 0.3, wmax 0.5 | n=2237, vmax 0.3, wmax 0.5 |

## Вывод
Отказ воспроизводится с идеальной локализацией: AMCL/Trust в нём не виноваты. Причина лежит в цепочке контроллер (RPP) → адаптер → MC (поворот на месте на 0,5 рад/с, Блокер 1), что согласуется с `research/docs/digital_twin/mc_turn_control_20261026.md`. В twin уже есть SIM-кандидат `mc_turn_controller` (research/tools/twin), в стек не включён.
Не проверено: причина ABORTED на уровне Nav2 (BT/progress checker) в этом прогоне; лог стека не сохранялся. Следующий шаг: прогон с сохранением логов Nav2 и сравнением кандидата поворота A/B.

## Причина ABORTED (прогон `a2-truth4`, трасса `results/a2-truth4/{trace,cmd_trace}.csv`, лог Nav2 `Failed to make progress`)
Хронология (truth-локализация, официальный MC v1.0.0), t от старта сценария:
| t, с | Что происходит |
|---|---|
| 3–8 | Команда ходьбы ≤0,3 м/с, фактическая скорость 0,4–0,5 м/с. Цель x=0,225 пройдена, остановка на x≈0,40 (перелёт ≈0,18 м). Курс ушёл с −0,02 до +0,39 рад за время ходьбы |
| 8–23 | RPP подаёт разворот −0,5 рад/с на месте. Фактическая угловая скорость ≈0 все 15 с (yaw 0,38→0,33 лишь дрейф) |
| ≈23 | `PoseProgressChecker` (0,1 м или 0,25 рад за 15 с) → `Failed to make progress` → Nav2 `ABORTED` |
| 25–40 | «обратно»: +0,5 рад/с даёт поворот лишь до yaw≈0,88 (≈0,5 рад) и затухает до 0, при команде 0,5; снова нет прогресса → ABORTED |

Вывод (подтверждено трассой): два независимых свойства MC v1.0.0 в twin, при идеальной локализации: (1) эффективная минимальная скорость ходьбы ≈0,4 м/с вместо команды 0,12–0,3 (перелёт, уход курса); (2) разворот на месте 0,5 рад/с не исполняется (CW) или затухает (CCW). Совпадает с Блокерами 1–2 и с `research/docs/digital_twin/mc_turn_control_20261026.md`.
Ограничение: в twin MC v1.0.0, у робота v1.1.0-r9-rc9; перенос количественных порогов на робота не доказан.
Следующий шаг (одно изменение за раз): A/B штатного контроллера и SIM-кандидата `mc_turn_controller` на этом же сценарии.

## A/B: SIM-кандидат `qazbot_mc_turn` (прогон `a3-turn1` против `a2-truth4`), одно изменение — контроллер/цель-чекер
Включение: `make scenario-turn` (`TWIN_LOCALIZATION=truth TWIN_MC_TURN=1 MAX_ANGULAR=0.8`, 0,8 рад/с только для кандидата, в sim). Плагины `qazbot_mc_turn::Controller` и `::SettledGoal` загружены (лог controller_server). Исходники кандидата — копия `research/tools/twin/mc_turn_controller` в `candidates/qazbot_mc_turn`; изменён только guard (требует `TWIN_SANDBOX=1`/домен 87 вместо `ROS_LOCALHOST_ONLY=1`), то же в копии `nav_control.launch.py` внутри образа (исходники control_backend не тронуты).
| | штатный RPP (`a2-truth4`) | кандидат (`a3-turn1`) |
|---|---|---|
| «туда» | ABORTED, xy 0,25 м, yaw 0,94 | ABORTED, xy 0,13 м, yaw 0,51 |
| «обратно» (разворот 180° и ход) | ABORTED, xy 1,74 м, yaw 0,97 | **SUCCEEDED**, xy 0,073 м, yaw 0,087 рад |
| разворот на месте | не исполняется | исполняется (yaw −2,7→−0,1 за ≈2 с) |
Остаётся: перелёт ≈0,19 м на ходьбе (эффективная скорость 0,4–0,5 при команде 0,3) → цель по xy вне допуска 0,10 → RPP строит возврат с разворотом на 180° и снова доводкой; «туда» заканчивается ABORTED. Это Блокер 2, его адресует отдельный SIM-кандидат торможения (`QAZBOT_SIM_MC_WALK`), не включён.
Статистика: по одному прогону на вариант; повторяемость не проверена.
