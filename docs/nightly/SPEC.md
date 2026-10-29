# QazBot Nightly — спецификация (сводка требований оператора)

Источник: два сообщения оператора 2026-10-30 (основная спецификация + Autonomous Engineering Extension). Здесь только сжатые нормативные требования; статус — в STATUS.md, решения — в DECISIONS.md.

## Жёсткие границы
- Только локальная симуляция. Физический X2: никаких команд, SSH, развёртывания. Результаты sim не разрешают авто-деплой параметров.
- Не ослаблять T1/T5, footprint, collision detection, safety margin; не расширять допуски цели ради PASS.
- Не менять одновременно P20, локализацию, costmap, контроллер. Один фактор за A/B.
- Один владелец файла: single-owner lease + отдельный git worktree на агента. Лимиты провайдера не обходить.
- Nav2 `SUCCEEDED` — не доказательство успеха. Столкновение/нарушение safety = hard fail независимо от reward.

## Уровни
- L1 быстрый C++ (геометрия, swept footprint, достижимость, упрощённая кинематика) — помечен как модель.
- L2 настоящий ROS 2/Nav2 + Virtual MC. L3 официальный MC + MuJoCo (1 worker, затем замер).

## Цикл
DETECT → REPRODUCE → ROOT CAUSE → FIX → A/B → REGRESSION → COMMIT → CONTINUE. Агенты вызываются по новой сигнатуре отказа, не на каждый тест; сначала `triage.json`.

## Оптимизация (порядок)
1. baseline + детерминированный A/B; 2. Bayesian/multi-objective; 3. coverage-guided; 4. RL — только если измерения докажут выигрыш.
Параметры: planners, RPP, smoother, скорости/ускорения, поворот, terminal alignment; costmap — отдельной серией.
Победители подтверждаются на unseen маршрутах/возмущениях.

## Парето-набор (требование оператора)
Хранить не одну «лучшую» конфигурацию: `baseline`, `safest`, `most_accurate`, `fastest_among_safe` + весь Парето-фронт (clearance, XY/yaw точность, время, recoveries). «Fastest» допустим только среди конфигураций с 0 hard-fail на всех сценариях.

## Fitness
+ реальное достижение XY/yaw, безопасные двери, малая ошибка траектории, зазоры, плавность, время, устойчивость.
− столкновения/margin (hard), осцилляции, stall, drift, coast, лишние replans/recoveries, неверный yaw, нестабильная локализация.
Невозможная цель: корректный безопасный отказ = успех.

## GPU / headless
Без GUI. GPU использовать только при измеренной выгоде; Nav2 и MuJoCo physics не считать GPU-ускоренными. Контроль CPU/RAM/VRAM и real-time factor; прогон при троттлинге помечается неквалифицированным.

## Evidence
Кадры: старт, подход к двери, мин. зазор, нарушение, replan/recovery, финал; 2D-визуализация planned/actual, footprint, clearance. Привязка к run_id/scenario_id/config. HTML nightly report.

## Состояние
SQLite WAL (очередь, индекс), статусы PENDING/RUNNING/COMPLETED/FAILED/INTERRUPTED/NEEDS_ENGINEERING/NEEDS_APPROVAL. Checkpoint после каждого сценария и инженерной задачи. Без перезаписи прошлых failures.
