# Current navigation optimization continuation

Read [docs/nightly/NAV_OPTIMIZATION.md](docs/nightly/NAV_OPTIMIZATION.md) and its linked evidence. P0 source fixes/native probe/archive implemented; live doctor and L3 qualification remain UNVERIFIED. Do not enable optimization before real P0 PASS and single-worker official MC baseline. Physical X2 untouched; existing queue remains frozen.

---

# 2026-10-31 — terminal LAB bootstrap

Продолжение инфраструктуры: [docs/nightly/LABCTL.md](docs/nightly/LABCTL.md), evidence [docs/evidence/lab_bootstrap_20261031/README.md](docs/evidence/lab_bootstrap_20261031/README.md). Control checkout — `lab/control`, интеграционная ветка — `lab/stable`, не checkout. Управление: `tools/nightly/labctl.sh doctor|start|status|stop`. Runtime acceptance ещё не завершён: текущая среда блокирует Docker/systemd/GitHub. Не трогать X2 и не объявлять новый SIM PASS по историческим отчётам ниже.

---

# Текущая передача Codex → Claude: bounded stall A/B

Ветка `nightly/codex-stall-20261030`, worktree `/home/adn/QazBot/control_backend_codex_nightly`. Читать [docs/nightly/HANDOFF.md](docs/nightly/HANDOFF.md) и [docs/nightly/CODEX_STALL_AB.md](docs/nightly/CODEX_STALL_AB.md). Один A/B выполнен, дальнейшие sim runs по этой задаче не разрешены. B успешен, но causal attribution confounded world/plant registration; не менять production default и не трогать физический X2. Соседний worktree не редактировать.

---

# Текущая передача Claude: P20 / TWIN-02, simulation only

Активная ветка `test/p20-twin02-sim-20261029`; начать с [docs/P20_TWIN02_HANDOFF.md](docs/P20_TWIN02_HANDOFF.md), затем [docs/HANDOFF.md](docs/HANDOFF.md) и [docs/TEST_STATUS.md](docs/TEST_STATUS.md). Сохранить чужие dirty worktree. Карта называется `main_office`, не `t2_nav2`.

Калибровка P20 и DDS recovery regressions прошли. Следующий открытый gate — отдельное время финального выравнивания полной CCW Nav2-цели (общая длительность 31.8 с, повторные XY-подходы). Recovery — экспериментальный C++ кандидат; реальный evidence provider и production wiring не готовы. Не устанавливать на X2, не ослаблять T1/T5/footprint/collision detection и не менять одновременно P20, localisation и costmap. Simulation processes этой сессии остановлены.

**Nightly lab (sim):** состояние в `python3 tools/nightly/nq.py status` и `docs/nightly/{STATUS,HANDOFF}.md`; оркестратор `qazbot-nightly` (systemd --user). Подробности: skill `qazbot-nightly`.

---

# Claude Code: QazBot X2

Прочитайте [AGENTS.md](AGENTS.md), затем [docs/HANDOFF.md](docs/HANDOFF.md).
Для результатов проверок используйте [docs/TEST_STATUS.md](docs/TEST_STATUS.md).

Продолжайте с опубликованного SHA/ветки из передачи. Сначала проверьте HEAD,
dirty files и актуальные процессы X2; старые PID не являются текущим состоянием.
Цель — первый безопасный физический NavigateToPose через RViz, последовательно
через T5, I1 E2E, review и отдельно разрешённую установку. Не начинайте аудит заново.

T5 уже установлен; не переустанавливайте. Не копируйте старый level_bridge целиком.
Не меняйте архитектуру, collision detection, реальный footprint и freshness guards.
Старт сенсорного стека/запись облака, deployment I1 и движение требуют своих
разрешений оператора. Не запускайте второй стек или два FAST-LIO.

На X2 ~/control_backend не Git checkout. Не делайте там git init. Исходники Git
и deployed-каталог — разные пути, указанные в HANDOFF.

Перед завершением обновите HANDOFF и опубликуйте коммиты в отдельной ветке.
По актуальному указанию оператора bags/логи робота публикуйте в GitHub под docs/evidence с SHA256 (lossless .db3.gz + metadata.yaml). Секреты и runtime не коммитьте.
