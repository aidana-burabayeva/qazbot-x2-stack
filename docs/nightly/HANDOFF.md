# Nightly — handoff

Ветка `test/p20-twin02-sim-20261029`, worktree `/home/adn/QazBot/control_backend_p20_twin02`. Читать: SPEC.md → DECISIONS.md → STATUS.md. Робот не трогать.

Воспроизвести CCW A/B (SIM only, новые имена результатов):
```
cmake сборка инструментов: см. tests/p20_sim/README.md (torso_odom + recorder → /tmp/qazbot-p20-build, recorder в timing/)
P20_RUN=ccw_<stamp> GOAL_ARGS="--only-ccw --record-phases --xy-tol 0.20 --yaw-tol 0.35" tests/p20_sim/run_nav2_trace.sh
python3 tools/nightly/phase_analysis.py /home/adn/QazBot/digital_twin/results/<run>
```
Каждый прогон — свежие контейнеры (runner делает --force-recreate и down). Результаты не перезаписываются.
Следующий шаг: фаза B (анализ main_office: помещения, граф связности, кандидаты дверей — пометить UNVALIDATED), затем C.
