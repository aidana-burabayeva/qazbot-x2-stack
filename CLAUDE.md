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
