# B1 — карточка scheduler A/B на физическом X2 (НЕ ВЫПОЛНЕНО)
Предпосылка: roadmap v1.3 §8/B1, P2-T. Физический T1 механизм уже измерен; эффект снятия CPU pinning ещё не подтверждён.
Требуется разрешение владельца на эту изменяющую runtime диагностическую пробу и подтверждение стоящего робота с пультом у оператора.

## Предлагаемый минимальный вариант
Не перезапускать ROS nodes и не загружать другой Python code. Меняется только affinity существующего FAST-LIO PID86463 и его threads:
A: CPUs5,6 (текущий baseline).
B: CPUs0–7 (без pin).
После измерения вернуть исходную affinity. Это причинно чище контролируемого рестарта: AMCL seed, origins, TF owners и загруженные версии остаются теми же.
Если PID/argv/starttime изменились, сценарий должен остановиться, а не применить affinity к новому PID.

## До опыта
- P0-C snapshot/hash и active-library manifest сохранены; C1/C2 остаются staged, не применять вместе с affinity A/B.
- Один длительно живущий observer source→DDS receipt→take + один лёгкий /proc scheduler sampler, без publishers MC/сервисных изменений.
- Пассивно подтвердить MC source=rc, STAND_DEFAULT/FSM4; adapter live=false/dry_run, published=0; coordinator busy=false/gate=false, нет активной цели.
- Оператор подтверждает стойку и доступный пульт. Если робот движется от rc, пробы не начинать.
- Проверить source/TF health; начало observer учитывается отдельно как известный DDS join, сначала warmup.

## Последовательность
1. Один observer на весь A/B; sampler 100 мс. Примерно 30 с warmup без новых участников.
2. A: 290 с с прежней affinity. На +40/+100/+160/+220 с один short-lived DDS join на 8 с: bare/sub/bare/sub. Каждое создание разнесено по времени, не пачка ROS nodes.
3. Убедиться в прежнем PID/starttime, stable stand/dry-run/no goal. Снять pin для всех FAST-LIO threads, проверить /proc/<pid>/task/*/status.
4. B: 290 с, те же join offsets и variants.
5. В finally вернуть исходную affinity всех прежних/new threads. Если чужая смена PID — не трогать новый процесс, зафиксировать отказ.
6. Анализировать одинаковые phase windows; не сравнивать warmup A с steady B.

## Что записать
header/DDS publish/receipt/take stamps, p50/p95/p99/max hops; FAST-LIO source publication gaps; Trust T1 scan/odom ticks и интервалы; per-thread runqueue wait, CPU/RAM, zero-stamp и clock-reset события.
Node drop/CPU starvation не лечить новыми timestamps, retries или изменением T1.

## Критерии и остановка
Исследовательская цель: FAST-LIO gap<0.25 с, T1 scan/odom ticks в join windows=0, p50 FAST-LIO hop не хуже baseline более чем на0.035 с; оценить повторяемость, не обещать статистическую надёжность по одной паре.
Прекратить дальнейшие joins/phase switch и вернуть affinity при движении/смене MC source, live/armed control, активной цели, потере самого observer/sampler, смене PID, zero-stamp/reset или неожиданном поведении робота.
Ожидаемые измеряемые T1 при join A сохраняются как результат, не маскируются.
MC не регистрируется и команды скорости не публикуются. Никаких изменений vendor files.
После результата — отдельный GO/NO-GO и решение о постоянном lio_cpus setting. Переключение native_imu и controlled stack restart — отдельные последующие шаги.

