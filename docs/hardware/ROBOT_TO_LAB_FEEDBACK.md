<!-- QAZBOT-CONTRACT {"contract_version": 1, "doc": "ROBOT_TO_LAB_FEEDBACK", "last_lab_integrated_sha": null, "last_lab_reviewed_sha": null} -->

# Robot → Lab feedback

Пишет `/robot-sync-validate` на стороне X2 (ветка обратной связи робота), читает `/nightly-resume` в лаборатории. Пока пусто — **замеров с X2 нет** (`HARDWARE_VERIFIED`: ничего).

- `last_lab_integrated_sha` — только изменения, реально интегрированные и проверенные на роботе.
- `last_lab_reviewed_sha` — всё, что уже разобрано (чтобы не читать повторно).
- Каждая запись: `HARDWARE_VERIFIED` | `UNVERIFIED` | `NEEDS_APPROVAL` | `NEEDS_HARDWARE` | `TWIN_MISMATCH`, с SHA робота и лаборатории, условиями, командами воспроизведения, SHA256 артефактов.

## Записи

- 2026-11-02 X2 — D-04 candidate C: **UNVERIFIED** на железе (OFFLINE_VERIFIED, V1 PASS), **NEEDS_APPROVAL** для статичной квалификации. D-05: **NEEDS_HARDWARE** (калибровка 0.10–0.60). Детали: docs/engineering/d04_d05_20261102/README.md. Production X2 не изменялся.
