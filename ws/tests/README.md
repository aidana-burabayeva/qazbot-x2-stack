# ws/tests — лабораторные стенды (не colcon test)

Юнит-тесты логики пакетов лежат в `ws/src/<package>/test/` и запускаются `colcon test`. Здесь — интеграционные стенды,
модели и офлайн-проверки (nav_e2e, controllers, collision_monitor, d04/d05, footprint, planner_clearance, s3,
localization, loc_trust ROS-e2e, motion_adapter ROS-фейки, twin). Результаты прогонов в репозиторий не кладутся
(`/home/agi/qazbot_archive/...`). Часть стендов ссылается на раскладку до консолидации или на bags в legacy
`logs/` (например `s3/test_native_limiter.py` → `logs/mc_execution23/...`); перед запуском с релизом их нужно портировать.
Запуск только в изолированном домене (88/87, `ROS_LOCALHOST_ONLY=1`).
