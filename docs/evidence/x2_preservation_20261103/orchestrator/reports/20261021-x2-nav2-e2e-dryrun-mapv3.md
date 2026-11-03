# map v3 (X2 imprint erase)
- v3: /home/agi/control_backend/ws/src/maps/first_map_nav2_v3/ (map.yaml identical to v2; sha f49dd066...)
- map.pgm sha256 f6032f75ffd386361993a4b4864cf0816713fc27f6d037c88d929ae5d48562fb (v2: 545630c1...)
- tool: control_backend/tools/derive_nav2_map.py --erase-imprint X Y R --erase-reason TEXT (opt-in; v2 re-derivation verified byte-identical: pgm, yaml, manifest)
- cmd: derive_nav2_map.py first_map first_map_nav2_v3 "D1 first_map (saved 2026-10-21T18:07:30+05:00)" --extra --erase-imprint -7.361 6.053 0.45 --erase-reason "X2 self-imprint during D1 mapping, operator-confirmed 2026-10-21"
- erased: 40 cells (39 occupied in 3 components, 1 unknown), no components skipped as walls; bbox map x[-7.525,-7.075] y[5.875,6.275]; px row/col [92,115]-[100,124]
- histogram v3: 0:4669 205:76222 254:55003
- checks: cells within 0.35 m of anchor non-free: 0; nearest non-free cell to anchor 1.05 m; 1.0 m segment along yaw -1.294 clear with 0.35 m clearance: TRUE (min dist to non-free 1.05 m)
- alt headings (L=1.0, clearance>=0.35): -60 deg (1.05), -30 deg (1.05), 90 deg (1.04) all clear
- first_map, tests/fixtures, launch defaults untouched.
## Render (+-1.5 m, '@' anchor, '#'occ '?'unk '.'free; 0.05 m/cell; rows top=north)
Before: imprint = ring of '#' around '@' ~0.4 m wide, rows ~ +-0.3 m (see v2 map.pgm). After: area around '@' fully '.' (see above ASCII; reproduce via script).
