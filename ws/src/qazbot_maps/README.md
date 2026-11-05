# qazbot_maps

Occupancy maps (nav2_map_server format) installed to `share/qazbot_maps/maps/<name>/`. Each map keeps its provenance
(`manifest.json` with `pgm_sha256`, `meta.yaml`). The localization trust node pins the served image by
`expected_map_sha256`.

| Map | Use |
|---|---|
| `d1_qazbot_20261029` | **production** (profile x2_d1), pgm sha256 72ee4094…b354 |
| `first_map_nav2_v2` | default of the localization/nav launch files when `map_yaml` is not given (legacy default) |
| `first_map_nav2_v3`, `first_map_nav2_v3_despeckled`, `first_map` | older maps / tests |

No nodes, topics or parameters. Usage: `$(ros2 pkg prefix --share qazbot_maps)/maps/d1_qazbot_20261029/map.yaml`.
