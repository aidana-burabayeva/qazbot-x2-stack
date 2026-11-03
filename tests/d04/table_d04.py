#!/usr/bin/env python3
"""Print the compact A/B/C/D table from results/summary.json (markdown)."""
import json
import sys

d = json.load(open(sys.argv[1] if len(sys.argv) > 1 else 'results/summary.json'))
S = d['summary']
cols = ['region_lethal_while_present_min', 'region_present_updates_with_mark', 'region_present_updates',
        'region_lethal_after_leave_final', 'updates_to_clear_after_leave', 'coll_any_while_present',
        'coll_any_frames_after_leave', 'coll_any_after_leave_final', 'edge_lethal_final', 'lethal_final',
        'final_cells_only_in_A', 'final_cells_only_in_variant', 'seed_kept']
print('| bag | scenario | var | ' + ' | '.join(cols) + ' |')
print('|' + '---|' * (3 + len(cols)))
for bag, scs in S.items():
    for sc, vs in scs.items():
        for v, m in vs.items():
            print(f'| {bag} | {sc} | {v} | ' + ' | '.join(str(m.get(c, '')) for c in cols) + ' |')
print()
print('| bag | var | t_bridge p50/p95/p99 ms | t_layer p50 ms (Python emul) | bytes/update | clear rays | mark pts |')
print('|---|---|---|---|---|---|---|')
for bag, scs in S.items():
    for v, m in scs['recorded_only'].items():
        print(f"| {bag} | {v} | {m['t_bridge_ms_p50']:.2f}/{m['t_bridge_ms_p95']:.2f}/{m['t_bridge_ms_p99']:.2f} | "
              f"{m['t_layer_ms_p50']:.1f} | {m['bytes_per_update']:.0f} | {m['n_clear_mean']:.0f} | {m['n_mark_mean']:.0f} |")
print()
print(json.dumps(d['meta']['bags'], indent=0))
print(json.dumps(d['meta']['lidar_attitude'], indent=0))
for bag, c in d['meta']['coverage'].items():
    print(bag, {k: (v['all_min_max_p05_p995'], v['band_min_max_p05_p995'], v['band_outside_old_fov'])
                for k, v in c.items() if isinstance(v, dict)}, c['floor_z_rel_imu'])
