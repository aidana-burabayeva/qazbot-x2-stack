"""Compact markdown table of all run JSONs in a results dir: python3 summarize.py results/"""
import json,glob,sys,os
D=sys.argv[1]
rows=[json.load(open(f)) for f in sorted(glob.glob(D+'/*_s*.json')) if 'summary' not in f]
order={'rpp':0,'dwb':1,'mppi':2}; so={'nominal':0,'static_block':1,'transient':2,'stale_cell':3}
rows.sort(key=lambda r:(r['model'],so[r['scenario']],order[r['controller']]))
def g(r,k,f=None):
    v=r.get(k); 
    if v is None: return '-'
    return f.format(v) if f else str(v)
print('| model | scenario | ctrl | result | t[s] | prog[m] | dev mean/max | min clr (all/box) | stop/resume | wz flips/s | vx jerk rms | P23 pure-rot / low-vx / switches | period p50/p95/p99 ms | CPU % / ms per cycle / RSS MB | log: coll / fail-prog / mppi-fail / missed-rate |')
print('|'+'---|'*15)
for r in rows:
    sr='-'
    if r['scenario']=='transient': sr=f"stop={r.get('stopped_while_present')} resume={r.get('resume_delay_s')}s"
    if r['scenario']=='static_block': sr=f"stopped_before={r.get('stopped_before_box')} passed={r.get('passed_box')}"
    print(f"| {r['model']} | {r['scenario']} | {r['controller']} | {r.get('status')}{' ERR '+r['error'] if r.get('error') else ''} | {g(r,'time_s')} | {g(r,'progress_m')} | {g(r,'dev_mean')}/{g(r,'dev_max')} | {g(r,'min_clearance')}/{g(r,'min_clear_box')} | {sr} | {g(r,'wz_flips_per_s')} | {g(r,'vx_jerk_rms')} | {g(r,'p23_pure_rot_frac')}/{g(r,'p23_low_vx_frac')}/{g(r,'p23_mode_switches')} | {g(r,'nav_period_ms_p50')}/{g(r,'nav_period_ms_p95')}/{g(r,'nav_period_ms_p99')} | {g(r,'cs_cpu_pct')}/{g(r,'cs_cpu_ms_per_cycle')}/{g(r,'cs_rss_mb_max')} | {g(r,'log_collision_ahead')}/{g(r,'log_failed_progress')}/{g(r,'log_mppi_fail')}/{g(r,'log_missed_rate')} |")
