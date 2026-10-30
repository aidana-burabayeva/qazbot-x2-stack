#!/usr/bin/env python3
"""Static HTML nightly report from run/nightly artefacts (queue, L1 signatures, L2 json, tournament, map overlay). Offline, no JS deps."""
import base64, html, json, sqlite3, time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RUN = ROOT / 'run/nightly'


def img(p):
    return f'<img style="max-width:100%" src="data:image/png;base64,{base64.b64encode(Path(p).read_bytes()).decode()}">' if Path(p).exists() else ''


def table(rows, head):
    return '<table><tr>' + ''.join(f'<th>{html.escape(str(h))}</th>' for h in head) + '</tr>' + ''.join('<tr>' + ''.join(f'<td>{html.escape(str(c))}</td>' for c in r) + '</tr>' for r in rows) + '</table>'


def main():
    db = sqlite3.connect(RUN / 'queue.db')
    q = db.execute('select name,kind,state,attempts,coalesce(reason,"") from tasks order by id').fetchall()
    sig = db.execute('select signature,first_task,count from failures order by count desc').fetchall()
    l2 = [json.loads(p.read_text()) for p in sorted(RUN.glob('l2_s*.json'))]
    t = ROOT / 'docs/nightly/tournament/l1_20261030/tournament.json'
    sel = []
    if t.exists():
        d = json.loads(t.read_text())
        for k, v in d['selected'].items():
            sel.append([k, json.dumps(v['cfg']) if v else '-', v and v['train']['hard_fail'], v and round(v['train']['success_rate'], 2), v and round(v['train']['mean_time_s'], 1), v and v.get('unseen', {}).get('hard_fail')])
    css = ':root{color-scheme:light dark}body{font:14px system-ui;margin:16px auto;max-width:1000px;padding:0 16px}table{border-collapse:collapse;width:100%}td,th{border:1px solid #8884;padding:3px 6px;text-align:left}h2{margin-top:28px}'
    body = f'<h1>QazBot nightly report</h1><p>{time.strftime("%Y-%m-%d %H:%M")} — SIMULATION ONLY; L1 is a model; nothing here authorises deployment to the physical X2.</p>'
    body += '<h2>Queue</h2>' + table(q, ['task', 'kind', 'state', 'attempts', 'reason'])
    body += '<h2>Failure signatures</h2>' + table(sig, ['signature', 'first task', 'count'])
    body += '<h2>L2 (real Nav2 chain) scenarios</h2>' + table([[x['scenario'], x['category'], x['nav2_rc'], x['clearance'].get('min_clearance_m'), x['clearance'].get('samples_in_collision')] for x in l2], ['scenario', 'category', 'nav2 rc (0=PASS)', 'min clearance m', 'contact samples'])
    body += '<h2>Config tournament (L1 model) — Pareto picks</h2>' + table(sel, ['pick', 'config', 'hard fails (train)', 'success', 'mean t s', 'hard fails (unseen)'])
    body += '<h2>Map regions / door candidates (UNVALIDATED)</h2>' + img(ROOT / 'docs/nightly/map/main_office_overlay.png')
    out = RUN / 'report.html'
    out.write_text(f'<!doctype html><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1"><title>QazBot nightly</title><style>{css}</style>{body}')
    print(out, out.stat().st_size)


if __name__ == '__main__':
    main()
