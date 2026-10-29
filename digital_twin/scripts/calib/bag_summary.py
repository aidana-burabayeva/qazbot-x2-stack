"""Per-topic count, rate and span of ROS2 sqlite3 bags: bag_summary.py BAG_DIR [BAG_DIR...]   (run in the dev container)."""
import sqlite3
import sys
from pathlib import Path

for d in map(Path, sys.argv[1:]):
    db = next(d.glob('*.db3'))
    c = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
    rows = c.execute('select t.name, t.type, count(m.id), min(m.timestamp), max(m.timestamp) from topics t left join messages m on m.topic_id=t.id group by t.id').fetchall()
    t0 = min(r[3] for r in rows if r[3])
    t1 = max(r[4] for r in rows if r[4])
    print(f'== {d.name}: {(t1 - t0) / 1e9:.1f} s')
    for n, ty, k, a, b in sorted(rows):
        span = (b - a) / 1e9 if k and k > 1 else 0
        print(f'  {n:55} {ty.split("/")[-1]:28} n={k:6} {f"{(k - 1) / span:7.2f} Hz" if span else "     -   "}')
