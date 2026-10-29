"""List cloud/scan/tf topic counts of the recorded physical-X2 bags in /evidence (read-only)."""
import glob
import sqlite3

for b in sorted(glob.glob('/evidence/*/bag/bag_0.db3')):
    try:
        c = sqlite3.connect(f'file:{b}?mode=ro', uri=True)
        t = {r[0]: r[1] for r in c.execute('select id,name from topics')}
        n = {t[i]: cnt for i, cnt in c.execute('select topic_id,count(*) from messages group by topic_id')}
    except sqlite3.DatabaseError as exc:
        print(b.split('/')[2], 'UNREADABLE', exc)
        continue
    print(b.split('/')[2], {k: v for k, v in n.items() if any(w in k for w in ('cloud', 'scan', '/tf', 'odom_level'))})
