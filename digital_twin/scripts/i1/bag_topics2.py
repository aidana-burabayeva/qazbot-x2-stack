"""Topic/message counts of a bag directory (sqlite3, read-only): bag_topics2.py DIR..."""
import glob
import sqlite3
import sys

for d in sys.argv[1:]:
    for b in sorted(glob.glob(f'{d}/*.db3')):
        try:
            c = sqlite3.connect(f'file:{b}?mode=ro', uri=True)
            t = {r[0]: (r[1], r[2]) for r in c.execute('select id,name,type from topics')}
            n = {t[i][0]: cnt for i, cnt in c.execute('select topic_id,count(*) from messages group by topic_id')}
            print(b, {k.split('/')[-1] + '@' + k.split('/')[1]: v for k, v in n.items()
                      if any(w in k for w in ('cloud', 'Odometry', 'imu', 'lidar_pointcloud', 'scan'))})
        except sqlite3.DatabaseError as exc:
            print(b, 'UNREADABLE', exc)
