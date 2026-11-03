import sqlite3, math
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
def read(db, topics, t0=None, t1=None):
    c=sqlite3.connect(db)
    tt={r[0]:(r[1],r[2]) for r in c.execute('select id,name,type from topics')}
    ids={i:(n,ty) for i,(n,ty) in tt.items() if n in topics}
    cls={i:get_message(ty) for i,(n,ty) in ids.items()}
    q='select topic_id,timestamp,data from messages where topic_id in (%s)'%','.join(str(i) for i in ids)
    if t0 is not None: q+=' and timestamp>=%d'%int(t0*1e9)
    if t1 is not None: q+=' and timestamp<=%d'%int(t1*1e9)
    q+=' order by timestamp'
    for tid,ts,data in c.execute(q):
        yield ids[tid][0], ts*1e-9, deserialize_message(data, cls[tid])
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def st(h): return h.stamp.sec+h.stamp.nanosec*1e-9
