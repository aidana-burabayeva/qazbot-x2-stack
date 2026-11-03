import sys, pickle, math
from mcap.stream_reader import StreamReader
from mcap.records import Schema, Channel, Message
from mcap_ros2.decoder import DecoderFactory
want = lambda t: ('cmd_vel' in t or 'odom' in t or t in ('/tf','/qazbot_d1_map/tf') or t.endswith('/tf') or 'amcl_pose' in t
                  or 'locomotion/velocity' in t or t.endswith('/goal_pose') or 'mc/common/state' in t)
T0,T1=float(sys.argv[2]),float(sys.argv[3])
f=open(sys.argv[1],'rb'); sr=StreamReader(f, record_size_limit=None)
schemas={}; chans={}; out={}; df=DecoderFactory(); decs={}; counts={}
try:
    for rec in sr.records:
        if isinstance(rec,Schema): schemas[rec.id]=rec
        elif isinstance(rec,Channel): chans[rec.id]=rec
        elif isinstance(rec,Message):
            c=chans.get(rec.channel_id)
            if c is None: continue
            counts[c.topic]=counts.get(c.topic,0)+1
            t=rec.log_time*1e-9
            if not (T0<=t<=T1) or not want(c.topic): continue
            if rec.channel_id not in decs:
                decs[rec.channel_id]=df.decoder_for('cdr',schemas[c.schema_id])
            try: m=decs[rec.channel_id](rec.data)
            except Exception as e: continue
            out.setdefault(c.topic,[]).append((t,m))
except Exception as e:
    print('stopped:',repr(e)[:200])
for k,v in sorted(counts.items()): print(k, v, len(out.get(k,[])), schemas[[c for c in chans.values() if c.topic==k][0].schema_id].name)
# convert to plain
def plain(m):
    if hasattr(m,'__slots__') or hasattr(m,'__dict__'):
        d={}
        for k in (getattr(m,'__slots__',None) or vars(m).keys()):
            d[k.lstrip('_')]=plain(getattr(m,k))
        return d
    if isinstance(m,(list,tuple)): return [plain(x) for x in m]
    return m
pickle.dump({k:[(t,plain(m)) for t,m in v] for k,v in out.items()}, open('win.pkl','wb'))
