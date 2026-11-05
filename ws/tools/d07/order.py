import rosbag2_py, numpy as np, pickle
BAG='/home/agi/control_backend/docs/evidence/nav_20261102_live/d04_static_bag_183015/d04_static_bag_183015_0.mcap'
rd=rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=BAG,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
rd.set_filter(rosbag2_py.StorageFilter(topics=['/qazbot_lio/odometry','/qazbot_lio/cloud_registered']))
def st(raw): return int.from_bytes(raw[4:8],'little',signed=True)*10**9+int.from_bytes(raw[8:12],'little')
od={}; cl={}; seq=[]
while rd.has_next():
    n,data,ts=rd.read_next(); s=st(data)
    (od if 'odom' in n else cl)[s]=(ts,len(data)); seq.append((ts,'o' if 'odom' in n else 'c',s))
pickle.dump((od,cl,seq),open('order.pkl','wb'))
miss=[1793701951310885906,1793702201910807847,1793702367310158967,1793702280310674142]
print('odom',len(od),'cloud',len(cl),'cloud stamps without odom',[s for s in cl if s not in od][:5])
d=np.array([(od[s][0]-cl[s][0])/1e6 for s in cl if s in od])
print('odom_recv - cloud_recv ms: min %.2f p1 %.2f p50 %.2f p99 %.2f max %.2f; frac odom AFTER cloud %.4f'%(d.min(),*np.percentile(d,[1,50,99]),d.max(),(d>0).mean()))
for s in miss:
    print(s, 'odom' if s in od else 'NO ODOM', ((od[s][0]-cl[s][0])/1e6 if s in od and s in cl else None), cl.get(s))
# rank of the three among the largest positive deltas
keys=[s for s in cl if s in od]; dd={s:(od[s][0]-cl[s][0])/1e6 for s in keys}
top=sorted(dd,key=lambda s:-dd[s])[:15]; print('top odom-after-cloud', [(s,round(dd[s],2)) for s in top])
