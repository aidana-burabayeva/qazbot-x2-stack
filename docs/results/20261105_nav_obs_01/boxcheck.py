"""NAV-OBS-01 stage 1 (no motion): localisation/TF/costmap consistency, box marking, Smac plan, footprint clearance.
usage: boxcheck.py [goal_ahead_m] [out.json]"""
import rclpy,time,math,json,sys,numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from nav2_msgs.srv import GetCostmap
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
from scipy import ndimage
AH=float(sys.argv[1]) if len(sys.argv)>1 else 4.0
rclpy.init();n=Node('boxcheck');TF={};be=QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT)
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def tf(m):
    for t in m.transforms: TF[(t.header.frame_id,t.child_frame_id)]=t.transform
n.create_subscription(TFMessage,'/qazbot_d1_map/tf',tf,be)
e=time.time()+3
while time.time()<e: rclpy.spin_once(n,timeout_sec=0.05)
M=TF[('map','odom')];O=TF[('odom','base_footprint')];ym=yaw(M.rotation)
x0=M.translation.x+math.cos(ym)*O.translation.x-math.sin(ym)*O.translation.y;y0=M.translation.y+math.sin(ym)*O.translation.x+math.cos(ym)*O.translation.y;yw=ym+yaw(O.rotation)
mo=(M.translation.x,M.translation.y,ym)
gx,gy=x0+AH*math.cos(yw),y0+AH*math.sin(yw)
print('pose map %.3f %.3f yaw %.3f ; goal %.3f %.3f'%(x0,y0,yw,gx,gy))
def getcm(which):
    c=n.create_client(GetCostmap,'/qazbot_d1_map/nav2/%s_costmap/get_costmap'%which);assert c.wait_for_service(5)
    f=c.call_async(GetCostmap.Request())
    while not f.done(): rclpy.spin_once(n,timeout_sec=0.05)
    r=f.result().map;a=np.array(r.data,dtype=np.uint8).reshape(r.metadata.size_y,r.metadata.size_x)
    return a,r.metadata.resolution,(r.metadata.origin.position.x,r.metadata.origin.position.y),r.header.frame_id
res={}
FOOT=[(0.17,0.229),(0.008,0.34),(-0.182,0.34),(-0.305,0.229),(-0.305,-0.229),(-0.182,-0.34),(0.008,-0.34),(0.17,-0.229)]  # approx of X2_FOOTPRINT (side/back); front 0.17
for which in ('local','global'):
    a,r,org,fr=getcm(which)
    lethal=(a==254)
    lab,nl=ndimage.label(lethal)
    cl=[]
    for k in range(1,nl+1):
        ys,xs=np.where(lab==k)
        px=org[0]+(xs+0.5)*r;py=org[1]+(ys+0.5)*r
        if fr=='odom':  # to map
            px,py=mo[0]+math.cos(mo[2])*px-math.sin(mo[2])*py,mo[1]+math.sin(mo[2])*px+math.cos(mo[2])*py
        cl.append((len(xs),float(px.mean()),float(py.mean()),float(np.ptp(px)),float(np.ptp(py))))
    # clusters near the corridor ahead (within 6 m of the start)
    near=[c for c in cl if math.hypot(c[1]-x0,c[2]-y0)<6.0 and c[0]>=2]
    print('%s costmap (%s): %d lethal cells, %d clusters; within 6 m of the robot: %s'%(which,fr,int(lethal.sum()),nl,[(c[0],round(c[1],2),round(c[2],2),round(c[3],2),round(c[4],2)) for c in near]))
    res[which]=dict(frame=fr,res=r,clusters=near,total_lethal=int(lethal.sum()))
    if which=='global': GA,GR,GO=a,r,org
c=ActionClient(n,ComputePathToPose,'/qazbot_d1_map/nav2/compute_path_to_pose');assert c.wait_for_server(10)
def ps(x,y,w):
    p=PoseStamped();p.header.frame_id='map';p.header.stamp=n.get_clock().now().to_msg();p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.z=math.sin(w/2);p.pose.orientation.w=math.cos(w/2);return p
g=ComputePathToPose.Goal();g.start=ps(x0,y0,yw);g.goal=ps(gx,gy,yw);g.use_start=True;g.planner_id='GridBased'
f=c.send_goal_async(g)
while not f.done(): rclpy.spin_once(n,timeout_sec=0.05)
h=f.result();rf=h.get_result_async()
while not rf.done(): rclpy.spin_once(n,timeout_sec=0.05)
rr=rf.result();pts=[(p.pose.position.x,p.pose.position.y) for p in rr.result.path.poses]
print('plan status',rr.status,'poses',len(pts))
if rr.status==4 and pts:
    L=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(pts,pts[1:]))
    ux,uy=math.cos(yw),math.sin(yw);dev=max(abs(-(p[0]-x0)*uy+(p[1]-y0)*ux) for p in pts)
    # footprint clearance: distance transform from lethal cells (global costmap, map frame)
    dist=ndimage.distance_transform_edt(GA!=254)*GR
    def dval(x,y):
        i=int((x-GO[0])/GR);j=int((y-GO[1])/GR);return dist[j,i] if 0<=i<dist.shape[1] and 0<=j<dist.shape[0] else 9.9
    mn=9.9;mn_c=9.9;costmax=0
    for k,p in enumerate(pts):
        nx,ny=pts[min(k+1,len(pts)-1)];th=math.atan2(ny-p[1],nx-p[0]) if k<len(pts)-1 else yw
        cth,sth=math.cos(th),math.sin(th)
        d=min(dval(p[0]+cth*fx-sth*fy,p[1]+sth*fx+cth*fy) for fx,fy in FOOT)
        mn=min(mn,d);mn_c=min(mn_c,dval(*p))
        i=int((p[0]-GO[0])/GR);j=int((p[1]-GO[1])/GR);costmax=max(costmax,int(GA[j,i]))
    print('path %.2f m (straight %.2f), max lateral detour %.2f m, min footprint-vertex clearance to lethal %.2f m, min centre clearance %.2f m, max cost on path %d'%(L,AH,dev,mn,mn_c,costmax))
    res['plan']=dict(length=L,max_lateral=dev,min_footprint_clearance=mn,min_centre_clearance=mn_c,max_cost=costmax,path=pts)
res.update(start=[x0,y0,yw],goal=[gx,gy,yw])
json.dump(res,open(sys.argv[2] if len(sys.argv)>2 else 'boxcheck.json','w'))
