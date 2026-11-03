# Per goal: print goal pose; 3 s after robot stops near goal (or every 10 s while active) print xy/yaw error from AMCL.
import rclpy,math,time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
rclpy.init();n=Node('orch_goal_watch');st={'g':None,'p':None,'t0':0,'last':0,'done':True}
def g(m):
    st.update(g=(m.pose.position.x,m.pose.position.y,yaw(m.pose.orientation)),t0=time.time(),done=False,last=time.time())
    print('GOAL x=%.2f y=%.2f yaw=%.0fdeg'%(st['g'][0],st['g'][1],math.degrees(st['g'][2])),flush=True)
def p(m): st['p']=(m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation),time.time())
n.create_subscription(PoseStamped,'/qazbot_d1_map/goal_pose',g,10)
n.create_subscription(PoseWithCovarianceStamped,'/qazbot_d1_map/amcl_pose',p,10)
hist=[]
while rclpy.ok():
    rclpy.spin_once(n,timeout_sec=0.2)
    if st['g'] is None or st['p'] is None or st['done']: continue
    gx,gy,gyaw=st['g'];x,y,yw,tp=st['p'];d=math.hypot(gx-x,gy-y);e=math.degrees(math.atan2(math.sin(gyaw-yw),math.cos(gyaw-yw)))
    hist.append((time.time(),x,y,yw))
    hist=[h for h in hist if time.time()-h[0]<4]
    still=len(hist)>5 and max(abs(math.degrees(h[3]-hist[-1][3])) for h in hist)<2 and max(math.hypot(h[1]-x,h[2]-y) for h in hist)<0.05
    if d<0.35 and still:
        print('FINAL t=%.1fs dist=%.2fm yaw_err=%.1fdeg (goal %.0f, robot %.0f)'%(time.time()-st['t0'],d,e,math.degrees(gyaw),math.degrees(yw)),flush=True);st['done']=True
    elif time.time()-st['t0']>90:
        print('STOP watching (90 s) dist=%.2fm yaw_err=%.0fdeg'%(d,e),flush=True);st['done']=True
    elif time.time()-st['last']>10:
        st['last']=time.time();print('  t=%.0fs dist=%.2fm yaw_err=%.0fdeg'%(time.time()-st['t0'],d,e),flush=True)
