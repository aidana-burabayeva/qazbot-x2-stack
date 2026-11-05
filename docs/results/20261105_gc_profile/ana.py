import rosbag2_py,math,json,sys,bisect,collections
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
B=sys.argv[1]
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=B,storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
ty={t.name:t.type for t in r.get_all_topics_and_types()}
want=['/qazbot_d1_map/nav2/cmd_vel_nav','/qazbot_d1_map/cmd_vel','/qazbot_d1_map/cmd_vel_authorized','/aima/mc/locomotion/velocity','/aima/mc/common/state','/qazbot_lio/odom_level','/aima/mc/leg_odometry','/qazbot_d1_map/motion_adapter/status','/qazbot_d1_map/tf','/qazbot_d1_map/localization_trust/status','/qazbot_d1_map/goal_coordinator/status','/qazbot_d1_map/nav2/navigate_to_pose/_action/feedback','/rosout']
cls={t:get_message(ty[t]) for t in want if t in ty}
D=collections.defaultdict(list)
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
t0=None
while r.has_next():
    tp,data,ts=r.read_next();t=ts*1e-9
    if tp not in cls: continue
    m=deserialize_message(data,cls[tp])
    if tp.endswith('cmd_vel_nav') or tp.endswith('/cmd_vel') or tp.endswith('cmd_vel_authorized'): D[tp].append((t,m.linear.x,m.angular.z))
    elif tp=='/aima/mc/locomotion/velocity': D[tp].append((t,m.source,m.forward_velocity,m.angular_velocity))
    elif tp=='/aima/mc/common/state': D[tp].append((t,m.input_source.name,m.body_status.value,m.fsm_state.current_state,m.speed_status.speed_mode.value))
    elif tp in('/qazbot_lio/odom_level','/aima/mc/leg_odometry'): D[tp].append((t,m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation),m.twist.twist.linear.x,m.twist.twist.angular.z))
    elif tp.endswith('motion_adapter/status'):
        j=json.loads(m.data);D[tp].append((t,j['state'],j['reason'],j['mc_source'],j['motion_armed'],j['counters']['yaw_loop'],j['counters']['yaw_meas_stale'],j['counters']['published'],j['counters']['saturated'],j['latch']))
    elif tp=='/qazbot_d1_map/tf':
        for x in m.transforms:
            k=(x.header.frame_id,x.child_frame_id)
            if k in(('map','odom'),('odom','base_footprint')): D['tf_'+k[0]+'_'+k[1]].append((t,x.transform.translation.x,x.transform.translation.y,yaw(x.transform.rotation)))
    elif tp.endswith('goal_coordinator/status'): j=json.loads(m.data);D[tp].append((t,j['busy'],j['gate'],j['fault'],j['latched']))
    elif tp=='/rosout': 
        if 'collision' in m.msg or 'Goal' in m.msg or 'progress' in m.msg: D[tp].append((t,m.name,m.msg[:100]))
import pickle;pickle.dump(dict(D),open(sys.argv[2],'wb'))
for k,v in D.items(): print(k,len(v),v[0][0] if v else '')
