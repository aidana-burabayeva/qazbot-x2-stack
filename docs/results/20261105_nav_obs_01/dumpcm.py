import rclpy,time,numpy as np,sys
from rclpy.node import Node
from nav2_msgs.srv import GetCostmap
rclpy.init();n=Node('dc')
out={}
for which in ('global','local'):
    c=n.create_client(GetCostmap,'/qazbot_d1_map/nav2/%s_costmap/get_costmap'%which);assert c.wait_for_service(5)
    f=c.call_async(GetCostmap.Request())
    while not f.done(): rclpy.spin_once(n,timeout_sec=0.05)
    r=f.result().map;md=r.metadata
    out[which+'_data']=np.array(r.data,dtype=np.uint8).reshape(md.size_y,md.size_x);out[which+'_res']=md.resolution;out[which+'_org']=np.array([md.origin.position.x,md.origin.position.y]);out[which+'_frame']=r.header.frame_id
np.savez(sys.argv[1],**out);print('saved',{k:(v.shape if hasattr(v,'shape') else v) for k,v in out.items()})
