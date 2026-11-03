import rclpy,math,time,collections
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PolygonStamped
rclpy.init();n=Node('orch_lc');d={}
q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(OccupancyGrid,'/qazbot_d1_map/nav2/local_costmap/costmap',lambda m:d.__setitem__('c',m),q)
n.create_subscription(PolygonStamped,'/qazbot_d1_map/nav2/local_costmap/published_footprint',lambda m:d.__setitem__('f',m),10)
t=time.time()
while time.time()-t<8 and not('c' in d and 'f' in d): rclpy.spin_once(n,timeout_sec=0.2)
c=d['c'];f=d['f'];I=c.info
xs=[p.x for p in f.polygon.points];ys=[p.y for p in f.polygon.points];cx=sum(xs)/len(xs);cy=sum(ys)/len(ys)
h=collections.Counter()
for j in range(I.height):
  for i in range(I.width):
    if c.data[j*I.width+i]>=100:
      x=I.origin.position.x+(i+.5)*I.resolution;y=I.origin.position.y+(j+.5)*I.resolution;r=math.hypot(x-cx,y-cy)
      if r<1.5: h[(int(math.degrees(math.atan2(y-cy,x-cx))//45)*45, round(r*4)/4)]+=1
print('LETHAL(100) cells by odom-bearing/range:',sorted(h.items()))
