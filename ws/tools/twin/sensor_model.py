"""Pure geometry/timing for the .66 observation approximation; no MC dynamics."""
from collections import deque
import math
import numpy as np


def wrap(a):
    return math.atan2(math.sin(a),math.cos(a))


def observe(pelvis, pelvis_lidar, imu_from_lidar):
    lidar=pelvis@pelvis_lidar
    imu=lidar@np.linalg.inv(imu_from_lidar)
    return np.array([imu[0,3],imu[1,3],math.atan2(lidar[1,0],lidar[0,0])])


class TwistWindow:
    def __init__(self, baseline):
        self.baseline=baseline;self.rows=deque()

    def add(self,t,pose):
        if self.rows and t<=self.rows[-1][0]:
            if t==self.rows[-1][0]:return None
            self.rows.clear()
        self.rows.append((t,*pose))
        while len(self.rows)>1 and t-self.rows[1][0]>=self.baseline:self.rows.popleft()
        t0,x,y,yaw=self.rows[0];dt=t-t0
        if dt<self.baseline:return None
        dx,dy=pose[0]-x,pose[1]-y;c,s=math.cos(pose[2]),math.sin(pose[2])
        return np.array([(c*dx+s*dy)/dt,(-s*dx+c*dy)/dt,wrap(pose[2]-yaw)/dt])


class WedgeRays:
    def __init__(self,profile):
        g,m=profile['scan'],profile['model'];self.bins=g['bins']
        angles=g['angle_min']+np.arange(self.bins)*g['angle_increment']
        relative=np.arctan2(np.sin(angles-math.radians(m['fov_center_deg'])),np.cos(angles-math.radians(m['fov_center_deg'])))
        self.indices=np.flatnonzero(abs(relative)<=math.radians(m['fov_deg']/2))
        self.ranges=np.arange(g['range_min'],g['range_max']+1e-8,m['ray_step_m'])
        a=angles[self.indices];self.xy=np.stack([np.outer(np.cos(a),self.ranges),np.outer(np.sin(a),self.ranges)],-1)

    def cast(self,pose,distance_at):
        c,s=math.cos(pose[2]),math.sin(pose[2]);xy=self.xy.reshape(-1,2)
        world=xy@np.array([[c,s],[-s,c]])+np.asarray(pose[:2])
        hit=(distance_at(world)<=.025).reshape(len(self.indices),len(self.ranges))
        out=np.full(self.bins,np.inf)
        valid=hit.any(1);out[self.indices[valid]]=self.ranges[hit[valid].argmax(1)]
        return out
