"""Bound final alignment that keeps rotating without approaching the requested pose.

Uses estimated navigation feedback only. It neither changes localization readiness
nor accepts a goal outside its tolerance. The coordinator aborts the current task.
"""
import math


class GoalAlignmentWatch:
    def __init__(self, timeout=30., radius=.30, improvement=.01, yaw_weight=.10):
        if not all(math.isfinite(x) and x>0 for x in (timeout,radius,improvement,yaw_weight)):
            raise ValueError('alignment watch parameters must be finite and positive')
        self.timeout,self.radius,self.improvement,self.yaw_weight=timeout,radius,improvement,yaw_weight
        self.reset()

    def reset(self):
        self._state=(None,None)

    @property
    def best(self):
        return self._state[0]

    @property
    def improved_at(self):
        return self._state[1]

    def observe(self,now,pose,goal):
        distance=math.hypot(pose[0]-goal[0],pose[1]-goal[1])
        if distance>self.radius:
            self.reset();return
        angle=abs(math.atan2(math.sin(pose[2]-goal[2]),math.cos(pose[2]-goal[2])))
        score=distance+self.yaw_weight*angle
        best,_=self._state
        if best is None or score<best-self.improvement:
            self._state=(score,now)

    def expired(self,now):
        _,at=self._state
        return at is not None and now-at>self.timeout
