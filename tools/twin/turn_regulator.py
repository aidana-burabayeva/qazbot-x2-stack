"""Experimental angle controller for the official MC SIM, using actual yaw feedback.
Drive above the observed gait-initiation threshold, then zero and observe a full
stand transition before deciding on a correction. No assumption cmd==actual rate.
"""
import math


def wrap(x):return math.atan2(math.sin(x),math.cos(x))


class TurnRegulator:
    def __init__(self,target,speed=.8,tolerance=.06,lead_time=.16,settle_time=3.0,max_attempts=4):
        if not (0<speed<=1 and tolerance>0 and lead_time>=0 and settle_time>=1):raise ValueError('invalid turn bounds')
        self.target=target;self.speed=speed;self.tolerance=tolerance;self.lead_time=lead_time
        self.settle_time=settle_time;self.max_attempts=max_attempts;self.phase='drive';self.attempts=1
        self.zero_at=None;self.done=False;self.failed=False

    def update(self,t,angle,rate):
        error=self.target-angle
        if self.done or self.failed:return 0.
        if self.phase=='drive':
            direction=1 if error>=0 else -1
            # Stop before the target using measured motion, not command integration.
            stopping=max(0,direction*rate)*self.lead_time+self.tolerance*.5
            if abs(error)<=max(self.tolerance,stopping):
                self.phase='settle';self.zero_at=t;return 0.
            return direction*self.speed
        if t-self.zero_at<self.settle_time or abs(rate)>.05:return 0.
        if abs(error)<=self.tolerance:self.done=True;return 0.
        if self.attempts>=self.max_attempts:self.failed=True;return 0.
        self.attempts+=1;self.phase='drive'
        return math.copysign(self.speed,error)
