"""Validate source time independently of callback arrival (no extrapolation)."""
import math

class SourceStamp:
    def __init__(self):self.last = None
    def accept(self, stamp_s, ros_now_s, mono_now_s, max_future_s=0.0):
        valid = (math.isfinite(stamp_s) and stamp_s > 0 and
                 (self.last is None or stamp_s > self.last) and
                 stamp_s <= ros_now_s + max_future_s)
        if not valid:return float('-inf')
        self.last = stamp_s
        return mono_now_s - max(0.0, ros_now_s - stamp_s)
