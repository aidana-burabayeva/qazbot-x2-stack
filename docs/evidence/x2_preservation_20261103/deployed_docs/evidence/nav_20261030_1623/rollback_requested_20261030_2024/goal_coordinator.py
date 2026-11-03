#!/usr/bin/env python3
"""Action ownership and command gate for the single Nav2 navigator; never imports a vendor SDK.

Public NavigateToPose forwards to the private Nav2 action. A fault closes the
command gate before cancellation. Resume requires terminal acknowledgement and
a final zero observed from the sole smoother publisher. No cached goal replay.
Exception (operator decision 2026-10-28): T1-only freshness loss (trust status t1_transient, READY kept) while
busy PAUSES: gate closed + zero, the Nav2 goal is NOT cancelled, and the gate reopens when trust returns. Loss
longer than t1_pause_max_s (default 3.0 s), or any other untrusted state, trips (cancel + latch) as before.
This node is coordination, not a second planner/controller/navigator.
"""
import json
import math
import time
import signal
from rclpy.signals import SignalHandlerOptions
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import TransitionEvent
from rclpy.task import Future
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from goal_alignment_watch import GoalAlignmentWatch

T1_PAUSE_MAX_S=3.0   # operator decision 2026-10-28: T1-only loss longer than this cancels the goal (param t1_pause_max_s)
QUALITY_PAUSE_MAX_S=10.0   # P26 2026-10-30: trust quality_pause (Q_localization_quality, transient) bound (param quality_pause_max_s)

class GoalCoordinator(Node):
    def __init__(self):
        super().__init__('goal_coordinator')
        self.busy=False;self.reserving=False;self.latched=False;self.fault='';self.child=None;self.public=None
        self.gate=False;self.cancel_sent=False;self.zero_seen=True;self.ever_command=False
        self.trusted=False;self.trust_at=float('-inf');self.command_gid=None
        self.driver_ready=False;self.driver_mode='unknown'
        self.driver_status={};self.driver_status_at=float('-inf')   # diagnostics only (rejection log)
        # Explicit dry-run driver path: a dry-run adapter (no MC publisher, no registration) may own a goal
        # only when this is set. It never grants MC access; the adapter itself has no MC output in dry run.
        self.allow_dry_run_driver=bool(self.declare_parameter('allow_dry_run_driver',False).value)
        self.trust_max_age=float(self.declare_parameter('trust_max_age_s',0.6).value)
        self.t1_pause_max=float(self.declare_parameter('t1_pause_max_s',T1_PAUSE_MAX_S).value)
        self.quality_pause_max=float(self.declare_parameter('quality_pause_max_s',QUALITY_PAUSE_MAX_S).value)
        self.trust_q=False   # P26: fresh transient status is a quality pause (own bound)
        self.pause_since=None;self.trust_t1=False;self.trust_status_at=float('-inf');self.trust_reasons=[]
        self.alignment_watch=GoalAlignmentWatch(timeout=float(self.declare_parameter('alignment_stagnation_s',30.0).value))
        self.execution_failure=''
        self.pub=self.create_publisher(Twist,'cmd_vel_authorized',1)
        self.status=self.create_publisher(String,'~/status',10)
        self.group=ReentrantCallbackGroup()
        self.arm=self.create_client(Trigger,'localization_trust/arm_check',callback_group=self.group)
        self.driver_arm=self.create_client(Trigger,'motion_adapter/arm_motion',callback_group=self.group)
        self.life=self.create_client(GetState,'nav2/bt_navigator/get_state',callback_group=self.group)
        self.trust_waiter=None
        self.nav_active=False;self.nav_state_future=None;self.nav_transition_seen=False
        self.create_subscription(TransitionEvent,'nav2/bt_navigator/transition_event',self.transition,10)
        self.client=ActionClient(self,NavigateToPose,'nav2/navigate_to_pose',callback_group=self.group)
        self.pose_client=ActionClient(self,NavigateToPose,'navigate_to_pose',callback_group=self.group)
        self.create_subscription(PoseStamped,'goal_pose',self.pose_goal,1)
        self.server=ActionServer(self,NavigateToPose,'navigate_to_pose',self.execute,
            goal_callback=self.goal,cancel_callback=self.cancel,callback_group=self.group)
        self.create_subscription(Twist,'cmd_vel',self.command,1)
        self.create_subscription(Bool,'localization_trust/trusted',self.trust,1)
        self.create_subscription(String,'localization_trust/status',self.trust_status,1)
        self.create_subscription(String,'motion_adapter/status',self.adapter,1)
        self.create_service(Trigger,'~/stop',self.stop)
        self.create_service(Trigger,'~/resume',self.resume)
        self.create_timer(0.05,self.check)
        self.create_timer(0.2,self.report)
    def transition(self,event):
        self.nav_transition_seen=True
        self.nav_active=event.goal_state.id==3
        if self.busy and not self.nav_active:self.trip('nav2_inactive')
    def state_observed(self,future):
        if not self.nav_transition_seen:self.nav_active=future.result().current_state.id==3
    def pose_goal(self,pose):
        goal=NavigateToPose.Goal();goal.pose=pose
        if not self.pose_client.server_is_ready():
            self.get_logger().error('Goal Pose action interface not ready');return
        future=self.pose_client.send_goal_async(goal)
        future.add_done_callback(lambda f:self.get_logger().info('Goal Pose accepted' if f.result().accepted else 'Goal Pose rejected by readiness policy'))
    def good(self):return self.trusted and time.monotonic()-self.trust_at <= self.trust_max_age
    def trust(self,m):
        self.trusted=bool(m.data);self.trust_at=time.monotonic()
        if self.trust_waiter is not None and not self.trust_waiter.done():self.trust_waiter.set_result(bool(m.data))
        if not m.data and self.busy and not self.latched:self.pause()   # classified (pause vs trip) by trust_status()/check()
    def trust_status(self,m):
        d=json.loads(m.data);self.trust_t1=bool(d.get('t1_transient')) and not d.get('trusted');self.trust_status_at=time.monotonic()
        self.trust_q=self.trust_t1 and bool(d.get('quality_pause'))
        self.trust_reasons=list(d.get('reasons') or [])   # diagnostics only (trip log)
        if self.busy and not d.get('trusted') and not self.trust_t1:self.trip('localization_untrusted')
    def pause(self):
        # T1-only loss: stop forwarding (zero now) but keep the Nav2 goal; check() reopens or trips.
        if self.pause_since is None:
            self.pause_since=time.monotonic();self.gate=False;self.pub.publish(Twist())
            self.get_logger().warn('trust lost while busy: gate paused (goal kept pending T1 classification)')
    def hold(self):
        # busy and not good(): pause; trip unless a FRESH trust status says T1-only, and trip if it lasts too long.
        self.pause();now=time.monotonic()
        if self.trust_t1 and now-self.trust_status_at<=self.trust_max_age:
            if self.trust_q:   # P26 quality pause: own (longer) bound
                if now-self.pause_since>self.quality_pause_max:self.trip('localization_quality_prolonged')
            elif now-self.pause_since>self.t1_pause_max:self.trip('localization_untrusted_t1_prolonged')
        elif now-self.trust_status_at>self.trust_max_age or now-self.pause_since>self.trust_max_age:self.trip('localization_untrusted')
    def adapter(self,m):
        d=json.loads(m.data)
        live_ok=bool(d.get('live') is True and d.get('registration')=='registered' and d.get('ready'))
        dry_ok=bool(self.allow_dry_run_driver and d.get('live') is False and d.get('mode')=='dry_run'
                    and d.get('registration')=='dry_run' and d.get('ready'))
        self.driver_ready=live_ok or dry_ok
        self.driver_mode='live' if d.get('live') is True else str(d.get('mode','unknown'))
        self.driver_status={k:d.get(k) for k in ('state','reason','latch','ready','registration','motion_armed')}
        self.driver_status_at=time.monotonic()
        if self.busy and not d.get('t1_pause') and (not self.driver_ready or d.get('reason')=='driver_not_armed'):
            self.trip('driver_not_ready')
        hard={'operator_takeover','control_not_acquired','robot_not_standing','mc_fsm_not_stable',
              'mc_state_stale','foreign_sdk_source_active','foreign_velocity_publisher','vendor_autonomy_active',
              'localization_untrusted','localization_lost','stop_requested','non_finite_command'}
        if self.busy and (d.get('state')=='latched' or d.get('reason') in hard):
            self.trip(d.get('latch') or d.get('reason') or 'adapter_latched')
    def trip(self,reason):
        if not self.latched:   # P10: log WHY (was silent; only the pause warning showed, which looked like a lost T1 pause)
            self.get_logger().error(f'TRIP {reason}: busy={self.busy} trust_t1={self.trust_t1} '
                f'trust_reasons={getattr(self,"trust_reasons",[])} -> gate closed, zero, cancel requested, latched')
        self.gate=False;self.latched=True;self.fault=reason
        self.pub.publish(Twist())
        self.request_cancel()
    def request_cancel(self):
        if self.child is not None and not self.cancel_sent:
            self.cancel_sent=True;self.cancel_future=self.child.cancel_goal_async()
    def command(self,m):
        # Humble rclpy here supplies only the message, not MessageInfo. This graph
        # check is diagnostic; it is not an atomic hardware ownership guarantee.
        endpoints=self.get_publishers_info_by_topic(self.resolve_topic_name('cmd_vel'))
        if len(endpoints)!=1:
            self.trip('command_publisher_count');return
        gid=bytes(endpoints[0].endpoint_gid)
        if self.command_gid is None:self.command_gid=gid
        elif gid!=self.command_gid:
            self.trip('command_publisher_changed');return
        vals=[m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z]
        if not all(math.isfinite(x) for x in vals):self.trip('non_finite_command');return
        self.ever_command=True;self.zero_seen=all(abs(x)<1e-12 for x in vals)
        if self.gate and self.busy and self.good() and not self.latched:self.pub.publish(m)
    def reject(self,why):
        # Diagnostics only: every admission term at the moment of rejection (no behaviour change).
        now=time.monotonic()
        self.get_logger().warn(f'Goal rejected ({why}): busy={self.busy} reserving={self.reserving} latched={self.latched} '
            f'fault={self.fault!r} trust={self.good()} trusted_raw={self.trusted} trust_age_s={now-self.trust_at:.3f} '
            f'zero={self.zero_seen} driver_ready={self.driver_ready} driver_mode={self.driver_mode} '
            f'driver_status={self.driver_status} driver_status_age_s={now-self.driver_status_at:.3f} '
            f'driver_arm_svc={self.driver_arm.service_is_ready()} trust_arm_svc={self.arm.service_is_ready()} '
            f'lifecycle_svc={self.life.service_is_ready()} nav_active={self.nav_active} nav2_ready={self.client.server_is_ready()}')
        return GoalResponse.REJECT
    async def goal(self,req):
        if not self.good() and self.arm.service_is_ready():
            # Tell localization_trust a NEW goal was requested during input loss: loss still present at a goal
            # request is "prolonged" and clears READY there (operator relocalization). Not awaited; rejected anyway.
            self.arm.call_async(Trigger.Request())
        if self.busy or self.reserving or self.latched or not self.good() or not self.zero_seen or not self.driver_ready or not self.driver_arm.service_is_ready() or not self.client.server_is_ready():
            return self.reject('precondition')
        if req.pose.header.frame_id!='map':return self.reject('frame_id='+repr(req.pose.header.frame_id))
        p,q=req.pose.pose.position,req.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)):return self.reject('non_finite_pose')
        if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>1e-3:return self.reject('quaternion_not_normalized')
        # The selected reviewed BT is mandatory; arbitrary trees could bypass execution policy.
        if req.behavior_tree:return self.reject('custom_behavior_tree')
        if not self.arm.service_is_ready() or not self.life.service_is_ready():return self.reject('trust_arm_or_lifecycle_service')
        self.reserving=True
        try:
            state=await self.life.call_async(GetState.Request())
            if state.current_state.id!=3:return self.reject(f'bt_navigator_state={state.current_state.id}')
            verdict=await self.arm.call_async(Trigger.Request())
            if not verdict.success:
                return self.reject('trust_arm_check: '+verdict.message)
            driver=await self.driver_arm.call_async(Trigger.Request())
            if not driver.success:
                return self.reject('driver_arm: '+driver.message)
            # Arm may be computationally expensive. Wait for a NEW trust observation
            # after it returns, rather than carrying a pre-check verdict into execution.
            self.trust_waiter=Future()
            clean=await self.trust_waiter
            if not clean or self.latched or not self.good() or not self.driver_ready:return self.reject(f'post_arm clean={clean}')
            self.busy=True;self.gate=False;self.cancel_sent=False;self.child=None;self.pause_since=None
            self.execution_failure='';self.fault='';self.alignment_watch.reset()
            return GoalResponse.ACCEPT
        finally:
            self.reserving=False;self.trust_waiter=None
    def cancel(self,h):
        self.trip('goal_cancelled')
        return CancelResponse.ACCEPT
    async def execute(self,h):
        self.public=h
        try:
            self.child=await self.client.send_goal_async(h.request,
                feedback_callback=lambda f:self.navigation_feedback(h,f.feedback))
            if not self.child.accepted:
                h.abort();return NavigateToPose.Result()
            self.gate=not self.latched and self.good() and self.driver_ready and self.pause_since is None
            if not self.gate and (self.latched or self.pause_since is None):self.trip(self.fault or 'localization_untrusted')
            result=await self.child.get_result_async()
            self.gate=False;self.pub.publish(Twist())
            if h.is_cancel_requested:h.canceled()
            elif self.latched or self.execution_failure:h.abort()
            elif result.status==GoalStatus.STATUS_SUCCEEDED:h.succeed()
            else:h.abort()
            return result.result
        except Exception as e:
            self.trip('action_transport_error');self.get_logger().error(str(e))
            if h.is_active:h.abort()
            return NavigateToPose.Result()
        finally:
            self.gate=False;self.busy=False;self.child=None;self.public=None;self.pause_since=None
    def check(self):
        if self.busy and not self.good() and not self.latched:self.hold()
        elif self.busy and self.pause_since is not None and self.good() and not self.latched and self.driver_ready:
            # Fresh trust is back within t1_pause_max_s: continue the SAME goal (controller never stopped computing).
            self.get_logger().info(f'trust back after {time.monotonic()-self.pause_since:.2f} s: gate reopened, same goal')
            self.pause_since=None;self.gate=self.child is not None
        if self.busy and self.alignment_watch.expired(time.monotonic()) and not self.latched:
            # Execution failure, not a localization/admission latch. A NEW goal is
            # allowed after the old action is terminal, zero and readiness checks.
            self.execution_failure=self.fault='goal_alignment_not_converging'
            self.gate=False;self.pub.publish(Twist());self.request_cancel()
    def navigation_feedback(self,h,feedback):
        if not h.is_active:return
        h.publish_feedback(feedback)
        current=feedback.current_pose
        if current.header.frame_id!='map':return
        def pose(p):
            q=p.orientation
            return (p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        actual,target=pose(current.pose),pose(h.request.pose.pose)
        if all(math.isfinite(x) for x in (*actual,*target)):
            self.alignment_watch.observe(time.monotonic(),actual,target)
    def stop(self,q,r):
        self.trip('operator_stop');r.success=True;r.message='gate closed; cancellation requested';return r
    def resume(self,q,r):
        r.success=bool(self.latched and not self.busy and not self.reserving and self.zero_seen and self.good())
        if r.success:self.latched=False;self.fault='';r.message='ready for a NEW goal; no cached command/goal replay'
        else:r.message='requires terminal old action, final source zero and current trust'
        return r
    def report(self):
        if self.nav_state_future is None and self.life.service_is_ready():
            self.nav_state_future=self.life.call_async(GetState.Request())
            self.nav_state_future.add_done_callback(self.state_observed)
        self.status.publish(String(data=json.dumps(dict(busy=self.busy,latched=self.latched,gate=self.gate,
            fault=self.fault,source_zero=self.zero_seen,trusted=self.good(),paused=self.pause_since is not None,
            driver_ready=self.driver_ready,driver_mode=self.driver_mode,
            ready=bool(self.driver_ready and self.driver_arm.service_is_ready() and self.nav_active and not self.busy and not self.reserving and not self.latched and self.good() and self.zero_seen and self.client.server_is_ready() and self.arm.service_is_ready() and self.life.service_is_ready()),
            child_uuid=[int(x) for x in self.child.goal_id.uuid] if self.child else None))))

def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    shutting_down=[False]
    def terminate(signum,frame):
        shutting_down[0]=True
    signal.signal(signal.SIGINT,terminate)
    signal.signal(signal.SIGTERM,terminate)
    node=GoalCoordinator()
    try:
        while not shutting_down[0] and rclpy.ok():
            rclpy.spin_once(node,timeout_sec=0.05)
    except KeyboardInterrupt:pass
    finally:
        shutting_down[0]=True
        node.gate=False
        node.pub.publish(Twist())
        node.server.destroy();node.client.destroy();node.pose_client.destroy()
        node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
