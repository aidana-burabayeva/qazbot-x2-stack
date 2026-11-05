#!/usr/bin/env python3
'''Offline reproduction using Nav2 C++ and generated YAML; starts no ROS nodes.'''
import argparse, importlib.util, json, os, subprocess
from pathlib import Path
import yaml
from launch import LaunchContext
p=argparse.ArgumentParser();p.add_argument('result_dir',type=Path);a=p.parse_args()
root=Path(__file__).resolve().parents[2];out=a.result_dir.resolve();out.mkdir(parents=True,exist_ok=False)
os.environ['ROS_LOG_DIR']=str(out)
spec=importlib.util.spec_from_file_location('x2_control',root/'ws/src/navigation/nav_control.launch.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
c=LaunchContext();c.launch_configurations.update(dict(ns='qazbot_d1_map/nav2',robot_radius='0.454',max_linear='0.3',max_angular='0.5',scan_topic='/qazbot_lio/scan',odom_topic='/qazbot_lio/odom_level',output_topic='/qazbot_d1_map/cmd_vel',xy_tol='0.10',yaw_tol='0.10'))
mod.setup(c,raw=True)
y=yaml.safe_load(next(out.glob('qazbot_ctl_*.yaml')).read_text())
r=y['/qazbot_d1_map/nav2/controller_server']['ros__parameters'];s=y['/qazbot_d1_map/nav2/velocity_smoother']['ros__parameters']
assert s['feedback']=='OPEN_LOOP'
assert s['max_velocity']==[.3,0.,.5] and s['min_velocity']==[0.,0.,-.5]
assert s['max_accel']==[.4,0.,1.] and s['max_decel']==[-.6,0.,-1.5]
assert r['FollowPath.use_collision_detection'] and not r['FollowPath.allow_reversing']
base=root/'candidates/nav2_smac_bounds/src/navigation2-1.1.18'
def method(path,name):
 text=path.read_text();start=text.index(name);begin=text.index('{',start);depth=1;end=begin+1
 while depth:
  depth+=(text[end]=='{')-(text[end]=='}');end+=1
 return text[start:end]
rpp=method(base/'nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp','void RegulatedPurePursuitController::rotateToHeading(')
smooth=method(base/'nav2_velocity_smoother/src/velocity_smoother.cpp','double VelocitySmoother::applyConstraints(')
cpp='''#include <algorithm>
#include <cmath>
#include <iostream>
using std::abs;
namespace geometry_msgs {namespace msg {struct Twist {struct V {double z=0;}; V angular;};}}
class RegulatedPurePursuitController {public: double rotate_to_heading_angular_vel_,max_angular_accel_,control_duration_; void rotateToHeading(double&,double&,const double&,const geometry_msgs::msg::Twist&);};
class VelocitySmoother {public: double smoothing_frequency_; double applyConstraints(double,double,double,double,double);};
'''+rpp+'\n'+smooth+f'''
int main() {{
 RegulatedPurePursuitController r; r.rotate_to_heading_angular_vel_={r['FollowPath.rotate_to_heading_angular_vel']};r.max_angular_accel_={r['FollowPath.max_angular_accel']};r.control_duration_=1./{r['controller_frequency']};
 VelocitySmoother sm; sm.smoothing_frequency_={s['smoothing_frequency']};
 bool start=true, ramp=true, stop=true; double first=0;
 for(double sign: {{-1.,1.}}) {{
  geometry_msgs::msg::Twist measured; double v=1,w=0; r.rotateToHeading(v,w,sign,measured);
  first=std::abs(w);start=start && v==0 && std::abs(w-sign*r.rotate_to_heading_angular_vel_)<1e-9;
  double actual_cmd=0;
  for(int k=0;k<10;k++) {{double prev=actual_cmd;actual_cmd=sm.applyConstraints(actual_cmd,w,1.,-1.5,1.);ramp=ramp && std::abs(actual_cmd-prev)<=.050000001 && std::abs(actual_cmd)<=.500000001;}}
  ramp=ramp && std::abs(actual_cmd-sign*.5)<1e-9;
  for(int k=0;k<7;k++) {{double prev=actual_cmd;actual_cmd=sm.applyConstraints(actual_cmd,0.,1.,-1.5,1.);stop=stop && std::abs(actual_cmd-prev)<=.075000001;}}
  stop=stop && std::abs(actual_cmd)<1e-9;
 }}
 std::cout<<"RPP first magnitude="<<first<<" reaches requested cap="<<start<<" smoother ramp="<<ramp<<" stop="<<stop<<"\\n";
 return start && ramp && stop ? 0:1;
}}
'''
(out/'turn_start.cpp').write_text(cpp)
subprocess.run(['g++','-std=c++17','-O0',str(out/'turn_start.cpp'),'-o',str(out/'turn_start')],check=True)
run=subprocess.run([str(out/'turn_start')],text=True,capture_output=True)
result=dict(passed=run.returncode==0,output=run.stdout,controller_frequency=r['controller_frequency'],rpp_max_angular_accel=r['FollowPath.max_angular_accel'],rpp_angular_cap=r['FollowPath.rotate_to_heading_angular_vel'],smoother_angular_accel=s['max_accel'][2],smoother_angular_decel=s['max_decel'][2],nodes_launched=False)
(out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));raise SystemExit(run.returncode)
