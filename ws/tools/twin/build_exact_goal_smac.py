#!/usr/bin/env python3
"""SIM-only isolated Smac build: retain requested XY without changing bounds checks."""
import difflib,hashlib,json,os,shutil,subprocess
from pathlib import Path
if any(os.environ.get(k)!=v for k,v in {'QAZBOT_SIM':'1','ROS_DOMAIN_ID':'87','ROS_LOCALHOST_ONLY':'1'}.items()):
    raise RuntimeError('isolated simulation only')
ROOT=Path(__file__).resolve().parents[3]
source=ROOT/'runtime_x86/build_ws/src/smac_only/nav2_smac_planner'
workspace=ROOT/'runtime_x86/exact_goal_ws';dest=workspace/'src/nav2_smac_planner'
relative=Path('src/smac_planner_2d.cpp');before=(source/relative).read_text()
old='    pose.pose = start.pose;'
assert before.count(old)==1
after=before.replace(old,old+'\n    pose.pose.position.x = goal.pose.position.x;\n    pose.pose.position.y = goal.pose.position.y;',1)
anchor='''  return plan;
}

rcl_interfaces::msg::SetParametersResult'''
assert after.count(anchor)==1
addition='''  // Preserve the requested sub-cell endpoint. The extra segment stays entirely
  // inside the already reached goal cell; do not append to tolerance fallbacks.
  if (!plan.poses.empty()) {
    const auto &last = plan.poses.back().pose.position;
    unsigned int end_x, end_y;
    if (costmap->worldToMap(last.x, last.y, end_x, end_y) &&
        end_x == mx_goal && end_y == my_goal &&
        std::hypot(last.x-goal.pose.position.x, last.y-goal.pose.position.y) > 1e-9) {
      auto exact = plan.poses.back();
      exact.pose.position.x = goal.pose.position.x;
      exact.pose.position.y = goal.pose.position.y;
      plan.poses.push_back(exact);
    }
  }

'''
after=after.replace(anchor,addition+anchor,1)
patch=''.join(difflib.unified_diff(before.splitlines(True),after.splitlines(True),fromfile='a/nav2_smac_planner/'+str(relative),tofile='b/nav2_smac_planner/'+str(relative)))
patch_file=ROOT/'candidates/nav2_exact_goal/goal_endpoint.patch';patch_file.parent.mkdir(exist_ok=True)
if dest.exists():
    if (dest/relative).read_text()!=after:raise RuntimeError('generated source changed: inspect before rebuilding')
else:
    dest.parent.mkdir(parents=True,exist_ok=True);shutil.copytree(source,dest);(dest/relative).write_text(after)
patch_file.write_text(patch)
subprocess.run(['colcon','build','--packages-select','nav2_smac_planner','--cmake-args','-DCMAKE_BUILD_TYPE=Release','-DBUILD_TESTING=OFF'],cwd=workspace,env=dict(os.environ,CMAKE_BUILD_PARALLEL_LEVEL='2'),check=True)
prefix=workspace/'install/nav2_smac_planner'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
manifest=dict(library_sha256=sha(prefix/'lib/libnav2_smac_planner_2d.so'),
              patch_sha256=sha(ROOT/'candidates/nav2_smac_bounds/bounds.patch'),goal_endpoint_patch_sha256=sha(patch_file),
              baseline_source_sha256=hashlib.sha256(before.encode()).hexdigest(),
              base='navigation2 1.1.20 + existing bounds patch + exact goal (simulation candidate)')
(prefix/'qazbot_bounds_fix.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps(manifest,indent=2))
