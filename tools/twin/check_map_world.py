#!/usr/bin/env python3
"""Compile generated MJCF and compare MuJoCo wall rays with the original map grid.
Run using runtime_x86/twin_tools_venv/bin/python (official MuJoCo version).
"""
import json
import math
from pathlib import Path
import sys
import cv2
import mujoco
import numpy as np
import yaml

root=Path(sys.argv[1]).resolve();meta=json.loads((root/'manifest.json').read_text())
# Manifest host paths are deliberate: this validator runs on the host.
y=yaml.safe_load(Path(meta['map_yaml']).read_text());a=np.flipud(cv2.imread(meta['image'],0))
p=a.astype(float)/255 if y.get('negate',0) else (255-a.astype(float))/255
occ=p>y['occupied_thresh'];res=y['resolution'];ox,oy,origin_yaw=y['origin']
model=mujoco.MjModel.from_xml_path(str(next((root/'robot').glob('*/model_info/scene.xml'))));data=mujoco.MjData(model)
wall_ids=[i for i in range(model.ngeom) if (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,i) or '').startswith('map_wall_')]
model.geom_group[:]=0;model.geom_group[wall_ids]=4;mujoco.mj_forward(model,data)
mask=np.zeros(6,np.uint8);mask[4]=1;gid=np.array([-1],np.int32)
sx,sy,heading=meta['start_map'];rows=[];errors=[];mismatch=0
cell_y,cell_x=np.nonzero(occ);xmin=cell_x*res;xmax=xmin+res;ymin=cell_y*res;ymax=ymin+res
c,s=math.cos(origin_yaw),math.sin(origin_yaw)
gx=c*(sx-ox)+s*(sy-oy);gy=-s*(sx-ox)+c*(sy-oy)
for theta in np.linspace(-math.pi,math.pi,360,endpoint=False):
    vx,vy=np.cos(theta),np.sin(theta)
    world_theta=theta-heading
    distance=mujoco.mj_ray(model,data,np.array([0.,0.,.9]),np.array([math.cos(world_theta),math.sin(world_theta),0.]),mask,1,-1,gid)
    measured=float(distance) if 0<=distance<=8 else None
    # Exact ray/cell intersections: sampling can miss a cell grazed near a corner.
    dx,dy=math.cos(theta-origin_yaw),math.sin(theta-origin_yaw)
    if abs(dx)<1e-12:
        nx=np.where((xmin<=gx)&(gx<=xmax),-np.inf,np.inf);fx=np.where((xmin<=gx)&(gx<=xmax),np.inf,-np.inf)
    else:
        tx1,tx2=(xmin-gx)/dx,(xmax-gx)/dx;nx=np.minimum(tx1,tx2);fx=np.maximum(tx1,tx2)
    if abs(dy)<1e-12:
        ny=np.where((ymin<=gy)&(gy<=ymax),-np.inf,np.inf);fy=np.where((ymin<=gy)&(gy<=ymax),np.inf,-np.inf)
    else:
        ty1,ty2=(ymin-gy)/dy,(ymax-gy)/dy;ny=np.minimum(ty1,ty2);fy=np.maximum(ty1,ty2)
    entry=np.maximum(np.maximum(nx,ny),0.);exit=np.minimum(fx,fy)
    candidates=np.where(exit>=entry-1e-10,entry,np.inf);nearest=float(candidates.min())
    expected=nearest if nearest<=8 else None
    if (expected is None)!=(measured is None):mismatch+=1
    if expected is not None and measured is not None:errors.append(abs(expected-measured))
    rows.append(dict(angle=float(theta),grid_m=expected,mujoco_m=measured))
result=dict(geoms=model.ngeom,wall_geoms=len(wall_ids),rays=len(rows),hit_mismatches=mismatch,
            absolute_error_max_m=max(errors),absolute_error_p95_m=float(np.percentile(errors,95)),rays_data=rows)
(root/'ray_validation.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='rays_data'},indent=2))
if mismatch or max(errors)>.025:raise SystemExit('map/world ray mismatch')
