#!/usr/bin/env python3
"""Generate an exact grid extrusion for official MuJoCo, without editing vendor files.
Only occupied cells are walls; unknown space is not invented geometry. The sim pelvis spawns
at (0,0); START specifies its map pose. Map image/YAML and generated transform are pinned.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET
import cv2
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[3]
DEFAULT_ROBOT=ROOT/'vendor/official/sim/sim_mujoco-x86-v1.0.0-20260522/configuration/robot/lx2501_3_t2d5'


def rectangles(mask):
    """Disjoint cell rectangles (x0,y0,x1,y1); merge identical row runs vertically."""
    active={};done=[]
    for y,row in enumerate(mask):
        padded=np.r_[False,row,False].astype(np.int8)
        edges=np.flatnonzero(np.diff(padded));runs=list(zip(edges[::2],edges[1::2]))
        current={}
        for a,b in runs:
            current[(int(a),int(b))]=active.pop((int(a),int(b)),(int(a),y,int(b),y))[:3]+(y+1,)
        done.extend(active.values());active=current
    return done+list(active.values())


def build(map_yaml,start,output,robot=DEFAULT_ROBOT):
    map_yaml=Path(map_yaml).resolve();robot=Path(robot).resolve();output=Path(output).resolve()
    y=yaml.safe_load(map_yaml.read_text());image=(map_yaml.parent/y['image']).resolve()
    a=cv2.imread(str(image),cv2.IMREAD_GRAYSCALE)
    if a is None:raise ValueError('map image unreadable')
    if y.get('mode','trinary')!='trinary':raise ValueError('only trinary maps supported')
    p=a.astype(float)/255.;p=p if y.get('negate',0) else 1-p
    blocked=np.flipud(p>float(y['occupied_thresh']))
    free=np.flipud(p<float(y['free_thresh']))
    res=float(y['resolution']);ox,oy,angle=map(float,y['origin']);sx,sy,heading=start
    if res<=0 or not np.isfinite([res,ox,oy,angle,*start]).all():raise ValueError('invalid map transform')
    c,s=math.cos(angle),math.sin(angle);dx,dy=sx-ox,sy-oy
    ix,iy=int(math.floor((c*dx+s*dy)/res)),int(math.floor((-s*dx+c*dy)/res))
    if not (0<=iy<a.shape[0] and 0<=ix<a.shape[1]) or not free[iy,ix]:raise ValueError('spawn is not in free space')
    # Require the physical body envelope free at the configured spawn.
    margin=cv2.distanceTransform(free.astype(np.uint8),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)[iy,ix]*res
    if margin<.52:raise ValueError(f'spawn clearance {margin:.3f}m < 0.52m')
    boxes=rectangles(blocked)
    # Exact discrete coverage check, including row/image flip.
    reconstructed=np.zeros_like(blocked,dtype=np.uint8)
    for x0,y0,x1,y1 in boxes:reconstructed[y0:y1,x0:x1]+=1
    if not np.array_equal(reconstructed,blocked.astype(np.uint8)):raise RuntimeError('rectangle coverage mismatch')
    output.mkdir(parents=True,exist_ok=False)
    dest=output/'robot'/robot.name;model=dest/'model_info';model.mkdir(parents=True)
    for entry in robot.iterdir():
        if entry.name!='model_info':(dest/entry.name).symlink_to(os.path.relpath(entry,dest),target_is_directory=entry.is_dir())
    for entry in (robot/'model_info').iterdir():
        if entry.name!='scene.xml':(model/entry.name).symlink_to(os.path.relpath(entry,model),target_is_directory=entry.is_dir())
    tree=ET.parse(robot/'model_info/scene.xml');world=tree.getroot().find('worldbody')
    hc,hs=math.cos(heading),math.sin(heading)
    for i,(x0,y0,x1,y1) in enumerate(boxes):
        gx,gy=(x0+x1)*res/2,(y0+y1)*res/2
        mx,my=ox+c*gx-s*gy,oy+s*gx+c*gy
        wx,wy=hc*(mx-sx)+hs*(my-sy),-hs*(mx-sx)+hc*(my-sy)
        ET.SubElement(world,'geom',name=f'map_wall_{i}',type='box',
            pos=f'{wx:.9f} {wy:.9f} 0.8',size=f'{(x1-x0)*res/2:.9f} {(y1-y0)*res/2:.9f} 0.8',
            euler=f'0 0 {angle-heading:.12f}',rgba='0.65 0.68 0.72 1',contype='1',conaffinity='1')
    tree.write(model/'scene.xml',encoding='utf-8',xml_declaration=True)
    manifest=dict(map_yaml=str(map_yaml),image=str(image),image_sha256=hashlib.sha256(image.read_bytes()).hexdigest(),
        yaml_sha256=hashlib.sha256(map_yaml.read_bytes()).hexdigest(),start_map=list(start),shape=list(a.shape),resolution=res,
        origin=y['origin'],rectangles=len(boxes),blocked_cells=int(blocked.sum()),spawn_clearance_m=float(margin),
        coverage_exact=True,unknown_is_solid=False,wall_height_m=1.6,robot=str(robot),
        transform='map_xy = R(start_yaw) @ sim_xy + start_xy; sim initial pelvis XY=(0,0)')
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');return manifest


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('map_yaml');ap.add_argument('output');ap.add_argument('--start',nargs=3,type=float,default=[-1.275,-4.825,0.]);ap.add_argument('--robot',default=str(DEFAULT_ROBOT))
    args=ap.parse_args();print(json.dumps(build(args.map_yaml,args.start,args.output,args.robot),indent=2))
