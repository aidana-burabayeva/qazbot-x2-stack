#!/usr/bin/env python3
"""Read-only frame-contract audit from recorded torso/world and plant/nav pose.

Raw torso and nav pelvis are different body points: a rigid fit residual is reported,
not assumed zero. Occupied-cell distance at either point is NOT a contact-force measurement.
"""
import argparse
import bisect
import csv
import gzip
import json
import math
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml


def fit_transform(raw,nav):
    raw,nav=np.asarray(raw,float),np.asarray(nav,float)
    u,_,vt=np.linalg.svd((raw-raw.mean(0)).T@(nav-nav.mean(0)))
    rotation=vt.T@u.T
    if np.linalg.det(rotation)<0:
        vt[-1,:]*=-1;rotation=vt.T@u.T
    translation=nav.mean(0)-rotation@raw.mean(0)
    predicted=raw@rotation.T+translation
    return rotation,translation,float(np.sqrt(np.mean(np.sum((predicted-nav)**2,axis=1))))


def audit(directory,start,map_directory):
    directory=Path(directory);map_directory=Path(map_directory)
    with gzip.open(directory/'motion.csv.gz','rt') as f:rows=list(csv.DictReader(f))
    def select(kind):return [(int(r['received_ros_ns'])/1e9,np.array([float(r['x']),float(r['y'])])) for r in rows if r['kind']==kind]
    truth=select('truth');torso=select('torso');times=[r[0] for r in torso];pairs=[]
    for t,nav in truth:
        k=bisect.bisect_left(times,t)
        i=min((i for i in (k-1,k) if 0<=i<len(torso)),key=lambda i:abs(times[i]-t))
        if abs(times[i]-t)<=.05:pairs.append((nav,torso[i][1]))
    if len(pairs)<20:raise ValueError('too few synchronized torso/truth samples')
    nav=np.array([p[0] for p in pairs]);raw=np.array([p[1] for p in pairs])
    rotation,translation,rmse=fit_transform(raw,nav)
    theta=math.atan2(rotation[1,0],rotation[0,0]);c,s=math.cos(start[2]),math.sin(start[2])
    world_rotation=np.array([[c,-s],[s,c]])
    world=raw@world_rotation.T+np.asarray(start[:2])
    meta=yaml.safe_load((map_directory/'map.yaml').read_text())
    pixels=np.flipud(np.asarray(Image.open(map_directory/meta['image'])))
    edt=distance_transform_edt(pixels!=0)*meta['resolution']
    def clearance(p):
        x,y=np.floor((p-np.asarray(meta['origin'][:2]))/meta['resolution']).astype(int)
        return float(edt[y,x]) if 0<=y<edt.shape[0] and 0<=x<edt.shape[1] else None
    return {'samples':len(pairs),'start_map':start,'fitted_raw_torso_to_nav_pelvis_yaw_rad':theta,
        'fitted_translation_m':translation.tolist(),'fit_rmse_m':rmse,
        'world_generator_expected_yaw_rad':start[2],'rotation_contract_difference_rad':math.remainder(theta-start[2],2*math.pi),
        'published_nav_pelvis_end':nav[-1].tolist(),'raw_torso_in_generated_world_map_end':world[-1].tolist(),
        'point_discrepancy_end_m':float(np.linalg.norm(world[-1]-nav[-1])),
        'published_nav_point_occupied_cell_distance_m':clearance(nav[-1]),
        'physical_torso_point_occupied_cell_distance_m':clearance(world[-1]),
        'limitations':['torso and pelvis are different body points; fit residual quantifies their relative positional motion',
                      'world map transform follows build_world manifest contract; contacts/forces are not recorded',
                      'no conclusion of contact-free motion can be drawn from nav-map clearance alone']}


def main():
    ap=argparse.ArgumentParser();ap.add_argument('directory');ap.add_argument('--start',required=True)
    ap.add_argument('--map',default='ws/src/maps/main_office');ap.add_argument('--out',required=True);a=ap.parse_args()
    result=audit(a.directory,[float(v) for v in a.start.split(',')],a.map)
    Path(a.out).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
