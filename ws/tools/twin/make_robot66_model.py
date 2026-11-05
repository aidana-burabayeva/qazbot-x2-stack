#!/usr/bin/env python3
"""Build a reversible MJCF inertia overlay from the robot's archived nominal URDF.
Checks body frames; folds fixed-link payloads with the parallel-axis theorem.
Contact geometry, friction, gains and limits are intentionally not fitted.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as E
import numpy as np


def numbers(text):return np.fromstring(text,sep=' ')

def rotation(rpy):
    r,p,y=rpy;cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])

def origin(element):
    T=np.eye(4)
    if element is not None:T[:3,3]=numbers(element.get('xyz','0 0 0'));T[:3,:3]=rotation(numbers(element.get('rpy','0 0 0')))
    return T

def aggregate(parts):
    mass=sum(m for m,p,I in parts);center=sum(m*p for m,p,I in parts)/mass
    inertia=sum(I+m*((p-center)@(p-center)*np.eye(3)-np.outer(p-center,p-center)) for m,p,I in parts)
    if mass<=0 or np.linalg.eigvalsh(inertia).min()<=0:raise ValueError('invalid inertia')
    return mass,center,inertia


def build(sim_robot,urdf,out):
    if out.exists():raise ValueError('output already exists')
    xml=E.parse(sim_robot/'model_info/x2.xml');root=xml.getroot();u=E.parse(urdf).getroot()
    links={l.get('name'):l for l in u.findall('link')};incoming={};children={}
    for j in u.findall('joint'):
        parent,child=j.find('parent').get('link'),j.find('child').get('link')
        incoming[child]=j;children.setdefault(parent,[]).append(j)
    bodies={b.get('name'):b for b in root.iter('body')};used=set();changes=[]
    for name,b in bodies.items():
        if name not in links:raise ValueError('missing URDF body '+name)
        if name!='pelvis':
            expected=origin(incoming[name].find('origin'));actual=np.eye(4);actual[:3,3]=numbers(b.get('pos','0 0 0'))
            if b.get('quat'):
                w,x,y,z=numbers(b.get('quat'));norm=math.sqrt(w*w+x*x+y*y+z*z);w,x,y,z=np.array([w,x,y,z])/norm
                actual[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
            elif b.get('euler'):actual[:3,:3]=rotation(numbers(b.get('euler')))
            if not np.allclose(expected,actual,atol=1e-5,rtol=0):raise ValueError('body-frame mismatch '+name)
        parts=[]
        def visit(link,T):
            inertial=links[link].find('inertial')
            if inertial is not None:
                if link in used:raise ValueError('inertial counted twice '+link)
                used.add(link);ti=T@origin(inertial.find('origin'));i=inertial.find('inertia');m=float(inertial.find('mass').get('value'))
                I=np.array([[float(i.get('ixx')),float(i.get('ixy')),float(i.get('ixz'))],[float(i.get('ixy')),float(i.get('iyy')),float(i.get('iyz'))],[float(i.get('ixz')),float(i.get('iyz')),float(i.get('izz'))]])
                if m>0:parts.append((m,ti[:3,3],ti[:3,:3]@I@ti[:3,:3].T))
            for j in children.get(link,[]):
                child=j.find('child').get('link')
                if child in bodies:continue
                if j.get('type')!='fixed':raise ValueError('unmodeled moving link '+child)
                visit(child,T@origin(j.find('origin')))
        visit(name,np.eye(4));mass,center,I=aggregate(parts)
        old=b.find('inertial');old_info=None if old is None else dict(old.attrib)
        if old is not None:b.remove(old)
        fmt=lambda seq:' '.join(f'{v:.12g}' for v in seq)
        E.SubElement(b,'inertial',dict(pos=fmt(center),mass=f'{mass:.12g}',fullinertia=fmt([I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]])))
        changes.append(dict(body=name,old=old_info,new_mass=mass))
    missed=[name for name,l in links.items() if l.find('inertial') is not None and name not in used and float(l.find('inertial/mass').get('value'))>0]
    if missed:raise ValueError('unaccounted URDF payloads '+str(missed))
    target=out/'robot/lx2501_3_t2d5';model=target/'model_info';model.mkdir(parents=True)
    for entry in sim_robot.iterdir():
        if entry.name!='model_info':(target/entry.name).symlink_to(os.path.relpath(entry.resolve(),target),target_is_directory=entry.is_dir())
    for entry in (sim_robot/'model_info').iterdir():
        if entry.name not in ('x2.xml','scene.xml'):(model/entry.name).symlink_to(os.path.relpath(entry.resolve(),model),target_is_directory=entry.is_dir())
    shutil.copyfile(sim_robot/'model_info/scene.xml',model/'scene.xml');xml.write(model/'x2.xml',encoding='utf-8',xml_declaration=True)
    manifest=dict(source_urdf=str(urdf),source_urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),
                  official_xml_sha256=hashlib.sha256((sim_robot/'model_info/x2.xml').read_bytes()).hexdigest(),
                  qualification='Archived nominal .66 URDF inertia overlay; not measured per-unit inertias or validated dynamics.',bodies=changes,
                  total_nominal_mass_kg=sum(x['new_mass'] for x in changes),unchanged=['contact geometry','friction','actuator limits','MC policy','MC binary'])
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');return manifest


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('sim_robot',type=Path);ap.add_argument('urdf',type=Path);ap.add_argument('output',type=Path);a=ap.parse_args()
    m=build(a.sim_robot.resolve(),a.urdf.resolve(),a.output.resolve());print('Nominal mass:',m['total_nominal_mass_kg'],'bodies:',len(m['bodies']))
