import math, sys, numpy as np, xml.etree.ElementTree as ET, json
import os
URDF = sys.argv[1]
#: directory holding meshes/*.STL (copy from X2: /agibot/software/aimdk/extra/mc-rl/x2_urdf/X2_URDF-v1.3.0/meshes)
MESH_DIR = os.environ.get('X2_URDF_DIR', os.path.dirname(os.path.abspath(URDF)))
root = ET.parse(URDF).getroot()
def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])
def origin(el):
    o = el.find('origin') if el is not None else None
    xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
    rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
    T = np.eye(4); T[:3, :3] = rpy_mat(*rpy); T[:3, 3] = xyz; return T
joints = {}
for j in root.findall('joint'):
    lim = j.find('limit'); ax = j.find('axis')
    joints[j.get('name')] = dict(type=j.get('type'), parent=j.find('parent').get('link'), child=j.find('child').get('link'), T=origin(j),
        axis=np.array([float(v) for v in ax.get('xyz').split()]) if ax is not None else np.array([0, 0, 1.0]),
        lo=float(lim.get('lower', 0)) if lim is not None else 0, hi=float(lim.get('upper', 0)) if lim is not None else 0)
child_joint = {j['child']: n for n, j in joints.items()}
links = {l.get('name'): l for l in root.findall('link')}
def axis_rot(axis, q):
    a = axis / np.linalg.norm(axis); K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    T = np.eye(4); T[:3, :3] = np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K; return T
def link_T(link, q):
    T = np.eye(4); chain = []
    while link in child_joint:
        n = child_joint[link]; chain.append(n); link = joints[n]['parent']
    for n in reversed(chain):
        j = joints[n]; T = T @ j['T'] @ (axis_rot(j['axis'], q.get(n, 0.0)) if j['type'] in ('revolute', 'continuous') else np.eye(4))
    return T
_stl = {}
def stl_pts(path):
    if path not in _stl:
        raw = np.fromfile(path, dtype=np.uint8); n = int(np.frombuffer(raw[80:84], dtype='<u4')[0])
        rec = raw[84:84 + 50 * n].reshape(n, 50)[:, 12:48].copy().view('<f4').reshape(n, 3, 3)
        v = rec.reshape(-1, 3).astype(float); _stl[path] = np.unique(np.round(v, 4), axis=0)
    return _stl[path]
def collision_pts(link):
    """Sample points covering each collision primitive (in the link frame)."""
    out = []
    for col in links[link].findall('collision'):
        g = col.find('geometry')[0]; To = origin(col); pts = []
        if g.tag == 'sphere':
            r = float(g.get('radius')); u = np.array([[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]], float)
            pts = u * r
        elif g.tag == 'box':
            s = np.array([float(v) for v in g.get('size').split()]) / 2
            pts = np.array([[sx*s[0], sy*s[1], sz*s[2]] for sx in (-1,1) for sy in (-1,1) for sz in (-1,1)])
        elif g.tag == 'cylinder':
            r, h = float(g.get('radius')), float(g.get('length')); ang = np.linspace(0, 2*math.pi, 24, endpoint=False)
            pts = np.array([[r*math.cos(a), r*math.sin(a), z] for a in ang for z in (-h/2, h/2)])
        elif g.tag == 'mesh':
            pts = stl_pts(os.path.join(MESH_DIR, g.get('filename').replace('./', '')))
            sc = g.get('scale'); pts = pts * (np.array([float(v) for v in sc.split()]) if sc else 1.0)
        if len(pts): out.append((To @ np.c_[pts, np.ones(len(pts))].T).T[:, :3])
    return np.vstack(out) if out else np.zeros((0, 3))
def world_pts(q, only=None):
    res = {}
    for l in links:
        if only and not only(l): continue
        P = collision_pts(l)
        if len(P): T = link_T(l, q); res[l] = (T @ np.c_[P, np.ones(len(P))].T).T[:, :3]
    return res
