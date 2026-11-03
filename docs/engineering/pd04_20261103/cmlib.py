import pickle, math, datetime as dt
import numpy as np
D = '/tmp/claude-1001/-home-agi/875283d9-732e-4c1e-ba4d-0d8188cfcd4d/scratchpad/pd04/ex'
small = pickle.load(open(D + '/small.pkl', 'rb'))
TF = np.array(small['tfs'])           # stamp, x, y, yaw
TF = TF[np.argsort(TF[:, 0])]
FOV = (-46.0, 44.5)


def sod(ns):
    d = dt.datetime.fromtimestamp(ns / 1e9)
    return d.hour * 3600 + d.minute * 60 + d.second + d.microsecond * 1e-6


def hms(s):
    s = float(s); return '%02d:%02d:%04.1f' % (s // 3600, (s % 3600) // 60, s % 60)


def T(h, m, s):
    return h * 3600 + m * 60 + s


def pose_at(ns):
    i = max(0, np.searchsorted(TF[:, 0], ns, side='right') - 1)
    return TF[i, 1:]


def to_body(xy, pose):
    x, y, th = pose
    c, s = math.cos(-th), math.sin(-th)
    dx, dy = xy[:, 0] - x, xy[:, 1] - y
    return np.column_stack((c * dx - s * dy, s * dx + c * dy))


def live_A():
    out = []
    for c in small['cm']:
        g = c['grid']
        iy, ix = np.nonzero(g == 254)
        xy = np.column_stack((c['ox'] + (ix + 0.5) * c['res'], c['oy'] + (iy + 0.5) * c['res']))
        out.append(dict(s=sod(c['stamp']), ns=c['stamp'], B=to_body(xy, pose_at(c['stamp'])), xy=xy))
    return out


def nav2(var):
    d = pickle.load(open(D + f'/nav2_{var}.pkl', 'rb'))
    frames = pickle.load(open(D + '/frames.pkl', 'rb')) if not hasattr(nav2, 'fr') else nav2.fr
    nav2.fr = frames
    out = []
    for r in d['recs']:
        if r['phase'] != 'run' or r['k'] < 0:
            continue
        ns = frames[r['k']]['key']
        out.append(dict(s=sod(ns), ns=ns, B=to_body(r['xy'], pose_at(ns)) if len(r['xy']) else np.empty((0, 2)), xy=r['xy']))
    return out


def zones(B):
    if not len(B):
        return dict(inner=0, edgeL=0, edgeR=0, rear=0, tot=0)
    b = np.degrees(np.arctan2(B[:, 1], B[:, 0])); r = np.hypot(B[:, 0], B[:, 1])
    n = r < 1.5
    return dict(inner=int((n & (b >= FOV[0]) & (b <= FOV[1])).sum()), edgeL=int((n & (b > FOV[1]) & (b <= 90)).sum()),
                edgeR=int((n & (b < FOV[0]) & (b >= -90)).sum()), rear=int((n & (np.abs(b) > 90)).sum()), tot=len(B))


def region_count(B, cells, res=0.05):
    """cells: set of (ix,iy) body-frame 5 cm keys"""
    if not len(B):
        return 0
    k = np.floor(B / res).astype(int)
    return int(sum((a, b) in cells for a, b in k))
