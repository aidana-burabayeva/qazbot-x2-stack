"""Point-to-plane ICP (Gauss-Newton, Huber weights) with numpy + scipy cKDTree.
Standard algorithm (Chen & Medioni 1992; Rusinkiewicz & Levoy 2001); used because no Open3D/small_gicp/KISS-ICP
python module exists on this host (checked 2026-10-23). Cross-check against PCL GICP: pcl_gicp_check.cpp.
register(src, tgt, T0) returns T_tgt_src (p_tgt = T p_src) plus quality metrics:
  fitness   = inlier fraction of source voxels at the final correspondence distance
  rmse      = weighted point-to-plane rms (m)
  eig       = eigenvalues of the 6x6 normal matrix (rot rad, trans m) per inlier; small -> degenerate direction
  cov       = sigma^2 H^-1 (OPTIMISTIC: assumes independent residuals)
"""
import numpy as np
from scipy.spatial import cKDTree

from lc_common import exp_so3


def voxel_down(P, v):
    k = np.floor(P / v).astype(np.int64)
    _, inv_, cnt = np.unique(k, axis=0, return_inverse=True, return_counts=True)
    out = np.zeros((cnt.size, 3))
    np.add.at(out, inv_.ravel(), P)
    return out / cnt[:, None]


def normals(Q, k=12, rmax=0.6):
    tree = cKDTree(Q)
    d, idx = tree.query(Q, k=k, distance_upper_bound=rmax)
    ok = np.isfinite(d).all(1)
    N = np.zeros_like(Q)
    flat = np.zeros(len(Q), bool)
    nb = Q[np.where(ok[:, None], idx, 0)]
    c = nb - nb.mean(1, keepdims=True)
    C = np.einsum('nki,nkj->nij', c, c) / k
    w, V = np.linalg.eigh(C)
    N = V[:, :, 0]
    flat = ok & (w[:, 0] < 0.2 * w[:, 1]) & (w[:, 1] > 1e-5)     # planar patches only
    return tree, N, flat


def register(src, tgt, T0, voxel=0.10, schedule=(1.0, 0.5, 0.3, 0.15, 0.10), iters=15, huber=0.05,
             max_src=12000, seed=0):
    S = voxel_down(src, voxel)
    if len(S) > max_src:
        S = S[np.random.default_rng(seed).choice(len(S), max_src, replace=False)]
    Q = voxel_down(tgt, voxel)
    tree, N, flat = normals(Q)
    T = T0.copy()
    info = {}
    for dmax in schedule:
        for _ in range(iters):
            P = S @ T[:3, :3].T + T[:3, 3]
            d, j = tree.query(P, k=1, distance_upper_bound=dmax)
            m = np.isfinite(d)
            m[m] = flat[j[m]]
            if m.sum() < 200:
                return T, dict(ok=False, reason=f'few correspondences ({int(m.sum())}) at {dmax} m')
            p, q, n = P[m], Q[j[m]], N[j[m]]
            r = np.einsum('ij,ij->i', p - q, n)
            J = np.hstack([np.cross(p, n), n])
            w = np.where(np.abs(r) <= huber, 1.0, huber / np.abs(r))
            H = J.T @ (J * w[:, None])
            g = J.T @ (w * r)
            dx = -np.linalg.solve(H + 1e-9 * np.eye(6), g)
            dT = np.eye(4)
            dT[:3, :3], dT[:3, 3] = exp_so3(dx[:3]), dx[3:]
            T = dT @ T
            if np.linalg.norm(dx[:3]) < 1e-6 and np.linalg.norm(dx[3:]) < 1e-5:
                break
    nin = int(m.sum())
    sig2 = float((w * r * r).sum() / max(1, nin - 6))
    ev = np.linalg.eigvalsh(H[3:, 3:] / nin)
    evr = np.linalg.eigvalsh(H[:3, :3] / nin)
    info = dict(ok=True, fitness=nin / len(S), n_src=len(S), n_in=nin, rmse=float(np.sqrt(sig2)),
                eig_trans=ev.tolist(), eig_rot=evr.tolist(),
                cov=(sig2 * np.linalg.inv(H)).tolist())
    return T, info
