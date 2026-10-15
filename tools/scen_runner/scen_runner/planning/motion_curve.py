"""공통 경로 샘플링/투영. 단위는 m, yaw는 rad."""
import numpy as np
from scipy.spatial import cKDTree

class Curve:
    def __init__(self, points):
        pts = np.asarray(points, dtype=float)
        ds = np.linalg.norm(np.diff(pts[:, :2], axis=0), axis=1)
        keep = np.r_[True, ds > 1e-6]
        raw_s = np.r_[0., np.cumsum(ds)][keep]
        pts = pts[keep]
        self.length = float(raw_s[-1])
        self.s = np.r_[np.arange(0., self.length, .25), self.length]
        self.xyz = np.stack([np.interp(self.s, raw_s, pts[:, i]) for i in range(3)], axis=1)
        self.heading = np.unwrap(np.arctan2(np.gradient(self.xyz[:, 1]), np.gradient(self.xyz[:, 0])))
        self.tree = cKDTree(self.xyz[:, :2])

    def pose(self, s):
        xyz = [float(np.interp(s, self.s, self.xyz[:, i])) for i in range(3)]
        return *xyz, float(np.interp(s, self.s, self.heading))

    def project(self, x, y):
        distance, idx = self.tree.query([x, y])
        return float(self.s[idx]), float(distance)
