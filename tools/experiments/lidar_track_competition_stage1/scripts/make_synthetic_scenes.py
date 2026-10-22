#!/usr/bin/env python3
"""Deterministic SYNTHETIC LiDAR scenes for a tracker math/implementation diagnostic.

DIAGNOSTIC_ONLY: ideal synthetic observations; never mixed with real-data performance.

usage: PYTHONDONTWRITEBYTECODE=1 python3 -B make_synthetic_scenes.py OUT_DIR
           [--scenes syn01_ego_stop_static,...] [--ground-z-base 0.0] [--no-validate] [--no-real-reference]

For every scenario OUT_DIR/<scene>/ gets (same columns as the frozen real inputs):
  lidar/live_NNNNNN.npy : float32 (N,4) x,y,z,intensity in the LiDAR SENSOR frame (x fwd, y left, z up)
  frames.csv            : harness/evaluator frame stream (columns of prepare_inputs.py frames.csv)
  gt.csv                : evaluation GT (columns of prepare_inputs.py gt.csv; kind=gt only, no ignore rows)
  frames_comp.csv       : competition-restricted stream (ALLOWED then FORBIDDEN columns of
                          prepare_competition_inputs.py) with TRUE vel_x / heading
  truth_visible.csv     : per scan x object: visible point count/mean vs GT box center, true velocities
plus OUT_DIR/INPUT_MANIFEST.json (structure of the frozen real INPUT_MANIFEST.json, role/split "synthetic"),
OUT_DIR/COMP_INPUT_MANIFEST.json, OUT_DIR/SYNTHETIC_MANIFEST.json and OUT_DIR/VALIDATION.json.

Geometry (all exact, no noise, no intra-scan motion distortion):
  world: flat ground plane z_world = 0.  base_link = rear-axle centre; its height above the ground is
         -GROUND_Z_BASE (default 0: base_link on the ground).  LiDAR at (0.80, 0.00, 1.45) in base_link, rpy 0.
  ego:   unicycle on base_link: x' = v cos psi, y' = v sin psi, psi' = omega, v' = a with piecewise-constant
         (a, omega) whose switch times lie on the 1 ms grid; v and psi are integrated analytically, x and y by
         Simpson's rule on a 1 ms grid (exact to ~1e-12 m for these integrands).
  objects: unicycle on the BEV box centre (same integration); box bottom on the ground.
  p_base = R(-ego_yaw) (p_world - ego_xy);  p_sensor = p_base - (0.80, 0, 1.45).
  GT box (base_link at t_lidar): x,y = BEV centre, z = GROUND_Z_BASE + h/2, l along heading, w across,
         yaw = heading - ego_yaw wrapped to [-pi, pi), vx,vy = GROUND velocity rotated into ego axes,
         gx,gy,yaw_global = world centre / heading.  Vehicles l >= w; pedestrians l < w (as in the real GT).
  ray caster: 16 rings at -15..+15 deg step 2 deg, azimuth step 0.2 deg over [-134.6, +134.6] deg (column
         order 0, -0.2, ..., -134.6, +134.6, ..., +0.2 like the real npy, rings bottom->top inside a column),
         nearest hit of {ground, every box}; returned only if 1.0 m <= range <= 55.0 m.  Intensity: ground 10,
         objects 50.
"""
import sys

sys.dont_write_bytecode = True

import argparse, csv, hashlib, json, math, os, platform, shutil  # noqa: E402

import numpy as np  # noqa: E402

LABEL = "DIAGNOSTIC_ONLY: ideal synthetic observations; never mixed with real-data performance"
RR = "/home/autonav/2026AIM/Lidar_experimments/track_competition_stage1_20261022_134204"
REAL_SCENE_DIR = RR + "/work/inputs_comp/scen93"            # read-only header reference
REAL_INPUTS = "/home/autonav/2026AIM/Lidar_experimments/obb_init_turn_follow_20261019_032258/work/inputs"
MARKER = ".synthetic_scene"

FRAMES_COLS = ["idx", "stem", "t_lidar", "stamp_ns", "dt_prev_s", "npy", "ego_x", "ego_y", "ego_z", "ego_yaw",
               "ego_speed_mps", "n_gt", "n_ignore"]
GT_COLS = ["idx", "stamp_ns", "kind", "track_id", "class_id", "x", "y", "z", "w", "l", "h", "yaw", "vx", "vy",
           "gx", "gy", "yaw_global"]
COMP_ALLOWED = ["idx", "stem", "stamp_ns", "npy", "vel_x_mps", "imu_yaw_rad", "imu_roll_rad", "imu_pitch_rad",
                "imu_wz_rps"]
COMP_FORBIDDEN = ["ego_x", "ego_y", "ego_z", "ego_speed_norm_mps", "vel_y_mps", "vel_z_mps", "accel_x", "accel_y",
                  "accel_z"]
TRUTH_COLS = ["idx", "track_id", "n_points_visible", "visible_point_mean_x", "visible_point_mean_y",
              "visible_point_mean_z", "box_center_x", "box_center_y", "yaw", "vx_ground_ego", "vy_ground_ego",
              "ego_v", "ego_omega",
              # extras (after the requested columns)
              "stem", "stamp_ns", "class_id", "box_center_z", "l", "w", "h", "gx", "gy", "yaw_global",
              "range_base_m", "bearing_base_deg", "in_gt", "dropout", "n_points_raycast", "ego_accel"]

HZ = 15
SENSOR = (0.80, 0.00, 1.45)
ELEV_DEG = [-15.0 + 2.0 * k for k in range(16)]
AZ_STEP_DEG = 0.2
AZ_HALF_FOV_DEG = 134.6
R_MIN, R_MAX = 1.0, 55.0
INT_GROUND, INT_OBJECT = 10.0, 50.0
GT_MAX_RANGE = 70.0
T0_EPOCH_S = 1800000000
EGO_FOOTPRINT = (4.6, 1.9, 1.0)          # length, width, rear overhang behind the rear axle (collision check only)

VEH_A = (4.600, 1.900, 1.500)
VEH_B = (4.575, 1.760, 1.470)
VEH_C = (5.173, 2.000, 1.710)
VEH_D = (4.970, 2.072, 1.509)
PED = (0.400, 0.600, 1.750)               # l < w like the real pedestrian GT (l 0.30-0.42, w 0.49-0.61)
CLS = {"vehicle": 0, "pedestrian": 1}


# ============================================================================ helpers
def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def ffmt(v, nd):
    s = "%.*f" % (nd, v)
    if s.startswith("-") and float(s) == 0.0:
        s = s[1:]
    return s


def sha_file(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def to_ms(t):
    m = int(round(t * 1000.0))
    assert abs(t * 1000.0 - m) < 1e-6, "phase time not on the 1 ms grid: %r" % t
    return m


class Motion:
    """Unicycle with piecewise-constant (a, omega) phases [(t0_s, t1_s, a, omega)], additive when overlapping."""

    def __init__(self, x0, y0, psi0, v0=0.0, phases=()):
        self.x0, self.y0, self.psi0, self.v0 = float(x0), float(y0), float(psi0), float(v0)
        self.phases = [(to_ms(t0), to_ms(t1), float(a), float(w)) for t0, t1, a, w in phases]
        self.T = -1

    def params(self):
        return {"x0": self.x0, "y0": self.y0, "psi0": self.psi0, "v0": self.v0,
                "phases_s_a_omega": [[p[0] / 1000.0, p[1] / 1000.0, p[2], p[3]] for p in self.phases]}

    def v_at(self, t):
        t = np.asarray(t, dtype=float)
        v = np.full(t.shape, self.v0)
        for t0, t1, a, _ in self.phases:
            if a:
                v = v + a * np.clip(t - t0 / 1000.0, 0.0, (t1 - t0) / 1000.0)
        return v

    def psi_at(self, t):
        t = np.asarray(t, dtype=float)
        p = np.full(t.shape, self.psi0)
        for t0, t1, _, w in self.phases:
            if w:
                p = p + w * np.clip(t - t0 / 1000.0, 0.0, (t1 - t0) / 1000.0)
        return p

    def build(self, T_ms):
        th = np.arange(2 * T_ms + 1) * 0.0005
        v, p = self.v_at(th), self.psi_at(th)
        fx, fy = v * np.cos(p), v * np.sin(p)
        h6 = 0.001 / 6.0
        dx = h6 * (fx[0:-1:2] + 4.0 * fx[1::2] + fx[2::2])
        dy = h6 * (fy[0:-1:2] + 4.0 * fy[1::2] + fy[2::2])
        ds = h6 * (v[0:-1:2] + 4.0 * v[1::2] + v[2::2])
        self.X = np.concatenate([[self.x0], self.x0 + np.cumsum(dx)])
        self.Y = np.concatenate([[self.y0], self.y0 + np.cumsum(dy)])
        self.S = np.concatenate([[0.0], np.cumsum(ds)])
        self.T = T_ms
        return self

    def ctrl(self, ms):
        a = w = 0.0
        for t0, t1, aa, ww in self.phases:
            if t0 <= ms < t1:
                a += aa
                w += ww
        return a, w

    def state(self, ms):
        assert 0 <= ms <= self.T
        t = ms / 1000.0
        a, w = self.ctrl(ms)
        return {"x": float(self.X[ms]), "y": float(self.Y[ms]), "psi": float(self.psi_at(t)),
                "v": float(self.v_at(t)), "omega": w, "a": a}

    def const_velocity(self, ms_a, ms_b):
        """True if the world velocity vector is constant on [ms_a, ms_b]."""
        for t0, t1, a, w in self.phases:
            if (a or w) and t0 < ms_b and t1 > ms_a:
                return False
        return True

    def is_static(self):
        return self.v0 == 0.0 and not any(a for _, _, a, _ in self.phases)


def obj(tid, cls, dims, motion, note=""):
    l, w, h = dims
    return {"id": tid, "cls": cls, "l": l, "w": w, "h": h, "motion": motion, "note": note}


# ============================================================================ scenarios
def scenarios():
    S = []
    # ---------------------------------------------------------------- syn01
    S.append({"name": "syn01_ego_stop_static", "duration_s": 6.0,
              "description": "ego stopped; two parked cars and a standing pedestrian",
              "ego": Motion(0, 0, 0, 0.0),
              "objects": [obj(1, "vehicle", VEH_A, Motion(12, 4, 0.3)),
                          obj(2, "vehicle", VEH_A, Motion(20, -5, 0.0)),
                          obj(3, "pedestrian", PED, Motion(8, -3, math.pi / 2))]})
    # ---------------------------------------------------------------- syn02 / syn07 world
    def parked_rows():
        out, tid, dims = [], 1, [VEH_A, VEH_B, VEH_C, VEH_D]
        for side, y in ((0, 4.0), (1, -4.0)):
            for j, x in enumerate(range(15, 111, 15)):
                yaw = 0.2 if (j + side) % 2 else 0.0
                out.append(obj(tid, "vehicle", dims[(j + side) % 4], Motion(x, y, yaw)))
                tid += 1
        out.append(obj(tid, "pedestrian", PED, Motion(40, 3, -math.pi / 2), "standing"))
        return out
    S.append({"name": "syn02_ego_straight_static", "duration_s": 8.0,
              "description": "ego straight 10 m/s; parked cars at y=+-4 every 15 m (x 15..105), standing pedestrian",
              "ego": Motion(0, 0, 0, 10.0), "objects": parked_rows()})
    # ---------------------------------------------------------------- syn03
    R = 8.0 / 0.25
    place = [(0.30, 8.0, 0.0, VEH_A), (0.55, -10.0, 0.0, VEH_B), (0.85, 12.0, 0.3, VEH_C),
             (1.10, -8.0, 0.0, VEH_D), (1.40, 15.0, math.pi / 2, VEH_A), (1.70, -14.0, -0.2, VEH_B),
             (2.00, 9.0, 0.0, VEH_C), (2.30, -20.0, 0.0, VEH_D), (2.60, 25.0, 0.5, VEH_A),
             (2.85, -9.0, 0.0, VEH_B)]
    objs = []
    for i, (th, d, yo, dims) in enumerate(place):
        rr = R + d
        objs.append(obj(i + 1, "vehicle", dims, Motion(rr * math.sin(th), R - rr * math.cos(th), th + yo),
                        "arc angle %.2f rad, %+.0f m from the path (+ = outside)" % (th, d)))
    rr = R + 5.0
    objs.append(obj(len(objs) + 1, "pedestrian", PED, Motion(rr * math.sin(1.25), R - rr * math.cos(1.25), 1.25 + math.pi / 2),
                    "standing 5 m outside the arc"))
    S.append({"name": "syn03_ego_turn_static", "duration_s": 8.0,
              "description": "ego 8 m/s constant left turn omega 0.25 rad/s (R=32 m); stationary cars 8-25 m inside/outside the arc",
              "ego": Motion(0, 0, 0, 8.0, [(0.0, 100.0, 0.0, 0.25)]), "objects": objs})
    # ---------------------------------------------------------------- syn04
    place = [(0, 10, 0.0, VEH_A), (60, 15, 0.5, VEH_B), (120, 10, math.pi / 2, VEH_C), (180, 15, 1.0, VEH_D),
             (-120, 10, -0.4, VEH_A), (-60, 15, 0.25, VEH_B)]
    objs = [obj(i + 1, "vehicle", dims, Motion(r * math.cos(math.radians(b)), r * math.sin(math.radians(b)), yw),
                "bearing %d deg, %d m from base_link at t=0" % (b, r))
            for i, (b, r, yw, dims) in enumerate(place)]
    objs.append(obj(7, "pedestrian", PED, Motion(0.0, 6.0, 0.0), "bearing 90 deg, 6 m"))
    S.append({"name": "syn04_ego_rotate_inplace", "duration_s": 6.0,
              "description": "ego v=0 rotating in place omega 0.4 rad/s (sensor lever-arm test); stationary cars at 10/15 m",
              "ego": Motion(0, 0, 0, 0.0, [(0.0, 100.0, 0.0, 0.4)]), "objects": objs})
    # ---------------------------------------------------------------- syn05
    S.append({"name": "syn05_ego_straight_moving", "duration_s": 8.0,
              "description": "ego 10 m/s; lead car 15 m ahead 12 m/s, -3 m/s^2 from t=3 s to 6 m/s (t=5 s); oncoming car y=+3.5 at -10 m/s from x=90; pedestrian crossing (+y) at x=92 at 1.4 m/s",
              "ego": Motion(0, 0, 0, 10.0),
              "objects": [obj(1, "vehicle", VEH_A, Motion(15, 0, 0.0, 12.0, [(3.0, 5.0, -3.0, 0.0)]), "lead"),
                          obj(2, "vehicle", VEH_D, Motion(90, 3.5, math.pi, 10.0), "oncoming"),
                          obj(3, "pedestrian", PED, Motion(92, -3, math.pi / 2, 1.4),
                              "crossing right->left at x=92 (beyond the lead car's reach; crosses the ego lane while "
                              "still out of range, then visible crossing the oncoming lane / roadside)")]})
    # ---------------------------------------------------------------- syn06
    S.append({"name": "syn06_target_turn_accel", "duration_s": 9.0,
              "description": "ego stationary; car A from (15,-10) heading +y, 0->8 m/s at 2 m/s^2 (0-4 s), left turn omega 0.35 rad/s (4-7 s), straight; car B crosses left->right at x=25, 6 m/s",
              "ego": Motion(0, 0, 0, 0.0),
              "objects": [obj(1, "vehicle", VEH_A, Motion(15, -10, math.pi / 2, 0.0,
                                                          [(0.0, 4.0, 2.0, 0.0), (4.0, 7.0, 0.0, 0.35)]), "A"),
                          obj(2, "vehicle", VEH_C, Motion(25, 20, -math.pi / 2, 6.0), "B")]})
    # ---------------------------------------------------------------- syn07
    S.append({"name": "syn07_irregular_dt", "duration_s": 8.0,
              "description": "world/ego of syn02; scan dt = 1/15 s + U(-15,+15) ms (RandomState(7)), quantised to ms; scans 25-26, 55-56, 85-86 of the jittered sequence dropped (no row)",
              "time": {"mode": "jitter", "seed": 7, "jitter_ms": 15.0, "drop_scans": [25, 26, 55, 56, 85, 86]},
              "ego": Motion(0, 0, 0, 10.0), "objects": parked_rows()})
    # ---------------------------------------------------------------- syn08
    S.append({"name": "syn08_occlusion_reacquire", "duration_s": 100 / 15.0,
              "description": "ego 10 m/s; stationary car (68,5) with ALL its points removed in scans 30-32, 60, 90-94 (GT kept); moving car 20 m ahead 8 m/s with points removed in scans 45-47",
              "ego": Motion(0, 0, 0, 10.0),
              "objects": [obj(1, "vehicle", VEH_A, Motion(68, 5, 0.0), "stationary, dropouts"),
                          obj(2, "vehicle", VEH_B, Motion(20, 0, 0.0, 8.0), "moving same lane, dropouts")],
              "dropouts": {1: [30, 31, 32, 60, 90, 91, 92, 93, 94], 2: [45, 46, 47]}})
    # ---------------------------------------------------------------- syn09
    S.append({"name": "syn09_ego_brake", "duration_s": 7.0,
              "description": "ego 12 m/s, -4 m/s^2 from t=1 s to standstill at t=4 s (x=30 m), then stopped; stopped lead car, parked cars, standing pedestrian",
              "ego": Motion(0, 0, 0, 12.0, [(1.0, 4.0, -4.0, 0.0)]),
              "objects": [obj(1, "vehicle", VEH_D, Motion(40, 0, 0.0), "stopped lead"),
                          obj(2, "vehicle", VEH_A, Motion(20, 4.5, 0.0)),
                          obj(3, "vehicle", VEH_B, Motion(32, -4.5, 0.15)),
                          obj(4, "vehicle", VEH_C, Motion(50, 4.2, -0.1)),
                          obj(5, "vehicle", VEH_A, Motion(28, 4.5, 0.0)),
                          obj(6, "pedestrian", PED, Motion(36, 3.5, math.pi), "standing")]})
    # ---------------------------------------------------------------- syn10
    ego10 = Motion(0, 0, 0, 4.0, [(0.0, 4.0, 1.5, 0.0), (0.0, 100.0, 0.0, 0.15)])
    tmp = Motion(0, 0, 0, 4.0, [(0.0, 4.0, 1.5, 0.0), (0.0, 100.0, 0.0, 0.15)]).build(16000)
    objs = []
    lead_s = 14.0
    k = int(np.searchsorted(tmp.S, lead_s))
    st = tmp.state(k)
    objs.append(obj(1, "vehicle", VEH_B, Motion(st["x"], st["y"], st["psi"], 9.0, [(0.0, 100.0, 0.0, 0.15)]),
                    "moving ahead: starts on the ego path at arc length %.0f m, 9 m/s, omega 0.15 rad/s (curvature 1/60 m = ego curvature at 9 m/s)" % lead_s))
    dims = [VEH_A, VEH_C, VEH_D, VEH_B]
    tid = 2
    for j, (s_arc, d) in enumerate([(15, -7.0), (30, 7.0), (45, -7.0), (60, 7.0), (75, -7.0), (90, 7.0), (105, -7.0),
                                    (120, 7.0)]):
        k = int(np.searchsorted(tmp.S, s_arc))
        st = tmp.state(k)
        x = st["x"] - d * math.sin(st["psi"])
        y = st["y"] + d * math.cos(st["psi"])
        objs.append(obj(tid, "vehicle", dims[j % 4], Motion(x, y, st["psi"]),
                        "roadside: ego arc length %d m, %+.0f m lateral (+ = left)" % (s_arc, d)))
        tid += 1
    k = int(np.searchsorted(tmp.S, 52.0))
    st = tmp.state(k)
    objs.append(obj(tid, "pedestrian", PED, Motion(st["x"] - 5.0 * math.sin(st["psi"]), st["y"] + 5.0 * math.cos(st["psi"]),
                                                     st["psi"] - math.pi / 2), "standing, arc 52 m, 5 m left"))
    S.append({"name": "syn10_ego_accel_turn_moving", "duration_s": 8.0,
              "description": "ego 4->10 m/s at 1.5 m/s^2 (0-4 s) while turning omega 0.15 rad/s; car ahead 9 m/s with omega 0.15 rad/s; roadside parked cars +-7 m; standing pedestrian",
              "ego": ego10, "objects": objs})
    for s in S:
        s.setdefault("time", {"mode": "regular"})
        s.setdefault("dropouts", {})
    return S


# ============================================================================ time grid
def scan_times(sc):
    tm = sc["time"]
    dur_ms = int(round(sc["duration_s"] * 1000.0))
    if tm["mode"] == "regular":
        n = int(round(sc["duration_s"] * HZ)) + 1
        out = [(k, (400 * k + 3) // 6) for k in range(n)]          # round(1000 k / 15) without float ties
        return [o for o in out if o[1] <= dur_ms + 1]
    rng = np.random.RandomState(tm["seed"])
    u = rng.uniform(-tm["jitter_ms"], tm["jitter_ms"], size=10 * HZ * int(sc["duration_s"] + 2))
    out, cum, k = [(0, 0)], 0.0, 0
    while True:
        cum += 1000.0 / HZ + float(u[k])
        k += 1
        ms = int(math.floor(cum + 0.5))
        if ms > dur_ms:
            break
        out.append((k, ms))
    drop = set(tm["drop_scans"])
    return [o for o in out if o[0] not in drop]


# ============================================================================ ray caster
def ray_dirs():
    n = int(round(AZ_HALF_FOV_DEG / AZ_STEP_DEG))                    # 673
    cols = np.concatenate([-np.arange(0, n + 1), np.arange(n, 0, -1)]).astype(float)
    az_deg = cols * AZ_STEP_DEG
    az = np.radians(np.repeat(az_deg, len(ELEV_DEG)))
    el = np.radians(np.tile(np.array(ELEV_DEG), len(az_deg)))
    D = np.column_stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    return D, len(az_deg)


def _safe(d):
    return np.where(np.abs(d) < 1e-15, np.where(d < 0, -1e-15, 1e-15), d)


def ray_box(D, c, yaw, hl, hw, hh):
    cs, sn = math.cos(yaw), math.sin(yaw)
    ox = -(cs * c[0] + sn * c[1])
    oy = -(-sn * c[0] + cs * c[1])
    oz = -c[2]
    dx = _safe(cs * D[:, 0] + sn * D[:, 1])
    dy = _safe(-sn * D[:, 0] + cs * D[:, 1])
    dz = _safe(D[:, 2])
    t1, t2 = (-hl - ox) / dx, (hl - ox) / dx
    tn, tf = np.minimum(t1, t2), np.maximum(t1, t2)
    t1, t2 = (-hw - oy) / dy, (hw - oy) / dy
    tn, tf = np.maximum(tn, np.minimum(t1, t2)), np.minimum(tf, np.maximum(t1, t2))
    t1, t2 = (-hh - oz) / dz, (hh - oz) / dz
    tn, tf = np.maximum(tn, np.minimum(t1, t2)), np.minimum(tf, np.maximum(t1, t2))
    hit = (tn <= tf) & (tn > 0.0)
    return np.where(hit, tn, np.inf)


def raycast(D, ground_s, boxes):
    """boxes: list of (cx, cy, cz, yaw, l, w, h) in the sensor frame. Returns pts (N,3) f64, label (N,) int."""
    with np.errstate(divide="ignore", invalid="ignore"):
        tg = np.where(D[:, 2] < 0.0, ground_s / np.where(D[:, 2] < 0.0, D[:, 2], -1.0), np.inf)
    T = [tg]
    idx = [-1]
    for i, (cx, cy, cz, yaw, l, w, h) in enumerate(boxes):
        if math.hypot(cx, cy) - 0.5 * math.hypot(l, w) > R_MAX:
            continue
        T.append(ray_box(D, (cx, cy, cz), yaw, 0.5 * l, 0.5 * w, 0.5 * h))
        idx.append(i)
    T = np.vstack(T)
    j = np.argmin(T, axis=0)
    t = T[j, np.arange(T.shape[1])]
    ok = np.isfinite(t) & (t >= R_MIN) & (t <= R_MAX)
    lab = np.array(idx)[j[ok]]
    return D[ok] * t[ok][:, None], lab


# ============================================================================ geometry checks
def rect_corners(cx, cy, l, w, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([(cx + dx * c - dy * s, cy + dx * s + dy * c)
                     for dx, dy in ((l / 2, w / 2), (-l / 2, w / 2), (-l / 2, -w / 2), (l / 2, -w / 2))])


def rects_overlap(a, b, margin=0.0):
    A = rect_corners(a[0], a[1], a[2] + 2 * margin, a[3] + 2 * margin, a[4])
    B = rect_corners(b[0], b[1], b[2] + 2 * margin, b[3] + 2 * margin, b[4])
    for P in (A, B):
        for i in range(2):
            e = P[i + 1] - P[i]
            ax = np.array([-e[1], e[0]])
            pa, pb = A @ ax, B @ ax
            if pa.max() < pb.min() or pb.max() < pa.min():
                return False
    return True


# ============================================================================ scene generation
def generate_scene(sc, out_root, D, gz, gen_sha):
    name = sc["name"]
    sd = os.path.join(out_root, name)
    if os.path.lexists(sd):
        if not os.path.isfile(os.path.join(sd, MARKER)):
            raise SystemExit("refusing to replace %s (no %s marker)" % (sd, MARKER))
        shutil.rmtree(sd)
    os.makedirs(os.path.join(sd, "lidar"))
    open(os.path.join(sd, MARKER), "w").write(LABEL + "\n")
    scans = scan_times(sc)
    T_ms = scans[-1][1] + 2
    ego = sc["ego"].build(T_ms)
    objs = sc["objects"]
    for o in objs:
        o["motion"].build(T_ms)
    ground_s = gz - SENSOR[2]
    t0_ms = (T0_EPOCH_S + 100 * int(name[3:5])) * 1000
    frames, gt_rows, comp, truth, labels, collisions = [], [], [], [], [], []
    vel_mem = []      # in-memory exact velocity check rows
    for idx, (scan_no, ms) in enumerate(scans):
        e = ego.state(ms)
        c, s = math.cos(e["psi"]), math.sin(e["psi"])
        stem = "live_%06d" % scan_no
        npy = os.path.join(sd, "lidar", stem + ".npy")
        tot_ms = t0_ms + ms
        t_str = "%d.%06d" % (tot_ms // 1000, (tot_ms % 1000) * 1000)
        stamp_ns = tot_ms * 1000000
        boxes, ost = [], []
        for o in objs:
            m = o["motion"]
            q = m.state(ms)
            dx, dy = q["x"] - e["x"], q["y"] - e["y"]
            bx, by = c * dx + s * dy, -s * dx + c * dy
            vwx, vwy = q["v"] * math.cos(q["psi"]), q["v"] * math.sin(q["psi"])
            vbx, vby = c * vwx + s * vwy, -s * vwx + c * vwy
            yaw = wrap(q["psi"] - e["psi"])
            ost.append({"q": q, "bx": bx, "by": by, "bz": gz + 0.5 * o["h"], "yaw": yaw, "vbx": vbx, "vby": vby})
            boxes.append((bx - SENSOR[0], by - SENSOR[1], gz + 0.5 * o["h"] - SENSOR[2], yaw, o["l"], o["w"], o["h"]))
            # exact-velocity check on the 1 ms trajectory (central difference, world frame)
            if 1 <= ms < m.T:
                fx = (m.X[ms + 1] - m.X[ms - 1]) / 0.002
                fy = (m.Y[ms + 1] - m.Y[ms - 1]) / 0.002
                switch = any(ms in (p[0], p[1]) for p in m.phases)
                vel_mem.append((abs(fx - vwx), abs(fy - vwy), switch))
        P, lab = raycast(D, ground_s, boxes)
        n_ray = np.bincount(lab[lab >= 0], minlength=len(objs)) if len(lab) else np.zeros(len(objs), int)
        keep = np.ones(len(lab), bool)
        drop_ids = set()
        for i, o in enumerate(objs):
            if scan_no in set(sc["dropouts"].get(o["id"], [])):
                keep &= lab != i
                drop_ids.add(i)
        P, lab = P[keep], lab[keep]
        inten = np.where(lab < 0, INT_GROUND, INT_OBJECT)
        arr = np.column_stack([P, inten]).astype(np.float32)
        np.save(npy, arr)
        labels.append(lab.astype(np.int16))
        # ego footprint + objects collision check (BEV, 0.0 margin)
        rects = [(1.0 * (EGO_FOOTPRINT[0] / 2 - EGO_FOOTPRINT[2]), 0.0, EGO_FOOTPRINT[0], EGO_FOOTPRINT[1], 0.0)]
        rects += [(st["bx"], st["by"], o["l"], o["w"], st["yaw"]) for o, st in zip(objs, ost)]
        for i in range(len(rects)):
            for j in range(i + 1, len(rects)):
                if rects_overlap(rects[i], rects[j]):
                    collisions.append((idx, i, j))
        n_gt = 0
        Pb = P + np.array(SENSOR)
        for i, (o, st) in enumerate(zip(objs, ost)):
            rng = math.hypot(st["bx"], st["by"])
            in_gt = rng <= GT_MAX_RANGE
            q = st["q"]
            if in_gt:
                n_gt += 1
                gt_rows.append([idx, stamp_ns, "gt", o["id"], CLS[o["cls"]], ffmt(st["bx"], 4), ffmt(st["by"], 4),
                                ffmt(st["bz"], 4), ffmt(o["w"], 4), ffmt(o["l"], 4), ffmt(o["h"], 4),
                                ffmt(st["yaw"], 6), ffmt(st["vbx"], 4), ffmt(st["vby"], 4), ffmt(q["x"], 4),
                                ffmt(q["y"], 4), ffmt(wrap(q["psi"]), 6)])
            sel = lab == i
            nv = int(sel.sum())
            mean = Pb[sel].mean(axis=0) if nv else (float("nan"),) * 3
            truth.append([idx, o["id"], nv] + [ffmt(v, 6) if nv else "nan" for v in mean] +
                         [ffmt(st["bx"], 6), ffmt(st["by"], 6), ffmt(st["yaw"], 9), ffmt(st["vbx"], 6),
                          ffmt(st["vby"], 6), ffmt(e["v"], 6), ffmt(e["omega"], 6),
                          stem, stamp_ns, CLS[o["cls"]], ffmt(st["bz"], 6), ffmt(o["l"], 4), ffmt(o["w"], 4),
                          ffmt(o["h"], 4), ffmt(q["x"], 6), ffmt(q["y"], 6), ffmt(wrap(q["psi"]), 9),
                          ffmt(rng, 6), ffmt(math.degrees(math.atan2(st["by"], st["bx"])), 4), int(in_gt),
                          int(i in drop_ids), int(n_ray[i]), ffmt(e["a"], 6)])
        dtp = "" if idx == 0 else "%.6f" % ((ms - scans[idx - 1][1]) / 1000.0)
        ex, ey, ez = ffmt(e["x"], 6), ffmt(e["y"], 6), ffmt(-gz, 6)
        eyaw, espd = ffmt(wrap(e["psi"]), 9), ffmt(e["v"], 6)
        frames.append([idx, stem, t_str, stamp_ns, dtp, npy, ex, ey, ez, eyaw, espd, n_gt, 0])
        comp.append([idx, stem, stamp_ns, npy, ffmt(e["v"], 6), eyaw, "nan", "nan", "nan",
                     ex, ey, ez, espd, ffmt(0.0, 6), "nan", "nan", "nan", "nan"])

    def wcsv(fn, cols, rows):
        p = os.path.join(sd, fn)
        with open(p, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(cols)
            w.writerows(rows)
        return p
    fpath = wcsv("frames.csv", FRAMES_COLS, frames)
    gpath = wcsv("gt.csv", GT_COLS, gt_rows)
    cpath = wcsv("frames_comp.csv", COMP_ALLOWED + COMP_FORBIDDEN, comp)
    tpath = wcsv("truth_visible.csv", TRUTH_COLS, truth)
    dts = [(scans[k][1] - scans[k - 1][1]) / 1000.0 for k in range(1, len(scans))]
    man = {"split": "synthetic", "role": "synthetic", "n_frames": len(frames), "n_gt_boxes": len(gt_rows),
           "n_ignore_boxes": 0, "n_gt_empty_frames": sum(1 for f in frames if f[11] == 0),
           "n_box_rows_valid_but_not_lidar_primary": 0,
           "dt_min_s": min(dts), "dt_max_s": max(dts), "n_dt_nonpositive": sum(1 for d in dts if d <= 0),
           "frames_csv": fpath, "frames_csv_sha256": sha_file(fpath), "gt_csv": gpath, "gt_csv_sha256": sha_file(gpath)}
    npys = [f[5] for f in frames]
    agg = hashlib.sha256()
    nbytes = 0
    for p in npys:
        agg.update(sha_file(p).encode())
        nbytes += os.path.getsize(p)
    comp_stats = {"n": len(comp), "vel_x_negative": sum(1 for r in comp if float(r[4]) < 0), "max_abs_vel_y": 0.0,
                  "frames_comp_sha256": sha_file(cpath), "src_frames_sha256": man["frames_csv_sha256"],
                  "role": "synthetic"}
    info = {"description": sc["description"], "duration_s": sc["duration_s"], "time": sc["time"],
            "t0_epoch_s": t0_ms / 1000.0, "n_frames": len(frames), "first_last_stem": [frames[0][1], frames[-1][1]],
            "n_objects": len(objs), "n_vehicles": sum(1 for o in objs if o["cls"] == "vehicle"),
            "n_pedestrians": sum(1 for o in objs if o["cls"] == "pedestrian"),
            "ego": sc["ego"].params(),
            "objects": [{"track_id": o["id"], "class": o["cls"], "class_id": CLS[o["cls"]], "l": o["l"], "w": o["w"],
                         "h": o["h"], "note": o["note"], "motion": o["motion"].params()} for o in objs],
            "dropouts_scan_numbers": {str(k): v for k, v in sorted(sc["dropouts"].items())},
            "n_gt_boxes": len(gt_rows), "points_per_scan": {"min": int(min(len(l) for l in labels)),
                                                            "median": int(np.median([len(l) for l in labels])),
                                                            "max": int(max(len(l) for l in labels))},
            "files": {"frames_csv_sha256": man["frames_csv_sha256"], "gt_csv_sha256": man["gt_csv_sha256"],
                      "frames_comp_csv_sha256": comp_stats["frames_comp_sha256"],
                      "truth_visible_csv_sha256": sha_file(tpath), "npy_count": len(npys), "npy_total_bytes": nbytes,
                      "npy_aggregate_sha256": agg.hexdigest()},
            "bev_overlaps_ego_or_objects": len(collisions)}
    mem = {"labels": labels, "vel_mem": vel_mem, "collisions": collisions, "objs": objs, "scans": scans, "ego": ego}
    return man, comp_stats, info, mem


# ============================================================================ validation
def read_csv(p):
    with open(p, newline="") as f:
        r = csv.reader(f)
        head = next(r)
        rows = list(r)
    return head, rows


def validate_scene(name, sd, mem, real_heads):
    out = {}
    # (3) headers / parsing / consistency
    hf, fr = read_csv(os.path.join(sd, "frames.csv"))
    hg, gr = read_csv(os.path.join(sd, "gt.csv"))
    hc, cr = read_csv(os.path.join(sd, "frames_comp.csv"))
    out["header_equal"] = {"frames.csv": hf == real_heads["frames.csv"], "gt.csv": hg == real_heads["gt.csv"],
                           "frames_comp.csv": hc == real_heads["frames_comp.csv"]}
    ok_parse = all(len(r) == len(hf) for r in fr) and all(len(r) == len(hg) for r in gr) and \
        all(len(r) == len(hc) for r in cr)
    ngt = {}
    for r in gr:
        ngt[int(r[0])] = ngt.get(int(r[0]), 0) + 1
    cons = len(cr) == len(fr) and all(int(f[11]) == ngt.get(int(f[0]), 0) for f in fr) and \
        all(f[0] == c[0] and f[1] == c[1] and f[3] == c[2] and f[5] == c[3] for f, c in zip(fr, cr)) and \
        all(int(f[3]) == int(round(float(f[2]) * 1000)) * 1000000 for f in fr) and \
        all(int(f[0]) == k for k, f in enumerate(fr))
    st = [int(f[3]) for f in fr]
    out["parse_ok"] = bool(ok_parse)
    out["row_consistency_ok"] = bool(cons)
    out["stamps_strictly_increasing"] = all(b > a for a, b in zip(st, st[1:]))
    # (1) object points inside their own GT box (sensor frame), tolerance 2 cm
    objs = mem["objs"]
    gt_by = {}
    for r in gr:
        gt_by.setdefault(int(r[0]), {})[int(r[3])] = [float(v) for v in r[5:12]]   # x y z w l h yaw
    n_obj = n_in = n_in0 = n_ground_bad = n_lab_mismatch = 0
    for f, lab in zip(fr, mem["labels"]):
        k = int(f[0])
        a = np.load(f[5]).astype(np.float64)
        if len(a) != len(lab):
            n_lab_mismatch += 1
            continue
        objm = a[:, 3] == INT_OBJECT
        if not np.array_equal(objm, lab >= 0):
            n_lab_mismatch += 1
        g = a[~objm]
        n_ground_bad += int(np.sum(np.abs(g[:, 2] - g[:, 2].mean()) > 1e-4)) if len(g) else 0
        for i, o in enumerate(objs):
            sel = lab == i
            if not sel.any():
                continue
            x, y, z, w, l, h, yaw = gt_by[k][o["id"]]
            p = a[sel, :3]
            cxs, cys, czs = x - SENSOR[0], y - SENSOR[1], z - SENSOR[2]
            c, s = math.cos(yaw), math.sin(yaw)
            dx, dy = p[:, 0] - cxs, p[:, 1] - cys
            u, v, dz = dx * c + dy * s, -dx * s + dy * c, p[:, 2] - czs
            for tol, cnt in ((0.02, "in"), (1e-4, "in0")):
                inside = (np.abs(u) <= l / 2 + tol) & (np.abs(v) <= w / 2 + tol) & (np.abs(dz) <= h / 2 + tol)
                if cnt == "in":
                    n_in += int(inside.sum())
                else:
                    n_in0 += int(inside.sum())
            n_obj += int(sel.sum())
    out["object_points"] = n_obj
    out["object_points_inside_own_gt_box_tol2cm_frac"] = n_in / n_obj if n_obj else None
    out["object_points_inside_own_gt_box_tol0.1mm_frac"] = n_in0 / n_obj if n_obj else None
    out["ground_points_off_plane_gt_0.1mm"] = n_ground_bad
    out["label_intensity_mismatch_scans"] = n_lab_mismatch
    # (4) velocity: written vx,vy vs finite differences of written gx,gy rotated into ego axes
    yaw_of = {int(f[0]): float(f[9]) for f in fr}
    t_of = {int(f[0]): int(f[3]) * 1e-9 for f in fr}
    ms_of = {k: mem["scans"][k][1] for k in range(len(mem["scans"]))}
    by_tid = {}
    for r in gr:
        by_tid.setdefault(int(r[3]), []).append((int(r[0]), float(r[14]), float(r[15]), float(r[12]), float(r[13])))
    mot = {o["id"]: o["motion"] for o in objs}
    e1, e5, n1, n5 = 0.0, 0.0, 0, 0
    for tid, rows in by_tid.items():
        rows.sort()
        m = mot[tid]
        for a_i, a in enumerate(rows):
            for step, acc in ((1, "1"), (5, "5")):
                if a_i + step >= len(rows):
                    continue
                b = rows[a_i + step]
                if b[0] != a[0] + step or not m.const_velocity(ms_of[a[0]], ms_of[b[0]]):
                    continue
                dt = (ms_of[b[0]] - ms_of[a[0]]) / 1000.0
                vwx, vwy = (b[1] - a[1]) / dt, (b[2] - a[2]) / dt
                yy = yaw_of[a[0]]
                vx, vy = math.cos(yy) * vwx + math.sin(yy) * vwy, -math.sin(yy) * vwx + math.cos(yy) * vwy
                err = max(abs(vx - a[3]), abs(vy - a[4]))
                if acc == "1":
                    e1, n1 = max(e1, err), n1 + 1
                else:
                    e5, n5 = max(e5, err), n5 + 1
    out["vel_fd_csv_1step_const_vel_max_abs_err_mps"] = e1
    out["vel_fd_csv_1step_pairs"] = n1
    out["vel_fd_csv_5step_const_vel_max_abs_err_mps"] = e5
    out["vel_fd_csv_5step_pairs"] = n5
    vm = mem["vel_mem"]
    ns = [max(a, b) for a, b, sw in vm if not sw]
    sw = [max(a, b) for a, b, s_ in vm if s_]
    out["vel_exact_1ms_central_diff_max_abs_err_mps"] = max(ns) if ns else 0.0
    out["vel_exact_1ms_central_diff_at_phase_switch_max_abs_err_mps"] = max(sw) if sw else None
    # ego: vel_x (frames_comp) vs central difference of the 1 ms ego trajectory projected on the heading
    ego = mem["ego"]
    eerr, eerr_sw = 0.0, 0.0
    for k, (_, ms) in enumerate(mem["scans"]):
        if 1 <= ms < ego.T:
            p = ego.psi_at(ms / 1000.0)
            fx = (ego.X[ms + 1] - ego.X[ms - 1]) / 0.002
            fy = (ego.Y[ms + 1] - ego.Y[ms - 1]) / 0.002
            vf = fx * math.cos(p) + fy * math.sin(p)
            vl = -fx * math.sin(p) + fy * math.cos(p)
            e = max(abs(vf - float(cr[k][4])), abs(vl))
            if any(ms in (q[0], q[1]) for q in ego.phases):
                eerr_sw = max(eerr_sw, e)
            else:
                eerr = max(eerr, e)
    out["ego_velx_vs_trajectory_max_abs_err_mps"] = eerr
    out["ego_velx_vs_trajectory_at_phase_switch_max_abs_err_mps"] = eerr_sw
    out["bev_overlaps_ego_or_objects"] = len(mem["collisions"])
    return out


def count_stats(truth_rows_by_scene):
    """Per-object visible point counts at 10/20/40 m (+-2 m), front hemisphere, dropout scans excluded."""
    res = {}
    for rows in truth_rows_by_scene:
        for r in rows:
            if r["dropout"] or r["bx"] <= 0:
                continue
            for c in (10, 20, 40):
                if abs(r["rng"] - c) <= 2.0:
                    res.setdefault("%s_%dm" % (r["cls"], c), []).append(r["n"])
    out = {}
    for k, v in sorted(res.items()):
        a = np.array(v)
        out[k] = {"n": int(len(a)), "median": float(np.median(a)), "p25": float(np.percentile(a, 25)),
                  "p75": float(np.percentile(a, 75)), "frac_zero": float((a == 0).mean())}
    return out


def real_reference():
    """Read-only: per-object point counts (points inside the real GT box, z > bottom+0.15) and points/scan."""
    import pandas as pd  # noqa: E402  (local import: only needed here)
    res, nps = {}, []
    for sc in ["scen01", "scen21", "scen40", "scen93"]:
        fr = pd.read_csv(os.path.join(REAL_INPUTS, sc, "frames.csv"))
        gt = pd.read_csv(os.path.join(REAL_INPUTS, sc, "gt.csv"))
        gt = gt[gt.kind == "gt"]
        by = {k: v for k, v in gt.groupby("idx")}
        for r in fr.itertuples(index=False):
            if r.idx % 3:
                continue
            P = np.load(r.npy).astype(float)
            nps.append(len(P))
            if r.idx not in by:
                continue
            P[:, 0] += SENSOR[0]
            P[:, 2] += SENSOR[2]
            for g in by[r.idx].itertuples(index=False):
                rng = math.hypot(g.x, g.y)
                b = [c for c in (10, 20, 40) if abs(rng - c) <= 2.0]
                if not b or g.x <= 0:
                    continue
                c_, s_ = math.cos(g.yaw), math.sin(g.yaw)
                dx, dy = P[:, 0] - g.x, P[:, 1] - g.y
                u, v = dx * c_ + dy * s_, -dx * s_ + dy * c_
                bot = g.z - g.h / 2
                m = (np.abs(u) <= g.l / 2 + 0.1) & (np.abs(v) <= g.w / 2 + 0.1) & (P[:, 2] >= bot + 0.15) & \
                    (P[:, 2] <= bot + g.h + 0.1)
                res.setdefault("%s_%dm" % ("vehicle" if g.class_id == 0 else "pedestrian", b[0]), []).append(int(m.sum()))
    out = {}
    for k, v in sorted(res.items()):
        a = np.array(v)
        out[k] = {"n": int(len(a)), "median": float(np.median(a)), "p25": float(np.percentile(a, 25)),
                  "p75": float(np.percentile(a, 75)), "frac_zero": float((a == 0).mean())}
    nps = np.array(nps)
    return {"scenes": ["scen01", "scen21", "scen40", "scen93"], "every_nth_frame": 3,
            "points_per_scan": {"n": int(len(nps)), "min": int(nps.min()), "median": float(np.median(nps)),
                                "max": int(nps.max())},
            "per_object_counts": out,
            "method": "points inside the real GT box expanded 0.1 m in l/w, z in [bottom+0.15, top+0.1] (base_link), front (x>0) objects"}


# ============================================================================ main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir")
    ap.add_argument("--scenes", default="")
    ap.add_argument("--ground-z-base", type=float, default=0.0,
                    help="ground height in base_link (default 0: base_link on the ground, as specified)")
    ap.add_argument("--no-validate", action="store_true")
    ap.add_argument("--no-real-reference", action="store_true")
    a = ap.parse_args()
    out = os.path.abspath(a.out_dir)
    for bad in ("/home/autonav/Lidar_Data", REAL_INPUTS):
        if out.startswith(bad):
            raise SystemExit("refusing to write into %s" % bad)
    os.makedirs(out, exist_ok=True)
    gen = os.path.abspath(__file__)
    gen_sha = sha_file(gen)
    D, ncol = ray_dirs()
    S = scenarios()
    if a.scenes:
        want = set(a.scenes.split(","))
        S = [s for s in S if s["name"] in want]
    gz = a.ground_z_base
    real_heads = {}
    for fn in ("frames.csv", "gt.csv", "frames_comp.csv"):
        with open(os.path.join(REAL_SCENE_DIR, fn), newline="") as f:
            real_heads[fn] = next(csv.reader(f))
    manifest = {"tool": gen, "tool_sha256": gen_sha, "scenes": {}, "holdout": [], "rules": __doc__}
    comp_man = {"source_inputs": out, "source_manifest_sha256": None, "allowed_columns": COMP_ALLOWED,
                "forbidden_columns": COMP_FORBIDDEN,
                "doc": "Synthetic competition-restricted stream: vel_x_mps = TRUE body-forward speed of base_link, "
                       "imu_yaw_rad = TRUE heading, imu_roll/pitch/wz = nan (as in the real stream); FORBIDDEN columns "
                       "carry the true ego_x/ego_y/ego_z/signed speed and vel_y = 0 (true lateral); vel_z/accel_* = nan "
                       "(as in the real stream).  " + LABEL,
                "scenes": {}}
    syn = {"label": LABEL, "generator": gen, "generator_sha256": gen_sha, "numpy_version": np.__version__,
           "python_version": platform.python_version(),
           "command": "PYTHONDONTWRITEBYTECODE=1 python3 -B %s %s%s" % (gen, a.out_dir,
                                                                     (" --ground-z-base %g" % gz) if gz else ""),
           "out_dir": out, "conventions": {
               "frames": "base_link = rear-axle centre, ground plane at base_link z = %g m (world ground z=0, ego_z = %g); "
                         "LiDAR at (0.80, 0.00, 1.45) in base_link, rpy 0 -> ground in the sensor frame at z = %g m"
                         % (gz, -gz, gz - SENSOR[2]),
               "world_to_ego": "p_base = R(-ego_yaw) (p_world - ego_xy); p_sensor = p_base - (0.80, 0, 1.45)",
               "gt_box": "x,y = BEV box centre in base_link at t_lidar; z = ground_z_base + h/2; l along heading, w across "
                         "(vehicles l>=w, pedestrians l<w as in the real GT); yaw = heading - ego_yaw wrapped to [-pi,pi); "
                         "vx,vy = GROUND (absolute) velocity of the box centre rotated into base_link axes; gx,gy,yaw_global "
                         "= world centre/heading; kind=gt; class_id 0 vehicle, 1 pedestrian; GT rows for every object whose "
                         "BEV centre is within %.0f m of base_link (independent of visibility, also during dropouts)" % GT_MAX_RANGE,
               "frames_csv": "ego_x, ego_y = base_link world position; ego_z = %g; ego_yaw = TRUE heading wrapped to [-pi,pi) "
                             "(%%.9f); ego_speed_mps = TRUE signed speed (real file: signed central difference); t_lidar on "
                             "the 1 ms grid, stamp_ns = t_lidar_ms * 1e6; dt_prev_s = t_lidar difference; n_ignore = 0" % (-gz),
               "frames_comp_csv": comp_man["doc"],
               "scan_timing": "15 Hz nominal: scan k at round(1000 k / 15) ms (dt 66/67 ms, mean exactly 1/15 s) so stamps "
                              "stay on the real data's millisecond grid; all truth is evaluated at that exact quantised time",
               "kinematics": "ego: unicycle on base_link (rear axle); objects: unicycle on the BEV box centre; piecewise-"
                             "constant (a, omega) with switch times on the 1 ms grid; v, psi analytic, x, y Simpson on 1 ms",
               "observation": "ideal: no noise, no motion distortion (whole scan at the t_lidar pose), nearest of ground "
                              "and boxes, 1.0 m <= range <= 55.0 m, intensity ground 10 / objects 50",
               "dropouts": "syn08: all points of the object removed in the listed scans (no re-cast behind it), GT kept",
               "time_epoch": "t0 = 1800000000 s + 100 s * scenario number"},
           "beam_pattern": {
               "synthetic": {"rings": len(ELEV_DEG), "elevation_deg": ELEV_DEG, "azimuth_step_deg": AZ_STEP_DEG,
                             "azimuth_fov_deg": [-AZ_HALF_FOV_DEG, AZ_HALF_FOV_DEG], "columns": ncol,
                             "rays_per_scan": int(len(D)), "range_m": [R_MIN, R_MAX],
                             "point_order": "column-major: azimuth 0, -0.2, ..., -134.6, +134.6, ..., +0.2; rings bottom->top"},
               "real_inferred": {"rings": 16, "apparent_elevation_deg_far_field_and_ground":
                                 [-14.61, -12.67, -10.72, -8.78, -6.83, -4.88, -2.93, -0.95, 1.04, 3.04, 5.14, 7.09,
                                  9.20, 11.19, 13.14, 15.31],
                                 "interpretation": "VLP-16 nominal -15..+15 step 2 deg; the apparent angles from the npy "
                                                   "origin are consistent with a ~4-5 cm vertical beam-origin offset "
                                                   "(range dependent: near objects show +0.2..0.3 deg)",
                                 "azimuth_step_deg": 0.2, "azimuth_fov_deg": [-134.6, 134.6],
                                 "point_order": "column-major from azimuth 0 clockwise, wrap at the rear gap, rings bottom->top",
                                 "max_range_m": "54-55.35 (3D)", "min_range_m": "<= 2.3 (objects); ground first hit ~6.6-6.8 m",
                                 "points_per_scan": "~9.3k-18k (p50 ~10k)", "intensity": "0-255, ground median ~28, objects ~100",
                                 "ground_in_sensor_frame_m": "~ -1.72 .. -1.80 (GT box bottoms ~ -0.35 m in base_link)"}},
           "scenarios": {}}
    vals, truth_stats = {}, []
    for sc in S:
        man, cst, info, mem = generate_scene(sc, out, D, gz, gen_sha)
        manifest["scenes"][sc["name"]] = man
        comp_man["scenes"][sc["name"]] = cst
        syn["scenarios"][sc["name"]] = info
        print("%-30s frames=%4d objects=%2d gt=%5d pts/scan min/med/max=%d/%d/%d overlaps=%d" % (
            sc["name"], man["n_frames"], info["n_objects"], man["n_gt_boxes"], info["points_per_scan"]["min"],
            info["points_per_scan"]["median"], info["points_per_scan"]["max"], info["bev_overlaps_ego_or_objects"]),
            flush=True)
        if not a.no_validate:
            v = validate_scene(sc["name"], os.path.join(out, sc["name"]), mem, real_heads)
            vals[sc["name"]] = v
            rows = []
            with open(os.path.join(out, sc["name"], "truth_visible.csv"), newline="") as f:
                for r in csv.DictReader(f):
                    rows.append({"cls": "vehicle" if r["class_id"] == "0" else "pedestrian",
                                 "n": int(r["n_points_visible"]), "rng": float(r["range_base_m"]),
                                 "bx": float(r["box_center_x"]), "dropout": r["dropout"] == "1"})
            truth_stats.append(rows)
    tot = manifest["scenes"].values()
    manifest["totals"] = {"n_scenes": len(manifest["scenes"]), "n_frames": sum(v["n_frames"] for v in tot),
                          "n_gt_boxes": sum(v["n_gt_boxes"] for v in tot),
                          "n_ignore_boxes": sum(v["n_ignore_boxes"] for v in tot),
                          "by_role": {"synthetic": {"scenes": len(manifest["scenes"]),
                                                    "frames": sum(v["n_frames"] for v in tot)}}}
    mp = os.path.join(out, "INPUT_MANIFEST.json")
    with open(mp, "w") as f:
        json.dump(manifest, f, indent=1)
    comp_man["source_manifest_sha256"] = sha_file(mp)
    with open(os.path.join(out, "COMP_INPUT_MANIFEST.json"), "w") as f:
        json.dump(comp_man, f, indent=1)
    syn["input_manifest_sha256"] = comp_man["source_manifest_sha256"]
    syn["totals"] = manifest["totals"]
    with open(os.path.join(out, "SYNTHETIC_MANIFEST.json"), "w") as f:
        json.dump(syn, f, indent=1)
    if not a.no_validate:
        agg = {"label": LABEL, "per_scene": vals}
        fr = [v["object_points_inside_own_gt_box_tol2cm_frac"] for v in vals.values() if v["object_points"]]
        npts = sum(v["object_points"] for v in vals.values())
        agg["summary"] = {
            "object_points_total": npts,
            "inside_own_gt_box_tol2cm_frac_min_scene": min(fr) if fr else None,
            "inside_own_gt_box_tol2cm_frac_overall": sum(v["object_points_inside_own_gt_box_tol2cm_frac"] * v["object_points"]
                                                         for v in vals.values() if v["object_points"]) / npts if npts else None,
            "inside_own_gt_box_tol0.1mm_frac_overall": sum(v["object_points_inside_own_gt_box_tol0.1mm_frac"] * v["object_points"]
                                                           for v in vals.values() if v["object_points"]) / npts if npts else None,
            "headers_equal_all": all(all(v["header_equal"].values()) for v in vals.values()),
            "parse_and_consistency_all": all(v["parse_ok"] and v["row_consistency_ok"] and v["stamps_strictly_increasing"]
                                             for v in vals.values()),
            "vel_fd_csv_1step_max_abs_err_mps": max(v["vel_fd_csv_1step_const_vel_max_abs_err_mps"] for v in vals.values()),
            "vel_fd_csv_5step_max_abs_err_mps": max(v["vel_fd_csv_5step_const_vel_max_abs_err_mps"] for v in vals.values()),
            "vel_exact_max_abs_err_mps": max(v["vel_exact_1ms_central_diff_max_abs_err_mps"] for v in vals.values()),
            "ego_velx_max_abs_err_mps": max(v["ego_velx_vs_trajectory_max_abs_err_mps"] for v in vals.values()),
            "bev_overlaps_total": sum(v["bev_overlaps_ego_or_objects"] for v in vals.values()),
            "ground_points_off_plane_total": sum(v["ground_points_off_plane_gt_0.1mm"] for v in vals.values()),
            "rounding_note": "gt.csv keeps the real %.4f format: gx/gy rounding (<=5e-5 m) bounds the 1-step FD error by "
                             "~1e-4/dt = 1.5e-3 m/s; the 5-step (>=0.33 s) FD bound is ~3e-4 m/s; the exact (unrounded) "
                             "check is the 1 ms trajectory derivative"}
        agg["synthetic_per_object_counts"] = count_stats(truth_stats)
        agg["synthetic_points_per_scan"] = {k: v["points_per_scan"] for k, v in syn["scenarios"].items()}
        if not a.no_real_reference:
            agg["real_reference"] = real_reference()
        du = 0
        for root, _, files in os.walk(out):
            for fn in files:
                du += os.path.getsize(os.path.join(root, fn))
        agg["disk_bytes_out_dir"] = du
        with open(os.path.join(out, "VALIDATION.json"), "w") as f:
            json.dump(agg, f, indent=1)
        print(json.dumps(agg["summary"], indent=1))
        print(json.dumps(agg["synthetic_per_object_counts"], indent=1))
        if "real_reference" in agg:
            print(json.dumps(agg["real_reference"], indent=1))
        print("disk_bytes_out_dir", du)


if __name__ == "__main__":
    main()
