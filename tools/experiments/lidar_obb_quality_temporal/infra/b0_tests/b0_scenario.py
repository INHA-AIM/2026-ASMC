#!/usr/bin/env python3
"""B0 harness core: scenario -> timed input events, synthetic geometry, and an independent CONTRACT ORACLE.

Pure Python (no ROS import) so it can be self-tested offline:
    python3 -I b0_scenario.py --catalogue ../fault_catalogue.json --selftest
The oracle re-implements the documented gating rules (not the C++ code paths) so that the truth table compares
Main's observed per-scan outputs against an independent expectation:
  LiDAR accept   Parser.cpp:272-275 (zero stamp), :311-316 (duplicate/regressing)
  IMU accept     Parser.cpp:592-596 (b_ImuReady), SensorPoseHistory.cpp:110-160 (barriers, backwards, yaw jump)
  CAN accept     Parser.cpp:523-525 (zero stamp -> NaN), SensorPoseHistory.cpp:162-207
  local lookup   SensorPoseHistory.cpp:306-357 (at-or-before, 0.12 s, invalid barriers)
  GPS sample     Parser.cpp:414-461 (status/stamp/finite/offsets/(0,0)/sanity), SensorPoseHistory.cpp:69-108 (IMU gate)
  sanity filter  GpsSanityFilter.cpp:9-60
  map pose       bounded_pose.cpp:32-72 (arrival cut, barrier, 0.30 s, 3 consecutive), SensorPoseHistory.cpp:402-411 (speed)
"""
import argparse
import json
import math
import struct

ORIGIN_E, ORIGIN_N = 302595.0, 4124145.0      # main.launch:234-235; MapOffsetsValid (test_frame_boundary_ros.cpp:156-157)
LIDAR_IN_BASE = (0.80, 0.0, 1.45)            # RUN_SENSOR_FUSION.md:19; test_frame_boundary_ros.cpp:17
GPS_IN_BASE = (0.35, 0.0, 1.25)              # RUN_SENSOR_FUSION.md:20; test_frame_boundary_ros.cpp:18
SANITY_BOUNDS = (-454.5159, -825.0458, 162.4841, 621.3542)   # main.launch:209-212
MS = 1000000

# ---------------------------------------------------------------- UTM zone 52 (Parser.cpp:425-435 uses GeographicLib)
_A_WGS, _F = 6378137.0, 1 / 298.257223563
_K0, _LON0 = 0.9996, math.radians(129.0)
_n = _F / (2 - _F)
_AA = _A_WGS / (1 + _n) * (1 + _n ** 2 / 4 + _n ** 4 / 64)
_AL = (_n / 2 - 2 * _n ** 2 / 3 + 5 * _n ** 3 / 16, 13 * _n ** 2 / 48 - 3 * _n ** 3 / 5, 61 * _n ** 3 / 240)
_BE = (_n / 2 - 2 * _n ** 2 / 3 + 37 * _n ** 3 / 96, _n ** 2 / 48 + _n ** 3 / 15, 17 * _n ** 3 / 480)
_DE = (2 * _n - 2 * _n ** 2 / 3 - 2 * _n ** 3, 7 * _n ** 2 / 3 - 8 * _n ** 3 / 5, 56 * _n ** 3 / 15)


def utm_forward(lat_deg, lon_deg):
    phi, lam = math.radians(lat_deg), math.radians(lon_deg) - _LON0
    c = 2 * math.sqrt(_n) / (1 + _n)
    t = math.sinh(math.atanh(math.sin(phi)) - c * math.atanh(c * math.sin(phi)))
    xi, eta = math.atan2(t, math.cos(lam)), math.atanh(math.sin(lam) / math.sqrt(1 + t * t))
    e = 500000 + _K0 * _AA * (eta + sum(a * math.cos(2 * (j + 1) * xi) * math.sinh(2 * (j + 1) * eta) for j, a in enumerate(_AL)))
    n = _K0 * _AA * (xi + sum(a * math.sin(2 * (j + 1) * xi) * math.cosh(2 * (j + 1) * eta) for j, a in enumerate(_AL)))
    return e, n


def utm_inverse(e, n):
    xi, eta = n / (_K0 * _AA), (e - 500000) / (_K0 * _AA)
    xp = xi - sum(b * math.sin(2 * (j + 1) * xi) * math.cosh(2 * (j + 1) * eta) for j, b in enumerate(_BE))
    ep = eta - sum(b * math.cos(2 * (j + 1) * xi) * math.sinh(2 * (j + 1) * eta) for j, b in enumerate(_BE))
    chi = math.asin(math.sin(xp) / math.cosh(ep))
    lat = math.degrees(chi + sum(d * math.sin(2 * (j + 1) * chi) for j, d in enumerate(_DE)))
    lon = math.degrees(_LON0 + math.atan2(math.sinh(ep), math.cos(xp)))
    for _ in range(3):   # Newton polish so utm_forward(utm_inverse(x)) == x to < 1e-6 m
        fe, fn = utm_forward(lat, lon)
        lat += (n - fn) / 111132.0
        lon += (e - fe) / (111320.0 * math.cos(math.radians(lat)))
    return lat, lon


def map_to_latlon(x, y):
    return utm_inverse(ORIGIN_E + x, ORIGIN_N + y)


# ---------------------------------------------------------------- rotations
def rpy(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def mul(m, v):
    return [m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2], m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2]]


def mul_t(m, v):
    return [m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2], m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2],
            m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2]]


def quat_from_rpy(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2),
                              math.cos(yaw / 2), math.sin(yaw / 2))
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# ---------------------------------------------------------------- scene
class Scene:
    def __init__(self, scenario, defaults):
        ego = dict(speed_mps=defaults.get('ego_speed_mps', 0.0), yaw_rate_rps=0.0, roll_deg=0.0, pitch_deg=0.0)
        ego.update(scenario['phases'][0].get('ego', {}))
        self.ego, self.x0 = ego, defaults['ego_start_map_xy']
        self.yaw0 = defaults['ego_yaw_rad']
        self.objdefs = defaults['objects']

    def ego_pose(self, t):
        w, v = self.ego['yaw_rate_rps'], self.ego['speed_mps']
        yaw = self.yaw0 + w * t
        if abs(w) < 1e-9:
            x, y = self.x0[0] + v * t * math.cos(self.yaw0), self.x0[1] + v * t * math.sin(self.yaw0)
        else:
            x = self.x0[0] + v / w * (math.sin(yaw) - math.sin(self.yaw0))
            y = self.x0[1] - v / w * (math.cos(yaw) - math.cos(self.yaw0))
        return x, y, yaw, math.radians(self.ego['roll_deg']), math.radians(self.ego['pitch_deg'])

    def object_truth(self, oid, t):
        d = self.objdefs[oid]
        c0, s0 = math.cos(self.yaw0), math.sin(self.yaw0)
        ex, ey = d['ego_frame_xy']
        yaw = self.yaw0 + d['yaw_rel']
        x = self.x0[0] + c0 * ex - s0 * ey + d['speed_mps'] * t * math.cos(yaw)
        y = self.x0[1] + s0 * ex + c0 * ey + d['speed_mps'] * t * math.sin(yaw)
        l, w, h = d['size_lwh']
        return (x, y, h / 2.0), yaw, (l, w, h)

    def to_base(self, p, t):
        x, y, yaw, roll, pitch = self.ego_pose(t)
        return mul_t(rpy(roll, pitch, yaw), [p[0] - x, p[1] - y, p[2]])

    def cloud(self, oids, t, step=0.15, ground_step=0.5):
        x, y, yaw, roll, pitch = self.ego_pose(t)
        R = rpy(roll, pitch, yaw)
        pts = []
        u = 2.0
        while u <= 40.0:        # ground ahead of the ego, world z = 0
            v = -15.0
            while v <= 15.0:
                pw = [x + math.cos(yaw) * u - math.sin(yaw) * v, y + math.sin(yaw) * u + math.cos(yaw) * v, 0.0]
                pts.append((pw, 0.0))
                v += ground_step
            u += ground_step
        for oid in oids:
            (cx, cy, _), oyaw, (l, w, h) = self.object_truth(oid, t)
            for bx, by, bz in _box_surface(l, w, h, step):
                pts.append(([cx + math.cos(oyaw) * bx - math.sin(oyaw) * by, cy + math.sin(oyaw) * bx + math.cos(oyaw) * by, bz], 1.0))
        out = []
        for pw, inten in pts:
            pb = mul_t(R, [pw[0] - x, pw[1] - y, pw[2]])
            out.append((pb[0] - LIDAR_IN_BASE[0], pb[1] - LIDAR_IN_BASE[1], pb[2] - LIDAR_IN_BASE[2], inten))
        return out

    def truth(self, oids, t):
        x, y, yaw, roll, pitch = self.ego_pose(t)
        res = {}
        for oid in oids:
            c, oyaw, dims = self.object_truth(oid, t)
            res[oid] = dict(map=list(c), base=self.to_base(c, t), yaw_map=wrap(oyaw), yaw_base=wrap(oyaw - yaw), dims=list(dims))
        return res

    def gps_fix(self, t):
        x, y, yaw, roll, pitch = self.ego_pose(t)
        g = mul(rpy(roll, pitch, yaw), list(GPS_IN_BASE))
        lat, lon = map_to_latlon(x + g[0], y + g[1])
        return lat, lon, g[2]          # altitude: base z = 0, map_altitude_origin_m default 0.0 (main.launch:8)


def _box_surface(l, w, h, step):
    def lin(a, b, n):
        return [a + (b - a) * i / (n - 1) for i in range(n)]
    xs, ys, zs = lin(-l / 2, l / 2, max(2, int(l / step) + 1)), lin(-w / 2, w / 2, max(2, int(w / step) + 1)), lin(0, h, max(2, int(h / step) + 1))
    pts = [(a, b, h) for a in xs for b in ys]
    pts += [(a, s * w / 2, z) for a in xs for z in zs for s in (-1, 1)]
    pts += [(s * l / 2, b, z) for b in ys for z in zs for s in (-1, 1)]
    return pts


def pack_cloud(points):
    return b''.join(struct.pack('<ffff', *p) for p in points)


# ---------------------------------------------------------------- scenario -> events
def ticks(scenario):
    k = 0
    for p, ph in enumerate(scenario['phases']):
        for i in range(ph['scans']):
            yield k, p, i, ph
            k += 1


def plan(scenario, defaults, t0_ns):
    """G4: events in PUBLICATION order. Each sample is published 2 ms after its nominal source time (IMU S-40/S-5,
    GPS S-30, CAN S-5, LiDAR S+12), like a real sensor bridge. The design version published every input of a tick
    at once at S+2 ms on separate TCPROS connections, so Main could process GPS before the older IMU sample
    and fail the GPS IMU gate (SensorPoseHistory.cpp AddGps) on a callback race (first G4 trial run)."""
    tick, buf = None, []
    for e in _plan_unsorted(scenario, defaults, t0_ns):
        if e['k'] != tick and buf:
            for x in sorted(buf, key=lambda x: x['pub_ns']):
                yield x
            buf = []
        tick = e['k']
        buf.append(e)
    for x in sorted(buf, key=lambda x: x['pub_ns']):
        yield x


def _pub(e, nominal_ns):
    e['pub_ns'] = nominal_ns + 2 * MS
    return e


def _plan_unsorted(scenario, defaults, t0_ns):
    """Yield the input events of one scenario in publication order (no ROS). Each event is a dict with
    ev, k, phase, i, mode, stamp_ns, t_truth (s, nominal scenario time) and payload fields.
    Publication timing: the feeder publishes all inputs of tick k at wall >= S_k + 2 ms, LiDAR 'lidar_after_inputs_ms' later."""
    per = int(round(defaults['period_s'] * 1e9))
    scene = Scene(scenario, defaults)
    last_lidar = 0
    first_in_phase = {}
    can_speed = scene.ego['speed_mps']        # G4: reported CAN speed ramps toward a phase 'can_speed_mps'
    for k, p, i, ph in ticks(scenario):
        s = t0_ns + k * per
        t = (s - t0_ns) / 1e9
        x, y, yaw, roll, pitch = scene.ego_pose(t)
        imu_mode, can_mode, lidar_mode = ph.get('imu', 'good'), ph.get('can', 'good'), ph.get('lidar', 'good')
        gps_mode = ph.get('gps', 'good')
        if 'gps_first_n' in ph and i >= ph['gps_first_n']:
            gps_mode = ph.get('gps_then', 'status0')
        first = first_in_phase.setdefault(p, k) == k
        # IMU (published first: GPS validity needs an IMU at/before the GPS stamp, SensorPoseHistory.cpp:89-99)
        if imu_mode != 'silent':
            for off in defaults['imu_offsets_ms']:
                st = s + off * MS
                if imu_mode in ('stale', 'behind_clock'):
                    st -= int(round(ph.get('imu_lag_s', 0.2) * 1e9))
                tt = t + off / 1000.0
                q = quat_from_rpy(roll, pitch, scene.ego_pose(tt)[2] + (0.5 if (imu_mode == 'yaw_jump' and first and off == defaults['imu_offsets_ms'][0]) else 0.0))
                if imu_mode == 'quat_invalid':
                    q = (0.0, 0.0, 0.0, 0.0)
                yield _pub(dict(ev='imu', k=k, phase=p, i=i, mode=imu_mode, stamp_ns=0 if imu_mode == 'stamp0' else st,
                                quat=q, yaw=scene.ego_pose(tt)[2], yaw_rate=scene.ego['yaw_rate_rps'], t_truth=tt),
                           s + off * MS)
        # GPS
        if gps_mode != 'silent':
            st = s + defaults['gps_offset_ms'] * MS
            tt = t + defaults['gps_offset_ms'] / 1000.0
            if gps_mode in ('stale_stamp', 'future_stamp'):
                st -= int(round(ph.get('gps_lag_s', 0.5) * 1e9))
            lat, lon, alt = scene.gps_fix(tt)
            eo, no, status = ORIGIN_E, ORIGIN_N, defaults['gps_status_good']
            if gps_mode == 'status0':
                status = 0
            elif gps_mode == 'latlon0':
                lat = lon = 0.0
            elif gps_mode == 'nan':
                lat = float('nan')
            elif gps_mode == 'offset_bad':
                eo += 1.0
            elif gps_mode == 'jump':
                xx, yy = x + ph.get('gps_jump_m', 60.0), y
                lat, lon = map_to_latlon(xx, yy)
            yield _pub(dict(ev='gps', k=k, phase=p, i=i, mode=gps_mode, stamp_ns=0 if gps_mode == 'stamp0' else st,
                            lat=lat, lon=lon, alt=alt, east_offset=eo, north_offset=no, status=status, t_truth=tt),
                       s + defaults['gps_offset_ms'] * MS)
        # CAN (/Ego_topic, main.launch:125 remap)
        if can_mode != 'silent':
            st = s + defaults['can_offset_ms'] * MS
            if can_mode == 'stale':
                st -= int(round(ph.get('can_lag_s', 0.2) * 1e9))
            # G4 (S28): ramp by <= 2.4 m/s per 0.1 s tick, inside the CAN jump gate 1 m/s + 15 m/s^2 * dt
            # (SensorPoseHistory.cpp AddCan), so a speed above 35 m/s is accepted as a plausible sample.
            target = ph.get('can_speed_mps', scene.ego['speed_mps'])
            can_speed = min(target, can_speed + 2.4) if target >= can_speed else max(target, can_speed - 2.4)
            v = can_speed + (10.0 if (can_mode == 'speed_jump' and first) else 0.0)
            yield _pub(dict(ev='can', k=k, phase=p, i=i, mode=can_mode, stamp_ns=0 if can_mode == 'stamp0' else st, speed=v,
                            t_truth=t + defaults['can_offset_ms'] / 1000.0), s + defaults['can_offset_ms'] * MS)
        if scenario.get('yolo_stub'):
            yield _pub(dict(ev='yolo', k=k, phase=p, i=i, mode='empty', stamp_ns=s - 5 * MS, t_truth=t), s - 4 * MS)
        # LiDAR
        if lidar_mode != 'silent':
            st = s
            if lidar_mode == 'stamp0':
                st = 0
            elif lidar_mode == 'duplicate':
                st = last_lidar
            elif lidar_mode == 'regress':
                st = last_lidar - 50 * MS
            elif lidar_mode == 'sim_epoch':
                st = 100 * 1000 * MS + k * per
            if lidar_mode == 'good' or lidar_mode == 'sim_epoch':
                last_lidar = st
            oids = ph.get('objects', [])
            yield _pub(dict(ev='lidar', k=k, phase=p, i=i, mode=lidar_mode, stamp_ns=st, nominal_ns=s, objects=oids,
                            t_truth=t, truth=scene.truth(oids, t)), s + defaults['lidar_after_inputs_ms'] * MS)


# ---------------------------------------------------------------- contract oracle
class Oracle:
    def __init__(self, cp):
        self.gps_age = int(cp.get('gps_max_age_s', 0.30) * 1e9)
        self.imu_age, self.can_age = cp.get('imu_max_age_s', 0.12), cp.get('can_max_age_s', 0.12)
        self.recovery, self.vmax = cp.get('recovery_samples', 3), cp.get('max_speed_mps', 35.0)
        self.future_lead = int(cp.get('gps_maximum_future_lead_s', 0.10) * 1e9)
        self.imu, self.can, self.gps = [], [], []        # accepted (stamp_ns, value)
        self.inv_imu = self.inv_can = 0
        self.imu_seen_ms = self.can_seen_ms = 0
        self.imu_acc = self.can_acc = None              # (stamp_ms, yaw|speed)
        self.san = None                                  # (x, y, t_s)
        self.last_lidar = 0

    @staticmethod
    def _src(ev, recv_ns):
        return ev['stamp_ns'] if ev['stamp_ns'] else recv_ns    # header zero -> ros::Time::now() (Parser.cpp:529,598)

    def add_imu(self, ev, recv_ns):
        q = ev['quat']
        norm = sum(c * c for c in q)
        st_ms = self._src(ev, recv_ns) // MS
        ready = ev['stamp_ns'] != 0 and math.isfinite(norm) and abs(norm - 1.0) <= 1e-3
        if not ready:
            self.inv_imu = max(self.inv_imu, ev['stamp_ns'] or st_ms * MS)
            return 'invalid'
        if self.imu_seen_ms and st_ms < self.imu_seen_ms:
            return 'backwards'
        gap = (st_ms - self.imu_seen_ms) / 1000.0 if self.imu_seen_ms else 0.0
        self.imu_seen_ms = st_ms
        if self.imu_acc and gap < 2.0:
            if st_ms < self.imu_acc[0]:
                return 'backwards'
            allowed = 0.0872664626 + 2.0943951024 * (st_ms - self.imu_acc[0]) / 1000.0
            if abs(wrap(ev['yaw'] - self.imu_acc[1])) > allowed:
                self.inv_imu = max(self.inv_imu, ev['stamp_ns'] or st_ms * MS)
                return 'yaw_jump'
        self.imu.append((ev['stamp_ns'], ev['yaw']))
        self.imu.sort()
        self.imu_acc = (st_ms, ev['yaw'])
        return 'accepted'

    def add_can(self, ev, recv_ns):
        st_ms = self._src(ev, recv_ns) // MS
        speed = ev['speed'] if ev['stamp_ns'] else float('nan')   # Parser.cpp:523-525
        if not math.isfinite(speed) or speed < 0:
            self.inv_can = max(self.inv_can, ev['stamp_ns'] or st_ms * MS)
            return 'invalid'
        if self.can_seen_ms and st_ms < self.can_seen_ms:
            return 'backwards'
        gap = (st_ms - self.can_seen_ms) / 1000.0 if self.can_seen_ms else 0.0
        self.can_seen_ms = st_ms
        if self.can_acc and gap < 2.0:
            if st_ms < self.can_acc[0]:
                return 'backwards'
            if abs(speed - self.can_acc[1]) > 1.0 + 15.0 * (st_ms - self.can_acc[0]) / 1000.0:
                self.inv_can = max(self.inv_can, ev['stamp_ns'] or st_ms * MS)
                return 'speed_jump'
        self.can.append((ev['stamp_ns'], speed))
        self.can.sort()
        self.can_acc = (st_ms, speed)
        return 'accepted'

    def add_gps(self, ev, arrival_ns, receipt_ros_ns):
        st = ev['stamp_ns']
        vals = (ev['lat'], ev['lon'], ev['alt'], ev['east_offset'], ev['north_offset'])
        bad = (ev['status'] == 0 or st == 0 or not all(math.isfinite(v) for v in vals) or
               abs(ev['east_offset'] - ORIGIN_E) > 1e-6 or abs(ev['north_offset'] - ORIGIN_N) > 1e-6 or
               (ev['lat'] == 0.0 and ev['lon'] == 0.0))
        if bad:
            self.gps.append(dict(stamp=st, arrival=arrival_ns, valid=False, why='input_invalid'))
            return 'invalid'
        # A source stamp materially ahead of callback receipt time is corrupt clock data. It is an
        # immediate unknown-time invalid barrier and, crucially, does not advance the sanity anchor.
        if st > receipt_ros_ns + self.future_lead:
            self.gps.append(dict(stamp=0, arrival=arrival_ns, valid=False, why='future_stamp'))
            return 'future_stamp'
        e, n = utm_forward(ev['lat'], ev['lon'])
        x, y, ts = e - ORIGIN_E, n - ORIGIN_N, st / 1e9
        ok, why = True, 'continuous'
        if not (SANITY_BOUNDS[0] <= x <= SANITY_BOUNDS[2] and SANITY_BOUNDS[1] <= y <= SANITY_BOUNDS[3]):
            ok, why = False, 'bounds'
        elif self.san is None:
            self.san, why = (x, y, ts), 'initial'
        else:
            el = ts - self.san[2]
            if el < -1e-3:
                ok, why = False, 'backwards'
            elif el >= 2.0:
                self.san, why = (x, y, ts), 'reanchor'
            elif math.hypot(x - self.san[0], y - self.san[1]) > 3.0 + 80.0 * max(0.0, el):
                ok, why = False, 'jump'
            else:
                self.san = (x, y, ts)
        if not ok:
            self.gps.append(dict(stamp=st, arrival=arrival_ns, valid=False, why='sanity_' + why))
            return 'sanity_' + why
        att = [s for s, _ in self.imu if s <= st]
        att_ns = att[-1] if att else 0
        valid = bool(att_ns) and (st - att_ns) / 1e9 <= self.imu_age and not (att_ns <= self.inv_imu <= st)
        self.gps.append(dict(stamp=st, arrival=arrival_ns, valid=valid, why='ok' if valid else 'imu_gate'))
        return 'accepted' if valid else 'imu_gate'

    def lidar(self, ev, receipt_ns):
        st = ev['stamp_ns']
        if st == 0 or st <= self.last_lidar:
            return dict(mode='REJECTED', local_reason='parser_rejected', map_reason=None)
        self.last_lidar = st
        imu = [s for s, _ in self.imu if s <= st]
        can = [(s, v) for s, v in self.can if s <= st]
        if not imu:
            return dict(mode='NONE', local_reason='no IMU at or before LiDAR scan', map_reason=None)
        if (st - imu[-1]) / 1e9 > self.imu_age:
            return dict(mode='NONE', local_reason='IMU is stale for LiDAR scan', map_reason=None)
        if not can:
            return dict(mode='NONE', local_reason='no vehicle speed at or before LiDAR scan', map_reason=None)
        if (st - can[-1][0]) / 1e9 > self.can_age:
            return dict(mode='NONE', local_reason='vehicle speed is stale for LiDAR scan', map_reason=None)
        if imu[-1] <= self.inv_imu <= st:
            return dict(mode='NONE', local_reason='imu_input_invalid', map_reason=None)
        if can[-1][0] <= self.inv_can <= st:
            return dict(mode='NONE', local_reason='speed_input_invalid', map_reason=None)
        speed = can[-1][1]
        if not math.isfinite(speed) or speed > self.vmax:
            # B0 plan D7 / orchestrator decision 6: a non-GPS map invalidity is an invalid local input
            # (output_gate::ClassifyGps kNonGpsInvalid) -> nothing on V2, not local_v2.
            return dict(mode='NONE', local_reason='local_ready', map_reason='map_speed_invalid')
        latest, consecutive, last_src = None, 0, 0
        for s in self.gps:
            if s['arrival'] > receipt_ns:
                continue
            if not s['valid'] and s['stamp'] == 0:
                latest, consecutive = s, 0
                continue
            if s['stamp'] > st or s['stamp'] <= last_src:
                continue
            if last_src and s['stamp'] - last_src > self.gps_age:
                consecutive = 0
            last_src, latest = s['stamp'], s
            consecutive = consecutive + 1 if s['valid'] else 0
        if latest is None:
            reason = 'gps_missing'
        elif not latest['valid']:
            reason = 'gps_invalid'
        elif st - latest['stamp'] > self.gps_age:
            reason = 'gps_stale'
        elif consecutive < self.recovery:
            reason = 'gps_recovering'
        else:
            return dict(mode='MAP', local_reason='local_ready', map_reason='gps_bounded_prediction' if st != latest['stamp'] else 'gps_observed')
        return dict(mode='LOCAL', local_reason='local_ready', map_reason=reason)


def run_oracle(events, contract, latency_ns=1 * MS):
    """events: dicts with ev/stamp_ns plus 'ros_ns' and 'mono_ns' publish times (feeder log) -> {k: oracle result}."""
    o, out = Oracle(contract), {}
    for e in events:
        if e['ev'] == 'imu':
            o.add_imu(e, e['ros_ns'] + latency_ns)
        elif e['ev'] == 'can':
            o.add_can(e, e['ros_ns'] + latency_ns)
        elif e['ev'] == 'gps':
            o.add_gps(e, e['mono_ns'] + latency_ns, e['ros_ns'] + latency_ns)
        elif e['ev'] == 'lidar':
            out[e['k']] = dict(o.lidar(e, e['mono_ns'] + latency_ns), stamp_ns=e['stamp_ns'], phase=e['phase'], i=e['i'])
    return out


def simulate_times(events, t0_ns, defaults, mono0_ns=10 ** 12):
    """Offline stand-in for the feeder's publish clock (dry run / self-test)."""
    for e in events:      # G4: publication 2 ms after the nominal sample time (see plan())
        e['ros_ns'] = e['pub_ns'] + 100000
        e['mono_ns'] = mono0_ns + (e['pub_ns'] - t0_ns) + 100000
        yield e


def selftest(catalogue):
    d, cp = catalogue['defaults'], catalogue['contract_parameters']
    report, ok = [], True
    lat, lon = map_to_latlon(*d['ego_start_map_xy'])
    e, n = utm_forward(lat, lon)
    utm_err = math.hypot(e - ORIGIN_E - d['ego_start_map_xy'][0], n - ORIGIN_N - d['ego_start_map_xy'][1])
    report.append('utm roundtrip error m=%.2e lat=%.7f lon=%.7f' % (utm_err, lat, lon))
    ok &= utm_err < 1e-4
    t0 = 1792000000 * 1000 * MS
    for sc in catalogue['scenarios']:
        events = list(simulate_times(plan(sc, d, t0), t0, d))
        res = run_oracle(events, cp)
        index = {(r['phase'], r['i']): r for r in res.values()}
        modes = ''.join({'MAP': 'M', 'LOCAL': 'L', 'NONE': 'N', 'REJECTED': 'R'}[res[k]['mode']] for k in sorted(res))
        bad = []
        for h in sc.get('hand_expect', []):
            got = index.get((h['phase'], h['scan']), {}).get('mode', 'NO_SCAN')
            if got != h['mode']:
                bad.append('phase%d.scan%d expect %s got %s' % (h['phase'], h['scan'], h['mode'], got))
        ok &= not bad
        report.append('%-32s %s %s' % (sc['id'], 'OK ' if not bad else 'BAD', modes))
        report.extend('    ' + b for b in bad)
    return ok, report


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--catalogue', required=True)
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()
    cat = json.load(open(a.catalogue))
    if a.selftest:
        good, lines = selftest(cat)
        print('\n'.join(lines))
        print('SELFTEST', 'PASS' if good else 'FAIL')
        raise SystemExit(0 if good else 1)
