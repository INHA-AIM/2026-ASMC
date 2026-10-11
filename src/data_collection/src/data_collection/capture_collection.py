"""Offline policies shared by the Capture coordinator, layout tool and tests."""
import math
import copy
from pathlib import Path
import sys


CLASSES = ('person', 'dog', 'deer', 'bicycle', 'car', 'motorcycle', 'bus', 'truck',
           'bollard', 'box', 'concrete_barrier', 'steel_barricade', 'traffic_drum',
           'plastic_barrier', 'traffic_cone', 'stroller')
PRESENCE = dict(stroller=0, plastic_barrier=172, steel_barricade=178,
                traffic_cone=618, concrete_barrier=953, box=1109,
                traffic_drum=1661, bollard=2298, deer=61, motorcycle=69, bicycle=230)
ASSETS = {
    'stroller': ('BabyStroller1', 'BabyStroller2', 'BabyStroller3'),
    'plastic_barrier': ('PE_Firewall_Orange', 'PE_Firewall_White'),
    'steel_barricade': ('Steel_Barricade',),
    'traffic_cone': ('Color_Cone', 'Rubber_Cone'),
    'concrete_barrier': ('Concrete_Barricade01', 'Concrete_Barricade02', 'concrete-barrier'),
    'box': ('CargoBox', 'WoodBox'),
    'traffic_drum': ('PE_Drum', 'RedBarrel', 'TrafficBarrel', 'YellowBarrel'),
    'bollard': ('Bollard01', 'Bollard04'),
    'deer': ('Deer1',), 'motorcycle': ('Scooter1', 'Scooter2', 'SportBike', 'Sportbike_01'),
    'bicycle': ('Bicycle_city', 'Bicycle_mountain', 'Bike1', 'Bike2', 'Bike3'),
}


def kph_to_mps(value):
    value = float(value)
    if not math.isfinite(value) or not 0 <= value <= 30:
        raise ValueError('collection speed must be 0..30 km/h')
    return value / 3.6


def motion_summary(samples, target_kph):
    moving=[r for r in samples if not r['capture_active']]
    speeds=sorted(r['speed_mps']*3.6 for r in moving)
    # Reach the requested speed for at least three seconds of measured control
    # intervals. A brief spike or downhill rollback is insufficient.
    reached_seconds=sum(min(.1,b['monotonic']-a['monotonic']) for a,b in zip(moving,moving[1:])
        if a['speed_mps']*3.6>=target_kph*.8 and b['speed_mps']*3.6>=target_kph*.8)
    progress=samples[-1]['station_m']-samples[0]['station_m'] if samples else 0.
    return dict(actual_speed_max_kph=max(speeds) if speeds else None,
        actual_speed_p50_kph=speeds[len(speeds)//2] if speeds else None,
        seconds_at_80pct_target=reached_seconds,forward_progress_m=progress,
        speed_achieved=reached_seconds>=3 and progress>5)


def allocation(count):
    """Deterministic largest-deficit allocation; unavailable assets stay unfilled."""
    if count not in (6, 12, 60, 120, 240):
        raise ValueError('use a staged layout size: 6/12/60/120/240')
    rare_count = round(count / 6)
    result = []
    for pool, total in ((tuple(k for k in PRESENCE if k not in ('deer', 'motorcycle', 'bicycle')), count-rare_count),
                        (('deer', 'motorcycle', 'bicycle'), rare_count)):
        weights = {k: 1 / math.sqrt(max(PRESENCE[k], 50)) for k in pool}
        norm = sum(weights.values()); used = {k: 0 for k in pool}
        for i in range(total):
            label = max(pool, key=lambda k: ((i+1)*weights[k]/norm-used[k], k))
            used[label] += 1
            result.append((label, ASSETS[label][(used[label]-1) % len(ASSETS[label])]))
    # Interleave rare objects around the whole route instead of placing them all at its end.
    obstacles, rare = result[:count-rare_count], result[count-rare_count:]
    output = []
    for i, item in enumerate(obstacles):
        output.append(item)
        if rare and (i+1)*rare_count//len(obstacles) > i*rare_count//len(obstacles):
            output.append(rare.pop(0))
    return output


def route_sample(points, station):
    remaining = max(0., station)
    for a, b in zip(points, points[1:]):
        length = math.hypot(b[0]-a[0], b[1]-a[1])
        if length <= 1e-8:
            continue
        if remaining <= length:
            t = remaining/length
            return tuple(a[j]+t*(b[j]-a[j]) for j in range(3)), math.atan2(b[1]-a[1], b[0]-a[0])
        remaining -= length
    return tuple(points[-1][:3]), math.atan2(points[-1][1]-points[-2][1], points[-1][0]-points[-2][0])


def point_route_distance(points, x, y):
    best = float('inf')
    for a, b in zip(points, points[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length2 = dx*dx+dy*dy
        if length2 <= 1e-12:
            continue
        t = max(0., min(1., ((x-a[0])*dx+(y-a[1])*dy)/length2))
        best = min(best, math.hypot(x-a[0]-t*dx, y-a[1]-t*dy))
    return best


def footprint_clear(points, center, half_diagonal, occupied, ego_half_width=1.1, margin=1.):
    """Bounding circles conservatively cover rotated footprints and all route segments."""
    if not all(math.isfinite(float(v)) for v in (*center, half_diagonal)) or half_diagonal <= 0:
        return False
    if point_route_distance(points, *center) <= half_diagonal + ego_half_width + margin:
        return False
    return all(math.hypot(center[0]-x, center[1]-y) > half_diagonal+r+margin for x,y,r in occupied)


def native_annotation_class(actor_id, ledger, source_kind=None, rgba=None):
    """Only verified native IDs resolve GT; parent arithmetic alone is insufficient."""
    matches = [r for r in ledger if str(actor_id) in tuple(map(str, r.get('native_ids', ())))
               and r.get('placement_approved') and r.get('identity_approved')]
    if len(matches) != 1 or matches[0]['target_class'] not in CLASSES:
        raise ValueError('unresolved or ambiguous native GT actor ID: ' + str(actor_id))
    row=matches[0]
    if source_kind is not None:
        typed=row.get('native_source_classes',{}).get(str(actor_id),{})
        label=typed.get(source_kind)
        expected=row.get('native_rgba',{}).get(str(actor_id),{}).get(source_kind)
        if label not in CLASSES or (rgba is not None and (expected is None or list(rgba)!=expected)):
            raise ValueError('unresolved native source-kind/instance-color identity: '+str(actor_id))
        return label,row['api_name']
    label=row.get('native_class_overrides',{}).get(str(actor_id),row['target_class'])
    if label not in CLASSES:raise ValueError('invalid reviewed component class')
    return label, row['api_name']


def observed_components(ledger, sources, calibrated_assets, source_colors=None, reviewed_colors=None):
    """Resolve only observed BBox IDs using an RGB-verified asset schema.

    No component is invented from parent arithmetic alone. The actual BBox ID
    and source kind must exist, and another spawned actor cannot own that ID.
    """
    rows=copy.deepcopy(ledger)
    roots={str(r['native_ids'][0]) for r in rows if r.get('native_ids') and not r.get('removed')}
    for row in rows:
        if not row.get('native_ids') or row.get('removed'):continue
        root=str(row['native_ids'][0]);schema=calibrated_assets.get(row['api_name'],{}).get('components',{})
        if not schema or not root.isdecimal():continue
        for code,spec in schema.items():
            native=root if int(code)==0 else str(int(root)*10000+int(code))
            if native not in sources:continue
            if native!=root and native in roots:raise ValueError('component conflicts with another native root')
            expected=spec['gt_source_label']
            if set(sources[native])!={expected}:
                row.setdefault('component_resolution_conflicts',[]).append(dict(native_id=native,
                    expected=expected,observed=sorted(sources[native]),reason='numeric GT ID is not globally source-kind unique'))
            if expected not in sources[native]:continue
            if spec['canonical'] not in CLASSES:raise ValueError('invalid calibrated component class')
            if native not in row['native_ids']:row['native_ids'].append(native)
            row.setdefault('native_class_overrides',{})[native]=spec['canonical']
            row.setdefault('native_source_classes',{}).setdefault(native,{})[expected]=spec['canonical']
            if source_colors is not None:
                colors=source_colors.get(native,{}).get(expected,set())
                reviewed=(reviewed_colors or {}).get(native,{}).get(expected)
                if reviewed is not None:
                    if tuple(reviewed) not in colors:raise ValueError('reviewed instance color is absent from this run')
                    row.setdefault('native_rgba',{}).setdefault(native,{})[expected]=list(reviewed)
                elif len(colors)==1:
                    row.setdefault('native_rgba',{}).setdefault(native,{})[expected]=list(next(iter(colors)))
                else:
                    row.setdefault('component_resolution_conflicts',[]).append(dict(native_id=native,
                        source_kind=expected,reason='instance color is ambiguous; per-image identity remains unresolved'))
        row['component_resolution_basis']='observed native BBox IDs and RGB-verified asset component schema'
    return rows


def collection_trial_passed(validation, summary, captures, restoration, exit_code):
    """Motion/file RPC success cannot conceal a failed stop or actuator restore."""
    restored = all(restoration.get(k) is True for k in ('stopped', 'gear_matches', 'mode_matches'))
    count_ok = (summary.get('succeeded', 0) >= captures if captures
                else validation.get('exit_reason') == 'route_complete')
    return bool(exit_code == 0 and restored and validation.get('tracking_passed') and
                summary.get('failed') == 0 and validation.get('speed_achieved') and
                validation.get('forward_progress_m', 0) > 5 and count_ok)


class StableRestoration:
    """Require a continuous stopped/gear/mode interval after actuator changes."""
    def __init__(self):
        self.since = None

    def update(self, now, stopped, gear_matches, mode_matches):
        if not (stopped and gear_matches and mode_matches):
            self.since = None
            return False
        if self.since is None:
            self.since = now
        return now - self.since >= .5


class RecoveryMonitor:
    def __init__(self, controller='mpc'):
        self.controller = controller
        self.completed = None
        self.restarts = 0
        self.failures = []

    def capture_completed(self, monotonic):
        self.completed = monotonic

    def check(self, now, state_received, state_valid, speed_mps,
              trajectory_received=None, trajectory_valid=False):
        if self.completed is None:
            return 'ready'
        fresh = state_valid and state_received is not None and state_received > self.completed and now-state_received <= .20
        if self.controller == 'mpc':
            fresh = fresh and trajectory_valid and trajectory_received is not None and trajectory_received > self.completed and now-trajectory_received <= .25
        if fresh and speed_mps >= .2:
            self.completed = None
            return 'ready'
        if now-self.completed >= 2.:
            self.failures.append('capture_resume_timeout')
            return 'fallback' if self.controller == 'mpc' else 'stop'
        return 'waiting'

    def manual_restart(self):
        self.restarts += 1
        return self.restarts >= 2


class LegacyPurePursuit:
    """Reuse the existing runner geometry, without its actor manager/reset loop."""
    def __init__(self, repo, points, target_kph, wheelbase=2.9, max_steer_deg=40.):
        runner = Path(repo)/'tools/aim_scenario_runner'
        sys.path.insert(0, str(runner))
        from zones.urban_drive_cases import UrbanRouteDriveCase
        self.impl = UrbanRouteDriveCase.__new__(UrbanRouteDriveCase)
        self.impl.cfg = dict(pure_pursuit_target_speed=float(target_kph),
            pure_pursuit_lookahead_m=8., pure_pursuit_wheel_base_m=wheelbase,
            pure_pursuit_max_steer_deg=max_steer_deg, pure_pursuit_min_speed=3.)
        self.impl.route_points = points
        self.impl.route_length_m = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(points,points[1:]))
        self.impl.pure_pursuit_last_s = None

    def command(self, x, y, yaw_rad, speed_mps):
        self.impl.cfg['pure_pursuit_lookahead_m']=8.
        result = self.impl.compute_pure_pursuit_command(dict(x=x,y=y,yaw_deg=math.degrees(yaw_rad)))
        if result['turn_delta_deg'] >= 15:
            initial_turn=result['turn_delta_deg']
            # Collection-only short lookahead avoids cutting tight route corners.
            # Keep the original PP formula and its speed cap, not a second tracker.
            self.impl.cfg['pure_pursuit_lookahead_m']=4.
            result=self.impl.compute_pure_pursuit_command(dict(x=x,y=y,yaw_deg=math.degrees(yaw_rad)))
            result['turn_delta_deg']=max(initial_turn,result['turn_delta_deg'])
        # Preserve legacy PP steering, reduce speed before curves and near the route end.
        if result['turn_delta_deg'] >= 15:
            result['target_speed'] = min(result['target_speed'], 20. if result['turn_delta_deg'] < 35 else 10.)
        if result['remaining_s'] < 4.:
            result['target_speed'] = 0.
        error = kph_to_mps(result['target_speed']) - speed_mps
        result.update(accel=max(0., min(1., .5*error)), brake=max(0., min(1., -.5*error)))
        return result
