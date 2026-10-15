"""Native pedestrian crossing plans; no ego or NPC movement commands here."""
import math
import random

import numpy as np

from scen_runner.config import TOOL_ROOT, load_config, load_yaml, repo_path
from scen_runner.planning.motion_curve import Curve
from scen_runner.planning.vehicle import VehicleSpec
from scen_runner.planning.traffic_geometry import load_links


def approach_pose(curve, station):
    """Extend the first tangent upstream rather than clamp negative stations."""
    x, y, z, heading = curve.pose(max(0., station))
    if station < 0:
        x += station * math.cos(heading)
        y += station * math.sin(heading)
    return x, y, z, heading


class CrossingGenerator:
    def __init__(self, seed=None):
        self.seed = seed if seed is not None else random.SystemRandom().randrange(2**32)
        self.rng = random.Random(self.seed)
        self.settings = load_yaml(TOOL_ROOT / 'config/dynamic_crossing.yaml')
        self.base = load_yaml(TOOL_ROOT / 'config/dynamic_validation.yaml')
        links = load_links(repo_path(self.base['link_map']))
        self.ego = Curve([p for lid in self.base['ego_links'] for p in links[lid]['points']])
        self.vehicle = VehicleSpec.from_config(load_config())

    def next_plan(self):
        return RandomCrossingPlan(self)


class RandomCrossingPlan:
    def __init__(self, generator):
        c, rng, vehicle = generator.settings, generator.rng, generator.vehicle
        self.ego = generator.ego
        self.order_test = None
        self.cfg = dict(generator.base)
        model = rng.choice(list(c['models']))
        spec = c['models'][model]
        self.models = [(spec['group'], model)]
        speed = rng.choice(c['speeds_kmh'])
        vn, ve = speed / 3.6, c['ego_speed_kmh'] / 3.6
        reaction, decel = c['ego_reaction_s'], c['ego_deceleration_mps2']
        if vn <= 0 or ve <= 0 or decel <= 0 or reaction < 0:
            raise ValueError('속도·제동 감속도는 양수, 반응 시간은 0 이상이어야 합니다.')
        stop_time = reaction + ve / decel
        stop_distance = ve * reaction + ve**2 / (2 * decel)
        band = vehicle.width / 2 + spec['radius_m'] + c['clearance_m']
        if c['end_lateral_m'] <= band:
            raise ValueError('끝점이 ego 진로 점유 폭 밖에 있어야 합니다.')
        for _ in range(1000):
            station = rng.uniform(*c['cross_station_m'])
            side = rng.choice((-1, 1))
            pattern = rng.choice(c.get('cross_patterns',(-1, 0, 1)))
            angle = pattern * math.radians(rng.uniform(*c['diagonal_angle_deg']))
            cosine = math.cos(angle)
            lateral = c['start_lateral_m']['left' if side == 1 else 'right']
            if lateral <= band:
                raise ValueError('생성점이 ego 진로 점유 폭 밖에 있어야 합니다.')
            q0 = lateral / cosine
            center_time = c['npc_start_delay_s'] + q0 / vn
            exit_time = center_time + band / (vn * cosine)
            entry_time = center_time - band / (vn * cosine)
            stop_gap = ve * center_time - stop_distance
            # Whole inflated NPC footprint stays ahead of the stopped front bumper.
            required_gap = (spec['radius_m'] + c['clearance_m']
                            + abs(band * math.tan(angle)) + c['minimum_stop_gap_m'])
            # The requested nearby spawn positions take priority. Report timing
            # feasibility instead of moving starts farther away to fit braking.
            timing_feasible = entry_time < stop_time < exit_time and stop_gap >= required_gap
            x, y, z, heading = self.ego.pose(station)
            tangent = np.array([math.cos(heading), math.sin(heading)])
            normal = np.array([-tangent[1], tangent[0]])
            direction = -side * cosine * normal + math.sin(angle) * tangent
            q1 = c['end_lateral_m'] / cosine
            start = np.array([x, y]) - direction * q0
            end = np.array([x, y]) + direction * q1
            start_station = station - q0 * math.sin(angle)
            end_station = station + q1 * math.sin(angle)
            if not (0 <= min(start_station, end_station)
                    and max(start_station, end_station) <= self.ego.length):
                continue
            # Without braking, ego's front reaches the crossing at NPC centre arrival.
            activation_station = station - vehicle.front_extent - ve * center_time
            activation = approach_pose(self.ego, activation_station)
            radius = math.dist(activation[:2], start)
            # Verify circle entry on the actual route, not only local straight geometry.
            upstream = np.linspace(activation_station - 40, activation_station - .05, 160)
            if any(math.dist(approach_pose(self.ego, s)[:2], start) <= radius for s in upstream):
                continue
            self.start = (*map(float, start), self.ego.pose(start_station)[2])
            self.end = (*map(float, end), self.ego.pose(end_station)[2])
            self.heading_deg = math.degrees(math.atan2(direction[1], direction[0]))
            self.move_distance = q0 + q1
            self.cfg.update(walk_speed_kmh=speed, activation_distance_m=radius)
            self.direction_label = ('왼쪽→오른쪽' if side == 1 else '오른쪽→왼쪽') + ' · ' + {
                -1: '대각선 후방', 0: '가로', 1: '대각선 전방'}[pattern]
            self.cross_xy = (x, y)
            self.activation_xy = activation[:2]
            self.stop_xy = approach_pose(self.ego, activation_station + stop_distance)[:2]
            self.timing = dict(ego_speed_kmh=c['ego_speed_kmh'], reaction_s=reaction,
                               deceleration_mps2=decel, npc_start_delay_s=c['npc_start_delay_s'],
                               npc_radius_m=spec['radius_m'], entry_s=entry_time,
                               center_s=center_time, exit_s=exit_time, stop_s=stop_time,
                               stop_distance_m=stop_distance, front_gap_m=stop_gap,
                               required_gap_m=required_gap, cross_station_m=station,
                               start_lateral_m=lateral, stop_timing_feasible=timing_feasible,
                               activation_station_m=activation_station, seed=generator.seed)
            return
        raise ValueError('계산 가정을 만족하는 횡단 경로가 없습니다. dynamic_crossing.yaml을 확인하세요.')
