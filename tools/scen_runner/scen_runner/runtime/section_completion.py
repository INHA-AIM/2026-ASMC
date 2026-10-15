"""Detect a forward section finish even when ego passes beside the end point."""
import math


class SectionCompletion:
    def __init__(self, definition, half_width_m=10., approach_m=25., arrival_radius_m=3.):
        self.curve = definition['curve']
        self.end_xy = tuple(definition['end_xy'])
        heading = definition['end_heading']
        values = (half_width_m, approach_m, arrival_radius_m, heading, *self.end_xy)
        if not all(math.isfinite(v) for v in values) or min(half_width_m, approach_m, arrival_radius_m) <= 0:
            raise ValueError('구간 종료선 폭·접근 거리·도착 반경은 유효한 양수여야 합니다.')
        self.half_width = half_width_m
        self.approach = approach_m
        self.radius = arrival_radius_m
        self.tangent = (math.cos(heading), math.sin(heading))
        self.normal = (-self.tangent[1], self.tangent[0])
        self.reset()

    def coordinates(self, xy):
        dx, dy = xy[0]-self.end_xy[0], xy[1]-self.end_xy[1]
        return (dx*self.tangent[0]+dy*self.tangent[1], dx*self.normal[0]+dy*self.normal[1])

    def reset(self, start_xy=None, now=None):
        self.armed = False
        self.completed = False
        self.last_sample = None
        if start_xy is not None:
            xy = tuple(start_xy)
            station, _ = self.curve.project(*xy)
            forward, side = self.coordinates(xy)
            self.last_sample = (xy, now, 0., station, forward, side)

    def update(self, state, now):
        if self.completed:
            return False
        p, v = state.transform.location, state.velocity
        xy = (p.x, p.y)
        speed = math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)/3.6
        if not all(math.isfinite(value) for value in (*xy, now, speed)):
            self.reset()
            return False
        station, _ = self.curve.project(*xy)
        forward, side = self.coordinates(xy)
        previous = self.last_sample
        self.last_sample = (xy, now, speed, station, forward, side)
        if previous is None:
            return False
        old_xy, old_time, old_speed, old_station, old_forward, old_side = previous
        elapsed = now-old_time
        if elapsed <= 0:
            return False
        possible = 2*max(speed, old_speed)*elapsed+2*self.radius
        if (math.dist(old_xy, xy) > possible or
                abs(station-old_station) > possible+2*self.half_width):
            # Nearby bends/parallel route segments must not turn an impossible
            # route-station jump into a finish at an earlier part of the route.
            self.armed = False
            return False
        near_end = max(station, old_station) >= self.curve.length-self.approach
        if near_end and old_forward < -self.radius:
            self.armed = True
        if not self.armed:
            return False
        # Preserve close forward arrivals, including stopping just before the
        # end point. Sideways/backwards motion alone must not count as arrival.
        arrived = (forward > old_forward and station >= self.curve.length-self.radius and
                   math.dist(xy, self.end_xy) <= self.radius)
        crossed = False
        if old_forward < 0. <= forward:
            fraction = -old_forward/(forward-old_forward)
            crossing_side = old_side+fraction*(side-old_side)
            crossed = abs(crossing_side) <= self.half_width
        if arrived or crossed:
            self.completed = True
            return True
        return False
