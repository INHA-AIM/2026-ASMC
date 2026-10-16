"""Scene repetition gate: pass CP14, then return to START/END."""
import math

from scen_runner.runtime.checkpoint_environment import segment_distance


class SceneCompletion:
    def __init__(self, checkpoints, start_end):
        self.checkpoint = next((p for p in checkpoints if p['id'] == '14'), None)
        if self.checkpoint is None or start_end is None:
            raise ValueError('씬 완주 판정에는 체크포인트 14와 START/END 위치가 필요합니다.')
        self.finish = start_end
        for point in (self.checkpoint, self.finish):
            if not all(math.isfinite(v) for v in point['xy']) or not (
                    math.isfinite(point['radius']) and point['radius'] > 0):
                raise ValueError('씬 완주 위치와 통과 반경은 유효한 값이어야 합니다.')
        self.reset()

    def reset(self, start_xy=None, now=None):
        self.checkpoint14_passed = False
        self.completed = False
        # Anchor the first running sample to the actual episode spawn location.
        # A first query already at CP14 must not qualify a teleport as a pass.
        self.last_sample = None if start_xy is None else (tuple(start_xy), now, 0.)

    def update(self, state, now):
        if self.completed:
            return False
        p, v = state.transform.location, state.velocity
        xy = (p.x, p.y)
        speed = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z) / 3.6
        if not all(math.isfinite(value) for value in (*xy, speed, now)):
            self.reset()
            return False
        previous = self.last_sample
        self.last_sample = (xy, now, speed)
        if previous is None:
            return False
        old_xy, old_time, old_speed = previous
        elapsed = now - old_time
        if elapsed <= 0:
            return False
        radius = max(self.checkpoint['radius'], self.finish['radius'])
        possible = 2*max(speed, old_speed)*elapsed + 2*radius
        if math.dist(old_xy, xy) > possible:
            # An external teleport invalidates the lap qualification as well as
            # the straight segment across the jump. Staying at its destination
            # cannot subsequently turn the teleport into a finish.
            self.checkpoint14_passed = False
            return False
        if not self.checkpoint14_passed:
            # Require an observed entry/crossing from outside CP14's circle.
            # This also rejects spawning or teleporting directly inside it.
            if (math.dist(old_xy, self.checkpoint['xy']) > self.checkpoint['radius'] and
                    segment_distance(self.checkpoint['xy'], old_xy, xy) <= self.checkpoint['radius']):
                self.checkpoint14_passed = True
            return False
        # Segment intersection also detects passing the finish between samples.
        if segment_distance(self.finish['xy'], old_xy, xy) <= self.finish['radius']:
            self.completed = True
            return True
        return False

    def snapshot(self):
        return dict(checkpoint14_passed=self.checkpoint14_passed, completed=self.completed)
