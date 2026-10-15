"""Reject old/remote scene data before releasing a freshly spawned ego."""
import math


def stamp(message):
    return message.header.stamp.to_sec()


class Readiness:
    def __init__(self, config):
        self.config = config['readiness']
        self.messages = {}
        self.reset({}, float('inf'))

    def reset(self, pose, cutoff):
        self.pose = dict(pose)
        self.cutoff = cutoff
        self.messages.clear()
        self.stable_since = None
        self.detail = '차량 상태·객체 정보·경로 수신 대기'

    def observe(self, name, message, received):
        self.messages[name] = message, received

    def _near(self, state):
        cfg = self.config
        values = (state.x, state.y, state.yaw, state.speed)
        yaw = math.radians(self.pose['yaw'])
        error = abs((state.yaw-yaw+math.pi) % (2*math.pi)-math.pi)
        return (state.valid and state.header.frame_id == 'map' and all(map(math.isfinite, values)) and
                math.hypot(state.x-self.pose['x'], state.y-self.pose['y']) <= cfg['position_tolerance_m'] and
                error <= math.radians(cfg['yaw_tolerance_deg']) and
                abs(state.speed) <= cfg['stopped_speed_mps'])

    def evaluate(self, now, ros_now):
        failure = self._failure(now, ros_now)
        if failure:
            self.detail = failure
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = now
        self.detail = '새 시작 위치의 입력 일치 확인 중'
        ready = now-self.stable_since >= self.config['stable_s']
        if ready:
            self.detail = '새 위치 차량 상태·객체 정보·경로·제어 준비 완료'
        return ready

    def _failure(self, now, ros_now):
        cfg = self.config
        for name, age in (('state', cfg['state_age_s']), ('objects', cfg['objects_age_s']),
                          ('trajectory', cfg['trajectory_age_s']), ('control', cfg['trajectory_age_s'])):
            row = self.messages.get(name)
            if not row:
                return f'{name} 미수신'
            message, received = row
            if not 0 <= now-received <= age:
                return f'{name} 수신 지연'
            if name != 'control':
                source = stamp(message)
                if source < self.cutoff or not -.05 <= ros_now-source <= age:
                    return f'{name} 이전 회차/오래된 데이터'
        state = self.messages['state'][0]
        if not self._near(state):
            return '차량 상태가 새 시작 위치·방향·정지 조건과 불일치'
        objects = self.messages['objects'][0]
        if objects.header.frame_id != 'map':
            return '객체 좌표계 불일치'
        trajectory = self.messages['trajectory'][0]
        if (not trajectory.valid or trajectory.failure_reason != 0 or not trajectory.points or
                trajectory.header.frame_id != 'map' or stamp(trajectory.ego_at_plan) < self.cutoff or
                not self._near(trajectory.ego_at_plan)):
            return f'새 위치 경로 미준비 · failure_reason={trajectory.failure_reason}'
        return None
