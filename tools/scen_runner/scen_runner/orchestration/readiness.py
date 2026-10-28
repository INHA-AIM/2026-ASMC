"""Reject old/remote scene data before releasing a freshly spawned ego."""
import math


def stamp(message):
    return message.header.stamp.to_sec()


class Readiness:
    def __init__(self, config):
        self.config = config['readiness']
        self.profile=config.get('readiness_profile','map')
        if self.profile not in ('map','shadow_local'):raise ValueError('invalid readiness profile')
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

    def _shadow_failure(self, now, ros_now):
        # Explicit simulator-test profile. The scenario engine separately checks
        # the native held_at pose before release. No map pose is fabricated.
        for name in ('family','local','gps','speed','control'):
            row=self.messages.get(name)
            if not row:return name+' 미수신 (local 준비)'
            m,received=row
            age=.25 if name=='control' else .35
            if not 0<=now-received<=age:return name+' 수신 지연 (local 준비)'
            if name!='control':
                h=m.observation.objects.header if name=='local' else m.header
                source=h.stamp.to_sec()
                if source<self.cutoff or not -.05<=ros_now-source<=age:return name+' 이전 회차/오래된 데이터 (local 준비)'
        family=self.messages['family'][0];local=self.messages['local'][0].observation.objects
        gps=self.messages['gps'][0];speed=self.messages['speed'][0];control=self.messages['control'][0]
        mode=self.messages.get('mode')
        if not mode or mode[0].data!=1:return '실제 GPS JAMMING 모드 대기'
        if gps.latitude!=0 or gps.longitude!=0:return 'local 시험은 실제 GPS 무효 구간에서만 허용'
        if not family.valid or family.header.frame_id!='base_link':return 'local family 무효'
        if local.header.frame_id!='base_link' or local.status not in (1,2):return 'local 관측 무효'
        if not math.isfinite(speed.velocity.x) or abs(speed.velocity.x)>.3:return 'local 시험 정차 확인 실패'
        if control.longlCmdType!=2 or not 0<control.velocity<=5. or control.brake>0:return '검증된 local 주행 명령 대기'
        return None

    def _failure(self, now, ros_now):
        if self.profile=='shadow_local':return self._shadow_failure(now,ros_now)
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
