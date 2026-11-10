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
        if name == 'mode' and (name not in self.messages or self.messages[name][0].data != message.data):
            self.stable_since = None
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
        selected = '재밍 차선·LiDAR·최종 경로' if self.messages['mode'][0].data == 1 else '일반 경로'
        self.detail = '새 시작 위치의 '+selected+' 입력 일치 확인 중'
        ready = now-self.stable_since >= self.config['stable_s']
        if ready:
            self.detail = '새 위치 차량 상태·'+selected+'·제어 준비 완료'
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

    def _fresh(self, name, age, now, ros_now, source=True):
        row = self.messages.get(name)
        if not row:
            return f'{name} 미수신'
        message, received = row
        if not 0 <= now-received <= age:
            return f'{name} 수신 지연'
        if source:
            payload = message.objects if name == 'lidar' else message
            source_time = stamp(payload)
            if source_time < self.cutoff or not -.05 <= ros_now-source_time <= age:
                return f'{name} 이전 회차/오래된 데이터'
        return None

    def _failure(self, now, ros_now):
        if self.profile=='shadow_local':return self._shadow_failure(now,ros_now)
        cfg = self.config
        if 'mode' not in self.messages:
            return 'mode 미수신'
        mode = self.messages['mode'][0].data
        if mode not in (0, 1):
            return 'mode 유효하지 않음'
        for name, age, source in (('state', cfg['state_age_s'], True),
                                  ('control', cfg['trajectory_age_s'], False),
                                  ('selector', cfg.get('selector_age_s', 1.5), False)):
            failure = self._fresh(name, age, now, ros_now, source)
            if failure:
                return failure
        state = self.messages['state'][0]
        if not self._near(state):
            return '차량 상태가 새 시작 위치·방향·정지 조건과 불일치'
        selected = '/ctrl_cmd/jamming' if mode == 1 else '/ctrl_cmd/normal'
        selector = dict(part.split('=', 1) for part in self.messages['selector'][0].data.split() if '=' in part)
        if (selector.get('selected_source') != selected or selector.get('command_fresh') != 'true' or
                selector.get('mode') != ('JAMMING' if mode == 1 else 'NORMAL')):
            return '선택 제어 입력 미준비 · '+selected
        if mode == 1:
            return self._jamming_failure(now, ros_now)
        for name, age in (('objects', cfg['objects_age_s']), ('trajectory', cfg['trajectory_age_s'])):
            failure = self._fresh(name, age, now, ros_now)
            if failure:
                return failure
        objects = self.messages['objects'][0]
        if objects.header.frame_id != 'map':
            return '객체 좌표계 불일치'
        trajectory = self.messages['trajectory'][0]
        if (not trajectory.valid or trajectory.failure_reason != 0 or not trajectory.points or
                trajectory.header.frame_id != 'map' or stamp(trajectory.ego_at_plan) < self.cutoff or
                not self._near(trajectory.ego_at_plan)):
            return f'새 위치 경로 미준비 · failure_reason={trajectory.failure_reason}'
        return None

    def _jamming_failure(self, now, ros_now):
        # Shadow-zone entry can select JAMMING while ego is still held at spawn.
        # Verify that controller's live inputs/path instead of waiting for an
        # unselected normal trajectory to recover from GPS blackout.
        cfg = self.config
        for name, age, source in (('lidar', cfg['objects_age_s'], True),
                                  ('jamming', cfg['trajectory_age_s'], True),
                                  ('jamming_control', cfg['trajectory_age_s'], False)):
            failure = self._fresh(name, age, now, ros_now, source)
            if failure:
                return failure
        lidar = self.messages['lidar'][0]
        if (not lidar.producer_session_id or lidar.objects.header.frame_id != 'base_link' or
                lidar.objects.status not in (1, 2)):
            return '재밍 LiDAR 유효 관측 미준비'
        statuses = [s for s in self.messages['jamming'][0].status if s.name == 'camera_bbox_lane_change']
        if len(statuses) != 1:
            return '재밍 경로 진단 미수신'
        values = {v.key: v.value for v in statuses[0].values}
        flags = ('enable_control', 'mode_active', 'ego_speed_fresh', 'imu_fresh', 'lidar_fresh')
        if any(values.get(key) != 'true' for key in flags) or values.get('final_path_check') != 'passed':
            return '재밍 입력/최종 경로 미준비 · '+statuses[0].message
        try:
            age = float(values['source_age_sec'])
        except (KeyError, ValueError):
            return '재밍 차선 원본 시각 미확인'
        if not math.isfinite(age) or not -.05 <= age <= cfg.get('lane_age_s', .5):
            return '재밍 차선 원본 지연'
        return None
