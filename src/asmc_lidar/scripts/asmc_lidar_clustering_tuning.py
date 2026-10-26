#!/usr/bin/env python3
"""Explicit opt-in control for the isolated pipeline, never the operational Main.

One six-value request at a time. The pipeline acknowledges at a scan boundary;
HTTP publication alone is not application. YAML and ROS parameters are not written.
"""
import copy
import json
import math
import threading
import time
import uuid

KEYS = ('NearRange', 'FarRange', 'NearEps', 'FarEps', 'RearNearEps', 'RearFarEps')


def validate(values):
    if not isinstance(values, dict) or set(values) != set(KEYS):
        raise ValueError('거리 경계 2개와 앞·뒤 연결 거리 4개가 필요합니다. 처리·웹 노드 버전을 확인하세요.')
    if any(isinstance(values[k], bool) or not isinstance(values[k], (int, float)) for k in KEYS):
        raise ValueError('설정값은 숫자여야 합니다.')
    v = {k: float(values[k]) for k in KEYS}
    if not all(math.isfinite(n) for n in v.values()):
        raise ValueError('유한한 숫자를 입력하세요.')
    if not (0.1 <= v['NearRange'] and v['NearRange'] + 1 <= v['FarRange'] <= 200):
        raise ValueError('거리 경계: 0.1 ≤ NearRange, NearRange+1 ≤ FarRange ≤ 200m')
    if not (0.05 <= v['NearEps'] <= v['FarEps'] <= 5):
        raise ValueError('앞 연결 거리: 0.05 ≤ NearEps ≤ FarEps ≤ 5m')
    if not (0.05 <= v['RearNearEps'] <= v['RearFarEps'] <= 5):
        raise ValueError('뒤 연결 거리: 0.05 ≤ RearNearEps ≤ RearFarEps ≤ 5m')
    return v


class ClusteringTuning:
    def __init__(self, rospy, target):
        from std_msgs.msg import String
        self.rospy, self.String = rospy, String
        self.lock = threading.Lock()
        self.status, self.received, self.pending = None, 0.0, None
        self.last_error = ''
        self.token = uuid.uuid4().hex
        self.publisher = rospy.Publisher(target + '/clustering_request', String, queue_size=1, latch=False)
        self.subscriber = rospy.Subscriber(target + '/clustering_status', String, self.on_status, queue_size=1)

    def on_status(self, msg):
        try:
            value = json.loads(msg.data)
            validate(value['current'])
        except (ValueError, KeyError, TypeError):
            return
        with self.lock:
            if (self.pending and self.status and value.get('producer_session_id')
                    != self.status.get('producer_session_id')):
                self.pending = None
                self.last_error = '처리 노드가 재시작되었습니다. 이전 요청의 적용은 미확정입니다. 현재 값을 확인하세요.'
            if self.pending and value.get('rejected_request_id') == self.pending['request_id']:
                self.pending = None
                self.last_error = value.get('rejection_error', '처리 노드가 요청을 거부했습니다.')
            self.status, self.received = value, time.monotonic()
            if (self.pending and value.get('request_id') == self.pending['request_id']
                    and value.get('state') in ('applied', 'rejected')):
                self.pending = None

    def snapshot(self):
        with self.lock:
            return {'enabled': True, 'token': self.token, 'pipeline': copy.deepcopy(self.status),
                    'status_age_s': time.monotonic() - self.received if self.received else None,
                    'pending': copy.deepcopy(self.pending), 'last_error': self.last_error}

    def submit(self, data):
        values = validate(data)
        with self.lock:
            if self.pending:
                raise ValueError('이전 요청의 적용 확인 대기 중입니다. 새로고침해도 중복 전송하지 않습니다.')
            if not self.status or time.monotonic() - self.received > 3:
                raise ValueError('처리 노드 상태가 없습니다. lidar.launch clustering_tuning:=true로 실행하세요.')
            if not self.status.get('enabled') or not self.status.get('adaptive'):
                raise ValueError('처리 노드의 clustering_tuning 및 Adaptive.Enabled가 필요합니다.')
            if self.status.get('state') in ('pending', 'waiting_for_valid_scan'):
                raise ValueError('처리 노드가 다른 설정 요청을 적용 중입니다.')
            if self.publisher.get_num_connections() == 0:
                raise ValueError('설정 수신 노드가 연결되지 않았습니다.')
            request_id = uuid.uuid4().hex
            self.last_error = ''
            self.pending = dict(values, request_id=request_id, submitted_unix_s=time.time())
            # Publish under the same lock so an immediate acknowledgement cannot race pending.
            try:
                self.publisher.publish(self.String(data=json.dumps(dict(values, request_id=request_id), allow_nan=False)))
            except Exception as exc:
                self.pending = None
                raise ValueError('설정 전송 실패: ' + str(exc)) from exc
        return {'accepted': True, 'state': 'pending', 'request_id': request_id}
