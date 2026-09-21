#!/usr/bin/env python3
"""주행가능영역 밖 트래킹 객체를 걸러 `last_lidar` 로 다시 발행하는 ROS 노드.

입력  /lidar/tracked_objects_map  (Msg/TrackedObjectArray, frame_id = "map")
출력  last_lidar                  (같은 타입. 살아남은 객체만)

판정은 OBB 의 네 꼭짓점으로 한다. 메시지의 `obb_corners[0..3]` 은 이미 map ENU 로
개별 변환된 값이라(PlannerObjectAdapter.cpp), 중심+yaw 로 박스를 다시 만들지 않는다
— yaw 규약(라디안/도, 법선 방향)이 어긋날 여지를 없애기 위해서다.

좌표계는 변환하지 않는다. 라이다의 map 프레임은 Parser.cpp 가 UTM52N 에서
eastOffset/northOffset(302595.0 / 4124145.0)을 뺀 ENU 이고, 구운 맵의
`local_origin_in_global` 이 같은 값이다.

실행 (Msg 는 중첩 워크스페이스에만 빌드되어 있어 그쪽 devel 을 source 해야 한다):

    source src/lidar+sensorfusion/devel/setup.bash
    python3 src/asmc_hd_map/development/won_tae_link_node/drivable_filter_node.py

    # 파라미터를 바꿔서
    python3 src/asmc_hd_map/development/won_tae_link_node/drivable_filter_node.py _margin_m:=0.5 _ratio_threshold:=0.5
"""
from __future__ import annotations

import os
import sys

import numpy as np
import rospy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from drivable_query import DrivableMask  # noqa: E402

try:
    from Msg.msg import TrackedObjectArray
except ImportError as exc:  # pragma: no cover - 실행 환경 문제
    rospy.logfatal(
        f'Msg/TrackedObjectArray 를 import 할 수 없습니다 ({exc}). '
        'src/lidar+sensorfusion/ 는 CATKIN_IGNORE 가 붙은 중첩 워크스페이스라 '
        '그쪽 devel 을 source 해야 합니다: '
        'source src/lidar+sensorfusion/devel/setup.bash')
    raise

DEFAULT_H5 = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          'out', 'link_node_map.h5')


def quads_from_message(objects) -> tuple:
    """객체 목록에서 (N,4,2) 하층 OBB 꼭짓점과, 기하가 성한지 여부를 뽑는다.

    `obb_corners` 는 0-1-2-3 이 하층, 4-5-6-7 이 상층이다
    (Tracking.cpp의 OBB 생성 코드가 0..3에 zMin, 4..7에 zMax를 넣는다).
    XY 판정에는 하층 4개면 충분하다. 순서는 조회 쪽에서 다시 정렬하므로
    여기서 둘레 순서를 가정하지 않는다.
    """
    quads = np.zeros((len(objects), 4, 2), dtype=np.float64)
    for i, obj in enumerate(objects):
        for k in range(4):
            corner = obj.obb_corners[k]
            quads[i, k, 0] = corner.x
            quads[i, k, 1] = corner.y
    finite = np.isfinite(quads).all(axis=(1, 2))
    # 꼭짓점이 한 점으로 모인 축퇴 박스는 비율이 무의미하다.
    span = quads.max(axis=1) - quads.min(axis=1)
    usable = finite & (span > 1e-3).any(axis=1)
    return quads, usable


class DrivableFilterNode:
    def __init__(self) -> None:
        h5_path = rospy.get_param('~h5', DEFAULT_H5)
        self.margin_m = float(rospy.get_param('~margin_m', 1.0))
        self.threshold = float(rospy.get_param('~ratio_threshold', 0.3))
        self.samples = int(rospy.get_param('~samples', 5))
        expected_map_id = str(rospy.get_param('~map_id', 'waize_katri_2025-10-23'))
        in_topic = rospy.get_param('~input_topic', '/lidar/tracked_objects_map')
        out_topic = rospy.get_param('~output_topic', 'last_lidar')
        self.queue_size = max(1, int(rospy.get_param('~queue_size', 10)))
        self.report_period_s = float(rospy.get_param('~report_period_s', 2.0))

        self.mask = DrivableMask.load(h5_path, margin_m=self.margin_m)
        if expected_map_id:
            # 판본이 다르면 좌표가 통째로 어긋나 조회 결과 전체가 틀린다.
            self.mask.check_map(expected_map_id)
        rospy.loginfo(
            f'[drivable_filter] map_id={self.mask.map_id} '
            f'shape={self.mask.mask.shape} ppm={self.mask.pixels_per_meter} '
            f'margin={self.margin_m} m  threshold={self.threshold}  '
            f'samples={self.samples}x{self.samples}')

        self.publisher = rospy.Publisher(out_topic, TrackedObjectArray,
                                         queue_size=self.queue_size)
        self.subscriber = rospy.Subscriber(in_topic, TrackedObjectArray,
                                           self.callback,
                                           queue_size=self.queue_size,
                                           tcp_nodelay=True)
        rospy.loginfo(f'[drivable_filter] {in_topic} -> {out_topic}')

        self.frames = 0
        self.n_in = 0
        self.n_kept = 0
        self.n_degenerate = 0
        self.last_report = rospy.Time(0)
        self.frame_checked = False

    # ------------------------------------------------------------------ 콜백
    def callback(self, msg: TrackedObjectArray) -> None:
        objects = list(msg.objects)
        kept = []
        if objects:
            quads, usable = quads_from_message(objects)
            ratio = np.zeros(len(objects), dtype=np.float64)
            if usable.any():
                ratio[usable] = self.mask.quads_inside_ratio(
                    quads[usable], samples=self.samples)
            self._check_frame_once(msg, quads, usable)
            for i, obj in enumerate(objects):
                if not usable[i]:
                    # 기하를 못 믿을 때는 버리지 않고 남긴다. 놓치는 쪽보다
                    # 남기는 쪽으로 기울인다 (뒷단이 다시 판단할 수 있게).
                    self.n_degenerate += 1
                    kept.append(obj)
                elif ratio[i] >= self.threshold:
                    kept.append(obj)

        out = TrackedObjectArray()
        out.header = msg.header
        out.objects = kept
        self.publisher.publish(out)

        self.frames += 1
        self.n_in += len(objects)
        self.n_kept += len(kept)
        self._report()

    # ------------------------------------------------------------ 자체 점검
    def _check_frame_once(self, msg, quads: np.ndarray, usable: np.ndarray) -> None:
        """첫 유효 프레임에서 프레임 이름과 좌표 범위를 점검한다.

        맵 bounding range 밖으로 좌표가 몰리는 큰 원점 불일치만
        찾는 후리스틱이다. 범위 안의 GPS 점프나 수 m 오차는 찾지 못한다.
        """
        if self.frame_checked or not usable.any():
            return
        self.frame_checked = True
        if msg.header.frame_id != 'map':
            rospy.logwarn(
                f'[drivable_filter] frame_id 가 "map" 이 아닙니다 '
                f'({msg.header.frame_id!r}). 입력이 ENU 가 아니면 판정이 전부 틀립니다.')
        centers = quads[usable].mean(axis=1)
        if not self.mask.looks_like_same_frame(centers):
            rospy.logerr(
                '[drivable_filter] 객체 좌표가 맵 범위 밖입니다 — 좌표계 원점이 '
                '다를 수 있습니다. 라이다의 eastOffset/northOffset 이 '
                f'{self.mask.local_origin_in_global[:2].tolist()} 인지 확인하세요 '
                '(rostopic echo /gps -n1 | grep -i offset).')
        else:
            rospy.loginfo(
                f'[drivable_filter] 좌표 범위 확인 OK — 객체 {int(usable.sum())}개가 '
                '맵 범위 안입니다.')

    def _report(self) -> None:
        now = rospy.Time.now()
        if (now - self.last_report).to_sec() < self.report_period_s:
            return
        self.last_report = now
        dropped = self.n_in - self.n_kept
        rospy.loginfo(
            f'[drivable_filter] {self.frames} 프레임  객체 {self.n_in} -> '
            f'{self.n_kept} (버림 {dropped}'
            f'{f", 기하불량 통과 {self.n_degenerate}" if self.n_degenerate else ""})')
        self.frames = self.n_in = self.n_kept = self.n_degenerate = 0


def main() -> None:
    rospy.init_node('drivable_filter')
    DrivableFilterNode()
    rospy.spin()


if __name__ == '__main__':
    main()
