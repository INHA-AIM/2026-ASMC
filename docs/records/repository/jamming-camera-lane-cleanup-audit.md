# 카메라 차선 변경 코드 정리 결과

> 아래 crop/legacy detector 명령은 과거 감사 기록이다. 현재 camera_bbox_detector.launch는 CPU 변환만 제공하며 legacy crop 추론 옵션은 제거됐다.

현재 정상 동작하는 세 실행 조합을 기준으로 이 패키지의 구형 코드를 정리했다.

- `asmc_perception/perception.launch camera_model_grid_max_hz:=5.0`
- `lane_boundary_adapter/camera_bbox_detector.launch hood_crop_bottom_ratio:=0.21`
- `lane_boundary_adapter/camera_bbox_lane_change.launch` (v3, 반복 변경, BBOX 제동 제외, RViz)

## 제거한 구형 파일 15개

모두 `src/jamming_lane_boundary_adapter/` 기준이다.

| 그룹 | 제거 파일 |
|---|---|
| 구형 실행 코드 | `src/camera_lane_follower_node.cpp`, `src/camera_lane_follower_avoidance_node.cpp`, `src/lidar_lane_roi_node.cpp` |
| 구형 launch | `launch/camera_lane_follow.launch`, `launch/camera_lane_avoidance.launch` |
| 구형 RViz | `rviz/camera_lane_follow.rviz`, `rviz/camera_lane_avoidance.rviz` |
| 구형 안내 | `CAMERA_LANE_FOLLOWING.md`, `CAMERA_LANE_AVOIDANCE.md` |
| 구형 테스트 launch | `test/camera_lane_follower.test`, `test/camera_lane_avoidance.test`, `test/lidar_lane_roi.test` |
| 구형 테스트 | `test/test_camera_lane_follower.py`, `test/test_camera_lane_avoidance.py`, `test/test_lidar_lane_roi.py` |

CMake의 구형 executable/링크/install/rostest 등록과 구형 라이다 코드 전용
`tf2_ros`, `tf2_geometry_msgs` 의존성을 함께 제거했다.
README와 로컬 판제 안내의 실행·문서 링크를 현재 BBOX 주행으로 변경했다.

## 공유 의존성 보존

구형 추종 테스트의 `HASH`, `family()`는 현재 BBOX 테스트에서도 사용하므로
`test/lane_family_fixture.py`로 분리했다. BBOX 판정·주행 테스트가 이 파일을 직접 읽는다.
절단 추적 테스트는 BBOX 판정 테스트를 통해 같은 데이터를 사용한다.

현재 주행 구성인 두 BBOX C++ 노드, crop/frame bridge, 메시지 3개,
BBOX launch 3개, RViz 2개, 설치 의존성과 회귀 테스트는 유지했다.
`camera_bbox_lane.launch`는 주행 launch의 내부 include이므로 삭제하지 않았다.
공통 adapter 라이브러리와 기존 팀의 두 read-only monitor 및 `shadow_*` 설정도 유지했다.

주행 속도, 반복 차선변경, 내 차선 전환, BBOX 제동 제외 동작은 수정하지 않았다.
기존 터미널 명령을 그대로 사용한다. 실제 주행 노드는 재시작하지 않았다.

## 작업 범위

소스 정리는 이 패키지 안에서만 수행했다. 외부 `asmc_perception`, `lidar+sensorfusion`,
Docker 소스 설정은 변경하지 않았다. 이들에 남아 있던 기존 로컬 변경도 원복하지 않았다.
YOLO는 여전히 `Main` 코드/가중치와 `Msg` 메시지를 사용한다.

삭제 전 패키지 백업: `/tmp/lane-adapter-cleanup-85ym2nwk/before.tar.gz`.
예전 `src/planning_control` 캐시와 `.tmp/hood-check` 조사 자료는 이번 범위 밖이므로 유지했다.

## 검증

- 패키지 재빌드 통과.
- 공통 C++ adapter 27개, RViz 계약 2개 통과.
- BBOX 주행 22개, 차로 판정 10개, 절단 추적 3개 통과. 총 64개.
- BBOX 테스트는 별도 ROS master와 테스트 명령 토픽에서 실행했다.
- 주행/YOLO launch의 노드 목록 및 남은 XML 구문 확인.
- 현재 주행 C++ 2개와 Python 실행 스크립트 2개는 정리 전과 바이트 단위로 동일하다.
- 외부 카메라/라이다/Docker 파일 474개의 정리 전후 해시가 일치한다.

빌드·공통 검사 로그(asmc-ros-noetic): `/tmp/lane-cleanup-build.log`.
BBOX 검사 로그(asmc-ros-noetic): `/tmp/lane-cleanup-tests-s3gi3q0c`.
구형 devel 실행 파일 3개는 소스 제거 후 잘못 실행되지 않도록 임시 백업으로 옮겼다:
`/tmp/lane-cleanup-retired-build-z3wcjt17` (asmc-ros-noetic).
실행 중 주행 노드는 건드리지 않았으며 기존 터미널 명령은 그대로 사용한다.
