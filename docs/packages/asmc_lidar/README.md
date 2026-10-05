# `asmc_lidar` L2-I1 당시 인계 — 한 경로, 구조물 억제 기본 OFF

> 이 문서는 **L2-I1 당시 기록**이다. 현재 추적·메시지·map 어댑터의 정본은
> [L2-I2 인계](./l2i2-handoff-20261005.md)이며, 아래 “추적 미제공” 설명은
> 현재 `unified.launch`에 더 이상 적용되지 않는다. 연구·회귀 명령은 보존한다.

이 문서는 `feat/asmc-lidar-integration`의 **후속 개발용 실행 기준선**이다. 운영 판제 입력의 교체나 전 구간 인지 성능 승인이 아니다. 기존 `lidar+sensorfusion` nested workspace, `/last_lidar`, `/lidar_nogps`, `/perception/objects`, 메시지 MD5, 센서·브릿지 설정은 그대로 둔다.

## 실행과 보는 방법

브릿지는 기존처럼 별도 터미널에서 실행한다. 이 독립 worktree를 시험하려면
컨테이너의 `/root/ws`가 `asmc-lidar-integration`을 마운트했는지 먼저 확인한다.
기존 `asmc-ros-noetic` 컨테이너가 다른 worktree를 마운트했다면 아래 `up`이
**같은 이름의 컨테이너를 재생성**하므로 실행 중인 노드를 종료하고 전환할 때만 수행한다.
`ASMC_DATA`와 `TWINLITE_SOURCE`를 별도로 지정했던 환경에서는 그 값을 유지한다.

```bash
cd /path/to/asmc-lidar-integration
export ASMC="$PWD"
./scripts/docker_ros_up.sh up
docker exec asmc-ros-noetic test -f /root/ws/src/asmc_lidar/launch/unified.launch
```

이후 Noetic 컨테이너의 `/root/ws`에서 빌드·환경을 올린 뒤:

```bash
source /opt/ros/noetic/setup.bash
cd /root/ws
catkin_make --only-pkg-with-deps asmc_lidar -j2
source devel/setup.bash
roslaunch asmc_lidar unified.launch
# bridge가 없을 때만 start_bridge:=true (local/competition/receive 기본)
```

필수 입력은 `/velodyne_points`, `/imu`뿐이다. `/gps`, HD맵, 카메라는 현재 이 노드의 구독자가 아니다. 일반도로→터널→복귀 또는 GPS 정상→무효가 와도 같은 `ground_v4/jaeho_support.yaml`, ROI, 10cm voxel, roll·pitch, 1.5m Euclidean/10..10000 voxel, L-shape OBB를 사용한다. `unified.launch`에 구간·미션·GPS 모드 인자는 없다. 센서 원점의 수평화 좌표는 내부용이며 ROS 공개 frame은 **후륜축 `base_link`**다.

| 후보 토픽 | 의미 |
|---|---|
| `/lidar_candidate/unified/nonground` | 지면 외 원본점. 구조물 의심점도 삭제하지 않은 상태. `source_index`는 해당 scan의 원본 인덱스 |
| `/lidar_candidate/unified/structure_evidence` | 기존 RANSAC 탐색에서 거부된 평면 지지점. **벽 GT 또는 안전하게 무시할 물체라는 뜻이 아님** |
| `/lidar_candidate/unified/markers` | 주황 OBB=프레임별 미분류 후보, 반투명 청록=미확정 구조물 지지 범위. 번호는 track ID가 아님 |
| `/lidar_candidate/unified/status` | 원본 stamp, 설정 SHA, 객체/구조물 후보 수, 억제 OFF, 계산시간, 입력 상태 |
| `/lidar_candidate/ground_live/jaeho/*` | 같은 scan의 V4 지면 분류 연구·회귀 토픽 |

RViz는 `unified.launch`에서 열리고, 오래된 입력·무효 IMU·시각 역행 시 점군/marker를 지운다. `rviz:=false`로 끌 수 있다. `status=valid`는 입력 유효성이지 객체 검출 성공이 아니다. 분류·속도·지속 ID는 아직 **unknown/미제공**이며 0속도나 차량으로 위장하지 않는다. 1.5m 군집은 재현 기준선일 뿐 전 구간 최적화된 임계값이 아니다. 지면 단계가 이미 삭제한 객체 하부점은 이 코어가 복구하지 못한다.

세 지면 방법을 수동 주행 중 비교할 때는 단일 검출 launch를 종료하고 다음을 실행한다.

```bash
roslaunch asmc_lidar ground_compare_live.launch
# 기본 all / V4 재호·Patchwork++·linefit 최신 비교 / RViz true
# 하나만 계산: method:=jaeho|patchwork|linefit
```

RViz의 Patchwork++는 **실험용·미채택**이다. 비교 결과를 운영 후보로 자동 연결하지 않는다. 예전 V1 재현이 필요하면 `comparison_config:=""`를 명시한다. 자세한 비교 근거는 [V4 선택 보고서](./l2a2-v4-ground-selection-20261005.md)에 있다.

## 코어와 확장 지점

`ground_live.cpp`는 공통 ROI·voxel·수평화와 V4 지면 라벨, RANSAC/동일-scan 보완 근거를 생성한다. `unified.cpp`는 **같은 출력과 원본점 계보**로 비지면 voxel을 군집화하고 L-shape OBB를 만든다. OBB의 중심·8코너·진행축은 실제 적용 수평화의 역변환 후 장착 변환을 각 한 번 적용한다. 원본점과 `structural_removed` 마스크는 모두 보존한다. 지면 탐색에서 거부된 평면의 연결 지지 구간은 미확정 구조물 근거로 분리하지만, 현재 판정이 객체점 삭제에 참여하지 않는다.

`unified.hpp::OptionalContext`는 향후 HD맵/카메라 문맥을 위한 source·frame·관측 시각·품질의 공통 입력이다. 누락/stale/map frame 그대로 전달된 입력은 unknown이다. 이번 버전은 문맥으로 객체를 확정·삭제하지 않는다. 카메라 주행가능영역의 장애물 구멍은 삭제 근거가 아니다. 나중에는 객체 footprint와 HD맵 차로·주행 영역의 **겹침/인접/횡단 가능성**을 별도 속성으로 계산해야 하며, GPS는 map 출력 가용성에만 관여해야 한다. local 추적 상태를 GPS로 초기화하면 안 된다.

손재호의 다음 작업은 구조물 지지 품질과 억제 회귀, 군집 분할·합침, 지속 ID·분류·속도·관측 품질이다. 기존 객체 성공/실패 분모와 벽 근처 차량·낮은 장애물·인접/횡단 객체 시험을 통과하기 전에는 구조물 억제를 켜지 않는다. 최종 판제용 typed 메시지와 GPS 정상/음영 map 출력은 tracker·판제와 별도 계약 후 연결한다. 현 표준 점군/Marker/Diagnostic 후보 토픽을 판제의 `/perception/objects` 대신 연결하면 안 된다.

## 회귀·롤백

```bash
# Noetic 컨테이너에서
cd /root/ws/build/asmc_lidar && ctest --output-on-failure
# 고정 snapshot을 같은 코어로 재처리 (출력이 이미 있으면 오류; 덮어쓰지 않음)
cd /root/ws
devel/lib/asmc_lidar/asmc_lidar_unified_offline \
  /data/lidar_assessment/20261004_gt_dual_ground/sync/snapshots.jsonl \
  src/asmc_lidar/config/autonav_v3.yaml /data/lidar_assessment/<새_실험명>/unified_sync.jsonl
```

V4 고정 입력·원본 코드 SHA·해시는 [manifest](../../../config/baselines/lidar_l2a2_v4_20261005.yaml), 재실행법은 [실험 도구](../../../tools/lidar/ground_v4/README.md), 원인·실패 사례는 [V4 보고서](./l2a2-v4-ground-selection-20261005.md)에 보존했다. 원본 손재호 revision `6435af1`, V4 구현 `7898ffe`, 독립 연구 브랜치 `feat/asmc-lidar-runtime`을 비교 기준으로 유지한다. 연구 자료를 새 운영 계약으로 오해하지 않는다.

롤백은 후보 `unified.launch`만 종료하면 된다. 기존 Main·판제·센서 설정을 바꾸지 않았으므로 운영 입력을 되돌리는 조치는 없다. 후보 승격 후라도 객체점 추가 소실, IMU/stamp 무효의 정상 빈 배열 오인, GPS 변화에 따른 local 결과 변화, 15Hz에서 누락/지연이 확인되면 억제·운영 연결을 보류한다.

이 패키지에 포함된 기존 `review.launch`/`ground_compare.launch`의 `general/tunnel` 재현 인자는 **연구용 과거 기준선**이다. 신규 `unified.launch`의 모드가 아니다.
