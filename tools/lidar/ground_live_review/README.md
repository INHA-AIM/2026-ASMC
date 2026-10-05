# LiDAR 실시간 지면 비교의 회귀 도구

L2-A2-V1은 `asmc_lidar::GroundComparison`과 같은 코어로 세 방법을 재처리하고
GT의 객체 온전함·배경 잔여 구조를 분석한다. 운영 토픽/판제/군집화는 바꾸지 않는다.
[보고서](../../../docs/packages/asmc_lidar/l2a2-v1-live-ground-review-20261004.md)에
현재 수치와 확인하지 못한 범위를 기록했다.

## 실행 환경과 입력

Noetic 컨테이너 `/root/ws`, 데이터 `/data`. 평가 도구는 Python3 NumPy/SciPy,
ROS 계약 시험은 rospy/tf가 필요하다. 입력은 이미 승인된 raw4,048·sync1,439
snapshot이며 기존 source.json, pitch/roll, 시간 출처를 보존한다.
동기 Sensor Record의 arrival은 synthetic이고 실측 네트워크 지연이 아니다.
GT semantic/instance는 채점기에만 입력되고 backend에는 들어가지 않는다.

```bash
source /opt/ros/noetic/setup.bash
cd /root/ws
catkin_make --pkg asmc_lidar -j3
source devel/setup.bash
cd build/asmc_lidar && ctest --output-on-failure
cd /root/ws
python3 tools/lidar/ground_live_review/test_metrics.py

IN=/data/lidar_assessment/20261004_gt_dual_ground
OUT=/data/lidar_assessment/20261004_ground_live_v1
# 결과 파일은 없는 이름을 지정해야 한다. 재생 순서/phase를 바꾸지 않는다.
./devel/lib/asmc_lidar/asmc_lidar_ground_live_offline \
  "$IN/sync/snapshots.jsonl" src/asmc_lidar/config/autonav_v3.yaml \
  "$OUT/new_sync.jsonl" guard25 all
python3 tools/lidar/ground_live_review/evaluate.py \
  "$IN/sync" "$OUT/new_sync.jsonl" "$OUT/new_sync_metrics"
# raw도 동일하게 sync를 raw로 바꾼다.
```

core의 mode는 `all|jaeho|patchwork|linefit`, Patchwork variant는
`guard25|fixed_height`다. 프로그램은 기존 출력을 덮어쓰지 않는다.
`--stride`로 metric smoke만 줄일 수 있지만 공식 진단값은 stride1 전수다.
개별 frame JSONL에는 객체 관측 높이3구간·관측범위·3개 거리 topology까지
들어 있고 summary는 weighted 통계다. summary만 보고 모든 객체가 같은
정도로 보존됐다고 해석하지 않는다.

## 기존 마스크와 반복 검증

```bash
python3 tools/lidar/ground_live_review/verify_frozen.py \
  "$OUT/new_sync.jsonl" "$IN/sync/results/jaeho_voxel_pitch.jsonl" \
  "$IN/reports/sync_patchwork_vlp16_sweep_v2.jsonl" \
  "$IN/reports/sync_linefit_vlp16_v1.jsonl" --allow-ordered-linefit-delta
python3 tools/lidar/ground_live_review/verify_repeat.py \
  "$OUT/raw_ordered.jsonl" "$OUT/raw_ordered_repeat.jsonl"
```

기본 parity는 세 방법 전부 strict다. 명시적인
`--allow-ordered-linefit-delta`만 bin 삽입 race 수정으로 인한 linefit의
각 원본 인덱스 차이를 보고한다. 재호·PP 차이는 여전히 실패한다.
임계값·군집화 튜닝 차이를 이 옵션으로 승인하면 안 된다.
`generate_linefit.py`는 고정 vendored 소스 SHA를 검사해 build 사본만 수정한다.
원본의 별개 atomic z/range 갱신은 동일 입력 반복에서도 분류가 달랐다.
fit/assignment의 thread tail 한계는 이번에 바꾸지 않는다.

## 독립 ROS 계약 검사

서로 다른 두 터미널에서 같은 별도 master를 사용한다. 운영 master11311과
같은 토픽 이름을 써도 master가 달라 충돌하지 않는다.

```bash
export ROS_MASTER_URI=http://127.0.0.1:11349
roslaunch -p 11349 asmc_lidar ground_compare_live.launch \
  rviz:=false cloud_topic:=/review/input imu_topic:=/review/imu
```

```bash
export ROS_MASTER_URI=http://127.0.0.1:11349
python3 tools/lidar/ground_live_review/test_ros_contract.py \
  "$IN/sync" "$OUT/sync_ordered.jsonl"
```

입력 측 IMU callback이 먼저 도착하도록 제어한 검사다. 실제 transport의
동기성을 보증하지 않는다. 각 점군의 source_index, intensity, 원본 ns stamp,
base_link 위치와 ground/structural/native partition을 모두 확인한다.

## live 계측

```bash
# 일반 master에서 실행. 이미 bridge가 있으므로 다시 켜지 않는다.
roslaunch asmc_lidar ground_compare_live.launch
python3 tools/lidar/ground_live_review/monitor.py "$OUT/new_live.json" --seconds 65
```

센서 stamp와 ROS 수신 시각을 구분하고 처리시간은 steady clock으로 잰다.
queue counter는 callback 이후의 내부 누락이며 UDP 또는 ROS 자체 누락 총수가
아니다. GT는 구독하지 않는다. 이 패키지의 live 결과를 판제 입력으로 바꾸지 않는다.

`simulator_smoke.py`는 명시적 `--confirm-simulator-move`에서만 작은 전진을
시도하고 pose/gear/mode를 복원한다. 정차 여부·전방 객체 검사에서 거부되면
상태를 바꾸지 않는다. 이번 실제 현장에서는 전방 객체 때문에 거부되어
운전 수락에 사용하지 않았다. full-route 또는 collision-free 증명이 아니다.

## 고정 산출물

`build_manifest.py`는 코드/설정/입력·GT/결과 hash를 기록한다.
원본은 읽기 전용이고 입력 인벤토리만 새 산출물 디렉터리에 만든다.
집계 manifest는 저장소 `config/baselines/lidar_l2a2_v1_live_20261004.json`에,
자세한 원본 상대경로·크기·SHA는 그 안의 input inventory SHA로 연결된다.
`plot.py`는 대표 고정 frame의 BEV/측면을 원본점 기준으로 저장한다.
그림이나 backend 군집을 정답으로 사용하지 않는다.
