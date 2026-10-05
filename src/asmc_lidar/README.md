# asmc_lidar — 단일 LiDAR 후보 실행과 지면 연구 비교

> **현재 실행·인계 정본:** [L2-I2 인계](../../docs/packages/asmc_lidar/l2i2-handoff-20261005.md).
> `unified.launch`는 GPS/미션 모드 없이 V4 지면·군집/OBB·후보 추적을 계산하고
> `base_link` 객체를 발행한다. 검증된 pose가 있으면 별도 노드가 `map` 후보를 발행한다.
> 구조물 자동 억제는 OFF, 판제 운영 `/perception/objects`는 그대로다.
> 아래 L2-A1/A2 항목은 연구·재현 이력이며, 과거의 “추적 없음” 설명은
> 연구용 `review.launch`를 가리킨다.

손재호 `6435af1`의 **프레임별** 터널 제거·군집·L-shape OBB를 재현하고,
동일 입력에 Patchwork++ 지면 분할을 독립적으로 돌린다. 운영 LiDAR의
`/last_lidar`, `/lidar_nogps`, 판제 `/perception/objects`와 연결되지 않는다.
TRAVEL AOS는 이 패키지에 채택하지 않았다. 기존 데이터에서 실패한 원거리
차량 군집을 Patchwork++가 해결했다는 의미도 아니다.

## 빌드와 실행

### 최신 선택: 재호 평면 높이 검사 + 동일 scan 지원 영역 보완

[V4 보고서](../../docs/packages/asmc_lidar/l2a2-v4-ground-selection-20261005.md)의
후속 개발 기준은 `jaeho_support`다. **판제 운영 승인은 아니다.**
L2-I1에서는 개발용 세 방법 비교 launch의 기본 설정만 V4 전체 비교로 바꿨다.
일반도로 RANSAC의 잘못된 지면 선택을 줄였지만 기존 객체 하부 삭제와 터널
잔여 마스크 변화는 남는다. 전용 터널 필터·군집·tracker 전체의 개선으로 해석하지 않는다.

기존 bridge를 켜둔 Noetic 컨테이너에서, 이전 비교 노드를 종료한 뒤:

```bash
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash
roslaunch asmc_lidar ground_compare_live.launch method:=jaeho \
  comparison_config:=/root/ws/src/asmc_lidar/config/ground_v4/jaeho_support.yaml
```

RViz는 자동 실행된다. 상태의 config_id, ground_anchor_used와 Ground plane fits를
함께 본다. 보완 평면은 시도 번호 4, 적용 범위는 현재 관측으로 제한한다.
Patchwork++ 연구 후보는 `method:=patchwork`,
`ground_v4/patchwork_support.yaml`로 실행한다. 경사 잔여는 줄었지만 추가
장애물 소실로 미채택했으므로 운영에 사용하지 않는다.
한 master에서 동일 비교 launch를 여러 개 켜지 않는다.

과거 V1을 재현하려면 `comparison_config:=""`를 명시한다.
출력은 계속 rear-axle base_link이며 기존 Main/판제 입력과 연결되지 않는다.

### 실시간 세 방법 지면 비교

현재 수동주행하며 비교하는 진입점은 `ground_compare_live.launch`다.
기본은 세 방법 + RViz이며 기존 bridge를 사용한다.

```bash
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash
roslaunch asmc_lidar ground_compare_live.launch
# bridge가 없을 때만 start_bridge:=true
# 한 방법만 계산: method:=jaeho 또는 patchwork 또는 linefit
```

일반도로 지면 비교이며 군집/OBB/추적은 실행하지 않는다. 모든 출력은 같은
rear-axle `base_link`이며 GT 없이 작동한다. `nonground`는 장애물 확정이 아니다.
고정 높이 +25cm 보호 Patchwork++는 `patchwork_variant:=guard25` 기본이고,
보호 전 고정 높이는 `fixed_height`다. 기존 `review.launch` 기본값은 바꾸지 않는다.

[실행 옵션과 토픽](../../docs/packages/asmc_lidar/ground-method-comparison-visualization.md),
[객체 하부와 잔여 표면 구조 평가](../../docs/packages/asmc_lidar/l2a2-v1-live-ground-review-20261004.md),
[재실행 도구](../../tools/lidar/ground_live_review/README.md)를 본다.

환경 조정 검토는 선택적인 `comparison_config`로 실행한다. **현재 비교 launch에서
생략하면 V4 전체 비교를 사용한다.** 아래 V2 설정은 과거 실험을 명시적으로 재현할 때 쓴다.

```bash
# 파라미터 조정 없이 linefit의 sector/점 처리 누락만 수정한 대조군
roslaunch asmc_lidar ground_compare_live.launch \
  comparison_config:=/root/ws/src/asmc_lidar/config/ground_v2/lf_covered.yaml
```

파일이 잘못되거나 알려지지 않은 설정이 있으면 시작을 거부한다. 설정 ID와 SHA는
`/lidar_candidate/ground_live/status`에 표시한다. 12개 단일 변수 설정과 원인
진단·재실행은 [V2 실험 안내](../../tools/lidar/ground_tuning/README.md)를 따른다.
설정 파일을 바꿔 시험할 때에는 노드를 재시작하여 적응 상태를 초기화한다.

### V3 공통 roll·pitch 및 터널 평면 진단 (연구용)

V3 설정을 **명시적으로** 선택하면 같은 scan의 roll·pitch를 세 방법에
적용하고 linefit의 모든 sector·점을 처리한다. 현재 V4 비교 기본값도 같은
roll·pitch와 linefit 전체 sector를 사용한다. 판제 토픽은 바뀌지 않는다.

```bash
roslaunch asmc_lidar ground_compare_live.launch \
  method:=all rviz:=true \
  comparison_config:=/root/ws/src/asmc_lidar/config/ground_v3/roll_pitch_all.yaml
# bridge가 없을 때만 start_bridge:=true 추가
```

RViz의 `Ground plane fits`에서 선택된 재호 평면의 중심·법선과 거부
시도를 볼 수 있다. 화면이 지워진 경우에는 `ground_live/status`의
IMU·센서 중단·queue 상태를 먼저 본다. 2026-10-05 동일 입력 GT 비교에서
linefit을 **후속 연구용 지면 입력**으로 선택했지만 보도·구조물 잔여와
실제 v3 터널 연속주행 gate가 남아 운영 채택은 하지 않았다.
이후 [새 터널·횡경사 주행](../../docs/packages/asmc_lidar/l2a2-v3-live-drive-review-20261005.md)을
완료했으나 램프 잔여 한계가 확인돼 전 구간 승격은 보류했다. 기존 운영 출력은 유지한다.
저장된 실제 터널을 기존 RViz로 볼 때는 live 비교 노드를 종료한 뒤 다음을 사용한다.

```bash
roslaunch asmc_lidar ground_compare_recording.launch \
  bag:=/data/lidar_assessment/20261005_ground_v3_drive/tunnel_path_all/drive_0.bag \
  start:=110
```

기록 launch는 candidate 결과만 재생하므로 기존 bridge를 유지할 수 있다.
원본 센서·제어·/clock은 재생하지 않는다. 램프 및 재평가 명령은 아래 안내를 본다.
[V3 결과와 미완료 조건](../../docs/packages/asmc_lidar/l2a2-v3-attitude-tunnel-selection-20261005.md),
[V3 재실행 명령](../../tools/lidar/ground_v3/README.md),
[고정 입력 manifest](../../config/baselines/lidar_l2a2_v3_20261005.yaml)를 본다.

### 기존 reference 및 Patchwork 단독 검토

Noetic 컨테이너의 저장소 루트 `/root/ws`에서:

```bash
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps asmc_lidar -j2
source /root/ws/devel/setup.bash
roslaunch asmc_lidar review.launch mode:=reference rviz:=true
roslaunch asmc_lidar review.launch mode:=ground_review rviz:=true
```

입력은 기존 bridge가 발행하는 `/velodyne_points`와 `/imu`다. 이번 실행기는
IMU가 관측 시각보다 늦지 않고 100ms 이내인 경우에만 계산한다. 구독
이름은 `cloud_topic`, `imu_topic` 인자로 바꿀 수 있다. 일반도로의
1.5m 군집 기준을 검토하려면 `profile:=general`을 추가한다. 일반도로의 분리된 지면 소스 단계는 100-frame mask parity를 확인했지만 전체 출력 parity는 아직 승인하지 않았다. 기본
`profile:=tunnel`은 Capture 검증에서 고정한 1.0m 터널 기준이다.

실행 중인 운영 ROS master와 분리해 bag을 검증할 때에는 `ROS_MASTER_URI`를
별도 포트로 설정하고 동일 환경을 쓰는 터미널에서 bridge/bag, review.launch,
`rostopic`을 실행한다. 이 패키지는 UDP 포트를 열지 않는다.

## 출력 계약

모든 점군·OBB는 rear-axle `base_link` 좌표이며 입력 scan의 원본 header stamp를
유지한다. `/lidar_candidate/` 아래에 다음을 발행한다.

| 토픽 | 내용 |
|---|---|
| `input_base` | 수신 점군을 장착 변환한 결과 |
| `ground`, `nonground`, `uncertain` | `ground_review`는 Patchwork++ 분류; `reference`는 기준선 ground/kept/기타 제거점 |
| `reference_obbs` | 현재 프레임의 기존 L-shape OBB, 추적 ID·속도 없음 |
| `status` | 입력·IMU·중복 stamp·처리·카메라 미수신·map 미가용 사유 |

RViz의 `ground`와 `nonground`는 모드에 따라 출처가 달라진다. `ground_review`의
`nonground`는 장애물 확정이 아니고, `reference`의 `uncertain`은 벽·천장 등
기준선이 제외한 점이다. 카메라 입력을 연결하지 않은 실시간 실행은 명시적으로
`missing`이며 유효 빈 검출이나 보호 OFF로 취급하지 않는다. 위치가 필요한
판제용 `map` 객체·추적 ID·속도는 이 **연구용** `review.launch`에서 발행하지 않는다.
신규 `unified.launch`의 별도 후보 토픽은 L2-I2 인계를 따른다.

## 고정 Capture 재생

기준 SHA와 오프라인 재생은 다음처럼 확인한다. 출력은 기존 파일이 없는 새 경로를
지정한다. `$ASMC_DATA`는 팀 저장소와 나란한 `data` 디렉터리다.

```bash
export ASMC_DATA=/data  # 데이터가 이 경로에 마운트된 컨테이너의 예
python3 -B tools/lidar/l2a1/check_manifest.py --repo-root . --data-root "$ASMC_DATA"
/root/ws/devel/lib/asmc_lidar/asmc_lidar_offline \
  "$ASMC_DATA/lidar_assessment/20260929/l1b4b_capture_gt/inputs_v4/snapshots.jsonl" \
  src/asmc_lidar/config/autonav_v3.yaml src/asmc_lidar/config/reference_tunnel.yaml \
  reference /tmp/l2a1_reference.jsonl
python3 -B tools/lidar/l2a1/verify.py \
  --candidate /tmp/l2a1_reference.jsonl \
  --baseline "$ASMC_DATA/lidar_assessment/20260929/l1b5a_capture_final_v1/run/geometry.jsonl" \
  --snapshots "$ASMC_DATA/lidar_assessment/20260929/l1b4b_capture_gt/inputs_v4/snapshots.jsonl"
```

`ground_review`도 같은 재생 명령의 mode만 교체한다. 비용 비교는 `catkin_make --only-pkg-with-deps asmc_lidar -j2 -DCMAKE_BUILD_TYPE=Release`로 빌드한 뒤 `python3 -B tools/lidar/l2a1/summarize.py --candidate <output.jsonl> --snapshots <snapshots.jsonl>`을 실행한다. 오프라인 결과에는
원본 점→voxel 인덱스, 기준선 label·OBB, Patchwork++ 원본 인덱스 분류,
단계별 비용이 있다. 정차 warmup에만 동일 stamp 반복을 허용하며 실시간
중복 stamp는 거부한다. Capture 105장면은 반복 관측을 포함하므로 독립
상황 105개의 검출 정확도 수락으로 해석하지 않는다.

고정 근거는 [L2-A1 manifest](../../config/baselines/lidar_l2a1_runtime_20261002.yaml),
[회귀 보고서](../../docs/packages/asmc_lidar/l2a1-runtime-review-20261002.md),
[판제 소비 계약](../../docs/contracts/perception/lidar-planning-consumer-v1.md)에 기록한다.
이 패키지의 기준 코드와 Patchwork++의 SHA·라이선스는 `vendor/`와 manifest를
함께 확인한다. vendored Patchwork++ 파일은 BSD-2-Clause이며 패키지 자체의 MIT 표기와 구분한다. `tools/lidar/frame_time`의 B1 구현은 한 소스로 빌드한다.

## L2-A2 네 조건 지면 비교

`ground_review` ROS 시각화는 여전히 **비교용**이다. 현재 고정 설정의
Patchwork++는 v3 Capture에서 같은 점 표현의 기존 지면 처리보다
차량·일반 장애물 점 보존과 지면 제거가 모두 낮아 **운영 미채택**으로
판정했다. 군집·추적·판제 입력은 그대로다. 상세 수치, 지표의 분모,
그림, 일반도로 source-stage parity와 기존 100-frame 보조 자료는
[L2-A2 보고서](../../docs/packages/asmc_lidar/l2a2-ground-comparison-20261002.md),
원본 SHA는 [L2-A2 manifest](../../config/baselines/lidar_l2a2_ground_20261002.yaml)를 본다.

```bash
export ASMC_DATA=/data
SNAP="$ASMC_DATA/lidar_assessment/20260929/l1b4b_capture_gt/inputs_v4"
OUT="$ASMC_DATA/lidar_assessment/20261002_l2a2_ground_compare"
./devel/lib/asmc_lidar/asmc_lidar_ground_compare "$SNAP/snapshots.jsonl" \
  src/asmc_lidar/config/autonav_v3.yaml src/asmc_lidar/config/reference_tunnel.yaml \
  tunnel "$OUT/new_four_way.jsonl"
python3 -B tools/lidar/l2a2/score.py --comparison "$OUT/new_four_way.jsonl" \
  --admission "$SNAP" --output "$OUT/new_score"
python3 -B tools/lidar/l2a2/test_offline_inputs.py \
  --binary ./devel/lib/asmc_lidar/asmc_lidar_offline \
  --snapshots "$SNAP/snapshots.jsonl" \
  --calibration src/asmc_lidar/config/autonav_v3.yaml \
  --reference src/asmc_lidar/config/reference_tunnel.yaml
```

새 출력 경로를 지정하고 기존 `$ASMC_DATA` 결과는 덮어쓰지 않는다. 지면
마스크의 `nonground`는 장애물 확정이 아니며 `StandingOBJ(109)`는
차량/보행자 recall 분모가 아니다. 2026-10-03 사용자 요청으로 군집화 전 지면 검토를 이어갔다.
[L2-A2-H1 높이 갱신 비교](../../docs/packages/asmc_lidar/l2a2-h1-height-review-20261003.md)에서는
격리 fixed-height 후보가 높이 붕괴를 막았지만 차량 2/35 완전 소실이 남았다.
다음 검토 제안은 원거리 지역 평면의 객체 오분류이며, 아직 군집화를 바꾸거나 이 패키지의
ground_review 기본값에 H1 후보를 연결하지 않았다. 실행은 기존과 동일하다.
