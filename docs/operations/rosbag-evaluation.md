# rosbag 기록·재생·평가

> **문서 역할:** Active Guide — Real Time UDP raw/result bag의 재현 가능한 운용
>
> 날짜가 있는 rate·latency와 종료된 GT 실험은
> [transport 검증 기록](../records/simulator/runtime-transport-2026-09.md)에 보존한다.

## 1. 사용 원칙

- 대회 운용·실시간 평가는 v2 sensor set의 Real Time UDP raw bag을 사용한다.
- 모델·판제 A/B는 같은 raw bag을 재생하고 모델별 result bag을 분리한다.
- 단일-frame 학습 GT는 Capture의 RGB·Semantic·Instance·box를 사용한다.
- Semantic, Instance와 `/Object_topic`은 개발 전용이며 모델 입력이나 본선 profile에 넣지 않는다.
- direct ROS/rosbridge GT와 Streaming GT는 종료된 실험으로 신규 개발 기본값이 아니다.
- raw run은 수정·덮어쓰기·재직렬화하지 않고 SHA-256과 lineage를 유지한다.

| 목적 | 경로 |
|---|---|
| 단일-frame supervised GT | v2 + Capture |
| 고주기 RGB sequence | v2 + Simulation Time Sensor Record |
| 실제 비동기·지연 평가 | v2 + UDP receive + raw bag |
| 모델 A/B | 동일 raw replay + 모델별 result bag |
| 과거 GT transport 재현 | privileged profile, 배포 금지 |

Capture는 pilot에서 약 1.4~1.7 Hz에 포화되고 요청마다 시뮬레이션이 멈췄으므로 주행 temporal
성능 측정에 쓰지 않는다. Sensor Record는 장착된 센서만 기록하며 Semantic/Instance channel을
자동 생성하지 않는다.

## 2. profile과 저장 구조

profile은 `src/data_collection/config/bag_profiles/`에 있다.

| profile | 내용 | 용도 |
|---|---|---|
| `full_raw` | RGB 4, Intensity LiDAR, GPS/IMU/Ego/TF/control | 전체 operational 입력 |
| `camera_raw` | RGB front/left/right와 차량 상태 | 카메라 모델 A/B |
| `lidar_raw` | Intensity LiDAR와 GPS/IMU/Ego | LiDAR 모델 A/B |
| `camera_semantic_result` | 카메라 추론 결과와 diagnostics | replay 결과 |
| `lidar_dev_gt_udp` | ID 5/11, Object와 상태 | archived privileged 실험 |
| `dev_oracle_udp` | RGB/Semantic, ID 5/11, Object | archived privileged 실험 |
| `dev_oracle_raw` | `dev_oracle_udp` canonical alias | archived privileged 실험 |

일반 profile에 GT topic이 들어가거나 GT profile에 `privileged_gt: true`,
`deployment_allowed: false`가 없으면 recorder가 시작하지 않아야 한다. 동일 topic publisher가
둘 이상이거나 bridge timestamp mode와 profile이 다를 때도 기록을 중단한다.

```text
/data/rosbags/
├── raw/<run_id>/
│   ├── metadata.yaml
│   ├── environment_timeline.jsonl
│   └── bags/<run_id>_0.bag
├── results/<raw_run_id>/<model_id>/
│   ├── metadata.yaml
│   └── bags/<model_id>_0.bag
├── audits/<run-or-comparison>/audit.json
└── rejected/<run_id>/...
```

raw는 무압축, result는 LZ4, split은 4 GiB, buffer는 1 GiB이며 시작 전에 20 GiB 여유를
확인한다. run ID는 KST 시작 시각, map, scenario, weather, simulation hour와 controller로
생성한다. gRPC 환경 조회에 실패해도 기록은 계속하고 `unknown`과 오류를 metadata에 남긴다.

```text
20260909_143012_kcity2025_comp_sample_foggy_1pm_wheel
```

## 3. raw 기록

먼저 receive mode bridge를 실행한다.

```bash
./scripts/udp_bridge.sh

roslaunch data_collection bag_recorder.launch \
  profile:=full_raw scenario:=comp_sample controller:=wheel
```

환경은 2초마다 `environment_timeline.jsonl`에 기록한다. gRPC를 사용할 수 없지만 조건을 알고
있으면 `weather:=foggy sim_hour:=13`처럼 시작값을 지정할 수 있고 override 사실도 남는다.

Ctrl-C 시 recorder는 rosbag에 SIGINT를 전달하고 closed bag SHA-256, 종료 rate와 `complete`
상태를 기록한다. 강제 종료돼 `.active` 또는 `recording` 상태가 남으면 원본을 보존하고
`rosbag reindex/fix` 여부를 별도 판단한다.

잘못된 run은 삭제하지 않고 사유와 함께 이동한다. 해당 raw를 parent로 둔 result도 같이
이동한다.

```bash
rosrun data_collection reject_bag_run.py \
  --run-id <run_id> --reason "scenario setup before recording was incomplete"
```

## 4. replay와 result lineage

live bridge를 끈 뒤 다음 순서를 지킨다.

```text
paused replay controller → inference node → result recorder → start service
```

첫 터미널:

```bash
roslaunch data_collection bag_replay.launch \
  bag_path:=/data/rosbags/raw/<raw_run_id> \
  required_profile:=camera_raw
```

두 번째 터미널에서 inference node를 시작한다. 카메라 runtime은 CUDA 첫 호출로 초반 frame을
잃지 않도록 기본 warmup 1회를 수행한다.

세 번째 터미널:

```bash
roslaunch data_collection bag_recorder.launch \
  profile:=camera_semantic_result \
  parent_raw_run_id:=<raw_run_id> \
  model_id:=<model_id> \
  checkpoint:=/artifacts/runs/<checkpoint>.pth
```

recorder가 clock 대기 상태에 들어간 뒤 네 번째 터미널에서 시작한다.

```bash
rosservice call /asmc_bag_replay/start
```

replay controller가 `/use_sim_time=true`를 먼저 설정하고 종료 후 원래 값을 복원한다. inference나
recorder를 먼저 띄우면 wall clock이 고정돼 result latency가 깨질 수 있다. replay 종료 후
기본 2초 post-roll을 기다린 다음 recorder와 inference를 닫는다.

result metadata에는 parent raw ID, model ID, checkpoint path/SHA-256, Git 상태와 diagnostics가
필수다. 존재하지 않는 checkpoint, 중복 model ID 또는 sim time이 아닌 상태에서는 기록을
시작하지 않는다. GT bag replay는 oracle 분석에서만 `allow_privileged_gt:=true`로 연다.

## 5. 감사와 비교

audit는 bag을 읽어 topic count/rate, header 역행, bag-header age와 필요한 pair gap을 계산하고
`audits/`에 결과를 쓴다.

```bash
rosrun data_collection audit_rosbag_run.py \
  /data/rosbags/raw/<run_id> \
  --json-out /data/rosbags/audits/<run_id>/audit.json

# 자동 gate에서 실패를 exit 2로 반환
rosrun data_collection audit_rosbag_run.py \
  /data/rosbags/raw/<run_id> --strict
```

비교할 때 다음을 따로 보고한다.

- sensor 설정 Hz와 subscriber 실측 Hz
- source/receive/result stamp와 header regression
- arrival gap과 실제 sequence drop
- 모델 input/output/replaced frame과 decode/inference error
- stage latency, end-to-end p50/p95, CPU/GPU/VRAM
- bag·checkpoint·config·Git SHA와 privileged/deployment 상태

sequence가 없는 stream의 긴 gap은 drop count가 아니다. `processing_fps`가 3-view 합계인 경우
카메라당 FPS와 구분하고, latest-frame의 `replaced_by_view`도 network loss로 해석하지 않는다.

## 6. 최소 수집 체크리스트

- [ ] 올바른 sensor/network profile과 `timestamp_mode=receive`
- [ ] profile 필수 topic의 publisher가 정확히 하나
- [ ] scenario, controller, weather와 simulation hour 기록
- [ ] raw free space와 run ID 중복 확인
- [ ] 정상 Ctrl-C 종료와 bag SHA-256 생성
- [ ] audit 결과와 미달 topic 기록
- [ ] result의 parent raw·model·checkpoint lineage 확인
- [ ] privileged input이 deployment result에 없는지 확인

구현 상세는 [data_collection README](../../src/data_collection/README.md), runtime UDP 진단은
[운영 가이드](./README.md)를 따른다.
