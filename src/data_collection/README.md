# data_collection

Capture 파일·raw/result bag·보조 ROS snapshot과 데이터 계보를 관리합니다.
일반 카메라/판제 launch나 빌드로 스폰·주행·녹화가 시작되지 않습니다.

| 목적 | 실행/문서 | 권한·한계 |
|---|---|---|
| 학습 RGB/GT 파일 | [Capture 절차](docs/capture-workflow.md) | Capture1Hz·한 요청, 원본 보존/파일 QA |
| 자동 Capture 주행 | capture_route_collection.launch, [파일럿 기록](../../docs/records/camera/camera-mpc-object-collection-pilot.md) | 개발 Ego·gRPC/MPC/PP 제어, 운영 주행과 동시 실행 금지 |
| 실제 비동기 지연·재현 | bag_recorder/bag_replay.launch, [bag 운영](../../docs/operations/rosbag-evaluation.md) | raw 불변·result별 모델/설정/부모 SHA |
| 일반 토픽 snapshot | sync_collector.launch | latest_timer 보조 수집; 외부 동기 tick/paired GT 보장 아님 |
| TwinLite 학습 파생물 | [native/cache/dataset 절차](docs/segmentation-dataset.md) | 기존384×640 cache·v6 split 이력, runtime 해상도와 별개 |

## Capture 시작과 복사

MORAI의 검증된 v3 센서셋·Capture Mode에서 먼저 정차3회 RGB·semantic·instance·BBox 완성을 확인합니다.
GT streaming bridge 확장은 필요하지 않습니다. manual trigger는 차량 제어를 하지 않으며 자동 route collector와 구분합니다.

```bash
roslaunch data_collection capture_mode_collector.launch capture_hz:=1.0 max_captures:=100
# 보조 토픽 snapshot만 필요한 경우
roslaunch data_collection sync_collector.launch profile:=dry_run
```

자동주행 수집은 승인된 배치·장부로만 실행하며 제어 생산자를 하나만 둡니다.
PP 대체는 Pure Pursuit + P 속도 제어이고 PID라고 안내하지 않습니다.
이 문서 갱신에서는 수집·스폰·주행을 실행하지 않습니다.

수집 종료 후 WSL host에서 복사합니다. run ID는 실제 수집 ID로 바꿉니다.

```bash
python3 scripts/sync_capture_mode_data.py --run-id <run_id> --profile bbox3 --dry-run
python3 scripts/sync_capture_mode_data.py --run-id <run_id> --profile bbox3 --jobs 4
```

bbox3_partial은 BBox 누락을 명시하며 누락 뷰를 빈 정답으로 만들지 않습니다.
원본은 `$ASMC_DATA/datasets/<run_id>`, 실행 장부는 `capture_runs/<run_id>`, 학습 export는
`dataset_versions/<version>`입니다. Windows 저장root·native-ID·완전성은 Capture 절차를 봅니다.

## Raw/result bag

```bash
roslaunch data_collection bag_recorder.launch profile:=full_raw scenario:=manual_route controller:=wheel
# live bridge 종료·별도 재현 환경에서 paused 시작
roslaunch data_collection bag_replay.launch bag_path:=/data/rosbags/raw/<run_id> required_profile:=camera_raw
```

replay clock → 추론/recorder → 재생 시작 순서를 [bag 운영](../../docs/operations/rosbag-evaluation.md)대로 지킵니다.
Capture의 simulator 정지와 Real Time UDP 성능은 같은 실험이 아닙니다.
Sensor Record는 현재 장착 센서의 파일 저장이며 GT를 자동 생성한다고 보장하지 않습니다.
GT·gRPC는 개발용이고 competition profile의 Object/Semantic/Instance socket은 기본 생성되지 않습니다.

[인자 전체](../../docs/operations/launch-reference.md) · [데이터 준비](../../docs/operations/data-preparation.md) ·
[데이터 계약](../../docs/contracts/dataset-labeling-split.md)
