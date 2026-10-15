# 음영구간 family 실행 최적화 — 2026-10-15

PR #85의 후속 구현이다. 시작점은 [실행 계약](../../contracts/perception/camera-runtime-v1.md),
이전 증거는 [TensorRT 변환·부하 검증](camera-twinlite-tensorrt.md)이다.
현재 기본은 독립 PyTorch이며 TensorRT 기본 전환과 PR 병합은 보류한다.

## 실행 경로와 수요

전방544×960 B1·좌우480×640 B2, YOLO/TwinLite15Hz·ResNet10Hz를 유지한다.
추론과 family 지면 투영은 같은 `lane_input.yaml`을 읽는다. 입력 해상도와 BEV의
미터 단위 격자는 별개이며, 보정·거리·차선 유효성 임계값은 변경하지 않았다.

필수 경로는 model-grid → 카메라별 지면 투영 → 3뷰 근거 통합 → shared family다.
영상 BEV는 별도 소비 경로다. 합성 BEV에 구독자가 없으면 뷰별 BEV 입력을 해제하고,
뷰별 출력도 수요가 없으면 native semantic 구독을 해제한다. PyTorch 복구 프로필도
`lazy_outputs=true`로 사용하지 않는 native 복원을 생략한다. Recorder 구독은 GUI와
동일한 정상 수요이며 기존 토픽·영상 좌표는 유지한다.

수요 해제·STANDBY·세대/cutoff 전환에서 동기화 그룹을 폐기하고 새 큐로 재구독한다.
이미 연결 해제된 ROS callback도 이전 그룹 epoch로는 결과를 전달하지 못한다.
같은 generation 값으로 재시작한 경우에도 성공 batch counter가 감소하면 로컬 epoch를
갱신한다. Family tracker와 대기 작업은 이전 epoch를 재사용하지 않는다.

독립 spline은 내부 `enable_independent_boundary_pipeline`을 명시한 경우에만 실행한다.
Marker/audit 구독만으로 활성화하지 않는다. 일반 `view.launch models:=lane`에는 실제
`LaneBoundaryFamily`를 그리는 family renderer가 포함된다. 같은 결과의 observed/inferred/
invalid 상태를 구분하며 다른 피팅을 생성하지 않는다. 출력은 기존
`/perception/lane/debug/shared_family_bev/compressed`다.

## 모드와 readiness

판제가 일반 구간 STANDBY → 음영 진입 전 PREPARE → 준비 확인 후 ACTIVE → 이탈 후
STANDBY를 요청한다. 위치 판단·HD맵 정책·제어는 카메라가 수행하지 않는다.

- STANDBY: 가중치 유지, 추론/피팅 중단, 큐 정리와 이전 family 무효화.
- PREPARE: 실제 새 3뷰 입력과 family 유효성까지 확인한다.
- ACTIVE: 새 결과만 공급하며 freshness350ms를 유지한다.

Family 메시지에는 generation 필드가 없으므로 기존 필드와 MD5를 유지하면서, 추론
생산자가 이번 세대에 완료한 stamp의 제한된 목록(최대256개)에 있는 피드백만 받는다.
이전 결과·알 수 없는 source·cutoff 이전 결과로 readiness를 만들지 않는다.
상태 서비스의 새 이름과 구형 alias는 같은 상태를 사용한다.

PR #85를 빌드한 `asmc-ros-noetic` 환경에서 실행한다. 기존 생산자를 종료한 뒤 사용한다.

```bash
roslaunch asmc_camera camera.launch
rosservice call /perception/camera/lane/set_mode "mode: 1
target_hz: 15.0"
rostopic echo /perception/camera/lane/status
# 준비 상태를 확인한 뒤 음영구간에서 ACTIVE
rosservice call /perception/camera/lane/set_mode "mode: 2
target_hz: 15.0"
# 구간 이탈
rosservice call /perception/camera/lane/set_mode "mode: 0
target_hz: 15.0"
```

시각화와 rqt는 별도이며 각각 한 세트만 실행한다.

```bash
roslaunch asmc_camera view.launch models:=lane
rosrun rqt_image_view rqt_image_view --clear-config /perception/lane/debug/shared_family_bev/compressed
```

TensorRT 후보 선택은 기존 private `_runtime.launch`의 `lane_runtime_config`로 한다.
복구는 후보를 종료하고 `camera.launch`를 다시 시작한다. 일반 공개 옵션은 늘리지 않았다.

## GC 진단과 이전 실패 해석

이전533ms generation2 GC 검사, TwinLite590ms와 family644ms 공백은 미해결 증거다.
GC를 끄거나 임계값·freshness를 변경하지 않는다. Family 피드백에는 scalar만 보존한다.
기존 lifecycle trace에 피드백 잠금 대기 시간을 추가했다.

`camera_runtime_probe.py --allocation-profile`은 별도 진단 실행에서 tracemalloc,
역직렬화/피드백 wall·thread CPU 시간, GC count/statistics와 종료 시 retained allocation/
tracked type 요약을 남긴다. 이 실행은 계측 비용 때문에 수락 지연 측정에 사용하지 않는다.
일반 probe는 기존 generation2 GC와 실제 입력 callback 계측을 유지한다.

이전 FOGGY r3의 `failure-lane.jpg`는 터널 외부의 다수 황색선 장면임을 영상으로 확인했다.
정확한 route station은 기록되어 있지 않아 unknown이다. 이 장면의 family 근거 부족은
음영 ODD 내부 품질 실패와 구분한다. 원래 실패 수치와 이미지는 삭제하지 않으며,
GC/공급 공백은 ODD 구분으로 면제하지 않는다.

## 고정 RGB·pose와 검증 상태

`prepare_shadow_family_fixture.py`가 기존 20260912 shadow-static bag 6개를 고정했다.
SUNNY/FOGGY×11/13/15시의 **2,356개 3뷰 묶음(7,068개 RGB 참조)**이다.
원본 JPEG를 복제하지 않고 bag/영상 SHA, header·fallback 정책, 동기화와 보간 pose를 기록한다.
연속 영상이며 독립 장면 수나 새 GT로 설명하지 않는다. 실제 촬영 시각의 동기 정확도를
보증하지 않는다. 구간은 기존 주행 시험의 station 경계로 구분한 개발 fixture다.

| 조건 | 3뷰 묶음 | 입구 전 | 음영 경계 내부 | 출구 후 |
|---|---:|---:|---:|---:|
| SUNNY11 |485|125|282|78|
| SUNNY13 |427|119|227|81|
| SUNNY15 |370|92|201|77|
| FOGGY11 |346|93|165|88|
| FOGGY13 |358|111|194|53|
| FOGGY15 |370|108|205|57|

Manifest SHA: `4b5cf3062216972a027682325ecc856150b77e184541d2f74fb142a364ced164`.
로컬 원본은 `data/runtime/camera-shadow-family-v1/fixture-r1/`, 작은 요약은
`config/baselines/evidence/camera_shadow_family_v1/`에 보존한다.

기존 `validate_twinlite_tensorrt.py --family-fixture`는 동일 JPEG와 production vectorizer를
사용해 PyTorch/TensorRT의 validity·role·support·기하 차이를 기록한다. 전용 로컬 ROS master
(11311 이외 포트)에서만 실행하며 공개 제어를 발행하지 않는다. 이 결과로 live readiness나
GT 정확도를 대신하지 않는다. 차이는 갤러리로 검토하고 자동 승격하지 않는다.

첫 시도는 Docker Desktop/WSL 통합과 MORAI 미실행으로 보류됐으며, 재실행 후 GPU parity와
음영구간 반복 시험을 진행한다. CPU 검사와 과거1,404장 품질 통과를 새 전체 수락으로 표시하지 않는다.
최종 수락은 모델 p95≤150ms·p99≤200ms, 최대공백≤250ms, YOLO/TwinLite 및 유효 family의
완전5초창≥10Hz, ResNet≥9.5Hz, VRAM≤90%, 정상 음영 ACTIVE readiness 저하0을 요구한다.
모드 준비·전환 구간은 별도 집계하고, 15Hz 달성도 최소 공급과 별도로 표시한다.

실제 Noetic launch 검사에서 옛 외부 checkpoint 인자가 component YAML보다 우선하던
문제를 확인했다. 기본 파일을 패키지 내부로 바꾸고 component config가 있을 때는 해당
YAML이 checkpoint를 소유하도록 수정했다. 최초 실패 로그와 수정 후77개 통과를 보존한다.
