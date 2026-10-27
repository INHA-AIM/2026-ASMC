# 전방 fusion 연결 검증

F1은 native 전방 matching의 실제 선택을 추적한다. 좌우는 수신·보정·원본 픽셀 투영까지만
구현돼 있다. 공개 객체/관측/추적 메시지와 mode 서비스는 변경하지 않는다.
[실측·수정 전 실패와 남은 제한](../../../docs/records/lidar/2026-10-27-front-fusion-verification.md)을 먼저 확인한다.

## 무엇을 확인하는가

- `diagnostics`는 debug OFF에서도 1Hz로 갱신한다. 뷰별 수신 사유·후보 수·header 시간차와
  최근100scan의 matching/drop 사유를 제공한다. 뷰 요약의 최근 후보와 **실제 선택 관측은 다르다**.
- 기존 `trace`는 구독자가 있을 때 전체 scan에 선택적인 `fusion` 블록을 직렬화한다.
  `emitting_scan_sequence/stamp`는 현재 호출, `decisions[].scan_sequence/stamp`는 실제 대기 scan이다.
  바깥 session/generation과 함께 읽는다. 과거 scan을 처리해도 현재 IMU로 투영하지 않는다.
- `decisions`는 호출당64개, 관측별 association은128개다. 제한 초과는 `*_omitted`로 집계한다.
  runtime에는 최근100scan의 작은 사유 집계만 유지한다. 전체 기록은 bag/파일에 저장한다.
- association에는 원본 bbox 배열 index·실제 IoU·신규 근거 반영 여부·이전/이후 분류가 있다.
  IoU는 기존 cluster bbox의 가로1.3/세로1.5 확대 후 값이다. 갤러리의 초록색 확대 전 투영 bbox와
  수치가 직접 같다고 해석하지 않는다. IoU0.25, 초기확인5회/변경3회/미스유예3회는 유지한다.
- `matched`는 한 scan과 영상이 시간/기하 검사를 통과했다는 뜻이다. 모든 track이 YOLO에
  연결됐다는 뜻이 아니며 `new_observation`, `iou_below_threshold`, `not_projected`를 함께 본다.
- HD맵 보호는 기존 fresh 양성 vehicle/dynamic 근거 검사로 결정한다. 이전 분류 유지와
  LiDAR 운동 상태를 새 YOLO 확인으로 승격하지 않는다.

## 직접 확인

브릿지를 실행한 뒤 각각 별도 터미널에서 실행한다. 동일 생산자를 중복 실행하지 않는다.

```bash
roslaunch asmc_camera camera.launch
roslaunch asmc_lidar lidar.launch
roslaunch asmc_lidar view.launch
```

웹 기본 상태에 전방 matching과 좌우 수신이 구분돼 나온다. 객체별 association과 원본 단계
상세가 필요하면 LiDAR를 실행할 때만 `debug:=true`를 추가한다. 웹 snapshot은 기존 주기로
표본화되므로 선택 객체의 연결이 해당 snapshot에 없으면 그대로 표시한다. 모든 사건은 trace로 본다.
판제 실행은 debug를 요구하지 않는다. 병행 시험은 LiDAR와 view 모두 `namespace:=/migration/lidar`로
격리한다. 네트워크·센서셋 선택은 [현재 MORAI 설정](../../../config/morai/README.md)을 따른다.

## 오프라인 재현

[manifest](../../../config/baselines/camera_lidar_f1_v1.json)는 F1의 원본 입력·설정 SHA와
run ID를 고정한 기록이다. 새 분석은 새 run을 등록하고 복사한 manifest의 run ID를 그 ID로 지정한다.
대용량 출력은 등록된 저장소 경로에만 쓴다. 기존 output 폴더가 있으면 도구가 거부한다.

```bash
source devel/setup.bash
cmake -S tools/perception/frame_time -B /tmp/asmc-c0-probes \
  -DASMC_C0_RUNTIME_LIB_DIR="$PWD/devel/lib" -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/asmc-c0-probes -j2
C0_PROBES=/tmp/asmc-c0-probes python3 tools/perception/test_frame_time_audit.py
python3 tools/perception/test_camera_lidar_fusion_audit.py
python3 tools/perception/audit_camera_lidar_fusion.py \
  --manifest '<registered-manifest.json>' --run-id '<registered-run-id>' \
  --bag '<simultaneous-input.bag>' --output '/artifacts/<registered-run-id>/analysis' \
  --probes /tmp/asmc-c0-probes --replay "$PWD/devel/lib/asmc_lidar/lidar_replay"
```

`--replay`는 진단 OFF/ON 각3회를 교차 실행한다. 동일 객체 결과 SHA·처리 p95·독립 프로세스
peak RSS를 기록한다. 녹화 도착 순서의 논리적 replay이므로 실제 worker 대기/ROS 공급률 수락과 다르다.
`summary.json`, `decisions.jsonl`, 디스크 SQLite 대응표, 정확한 RGB의 `index.html`을 생성한다.
원본 RGB는 topic/header stamp/sequence가 정확히 하나 대응할 때만 사용한다. 다른 시각 영상으로
대체하지 않는다. side 갤러리는 같은 과거 scan의 이미 변환된 `base_link` 8코너를 native
`camera::project`로 투영한다. 카메라 시간차는 표기하며 ego motion을 보정했다고 주장하지 않는다.

`/morai_sensor_bridge/frame_timing`은 센서 topic/frame/header/sequence로 대응한다. bridge session과
clock도 검사한다. camera `morai_sensor`와 LiDAR `velodyne_device_hour`는 직접 빼지 않는다.
header ±50ms는 receive 시각의 연결 기준이며 촬영 시각 동기화 증명이 아니다.

`run_camera_lidar_fusion_trial.py --help`는 명시적인 simulator 시험 도구다. 기본 운영 실행에
연결되지 않는다. 사용자 승인·새 run·충분한 공간·단일 생산자·무제어 송신을 확인하고 `--execute`로
실행한다. 시험 소유 `/migration/lidar`만 reset하고 자기 actor만 제거한다. 실제 차량용 도구가 아니다.

## 재호의 다음 작업

1. 현재 fixture에서 시간 선택·분류 확인 횟수·보호 근거가 유지되는지 먼저 확인한다.
2. 좌우 matching은 뷰별 관측 ID와 보정을 사용해 비차단으로 확장한다. 하나의 관측을 여러 확인 횟수로 세지 않는다.
3. 여러 객체/큰 가드레일 cluster의 오매칭은 실제 갤러리로 판단한다. 좌표 결함과 IoU 알고리즘 한계를 구분한다.
4. 실제 시각 정렬은 검증된 clock 대응 또는 상대 운동 입력을 확보한 뒤 별도 단위로 한다.
