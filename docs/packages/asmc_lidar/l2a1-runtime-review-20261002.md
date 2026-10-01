# L2-A1: 독립 LiDAR 실행 기준선과 Patchwork++ 어댑터 검증

상태: **격리 검증 완료, 운영 미채택**. 검토 브랜치 `feat/asmc-lidar-runtime`은
기존 `/last_lidar`, `/lidar_nogps`, `/perception/objects`, 판제 코드, 메시지 MD5,
센서 설정을 바꾸지 않는다. 이 보고서는 패키지를 실행·비교할 수 있게 된 결과이지
Patchwork++의 장애물 검출 정확도나 전 미션 주행 성능을 승인한 기록이 아니다.

## 입력과 재현 경계

- 기준 커밋 `00e48a7`, 손재호 고정 소스 `6435af1`, Patchwork++ 공식 C++
  checkout `3e6903a1`. 실제 복사한 코어·센서 설정·입력 SHA는
  [manifest](../../../config/baselines/lidar_l2a1_runtime_20261002.yaml)와
  `python3 -B tools/lidar/l2a1/check_manifest.py --repo-root . --data-root "$ASMC_DATA"`
  로 확인한다. 12개 SHA 검사 통과.
- Capture `inputs_v4/snapshots.jsonl`의 105장면, `missing`/`valid_empty` 각
  105개 측정 조건과 조건별 정차 warmup 5회, 총 1,050 replay row를 사용했다.
  측정 210개는 서로 독립된 210개 상황이 아니다.
- 검증의 기준선은 당시 고정된 `l1b5a_capture_final_v1/run/geometry.jsonl`.
  점군·자세·카메라 조건을 같은 snapshot에서 재생한다. 당시 live 내부 상태
  전체 또는 연속 주행 추적을 복원했다고 주장하지 않는다.
- 기본 터널 profile의 기존 ROI, 10 cm voxel, 지면·천장·벽 제거, 1.0 m
  군집, L-shape OBB 설정을 고정했다. 일반도로 profile은 소스상 1.5 m
  Euclidean 경로를 별도 구현했지만 고정 일반도로 출력 fixture가 없어
  **수치 parity는 아직 미검증**이다.

## 기준선 재현과 좌표·시간

Noetic 컨테이너에서 `asmc_lidar` Release 빌드, 기존·확장 core test, 고정 Capture
재생, `verify.py`가 통과했다. 공유 B1 C++ 시험 1개와 기존 LiDAR Python
회귀 63개도 통과했다. Python 시험 중 Git worktree 경로 검사 1개는 컨테이너에
호스트 `.git` metadata를 마운트하지 않아 재실행이 필요했고 호스트에서 통과했다. 측정 210개 모두 다음 항목이 일치했다.

| 항목 | 결과 |
|---|---:|
| voxel 개수·frozen FNV hash | 210/210 |
| 원본 점→voxel 계보 | 210/210 |
| 제거 label | 210/210 |
| 살아남은 OBB 개수·순서 | 210/210 |
| `base_link` OBB 중심·8개 코너 | 210/210, 최대 오차 각각 0.00005275 m / 0.00005885 m |

원본 관측 stamp는 ns 정수로 유지한다. 수신 시각·monotonic 처리시간과
구분하며 bridge 수신 stamp를 광선별 측정시각으로 승격하지 않는다. 공개
시각화 점군과 OBB 코너는 장착 변환과 실제 적용한 수평화 역변환 후 rear-axle
`base_link`로 보낸다. 지도 pose가 없으므로 map 객체를 만들지 않는다.
객체는 프레임별 검출일 뿐 track ID·속도가 없다. 3D 코너가 OBB의 정본이며,
원본의 yaw·지면 상대 높이 같은 부가 필드는 공개 판제 계약이 아니다.

기존 B1 변환 시험 및 새 nonzero roll/pitch/장착 회전·이동, intensity·원본
점 대응, 중복·역행·영점 stamp, IMU 누락, 비유한 점·경사면·낮은 객체·수직면
합성 검사를 수행했다. 합성·정차 입력의 성공을 실제 검출 정확도로 확대하지 않는다.

## Patchwork++ 독립 어댑터

Patchwork++는 원본 유한 ROI 점을 센서 원점 기준으로 수평화해 입력한다.
`sensor_height=1.80 m`, 1–60 m, RNR OFF는 이전 평가와 연결한 **초기
가설값**이며 Autonav v3 장착 z=1.45 m와 다르다. 기준선의 벽 필터에
Patchwork++ 분류를 주입하지 않았다. 기존 필터는 단일 지면 평면 높이를
다시 사용하므로 결합은 별도 설계·회귀가 필요하다.

| 측정 210조건의 점 계보 | 점 수 |
|---|---:|
| raw 전체 | 4,032,000 |
| 기준 ROI 안 | 2,681,146 |
| Patchwork++ ground | 734,678 |
| Patchwork++ nonground | 1,946,468 |
| ROI 안 미분류 | **0** |
| ROI 밖 (분류 대상 아님) | 1,350,854 |

원본 인덱스 범위·중복과 ROI coverage 검사는 측정 210/210에서 통과했다.
`nonground`는 곧 장애물이 아니며 ground도 HD맵 주행가능영역이 아니다.
정답 geometry가 없는 점을 분류 결과와 대조하여 정확도라고 표현하지 않는다.
debug/Release 두 빌드의 1,050 row에서 처리시간을 제외한 출력이 동일했다.

같은 210 측정조건에서 Release 빌드의 단계별 CPU 시간(ms)은 다음과 같다.
컨테이너 내 오프라인 단독 실행 계측이며 카메라·bridge·판제 동시 실행
지연이나 차량의 end-to-end latency가 아니다.

| 단계 | p50 | p95 | p99 |
|---|---:|---:|---:|
| 원본 ROI·voxel 전처리 | 2.9814 | 3.6860 | 4.1697 |
| 기존 기준선 필터·OBB | 1.8343 | 5.3600 | 5.8862 |
| Patchwork++ 입력 복사 | 0.0611 | 0.1041 | 0.1416 |
| Patchwork++ 지면 분할 | 1.0735 | 1.4552 | 2.0912 |
| 원본 인덱스·좌표 출력 복원 | 0.2687 | 0.3787 | 0.4678 |

최적화 없는 빌드에서 더 큰 수치가 관측됐고, 이는 Release 결과와 섞어
실시간 성능을 판단하지 않는다. 어댑터 분할 속도만으로 Patchwork++를
운영 경로에 넣거나 기존 지면 제거를 대체할 수 없다.

## ROS 재생과 미검증 항목

별도 ROS master에서 정차 bag `stationary_v1/stationary_0.bag`의
`/imu`, `/velodyne_points`만 8초 재생했다. `/lidar_candidate/status`에서
시작 전 `sensor_missing` 1건 뒤 `valid` 107건이 수신됐고,
`/lidar_candidate/input_base` header는 원본 stamp와 `base_link`였다.
`reference`와 `ground_review` 모두 별도 master에서 8초 재생 시
`valid` 107건과 `base_link` 점군 header를 확인했다. 기존 운영 노드는 실행하지 않았다. 이 bag은 고정 정차 검사이며 연속 주행 수락이 아니다.

실시간 review의 카메라는 기본 **미수신**이며 YOLO를 자동 실행하지 않는다.
고정 재생에서는 `valid_empty`를 따로 시험했지만 양성 box·미지원 class를
새 패키지의 실시간 보호 경로에 연결하지 않았다. GPS 전환, tracker,
map 변환, detector 성능, GPU/CPU full-stack 부하, 일반도로 parity도 미검증이다.
시뮬레이터 gRPC 주행이나 장애물 새 배치는 이번 A1 기준선 재현에 필요한
모호성이 남지 않아 수행하지 않았다. 물체 배치·연속 주행은 A2 이후
정답/동기 계약을 확인하고 진행한다.

## 다음 gate

L2-A2에서는 동일 원본 ROI, 동일 정답 geometry와 낮은 장애물·보행자·차량
장면에서 기존 지면 분할과 Patchwork++를 비교한다. 기존 voxel과 원본 점
표현 차이를 별도 대조군으로 분리하고, 지면 보존·객체 보존·처리시간을 함께
본다. 일반도로 fixture와 양성 YOLO 보호의 입력 계약도 확보한다.
그 결과를 검토하기 전에는 Patchwork++를 터널 벽 필터나 판제 입력에
연결하지 않는다. 실패 시 review 패키지만 비활성화하면 기존 토픽은 유지된다.
