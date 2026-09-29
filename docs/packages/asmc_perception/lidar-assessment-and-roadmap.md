# LiDAR 인지 패키지 완성도 평가와 개선 로드맵

> **문서 상태:** Assessment Record + Working Proposal
> **최종 정리:** 2026-09-29 (L1-B4-B 실제 Capture GT 비교, 군집화 후속 단위 선정, 운영 미연결)
> **대상:** LiDAR 인지 담당 · 판단제어 파트 · 인지 통합
> **대상 코드:** [`src/lidar+sensorfusion/`](../../../src/lidar+sensorfusion/)
> **읽는 순서:** 0장은 2026-09-26~29의 판단과 착수 순서다. 1~13장은 2026-09-21의
> main 계열 소스 분석·문헌 검토 기록을 보존하며, 후속 브랜치에 그대로 적용하지 않는다.
> 실행 정책이 달라진 과거 제안에는 정정 표시를 붙였다.

이 문서는 LiDAR 패키지의 현재 완성도를 단계별로 평가하고, 판단제어 파트에서 보완로직으로
메우고 있는 증상들의 코드상 원인을 특정한 뒤, 개선안을 학습 필요 여부로 나누어 정리한다.

인지 전체 방향은 [카메라·MGeo·LiDAR 융합 로드맵](./roadmap.md)이 정본이다.
이 문서는 그 로드맵의 LiDAR 항목을 상세화한 Working Proposal이다. 현재 판제 공통
소비 의미는 [L1-A 계약](../../contracts/perception/lidar-planning-consumer-v1.md)에
설계 기준으로 고정했다. 운영 토픽/메시지 변경이 승인된 것은 아니다.
판제가 소비하는 메시지 계약을 바꾸는 항목은 [`src/interfaces/`](../../../src/interfaces/README.md)에서
합의한 뒤에만 승격한다. 이 문서의 분석·후보 선택은 판제 패키지 수정이나 팀원 브랜치의
병합을 의미하지 않는다.

> **2026-09-25 진행 순서 메모:** P5-E 인지 운영 closeout 뒤 LAN2LAN 리허설보다 이 문서의
> LiDAR/fusion 기준선 계측과 담당자 공동 개선을 먼저 진행한다. 우선순위는 (1) 센서 frame과
> latency 기준선, (2) 지면 제거, (3) 음영구간 터널 벽·천장 제거, (4) 거리 적응형 clustering과
> tracking 안정화, (5) HD Map polygon을 이용한 고주로 주행 차로 귀속이다. 기존 nested
> workspace와 메시지는 담당자 handoff 없이 이동·삭제하지 않는다. 아래 분석과 외부 연구
> 구현은 후보를 고르는 근거이며, 현재 코드를 곧바로 교체하라는 결정은 아니다.

이 메모의 공동 개발 방식은 2026-09-29 사용자 결정으로 갱신됐다. 손재호와 개발 시간이
맞지 않으므로 각자 독립 브랜치에서 개선하고 고정 입력·revision으로 비교해 선택한다.
팀원 변경이나 승인을 기다리는 것을 격리 실험의 선행조건으로 두지 않으며, 운영 인계와
판제 계약 변경은 여전히 별도 검토한다. 이전 메모는 당시 결정 이력으로 보존한다.

로컬 조사 checkout은 팀 repository 밖의
`../external/baselines/`에 있으며 `patchwork-plusplus(-ros)`,
`linefit_ground_segmentation`, `depth_clustering`, `SimpleTrack`, `AB3DMOT`, `FastPillars`,
`CenterPoint`, `OpenPCDet`, `Lidar_AI_Solution` 등이 준비돼 있다. 이 디렉터리는 읽기·benchmark
참고용이지 현재 build dependency가 아니다. 코드를 채택할 때는 license, upstream revision,
VLP-16/15 Hz 실측과 우리 좌표·message adapter를 먼저 별도 manifest로 고정한다.

---

## 0. 현재 기준선과 개선 착수 순서 — 2026-09-26

### 0.1 기준 commit, 담당 경계, 개발 환경

P5-E는 PR #56으로 `main`의 `1ce434d`에 반영됐다. 다음 개발은 LiDAR/fusion 개선이며,
LAN2LAN 리허설은 패키지·인지 계약과 기능 검증 뒤에 진행한다. HD맵 geometry와 카메라
차선·주행가능영역은 이미 마련된 입력으로 사용하고, LiDAR는 장애물 검출·추적 품질에 집중한다.

| 비교 대상 | 고정 revision / 작성자 | 이번 판단에서의 역할 |
|---|---|---|
| 팀 main | `1ce434d` | P5 완료 후 공통 기준선 |
| `feature/lidar-structural-surface-filter` | `6435af1`, `sonshiny`(손재호) | 현재 터널 구조물 제거·`/lidar_nogps` 인계 후보 |
| 같은 브랜치의 선행 commit | `77c6c2c`, `sonshiny` | 순수 PCL 코어·ROS 연결·합성 테스트·bag 평가기 추가 |
| `feature/lidar-center-drivable-filter` | `003edf1`, `sonshiny` | 이전 도로 overlap 필터 작업 이력 |
| `feature/unknown-only-drivable-boundary` | `8b3f0b1`, `ysj20030825-lab`(양서준) | 별도 판제 변경; 손재호 브랜치와 메시지 차이 확인 필요 |

손재호의 두 commit은 `1ce434d`에서 직접 분기했다. 코드 검토는 이 revision으로 고정하며
브랜치 최신화 여부를 실험 기록에 남긴다. 음영구간 판단제어 담당은 사용자 설명상 이강태다.
서준의 공개 브랜치를 강태의 작업으로 간주하거나, 아직 push되지 않은 작업까지 확인했다고
해석하지 않는다. 팀원 작업을 분석용 checkout으로 읽고, 개선 후보는 별도 실험에서 비교한다.

L1-B4-A는 분석 완료점 `c536c3c`에서 `feat/lidar-l1b4-geometry`로 독립 분기했다.
손재호 `6435af1`은 읽기 전용 비교 기준이고 향후 변경은 다른 revision의 실험으로 기록한다.
후보는 `tools/lidar/tunnel_geometry/`에만 있으며 운영 nested workspace를 수정하지 않는다.

서준 브랜치는 `Msg/TrackedObject.msg`와 `asmc_msgs/DetectedObject.msg`에
`motion_state`와 enum 상수를 추가한다. ROS1 MD5가 달라지므로 소스 일부만 가져오거나
다른 브랜치의 `devel`을 overlay해 연결하면 통신 호환성이 깨질 수 있다. main/서준의
현재 MD5와 실제 consumer 요구는 [L1-A 계약](../../contracts/perception/lidar-planning-consumer-v1.md)에
고정했다. 후속 구현 전 producer/consumer의 새 버전·호환 adapter를 합의한다.

개발 실험은 사용자가 제공한 **26r1**을 기본으로 한다. gRPC, 객체/instance GT와 제어 가능한
시나리오는 재현·채점에 유용하다. GT는 평가기와 개발용 제어에만 사용하고, 인지 runtime의
ROI·검출·추적 입력에 섞지 않는다. 최종 수락에는 참가팀 배포 **MolitComp에서 GT 없이**
동일 센서·토픽 계약으로 실행되는 검증을 포함한다. 26r1에서 성공했다는 이유만으로 그 검증을
대체하지 않는다.

현재 장비는 i5-14600K(20 logical CPUs), 컨테이너 가용 메모리 약 15 GiB,
RTX 4060 Ti 8 GiB다. 현재 로컬 실험에서는 MORAI도 같은 GPU 자원을 공유한다.
5장의 VRAM 분할·25 ms 목표는 당시의 예산 제안이며 측정 결과가 아니다. CPU/GPU 사용량,
입력 cloud 수 대비 처리 수, capture-to-output 지연을 camera/HD맵과 동시 실행하며 측정한다.
센서 설정 목표는 15 Hz이고, 손재호의 확인에 따라 완성 cloud 약 12 Hz 입력도 허용한다.
출력을 억지로 15 Hz로 복제하거나 오래된 관측의 stamp를 갱신하지 않는다.

### 0.2 손재호 인계 메시지와 실제 코드의 일치 여부

인계한 `Main main.launch`에서 `start_velodyne_driver:=false`,
`enable_sensor_fusion:=true`, `enable_tunnel_experimental_filter:=true`,
`tunnel_dynamic_roi_enable:=true`, `tunnel_wall_search_min_abs_y_m:=1.75`,
`tunnel_wall_remove_distance_m:=0.25`를 사용하면 의도한 새 경로가 실행된다.

| 출력 | GPS 정상 | GPS 음영 | 실제 의미 |
|---|---|---|---|
| `/last_lidar` | 도로 overlap 필터를 통과한 객체 | 빈 배열 | `Msg/TrackedObjectArray`, `map` 좌표 |
| `/lidar_nogps` | 빈 배열 | 새 터널 필터와 기존 tracker를 통과한 객체 | 같은 message, 헤더 `velodyne`; `base_link` 아님 |
| `/lidar/tracked_objects_map` | 전체 객체 | **여전히 map 객체 발행** | GPS 유효성 gate 없는 진단/기존 경로; 음영구간 정본으로 사용 불가 |
| `/lidar/tunnel/objects` | 비활성/clear | 필터 단계의 객체 | 프레임별 진단 결과; 최종 persistent track 인계 토픽 아님 |

새 코드는 `ROI+Voxel → roll/pitch 수평화 → 천장 컷 → 지면 후보/재적합/시간 재사용 →
좌우 벽 segment 검증 → 1.0 m clustering → OBB 필터`의 생존 cluster를 기존 tracker에
넣는다. 따라서 단순 시각화 실험이 아니라 실제 `/lidar_nogps` 출력까지 연결돼 있다.
GPS 정상에서는 기존 pitch-only 보정·0.3 m 지면 RANSAC·1.5 m clustering 경로가 유지된다.

긍정적인 변경은 코어와 ROS의 분리, 지면 높이·기울기 검사와 refit, 벽의 독립적인 좌우
추정·지속성·ego-motion 보정, intensity 보존, 보호 객체 검사, 단계별 diagnostics다.
이 브랜치는 로컬 컨테이너 빌드와 **27개 합성 테스트**를 통과했다. catkin 집계의 54는
중첩된 JUnit 결과를 중복 집계한 값으로, 독립 테스트 수는 27개다. 실주행 정확도나
손재호가 보고한 **추가 17 ms**는 이 합성 테스트로 입증되지 않으며 별도 실험 지표로 관리한다.

### 0.3 먼저 해결할 계약·구조 문제

아래 위치는 모두 `6435af1`의 `src/lidar+sensorfusion/src/Main/` 기준이다.
발생 빈도는 실험으로 판단하되, 코드상 좌표·상태 의미의 차이는 먼저 동결한다.

| 우선 문제 | 소스 근거 | 개선 시 지켜야 할 조건 |
|---|---|---|
| 수평화된 좌표를 raw sensor frame으로 표기 | `TunnelFilterCore.hpp:10`, `TunnelFilterCore.cpp:502`, `PlannerObjectRosPublisher.cpp:44` | 실제 처리 frame을 명시하고 inverse-leveling과 sensor→base extrinsic을 한 번만 적용 |
| 새 roll 보정과 map adapter 불일치 | `PlannerObjectAdapter.cpp:57`은 pitch만 역보정 | roll/pitch가 모두 있는 synthetic roundtrip으로 정상·터널 경로 검증 |
| mount 높이와 실제 노면 높이를 혼동 | `TunnelFilterCore.hpp:79`의 예상 지면 `-1.45`; `setting.yaml:14`의 mount `+1.45` | rear-axle 원점의 노면 상대 높이를 포함해 ground prior·fallback·height cut을 검증 |
| 무효 GPS가 map 출력과 로컬 객체 생성에 남음 | `PlannerObjectRosPublisher.cpp:449`는 GPS gate보다 먼저 발행; `PlannerObjectAdapter.cpp:211`은 map 유한성 실패 시 객체 제외 | 로컬 geometry를 map 변환 실패와 분리; map projection은 fresh pose가 있을 때만 허용 |
| GPS dropout/단발 불량에서 mode 전환 불안정 | `Parser.cpp:278`은 status/zero만 검사, latch는 invalid 3회 뒤 설정 | age·finite·range 검사; 진입·복구·미수신 정책과 상태를 명시 |
| 정상↔음영에서 ID와 운동 이력 초기화 | `LidarProcess.cpp:384`에서 tracker queue clear | 모드 전환을 메시지 routing과 내부 tracker 좌표 변경으로 혼동하지 않음 |
| 시간 정렬과 ego-motion 정보 부족 | `main.cpp:358`의 latest buffer 복사, `DeadReckoning.cpp:4`의 단순 memcpy | cloud 관측 시각의 pose/IMU/속도, 최대 age와 gap을 함께 기록 |
| GPS 음영인데 motion classifier가 ENU 이동량 사용 | `MotionClassifier.cpp:358` 및 `FrameFilter.cpp:46` | map evidence 유효성을 별도 gate하고 local 상대/절대속도의 의미를 명시 |
| fusion ON/OFF가 위치 자체를 바꿈 | `FusionLiDAR.cpp:127`의 CoreX/Y/Z AABB 덮어쓰기 | 클래스 실험에서 기하/추적 추정치를 동일하게 유지 |

현재 LiDAR mount는 rear-axle 기준 `(0.80, 0, 1.45)m`이다(`setting.yaml:10`).
`/lidar_nogps`를 `base_link`라고 이름만 바꾸면 0.8 m 종방향 offset이 빠진다.
기존 출력에는 roll/pitch 수평화까지 들어 있으므로 단순 static TF만 적용하는 것으로도
충분하지 않다. 이 차이를 확인한 상태에서 legacy output은 호환 adapter로 보존한다.

**장착 높이 1.45 m는 센서와 노면 사이 높이가 아니다.** `base_link`는 rear-axle 원점이며
노면 위에 있다. 기존 카메라 보정 파일
[`cameras_v2.yaml`](../../../src/asmc_perception/config/bev/cameras_v2.yaml)은 당시 LiDAR fit의
base-link 노면 높이 `-0.354 m`/`-0.346 m`를 기록하고, v2/v3 카메라 설정은 `-0.35 m`를
명시적으로 사용한다. 수평·무회전 근사에서는 `z_ground_sensor = z_ground_base - 1.45`이므로
약 `-1.80 m`가 되며, `-1.45 m`를 바로 지면으로 놓는 가정과 다르다.

L1-A의 예비 offline 첫 프레임에서 터널 기준 지면은 약 `-1.8376 m`, 기존 RANSAC fit은
약 `-1.8403 m`였다. 이는 한 프레임의 관측이며 전 구간의 일정 offset이나 slope를 확정하는
값이 아니다. 동시각 자세·노면 위치·차량 높이와 여러 장면의 plane 분포를 보고 prior를 정한다.
이 프레임의 평면 원점 높이를 예로 들면 다음 차이가 생긴다.

| 현재 상수/정책 | 예비 지면 높이 기준 의미 | 확인할 실패 모드 |
|---|---|---|
| 첫 유효 fit 전 fallback `-1.45+0.20=-1.25 m` 이하 제거 | 노면에서 약 `0.59 m` 높이까지 제거 | 낮은 장애물 점을 지면으로 버릴 가능성 |
| `CoreZ<=-1.3` ground-noise 분류 | centroid가 노면 위 약 `0.54 m` 이하여도 noise 후보 | 기존 문서의 약15cm 기준 설명보다 넓은 영향; 낮은 물체/차량 밑단 검증 |
| ground 후보/원점 높이 하한 `-1.90 m` | 예비 평면보다 아래 여유가 약 `6 cm` | 경사·자세·차량 높이 변화로 support가 잘리거나 fit 거부 |
| 선제 천장 cut `z_sensor>2.55 m` | 노면 기준 약 `4.39 m` | sensor-relative 컷과 road-relative OBB 필터의 서로 다른 높이 의미 |

fallback은 **이전에 유효 fit이 없을 때** 위 예상 높이를 사용하며, 유효 이력이 있으면 마지막
높이를 재사용한다(`TunnelFilterCore.cpp:784`). 성공한 fit을 매번 `-1.45 m`로 덮어쓰는
코드는 아니다. 천장 컷도 측정 offset만큼 일괄 낮추지 않는다. 노면 상대 높이·천장 구조·
대형 차량 보존을 함께 확인한다. 이 높이 계약을 후속 낮은 장애물/경사/높은 차량 회귀에 넣는다.

`DeadReckoning.cpp`라는 파일명이 강건한 no-GPS odometry 구현을 뜻하지 않는다.
**연속 `odom` 좌표계에서 추적하고 fresh pose로 map에 투영하는 구조는 제안**이며,
음영구간에서 사용할 wheel/CAN 속도·IMU yaw의 가용성, 지연, drift, simulator release별
제약을 먼저 확인해야 한다. 해당 근거가 부족하면 ego-motion을 보상한 local tracking과
품질 상태를 우선 제공하고, 검증되지 않은 절대속도를 만들어내지 않는다.

### 0.4 벽과 장애물이 함께 남는 이유와 ROI 설계

손재호의 `dynamic_roi`는 GPS 모드에 따라 `x[-30,40], y[-10,10], z[-2,3]` 박스를
추가 적용하는 기능이다. HD맵·카메라 인지 결과로 경계를 만드는 기능은 아직 없다.
일반도로의 후단 필터도 `main.launch:88`의 원태 구 bake를 읽으므로, 완성된 operational
HD맵 polygon과 동일한 영역을 사용한다고 해석하면 안 된다.

벽 잔여점에는 다음과 같은 구조적 상호작용이 있다.

1. `TunnelFilterRos.cpp:203`은 YOLO 박스를 10%+8px 확대하고, 가까운 거리 분위수를
   기준으로 차량은 6 m 깊이까지 보호한다. 같은 박스·깊이의 벽점도 보호될 수 있다.
2. 보호점은 벽 적합과 제거에서 제외된다(`TunnelFilterCore.cpp:1004,1274`). 이후
   1.0 m clustering이 벽 잔여점과 차량을 합치면 보호 비율을 충족한 큰 OBB도 남을 수 있다.
3. YOLO timestamp가 50 ms 안에 맞지 않으면 전방 FOV·30 m 안의 점을 ambiguous로
   남긴다. 보호 기능을 명시적으로 꺼도 `ComputeProtection`이 ambiguous를 계산하므로
   LiDAR-only 실험에 같은 fallback이 남는다(`TunnelFilterRos.cpp:981`).

따라서 벽 제거 거리만 키우는 실험으로 결론을 내리지 않는다. YOLO available/valid-empty/
stale/disabled, protected/ambiguous 점 수, 차량에 붙은 벽 비율과 차량점 보존률을 함께 본다.

새 ROI는 아래 원칙으로 설계한다.

- GPS 정상: 현재 operational HD맵의 차로·주행영역과 fresh pose로 **여유를 둔 관심 영역**을
  구성한다. 도로 진입 차량·횡단 객체·경계에 걸친 물체의 관측과 기존 track을 보존한다.
- 음영구간: `/perception/lane/boundary_family`에서 support·confidence·유효 구간을
  확인한 외곽 경계 사이의 envelope를 사용하고, 위치/피팅 오차를 반영한 margin을 둔다.
  단일 프레임 변화로 ROI가 급격히 수축하지 않도록 시간 정책을 둔다.
- 카메라 drivable semantic은 장애물 픽셀을 non-drivable로 만들 수 있다. **semantic mask의
  구멍을 LiDAR 장애물 삭제 마스크로 사용하지 않는다.** 경계 기반 corridor와 coverage,
  ground/support 증거로 사용하고, 장애물의 footprint와 수직 점들은 유지한다.
- 카메라 coverage 밖, 경계 누락, 낮은 confidence, stale 입력은 unknown이다.
  지원하지 않는 구간을 직선 외삽해 확정 ROI로 만들거나 free로 간주하지 않는다.
  검증된 넓은 fallback ROI로 복귀하며 상태를 진단한다.
- 지면 높이와 유효 외곽 경계로 벽/천장 후보를 줄이되, 높이·평면 구조 검증과 실제
  차량·보행자 보존 검사를 함께 둔다. 정적 장애물도 추적 실패만으로 사라지면 안 된다.

### 0.5 판제에 넘길 정보와 고주로 차로 귀속

LiDAR는 관측 시각의 장애물 위치·footprint/OBB·속도 의미·ID·추적 품질을 일관되게 만든다.
GPS 정상 소비자는 검증된 map projection을 사용할 수 있고, 음영구간 소비자는 rear-axle
`base_link` 기준 정보를 사용할 수 있어야 한다. 내부 tracker를 GPS 상태마다 바꾸거나
map→body→map 변환을 반복할 필요가 없는 의미를 [소비 계약](../../contracts/perception/lidar-planning-consumer-v1.md)에
설계 기준으로 고정했으며, 구현은 다음 단위에서 검증한다.

현재 `/last_lidar`, `/lidar_nogps`, `Msg/*`는 담당자 인계 경계다. 새 canonical candidate와
legacy adapter를 분리하고, field·MD5·stamp·frame·속도 정의와 active-mode 상태를 명시한다.
토픽 삭제·message 필드 추가·판제 fallback 변경은 이 문서만으로 진행하지 않는다.
특히 빈 배열은 **정상 관측 결과 객체가 없음**, 무발행/invalid는 **센서 또는 처리 상태 문제**다.
둘을 새 stamp의 빈 배열로 합치면 장애물이 사라진 것처럼 보인다.

고주로 차량의 차로 귀속은 `Lane.id/link_id/lane_number`와 객체 footprint의 겹침을 활용한다.
중심점 포함 여부 하나로 확정하지 않고 OBB footprint-overlap, 인접 차로 후보, 진행 방향,
이전 귀속과 hysteresis를 함께 평가한다. 차선을 밟거나 차로 변경 중이면 복수 후보/ambiguous
상태를 남긴다. 차로 번호는 HD맵의 competition-driving scope를 따르고, A433 연결 버스전용
차로처럼 대회 주행 밖의 차로를 단순 좌우 순번으로 다시 번호 매기지 않는다.
객체가 주행영역 밖이라는 사실과 감지할 필요가 없다는 판단은 분리한다.

gap/TTC·차선변경 실행 결정은 판제 책임이다. 기존 판제의 projection 계층과 중복되지 않도록
인지가 제공할 차로 귀속 진단/계약과 판제가 계산할 feature를 먼저 확인한다.

### 0.6 새 실험 기록과 기존 문헌의 사용 범위

새 실험 기록은 source commit, 실행 명령, 센서/profile SHA, bag SHA, 입력/출력 count,
frame/MD5/stamp audit, CPU/GPU·지연 percentile, GT/라벨 가용 여부를 각각 남긴다.
runtime smoke, 합성 테스트, replay 구조물 억제, GT 정확도를 서로 같은 수락 결과로
표현하지 않는다. Live/replay 수치와 미검증 범위는 [L1-A 검증 보고서](./lidar-l1a-validation-20260926.md)와
[기계 판독 기준선](../../../config/baselines/lidar_l1a_20260926.yaml)에서 관리하고,
이 문서에는 결론과 착수 gate만 반영한다.

외부 레포의 revision·license·추가/유지/보류 근거는 워크스페이스 문서
`docs/papers/lidar-reference-readiness-20260926.md`에서 관리한다(팀 레포 밖).
현재 참고 checkout은 build dependency가 아니며, 필요한 후보만 isolated 환경에서 실행한다.
1~13장의 다른 센서·CPU·데이터셋 FPS를 우리 VLP-16 점 수 비율로 나누어 runtime 성능으로
간주하지 않는다. 예전의 `TRAVEL 제외`, `SimpleTrack 코드 사용 불가`, 고정 DNN VRAM
예산 등은 당시 비교 후보 선정 기록이며, 동일 입력·환경의 실험 없이 절대 결론으로 쓰지 않는다.

학습은 라벨·시간 축·sensor extrinsic 감사와 holdout scenario 분리 후 착수한다.
Capture 자료는 검증된 단일 프레임 검출 실험에 먼저 쓰고, 연속 tracking·속도 평가는
실제 시뮬 경과시간과 동기화가 확인된 자료를 사용한다. backbone 교체는 허용된 연구 후보지만,
불명확한 frame/stamp 계약을 가리기 위해 큰 모델을 먼저 도입하지 않는다.

### 0.7 실행할 개발 단계와 완료 gate

| 단계 | 범위 | 완료 조건 |
|---|---|---|
| **L1-A 기준선 마감** | main/손재호/서준 consumer·센서·자료 감사, isolated build/test, live/offline 계측 | [검증 보고서](./lidar-l1a-validation-20260926.md), [manifest](../../../config/baselines/lidar_l1a_20260926.yaml), [소비 계약](../../contracts/perception/lidar-planning-consumer-v1.md)로 재현 가능. 전체 미션 수락은 아님 |
| **L1-B1: 좌표·시간 코어** | sensor/leveled/base_link/map 정합, 관측 stamp·processing·receipt 분리 | 격리 구현·시험 완료, [결과와 한계](./lidar-l1b1-validation-20260927.md). 운영 연결·연속 주행 수락은 미완료 |
| **L1-B2: GPS 전환·출력 선택** | status/좌표/stale/NaN/점프·복구, local/map/판제 가용성 분리 | [격리 후보와 실제 bag 감사](./lidar-l1b2-validation-20260927.md) 완료. 연속 주행·관측 pose·판제 전환은 미수락 |
| **L1-B3: camera 매칭** | YOLO11m /YOLO/BBox의 50ms stamp와 0/25ms 도착 순서 비교 | [스케줄러](./lidar-l1b3-validation-20260928.md)·[L1-B3-G](./lidar-l1b3g-validation-20260928.md): 2,562 scan에서 1,149→1,453(+304), timeout 1,109. same-scan geometry도 재생했으나 valid-empty fallback 위험으로 운영 미채택 |
| **L1-B3-G: 기하·AIM 감사** | 고정 same-scan geometry로 pinned filter 비교, AIM session/frame/time/GT 적합성 | 1,011 valid scans 중 geometry 107건 변화; 음영 99건 모두 valid-empty 선택. AIM 174 sessions, LiDAR 35/카메라 28 표본 확인; GT·실동시성 승인 없음 |
| **L1-B4-A: 독립 기하 보호** | camera 상태/보호 근거 분리, 관측 벽 지지 구간·잔차·국소 돌출부 보호 | [비교 기록](./lidar-l1b4-validation-20260929.md). 합성 진급 기준 통과와 실제 구조물 조각의 다수 객체 출력을 구분; 운영 미채택 |
| **L1-B4-B: Autonav 실제 GT 비교** | 새 Capture 105개의 점 보존·군집·OBB 단계별 비교, 실제 배치 우선 | [격리 검증](./lidar-l1b4b-capture-validation-20260929.md) 완료. 기존 wall-stage 대상점 100% 보존, 후보 조각 증가로 운영 미채택 유지. 보행자/연속 주행 수락은 미완료 |
| **L1-B5-A: 관측 객체 군집화** | 충분한 점의 차량 22개 실패와 기존 성공/구조물 합침 회귀를 고정, 후보 하나만 비교 | 다음 승인 단위. ROI·ground·wall·OBB를 동시에 변경하지 않고 완전성/순도·합침·비용 비교; 운영 미연결 |
| **독립 COCO 객체 모델 비교** | YOLO11m ↔ 11n/s ↔ 26n/s/m, 동일 이미지/옵션/weight SHA와 동시 부하 | 클래스·box·stamp 호환 및 차량/보행자 정확도와 매칭률·GPU/CPU/지연 수락; FP16/TensorRT는 그 후 별도 변수 |
| **L2 ROI·ground·clustering 비교** | boundary-family envelope와 fallback, 기존/linefit/Patchwork++ 등 지면 후보, range-adaptive/range-image clustering | 동일 bag에서 가까운 차량/낮은 장애물 recall 악화 없이 벽·천장 FP 감소; count·지연·자원 보고 |
| **L3 추적 안정화** | ego-motion 품질에 맞는 상태 좌표, 관측 앵커·연관·생애주기·불확실도 | 정상↔음영 ID 연속성, 정지/기동 객체 속도오차, 가림 후 복귀, 고주로 p95 지연 검증 |
| **L4 HD맵 차로 귀속·선택적 퓨전** | footprint overlap/ambiguity, YOLO 선택적 semantic 보조, 판제 인계 | 차로 경계/변경·진입 객체 유지, 클래스 없이도 장애물 보존, consumer joint replay |
| **L5 필요 시 학습 후보** | capture 라벨 검증, 단일-sweep 경량 검출 baseline, 별도 training 컨테이너 | holdout 개선 및 동시 runtime 예산 충족; 기하 baseline과 OOD/unknown 비교 |
| **후속 배포** | MolitComp no-GT 수락, 담당자 검토/PR, LAN2LAN 리허설 | GT 의존 없는 실행·rollback·네트워크 현장 검증 |

좌표·시간 및 GPS 전환 계약을 통과하기 전에는 L2 알고리즘을 기본 경로로 교체하지 않는다. 판제의 정지/보수 동작을
완화하거나 오래된 관측을 fresh하게 위장해서 성능 지표를 높이지 않는다. 기존 안전 동작의
변경은 degraded 상태·관측 age·unknown 처리와 함께 판제 담당자가 결정한다.

### 0.8 L1-B1 결과와 운동보정의 경계 — 2026-09-27

지면 제거→전처리→ROI→객체→추적의 품질을 개선하되 같은 관측의 좌표·시간을 먼저
고정해야 비교가 공정하다. 이번에는 **독립 C++/Eigen 코어**만 추가했다. 결과·재실행·SHA는
[L1-B1 보고서](./lidar-l1b1-validation-20260927.md)와 [manifest](../../../config/baselines/lidar_l1b1_20260927.yaml)에 있다.
운영 패키지·판제·메시지는 수정하지 않았다.

| 기능 | 의미·필요 정보 | 이번 범위 |
|---|---|---|
| 장착 변환 | sensor를 rear-axle body frame으로 옮기는 고정 회전·이동 | 코어 구현. GPS 안테나 lever arm과 별개 |
| roll/pitch 수평화 | 다이브/스쿼트에 따른 pitch·롤을 중력 기준으로 표현. 상하 이동 자체나 스캔 왜곡은 해결하지 않음 | 이미 적용한 행렬을 역변환해 body로 복원. 새 지면 처리 미연결 |
| 스캔 deskew | 점마다 다른 취득 시각·차량 pose로 한 기준 시각에 정렬 | 보류. 현재 XYZI에는 점별 time/ring이 없고 MORAI 생성 방식 미확인 |
| 프레임 간 자차 운동보상 | 연속 scan/track 관측을 같은 시간·상태 기준으로 비교 | 후속 GPS/twist/추적 단위. 이번 단일 관측 변환과 다름 |

`base_link`는 차체와 함께 기울어진다. 수평화 좌표를 `base_link`라고 부르면 카메라·맵·GT와
어긋난다. `p_base = T_base_sensor · L_applied⁻¹ · p_leveled`,
`p_map = T_map_base(t_obs) · p_base`이며 raw에는 `L⁻¹`을 쓰지 않는다.
중심과 8개 코너에 동일 변환을 적용하고 3D 코너를 yaw-only 박스로 재생성하지 않는다.
GT도 API별 sensor/GPS/차량 중심·rear-axle 원점을 확인해 평가 기준으로 변환해야 한다.
이번에 gRPC 중심 원점을 추정하거나 비동기 GT를 동기 정답으로 사용하지 않았다.

강체 변환은 점 수·intensity를 줄이지 않는다. 부동소수 반올림은 있지만 voxel처럼 여러
점을 대표값으로 합치는 정보 축약은 아니다. 5개 raw cloud와 100개 이미 ROI 처리된
Capture cloud의 count/intensity·roundtrip을 검사했다. 원본 2,562 scan은 stamp/도착 순서
검사 대상이지 전부의 XYZ 성능 평가가 아니다. 기존 **약 0.35 ms** 자세 보정은 ROI+voxel
이후 계측이며 raw 전체 deskew 비용으로 일반화하지 않는다.

카메라와 LiDAR의 시간·자세 이력 공유는 유용하다. 촬영 시각의 BEV 지면 투영은 별도
검증하며 IMU만으로 노면 경사·차체 상하 이동을 모두 알 수는 없다. 동시각 센서 간 고정
extrinsic 투영에 전역 자세를 중복 적용하지 않는다.

기존 adapter는 pitch-only 합성 입력에서는 일치하지만 roll 추가 시 이 fixture에서 중심
약 **0.250 m**, 코너 최대 **0.387 m** 차이가 난다. 후보 double 합성 오차는 `3.56e-15`
이하다. 코드 원인의 재현이지 전 구간 주행 오차 측정은 아니다. 실제 bag에 당시 수평화
행렬/pose snapshot이 없어 정확한 legacy 전후 재현이라고 하지 않는다.

기존 24개 + Autoware distortion corrector/LIO-SAM 2개를 고정 SHA로 목록화하고
열람·주요 입출력 코드 검사·빌드·실험을 분리했다. 새 2개는 sparse checkout 참고용이며
ROS2/SLAM 의존성이 아니다. 지면 첫 추가 비교는 **linefit**, Patchwork++는 동일 조건
재검증, SimpleTrack은 연관/생애주기 참고다. GroundGrid는 유효 odometry, DipG-Seg는
VLP16 투영 검증이 선행이다. 학습 검출기/MOS는 후순위다.

L1-B2의 격리 결과는 [별도 기록](./lidar-l1b2-validation-20260927.md)에 남겼다. YOLO11m 객체
box의 즉시/25ms scheduler, 실제 same-scan filter geometry와 AIM 적합성은 각각
[L1-B3](./lidar-l1b3-validation-20260928.md) 및 [L1-B3-G](./lidar-l1b3g-validation-20260928.md)에
나눠 기록했다. corrected run은 2,562 scan에서 304개 추가 매칭, timeout 1,109이며 1,011 valid
filter inputs 중 107개 geometry가 달라졌다. 음영 변화 99건은 모두 valid-empty에 따른
ambiguous-wall fallback 비활성화였다. 대기 후보는 운영에 연결하지 않는다. AIM의 대표 payload
format은 확인했지만 physical simultaneity, route별 map 대응, 객체 GT 정확도는 미수락이다.
기존 planner 입력은 교체하지 않았다.

### 0.9 L1-B4-A 독립 벽·장애물 분리 비교 — 2026-09-29

[비교 보고서](./lidar-l1b4-validation-20260929.md)와
[도구/재실행](../../../tools/lidar/tunnel_geometry/README.md)에 고정 입력·SHA와 세 비교군을
남겼다. 카메라 valid-empty를 미수신처럼 취급하는 대조군, pinned 기준선, 벽 잔차·지지 구간과
국소 전경 연결성을 사용하는 후보를 비교한다. 양성 camera 보호와 기하 보호는 별도이며
기하 점을 car/person class로 위장하지 않는다. 원래 ground/voxel/wall fit/군집/L-shape OBB와
제거 조건은 변경하지 않았다. 기존 36조건에 희소 first-return·양쪽 벽·단차/곡면·가림 등을
추가한 116조건/580 frame을 같은 입력으로 재생했다.

핵심 합성 car/person/low-unknown 세 사례는 wall-stage 객체점 100% 보존,
final completeness/purity 100%와 미수신 기준 잔여 벽 100% 감소를 보였다.
하지만 실제 음영 정차 입력에서는 후보의 최종 객체가 평균 약 17~19개/scan으로
늘고 벽 근처의 좁고 높은 OBB가 다수 남았다. 정답 없는 객체 수를 오탐률로
치환하지는 않지만, 합성 통과만으로 안전한 장애물 출력을 수락할 근거도 없다.
현재 후보는 **운영 미채택**이며 실제 구조물과 장애물을 구분할 후속 실험을 요구한다.

희소 입력 중 낮은 장애물 반환점이 0인 사례는 검출 성공으로 세지 않는다.
입력 관측 부족, ground/ceiling 선행 삭제, wall 삭제, cluster 병합·최소점, OBB 제거를
구분해야 한다. AIM 7개 표본의 형식·장착 roundtrip만 추가 확인했고, attitude/차량 기준점
미확인으로 filter replay 승인은 0이다. YOLO 교체, tracker/ground 변경, GPS 운영 연결은
이 실험 결과에 섞지 않았다.

---

### 0.10 L1-B4-B 실제 Capture GT 결과 — 2026-09-29

[상세 보고서](./lidar-l1b4b-capture-validation-20260929.md)와
[재실행 도구](../../../tools/lidar/capture_validation/README.md)는 새 Capture 105개를
Instance 점 GT로 채점한다. camera missing/empty는 통제 조건이며 실제 YOLO 도착 기록이 아니다.
각 장면 5회 정차 warmup·독립 reset으로 비교했고 같은 관측을 연속 tracking으로 해석하지 않았다.
GT box 불일치 43관측과 잘못된 native IMU nSecs 81개는 제한으로 남겼다.

현재 실제 배치에서 baseline도 wall-stage Vehicle/Obstacle 점 20,120개를 모두 보존했다.
후보가 labelled target 지원 객체를 늘리지 못한 반면 최종 출력은 empty baseline 312개에서
1,549개로 늘었다. GT 없는 구조물 조각을 측정 FP로 단정하지 않지만 운영 채택 근거는 없다.
충분한 voxel이 있는 Vehicle 91관측 중 19개는 불완전한 군집, 3개는 군집 미생성이다.
따라서 극단적 벽 근접 배치 추가 수집보다 **기존 자료의 군집 분할·누락 비교 L1-B5-A**를
다음 독립 수정으로 제안한다. 거리 적응형/높이 제약 XY 후보는 실패 간격 분석 후 하나를 선정한다.
보행자/낮은 물체·실제 YOLO 부하·연속 주행·GPS 전환 수락과 운영 연결은 여전히 미완료다.
이전 상세 기록은 그대로 보존한다.

---

## 1. 이전 평가의 범위와 한계 — 2026-09-21 기록

> 아래 1~13장의 "현재"는 최초 소스 분석 시점을 가리킨다. 일반도로/main 계열 결함의
> 근거와 후보 설계의 상세 논의는 보존하지만, 현재 실행 순서와 승격 기준은 0장을 따른다.
> 소스 감사에서 도출한 원인 가설과 타 데이터셋 논문 수치를 대회 시나리오의 실측으로 읽지 않는다.

- **방법:** 소스 정독과 문헌 조사. **실행·계측은 하지 않았다.**
- `main.launch`의 `benchmark_lidar_pipeline`으로 단계별 계측 기능이 이미 구현되어 있으나
  저장된 결과 로그가 리포지토리에 없다. 이 문서의 **연산량 관련 수치는 전부 추정치**이며,
  개선 착수 전 해당 벤치마크를 먼저 돌려 기준선을 확보해야 한다.
- 정확도 관련 주장은 코드 경로와 상수값에서 연역한 것이다. 실제 시나리오에서의 발생 빈도는
  확인하지 않았다.

---

## 2. 현재 파이프라인

`LidarProcess()` 한 프레임의 처리 순서다.

```text
RAW UDP/ROS
  -> MakePointCloudFromRaw
  -> Roi              고정 직육면체 + 자차 박스 제외
  -> Voxel            고정 0.1 m leaf
  -> CorrectingPoints IMU pitch 역회전
  -> NormalVectorFiltering   단일 평면 RANSAC 지면 제거
  -> Clustering       Euclidean, 고정 tolerance
  -> Objectinfo_for_km  클러스터별 L-Shape OBB 생성
  -> FrameFilter -> CompareCentroids
                      Hungarian 연관 + CA Kalman + 절대속도 산출
  -> MotionClassifier 동/정적 판정 (트래킹 속도는 변경하지 않음)
  -> FusionProcess    runCameraFusion(YOLO 클래스) -> FusionLiDAR(AABB 갱신)
  -> PlannerObjectRosPublisher / PlannerObjectSender
```

주요 출력

| 출력 | 내용 | 주행가능영역 필터 |
|---|---|---|
| `/lidar/tracked_objects_map` | map 좌표 tracked object 전체 | **없음** |
| `/last_lidar` | 동일 내용 | `road_ratio >= 0.3` 적용 |
| UDP 7510 | Frenet planner 전용 객체 | 별도 확인 필요 |

---

## 3. 완성도 평가

| 단계 | 구현 | 평가 | 조치 |
|---|---|---|---|
| UDP/ROS 수신·중복 프레임 스킵 | `MakePointCloudFromRaw` | 양호 | 유지 |
| ROI | 고정 직육면체 `x[-30,50] y[±20] z[-2,3]` | 취약 | 교체 |
| Voxel | 고정 0.1 m | 보통 | 조건부 유지 |
| IMU 보정 | pitch만 적용, roll 없음, intensity 유실 | 결함 | 수정 |
| 지면 제거 | 단일 평면 RANSAC, thr 0.3 m, iter 20 | **최대 약점 1** | 교체 |
| 클러스터링 | Euclidean 고정 tolerance 1.5 m | **최대 약점 2** | 교체 |
| OBB | search-based L-Shape(Zhang) + yaw EMA + 재투영 | **잘 구현됨** | 유지·보강 |
| 연관 | Hungarian, cost = w·dist + w·IoU | 보통 | 보강 |
| 필터 | CA Kalman 단일, Q=0.003I, R=0.01I, IMM 전부 주석 | 취약 | 교체 |
| 절대속도 | `rel_v + CAN속도`, body frame | 결함 | 재설계 |
| 동/정적 판정 | `MotionClassifier` median + streak + candidate 승격 | 과잉 보상 계층 | 원인 제거 후 축소 |
| 카메라 시간 동기 | `FusionTimeMatcher` + 큐 기반 | **매우 양호** | 유지 |
| 카메라 연관 | 투영 AABB 확대 후 IoU | 취약 | 보강 |
| HD Map 연동 | `road_mask`를 퍼블리셔에서만 사용 | 자원 낭비 | 앞단 이동 |

**총평.** 구조 분해와 경계 설정(퓨전 시간 동기, planner adapter, motion classifier 분리)은
교과서적으로 잘 되어 있다. 문제는 각 단계의 알고리즘 선택과 상수값이며, 상단의 손실이
하단에서 보상 로직을 계속 만들어내고 있다.

핵심 한 줄: **HD Map과 카메라 주행가능영역이라는 강한 사전정보를 이미 보유하고 있는데,
파이프라인 맨 끝에서 결과를 버리는 용도로만 쓰고 있다.**

---

## 4. 증상별 원인 분석

판단제어 파트가 보완로직으로 메우고 있는 증상 세 가지를 코드 근거와 함께 특정한다.

### 4.1 정적 장애물의 높이가 틀리거나 물체가 사라진다

원인이 네 개 겹쳐 있다.

**(a) RANSAC이 장애물 밑단 30 cm를 제거한다.**
`src/Main/src/Lidar/ImuRansac/Ransac.cpp`

```cpp
pcl_Seg.setMaxIterations(20);
pcl_Seg.setDistanceThreshold(0.3);   // 코드 주석에는 20cm로 적혀 있음
```

80 m x 40 m 영역을 **단일 평면 하나**로 근사하면서 허용 오차가 30 cm다. 평면에서 30 cm
이내의 모든 점이 삭제되므로

- 라바콘, 드럼통, 연석, 낮은 파편은 몸통째 사라진다
- 차량은 바닥 30 cm 이상이 잘려 `f32_OBB_ZMin`이 실제보다 위에 잡힌다
- 결과적으로 `height`가 **상시 과소평가**된다

**(b) RANSAC iteration 20은 통계적으로 부족하다.** PCL 기본값은 50이다. 벽을 걸러내기 위해
3회 반복하도록 했지만(`s32_Max_iterations = 3`), 20 샘플로는 "가장 큰 평면"의 선택이
프레임마다 달라진다. 지면 높이가 프레임 간에 출렁이고, 같은 물체가 떴다 사라졌다 한다.

**(c) 지면을 못 찾으면 전체 통과.**

```cpp
if (!ground_removed) { *pcl_Cloud4 = *pcl_Cloud3_4; ROS_WARN_THROTTLE(...); }
```

폴백 자체는 안전한 선택이지만, 지면이 그대로 들어오면 4.3의 1.5 m 클러스터링과 만나
장면 전체가 하나의 덩어리가 된다.

**(d) 하드코딩된 높이 컷이 물체를 노이즈로 처리한다.**
`src/Main/src/Lidar/Tracking/Tracking.cpp`

```cpp
if (st_CurrentObj.f32_CoreZ <= -1.3) { st_CurrentObj.str_ObjectClass = "지면노이즈"; }
```

LiDAR 장착 높이가 1.45 m이므로 지면은 센서 좌표로 약 -1.45 m다. 즉 **높이 30 cm 이하의
물체는 centroid가 -1.3 이하로 내려가 무조건 "지면노이즈"** 로 분류되고,
`PlannerObjectAdapter`의 `IsNoise()`가 판제로 내보내지 않는다. "정적 장애물이 사라진다"의
직접 원인이다.

> **2026-09-26 정정:** 위 높이 환산은 mount 기준 원점과 노면을 같다고 가정한 초판의 오류다.
> 1.45 m는 rear-axle 기준 mount이고 노면은 base_link 아래다. 0.3의 기존 카메라 ground
> calibration과 예비 LiDAR fit을 참조한다. `CoreZ` 하드컷은 여전히 문제 후보지만 실제 삭제
> 높이는 지면 fit·객체 형상·후단 클래스 갱신에 따라 측정해야 한다.

### 4.2 장애물 속도가 부정확하다

**(a) 절대속도 공식이 종방향 1차원 가정이다.**
`src/Main/src/Lidar/Tracking/Tracking.cpp`

```cpp
const float32 v_mps = st_GPSIMU.st_CAN.f32_CurrentVel_KPH / 3.6f;
const float32 f32_Abs_vx = f32_Rel_vx + v_mps;   // x축에만 더함
const float32 f32_Abs_vy = f32_Rel_vy;           // y 보정 없음
```

`compensate_ego_yaw()`가 상태를 `T(-dpsi)`로 회전시켜 회전 성분을 1차 근사로는 처리한다.
그 결과 Kalman이 추정하는 값은 사실상 `-v_ego`이고, 절대속도는 **크기가 큰 두 값의 차**가
된다. 고주로 80 km/h면 22.2 m/s짜리 두 값을 빼는 것이라, 필터가 3 %만 지연돼도
**0.7 m/s(2.4 km/h)의 가짜 속도**가 생긴다. `motion_lateral_threshold_kph = 3.0`과
거의 같은 크기다. 즉 **속도 오차가 자차 속도에 비례해 커진다.**

**(b) 곡선 진입 시 과도응답이 그대로 나타난다.** 요레이트가 변하는 순간 Kalman의 상대속도
추정이 뒤따라가지 못하고, 그 지연이 정지 물체의 가짜 횡속도로 나타난다. 30 m 밖 가드레일
이면 요레이트 변화 0.1 rad/s만으로 3 m/s(10.8 km/h)가 튄다.

**(c) 측정 노이즈가 거리와 무관하게 고정이다.**
`src/Main/src/Lidar/Kalman/Kalman.cpp`

```cpp
Eigen::Matrix<float,6,6> Q = ... * 0.003f;
Eigen::Matrix2f          R = Eigen::Matrix2f::Identity() * 0.01f;   // 표준편차 10 cm
```

60 m 밖 트럭의 클러스터 중심을 표준편차 10 cm로 신뢰한다. 실제로는 가림이 바뀔 때마다
centroid가 1 m 이상 점프한다. 뒷면만 보이다 옆면이 보이면 중심이 차량 길이의 절반만큼
이동한다. 필터가 이 점프를 그대로 속도로 변환한다.

**(d) 관측점이 centroid다.** L-Shape OBB를 잘 만들어 두고도 Kalman 측정값은 클러스터 평균점
(`pcl_Cloud5`)이다. centroid는 LiDAR에서 가장 불안정한 관측량이다.

**(e) 그 결과로 `MotionClassifier`(393줄)와 판제 보완로직이 생겼다.** median + streak +
candidate 승격은 정교하지만 **원인 제거가 아니라 증상 마스킹**이고, `history_size = 5`
만큼(15 Hz 기준 약 0.33 s) 반응이 지연된다.

### 4.3 도로 밖 기물까지 잡혀 ID 트래킹과 장애물 구별이 어렵다

**(a) 클러스터 tolerance 1.5 m가 지나치게 크다.**
`src/Main/src/Lidar/Clustering/Clustering.cpp`

```cpp
pcl_Ec.setClusterTolerance (1.5);   // 코드 주석에는 100cm로 적혀 있음
pcl_Ec.setMinClusterSize (10);
```

차선 폭이 3.5 m다. 1.5 m면 갓길 가드레일과 1차로 차량이 한 덩어리가 되고, 보행자가 차량
옆 1.4 m에 서 있으면 합쳐진다. 터널에서는 벽 전체가 하나의 거대 클러스터가 된다.
합쳐진 클러스터는 중심이 크게 움직이므로 **OBB 붕괴 -> 속도 폭주 -> ID 스위치**로 이어진다.
증상 세 가지가 여기서 한꺼번에 나온다.

**(b) `setting.yaml`의 클러스터링 설정이 코드에 연결되어 있지 않다.**
`LiDAR.Clustering.Eps: 0.5`, `MinPoints: 5`가 `main.cpp`에서 `st_LiDAR.st_Clustering`으로
파싱까지 되지만 `Clustering.cpp`는 그 값을 읽지 않는다. **yaml을 고쳐도 아무 일도 일어나지
않는다.**

**(c) 고정 tolerance는 거리에 따라 반대로 틀린다.** VLP-16 기준 수직 빔 간격(2도)은
10 m에서 약 0.35 m, 50 m에서 약 1.75 m다. 1.5 m 고정은 **근거리 과병합, 원거리 과분할**이다.

**(d) `road_mask`를 맨 끝에서만 사용한다.**
`src/Main/src/PlannerInterface/PlannerObjectRosPublisher.cpp`의 `road_ratio < 0.3` 필터는
RANSAC, 클러스터링, 트래킹, 퓨전을 **전부 수행한 뒤**에 적용된다. 연산량은 그대로 소모하고,
도로 밖 점이 이미 지면평면 적합과 클러스터를 오염시킨 뒤라 효과도 절반이다.
또한 이 필터는 `/last_lidar`에만 걸리고 `/lidar/tracked_objects_map`은 필터 없이 나간다.

### 4.4 증상과 별개로 확인된 결함

| 위치 | 내용 | 영향 |
|---|---|---|
| `Lidar/ImuRansac/ImuFusion.cpp` | `pcl_NewPoint.intensity` 미복사 → 미초기화 값 전파 | intensity 전 구간 사용 불가. 노면 반사도 기반 판단 불가 |
| `Lidar/ImuRansac/ImuFusion.cpp` | roll 보정 없음, 출력 cloud의 width/height 미갱신 | 캔트 구간 지면평면 오차 |
| `SensorFusion/FusionLiDAR.cpp` | `UpdateAABB()`가 `f32_CoreX/Y/Z`를 AABB 중심으로 덮어씀 | 판제가 받는 `cluster_center`는 Kalman 추정치가 아닌 생 AABB 중심. **위치와 속도가 서로 다른 추정기에서 나온다** |
| `SensorFusion/FusionLiDAR.cpp` | `if (obj.s32_MissCount <= 0) continue;` 조건 반전 | `/lidar/object_boxes`에 유령 트랙만 그려짐(디버그 한정) |
| `src/Msg/msg/Yolo_bbox.msg` | confidence 필드 없음. `conf_thres` 기본 0.7 | 퓨전이 신뢰도 가중 불가. 0.7은 원거리 보행자를 버리는 값 |
| `Lidar/Tracking/Tracking.cpp` | `objPoints = &vec_AllClusterIndices[i]`, 값 전달 파라미터의 주소 | dangling pointer. 현재 역참조는 주석 처리된 `merge()`뿐이라 잠복 상태 |
| `Lidar/IMMFilter/IMMFilter.cpp` | IMM 구현 전체가 주석 | 끼어들기·급감속 등 기동 물체 대응 없음 |
| `Lidar/Tracking/Tracking.cpp` | L-Shape closeness 구현이 원논문 Algorithm 4와 다름. 논문은 축마다 점집합 전체의 l2 norm이 작은 경계 **하나**를 고른 뒤 per-point min을 취하는데, 코드는 per-point로 양쪽 경계 중 가까운 쪽을 취한다 | 정상 L자에서는 결과가 비슷하나, 병합되거나 가늘고 긴 클러스터에서 먼 쪽 경계에 붙은 점이 점수를 부풀려 방향이 틀어질 수 있다 |
| `Lidar/Tracking/Tracking.cpp` | 신규 검출에 즉시 ID 발급 (tentative 상태 없음) | 깜빡이는 클러스터가 ID를 계속 새로 발급. 반면 미매칭 트랙 25프레임 유지는 **문제가 아니다** — 6.5 (b) 참조 |

---

## 5. 이전 실행 환경 예산 가정

> **2026-09-26 정정:** 아래는 시뮬레이터가 별도 PC일 때의 배포 가정과 미측정 예산이다.
> 현재 로컬 26r1은 GPU를 함께 사용하며, 실제 자원 조건은 0.1과 L1-A 보고서를 따른다.

대회 현장과 개발 환경 모두 **시뮬레이터는 별도 PC에서 구동하고 LAN으로 연결**한다.
따라서 이 PC의 자원은 전부 우리 코드 실행에 쓴다.

| 자원 | 값 |
|---|---|
| GPU | RTX 4060 Ti 8 GB |
| CPU | i5-14600K (14코어 / 20스레드) |
| RAM | 15.5 GiB |

단, 시뮬레이터가 빠졌다고 해서 GPU가 비는 것은 아니다. 같은 PC에서 아래가 동시에 돈다.

| 소비자 | 예상 VRAM | 비고 |
|---|---|---|
| 카메라 semantic (TwinLiteNet+ × 3 view) | 1.0 ~ 1.5 GB | 경량 모델 |
| YOLO11m (객체 검출) | 1.0 ~ 1.5 GB | TensorRT/FP16 기준 |
| **LiDAR 인지에 배정 가능** | **2 ~ 3 GB** | |
| CUDA context, RViz, 여유 | 1 GB 이상 | |

**지연 예산.** LiDAR 15 Hz이므로 프레임당 66.7 ms. 판제와 다른 노드가 같은 CPU를 쓰므로
**LiDAR 체인 전체를 25 ms 이하**로 잡는 것을 목표로 한다.

**입력 밀도.** VLP-16은 단일 리턴 30만 점/초, 15 Hz에서 **약 2만 점/프레임**이다. KITTI(64ch)나
nuScenes(32ch)보다 훨씬 희소하다. 이는 두 가지를 뜻한다.

- Pillar 계열 3D 검출기의 연산 부담은 논문 수치보다 훨씬 작다
- 반대로 **공개 데이터셋 사전학습 가중치는 그대로 전이되지 않는다.** 반드시 자체 학습이 필요하다

**RAM이 VRAM보다 먼저 병목이 된다.** 15.5 GiB에서 ROS 스택과 학습 작업을 동시에 돌리지 않는다.
학습은 주행 검증과 시간대를 분리한다.

---

## 6. 개선안

### 6.0 GT → 센서 전환 분석 (이 문서의 전제)

**이 절을 먼저 읽는다.** 대회에서는 시뮬레이터 정답값이 사라지고 모든 입력이 센서 기반이
되어야 한다. 현재 판제는 정답값으로 돌고 있으므로, LiDAR 품질 개선만으로는 전환이 끝나지
않는다. **판제 쪽에도 반드시 필요한 변경이 있다.**

#### 6.0.1 현재 연결 상태

```text
[현재 - 개발 중간 단계]
  MORAI GT (/Object_topic)
      -> behavior_planner/scripts/morai_objects_adapter.py
      -> /perception/objects  (asmc_msgs/DetectedObjectArray)
      -> Frenet / Behavior

  MORAI GT (/Ego_topic)
      -> asmc_bridge/src/vehicle_state/ego_vehicle_state_dev_node.cpp
      -> /vehicle/state

  LiDAR 패키지
      -> /lidar/tracked_objects_map, /last_lidar  (Msg/TrackedObjectArray)
      -> UDP 7510
      -> (판제가 구독하지 않음)
```

```text
[대회 구성 - 목표]
  LiDAR + 카메라  -> /perception/objects   (asmc_msgs/DetectedObjectArray)
  GPS + IMU       -> /vehicle/state        (asmc_bridge/src/vehicle_state/vehicle_state_node.cpp)
```

**좋은 소식이 하나 있다.** `/vehicle/state`는 **센서 기반 경로가 이미 구현되어 있다.**
`vehicle_state_node.cpp`가 `/gps`, `/imu`를 받아 속도 추정(GPS 보정 gain, IMU 가속도 필터,
innovation 제한)까지 수행한다. 즉 측위 쪽은 GT 노드를 dev 노드로 두고 센서 노드로 교체하는
구조가 이미 준비되어 있다. **객체 쪽만 비어 있다.**

이 사실은 6.5 변경 1(map frame 추적)의 전제이기도 하다. map frame 추적은 측위 품질에
의존하는데, 그 경로가 이미 존재하므로 품질 계측만 하면 된다.

#### 6.0.2 계약

canonical topic은 [`src/interfaces/README.md`](../../../src/interfaces/README.md)의
`/perception/objects` : `asmc_msgs/DetectedObjectArray`이며 `Frenet`과 `FSM`이 구독한다.
`ConvertDetectedObjects()`가 읽는 필드는 다음이 전부다.

| 필드 | 사용처 |
|---|---|
| `id` | **`AvoidTargetResolver`와 FOLLOW leader가 ID로 대상을 추적한다.** 중복 ID는 메시지 전체 거부 |
| `type` (uint8) | `behavior_planner`가 VEHICLE / PEDESTRIAN / STATIC_OBSTACLE로 분기 |
| `center` (xyz) | 위치 |
| `yaw` | heading. 판제가 이걸로 `yaw_rate`를 추정한다 |
| `vx`, `vy` | map frame 절대속도 [m/s]. `speed = hypot(vx, vy)` |
| `yaw_rate` | **`MergeGapEvaluator`의 CTRV 예측에 직접 들어간다** |
| `width`, `length`, `height` | 셋 다 양수여야 함 |
| `obb_corners[8]` | 충돌검사의 authoritative geometry |

`classification` 문자열, `is_dynamic`, confidence, 관측 신뢰도는 **계약에 없다.**

#### 6.0.3 GT와 실센서의 근본적 차이 — 판제가 받는 충격

이 표가 이번 전환의 핵심이다. 각 항목은 코드 근거를 확인한 것이다.

| # | GT 특성 | 실센서 | 판제 쪽 결과 | 근거 |
|---|---|---|---|---|
| 1 | **가림과 무관하게 전부 제공** | 가려진 영역은 아무것도 없음 | **"관측 못 함"과 "비어 있음"을 구별할 수단이 없다.** 판제가 미관측 영역을 자유공간으로 취급 | 계약에 unknown 표현 없음 |
| 2 | ID가 영구 고정 | ID 이탈 발생 | `AvoidTargetResolver`가 `AvoidTargetStatus::LOST` 반환. FOLLOW leader와 `coast_exempt_object_id`도 ID 참조 | `Maneuver/Avoid/AvoidTargetResolver.cpp`, `Pipeline/TrajectoryRequest.hpp` |
| 3 | 속도 정확 | 잡음 있음 (4.2) | gap / TTC 오판 | `speed = hypot(vx, vy)` 직접 사용 |
| 4 | heading 정확 | OBB yaw 흔들림 + pi 모호성 | **판제가 heading 이력으로 추정하는 `yaw_rate`가 잡음을 증폭한다.** 그 값이 CTRV 원호 예측 `speed/yaw_rate * (sin(psi + w t) - sin psi)`에 그대로 들어간다 | `Maneuver/Merge/MergeGapEvaluator.cpp:57-61` |
| 5 | 거리 무제한 | 차량 40 ~ 50 m, 보행자 25 ~ 30 m (6.8) | 원거리 의사결정 근거 소실 | 8.4 |
| 6 | 항상 유효 | 가끔 불량 프레임 | **변환 실패 시 판제가 즉시 급정지한다** | 아래 |

**6번이 가장 급하다.**

```cpp
// frenet_planner_node.cpp:236-245
const ConversionStatus objects_status = ConvertDetectedObjects(snapshot.objects, objects);
if (objects_status != ConversionStatus::OK) {
    ROS_WARN_THROTTLE(1.0, "Frenet STOP required: objects conversion failed (%s)", ...);
    publishStop(snapshot, asmc_msgs::Trajectory::FAILURE_INVALID_INPUT);
    return;
}
// v2_planning.cpp:337-350 도 동일하게 MakeStopTrajectory()를 발행한다
```

`ConvertDetectedObjects()`는 **객체 하나가 잘못되면 그 프레임의 모든 객체를 버리고 오류를
반환한다.** 개별 객체를 건너뛰지 않는다. 거부 조건은 비유한값, 치수 0 이하, **중복 ID**,
`ValidateCorners8` 실패, `z_range.max - z_range.min <= 1e-6`이다.

GT 입력에서는 변환 실패가 곧 소프트웨어 버그이므로 정지가 옳은 설계다. **센서 입력에서는
간헐적 불량 프레임이 정상 동작의 일부다.** 객체 하나 때문에 주행 중 차가 서면 안 된다.

**공정하게 적자면 현재 LiDAR 어댑터는 이미 상당히 방어적이다.** `MakePlannerObjects()`가
유한성, 최소 치수(`kMinBoxSide = 0.05`), corner 유한성을 검사한다. 판제 쪽 임계도 관대하다
(`min_xy_area = 1e-6`, `duplicate_epsilon = 1e-6`). **그러나 ID 유일성 검사는 어느 쪽에도
없다.** 그리고 어떤 방어도 100 %가 아니므로, "한 번 실패 = 급정지"라는 결합 자체를 끊어야 한다.

#### 6.0.4 과거 제안: 판제 측 변경 3가지 — 자동 적용하지 않음

> **2026-09-26 정정:** 아래 P1~P3는 당시 제안이며 판제 담당자의 확정 정책이 아니다.
> 특히 malformed 객체를 조용히 버리거나 stale 목록을 재사용하면서 정상 입력으로 표시하면
> 실제 장애물을 잃을 수 있다. 오류 종류·관측 age·unknown/degraded 상태와 소비자 동작을
> 먼저 합의한다. fail-closed 정지는 인지 통합이 임의로 완화할 대상이 아니다.

| ID | 변경 | 위치 | 이유 |
|---|---|---|---|
| **P1** | 객체 변환 실패를 **프레임 전체 거부 → 불량 객체만 건너뛰기**로 완화. 유효 객체가 하나도 없으면 직전 프레임을 짧은 grace(예: 0.3 s) 동안 재사용하고, 그 후에야 정지 | `message_converter.cpp`, `frenet_planner_node.cpp`, `v2_planning.cpp` | 6.0.3의 6번. **안전 결합 해제** |
| **P2** | `yaw_rate`를 heading 이력이 아니라 **속도 벡터에서 유도**한다(속도 임계 이상일 때). 저속에서는 0으로 두고 CTRV 대신 직선 예측 | `MergeGapEvaluator.cpp` 및 yaw_rate 추정부 | 6.0.3의 4번. 잡음 증폭 차단 |
| **P3** | AVOID 대상과 FOLLOW leader를 **ID 단독이 아니라 (ID, 위치·속도 근접성)** 으로 재획득. ID가 바뀌어도 같은 물체면 이어서 추적 | `AvoidTargetResolver.cpp` | 6.0.3의 2번. ID 이탈 내성 |

초판은 P1을 필수 완화로 단정했으나 이 권고는 철회한다. 변환 거부율을 낮추는 producer
품질 개선은 진행하고, consumer의 정지·grace·부분 수락 정책은 오류가 가리는 위험까지
측정한 뒤 판제 오너와 결정한다. P2/P3 또한 실제 입력 계약과 회귀 검증이 필요하다.

#### 6.0.5 제안: 미관측 영역 전달 — 이번 전환의 가장 큰 안전 간극

6.0.3의 1번은 메시지 하나로 해결되지 않는 **구조적 간극**이다. GT는 가림을 모르지만 LiDAR는
가림이 본질이다. 선행 차량 뒤, 커브 안쪽, 터널 벽 뒤는 **비어 있는 것이 아니라 모르는 것**인데,
현재 계약에는 그걸 표현할 방법이 없다. 판제는 이를 자유공간으로 본다.

**두 가지 선택지가 있고, 두 번째를 권한다.**

| 안 | 내용 | 비용 |
|---|---|---|
| (a) 3-state 점유 격자 | `nav_msgs/OccupancyGrid`로 free / occupied / unknown BEV 발행 | 신규 토픽 + 판제가 격자 샘플링 로직 신설 |
| **(b) 차로별 관측 지평 (권장)** | **각 차로 중심선을 따라 "가림 없이 관측된 최대 거리"** 하나를 발행 | 신규 토픽 1개 + 판제는 속도 상한 한 줄 |

> **2026-09-26 보완:** 아래 관측 지평은 연구 후보다. 가장 먼 return이나 camera coverage가
> 그 사이의 자유공간을 보장하지 않는다. 빔 사이·가림·높이·센서 최소 검출 크기와 방향별
> support를 고려해야 하고, 스칼라 축약이 항상 보수적으로 틀린다는 보장은 없다.

**당시 (b)를 권한 이유.**

- **Frenet과 자연스럽게 맞는다.** 결과가 차로별 스칼라 `s_observed` 하나다. 판제는
  "제동거리가 관측거리를 넘지 않도록" 속도를 제한하면 된다. 고전적인 *assured clear distance*
  원칙이고, 판제 변경은 속도 상한 계산 한 곳이다.
- range-image 최대 return을 차로에 투영하는 구상은 간단하지만 중간 free-space의 증명이
  아니다. 가림·빔 사이·검출 크기 검증이 추가로 필요하다(2026-09-27 정정).
- 스칼라 과대 추정은 안전하지 않다. 격자보다 항상 강건하거나 보수적이라는 종전 주장은
  철회한다. 검증된 연속 관측 구간을 어떻게 축약할지 비교해야 한다.
- **정적 장애물 표현과 독립이다.** 6.4의 정적 객체 발행과 겹치지 않고 보완한다.

메시지는 기존 `asmc_msgs` 스타일로 작게 잡는다.

```text
# LaneObservationHorizon.msg (제안)
std_msgs/Header header        # frame_id = map
string[]  lane_id             # 아래 배열과 평행
float64[] s_observed_m        # 해당 차로에서 가림 없이 관측된 최대 route station
float64[] confidence          # 0~1. 점 밀도 기반
```

**이것은 신규 계약이므로 [`src/interfaces/`](../../../src/interfaces/README.md)에서 합의한다.**
다만 6.0.3의 1번은 합의 없이 우회할 방법이 없는 항목이라, 이번 전환에서 반드시 논의 대상에
올려야 한다.

#### 6.0.6 GT 없이 주행할 때만 드러나는 두 가지 (LiDAR 측 필수)

**(1) 객체 입력 timeout 0.30 s — 15 Hz 기준 4.5 프레임 여유뿐이다.**

`src/frenet_planner/config/params.yaml`

```yaml
runtime:
  timeouts:
    vehicle_state: 0.50
    objects: 0.30        # <- LiDAR 15 Hz = 66.7 ms 주기, 약 4.5 프레임
    behavior: 0.50
```

입력이 stale이면 판제는 **정지 궤적을 발행한다**(`frenet_planner_node.cpp:152-157`,
`v2_planning.cpp:288-312`). GT 어댑터는 MORAI가 보낼 때마다 발행하므로 stale이 사실상
발생하지 않지만, **LiDAR는 다르다.**

- `LidarProcess()`는 점군이 비었거나 타임스탬프가 직전과 같으면 **조기 반환**한다
- `PlannerObjectRosPublisher::Publish()`는 `stamp == last_stamp_`이면 **발행하지 않는다**
- 즉 UDP 유실, 중복 프레임, WSL 스케줄링 지터가 그대로 미발행으로 이어진다

> **2026-09-26 정정 — 조치 L1:** 새 유효 cloud를 처리해 객체가 없을 때만 해당 **관측
> stamp**의 빈 배열을 발행한다. 센서 미수신·invalid cloud·처리 실패를 새 stamp의 빈 배열로
> 바꾸지 않는다. heartbeat가 필요하면 별도 health/diagnostics로 last-observation age와
> 실패 이유를 발행한다. 기존 객체의 stamp를 반복 갱신해 timeout을 피하는 방안은 채택하지 않는다.

> **2026-09-26 정정 — 조치 L2:** 25 ms는 제안 목표이고 0.30 s는 당시 consumer timeout이다.
> 알고리즘 시간·scan age·queue 지연·발행 간격은 다른 값이므로 end-to-end p95/p99와
> consumer age를 따로 측정한다. 타 논문 점 수를 나눈 M-detector 약14 ms 추정 등을
> 예산이 충족됐다는 근거로 사용하지 않는다.

**(2) 객체 위치에 지연 보상이 없다.**

`message_converter.cpp`는 객체를 `header.stamp` 시점 값 그대로 쓰고 현재 시각으로
전방 예측하지 않는다. 의도된 설계이며(“`now()`로 유효성을 위장하지 않는다”) 타임스탬프
규율로서는 옳다. 문제는 GT와 LiDAR의 지연이 다르다는 점이다.

| | 지연 | 100 km/h에서의 위치 오차 |
|---|---|---|
| GT 어댑터 | 거의 0 | 무시 가능 |
| LiDAR | 스캔 주기 + 파이프라인 = **60 ~ 100 ms 추정** | **1.7 ~ 2.8 m** |

> **2026-09-26 정정 — 조치 L3:** publish-time 예측은 자동 적용하지 않는다. 관측 시각과
> state-estimate 시각, 예측 horizon·uncertainty·실측 여부를 구분하는 계약이 필요하다.
> OBB corner·center·속도도 같은 시각으로 변환해야 한다. 기존 message가 이를 표현하지
> 못하면 원본 관측 stamp를 유지하고 consumer와 보상 책임을 먼저 정한다.

이 세 항목의 현행 수락 조건은 0.7의 L1-B로 묶는다. GT 실행에서도 지연·누락 문제는
발생할 수 있으며, 센서 전환 실험에서 드러난 문제를 관측 freshness 위장으로 숨기지 않는다.

#### 6.0.7 통합 작업 순서

| 순서 | 작업 | 담당 | 판제 변경 |
|---|---|---|---|
| I1 | LiDAR가 `/perception/objects`에 `asmc_msgs/DetectedObjectArray` 발행 + **ID 유일성 보증** | LiDAR | 없음 |
| I2 | **P1** 변환 실패 완화 | 판제 | 필요 |
| I3 | 7.1의 품질 개선 (Phase 0 ~ 2) | LiDAR | 없음 |
| I4 | **P2** yaw_rate 유도 방식 변경 | 판제 | 필요 |
| I5 | **P3** ID 이탈 내성 | 판제 | 필요 |
| I6 | 관측 지평 (6.0.5 (b)) | 양쪽 | 필요, 계약 합의 |

> **2026-09-26 정정:** I1은 직렬화만의 문제가 아니다. frame·속도·yaw-rate·ID·관측
> 유효성 의미와 branch별 MD5가 일치하는 adapter 계약 및 replay를 먼저 통과해야 한다.
> 아래 I/P 순서는 과거 제안이며 현재 순서는 L1-A→L1-B→L2 이후다.

**I1을 먼저 하면 평가 기준선이 공짜로 생긴다.** GT 어댑터를 지우지 말고 남겨두면 같은 bag에서
정답값 대비 판제 거동을 직접 비교할 수 있다. 10장의 계측과 8.6의 결정 실험에 그대로 쓴다.
**대회 직전에 GT 어댑터를 launch에서 제거하는 것이 마지막 단계다.**

### 6.1 ROI를 파이프라인 맨 앞으로 (HD Map ∪ 카메라 BEV)

> **2026-09-26 수정:** 아래 단순 OR mask와 기대효과는 당시 가설이다. 이를 점 단위 hard
> 삭제식으로 바로 구현하지 않는다. 카메라 non-drivable hole은 바로 감지할 장애물일 수
> 있고, union도 두 입력이 함께 누락한 영역은 복구하지 못한다. 현재 설계는 0.4의
> boundary envelope·margin·support/age gate·unknown/fallback·도로 진입 객체 보존이다.

당시 이용 가능한 입력과 제안식을 비교 기록으로 남긴다.

| 재료 | 위치 | 형식 |
|---|---|---|
| 구 bake 래스터 도로 마스크 | 당시 `src/link_node/out/road_mask.png`; 현재 `src/asmc_hd_map/development/won_tae_link_node/out/road_mask.png` | 5 px/m = 0.2 m/cell, map frame; 최종 operational geometry와 구분 |
| 벡터 차로·주행영역 | `/hd_map/global/*` (`asmc_msgs/DrivableArea`, `Lane`) | map frame 폴리곤 |
| 카메라 BEV 주행가능확률 | `asmc_perception` 정본 `/perception/bev/camera/drivable_probability` | base_link, 0.1 m/cell |
| 차선 경계 | 현행 정본 `/perception/lane/boundary_family` | 음영구간 포함; legacy 개별 관측은 기본 OFF |

**초기 제안식 — 아래 식만으로 장애물 삭제 여부를 결정하지 않는다.**

```text
keep(p) = HDMap_drivable(p, margin)  OR  CameraBEV_drivable(p)  OR  NearFieldBand(p)
```

- **HD Map 항:** 매 프레임 map→body 2D 변환을 1회 계산한 뒤, 점당 곱셈 2회와 마스크
  인덱싱 1회. 2만 점 기준 0.3 ms 미만으로 추정한다.
- **카메라 BEV 항:** GPS 음영이나 맵 정합 실패 구간의 보험이다.
- **두 항은 반드시 합집합(OR)이다.** 교집합으로 두면 한쪽이 실패할 때 시야를 잃는다.
- **NearFieldBand:** 자차 반경 10 ~ 15 m는 마스크와 무관하게 무조건 통과시킨다.
  연석 위로 진입하는 보행자, 갓길에 걸친 차량을 놓치면 안 된다. **타협 불가 안전장치다.**

**기대 효과**

1. 고주로·터널 장면에서 도로 영역은 현재 ROI 박스의 25 ~ 40 % 수준으로 추정된다.
   RANSAC은 O(n)×iteration, Euclidean 클러스터링은 KdTree로 O(n log n)이므로 전처리
   3 ~ 4배 단축이 현실적 기대치다. **실측 필요.**
2. 더 중요한 부수 효과: **도로 영역만 남으면 단일 평면 RANSAC 가정이 비로소 성립한다.**
   터널 벽, 비탈면, 가드레일이 평면 후보에서 제외되므로 iteration을 늘려도 되고
   threshold를 0.3 m에서 0.12 m로 조일 수 있다.
3. 전봇대, 터널 벽, 도로 밖 기물이 **클러스터링 이전에** 제거되어 과병합과 ID 스위치가
   동시에 줄어든다.

**선행 검증 (필수).** ROI를 좁히기 전에 `road_mask`와 LiDAR의 정합 오차를 반드시 계측한다.
GPS 헤딩 오차 1도면 50 m 지점에서 0.9 m가 밀린다. 현재 `drivable_margin_m = 1.0`으로는
부족할 수 있다. `/lidar/debug_clustered_map`을 `road_mask` 위에 겹쳐 오차 분포를 먼저 본다.
**정합이 맞지 않는 상태에서의 ROI 축소는 장애물을 지우는 기능이 된다.**

### 6.2 지면 분할 교체

단일 평면 RANSAC을 **동심 구역 모델** 기반으로 바꾸는 것이 정석이다.

당시에는 **Patchwork++** (IROS 2022, KAIST)를 1순위로 제안했다. 현재 첫 추가 비교는
linefit이며 아래 설계 장점과 Patchwork++ 기존 결과를 동일 조건으로 재검증한다(0.8).

- 거리·방위별 구역마다 개별 평면을 적합 → 종/횡 경사, 언덕, 캔트 모두 대응
- **A-GLE**: 직전 프레임 결과로 파라미터를 자동 적응 (튜닝 노동 제거)
- **R-VPF**: 옹벽·연석처럼 지면이 층을 이룰 때 대응 → 가드레일·연석 오분류 감소
- **TGR**: 부분 미분할 복구 → **4.1의 "물체가 프레임마다 사라진다"에 직접 대응**
- **RNR**: 반사 허상 제거
- CPU 단일 스레드 실시간. C++ 구현과 ROS1 래퍼 존재. 학습 불필요

**논문 실측 근거 (Patchwork++ Table II, i7-7700K, SemanticKITTI seq 05, 64ch).**

| 방법 | 속도 | 비고 |
|---|---|---|
| LineFit | **58.96 Hz** | 가장 빠름 |
| Patchwork++ | **54.85 Hz** | TGR 제외 시 67.84 Hz |
| Patchwork | 43.97 Hz | |
| R-GPF | 35.30 Hz | |
| GPF | 29.72 Hz | |
| **RANSAC (현재 방식)** | **15.43 Hz** | **벤치마크 중 가장 느린 축** |
| CascadedSeg | 13.07 Hz | |

**2026-09-27 정정:** 표의 RANSAC은 인용 논문의 구현/64ch 조건이지 우리 실행 속도가 아니다.
Patchwork++로 정확도·속도가 함께 개선된다고 확정하지 않는다. L1-A에서는 낮은 장애물
보존과 보행자 보존의 상충이 있었다. 점 수 1/6으로 실행시간을 환산하지 않는다.

**대안:** `linefit_ground_segmentation`은 ROS1 네이티브이고 의존성이 더 가볍다. 위 표에서
가장 빠르며, 논문(Himmelsbach, IV 2010)의 핵심은 **국소 직선 적합으로 경사 지형을 다루는
것**이라 우리 문제(단일 평면 가정 붕괴)에 직접 대응한다. 통합 난이도를 먼저 낮추고 싶으면
이쪽을 먼저 시도한다.

**Patchwork++ 도입 전 임시 조치:** threshold 0.3 → 0.12, iteration 20 → 50, 그리고 ROI를
전방/중간/후방 3구역으로 나누어 각각 평면을 적합한다. 이것만으로도 밑단 손실이 크게 줄어든다.

### 6.3 클러스터링 교체

**1단계 (즉시).** yaml 값을 실제로 읽도록 연결하고 tolerance를 거리 함수로 바꾼다.

```text
tol(r) = clamp(0.25 + 0.02 * r, 0.30, 0.90)      r = 센서 거리 [m]
```

10 m에서 0.45 m, 50 m에서 0.90 m다. PCL의 `EuclideanClusterExtraction`은 가변 tolerance를
지원하지 않으므로 `pcl::ConditionalEuclideanClustering`으로 교체하면 조건 함수에 거리
의존식을 넣을 수 있다.

> **중요:** 이 거리 비례 방식은 새로운 아이디어가 아니다. **현재 OBB 코드가 채택한 바로 그
> 논문(Zhang et al., IV 2017)의 Algorithm 1이 이미 `r = a0 * ||X||`, 즉 센서 거리에 비례하는
> 탐색 반경을 규정한다.** 논문은 그 근거로 "레이저 스캔의 경도 해상도가 거리에 따라 커지기
> 때문"이라고 명시한다. 현재 코드는 같은 논문의 L-Shape 부분만 가져오고 세그멘테이션 부분은
> 고정 1.5 m로 두었다. **이 항목은 이미 채택한 논문의 나머지 절반을 완성하는 작업이다.**

**2단계 (본격). Range-image depth clustering을 1순위로 한다.**

| 후보 | 실측 (논문) | 판단 |
|---|---|---|
| **Range-image BFS** (Bogoslavskyi & Stachniss, IROS 2016) | **16-beam 2.4 ms** (모바일 i5-5200U) / **1.5 ms** (i7-4770K). 64-beam 8.6 / 4.7 ms | **1순위** |
| Curved-Voxel Clustering (CVC) | VLP-16 실데이터에서 **20 Hz 이상**, 합성 데이터 30 Hz 이상 (= 33 ~ 50 ms) | 후순위 |
| TRAVEL | **20 ~ 30 Hz** (= 33 ~ 50 ms, 기존 인용 조건) | 후순위 비교 후보; 우리 예산 초과는 미측정 |

**세 가지 근거로 range-image 방식을 택한다.**

1. **속도 후보 근거.** 인용 조건의 16-beam에서 2.4 ms다. i5-14600K의 1 ms 미만은
   미검증 추정이며 채택 근거로 쓰지 않는다. 논문은 PCL
   Euclidean 대비 **"세 자릿수 배 빠르다(4 ms vs 4 s per scan, 64-beam)"** 고 보고한다.
   현재 코드가 쓰는 것이 바로 그 PCL Euclidean이다.
2. **과병합 해소가 논문에서 직접 확인된다.** 논문 Fig. 8은 **VLP-16 실외 장면에서 서로 가까이
   서 있는 사람들을 올바르게 분리**하는 반면, 격자 기반 방식은 자전거들을 두 개의 큰 덩어리로
   병합한다고 보고한다. 4.3에서 지적한 증상과 정확히 같은 실패 모드다.
3. **KdTree가 필요 없다.** 16 x 1800 레인지 이미지에서 BFS만 돌린다. 빔이 적을수록 유리하다.

같은 논문의 파이프라인은 각도 기반 지면 제거도 포함하므로, 6.2와 6.3을 함께 대체하는
경로로도 검토할 수 있다.

**CVC와 TRAVEL은 후순위 비교 후보로 유지한다(2026-09-27 정정).** 종전에는 CVC의
L-CAS 실내 VLP-16 및 TRAVEL의 20~30 Hz 인용으로 25 ms 예산 초과라고 판단했으나,
입력·CPU·계측 범위가 달라 직접 비교할 수 없다. range-image도 우리 장면에서 검증한다.
TRAVEL은 두 단계를 함께 변경하므로 개별 알고리즘 평가 이후에 검토한다.

**C-ARC는 비반복 스캔(Livox Mid-360) 대상**이므로 반복 스캔인 VLP-16에는 직접 적용 대상이
아니다. 참고 자료로만 둔다.

### 6.4 정적·동적 이중 경로 아키텍처 (신규 제안)

현재는 모든 장애물이 "클러스터 → OBB → 트랙 → ID"라는 단일 경로를 통과한다. 그래서
클러스터링이나 ID 관리가 실패하면 **정적 장애물까지 같이 사라질 수 있다.** 다만 정적
객체에도 판제 대상 유지용 ID가 필요할 수 있고 정적 분류가 관측 속도 0의 확정은 아니다.

**제안: 경로를 둘로 나눈다.**

```text
비지면 점군 (주행가능영역 내부)
  ├── 동적 경로:  클러스터링 → L-Shape OBB → 트래킹 → TrackedObject (ID, 속도, 클래스)
  └── 정적 경로:  BEV 점유 격자 (ID 없음, 속도 없음, 클래스 없음)
```

**정적 경로 사양 제안**

- `nav_msgs/OccupancyGrid`, map frame 또는 base_link, 0.2 m/cell
- 100 m × 40 m = 500 × 200 = 10만 셀. uint8이면 100 KB. 갱신 비용 1 ms 미만으로 추정
- 셀마다 **최대 높이(max-z)** 를 함께 유지하면 높이 정보가 클러스터링과 무관하게 보존된다
  → 4.1의 "높이를 못 잡는다" 증상이 정적 물체에 대해서는 구조적으로 해소된다
- 자차 운동 보상 후 N프레임 누적 + 시간 감쇠(evidence decay). 희소한 VLP-16에서 특히 효과적
- 동적으로 확정된 트랙이 점유한 셀은 정적 격자에서 제외한다

**이 구조의 이점**

- 공사 구조물, 파편, 비정형 장애물처럼 **클래스가 없는 물체를 클러스터링 성패와 무관하게**
  판제에 전달한다
- Frenet planner가 격자를 직접 샘플링해 충돌 검사할 수 있어 소비 측 로직이 단순해진다
- 정적 물체의 깜빡임(ID 소실로 인한 소멸)이 원리적으로 사라진다
- 학습 불필요, 추가 의존성 없음

**먼저 할 것: 계약 안에서 해결하는 대안 (판제 수정 0).**

점유 격자는 신규 메시지를 요구하므로 판제 작업이 따라온다. 그 전에 **같은 목적을 기존 계약
안에서 달성할 수 있다.**

> **2026-09-27 철회:** 정적/UNKNOWN이라는 이유만으로 `vx=vy=0`, `type=3`을 강제하지
> 않는다. 기존 STATIC_OBSTACLE 분기와 OBB 충돌 검사는 재사용 가능하지만 motion 근거와
> 속도 유효성을 구분해야 한다. 의미 불명확한 속도는 unknown이며 현재 wire/consumer에
> 이를 전달할 계약을 합의하기 전에는 운영 발행하지 않는다.

"정적 장애물이 사라진다"는 증상의 실제 원인은 4.1의 지면제거·높이컷이지 표현 형식이 아니다.
**그 원인을 고치고 정적 객체를 버리지 않고 내보내는 것만으로 대부분 해결된다.**

**다만 두 가지를 반드시 지킨다.**

1. **개수 상한.** ROS 발행 경로의 `map_output_max_objects` 기본값은 200이지만 UDP 경로는
   프로토콜상 20이 상한이다. 정적 클러스터를 모두 내보내면 상한을 넘긴다. **자차 경로
   기준 거리순으로 자르고**, 잘린 개수를 진단 로그로 남긴다.
2. **발행 전 검증.** 6.0.3에서 적었듯 변환기는 객체 하나가 잘못되면 프레임 전체를 버리고
   판제가 급정지한다. 치수 0 이하, 비유한값, **중복 ID**가 없는지 발행 직전에 확인한다.
   정적 객체를 많이 내보낼수록 이 위험이 커진다. **판제 측 P1(변환 실패 완화)이 들어가기
   전까지는 정적 객체 발행 개수를 특히 보수적으로 잡는다.**
3. **가림은 이 방법으로 해결되지 않는다.** 정적 객체를 다 내보내도 "보이지 않는 곳"은
   여전히 표현되지 않는다. 그 부분은 6.0.5의 관측 지평이 담당한다. 두 기능은 겹치지 않는다.

**점유 격자는 위 대안의 한계가 실제로 확인된 뒤에 다시 꺼낸다.** 상한 20~200을 넘는 비정형
장애물이 실제로 문제가 되거나, 박스로 표현하면 과도하게 부풀려지는 구조물(연속 방호벽,
공사 구간)이 반복해서 관측될 때가 그 시점이다. 그때
[`src/interfaces/`](../../../src/interfaces/README.md)에서 합의한다.

### 6.5 트래킹·속도 재설계

판제 보완로직을 제거하기 위한 핵심 항목이다.

**변경 1의 과거 제안: 추적을 map/ENU 프레임에서 수행한다.**

> **2026-09-26 대체:** map tracking만으로 정지 물체 속도가 정확히0이 되거나 보완 계층이
> 불필요해지는 것은 아니다. pose 시간 오차·GPS 복귀 jump·검출 geometry 변화가 남는다.
> 현재 제안은 0.3의 연속 local/odom 추적과 선택적 fresh-map projection이며, no-GPS
> wheel/IMU 품질 검증이 선행이다. 아래 map/ENU·body fallback은 당시의 비교안으로 보존한다.

`Body2ENU`, `TransformLidarPointToMap`이 이미 구현되어 있다. 관측을 map 좌표로 올린 뒤
Kalman을 돌리면

- 유효한 관측 시각 pose가 있으면 자차 운동을 보상할 수 있지만 측위/시간/검출 오차가 남는다.
- 곡선·회전 항 누락을 줄여도 가드레일 등의 가짜 동적 판정이 자동 소멸하지 않는다.
- 이미 절대속도로 추정한 값에는 CAN을 다시 더하지 않는다. frame 이름만으로 속도의
  의미가 확정되지는 않는다.
- `MotionClassifier`/판제 보완 제거는 실제 회귀가 확보된 뒤 판단한다.

**전제 조건:** GPS/DR 품질. `DeadReckoning.cpp`가 36줄뿐이라 음영구간 드리프트가 관건이다.
**하이브리드를 권한다.** 연관(association)은 body frame에서, 상태 추정은 map frame에서
수행하고, 측위 품질이 임계 이하로 떨어지면 body frame 속도로 폴백하는 게이트를 둔다.

**변경 2: 관측점을 centroid에서 안정 앵커로 바꾼다.**

- **L자의 가장 가까운 가시 코너를 앵커**로 Kalman에 넣는다. 가림이 바뀌어도 코너는 실제로
  관측되는 점이라 튀지 않는다
- 클래스가 `car`로 확정되면 **L, W를 고정(size lock)** 하고 박스를 코너에 맞춘다. 중심이
  기하학적으로 결정되어 jitter가 사라진다
- 가시 코너가 바뀔 때는 **앵커 스위칭을 명시적으로 처리**한다(박스 치수만큼의 오프셋 점프를
  보상). 이 처리가 없으면 오히려 더 나빠진다

**근거 (L-Shape Fitting-Based Vehicle Pose Estimation, IEEE T-IV 2021).** 논문은 같은 문제를
이렇게 진술한다. "가림과 시점 변화 때문에 LiDAR 점은 스캔마다 안정적이지 않다. 그래서 우리는
**차량의 가장 가까운 코너를 추적하며, 이쪽이 더 안정적이다.**" 우리가 관측점으로 쓰는 클러스터
centroid가 바로 그 불안정한 값이다.

**추가 개선: OBB 각도 탐색 범위를 사전정보로 좁힌다. (신규)**

Zhang 논문은 명시적으로 **"탐색 공간은 추적 시스템이나 비전 장면 이해가 뒷받침되면 줄일 수
있다"** 고 적는다. 현재 코드는 매 클러스터마다 0 ~ 90도를 1도 간격으로 90회 전수 탐색한다.

```text
현재:  theta in [0, 90) deg, 1 deg step        -> 90 회
개선:  중심 = 직전 프레임 yaw (ego yaw 보정 후) 또는 HD Map 차로 heading
       theta in [중심 -15, 중심 +15] deg       -> 30 회
```

- **연산:** OBB 단계 약 3배 단축
- **안정성:** 90도 축 전환과 프레임 간 yaw 점프가 구조적으로 줄어든다. 현재의 yaw EMA
  (`kYawFilterAlpha = 0.25`)로 사후 보정하는 것보다 근본적이다
- **HD Map 자산을 그대로 쓴다.** 도로 위 차량의 heading은 차로 heading에서 크게 벗어나지
  않는다. 신규 트랙(직전 yaw 없음)에는 차로 heading을, 기존 트랙에는 직전 yaw를 중심으로 쓴다
- **폴백 필수:** 차로를 벗어난 물체나 교차로 진입 차량을 위해, 좁힌 범위에서의 최고 점수가
  임계 이하이면 전수 탐색으로 되돌린다

**변경 3: 운동 모델 고도화는 우선순위를 낮춘다. (문헌 검증 결과 하향 조정)**

당초 CTRA/IMM 복원을 중간 우선순위로 제안했으나, SimpleTrack의 실측이 이를 지지하지 않는다.

| 데이터셋 | KF | CV | 결론 |
|---|---|---|---|
| Waymo (차량 MOTA) | 0.5612 | 0.5515 | 차이 미미 |
| nuScenes (AMOTA) | 0.687 | 0.690 | 차이 미미 |
| nuScenes 10 Hz 설정 | 0.696 | 0.693 | 고주파에서 KF가 근소 우위 |

논문 결론은 **"두 운동 모델은 대체로 비슷한 성능을 보인다"** 이며, KF의 우위는 주로 박스
refinement에서 오고 **고주파 데이터일수록 KF가 적합**하다는 것이다. 우리는 15 Hz로 고주파
쪽에 속하므로 **현재의 CA Kalman으로 충분하며, CTRA/IMM은 과공학일 가능성이 높다.**

**대신 변경 2(앵커), 변경 4(R), 변경 5(연관·생애주기)에 자원을 집중한다.** SimpleTrack의
실험은 "좋은 검출 박스"를 전제하는데, 우리 문제는 그 전제 자체가 깨져 있다. 즉 우리에게
부족한 것은 운동 모델의 표현력이 아니라 **측정값의 품질과 연관·생애주기 관리**다.

CTRA/IMM은 변경 2, 4, 5를 적용하고도 heading이 불안정할 때 재검토한다.

**변경 4: 측정 노이즈를 거리 의존으로 바꾼다.**

```text
R = diag(sigma_r^2, sigma_t^2)
sigma_r = 0.05 + 0.01 * r     반경 방향
sigma_t = 0.05 + 0.03 * r     접선 방향 (가림 영향이 크다)
```

60 m 트럭이면 0.65 m / 1.85 m다. 현재 0.1 m 고정보다 훨씬 정직하고, centroid 점프가
속도로 전환되지 않는다.

**변경 5: 연관과 생애주기를 정리한다. (문헌 검증으로 내용 수정)**

**(a) 현재의 비용 함수 + Hungarian 조합이 SimpleTrack이 지목한 최악 조합이다.**

현재 `Global.cpp`의 가중치는 `COST_W_DIST = 1.0`, `COST_W_IOU = 1.0`, `COST_THRESHOLD = 3.0`
이다. IoU 비용은 [0, 1]로 유계인 반면 거리 비용은 미터 단위로 0 ~ 3까지 가므로 **사실상
거리가 비용을 지배한다.** 그런데 SimpleTrack은 다음을 실측으로 보인다.

> IoU 계열 지표는 Hungarian과 greedy 모두에서 잘 동작하지만, **거리 기반 지표는 greedy를
> 선호한다.** 거리 지표는 값의 범위가 커서 전역 최적을 구하는 Hungarian이 이상치에 악영향을
> 받기 때문이다.

→ **조치:** 비용을 **BEV GIoU 중심**으로 바꾸면 Hungarian을 그대로 유지할 수 있다.
GIoU는 겹침이 0일 때도(트랙 초기, 급격한 운동) 값이 정의되므로 IoU의 실패 모드와 거리
지표의 실패 모드를 동시에 피한다. 논문 권장 게이트는 `GIoU > -0.5`다.

**(b) ID 스위치의 원인은 연관이 아니라 조기 종료다.**

SimpleTrack이 Waymo에서 모든 ID 스위치를 분류한 결과는 다음과 같다.

| 원인 | 차량 | 보행자 |
|---|---|---|
| **조기 종료 (early termination)** | **95 %** | **91 %** |
| 잘못된 연관 | 5 % | 9 % |

원인은 점군 희소성과 가림이다. **VLP-16은 KITTI/Waymo보다 훨씬 희소하므로 우리 쪽에서 더
심할 것으로 본다.**

> **이전 판 권고 철회.** 초판에서 "유령 수명을 25프레임에서 10프레임으로 줄인다"고 적었으나
> **이는 잘못된 방향이다.** 첫째, 위 데이터가 정반대를 가리킨다. 둘째, `MakePlannerObjects()`는
> 이미 `s32_MissCount != 0`인 트랙을 판제로 내보내지 않으므로 **유령 수명이 길어도 판제에
> 노출되는 비용이 없다.** 25프레임 유지는 유지하거나 오히려 늘린다.

**(c) 2단계 연관을 클러스터링 이중 임계로 구현한다.**

SimpleTrack의 2단계 연관은 "저신뢰 검출이라도 운동 예측과 일치하면 트랙 수명을 연장한다"는
것이다. 우리 파이프라인에는 검출 신뢰도가 없으므로 **클러스터링을 두 임계로 두 번 돌려
대체한다.**

```text
1단계: 엄격 클러스터  (MinClusterSize 10 이상)  -> 신규 트랙 생성 + 주 연관
2단계: 관대 클러스터  (MinClusterSize 3 ~ 5)    -> 1단계에서 미매칭된 트랙만 구제 연관
                                                   (신규 트랙 생성 금지)
```

두 임계 비교는 조기 종료를 줄이는 후보지만, 논문의 2.4 ms를 두 배 해 **5 ms 미만이라고
보장하지 않는다**(2026-09-27 정정). 점 공유/중복 검출·연관 비용까지 실제로 측정한다.

**(d) 생애주기에 tentative / confirmed 상태를 도입한다.** 현재는 미매칭 검출에 즉시 새 ID를
발급한다. M-of-N(예: 3/5) 확정 전에는 판제로 내보내지 않는다. **이것은 (b)의 수명 연장과
모순되지 않는다.** 트랙 생성은 보수적으로, 트랙 종료는 관대하게 가는 것이 맞는 방향이다.

**(e) 구현체 도입은 신중하게. (문헌 검증으로 추가)**

| 구현 | 실측 속도 | 판단 |
|---|---|---|
| **AB3DMOT** | **207.4 FPS** (KITTI, GPU 불필요) | 가장 저렴한 출발점 |
| **Fast-Poly** | **34.2 FPS** (nuScenes, CPU 전용), 75.8 % AMOTA | 실행 가능한 상위 대안 |
| Poly-MOT | **3 FPS** (기존 인용 조건) | 지역 평가 미실행; 이 수치만으로 제외하지 않음 |
| SimpleTrack | **0.51 FPS** (기존 인용 조건) | 연관·생애주기 참고; 지역 성능은 미측정 |

**2026-09-27 정정:** SimpleTrack의 연관·생애주기를 우선 참고하되 타 데이터셋 FPS로 코드
사용 불가 판정을 내리지 않는다. 후보별 동일 box sequence에서 계측한다. C++ 재구현도
비용이 없지 않으며 AB3DMOT는 비상업 연구 license, FastPoly는 별도 LICENSE 확인이 필요하다.

### 6.6 카메라 퓨전 보강

**현재 방식의 약점.** `src/Main/src/SensorFusion/FusionCamera.cpp`

```cpp
float32 width  = (...) * 1.3f;    // LiDAR 투영 박스를 임의로 확대
float32 height = (...) * 1.5f;
if (iou > FUSION_IOU_THRESHOLD)   // 0.25
```

- 정의가 다른 두 박스(YOLO의 가시 실루엣 vs LiDAR 클러스터 투영)의 IoU는 거리에 따라 크게
  변한다. 1.3 / 1.5 스케일은 그것을 상수로 땜질한 것이다
- 원거리 클러스터는 투영 박스가 작아 IoU가 구조적으로 낮다 → **먼 보행자가 확정되지 않는다**
- 그리디 매칭이고, YOLO confidence를 사용하지 않는다

**개선안**

1. **IoU 대신 point-in-box ratio(프러스텀 연관).** LiDAR 클러스터 점 중 YOLO 박스 안에
   투영되는 비율을 쓴다. 거리 불변이고 스케일 상수가 사라진다. 비용은 `1 - ratio`
2. **Hungarian + confidence 가중.** `Yolo_bbox.msg`에 `float32 conf`를 추가하고
   (4.4의 결함), `conf_thres`를 0.7에서 0.35로 낮춘 뒤 신뢰도를 비용에 반영한다
3. **클래스를 내부적으로 확률로 유지하되, 판제에는 기존 `type` 정수로 내보낸다.**
   트랙마다 `p(car)`, `p(person)`, `p(unknown)`을 혼동행렬 기반 베이즈로 갱신한다. 현재의
   `continuity_counter` + `is_class_locked`는 이산적이라 진동한다.
   **다만 `asmc_msgs/DetectedObject`에는 확률 필드가 없고 `uint8 type`뿐이다.**
   스키마를 바꾸면 판제 수정이 따라오므로, **확률은 LiDAR 내부에만 두고 발행 시점에
   argmax를 `type`으로 변환한다.** 확률이 낮으면 `type = 3`(기타)로 보수적으로 떨어뜨린다.
   판제 수정 0이면서 진동은 사라진다. 확률을 판제가 직접 쓰고 싶어질 때 스키마 확장을
   논의한다
4. **CLOCs 구조 채택** — 6.8의 3D 검출기를 도입하는 경우. 2D·3D 후보를 NMS 이전에 희소
   텐서로 합치고 작은 2D CNN으로 일관성을 학습한다. 프레임당 3 ms 미만 추가
5. **동/정적 판정의 독립 근거 추가.** 연속 스캔을 자차 운동 보상 후 비교해 점 단위로
   동/정적을 판정하는 MOS(Moving Object Segmentation) 계열을 둔다. 트래킹 속도와 **독립된
   증거**라 `MotionClassifier`의 휴리스틱을 대체할 수 있다. 학습 없이 쓸 수 있는 구현은
   7.1 표의 M-detector를 참조한다

### 6.7 고주로 차로별 점유와 추월

> **2026-09-26 보완:** 중심점 point-in-polygon만으로 차로를 확정하지 않는다. 0.5의
> footprint overlap·복수 후보·hysteresis를 적용하고, 실제 판제 feature와 중복 여부를
> 먼저 확인한다. 후방 ROI의 숫자도 검출거리·상대속도·센서 가용성을 측정한 뒤 정한다.

아래는 당시의 간단한 연계안과 계약 검토 기록이다.

**먼저 할 것: 이 레이어는 LiDAR가 아니라 판제 쪽에 두는 것이 맞을 수 있다 (판제 수정 0).**

차로 배정에 필요한 입력은 **map frame 객체 + HD Map** 둘뿐이다. 그런데 `behavior_planner`는
이미 `/perception/objects`(map frame)를 구독하고 HD Map도 갖고 있으며,
`feature/object_projector.cpp`와 `feature_builder.cpp`에서 **객체를 차로에 투영하고 gap을
계산하는 로직을 이미 수행한다.** 즉 아래 `LaneOccupancy`를 LiDAR 패키지가 새로 만들면
**기능이 중복되고 신규 메시지 계약까지 늘어난다.**

> **권고: LiDAR 패키지는 `/perception/objects`의 품질(위치·속도·치수·ID 연속성)을 올리는 데
> 집중하고, 차로 점유·gap·TTC는 판제의 기존 feature 레이어에서 계산한다.**
> LiDAR 쪽이 추가로 할 일은 **고주로용 ROI 프로파일(후방 −60 m)** 하나뿐이며 이건 판제
> 수정이 0이다.

아래 메시지 설계는 **판제 파트가 자기 쪽에 두기로 결정할 경우의 참고안**으로 남긴다.
LiDAR 패키지가 이걸 발행하기로 먼저 정하지 않는다.

**참고안: `LaneOccupancy`**

```text
int8    lane_offset          # 0 = 자차로, -1 = 좌, +1 = 우
string  lane_id              # asmc_msgs/Lane.id
float64 s_ego_m              # 자차 route station

int32   lead_track_id
float64 lead_gap_m           # Frenet s 방향 간격
float64 lead_speed_mps       # 절대 종방향 속도
float64 lead_ttc_s

int32   rear_track_id
float64 rear_gap_m
float64 rear_speed_mps
float64 rear_ttc_s

bool    gap_feasible         # 차선변경 가능 1차 판정
float64 occlusion_ratio      # 해당 차로 중 미관측 비율
```

**구현 경로 (기존 자산만 사용)**

1. `MakePlannerObjects()`가 이미 `obb_center_map`과 `velocity_map_mps`를 만든다
2. `/hd_map/global/*`의 `Lane.polygon` / `Lane.centerline` + `lane_number` +
   `route_station_start_m`로 **point-in-polygon 1회**로 차로를 배정한다.
   트랙 20개 × 차로 4개 = 80회, 비용 무시 가능
3. 차로 중심선으로 Frenet (s, d) 변환 → 종방향 간격과 상대속도를 직접 얻는다.
   유클리드 거리보다 판제 친화적이다
4. `Lane.left_adjacent_lane_id` / `right_adjacent_lane_id`로 인접 차로 후보를 뽑고
   각 차로의 (lead, rear) gap과 TTC를 계산한다

**ROI 프로파일을 고주로 전용으로 분리한다.** 현재 `x [-30, 50]`인데 **추월 판단에는 후방이
더 중요하다.** 뒤에서 100 km/h로 접근하는 차량이 -30 m에서 처음 보이면 약 1.1초 뒤 충돌
지점이다. 고주로 모드에서는 **`x [-60, 80]`, 횡방향은 자차로 기준 ±2차로(약 ±7 m)** 로
잡는다. **면적은 오히려 줄면서 유효 시야는 넓어진다.** 6.1의 맵 기반 ROI가 주는 가장 실용적인
이득이다.

### 6.8 딥러닝 3D 검출기 도입 판단

**결론부터: 기하 파이프라인을 DNN으로 대체하지 않는다. 병렬 트랙으로 개발해 late fusion으로
덧붙인다.** 학습 데이터 분포 밖(공사 구조물, 특이 장애물)에서 DNN은 조용히 실패하고,
그 실패는 6.4의 정적 점유 격자가 없으면 그대로 사고가 된다.

**후보 비교 (VLP-16 단일 스윕 약 2만 점 기준)**

| 후보 | 적합성 | 논문 실측 근거 |
|---|---|---|
| **PointPillars + TensorRT** | 1순위 | 논문 실측 **62 Hz**(기본) / **105 Hz**(경량), i7 + GTX 1080 Ti, KITTI 64ch. CUDA 복셀화 + TensorRT 백본 + CUDA 후처리가 완제품으로 공개. **SPConv 불필요** |
| **CenterPoint-Pillar** | 1순위 동급 | 앵커 프리라 방향 추정이 안정적. 논문 전체 시스템 11 ~ 16 FPS(Waymo/nuScenes). **속도 헤드는 아래 주의사항 참조** |
| FastPillars | 여유 있을 때 | SPConv 없이 TensorRT 친화. **MAPE 모듈이 소형 3D 객체(보행자) 강화** 목적으로 설계됨 |
| PillarNeXt | 참고 | Waymo, TITAN RTX 실측: Pillar-S **79 ms** vs Voxel-S **169 ms**, 정확도는 동급(3D APH 63.18 vs 64.67, 보행자 BEV는 Pillar 우위). 대범위 Waymo 설정 수치이므로 우리 설정에 그대로 대입하지 않는다 |
| BEVFusion / VoxelNeXt | 후순위 | 8 GB 공유 환경에서 자원·의존성 검증 필요; 실행하지 않고 무조건 불가로 판정하지 않음 |

**SPConv 회피가 Pillar 계열을 택하는 핵심 이유다.** FastPillars 논문은 SPConv의 배포 문제를
세 가지로 정리한다. (1) TensorRT 내장 연산이 아니라 커스텀 플러그인을 직접 써야 하고,
(2) 양자화 배포가 까다로우며, (3) 최신 하드웨어에서 최적화가 어렵다. **CenterPoint의 표준
voxel 백본은 SPConv를 쓰므로, 반드시 Pillar 변형을 선택해야 한다.**

> **2026-09-27 재정정:** 특정 시간 입력 구성을 모든 CenterPoint 구현의 필수 조건으로
> 일반화했던 설명은 잘못이다. 고정 upstream의 `center_head.py`는 `forward(x)`와 설정별
> velocity branch, loader는 sweep/time-lag를 사용한다. 단일 스윕 라벨 회귀의 구성 가능성과
> 운동의 관측 가능성은 별개다. Capture 시계열·속도 label 품질을 확인하기 전 학습을 승인하지 않는다.

**연산 예산 산정**

- Pillar 격자를 `x [-60, 80] m`, `y [-20, 20] m`, 0.2 m로 잡으면 700 × 200 = 14만 pillar
- sparse point/pillar 전처리와 dense BEV backbone 비용을 구분한다. 점이 적어도 격자 비용은 남는다.
- 과거 **10~20 ms, VRAM 2 GB 이내** 추정은 미측정 예산 가설이다. 실제 15 Hz 예산 충족이나
  다른 모델의 8 GB 초과를 이 숫자로 판정하지 않는다.

**정직한 한계.** VLP-16 단일 스윕으로 50 m 밖 차량에 맺히는 점은 수십 개 수준이다.
DNN 검출의 신뢰 구간은 현실적으로 **차량 40 ~ 50 m, 보행자 25 ~ 30 m** 정도로 잡아야 한다.
그 밖은 기하 파이프라인 + HD Map + 트래킹 예측이 담당한다. **원거리 성능을 DNN에 기대하고
ROI를 넓히는 설계는 하지 않는다.**

---

## 7. 학습 필요 여부별 정리

가장 중요한 분류다. **학습 없이 즉시 적용 가능한 항목만으로도 4장의 증상 대부분이 해소된다.**

### 7.1 학습 불필요 — 우선 적용

이 표는 **2026-09-21의 후보 우선순위 기록**이다. 실측 열은 인용 논문의 조건이며
우리 runtime의 측정값이 아니다. 새 timestamp heartbeat, 즉시 message 필드 추가,
무조건 map 추적, hard semantic ROI 권고는 앞 절의 2026-09-26 정정으로 대체됐다.
현재 구현 순서는 0.7이며 판제 수정·message 호환 비용은 L1-B에서 재확인한다.

| # | 항목 | 대응 증상 | 난이도 | 실측·비고 |
|---|---|---|---|---|
| **0** | **`/perception/objects`에 `asmc_msgs/DetectedObjectArray` 발행** | 통합 | **낮음** | **판제 수정 0.** `MakePlannerObjects()`가 필요한 값을 이미 전부 보유. GT 어댑터와 A/B 가능 (6.0) |
| **L1** | 별도 health + 유효 관측만 원래 stamp로 발행 | GT 전환 | 계약 검토 | 새 stamp 빈 배열 heartbeat 권고 철회; 6.0.6 정정 |
| **L3** | 관측 시각/예측 시각 분리 검토 | GT 전환 | 계약 검토 | publish-time 재표기를 단순 수정으로 권하지 않음; 6.0.6 정정 |
| 1 | 거리 적응 클러스터 tolerance + yaml 배선 | 4.3 | **매우 낮음** | **Zhang 논문 Alg.1이 이미 거리 비례 탐색 반경을 규정.** 코드가 채택한 논문의 미구현 절반 |
| 2 | RANSAC threshold/iteration 조정, 3구역 분할 | 4.1 | **매우 낮음** | 상수 변경 |
| 3 | `f32_CoreZ <= -1.3` 하드컷 제거 | 4.1 | **매우 낮음** | 지면평면 상대 높이 기준으로 대체 |
| 4 | `UpdateAABB`의 Core 덮어쓰기 제거 | 4.4 | **매우 낮음** | 위치·속도 추정기 일치 |
| 5 | intensity 복사, roll 보정 | 4.4 | **매우 낮음** | |
| 6 | YOLO confidence 전달, `conf_thres` 0.7 → 0.35 | 4.4 | **낮음** | msg 필드 추가 |
| 7 | 거리 의존 R | 4.2 | **낮음** | |
| 8 | **OBB 각도 탐색 범위 축소** (직전 yaw / 차로 heading ±15도) | 4.2, 연산량 | **낮음** | **Zhang 논문이 명시적으로 제안**. 90회 → 30회 |
| 9 | HD Map ∪ 카메라 BEV 점 단위 ROI | 4.3, 연산량 | 낮음 | 자산 전부 보유. 맵 prior 전/배경 분리는 3D PC survey가 뒷받침 |
| 10 | **연관 비용을 BEV GIoU로 교체** | 4.3 | 낮음 | **현재의 거리지배 비용 + Hungarian이 SimpleTrack이 지목한 최악 조합** |
| 11 | **Patchwork++** 지면 분할 | 4.1 | 중간 | **54.85 Hz** (i7-7700K, 64ch). 현재 RANSAC은 **15.43 Hz** |
| 12 | **Range-image BFS 클러스터링** | 4.3, 연산량 | 중간 | **16-beam 2.4 ms**(모바일 i5). PCL Euclidean 대비 세 자릿수 배 |
| 13 | **클러스터링 이중 임계 2단계 연관** | 4.3 | 중간 | 95%는 인용 데이터 조건. 5 ms 미만·우리 ID 개선 모두 실측 필요 |
| 14 | L-Shape 코너 앵커 + size lock + 앵커 스위칭 | 4.1, 4.2 | 중간 | T-IV 2021이 "코너가 더 안정적"이라고 명시 |
| 15 | map frame 추적 (하이브리드 폴백 포함) | 4.2 | 중간 | `Body2ENU` 기보유 |
| 16 | tentative / confirmed 생애주기 | 4.3 | 중간 | 생성은 보수적으로, 종료는 관대하게 |
| 17 | point-in-box 프러스텀 연관 + 클래스 확률 | 4.4 | 중간 | |
| 18 | 정적/UNKNOWN의 geometry·속도 유효성 보존 | 4.1, 4.3 | 계약 검토 | 0 강제 권고 철회. unknown 인계와 consumer 호환을 먼저 합의(6.4) |
| 19 | **M-detector** 동적 점 검출 | 4.2 | 중간 | **학습 불필요.** KITTI 인용 84.5 ms. VLP-16 약14 ms 비례 환산 철회; pose/time 전제 후 계측 |
| 20 | 고주로 전용 ROI 프로파일 (후방 −60 m) | 고주로 추월 | 낮음 | **판제 수정 0.** 차로 점유·gap 계산은 판제의 기존 feature 레이어에 둔다 (6.7) |
| - | ~~CTRA/IMM 운동 모델~~ | - | - | **하향 조정.** SimpleTrack 실측상 KF와 CV 차이 미미 (6.5 변경 3) |
| - | TRAVEL | - | - | 후순위 비교. 20~30 Hz 타 환경 수치로 제외하지 않음(6.3 정정) |
| - | ~~Curved-Voxel Clustering~~ | - | - | **후순위.** VLP-16에서 20 ~ 30 Hz, range-image보다 한 자릿수 느림 |
| - | ~~BEV 점유 격자 / `LaneOccupancy` / 클래스 확률 스키마~~ | - | - | **판제 수정 필요.** 각 절의 계약 안 대안을 먼저 적용하고, 한계 확인 후 재논의 |

**19번 M-detector를 특히 권한다.** MOS 계열(LiDAR-MOS, 4DMOS, MotionBEV)은 전부 학습이
필요한데, M-detector는 occlusion 원리 기반이라 **학습 없이** 점 단위 동/정적 판정을 준다.
논문이 강조하는 두 번째 장점이 우리에게 더 중요하다.

> M-detector는 장면에서 **실제로 움직이는** 것에만 반응하므로, 단지 **움직일 수 있는**
> 객체가 아니라 진짜 움직이는 객체의 점을 검출한다. 반면 학습 기반 방법은 움직일 수 있는
> 객체의 점을 쉽게 검출해 버린다.

주차 차량, 정차 중인 버스를 정적으로 유지해야 하는 우리 상황에 정확히 맞는 성질이다.
다만 **공짜가 아니다.** 논문의 KITTI seq 00 실측이 프레임당 84.48 ms(10 Hz 주기 100 ms 대비)
이다. 과거 VLP-16 환산 약14 ms를 예산에 넣었던 제안은 철회하며, pose/시간축 조건을
확보한 뒤 실제 입력으로 계측한다.

**구현체 도입 시 주의 (7.1 항목 10, 13, 16 관련).** SimpleTrack/Poly-MOT의 타 환경 FPS로
사용 불가 판정하지 않는다. AB3DMOT/FastPoly도 우리 자원·license·입력 조건 검증이 필요하다(6.5(e)).

### 7.2 경량 학습 트랙 — Capture / 동기 수집 데이터 활용

**Capture Mode 데이터의 제약을 먼저 확정해야 한다.**

Capture Mode는 저장 중 시뮬레이터를 정지시킨다. 측정 결과 연속 캡처 간 실제 변위는 보고된
속도와 경과 시간이 함의하는 값의 **약 0.39배**다. 벽시계와 시뮬 시간은 정지 구간에도
진행하므로 **파일만 보면 깨끗한 시계열로 보이지만 아니다.**

| 학습 형태 | Capture 데이터 | 근거 |
|---|---|---|
| 단일 스윕 3D 검출 (박스, 치수, heading) | **가능** | 프레임별 라벨은 정확한 순간 진리값 |
| 세그멘테이션 | **가능** | 동일 |
| 단일 프레임에서 라벨 속도를 회귀 | 원리적으로는 가능 | 라벨의 박스별 상대속도가 정확하다. 다만 단일 스윕에서 속도를 추론하는 것은 본질적으로 어려운 과제다 |
| **CenterPoint 방식 velocity head** | 구성·라벨에 따라 조건부 | 모든 구현이 두 map-view를 요구하지 않음. 시간 누적/속도 label 의미·정확도는 별도 검증 필요 |
| **다중 스윕 누적 입력** | **불가** | 동일 |
| 트래킹·오도메트리·스캔 정합 학습 | **불가** | 동일 |

> **2026-09-27 검수:** 초판의 무조건 가능도, 이전 정정의 무조건 불가도 과도했다.
> `CenterPoint@3cf7d87` head/loader를 구분해 검사했다. 단일 입력으로 velocity label을
> 회귀할 수 있는 구성과 실제 동작을 관측하는 능력은 다르다. 현재 Capture를 연속 시간축
> 평가로 승인하지 않는 결론은 유지하되 모든 velocity head의 입력을 동일하다고 하지 않는다.

**첫 비교에서는 검출기 속도 head로 문제를 해결했다고 가정하지 않는다.** 박스 검출과
시간축/자차 운동이 유효한 추적을 분리 평가한다. 추적 또한 map 이름만으로 절대속도가
보장되지 않는다(6.5 정정). Capture 정차 간격으로 속도를 평가하지 않는다.

시계열 학습이 꼭 필요해지면 **rosbag 기반 동기 수집 데이터**를 별도로 확보한다.
자차 속도를 캡처 프레임 간 차분으로 구하지 않는다. 라벨의 ego velocity를 쓰되 그 값은
**body frame**임에 주의한다.

**데이터 현황.** Capture Mode는 모든 수용 run에 대해 LiDAR를 이미 저장했다. 다만 MORAI 자체
동작으로 **카메라와 다른 SensorData 루트에 기록되어 아직 리포지토리로 import되지 않았다.**
재수집은 불필요하고 import 작업만 필요하다. 이름이 변경된 run들은 LiDAR 파일명이 이전
run id를 유지하므로, 매핑은 프레임 집합 포함관계로 유도한다.

**권장 학습 구성**

| 항목 | 값 |
|---|---|
| 모델 | **PointPillars 또는 CenterPoint-Pillar, 단일 스윕, 속도 헤드 없이** |
| 입력 | `x [-60, 80]`, `y [-20, 20]`, `z [-2, 4]`, pillar 0.2 m |
| 클래스 | `car`, `truck/bus`, `pedestrian` 3종 |
| 라벨 | MORAI GT 객체 리스트 자동 변환 (사람 라벨링 0) |
| 프레임워크 | OpenPCDet 커스텀 데이터셋 |
| 배포 | ONNX → TensorRT FP16 |

**주의 사항**

- 격자 범위를 voxel 크기로 나눈 값이 **16의 배수**가 되도록 맞춘다(OpenPCDet 요구사항)
- **클래스 불균형을 먼저 확인한다.** 보행자 프레임이 적으면 보행자 검출은 학습해도 쓸 수 없다.
  이전 미사용 `morai_3d_detection` 분석기는 P4-C6에서 제거했으므로, 실제 LiDAR Capture import
  포맷이 정해진 뒤 `tools/perception`에 manifest 기반 분포 audit를 새로 둔다
- 학습은 주행 검증과 시간대를 분리한다. RAM 15.5 GiB에서 ROS 스택과 동시 실행하지 않는다
- **검증 기준을 먼저 정한다.** 기하 파이프라인 대비 어떤 거리 구간에서 얼마나 좋아야
  채택할지 정하지 않고 학습부터 시작하지 않는다

### 7.3 도입하지 않을 항목

| 항목 | 이유 |
|---|---|
| BEVFusion, VoxelNeXt 등 대형 모델 | 8 GB 공유 환경에 과하고 SPConv 배포가 어렵다 |
| MOS 계열 학습 모델 (LiDAR-MOS, 4DMOS, MotionBEV) | 학습 비용 대비 M-detector로 충분. 필요 시 후순위 |
| 기하 파이프라인의 DNN 전면 대체 | 분포 밖 실패가 안전에 직결 |

---

## 8. 미결정 사항: YOLO 의존 여부

**이 장은 결론이 아니라 열어둔 결정이다.** 지금 확정하지 않고, Phase 0~2 개선 후 같은
기준으로 계측해서 정한다.

**문제 제기.** LiDAR 인지가 충분히 강건해져 동/정적 구분과 소형 물체·보행자 트래킹이
안정되면 YOLO가 불필요할 수 있다. 그러면 카메라는 주행가능영역과 차선에 전념하고, 장애물
회피·차로 추종·교차로 합류를 LiDAR 기하 + 카메라 semantic 퓨전만으로 수행한다.
반대로 원거리 보행자처럼 LiDAR 점이 근본적으로 부족한 대상은 카메라가 유일한 근거다.

### 8.1 비교할 세 가지 구성

| 구성 | 클래스 근거 | 카메라 역할 | 비고 |
|---|---|---|---|
| **A. LiDAR + YOLO 클래스 퓨전** | YOLO 2D 박스 ↔ 클러스터 연관 | YOLO + 주행가능영역/차선 | 현재 기본 |
| **B. YOLO 미사용** | 기하 단서만 (OBB 치수, 동적 여부, 점 밀도) | 주행가능영역/차선만 | 퓨전 경로 전체 비활성 |
| **C. YOLO 보조 (비게이팅)** | YOLO는 선택적 속성 | A와 동일 | **결정 전 기본값으로 권장** |

**C의 정의:** 클래스가 없다는 이유로 객체를 버리거나 무시하지 않는다. 클래스는 있으면 더
나은 결정을 하고, 없으면 보수적으로 동작하는 부가 정보다. 이 상태를 유지하면 실험 결과가
어느 쪽으로 나와도 구조 변경 없이 따라갈 수 있다.

### 8.2 A/B는 지금도 전환 가능하다

`main.launch`의 `enable_sensor_fusion` 인자가 YOLO 노드와 `FusionProcess`를 함께 게이팅한다.
`enable_sensor_fusion:=false`로 두면 **구성 B가 그대로 실행된다.**

- 객체 퍼블리시는 계속된다. `Publish()` 호출은 게이트 밖에 있다
- `str_ObjectClass`가 `unknown`으로 남고 `MapObjectType()`은 3(기타)을 반환한다
- **부수 효과: `UpdateAABB()`가 실행되지 않으므로 4.4의 `f32_CoreX/Y/Z` 덮어쓰기 결함이
  사라진다.** 즉 현재 상태에서는 B 구성 쪽이 위치 추정이 오히려 더 정확하다.
  **이 차이를 보정하지 않고 A/B를 비교하면 잘못된 결론이 난다.** Phase 0에서 해당 결함을
  먼저 고친 뒤 비교한다

**함께 정리할 것:** `SetLiDARClassWithGroundNoise()`는 선언과 정의만 있고 **호출되지 않는다.**
현재 `noise` 라벨을 생성하는 경로는 존재하지 않으며, `지면노이즈`만 `Tracking.cpp`의
z 하드컷에서 만들어진다. 4.1(d)를 수정할 때 이 함수의 존치 여부도 함께 결정한다.

### 8.3 클래스에 의존하는 기능과 기하 대체재

| 기능 | 현재 클래스 의존 | 기하 대체 가능성 |
|---|---|---|
| planner `type` 필드 | `MapObjectType()` 1/2/3 | **가능** — OBB 치수 기반 geometric class |
| size lock (6.5 변경 2) | `car` 확정 필요 | **가능** — 트랙 수명 동안의 OBB 치수 히스토그램 |
| 클래스별 운동 모델 (6.5 변경 3) | car→CTRA, person→CV | **가능** — IMM이면 관측 동역학으로 자동 선택 |
| 차로 점유·추월 (6.7) | 차량 여부 | **가능** — 치수 + 차로 내 위치 + 속도로 충분 |
| **보행자 특별 취급** | **필수** | **어려움 — 8.4 참조** |

앞의 네 개는 전부 기하로 대체된다. 쟁점은 마지막 하나다.

### 8.4 핵심 쟁점: 보행자와 기둥의 구별, 그리고 탐지 거리 비대칭

정지한 보행자와 전봇대는 LiDAR 기하만으로는 매우 유사하다. 발자국 0.3 ~ 0.6 m, 높이
1.6 ~ 1.8 m, 둘 다 정지다. 구별 단서는 미세한 형상과 시간에 따른 변화뿐이다.

**그리고 탐지 거리 비대칭이 결정적이다.**

| 거리 | 카메라(1280x720) | VLP-16 |
|---|---|---|
| 20 m | 보행자 약 150 px 높이 | 수직 간격 0.7 m → 빔 2 ~ 3개 |
| 40 m | 약 75 px | 수직 간격 1.4 m → 빔 1 ~ 2개 |
| 60 m | 약 50 px, 검출 가능 | 수직 간격 2.1 m → **빔 0 ~ 1개** |

즉 구성 B에서는 **원거리 보행자를 "장애물"로는 감지해도 "보행자"로는 인식할 수 없다.**
그것이 허용되는지가 판단 기준이다.

> **코드 확인 결과 이것은 가정이 아니라 실제 의존이다.**
> `behavior_planner/src/feature/feature_builder.cpp`는 `ObjectType::VEHICLE`,
> `ObjectType::PEDESTRIAN`, `ObjectType::STATIC_OBSTACLE`을 **각각 다른 분기로 처리한다.**
> 즉 판제는 이미 보행자를 차량·정적물과 다르게 다루도록 구현되어 있다. 구성 B로 가면
> `type`을 기하 단서만으로 채워야 하고, 원거리에서는 `PEDESTRIAN` 분기가 사실상 죽는다.
> **판제 코드를 바꾸지 않고도 구성 B가 성립하려면, LiDAR가 기하만으로 `type`을 충분히
> 정확하게 채울 수 있어야 한다는 뜻이다.** 8.6 실험은 이 점을 반드시 측정 항목에 넣는다.

- 회피만 하면 되는 미션이면 허용된다. 6.4의 정적 점유 격자가 클래스 없이 처리한다
- 보행자에게 다른 거동(더 큰 횡방향 여유, 더 이른 감속, 정지)이 필요한 미션이면 허용되지 않는다
- **대회 채점 항목에 보행자 인식이 명시되어 있으면 구성 B는 제외된다.** 규정 확인이 선행이다

### 8.5 구성 B가 얻는 것

- VRAM 1.0 ~ 1.5 GB와 GPU 시간 회수 → 그대로 6.8의 3D 검출기 예산으로 전용 가능
- `FusionCamera.cpp` 827줄과 시간 동기 큐 전체가 실행 경로에서 빠진다. 유지보수 면적 감소
- 실패 모드 제거: 잘못된 클래스가 트랙에 고정되는 문제, 동기 실패로 인한 프레임 drop,
  ego 보닛 오탐 필터 같은 예외 처리
- 카메라 자원을 주행가능영역·차선에 집중. 음영구간에서 이미 검증된 강점이다

**주의:** B가 성립하려면 6.4(정적 점유 격자)와 7.1의 학습 불필요 항목이 **먼저** 들어가야 한다.
현재 상태에서 YOLO만 끄면 클래스 없는 트랙이 그대로 나가는 것이라 비교 자체가 무의미하다.

### 8.6 결정 실험 설계

**공통 조건.** Phase 0 수정 적용 후. 동일 rosbag을 구성만 바꿔 재생. MORAI GT 기준.

**시나리오.** 고주로 주행·추월, 교차로 합류, 보행자 조우, 정적 장애물 회피, GPS 음영구간.

**지표.** 거리 구간 `0-20 / 20-40 / 40-60 / 60+ m`로 분리해 측정한다.

| 지표 | 보는 이유 |
|---|---|
| 객체 단위 recall (클래스 무관) | "장애물로는 보이는가" — B 구성의 하한선 |
| **보행자 recall / FN** | **안전 결정 지표. 이 값이 결정을 좌우한다** |
| 차량 recall, 오탐율 | 추월 판단의 신뢰도 |
| ID 스위치, 트랙 수명 | 추적 안정성 |
| 속도 RMSE | 판제 보완로직이 계속 필요한지 |
| 지연 p95, VRAM | 예산 |
| 판제 결과: 회피 성공률, 불필요 급제동 횟수, 차선변경 성공률 | 최종 목적 함수 |

**사전에 합의할 채택 기준 (아래는 예시. 팀에서 확정한다).**

- 구성 B를 채택하려면 **0 ~ 40 m 보행자 FN이 A 대비 악화되지 않아야** 한다
- 그리고 판제 결과 지표가 A 이상이어야 한다
- 둘 중 하나라도 미달이면 **구성 C를 기본으로 유지**한다

**결과를 먼저 보고 기준을 정하지 않는다.** 기준을 먼저 적고 실험한다.

### 8.7 결정 전까지 지켜야 할 설계 규칙

결정을 미루려면 **어느 쪽으로 가도 되는 상태**를 유지해야 한다. 아래는 그 조건이다.

1. **클래스를 게이트로 쓰지 않는다.** 클래스가 없다는 이유로 객체를 드롭하지 않는다.
   현재 `IsNoise()`가 클래스 문자열로 필터링하는데, 이를 기하 기준(높이, 점유, 지면 상대
   높이)으로 옮긴다
2. **판제 인터페이스에 클래스 확률을 싣는다.** `type` 정수 대신 `p(car)`, `p(person)`,
   `p(unknown)` (6.6 항목 3). 판제가 "클래스 없음"을 보수적으로 처리할 수 있고,
   **YOLO를 빼도 메시지 스키마가 바뀌지 않는다**
3. **geometric class를 항상 산출한다.** YOLO 유무와 무관하게 OBB 치수·높이·점 밀도로
   `vehicle-like / pedestrian-like / structure-like / unknown`을 계산해 별도 필드로 싣는다.
   구성 B에서는 주 근거가 되고, 구성 A에서는 YOLO 오분류를 잡는 교차 검증이 된다
4. **6.4의 정적 점유 격자를 클래스와 무관하게 유지한다.** 이것이 구성 B의 안전 하한선이자,
   구성 A에서도 DNN·YOLO가 놓친 물체에 대한 마지막 방어선이다
5. **A/B 전환은 launch 인자 하나로 유지한다.** 코드 분기를 늘리지 않는다.
   `enable_sensor_fusion`을 그대로 쓰되, 이름을 의도가 드러나게 정리하는 것은 검토할 수 있다

이 다섯 가지를 지키면 실험 결과가 A든 B든 C든 **구조 변경 없이 스위치 하나로 따라간다.**

---

## 9. 이전 로드맵 — 2026-09-21 설계 기록

> 이 장의 Phase 0~3와 기간은 당시 제안이다. P1 변환 실패 완화와 L1 heartbeat의
> 무조건 적용은 철회했고, 나머지 후보도 일괄 적용하지 않는다. **현재 개발 순서는 0.7의
> L1-A, L1-B, L2~L5**이며 판제 오너십·실험 수락·message 호환 gate를 따른다.

초기에는 LiDAR 품질(L)과 GT→센서 전환(I·P)을 병렬로 진행하는 안을 다음과 같이 제안했다.

```text
LiDAR 트랙 :  Phase 0 --------- Phase 1 --------- Phase 2 --------- Phase 3
전환 트랙   :  I1 + P1 ---- P2, P3 ---------- 관측 지평(I6) ---- GT 어댑터 제거
```

### Phase 0 — 즉시 (0.5 ~ 1일, 되돌리기 쉬움)

**LiDAR 측.** 7.1 표의 0번(`/perception/objects` 발행, **ID 유일성 보증 + L1 heartbeat 포함**)을 먼저 한다.
이후 모든 개선의 효과를 판제 거동으로 직접 측정할 수 있게 해준다. 이어서 7.1 표의 1 ~ 8번.
전부 상수 변경과 버그 수정이며, 10번(연관 비용 GIoU 교체)도 함께 넣을 수 있으면 넣는다.

**판제 측. P1(변환 실패 완화)을 같은 주에 착수한다.** 객체 하나 때문에 급정지하는 구조가
남아 있으면 이후 어떤 LiDAR 개선도 주행으로 확인할 수 없다. **의존성이 없으므로 LiDAR
작업을 기다리지 않는다.**

착수 전 `benchmark_lidar_pipeline`으로 **기준선을 반드시 먼저 확보한다.**
GT 어댑터는 **지우지 않는다.** 정답값 대비 비교가 이후 모든 평가의 기준이 된다.

### Phase 1 — 1주

**LiDAR 측**

- `road_mask` ↔ LiDAR 정합 오차 계측 **(선행 필수)**
- `Roi()`에 HD Map ∪ 카메라 BEV ∪ 근거리밴드 마스크 적용
- Patchwork++(또는 linefit) 통합
- range-image BFS 클러스터링 도입
- 고주로 전용 ROI 프로파일 분리 (`x [-60, 80]`)

**판제 측**

- **P2** `yaw_rate`를 속도 벡터에서 유도
- **P3** AVOID 대상·FOLLOW leader의 ID 이탈 내성

P2와 P3는 LiDAR 품질이 나빠도 효과가 있고, 좋아져도 손해가 없다. 먼저 넣어두면 Phase 2의
LiDAR 개선 효과를 왜곡 없이 관측할 수 있다.

### Phase 2 — 2 ~ 3주

**LiDAR 측**

- map frame 추적 이전 (+ 측위 품질 게이트, body frame 폴백)
- L-Shape 코너 앵커 + size lock + 앵커 스위칭
- 거리 의존 R, OBB 각도 탐색 범위 축소
- 클러스터링 이중 임계 2단계 연관 + tentative / confirmed 생애주기
- 퓨전을 point-in-box + Hungarian + 클래스 확률(내부)로 교체
- 정적/UNKNOWN 객체의 geometry와 관측 속도 유효성을 보존(0 강제 발행 권고 철회, 6.4)

**양측 합의 항목**

- **I6 관측 지평(6.0.5 (b))** 설계 합의와 구현. LiDAR가 차로별 `s_observed`를 발행하고
  판제가 속도 상한에 반영한다. **GT → 센서 전환에서 남는 마지막 안전 간극이다.**

**Phase 2 완료 시점에 8장의 A/B/C 결정 실험을 수행한다.** 그 전에는 구성 C(YOLO 비게이팅
보조)를 기본값으로 유지하고, 8.7의 설계 규칙 다섯 가지를 Phase 2 작업에 함께 반영한다.

### Phase 3 — 병렬 연구 트랙

- Capture LiDAR import, 데이터셋 분포 분석
- GT 자동 라벨링 → Pillar 계열 단일 스윕 학습 (속도 헤드 없이. 속도는 트래커 담당)
- TensorRT FP16 변환, 지연 실측
- CLOCs 방식 late fusion으로 기하 파이프라인과 **병합** (대체 아님)
- M-detector로 동/정적 독립 판정
- 8장 결정 결과가 구성 B이면 YOLO 회수분(VRAM 1.0 ~ 1.5 GB)을 3D 검출기 예산으로 재배정

### 최종 — 대회 구성 고정

- launch에서 **`morai_objects_adapter.py` 제거**, `/perception/objects`를 LiDAR가 단독 발행
- `vehicle_state`를 `ego_vehicle_state_dev_node`(GT)에서 `vehicle_state_node`(GPS+IMU)로 전환
- 두 전환을 **동시에 하지 않는다.** 객체를 먼저 센서로 바꿔 안정화한 뒤 측위를 바꾼다.
  둘을 같이 바꾸면 회귀 원인을 분리할 수 없다
- 전환 전후로 10장의 지표를 같은 bag에서 비교한다

---

## 10. 검증 항목

아래는 이전 검증 지표/목표 기록이며, 현재의 정량 수락 임계는 L1-A 계측과 담당자 검토로
확정한다. 25 ms 알고리즘 목표와 0.30 s consumer freshness를 혼동하지 않는다.
입력 누락을 heartbeat로 숨기지 않고 원래 stamp 기준 처리율·관측 age·end-to-end 지연을
함께 기록한다.

| 항목 | 측정 방법 | 목표 |
|---|---|---|
| 단계별 지연 | `benchmark_lidar_pipeline` | 체인 전체 25 ms 이하. **p99를 본다** — 0.30 s 초과 시 판제 정지 |
| 단계별 점 수 | 동일 벤치마크의 `[LIDAR_POINTS]` | ROI 적용 후 감소율 확인 |
| 맵 정합 오차 | `/lidar/debug_clustered_map` vs `road_mask` 중첩 | 마진 설정 근거 확보 |
| **정지 물체 속도** | 직선 구간과 곡선 구간을 분리해 분포 비교 | 곡선에서 증가하지 않을 것 |
| ID 스위치 | 트랙 수명 분포, 동일 물체 ID 변경 횟수 | 감소 |
| 높이 오차 | GT 대비 OBB height | 과소평가 해소 |
| 클래스 확정 | 거리 구간별 확정률과 확정까지 걸린 프레임 수 | 원거리 보행자 확정률 개선 |
| 미검출 | GT 객체 중 판제로 전달되지 않은 비율 (거리·클래스별) | 감소 |
| **발행 간격** | `/perception/objects` 인접 메시지 간 시간 분포 | **최대값 0.30 s 미만.** 초과 = 정지 |
| **인지 지연** | `header.stamp`와 판제 수신 시각의 차 | 계측 후 L3 적용 여부 결정 |
| **변환 거부율** | `ConversionStatus != OK` 발생 빈도와 사유별 분포 | **0에 수렴.** P1 적용 전에는 급정지 횟수와 동일 |
| **ID 유지율** | 같은 GT 객체가 유지한 LiDAR 트랙 ID 수 | 1에 가까울수록 좋음. AVOID·FOLLOW 신뢰도의 직접 지표 |
| **`yaw_rate` 잡음** | 정지·직진 물체의 판제 추정 `yaw_rate` 분포 | 0 근처. P2 효과 판정 |
| **관측 지평** | 차로별 `s_observed`와 실제 제동거리 비교 | 제동거리 ≤ 관측거리 위반 0건 |

**정지 물체 속도를 직선/곡선으로 나누어 보는 것이 4.2 진단의 핵심이다.** 곡선에서만 커지면
ego yaw 보정 문제이고, 속도에 비례해 커지면 상대속도 차분 구조 문제다.

**모든 지표는 GT 어댑터와 LiDAR 발행을 번갈아 물려 같은 bag에서 비교한다.** GT 쪽 값이
그 지표의 상한이자 판제가 설계된 조건이다. 두 값의 차이가 곧 "전환 비용"이며, 어떤 지표에서
차이가 큰지가 다음 작업의 우선순위를 결정한다. **대회 직전까지 GT 어댑터를 지우지 않는
이유가 이것이다.**

---

## 11. 미해결 확인 사항

1. **IMU yaw 부호·단위 규약.** `Kalman.cpp`의 `kYawSign = +1.f` 가정이 MORAI IMU와 맞는지
   확인이 필요하다. 부호가 반대면 ego yaw 보정이 오차를 상쇄가 아니라 2배로 만든다.
   직선 주행과 곡선 주행에서 정지 물체의 `f32_AbsVy`를 비교하면 판별된다.
   **곡선에서만 커진다면 부호 문제다.**
2. **`road_mask`의 `map_id`.** `drivable_map_id`가 특정 값으로 고정되어 있다. 대회 맵과
   불일치하면 `LoadDrivableMask`가 예외를 던져 노드가 기동하지 않는다.
3. **UDP 7510 경로를 계속 유지할지.** 판제의 canonical 입력은 `/perception/objects`이고
   UDP 7510을 파싱하는 코드는 판제 쪽에서 찾지 못했다. 6.0의 ROS 발행으로 통일하면
   UDP 경로와 `Msg/TrackedObjectArray`를 정리할 수 있다. 사용처를 먼저 확인한다.
   또한 주행가능영역 필터는 `/last_lidar`에만 적용되어 있다.
4. **판제가 실제로 소비하는 필드.** `cluster_center`와 `obb_center` 중 무엇을 쓰는지에 따라
   4.4의 `UpdateAABB` 결함이 미치는 영향이 달라진다.
5. **대회 채점 항목에 보행자·차량 클래스 인식이 명시되어 있는지.** 8장 결정의 전제 조건이다.
   명시되어 있으면 구성 B는 선택지에서 제외된다.
6. **P1의 grace 정책을 얼마로 둘지.** 직전 객체 목록을 몇 초까지 재사용할지는 안전과
   반응성의 교환이다. 0.3 s는 제안값이며 판제 파트가 정한다.
7. **`vehicle_state`의 GPS/IMU 경로 품질.** 6.5 변경 1(map frame 추적)과 6.0.5(관측 지평)
   둘 다 측위에 의존한다. 음영구간 드리프트를 먼저 계측해야 두 항목의 설계가 정해진다.
8. **관측 지평을 LiDAR만으로 계산할지, 카메라 BEV coverage와 합칠지.**
   `asmc_perception` camera BEV runtime이 카메라별 coverage를 산출한다. 합치면
   음영구간에서 더 멀리 보지만
   계약이 커진다. 6.0.5 합의 시 함께 결정한다.

---

## 12. 참고 문헌

### 지면 분할

| 논문 | 출처 | 코드 |
|---|---|---|
| Patchwork++: Fast and Robust Ground Segmentation Solving Partial Under-Segmentation (IROS 2022) | <https://arxiv.org/abs/2207.11919> | <https://github.com/url-kaist/patchwork-plusplus> · ROS1: <https://github.com/url-kaist/patchwork-plusplus-ros> |
| TRAVEL: Traversable Ground and Above-Ground Object Segmentation (RA-L 2022) | <https://ieeexplore.ieee.org/document/9794594> | <https://github.com/url-kaist/TRAVEL> |
| Fast Segmentation of 3D Point Clouds for Ground Vehicles (linefit, IV 2010) | <https://ieeexplore.ieee.org/document/5548059> | <https://github.com/lorenwel/linefit_ground_segmentation> |
| GSeg3D: Grid-Based Safety-Critical Ground Segmentation | <https://arxiv.org/pdf/2603.04208> | — |

### 클러스터링 · 세그멘테이션

| 논문 | 출처 | 코드 |
|---|---|---|
| Fast Range Image-Based Segmentation of Sparse 3D Laser Scans (IROS 2016) | <https://www.ipb.uni-bonn.de/pdfs/bogoslavskyi16iros.pdf> | <https://github.com/PRBonn/depth_clustering> |
| Curved-Voxel Clustering for Accurate Segmentation with Real-Time Performance | <https://www.researchgate.net/publication/338938419> | — |
| C-ARC: Continuous-Adaptive Range Clustering | <https://arxiv.org/pdf/2606.18948> | — |

### 3D 객체 검출

| 논문 | 출처 | 코드 |
|---|---|---|
| PointPillars: Fast Encoders for Object Detection from Point Clouds (CVPR 2019) | <https://arxiv.org/abs/1812.05784> | <https://github.com/open-mmlab/OpenPCDet> |
| CenterPoint: Center-based 3D Object Detection and Tracking (CVPR 2021) | <https://arxiv.org/abs/2006.11275> | <https://github.com/tianweiy/CenterPoint> |
| CenterPoint ONNX/TensorRT 내보내기 | — | <https://github.com/CarkusL/CenterPoint> |
| NVIDIA Lidar AI Solution (CUDA-PointPillars, CUDA-CenterPoint, cuPCL) | — | <https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution> |
| FastPillars: A Deployment-friendly Pillar-based 3D Detector | <https://arxiv.org/abs/2302.02367> | <https://github.com/StiphyJay/FastPillars> |
| PillarNeXt: Rethinking Network Designs for 3D Object Detection (CVPR 2023) | <https://arxiv.org/abs/2305.04925> | <https://github.com/qcraftai/pillarnext> |
| OpenPCDet 커스텀 데이터셋 튜토리얼 | <https://github.com/open-mmlab/OpenPCDet/blob/master/docs/CUSTOM_DATASET_TUTORIAL.md> | — |

### 카메라-LiDAR 퓨전

| 논문 | 출처 | 코드 |
|---|---|---|
| CLOCs: Camera-LiDAR Object Candidates Fusion (IROS 2020) | <https://arxiv.org/abs/2009.00784> | <https://github.com/pangsu0613/CLOCs> |
| Fast-CLOCs (WACV 2022) | <https://ieeexplore.ieee.org/document/9706631> | — |

### 3D 다중 객체 추적

| 논문 | 출처 | 코드 |
|---|---|---|
| SimpleTrack: Understanding and Rethinking 3D MOT (ECCV 2022 W) | <https://arxiv.org/abs/2111.09621> | <https://github.com/tusen-ai/SimpleTrack> |
| AB3DMOT: A Baseline for 3D Multi-Object Tracking (IROS 2020) | <https://arxiv.org/abs/2008.08063> | <https://github.com/xinshuoweng/AB3DMOT> |
| Poly-MOT: Polyhedral Framework for 3D MOT (IROS 2023) | <https://arxiv.org/abs/2307.16675> | <https://github.com/lixiaoyu2000/Poly-MOT> |
| Fast-Poly: A Fast Polyhedral Framework for 3D MOT (RA-L 2024) | <https://arxiv.org/abs/2403.13443> | <https://github.com/lixiaoyu2000/FastPoly> |

### 차량 자세 추정 · OBB

| 논문 | 출처 |
|---|---|
| Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners (IV 2017, Zhang et al.) — **현재 코드가 채택한 방법** | <https://ieeexplore.ieee.org/document/7995698> |
| L-Shape Fitting-Based Vehicle Pose Estimation and Tracking Using 3D-LiDAR (IEEE T-IV 2021) — 코너 앵커 추적 | <https://ieeexplore.ieee.org/document/9427213> |
| Low-cost LIDAR based Vehicle Pose Estimation and Tracking | <https://arxiv.org/abs/1910.01701> |

### 동적 물체 분할 (MOS)

| 논문 | 출처 | 코드 | 학습 |
|---|---|---|---|
| **M-detector: Moving Event Detection (Nature Communications 2023)** | <https://www.nature.com/articles/s41467-023-44554-8> | <https://github.com/hku-mars/M-detector> | **불필요** |
| LMNet: Moving Object Segmentation in 3D LiDAR Data (RA-L 2021) | <https://arxiv.org/abs/2105.08971> | <https://github.com/PRBonn/LiDAR-MOS> | 필요 |
| 4DMOS: Receding Moving Object Segmentation (RA-L 2022) | <https://arxiv.org/abs/2206.04129> | <https://github.com/PRBonn/4DMOS> | 필요 |
| MotionBEV: Attention-Aware Online LiDAR MOS (T-IV 2023) | <https://arxiv.org/abs/2305.07336> | <https://github.com/xiekkki/motionbev> | 필요 |

### HD Map 활용 · 차선변경 계획

| 논문 | 출처 |
|---|---|
| 3D Point Cloud Processing and Learning for Autonomous Driving (IEEE SPM 2021) — 맵 prior 기반 전/배경 분리 | <https://arxiv.org/abs/2003.00601> |
| Maps for Autonomous Driving: Full-process Survey and Frontiers (2025) | <https://arxiv.org/abs/2509.12632> |
| Lane-Change Initiation and Planning for Highly Automated Driving on Freeways (2020) | <https://arxiv.org/abs/2007.14032> |
| Safe and Efficient Lane-Changing: Double Quintic Polynomial with TTC (2025) | <https://arxiv.org/abs/2509.00582> |

---

## 13. 문헌 검증 기록

2026-09-21에 11장 참고 문헌의 원문 PDF를 직접 확인하고 이 문서의 주장을 대조했다.
아래는 **검증 과정에서 바로잡은 초판의 오류**다. 같은 실수를 반복하지 않도록 남긴다.

| # | 초판 서술 | 원문 확인 결과 | 조치 |
|---|---|---|---|
| 1 | CenterPoint 속도 헤드를 Capture 데이터로 학습 가능 | 당시 두 map-view 필수라고 정정했으나 **이 일반화도 오류** | 2026-09-27 6.8/7.2 재정정. head 구성·label 의미·실제 시간축을 따로 검증 |
| 2 | 유령 트랙 수명을 25 → 10프레임으로 축소 | ID 스위치의 **95 %(차량) / 91 %(보행자)가 조기 종료** 때문 | 6.5 (b)에서 권고 철회. 수명 유지 또는 연장 |
| 3 | Curved-Voxel Clustering 15 ~ 17 ms (64ch) | 논문은 **이미 VLP-16**으로 평가했고 **20 ~ 30 Hz(33 ~ 50 ms)** | 6.3에서 후순위로 격하 |
| 4 | TRAVEL로 6.2와 6.3을 한 번에 교체 | 당시 논문 20~30 Hz로 예산 초과 판정 | 2026-09-27: 다른 환경 FPS만으로 제외한 판단 철회, 후순위 비교 |
| 5 | PointPillars "PyTorch 42.4 Hz → TRT 105 Hz" | 논문은 **62 Hz(기본) / 105 Hz(경량)**. 42.4는 검색 요약의 오류 | 6.8 정정 |
| 6 | CTRA/IMM 운동 모델을 중간 우선순위로 | SimpleTrack 실측상 **KF와 CV의 성능 차이가 미미** | 6.5 변경 3에서 하향 |
| 7 | SimpleTrack·Poly-MOT를 구현체로 도입 | 당시 인용 0.51/3 FPS로 사용 불가 판정 | 2026-09-27: 타 조건 수치로 사용 불가 판정 철회, 동일 입력 별도 평가 |
| 8 | 개선안의 판제 측 비용을 명시하지 않음 | **LiDAR 출력과 판제 입력이 아직 연결조차 되어 있지 않았다.** 판제는 `/perception/objects`를 구독하고 현재 MORAI GT 어댑터가 이를 발행한다 | **6.0 신설** 후, GT → 센서 전환 관점으로 전면 재작성 |
| 9 | 점유 격자·`LaneOccupancy`·클래스 확률을 판제 비용 언급 없이 제안 | 셋 다 신규 메시지 또는 스키마 변경을 요구 | 6.4 / 6.6 / 6.7에 **계약 안 대안**을 각각 추가하고 원안은 후순위로 |
| 10 | 판제가 GT 전제로 만들어졌다는 점을 분석하지 않음 | **변환 실패 시 판제가 급정지**를 낸다. ID로 AVOID 대상을 추적한다. `yaw_rate`를 heading 이력으로 추정해 CTRV 예측에 넣는다. **가림을 표현할 수단이 계약에 없다** | 6.0.3 표, 6.0.4의 P1 ~ P3, 6.0.5의 관측 지평 제안 |

**검증 과정에서 새로 찾은 근거는 다음 위치에 반영했다.**

- Zhang 논문 Algorithm 1의 거리 비례 세그멘테이션 → 6.3, 7.1 항목 1
- Zhang 논문의 탐색 공간 축소 제안 → 6.5 변경 2 하위, 7.1 항목 8
- 코드의 closeness 구현이 Zhang Algorithm 4와 다른 점 → 4.4
- Patchwork++ Table II의 RANSAC 15.43 Hz 대비 → 6.2
- Bogoslavskyi의 16-beam 2.4 ms, PCL 대비 세 자릿수 배, VLP-16 보행자 분리 → 6.3
- SimpleTrack의 "거리 지표 + Hungarian은 나쁜 조합" → 6.5 (a)
- SimpleTrack의 조기 종료 통계와 2단계 연관 → 6.5 (b), (c)
- L-Shape T-IV 2021의 "코너가 centroid보다 안정적" 인용 → 6.5 변경 2
- FastPillars의 SPConv 배포 난점 3가지 → 6.8
- M-detector의 "truly moving vs movable" 성질과 84.48 ms 실측 → 7.1

**코드 재확인으로 찾은 것 (논문 아님).**

- `/perception/objects` 계약과 현재 GT 어댑터 연결 상태 → 6.0
- `ConvertDetectedObjects()`가 객체 하나 오류에 **프레임 전체를 거부** → 6.0, 6.4
- `MakePlannerObjects()`에 **ID 유일성 검사 없음** → 6.0.3
- 변환 실패 시 `publishStop` / `MakeStopTrajectory` → 6.0.3, P1
- `AvoidTargetResolver`가 ID로 대상 추적, 상실 시 `LOST` → 6.0.3, P3
- `MergeGapEvaluator`가 `yaw_rate`로 CTRV 원호 예측 → 6.0.3, P2
- `vehicle_state_node.cpp`가 **GPS+IMU 기반으로 이미 구현되어 있음** → 6.0.1
- `params.yaml`의 `timeouts/objects = 0.30` 과 stale 시 정지 → 6.0.6 L1, L2
- `LidarProcess()` 조기 반환과 `Publish()` stamp dedup으로 **미발행 경로 존재** → 6.0.6 L1
- `message_converter.cpp`에 **객체 지연 보상 없음**(의도된 설계) → 6.0.6 L3
- `behavior_planner`가 VEHICLE / PEDESTRIAN / STATIC_OBSTACLE로 **이미 분기** → 8.4
- `behavior_planner`가 객체 차로 투영과 gap 계산을 **이미 수행** → 6.7
- 연관 가중치 실측 `COST_W_DIST=1.0`, `COST_W_IOU=1.0`, `THRESHOLD=3.0` → 6.5 (a)
- `SetLiDARClassWithGroundNoise()`가 **정의만 있고 호출되지 않음** → 8.2

**아직 검증하지 않은 것.** 이 문서의 연산량 추정치(ROI 축소 효과, VLP-16 환산 지연)는 여전히
추정이다. 10장의 계측을 수행하기 전까지는 근거가 아니라 가설로 취급한다.

### 13.1 L1-B1 재검수 기록 — 원 주장과 정정 이유 보존

상세 설계와 이전 수치는 삭제하지 않고 해당 위치의 판단을 수정했다. 아래는 이번 변경의
대조표다. 논문 전체를 새로 재현했다는 뜻이 아니라, 고정 참고 코드의 입력·시간·좌표 전제와
L1-A/B1 실험 근거가 허용하는 결론으로 낮춘 것이다.

| 이전 주장 | 정정·이유 | 반영 위치 |
|---|---|---|
| TRAVEL 20~30 Hz라 제외, SimpleTrack 0.51 FPS/Poly-MOT 3 FPS라 사용 불가 | 데이터·객체 수·하드웨어·측정 범위가 다름. 동일 입력 성능 비교 미실행 | 6.3, 6.5(e), 7.1, 13 |
| RANSAC은 가장 느리고 부정확, Patchwork++면 둘 다 개선 | 표는 타 환경. L1-A에서 낮은 장애물/보행자 보존 상충 확인 | 6.2, 0.8 |
| VLP16 점 수 1/6이니 M-detector 약14 ms, DNN 10~20 ms/2 GB | 고정 비용·dense BEV·시간 이력·copy 비용을 무시. 미측정 수치로 예산 확보 불가 | 6.2, 6.8, 7.1 |
| range BFS 두 번이면 5 ms 미만 | 인용 환경의 단일 알고리즘 시간을 우리 연관 포함 체인에 환산 불가 | 6.3, 6.5(c), 7.1 |
| map 추적이면 정지 속도 0, 회전 과도응답·classifier 필요성 소멸 | 관측 시각/pose/검출 변동·복귀 점프가 남음. 속도 의미와 운동 항부터 확인 | 6.5, 공통 소비 계약 |
| 관측 지평 스칼라는 항상 보수적, 최대 return이면 끝 | beam 사이·가림에 의해 free-space 과대 판정 가능 | 6.0.5 |
| 모든 CenterPoint velocity head는 두 map-view 필수라 단일 sweep 불가 | `center_head.py`와 loader의 역할이 다름. 구성 가능성과 운동 관측 가능성을 구별 | 6.8, 7.2, 13 |
| 정적 클러스터는 ID/속도 불필요, type=3/속도0이면 판제 수정0 | ID는 대상 유지에 필요할 수 있음. UNKNOWN/정적 class가 정지 motion 근거 아님 | 6.4, 7.1, 9 |
| 자세 보정 약0.35 ms를 raw 운동보정 비용으로 사용 | ROI+voxel 후 특정 단계 계측. B1은 raw 강체 변환과 copy만 별도 계측; deskew 아님 | 0.8, B1 보고서 |

코어·참고 자료의 SHA와 선택된 코드 경로는 B1 manifest/reference audit에 기록했다.
신규 공식 참고는 [Autoware distortion corrector](https://autowarefoundation.github.io/autoware_universe/main/sensing/autoware_pointcloud_preprocessor/docs/distortion-corrector/)와
[LIO-SAM 점별 시간 안내](https://github.com/TixiaoShan/LIO-SAM#prepare-lidar-data)다.
각각 `time_stamp`/twist/IMU, `time`/ring/IMU 입력이 필요하며 현재 XYZI에 추측해서 붙이지 않는다.
