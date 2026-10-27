# 관측 연결·위치·유효성 개선 마일스톤

2026-10-27 공동 작업 기준이다. **안승현은 3뷰 fusion, 손재호는 터널 통합 뒤 M3 관측 위치·OBB와
M4 추적·속도·운동 상태를 맡는다.** 장원태의 YOLO detect 추가학습은 병행한다. 모델 교체보다
현재 관측을 정확히 연결하고, 관측 위치와 유효성을 안정화하는 작업을 우선한다.
이 문서는 후속 개발 범위이며 아래 후보를 이미 운영 적용한 것으로 읽지 않는다.

## 출발점과 이번 PR 경계

- PR #118의 F1이 main `22577363`에 병합됐다. 되돌리지 않고 후속 PR에 연구 근거·역할·작업표를 정리한다.
- [F1 결과](../../../docs/records/lidar/2026-10-27-front-fusion-verification.md)는 전방 실제 matching,
  좌우 수신/native 재투영, 관측 식별 결함 수정까지다. **좌우 matching은 다음 PR**이다.
- 별도 연구 브랜치의 [선별 통합 기록](../../../docs/records/lidar/2026-10-27-research-integration.md)은
  과거 결과와 실행 소스를 보존한다. 원본 clock 대응·독립 위치/속도 GT는 여전히 미확보다.
- [재호의 PR #119](https://github.com/INHAautonav/2026-ASMC/pull/119)는 확인 시점에 미병합이다.
  터널 구현과 오프라인 대조를 마쳤고, head `90415a11`에서 F1 main을 병합했다.
  병합 후 회귀·실시간 공동 수락은 남아 있다. 과거 base `93666315`로 현재 main을 되돌리지 않는다.
- 이번 PR은 운영 소스·설정·공용 메시지·LAN2LAN/bridge를 바꾸지 않는다. 기존 실행/판제 입력은 유지한다.

## 작업 순서와 담당

번호는 작업 식별자다. M1과 M2는 병행 가능하고 M3→M4는 같은 관측 기준선을 순서대로 사용한다.

| 단위 | 담당·현재 상태 | 변경 범위·선행 조건 | 완료 산출물 |
|---|---|---|---|
| M0 · 실패 사례 연결 | 승현, 재호 공동 검수 · 미착수 | 기존 C0/F1·터널 replay/StageObserver 재사용. 빠진 단계만 선택 사례에 추가 | 군집 혼입 / 위치 변화 / 연관·miss / 전달 필터 문제를 구분한 사례 장부 |
| **M1 · 3뷰 fusion** | **승현 · 미착수** | F1 유지, 뷰별 비차단 선택·동일 scan context·중복 근거 통합. tracker/속도는 유지 | 전방 회귀 + 좌우 연결·뷰 중단/복귀·중첩/충돌의 검증 가능한 분류 근거 |
| M2 · 터널 통합 수락 | 재호 주관, 승현 연결 확인 · 검증 대기 | PR #119 구현·오프라인 결과 재사용. F1 main 병합 후 통합 회귀 | GPS0/중단/복귀·실제 장애물 보존·확인 지연·local 공급 검증 |
| **M3 · 관측 위치·OBB 안정화** | **재호, 승현 지원 · 미착수** | M0 사례·M2 통합 기준선. 우선 기존 L-shape와 관측 기준점 대조 | 가시 면/부분 가림에서 중심·yaw·치수 변화 원인과 한 가지 개선 |
| **M4 · 연관·속도·운동 상태** | **재호 · 미착수** | M3 관측 입력 고정. innovation/공분산·miss·재초기화 중 한 원인씩 대조 | ID 전환·정적 오동적·속도 튐·유효성/반응 지연 개선 및 한계 |
| M5 · 판제 공동 수락 | 인지·판제 공동 · 미착수 | 선택한 M1–M4 변경과 새 detect 모델을 각각 검증한 뒤 통합 | 일반/정적/동적/로터리/고주로/터널 주행과 공급·장애 대응 결과 |

M0는 C0/F1을 다시 수행하는 큰 진단 프로젝트가 아니다. 터널 scan64/112/118 및 일반 구간
대표 사례에서 **현재 코어**를 bag 처음부터 replay하고, anchor 주변만 상세 표시한다. 오래된
track ID76을 현재 같은 물체의 ID로 가정하지 않는다. 당시 raw 점 실험과 현재 voxel 평균점의
연결을 일대일로 꾸미지 않으며, 확정할 수 없는 detection→track 연결은 unknown으로 남긴다.
기존 trace로 설명되는 항목에는 새 계측을 만들지 않는다.

## M1 — 승현이 다음 PR에서 진행할 범위

1. 전방의 receive header ±50ms·최대 대기200ms·unique 관측 사용과 분류 확인 정책을 회귀 기준으로
   고정한다. 뷰별 후보 큐를 독립시키고 한 뷰 중단이 다른 뷰를 기다리게 하지 않는다.
2. scan context·점군은 한 번 확보하고 세 뷰가 참조한다. 투영은 현 native 코어와 원본 크기
   전방1280×720·측방640×480을 사용한다. 좌우 카메라를 전방 보정으로 투영하지 않는다.
3. 근거 키는 카메라 session/generation/view/stamp와 bbox index, LiDAR session/generation/scan으로
   구분한다. 같은 영상 재조회는 확인 횟수를 늘리지 않는다. 같은 scan의 여러 뷰가 한 track을
   볼 때 확인 횟수를 세 배로 늘리지 않는 통합 단위를 먼저 확정한다.
4. 여러 뷰의 class가 충돌하면 근거·가시성·시각을 남기고 unknown/기존 유효 분류 유지 정책을
   명시한다. 마지막 도착 뷰가 덮어쓰거나 confidence만으로 항상 결정하지 않는다.
5. fresh 양성 vehicle/dynamic_obstacle 근거만 HD맵 보호로 전달한다. 카메라 빈 결과·미수신은
   LiDAR 객체 삭제 사유가 아니며 LiDAR 속도와 과거 분류를 신규 카메라 확인으로 승격하지 않는다.
6. 2D box만 있고 LiDAR 지지가 없는 경우 3D 위치·속도를 만들어내지 않는다. 점 지지 분리는
   혼입 사례가 남을 때 별도 후보로 대조하며, 3뷰 확장과 tracker 교체를 한 번에 하지 않는다.

전방 단독 결과 동등성, 중첩 뷰의 중복 확인 방지, 한 뷰 중단/빈 결과, 재시작·세대 전환,
GPS 무효 local 투영을 먼저 검사한다. 기존 trace/갤러리로 전후 연결 근거와 비용을 제시한다.
서로 다른 원본 clock을 직접 차감하지 않는 F1 receive 기준 구현은 가능하다. 정확한 촬영 시각
정렬이나 ego motion 보정을 완료했다고 주장하려면 별도 clock 근거가 필요하다.

## M2–M4 — 재호 인수와 교차 리뷰

PR #119가 보고한 40개 CTest·47회 replay, 실제 차량138/138·시설물97/97 보존은 해당 고정 자료의
결과다. GPS 정상4,230프레임의 전처리 일치와 전체 track/속도/상태 메시지 일치는 다르다.
후방 잔여 후보와 GPS 강제 무효 스트레스의 전방 후보가 남아 있어 모든 터널 해결로 표시하지 않는다.

통합 시 F1 진단·관측 epoch 검사·fresh camera 보호를 유지한다. 정상 GPS3회 복구와 후보별
3/5/6회 확인의 **최초 전달 지연**, GPS0 상태의 local 공급, 지도 무효 상태, 실물 돌출부·작은
시설물·흰선 밖 여유 공간 객체 보존을 확인한다. 벽 추적용 signed CAN/IMU 운동 보상과 객체의
상대/절대속도 유효성을 혼동하지 않는다. 자세한 구현은 PR #119 정본을 따른다.

M3에서는 관측 point centroid·OBB center·KF 기준점을 분리해 기록한다. 가시 면 변화에 따른
중심 이동, yaw의 π/2 또는 π 모호성, 치수 변화·벽 혼입을 먼저 구분한다. 참조 yaw/크기로
큰 OBB를 강제 유지하거나 현재 장애물을 사라지게 만드는 후보는 채택하지 않는다.

M4는 안정된 M3 관측에 대해 연관 오류면 covariance 기반 gate, miss/재관측 문제면 lifecycle을
먼저 고른다. Q/R·연관 비용·운동 모델·확인 횟수를 동시에 바꾸지 않는다. GT가 없으면 속도
정확도 대신 내부 일관성만 보고하고, UNKNOWN/무효를 유효한0속도나 static으로 바꾸지 않는다.

## 우선 참고할 기준선과 적용 조건

| 우선 | 참고 기준선 | 현재 코드에 적용할 부분과 한계 |
|---|---|---|
| M1 | [EagerMOT 공식 구현](https://github.com/aleksandrkim61/EagerMOT) | 2D/3D 관측의 연관·한 센서만 있을 때의 근거 관리 참고. 기존 거대 군집을 점 단위로 분리하는 기법은 아님. 전체 tracker 이식은 보류 |
| M1 이후 혼입이 남을 때 | 기존 E8 box + 깊이/연결 지지점 후보 | 과거 scan64의 배경 혼입 감소는 가능성 근거. 최근접 성분이 항상 객체가 아님. 점/이웃 탐색 상한·원본 index·unknown 보존부터 구현해야 함 |
| M3 | [L-shape 기반 Autoware shape estimation](https://github.com/autowarefoundation/autoware_universe/blob/main/perception/autoware_shape_estimation/README.md) | 이미 있는 L-shape의 reference yaw·shape·실패 보존을 대조. 규칙 기반 경로만 참고하며 ROS2/PointNet 전체 도입 아님 |
| M4 | [Probabilistic 3D MOT](https://github.com/eddyhkchiu/mahalanobis_3d_multi_object_tracking) | residual과 covariance가 맞는지 확인 후 Mahalanobis gate 대조. 불확실성 수치 없이 gate만 추가하지 않음 |
| M4 | [SimpleTrack](https://github.com/tusen-ai/SimpleTrack) | 관측 확인·miss 유지/종료·재관측을 분리해 검토. 전체 검출기/추적기 교체 대신 재현된 규칙 하나씩 대조 |
| 조건부 | [PCL Conditional Clustering](https://pointclouds.org/documentation/tutorials/conditional_euclidean_clustering.html), [depth_clustering](https://github.com/PRBonn/depth_clustering) | 실제 표면 혼입이 남을 때만 군집 대조. 다른 점을 통해 연결될 수 있고, 센서 투영·원본 index와 계산 비용 검증 필요 |

FEC/FastPoly는 각각 군집/연관 계산이 병목일 때, Poly-MOT는 관측이 정상인데 운동 모델이 틀릴 때,
Patchwork++는 지면 실패가 입증됐을 때만 검토한다. 지면·벽·군집·tracker 전체 교체를 기본 과제로
두지 않는다. 학습 Plan B(CenterPoint-Pillar)와 Plan A(HARP 계열+영상 semantic)는 이 개선 이후
고전 처리의 한계가 남고 도메인 GT·GPU 예산을 확보할 때 별도 실험한다.

## YOLO 인수 — detect 우선, seg는 조건부

장원태에게 요청할 우선 산출물은 **기존16종/3그룹의 detect 추가학습 모델**이다. 원본 클래스 순서,
데이터/split SHA·ignore 규칙·epoch/EMA 출처·동일 val 비교·실패 사례·배포 export를 함께 인수한다.
현재 B1/B2·mask·confidence/NMS 설정에서 재검증하며 새 모델과 3뷰 알고리즘의 효과를 분리한다.

[공식 segmentation 출력](https://docs.ultralytics.com/tasks/segment/)에는 box·class·confidence와
instance mask가 있지만 현재 detect engine의 binding·후처리에 그대로 끼울 수 있다는 뜻은 아니다.
2D mask도 가림·배경·시각 불일치를 없애거나 LiDAR가 못 본 깊이를 제공하지 않는다.

과거 라벨 감사에서 train의 mask 메타데이터 양성은 native RGBA64,966 + packed5,162개였으나
box-only80,485개도 있었다. 완전 양성 프레임에서 모두 mask 메타데이터가 있는 train은186장,
실제 mask 점검은48개 표본이었다. 이를 전체 mask GT 검증 완료나 현재 데이터 재감사로 해석하지 않는다.
Partial GT를 배경으로 학습하거나 box를 사각 mask GT로 바꾸지 않는다.
우선 detect 개선을 납품하고 mask 원본/index를 보존한다. seg는 작은 고정 pilot에서 16종 mask
품질·box recall·원본 좌표 복원·TensorRT binding·3뷰 전체 지연을 검증한 뒤 별도 채택한다.

## 병행 개발의 경계와 수락

- 승현: `fusion/`, 카메라 `geometry/`·수신·HD맵 보호 연결. 재호: `experimental/`/터널 전처리,
  `detection/obb`, `tracking/`·운동 상태. `pipeline/`, `ros/runtime`, trace와 공용 타입은 공동 리뷰한다.
- 점군·변환 context를 재사용하고 모델/뷰마다 깊은 복사를 추가하지 않는다. 같은 observation의
  재사용 가능성과 분류 확인 횟수는 서로 다른 개념이다.
- 세 작업 모두 `local`은 GPS와 독립, `map`은 유효 scan pose에서만 제공한다. 판제 입력·TTC·추종·
  정지·회피 정책은 판제 소유다. 진단 JSON을 판단 입력으로 연결하지 않는다.
- 새 알고리즘은 고정 입력·출력 차이·실패 사례·p50/p95/p99·RSS·공급 공백을 남긴다. 새 추론 모델은
  별도 회귀한다. 정상 공급≥10Hz·공백≤250ms는 기존 수락 목표이며 F1 정차473.47ms 실패를 면제하지 않는다.
- 동시 원본 시각/독립 GT가 없으면 비교 불가로 남긴다. 갤러리가 매끄럽다는 이유로 정확도·운영
  수락을 선언하지 않는다. 개발용 Ego GT는 평가에만 사용한다.

현재 실행과 복구는 [planning-runtime](planning-runtime.md), 상태 요약은 [공동 작업표](migration-tasks.md),
F1 재현은 [fusion-verification](fusion-verification.md)이 정본이다.
