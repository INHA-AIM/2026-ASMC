# 인지 개발 로드맵

현재 카메라 모델·실행 구조는 고정하고 판제·LiDAR 소비와 반복 주행 검증을 우선합니다.
학습·데이터 확대·새 모델 비교는 실제 실패와 요청에 연결합니다.
세 모델은 상시 ACTIVE·발행을 기본으로 하며 [구간별 소비](../handoffs/perception-to-planning.md)는 판제 책임입니다.

| 단계 | 현재 상태 | 다음 확인 |
|---|---|---|
| YOLO 3뷰 | E8/사전학습 선택·TensorRT·16종→3그룹·전방960, 카메라 단일 생산자 | 차량 자체 누락·중복·그룹 전환·입력 중단 대응 |
| TwinLite/family | Large v7 E8 기본·전방544×960/측방480×640·TensorRT·상시 model-grid→family | 새 모델의 투영 범위·피팅 품질, A126 진입 전 유효 family·음영 한정 소비·freshness/fallback |
| 신호등 | camera4 ResNet10Hz·typed 관측 | 신호등 생산자 하나, stale 시 legacy 우회 없음 |
| Main 연결 | 전방 CPU 어댑터·unique ±50ms/200ms·기존 판제 출력 | 최신 판제와 실제 주행, 원본 이름/그룹/운동 분리 |
| LiDAR 이전 | `asmc_lidar` 좌표/시간 코어·3뷰 수신/버퍼/투영 라이브러리 | 손재호의 실행 연결·다중관측 통합·검출/추적 이전 |
| HD Map | 정적 global/local·mission products | 판제 local map copy와 정합; 지도와 카메라 관측의 권위 분리 |
| 자료·학습 | 원본 보존·v11 기반 E8 배포 | 추가학습 요청 시 실패 클래스/조건만 보강·고정 split 재검증 |

일반 구간은 HD맵 기반 판제이며 차선 주행 소비는 음영 ODD에 한정합니다.
카메라는 구간 진입/이탈이나 회피 방향을 결정하지 않습니다. BEV 영상은 수요가 있을 때만
계산하며 family는 model-grid의 직접 지면 투영을 사용합니다.

## 품질 문제를 실행 문제와 분리한다

bus/truck의 간헐적 누락, 차로 정면 bicycle/dog, dog2/dog10 약세, 터널·범퍼 앞 근거리 관측을
실패 목록으로 유지합니다. 사용자가 현재 운영상 허용하고 판제 요청 전 추가학습을 보류했습니다.
Semantic이 정상인데 family 공급/역할이 잘못되면 실행·피팅·보정부터 조사합니다.
Semantic 자체가 잘못되면 TwinLite, 객체 박스 누락·혼동은 YOLO의 데이터/학습을 검토합니다.

추가학습은 COCO 확대나 두 모델 동시 학습을 기본으로 삼지 않습니다.
대회 에셋·SUNNY/FOGGY×11/13/15·정면/터널/근거리·하위 종류를 보강하고
기존 객체 회귀·배경 FP·중복 FP·3그룹/16종 품질과 지연을 함께 확인합니다.
RF-DETR는 지연 비교 후 현재 후보에서 제외했으며 다시 전 조합을 탐색하지 않습니다.
Teacher는 위치 초안과 이름 확정 근거를 분리하고 미검출을 negative로 만들지 않습니다.

## 다음 단계 — Large E8 기준 family 범위·품질

다음 카메라 작업은 Large E8 기준의 family 범위·품질 점검이다. 기본 전환은 사용자 승인에
따른 결정이며 [E8 검증 기록](../records/camera/2026-10-18-twinlite-large-deployment.md)의
SUNNY freshness3건·별도 재생 공급 저하·황색차선 회귀를 해결된 항목으로 바꾸지 않는다.

1. 현재 계산 범위 x[-5,20]m·y[-8,8]m, raster0.10m와 판제 검토 범위 x[0,15]m를 구분한다.
   Family는 semantic model-grid를 직접 지면으로 투영하므로 raster 해상도를 바꿔 정확도가
   개선된다고 가정하지 않는다. 추론/투영의 실제 입력은 전방544×960·좌우480×640이다.
2. 기존 SUNNY/FOGGY×11/13/15시·터널 입구/내부/출구 자료에서 근거리·0–5/5–10/10–15/15–20m별
   근거와 observed/inferred/invalid 비율·곡선 연결·폭/역할을 확인한다. 독립 GT가 없으면
   지상 위치 정확도를 확정하지 않고 cross-view 정합과 사례 검토로 표시한다.
3. 근거 누락과 보간/피팅 문제를 분리한 뒤 범위·지원 기준·피팅 설정을 한 항목씩 대조한다.
   같은 RGB/pose/model-grid와 전후 곡선·유효성·연산 시간을 비교하며 근거 없는 외삽을 늘리지 않는다.
4. 기본 전환 때 피팅·보정·격자·freshness는 유지한다. IMU/지면 보정과 새로운 fusion 구현은
   현재 평면 가정에서 확인된 투영 오차가 있을 때 별도 근거로 결정한다.

이 점검과 판제/LiDAR의 소비 검증은 역할을 나눠 진행한다. 새 수집·재학습이나 담당자의
구현 범위를 이번 기본 전환에 추가하지 않는다.

## 연결 문서

- [현재 카메라 구조](../../src/asmc_camera/docs/integration.md), [실행 계약](../contracts/camera-runtime-v1.md)
- [객체 에셋·라벨](../competition/obstacle-assets-and-labels.md), [데이터 준비](../operations/data-preparation.md)
- [카메라→LiDAR](../handoffs/camera-to-lidar.md), [음영 차선→판제](../handoffs/camera-to-planning.md)
- [대회 완료·담당별 수락](consolidation-roadmap.md)
- [과거 설계·실험·후보 비교](../records/camera/perception-roadmap-through-pr85.md)
