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
| LiDAR 이전 | `asmc_lidar` 좌표/시간 코어·3뷰 수신/버퍼/투영 라이브러리, Main 검출/추적·전방 fusion의 shadow 노드(운영 미연결) | 손재호의 운영 실행 연결·다중관측(3뷰) 통합·MotionClassifier·판제 후처리 이전 |
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

## 다음 단계 — source 정합과 실제 가림

TwinLite Large E8·전방544×960/측방480×640은 유지한다. 원본 시각 IMU·공동 폭 계산을
[잔차·10분 수락](../records/camera/2026-10-18-joint-family-stabilization.md) 후 기본 전환했다.
이전 Large 배포·M1-rest·폭 완화 후보의 실패는 당시 기록을 보존한다.
새 정차 시험의 통과를 전체 주행·판제 제어 수락으로 확대하지 않는다.

1. **3뷰 중첩·시각 정합:** 동일 차선의 카메라별 위치 차이·중복 가중과 raw source skew를
   분리한다. 드문 M0 복귀의 순간 곡선 변화도 관측하며 freshness를 늘려 감추지 않는다.
2. **실제 가림·출구 소실:** 기존 역할 제거5건은 스트레스 시험이다. 실제 RGB 가림과 내부 공백/
   관측 밖 외삽을 나눠 support·observed/inferred 판정이 유지되는지 확인한다.
3. **범위:** 계산 x[-5,20]·y[-8,8]m, raster0.10m와 판제 검토 x[0,15]m를 구분한다.
   직접 model-grid 투영이므로 raster를 조밀하게 만들어 정확도가 좋아졌다고 설명하지 않는다.
4. **조건부 이동 정렬:** 터널에서 유효한 상대 속도·yaw 입력을 확보하고 잔차의 원인이 시각 차이일
   때만 뷰 사이 ego motion 정렬을 진행한다. 개발용 Ego pose·과거 GPS를 운영 입력으로 사용하지 않는다.
   과거 근거 누적은 그 이후이며 모델·대체 곡선 탐색을 동시에 확대하지 않는다.

차선 주행 소비는 음영 ODD에 한정하고 판제/LiDAR의 구현 책임을 유지한다.
현재 BBox adapter의 config SHA·정본 보정·같은 영상의 IMU plane 연결은 구현됐다.
[카메라→판제 인수](../handoffs/camera-to-planning.md)에서 남은 실제 공동 주행을 확인한다.

## 현재 담당과 연구 확대 조건 — 2026-10-20

안승현은 [L0–L8](../../src/asmc_lidar/docs/migration-tasks.md)로 기존 Main을 유지하며 이관한다.
손재호는 별도 브랜치에서3뷰 카메라–LiDAR fusion을 병행하고, 장원태는 YOLO 후속 학습을
담당한다. 안승현은 기존 카메라 계약·보정·진단 지원과 데이터 인계, 시뮬 PC 준비를 맡는다.
판제의 HD 좌표/지도 번들·원본 클래스/그룹 해석은 해당 소비자가 적용하고,
[HD 확인표](../handoffs/hd-map-to-planning.md#판제-적용-전-확인--2026-10-20-코드-대조)와
[객체 확인표](../handoffs/perception-to-planning.md#판제-적용-확인표)로 인수 여부를 남긴다.

새 모델 연구·ego motion 보정·공통 BEV 재설계를 카메라 담당의 대회 전 필수 과제로 두지 않는다.
먼저 손재호의 현행 기하 검출·구조물 처리에서 객체 누락·벽 혼입·ID/속도 실패를 고정 입력과
실제 주행으로 확인한다. 시간·좌표·관측 부족·분리·연관 중 원인을 분리한 뒤 고전 기법으로
해결되지 않는 조건에서만 해당 학습/표현 후보 하나를 대조한다. 연구 브랜치의 실행 성공을
운영 채택이나3뷰 fusion 완료로 표시하지 않는다.

신호등은 [현재 ResNet BF16·TensorRT 검토](../../src/asmc_camera/docs/traffic-light-runtime.md)를
따르며 이번 문서 보강으로 모델·전처리·backend를 변경하지 않는다.

## 연결 문서

- [현재 카메라 구조](../../src/asmc_camera/docs/integration.md), [실행 계약](../contracts/camera-runtime-v1.md)
- [객체 에셋·라벨](../competition/obstacle-assets-and-labels.md), [데이터 준비](../operations/data-preparation.md)
- [카메라→LiDAR](../handoffs/camera-to-lidar.md), [음영 차선→판제](../handoffs/camera-to-planning.md)
- [대회 완료·담당별 수락](consolidation-roadmap.md)
- [과거 설계·실험·후보 비교](../records/camera/perception-roadmap-through-pr85.md)
