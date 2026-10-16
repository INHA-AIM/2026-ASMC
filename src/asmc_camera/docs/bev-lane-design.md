# 인지 GT·BEV·차로 구조화 계획

> **문서 역할:** Detailed Design Note — GT 정책 근거와 BEV/lane 후보 상세
> **현재 코드 진입점:** [Perception 개발·통합 가이드](./integration.md)
> **당시 우선순위 제안:** [perception-map-fusion-roadmap.md](../../../docs/architecture/perception-roadmap.md)
> **문서 허브:** [README.md](../../../docs/README.md)
> 기준일: 2026-09-14
> 담당: 안승현
> 상태: v6·3-camera BEV·ground-plane 보정과 typed handoff 완료, 음영구간 joint lane-family 확대 gate 통과
> 관련: [perception-dataset-labeling-split.md](../../../docs/contracts/dataset-labeling-split.md)

팀 전체 map-camera-LiDAR 구조, provisional topic과 대회판 이식 일정은
[카메라·MGeo·LiDAR 인지 융합 개발 로드맵](../../../docs/architecture/perception-roadmap.md)을 따른다.
이 문서는 GT 정책과 BEV/차로 구조화 세부 근거를 보존한다.
기준일 당시의 합의·보류·평가 상태를 보존한 설계 기록이므로 현재 package 실행법이나 동결
topic contract로 해석하지 않는다. 후속 구현으로 상태가 달라져도 원래 비교 근거와 수치는
삭제하지 않는다.

## 1. 현재 합의와 보류 항목

| 항목 | 현재 방향 | 상태 |
|---|---|---|
| 차선 GT | 가려진 차선을 복원하지 않고 보이는 차선만 positive | 합의 |
| 정지선 | MGeo/전역경로 정지점을 1차 사용; 영상은 all-view road-marking class로 보조 검증 | 적용·평가 중 |
| 3-camera BEV | RGB를 합쳐 재추론하지 않고 카메라별 segmentation 확률을 공통 ego-BEV로 투영 | 합의 |
| BEV 구현 시점 | 현재 v2 센서 profile로 BEV·vector 기준선을 먼저 완성하고 센서 경량화는 후속 통제 실험 | 합의 |
| 장애물 점유 | LiDAR-primary occupancy/tracking이 담당 | 합의 |
| drivable의 객체 픽셀 | actor-negative(B): 차량·보행자·`obstacle`을 non-drivable로 감독 | 잠정 채택 |
| 차로 변경 인터페이스 | mask와 별도로 ego/adjacent lane topology를 구조화 | 추가 설계 필요 |
| 추가 데이터 시나리오 | 주최 측 예제·제밍구간의 통제 변형 사용, 범용 랜덤 생성기는 보류 | 합의 |
| 정지선 배포 | MGeo/전역경로 정지점 1차, vision head는 보조·연구 확장 | 합의 |
| 공통 좌표 | 후륜축 중앙 `base_link`, x-forward/y-left/z-up; raster 범위는 overlay 후 결정 | 좌표 정본·범위 보류 |

개발 시각화는 열화 경계를 관찰하기 위해 `x[-5,30]×y[-12,12] m` exploration grid를
사용한다. 기존 `[-5,20]×[-8,8] m` 정식 pilot 기본값과 정확도 gate는 그대로이며,
개발 화면의 큰 직사각형을 판제 ODD로 해석하지 않는다.

BEV의 최종 구현은 센서 profile 동결 뒤에 하되, 카메라 calibration과 좌표계,
letterbox 역변환, 출력 메시지의 timestamp 계약은 데이터 수집 단계부터 보존한다.

## 2. Drivable GT 정책 비교

실무에서 `road`, `free space`, `drivable area`는 같은 말이 아니다. 프로젝트가 원하는
출력이 무엇인지에 따라 객체 픽셀의 정답도 달라진다. 하나의 절대 표준이 있는 것이
아니라, 정적 지도 요소와 동적 occupancy를 각각 예측한 뒤 융합하는 구성이 일반적이다.

| 정책 | 객체 픽셀 | 모델이 배우는 의미 | 장점 | 단점 |
|---|---|---|---|---|
| 관측 free-space | background, loss 포함 | 지금 영상에서 비어 있는 주행 가능 표면 | 카메라만으로도 차량·사람 위치에 구멍이 생김 | 도로 인식이 객체 인식과 결합되고, 일시적 점유 때문에 도로 형상이 흔들림 |
| amodal road surface | drivable positive | 객체 아래까지 이어지는 도로 지형 | 차로 구조가 연속적이고 BEV topology에 유리 | 현재 visible Semantic만으로 가려진 표면을 직접 알 수 없어 지도·시계열 보간이 필요하며 객체 위도 drivable로 출력 |
| 분리형 static surface | **ignore, loss 제외** | 보이는 정적 도로 표면만 감독하고 객체 점유는 별도 계층에 위임 | LiDAR occupancy와 책임이 분명하고 transient actor가 road 학습을 오염시키지 않음 | 객체 픽셀 출력은 감독되지 않아 단독 결과가 임의적일 수 있고, 최종 출력에는 occupancy 융합이 필수 |

초기 목표에는 세 번째 정책이 잘 맞았지만 통제 A/B에서 actor-negative(B)가 actor leak을
줄이고 전체 drivable/lane 성능도 회귀하지 않아 현재는 첫 번째에 가까운 B를 잠정
채택했다. 최종 안전 판단은 어느 정책에서도 카메라 drivable만 사용하지 않고 LiDAR
occupancy와 판제 문맥을 함께 사용한다.

### 2.1 Background와 ignore의 정확한 차이

binary target에서 객체를 “마스크에 포함하지 않는다”는 표현만 사용하면 픽셀 값은 `0`이
되고, 이는 보통 background 정답이다. 모델이 그 위치를 drivable로 예측하면 loss가
증가한다. 이것이 현재 v1 동작이다.

ignore는 별도의 유효성 정보가 필요하다.

| 픽셀 예 | drivable target | drivable valid | 학습 효과 |
|---|---:|---:|---|
| Asphalt | 1 | 1 | drivable로 학습 |
| Sidewalk/building | 0 | 1 | background로 학습 |
| Vehicle/pedestrian/obstacle 후보 | 0 또는 임의 | **0** | loss와 IoU 계산에서 제외 |
| Letterbox padding | 0 또는 임의 | 0 | loss와 IoU 계산에서 제외 |

구현 시 target에 `255`를 섞기보다 binary target `{0,1}`을 유지하고 head별
`drivable_valid_mask`를 별도로 저장하는 안을 우선한다. 기존 공통 `valid_mask`는 영상
ROI/padding만 표현하므로 객체 ignore를 나타내지 못한다. 최종 학습 마스크는 다음과 같다.

```text
drivable_train_valid = geometric_valid AND NOT drivable_ignore
lane_train_valid      = geometric_valid
stopline_train_valid  = geometric_valid
road_marking_valid    = geometric_valid
```

ignore는 객체를 자동으로 background나 drivable로 만드는 기능이 아니다. 해당 픽셀에
gradient를 주지 않을 뿐이다. 추론 결과를 BEV로 옮길 때에는 카메라 confidence/coverage와
LiDAR occupancy를 함께 사용해 객체 점유 영역을 최종 주행가능영역에서 제거해야 한다.
시뮬레이터 평가용으로는 같은 Semantic/Instance에서 `actor_occupancy_gt`를 별도 파생해
보존한다. 이것은 TwinLite drivable target이 아니라 LiDAR 점유·융합 결과의 privileged
GT이며, ignore mask와 동일한 개념이 아니다.

### 2.2 클래스별 초기 후보

- Drivable ignore: `vehicle`, `sedan`, `suv`, `truck`, `bus`, `van`, `wagon`,
  `mpv`, `pedestrian`, `obstacle`.
- 화면에 고정된 `ego_vehicle` hood/body는 background다. 큰 고정 영역을 actor ignore로
  버리지 않는다.
- Drivable background 유지: sidewalk, road edge, building과 도로가 아닌 고정 구조물.
- `standing_object`는 현재 K-City Capture 실측에서 가드레일·중앙분리대 부속물,
  가로등/카메라 지주, 문형 고정 구조물 등을 함께 포함했다. 도로의 영구 경계·시설물이므로
  기본은 drivable background로 둔다. 시나리오에서 스폰한 콘·차단물이 이 클래스로 섞이는지는
  별도 pilot에서 확인하며, MORAI의 독립 `obstacle` 클래스와 혼동하지 않는다.
- Lane/stopline은 visible-only이므로 actor 픽셀을 positive로 복원하지 않는다. 초기에는
  해당 픽셀을 일반 negative로 두고, 가림이 심한 데이터에서 경계 오차가 커질 때만
  head별 occlusion ignore를 별도 A/B한다.

## 3. 객체 포함 데이터셋 결정 게이트

### P0 — 학습 전 반드시 결정

1. 동일 위치에서 객체 없음/차량/보행자/장애물을 배치한 짧은 pilot run을 만든다.
2. Semantic 및 Instance 색상·가림 형태를 육안과 pixel count로 확인한다.
3. 위 클래스별 ignore 표를 동결하고 dataset policy ID에 기록한다.
4. `drivable_ignore_mask`와 평가용 `actor_occupancy_gt`의 클래스 구성을 각각 기록한다.
5. mask baker, cache, manifest, adapter, loss, metric이 같은 head-valid 정의를 쓰도록 한다.
6. 기존 객체 없는 v1은 보존하고 새 파생 데이터셋을 v2로 만든다.

2026-09-04 기준 1~4번은 8개 객체 포함 run, 14,529개 3-view sample의 전수 Semantic
감사로 완료했다. 미등록 팔레트는 없고 actor Semantic 픽셀과 Instance nonzero 대응률은
100%였다. `standing_object`는 터널·가드레일·지주 계열 고정 구조물, `obstacle`은 콘과
위험물/화염 소품으로 확인했다. 정책 ID는 `kcity_static_surface_actor_ignore_v2`다.
5~6번도 완료했다. 신규 8개 run의 4,843 frame에서 정차 근접 중복 891개를 비파괴
선별로 제외했고, 기존 4개 run을 포함한 8,667 frame/26,001개 3-view sample에 native
v2와 `384×640` cache를 만들었다. dataset adapter, 2-head loss와 metric은 head별 valid
mask를 사용한다.

### P1 — 통제된 추가 수집

- `weather × route/spot × traffic_density × actor_seed × controller`를 run metadata에 기록한다.
- seed 0은 객체 없는 대조군으로 유지하고 같은 route의 객체 seed와 짝지어 비교한다.
- train/val/test는 frame이 아니라 route/run/actor seed 단위로 격리한다.
- 정지선은 원거리 접근, 중거리, 정지 직전, 통과 후와 부분 가림을 구분해 수집한다.
- clear/fog 각각에서 직선, 곡선, 교차로, 합류, 차로 변경 구간을 포함한다.
- 현재 1 Hz Capture Mode는 단일-frame 학습에 사용하고 temporal 연구용 수집은 별도
  5~10 Hz profile과 timestamp/drop 검사를 둔다.
- 범용 랜덤 시나리오 생성기를 새로 만들지 않는다. 주최 측 예제 시나리오와 제밍구간을
  기준으로 NPC 밀도, 정적 장애물, 돌발상황, 맑음/안개만 통제 변형한다.
- 대회 우선순위는 제밍구간의 lane/drivable corridor 유지와 LiDAR 장애물 점유의 융합이다.
  합류·합류 차로 변경 데이터도 수집하지만 localization이 정상일 때는 MGeo link/node를
  1차 기준으로 사용한다.

### P2 — Drivable A/B 후 road-marking 확장

1. 기존 `best_mean.pt`를 신규 hard run에 먼저 추론해 재학습 전 기준치를 남긴다. **완료:**
   v4 독립 val 전체 평균 `0.8210`, front `0.7503`, foggy `0.7582`, foggy 3pm `0.7167`.
2. 현재 lane/drivable 2-head를 v4 train 26,001 samples로 warm-start 재학습하고,
   이후 별도로 수집한 sample-b val 1,404 samples에서 checkpoint를 선택한다. 첫 sunny 11am의
   날씨 전환 4 frame은 명시적으로 제외했고 원본은 보존했다.
3. 전체 평균 외에 view, weather, route type, actor density, occlusion bin별 지표를 낸다.
4. 같은 split/checkpoint에서 actor-ignore(A)와 actor-negative(B)만 바꿔 drivable 정책을
   먼저 선택한다. **완료:** B의 도로 문맥 actor leak `14.26% → 3.20%`, false-hole
   `+0.049%p`, 공통 GT drivable/lane IoU 비회귀를 확인해 B를 잠정 채택했다.
5. 선택 정책을 고정한 뒤 lane classifier를 `background/white/yellow/stopline` 4-class로
   바꾼다. 세 view 모두 정답/loss를 사용하고 class·view별 IoU와 recall을 낸다.
   **준비 완료:** v6 active manifest, 2→4 output adapter, multiclass loss/metric과 무학습
   실제 batch preflight까지 확인했다. **Tiny-overfit 완료:** sunny/foggy×3-view×3 foreground
   기준의 균형 18장에서 road-marking mIoU `0.0304 → 0.7919`, stopline IoU `0.8876`으로
   class mapping과 학습 배관을 확인했다. **본학습 완료:** 18 epoch best에서 drivable
   `0.98343`, road-marking `0.89555`, white/yellow/stopline 각각
   `0.87861/0.93277/0.87528`이다. Live backend는 task를 자동 감지하고 4-class argmax를
   `/perception/camera/{view}/road_marking/class_id`로 발행한다.
6. 정지선은 pixel IoU 외에 검출 recall, false positive/frame, front IPM 및 BEV 거리
   MAE를 평가한다.
7. 객체 ignore 영역을 제외한 segmentation metric과 LiDAR occupancy를 합친 최종 safe-area
   metric을 구분한다.

대회 배포에서는 MGeo/전역경로에 정지점을 두고 localization 오차보다 큰 안전 margin을
적용하는 방식을 1차로 검증한다. 정지선 head는 지도 오차·GPS 음영의 보조 수단이자 대회
이후 확장으로 유지하되, 제밍구간 lane/drivable 안정화보다 우선하지 않는다.
sample-b는 model selection용 val이다. 최종 성능 판단에는 재학습 이후 규정 시간
11am/1pm/3pm에서 새로 수집한 독립 sunny/foggy test run을 사용한다.

### P3 — 실시간 profile과 BEV

1. 세 카메라 live 입력으로 FPS, p50/p95 latency, UDP drop, GPU/CPU memory를 측정한다.
2. 현재 v2 센서 해상도·주기·압축률을 기준선으로 고정하고 먼저 camera-to-BEV/vector를 완성한다.
3. native GT loss budget으로 model-grid direct, native 복원과 hard/soft 융합 손실을 분리한다.
4. 검증된 표현만 ego-BEV에 옮긴다. 1차 audit에서는 model-grid hard direct가 개선됐고
   단순 4-class soft 평균은 gate를 통과하지 못했다.
5. LiDAR occupancy 차감과
   `safe_drivable` 생성 책임은 인지·퓨전·판제 인터페이스 협의 후 확정한다.
6. 제공 MGeo를 map prior와 camera-BEV 정합 평가에 모두 사용한다. 본선에서는 제공·허용된
   지도와 측위만 사용하고 GT/oracle sensor output을 배포 입력으로 누설하지 않는다.
7. vector 기준선까지 완성한 뒤에만 센서 해상도·10/15/20 Hz를 낮추는 부하 최적화를 수행한다.

**2026-09-11 보정:** MORAI `base_link` 후륜축 중심과 노면을 같은 `z=0`으로 취급했던
오류를 수정했다. 두 구간 LiDAR fit을 바탕으로 정적 노면 `z=-0.35 m`를 calibration의
필수 계약으로 두고 homography, model-grid evidence, 시각화와 offline 평가를 모두 같은
평면에 연결했다. 동일 validation 재평가에서 white/yellow 지표는 크게 개선됐지만 모든
범위 후보는 여전히 기준을 통과하지 못했다. 기존 `z=0` artifact는 비교용으로 보존하되
superseded로 표시하며, 현재 BEV 범위는 lane candidate 필터와 피팅을 다시 평가하기 전까지
provisional이다.

2026-09-10 pilot에서는 native 확대나 lane raster 없이 `384×640` model-grid pixel을 연속
`base_link (x,y)` evidence로 옮기는 경로까지 채택했다. 현 association+B-spline은 direct
evidence보다 white/yellow 거리 지표가 나빠지고 offline p95 지연도 목표를 넘어서 debug로만
유지한다. 이에 따라 `x[-5,20]×[-8,8] m`도 validated 범위가 아니라 provisional display
ROI다. ground-plane/letterbox calibration 검산 뒤 음영구간 rejection 원인 계측,
pseudo-GT 검산과 shared lane-family 대표 offline A/B까지 완료했다. 일반 구간의
MGeo candidate matching과 비차로 표식 제거는 그 뒤에 진행한다. 수동 검수를 마친 뒤 같은
native GT·hash 입력으로 범위를 다시 선정한다.

white/yellow class에는 같은 색의 주차칸과 안전지대 표식도 포함될 수 있으므로 색상 spline을
곧바로 차로 경계로 해석하지 않는다. 짧은 횡단 표식은 종방향 support로 제거하되, 평행한
주차선처럼 기하만으로 모호한 후보는 `unknown/non-lane marking` 상태로 보존한다. 60-frame
검수에서 false bridge와 비차로 표식 오검출을 별도로 집계하고, 후자는 2% 이하일 때만
lane-boundary/range gate를 통과한 것으로 본다. 기하·drivable 문맥이 부족하면 MGeo 또는
lane-boundary 전용 GT를 추가한다.

2026-09-12에는 비차로 표식이 없는 GPS 음영 왕복 2차로를 첫 ODD로 제한해 노란 중앙 경계와
양쪽 흰 outer boundary를 최대 3개로 선별하고 short-term stable ID를 부여했다. 12-run
replay에서 yellow 가용성은 clear/static `98.03/96.25%`, 세 경계 동시 완성률은
`82.95/71.27%`였다. 출력률과 기술 배관은 통과했지만 정적 장애물에서 right outer 누락이
남으므로 판제는 부분 관측·confidence·stale timeout을 전제로 개발한다. 인지는 경계의
색상·왼쪽→오른쪽 순서·metric polyline만 제공하며 home/opposing 및 회피·복귀 의미는
route/Frenet 문맥을 가진 판제가 결정한다. 일반 구간 MGeo matching과 재학습은 이 제한 ODD
handoff 뒤에 진행한다.

### P4 — 음영구간 joint lane-family 보강

현재 다음 세 현상을 분리한다.

- `/perception/lane/debug/markers`와 segmentation은 유지되지만 선택된 yellow boundary가
  빠지는 producer 후보선별 문제
- typed boundary는 유효하지만 `/planning/debug/camera_lane_strips`가 지워지는 consumer 또는
  Marker lifecycle 문제
- 급가감속 pitch와 차량 횡이동 때문에 실제 metric curve가 frame 사이에서 이동하는 문제

현재 producer는 confidence가 임계값보다 낮은 model-grid evidence를 먼저 제거하고, x-bin
기반 greedy association 뒤 가장 높은 yellow track 하나를 spline 전 선택한다. 각 spline은
독립 피팅되며 temporal tracker는 local ID만 보존하고 형상은 예측하지 않는다. consumer
visualizer도 매 frame `DELETEALL`을 발행한다. 따라서 모델 class mask가 안정적이어도 낮은
confidence의 얇은 연결부, track score 교체 또는 한 frame의 unusable 상태가 큰 깜박임으로
보일 수 있다. threshold `0.50→0.45` 단독 변경이 개선되지 않았으므로 임계값 완화만 반복하지
않는다.

첫 ODD의 map 근거는 제공 sample scenario의 `shadedAreaList`와 MGeo link
`A2256W000126/A2256W000131`이다. 두 link width와 중심선 간격은 약 3.5 m이며 음영 volume의
길이와 거의 일치한다. 단, 현재 배포 묶음에는 참조 lane-boundary의 실제 point geometry가
없으므로 link-derived 세 경계는 `map_pseudo_gt`로 기록한다. 생성 과정은 다음과 같다.

1. 두 반대방향 centerline을 같은 진행방향과 arc-length로 resample한다.
2. 대응 중심점 중간으로 yellow road-centre curve를 만든다.
3. 국소 접선에서 법선을 만들고 좌우로 width만큼 offset해 두 white outer를 만든다.
4. 음영 polygon 밖의 segment는 제거하고 source file/hash, link ID와 유효 s 범위를 저장한다.
5. 카메라 semantic 도색점과 LiDAR intensity 도색점으로 30~60 frame overlay 검산한다.

runtime 후보는 세 곡선을 별도로 피팅하지 않고 공유 cubic B-spline `C(s)`와 완만히 변하는
폭 `w(s)`를 추정한다. 관측 point와 각 역할 곡선 사이 robust weighted residual에 다음 soft
prior를 더한다.

- 같은 s에서 세 경계 접선의 평행성
- `w(s)`의 3.5 m map prior와 허용범위
- `w'(s)`와 spline curvature의 급변 억제
- source confidence·geometry quality·카메라 overlap 품질
- ego-motion으로 현재 frame에 옮긴 직전 lane-family와의 시간 연속성

세 선 관측 시에는 모두로 상태를 갱신한다. yellow+white 또는 white+white만 있으면 법선
offset으로 누락 경계를 `structure_inferred`로 만든다. 한 선만 남으면 최근 완전 관측과
ego-motion이 유효할 때만 짧게 갱신하며, 모두 사라지면 `temporal_predicted`를 최대
0.2~0.3초 유지한 뒤 invalid로 전환한다. 관측·구조추정·시간예측은 confidence, age와
유효거리도 서로 다르게 내보낸다.

새 학습 모델은 이번 gate에 넣지 않는다. 논문·코드의 차용 범위는 다음과 같다.

| 연구 | 차용 내용 | 이번에 도입하지 않는 것 |
|---|---|---|
| LaneCPP, CVPR 2024 | continuous spline의 평행성·곡률·노면 prior | 전체 3D lane network |
| Differentiable LSQ, ICCVW 2019 | confidence-weighted fitting, regularization | 구 ERFNet과 학습 layer 직접 복사 |
| RVLD, ICCV 2023 | 이전 결과를 현재 frame으로 정렬한 뒤 재사용 | learned flow/feature network |
| BézierLaneNet·BSNet | 연속 curve sampling과 local/global 표현 비교 | TwinLiteNet+ 교체 |
| LaneAF·StructLane | lane instance association과 구조 관계 | affinity/attention head 추가 |

첫 구현 단위는 shared-family runtime 출력 변경 전의 **진단·GT 정본화**다.

1. 단계별 rejection reason과 boundary 상태 전이, longest outage를 result bag에 남긴다.
2. Marker `DELETEALL`로 인한 화면 깜박임과 canonical message dropout을 분리한다.
3. `shadow_zone_kcity2025` 설정과 map pseudo-GT 생성/audit 도구를 만든다.
4. 기존 12-run+4 maneuver result를 새 지표로 재집계해 independent spline 기준선을 고정한다.
5. 이 단계가 끝나기 전에는 spline 수식, public message, planning fallback과 모델 weight를
   바꾸지 않는다.

2026-09-13 구현에서는 각 stage audit와 별도 result-bag 토픽, producer/consumer의 explicit
Marker DELETE, versioned shadow-zone 설정, pseudo-GT 생성·정합 audit를 추가했다. public
boundary 의미와 모델 weight는 유지했다. 최초 map pseudo-GT는 yellow 115.71m, link 간격
중앙값 3.511m이며 Sunny 13시 clear raw bag의 camera/LiDAR 각 48 frame 자동 gate를
통과했다. 이후 2026-09-12 음영 12-run의 LiDAR 도색 반사점으로 heading 후보마다 세
boundary offset을 다시 추정했다. joint 최적값은 음영 map pseudo-GT 전용
`ego_heading_offset=-1.25°`, road center `0.00m`, 양쪽 폭 `3.48/3.52m`다. 같은
48-frame audit의 경계 0.2m 이내 고반사점 비율은 `18.749%→37.854%`, 대조구간은 계속
`0.000%`였다. 이 offset은 차량 heading이나 camera extrinsic을 전역 변경하지 않는다.
정합 근거는 강화됐지만 paint survey가 아니므로 `deployment_allowed=false`다.

2026-09-14에는 clear Sunny/Foggy 각 1개, lateral sweep Sunny 1개, static avoid-return
Foggy 1개를 신규 `/perception/lane/pipeline_audit`와 함께 별도 result ID로 replay했다.
모든 result가 기술 gate를 통과했고 boundary/audit 토픽은 약 `9.57~10.25Hz`였다. clear
complete family는 `82.57/88.47%`, lateral sweep `74.05%`, avoid-return `83.50%`였다.
불완전 또는 yellow 누락 frame은 모두 `spline_residual_above_limit`와 동시에 발생했다.
반면 confidence 탈락은 적었고 association의 lateral rejection은 정상 complete frame에도
항상 존재해 단독 원인으로 볼 수 없었다. 따라서 다음 구현은 confidence threshold 완화가
아니라 세 경계를 함께 설명하는 weighted shared lane-family의 residual 안정화다.

2026-09-14에는 weighted shared lane-family를 별도 offline consumer로 독립 구현했다. 하나의
parametric cubic B-spline과 법선 방향 좌·우 offset으로 세 경계를 공동 설명하며,
`3.48/3.52m` 폭은 robust objective의 soft prior다. 한 선만으로 family를 만들지 않고 최소
두 역할이 있어야 누락 경계를 `structure_inferred`로 만든다. 기존 runtime boundary와 planning
토픽은 변경하지 않았다.

대표 4개 raw 1,255 frame 중 공지 음영 polygon 내부 508 frame에서 independent 대비 3경계
complete가 `78.74→100.00%`, yellow 관측/output이 `85.63→100.00%`로 개선됐다. map 기준
p95는 `0.3761→0.3775m`로 사실상 유지됐고, 거리별 p95는 0~10/10~15/15~20m에서 각각
`0.1844→0.1629`, `0.2954→0.2602`, `0.3997→0.3897m`였다. left reference-y jitter p95도
`0.0303→0.0213m`로 감소했다. 대표 정량 gate는 통과했다.

다만 기존 residual gate를 통과한 독립 spline과 기존에 버리던 track도 포함한 shared fit의
raw-evidence residual은 같은 모집단이 아니며, shared frame-level p95 분포가
`0.2844→0.5380m`로 커졌다. 따라서 현재 판정은 `promising_offline_candidate`이고 runtime
승격은 금지한다. 후속 대표 gate에서는 role curve로부터 0.35m 안쪽인 실제 metric support만
`observed`로 인정하고, 두 역할 이상의 local support와 최대 0.75m 내부 공백만 family
유효구간으로 남겼다. 단일 역할 원거리 꼬리는 `invalid`로 잘라냈다. 대표 1,200 frame의
고잔차 75 frame은 모두 outlier를 포함했고, 고잔차 overlay 54장에서는 명백한 횡단 false
bridge가 보이지 않았다. map과 비교 가능한 구조 추정 point 12,593개도 0.50m 이내였으나
이는 pseudo-GT 기반 review signal일 뿐 정답 판정이 아니다.

동일 후보를 12+4 전체 matrix 4,075 frame으로 확대했다. ODD 3경계 complete는
`84.38→100.00%`였다. 서로 다른 출력 표본을 단순 합친 map p95는 `0.4013→0.4077m`지만,
같은 run·frame·role의 paired p95를 run별 층화하면 16개 run이 모두 개선됐고 변화 중앙값은
`-0.0582m`, 전체/거리 bin 최악 악화는 `+0.0193m`였다. maneuver 4개에서는 map p95가 약
10cm 개선됐고, inferred sample 43,038개 중 0.50m 초과 map 불일치는 9개였다. 확대 gate는
통과했다. shared support state는 planning 출력과 분리한 debug runtime sidecar로 이식했다.
다음은 직접 주행에서 상태 전환과 깜박임을 검수하고 shared fitting 연산을 최적화한 뒤
temporal 보상 없이 판제 closed-loop를 먼저 확인한다. 전 구간 MGeo candidate matching,
대회판 fine-tuning과 센서 경량화는 음영구간 closed-loop handoff 뒤로 둔다.

직접 RViz 검수 후 우선순위를 조정했다. `observed`는 정상 후보,
`structure_inferred`는 강등된 제한 후보이며 `invalid`는 family 곡선이 차선에 맞아 보여도 local
support가 없으므로 판제가 사용하지 않는다. 4개 maneuver result bag의 `1,126` frame에서 family
accepted rate `100%`, 최장 reject 연속 0 frame을 확인했다. 곡선 sample 중 usable은 `77.18%`,
그중 구조 추정은 `13.21%`였으며 invalid `22.82%`는 항상 제외한다. shared fit p50/p95는
`39.68/51.00ms`, 출력률은 run별 `8.857~9.277Hz`였다.

현재 shared geometry를 runtime 기준선으로 동결하고 동일 출력 비회귀 조건의 연산 최적화와
temporal 없는 판제 A/B를 먼저 수행한다. 두 run이 9Hz 목표를 근소하게 놓쳤으므로 debug
sidecar를 곧바로 정식 인터페이스로 승격하지 않는다. 해당 출력률은 기존 live perception과
GPU/CPU를 공유한 보수적 개발 부하 수치이므로 standalone 조건에서 재측정한다. Ego-motion과
IMU 자세 보정은 판제가
dropout을 문제로 요청하거나 나머지 기능 완성 뒤 필요성이 측정될 때만 별도 옵션으로 구현한다.

기존 temporal tracker의 대표 replay stable-ID 교체는 left 1회, yellow 6회였다. shared의
역할 label은 구조적으로 고정되므로 이를 곧바로 shared ID switch 0회로 비교하지 않는다.
실제 곡선 identity와 형상 연속성은 runtime 기준선에서 먼저 평가하고, 필요할 때만 여는
ego-motion temporal 단계에서도 별도로 재평가한다. offline Python
대표 audit p50/p95 `77.34/95.03ms`, 확대 audit `73.04/90.63ms`는 공통 전처리와 두 방법을
모두 포함하므로 runtime latency가 아니다.

## 4. MGeo·시나리오 자동화 활용 범위

MGeo node/link는 학습 시 privileged GT와 평가 기준으로 유용하다. link의 주행 방향,
좌우 차로 변경 가능 여부와 목적 link, lane boundary geometry/속성 등을 이용해 다음을
자동 산출할 수 있다.

- ego가 속한 GT link/lane과 좌·우 인접 lane ID
- 차로 변경 가능 방향과 구간
- GT lane boundary/centerline을 ego 좌표계로 변환한 polyline
- 교차로 접근 link와 정지선 후보 위치 및 ego-to-stopline 거리
- route type별 수집 coverage와 예측 topology 정확도

정지선은 Semantic mask를 학습 정답으로 유지한다. MGeo의 node/traffic-light 연결 또는
lane boundary 속성으로 얻은 위치는 거리 평가와 수집 구간 생성에 사용한다. K-City 2025
파일에서 정지선 geometry와 시뮬 좌표가 정확히 대응하는지는 소량 overlay로 먼저 검증한다.

참고 레포는 직접 runtime dependency로 묶지 않고 설계를 선별 이식한다.

| 참고 레포 | 재사용 가치 | 선행 확인 |
|---|---|---|
| `external/team/traffic_runner` | K-City 2025 MGeo 접근 route, SUNNY/FOGGY×시간×object seed 매트릭스, progress 재개, manifest 패턴 | 단일 신호등 카메라 recorder를 Capture Mode 3-view trigger와 run 단위 저장에 맞게 교체 |
| `external/team/aim_ws-jang/aim_scenario_runner` | rolling NPC, 인접/반대 차로 배치, 보행자·횡단보도 관리, route randomization | 현재 `R_KR_PG_K-City`/구 MGeo 설정을 K-City 2025 API·ID·모델 목록으로 호환 검증 |

이식 전에는 dry-run으로 조합 수와 좌표를 출력하고, 한 route/한 seed에서 spawn·despawn,
Ego 보호, Capture trigger 동기화, cleanup을 확인한다. 자동화가 만든 모든 run에는 config
snapshot/hash, map/version, seed, actor 목록·초기 pose, weather/time, route/link IDs,
controller, sensor-set hash를 저장한다.

## 5. BEV와 차로 변경 인지 패키지 초안

차로 변경 판단이 mask를 다시 해석하지 않도록 raster와 vector/topology 출력을 함께 둔다.

```text
3-camera RGB
  -> shared drivable + 4-class road-marking segmentation
  -> camera probabilities + per-view validity
  -> calibrated ego-BEV fusion
  -> lane-color metric evidence
  -> lane-boundary candidate discrimination
  -> vector extraction
  -> ego/left/right lane association
  -> LiDAR occupancy/tracks association
  -> perception message
```

### 5.1 Raster 계층

- `drivable_probability`, `road_marking_probability`와 class별 파생 확률
- camera coverage/validity와 fusion confidence
- LiDAR occupancy는 별도 layer로 유지하며 `safe_drivable` 차감 소유권은 판제·퓨전과 합의
- BEV frame ID, resolution, origin, width/height, timestamp

### 5.2 Vector/topology 계층

- ego lane ID(로컬 임시 ID), centerline, left/right boundary, width, confidence
- left/right adjacent lane 존재 여부와 각 centerline/boundary
- boundary type/color와 crossing 허용 여부는 vision 관측값과 MGeo 참고값의 source를 구분
- 각 lane의 longitudinal valid range, age, occluded ratio
- stopline polyline, ego 경로와의 교점 거리, confidence, age
- 카메라 source bitmask와 LiDAR track association ID

일반 debug spline은 여전히 위 vector/topology 계약이 아니라 `lane candidate`다. 다만 첫
음영구간 ODD에서 선별한 경계는 `asmc_msgs/LaneBoundaryObservationArray`로 typed provisional
handoff를 구현했다. 이 메시지는 N개 경계와 색상·순서·metric polyline만 표현하고
home/opposing/절대 차로 번호는 담지 않는다. 전 구간 표식 분류와 topology 계약은 별도다.

인지 파트는 관측된 차로 구조·경계·신뢰도와 장애물 association을 제공한다. “지금 차선을
변경한다”는 최종 결정은 route, 규정, gap acceptance, LiDAR TTC를 결합하는 판단 파트가
내린다. GPS/IMU 음영에서는 차량 원점과 연결된 drivable component 및 시간적으로 추적한
lane centerline으로 ego lane을 유지하고, MGeo lane ID는 유효 pose가 돌아왔을 때 재연결한다.

### 5.3 다음 설계 회의에서 확정할 항목

1. 후륜축 중앙 `base_link` 검증과 실제 overlay 기반 BEV 범위/해상도
2. provisional boundary observation에서 전 구간 lane centerline/topology 계약으로 승격할 조건
3. confidence, age, occlusion, invalid 상태의 의미와 timeout
4. 끊긴 차선·교차로에서 topology 유지 규칙
5. 카메라 확률 융합과 LiDAR occupancy 우선순위
6. MGeo ID를 배포 메시지에 포함할 조건과 localization 불확실성 표현
7. 차선 변경 판단 파트가 요구하는 최소 look-ahead와 인접차로 길이

## 6. 당장 개발하지 않는 항목

- BEV 최종 grid와 전 구간 topology ROS custom message 확정
- learned view transform 또는 temporal BEV
- 카메라 객체 검출 head 추가
- 카메라 패키지 내부에서 MGeo bake/local crop까지 직접 구현하는 기능

이 항목들은 4-class live profile, map-camera overlay와 판단 파트 인터페이스 요구가
확보된 뒤 진행한다.

## 7. 규정 v1.1 해석 메모

과거 v1.1 검토 메모다. 신호 채점 수치는 v1.2 후속 공지(팀 반영일 2026-10-06)로 아래에 갱신했으며,
현재 규정 정본은 [대회 요약](../../../docs/competition/README.md#51-교차로-신호준수-후속-공지--2026-10-06-반영)을 따른다.

- “바퀴 4개가 중앙선·실선 등 주행 영역을 벗어나거나 주어진 경로 이탈”은 차로 침범
  패널티 기준이 아니라 운영요원의 **주행 불능 판정** 기준이다.
- 별도 차로 준수 채점은 실선·중앙선 접촉이 3초 지속될 때마다 5초 패널티이며,
  침범 판정은 **바퀴 1개 기준**이다. 따라서 4개 바퀴가 모두 이탈하지 않으면 차로
  패널티가 없다는 해석은 맞지 않는다.
- 교차로 신호는 차량 앞바퀴가 정지선을 넘을 때의 신호로 판정하고 미준수 시 **30초**
  패널티다(기존 15초에서 변경). 판정 대상 교차로 4개소 모두 미준수하면 해당 차시 실패이며,
  동일 교차로는 체크포인트 복귀 후 재통과도 1회만 집계한다.
  지도 기반 정지점을 쓰더라도 ego 기준점과 앞바퀴 위치, MGeo 정지선 오차를
  실측한 뒤 안전 margin을 정한다.
