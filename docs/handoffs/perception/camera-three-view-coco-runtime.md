# 3뷰 COCO7 통합 실행·모델 선정

이 문서는 #73·#74·#75의 누적 작업을 흡수한 통합 작업의 시작점이다.
작업 worktree는 `asmc-camera-objects`, 브랜치는 `feat/camera-three-view-coco-runtime`이다.
`2026-ASMC`는 main checkout, `asmc-camera-verify`는 rename 재현용이다.
컨테이너 `/root/ws`가 어느 worktree를 읽는지 확인한 뒤 실행한다.
폴더명과 Git 브랜치명은 다르며, build/source만으로 마운트 대상은 바뀌지 않는다.

## 사용자 기본 모델 확정과 남은 작업 (2026-10-09)

사용자가 실시간 좌/전방/우 mosaic의 정상 동작을 확인하고 **YOLO26l detect /
전방1280 / 좌우640×480 / TensorRT FP16 / 2+1 batch / 뷰별15Hz /
confidence0.40 / 3뷰 Ego mask ON**을 개발 기본 모델로 확정했다.
현재 COCO7 출력은 원본 7개 라벨을 유지한다. 이 확인은 아래 실패를 면제하거나
PR #76 병합·전체 스택 운영 수락을 승인한 것으로 확대하지 않는다.

남은 작업은 다음 순서로 진행한다.

1. **현재 통합 PR 마무리:** 최종 L detect/0.40으로 주·음영 bag의 fusion 반복
   검증, 입력/출력 공백 원인 분리, 장시간 전체 스택 검증을 수행한다. 기존 판제
   trajectory deadline 실패와 카메라 공급 5/6 통과 기록을 보존한다. 호스팅 CI
   실패도 해결·확인하며, 현재 전방 fusion의 `unique` 기본 정책은 유지한다.
2. **대회 장애물 확대:** COCO7이 지원하지 않는 상자·드럼통·바리케이드·유모차
   등을 YOLOE의 검토한 실제 객체 이름으로 비교한다. 구조물 오검출을 일으킨
   prompt를 무작정 복원하지 않는다. 네 소비 그룹은 이 후속 실험에서 원본 라벨과
   별도로 검토하고, 현재 COCO7 출력 정책은 유지한다.
3. **기존 데이터로 추가학습:** Capture의 RGB/semantic/instance/BBox 대응과
   세부 라벨을 검수하고 AIM 라벨 가능 범위를 감사한다. teacher 자동 라벨은
   검수 전 초안으로 취급한다. 비교 결과로 모델 하나를 선정해 학습하며 일반
   구간 성능과 새 장애물 성능을 함께 확인한다.
4. **후속 통합:** `bounded_reuse`의 별도 수락, 3카메라 기하·시간 연관과 중복
   관측 처리, 판제의 준비/활성화 연결, 4070 SUPER의 engine 재생성·장시간
   검증을 진행한다. 현재 운영 fusion은 전방 bbox이며 3뷰 mask fusion은 미구현이다.

TensorRT·batch·3뷰 mask·모델별 상태 서비스·mosaic은 이미 구현돼 있다.
기존 `perception.launch`의 모델 기본값 승격은 별도 적용 단계이며 현재 선택은
`camera_coco7.launch`로 사용한다. LiDAR 구조물·군집·OBB·속도 개선은 재호에게
인계한 과제이고, 구간 결정·차선 결과 소비는 판제 담당이다.

## Confidence 후속 선정과 M detect/1280 대비 차이

모델·해상도·batch는 L detect/1280·3뷰15Hz·2+1을 유지하고, 생산자 컷만
**0.50에서0.40으로 변경**했다. 기존 YOLO11m 설정은0.70·전방·4클래스였으므로
숫자 컷이 같은 검출 품질을 뜻하지 않는다. 아래 M/L 비교는 이전 동일0.25 추론을
동일0.50으로 후필터링한2250장 결과와 같은 회차 C6 선별값이다.

| 지표 | M detect/1280 | L detect/1280 | 차이 |
|---|---:|---:|---:|
| TP |282|326|+44 (+15.6%)|
| FP |20|28|+8|
| macro F1 |0.2335|0.2676|+0.0342|
| 높이20px 이상 GT recall |74.5%|81.4%|+7.0%p|
| 동일 회차 C6 wall p95 최댓값 |73.3ms|88.4ms|+15.1ms|
| 동일 회차 전체 VRAM peak |5570MiB|5753MiB|+183MiB|
| 공통 안정 검출 객체의 최초 시점 |기준|먼저8·동일16·늦게1|추가 획득2|

L1280을 각각 실제 predictor 컷0.30·0.40으로2252장 재추론했다. 기존 실제0.50과
같은 GT·weight·mask·RGB SHA를 확인했다. 아래 표는 사용자가 검토한 정확히2장만
제외한2250장 결과이며 원본2252장 점수도 보존한다. TP/FP는 프레임별 박스 수다.

| 실제 predictor 컷 | TP | FP | precision | macro F1 | 높이20px 이상 recall |
|---|---:|---:|---:|---:|---:|
| 0.50 |326|27|92.4%|0.2677|81.4%|
| **0.40** |**352**|**52**|**87.1%**|**0.2793**|**84.3%**|
| 0.30 |369|89|80.6%|0.2867|87.0%|

0.40은0.50의 정답을 잃지 않고26개를 회복하지만 FP25개가 추가된다.
추가 FP는 다른 검수 라벨과 겹치는13개·중복1개·GT 미대응11개다.
전체52개는 라벨 혼동28·중복3·미대응21개다. 미대응을 모두 배경이라고 확정하지
않으며, 예를 들어 먼 사람 검출이 GT와 IoU0.5를 못 넘은 사례도 있다.
0.30은0.50 대비 정답43개 회복·FP62개 추가다. 수학적 F1은0.30이 가장 높지만
0.40을 조기 검출과 오검출의 개발 절충값으로 선택했다.

[전체 오검출·회복 정답 갤러리](http://localhost:8768/camera-coco7-live-v1/confidence-followup-v3/index.html)는
컷·오류 종류·0.50 대비 추가 여부로 필터한다. 모든 FP와 회복 GT를 보여주며,
[터널 배경 truck0.402](http://localhost:8768/camera-coco7-live-v1/confidence-followup-v3/0166.jpg)은
판제에서 단순 클래스·confidence만으로 사용하기 어려운 사례다. 이 실패를 카메라
측 후속 보완 대상으로 남긴다. 근거리 실제 객체를 함께 지울 수 있는 크기·위치 규칙은
추가하지 않았다. 이번 컷 선정으로 전체 스택 운영 수락이나 FP 안전성을 주장하지 않는다.

0.40 C6 확인은 SUNNY/FOGGY 각3회다. YOLO14.65–14.95Hz, p95≤97.7ms,
전체 GPU peak5541MiB, 중복·역행0이었다. 카메라 공급은5/6회 통과하고,
SUNNY r3은 YOLO 최대 공백319.3ms·TwinLite427.6ms·ResNet356.5ms로 실패했다.
같은 ACTIVE 약4.3초 지점에서 전방/측방 입력 공백288.7/350.2/266.5ms,
camera4 입력386.8ms, resource sampler 호출356.0ms가 겹쳤다. 입력 stamp 간격도
늘었으므로 단순 YOLO 연산 과부하로 확정하지 않는다. cutoff가 원인이 아니라고
입증한 것도 아니다. 모든 실행의 판제 trajectory deadline 실패와 이 신규 공백을
보존하고 **0.40은 개발 선정, 전체 운영 수락은 보류**한다. 반복을 선별해6/6 통과로
표시하거나 공백 gate를 완화하지 않았다.

동일0.40 mask ON/OFF2252장에서는 전방1·좌측1·우측0 신규 class-aware 누락과
전방1·좌측1 회복이 있었다. 전방000270은 car0.404가 사라지고 truck0.510이
동일 형상에 남은 라벨 누락이며, 좌측000233은 높이11px 차량 박스 소실이다.
이는 이전0.50의 실패2장과 다른 관측이며 따로 보존한다. 사용자 선택에 따라 mask ON을
유지하되 신규 실패0 수락은 주장하지 않는다. 고정0.40으로 이미 열람한 heldout2474장도
재확인해 TP977/FP68/macroF1 0.280890이었다(이전0.50 TP899/FP34/F1 0.268085).
이것은 컷 선정 후 회귀이며 새로운 독립 시험 자료가 아니고 재튜닝에 쓰지 않았다.

최신 수치·원본 SHA·실패는 [confidence 후속 증거](../../../config/baselines/evidence/camera_coco7_live_v1/confidence_followup_decision.json)를 따른다.

## 현재 기본 모델: 8개 조합 재검토

현재 개발 기본값은 **YOLO26l detect / 전방1280 / 좌우640×480 / TensorRT FP16 /
2+1 batch / 뷰별15Hz / confidence0.40**이다.
`camera_coco7.launch`가 `config/camera/yolo26l_detect_coco7_three_view.yaml`을 선택한다.
기존 M detect 설정과 M/L seg 비교 설정은 재현·롤백용으로 보존한다.

초기 M-seg/960 추천은 당시의 높은 confidence·기존 FP 상한·근접 차량 보존과
3회 통합15Hz 결과, 향후 mask 활용 가능성을 함께 고려한 잠정안이었다.
그 후 mask ON/OFF에서 신규 누락이 확인돼 운영 수락되지 않았다. 뒤이은 M detect/1280
추천도 M detect/1280과 L-seg/960의 비교 결과였다. 이번에는 그 두 후보에 한정하지 않고
M/L × detect/seg ×960/1280 **8개 조합을 같은0.50 기준으로 다시 비교**했다.
과거 추천을 모든 조건의 최적 모델이었다고 해석하지 않는다.

Capture selection2252프레임의 기존 실제0.25 추론 결과를 재사용해0.50에서 선별했다.
사용자가 운영 비교에서 제외한 기둥·잘못 스폰한 차량 바퀴는 RGB SHA로 고정한
정확히2장만 제외했다. 같은 실행의 다른 프레임·종류 전체를 제외하지 않았고
원본 점수와 실패 이미지는 변경하지 않았다. 아래는 **2250장 보조 비교**다.

| 모델 | 전방 너비 | TP | FP | macro F1 | 이번 C6 선별 YOLO wall p95 최댓값 |
|---|---:|---:|---:|---:|---:|
| M detect |960|285|22|0.2265|74.6ms|
| M detect |1280|282|20|0.2335|73.3ms|
| L detect |960|318|21|0.2563|89.8ms|
| **L detect** |**1280**|**326**|**28**|**0.2676**|**88.4ms**|
| M seg |960|278|24|0.2359|91.2ms|
| M seg |1280|250|19|0.2362|98.3ms|
| L seg |960|300|20|0.2372|102.5ms|
| L seg |1280|315|53|0.2485|104.2ms|

TP/FP는 프레임별 class-aware IoU≥0.5 대응이다. 검수 GT는 person/car/bus/truck만 있으며,
dog/bicycle/motorcycle의 AP를 이 자료로 주장하지 않는다. GT2807개 중2242개는
높이10px 미만이다. 높이20px 이상 recall은 L detect/1280이81.4%, M detect/1280이74.5%다.
평균 F1을 대회 전체 객체 정확도라고 확대하지 않는다.

공통으로 안정 검출된 Capture 객체에서 L detect/1280은 M detect/1280보다
8개는 먼저,16개는 같은 입력,1개는 늦게 안정 검출했다. L-seg/960과는8/17/2개다.
이는 연속 주석 입력5개 중3회 대응의 source frame 기준이며 wall 시간 비교가 아니다.
두 모델에서 서로 다른 객체 집합의 평균 최초 검출 시간을 직접 비교하지 않는다.

같은 SUNNY/FOGGY의12·30·50m 정지 스폰42개 장면을 각5장·3뷰2+1으로 실제 재추론했다.
전방 중앙 검토 영역에 기대 라벨이3장 이상 관측된 장면은 L detect/96030개,
L detect/128029개, M detect/128026개, L-seg/96028개다. 해당 박스가 배치한 actor에
정확히 속하는 pixel GT가 아니므로 recall/AP로 쓰지 않는다. 개·자전거·이륜차의
원거리 미검출은 남고, 추가학습 전 모든 대회 객체를 지원한다고 하지 않는다.

8개 조합은 MORAI+TwinLite+ResNet+LiDAR/fusion+판제/제어 preview에서 각1회 선별했다.
이후 유력한 L detect/960·1280만 SUNNY/FOGGY 각각3회 반복했다. 더 낮은 품질·더 큰
비용의 후보까지 전체 반복을 확대하지 않았다. L detect 두 구성은12회 모두 카메라 공급과
15Hz 목표를 통과했다. L detect/1280의6회는14.75–14.90Hz, wall p95≤95.6ms,
p99≤109.5ms, 최대 공백201.7ms, 중복·역행 stamp0이었다. 전체 VRAM peak는
6회5514MiB, 앞선 선별까지 포함하면5753MiB로90% 상한 이내다. L detect/960은
14.75–14.95Hz·p95≤91.4ms였다. 이 정도 지연 차이보다 검수 검출률·조기 검출을 우선했다.
원래150/200ms gate를 완화하지 않아도 후보가 들어왔으며 미래 최적화 효과를 가정해
실패를 통과로 바꾸지 않았다.

L-seg/1280 선별의 출력 공백 약398ms에는 센서 입력 공백341–358ms도 겹쳤다.
이를 seg 모델의 계산 과부하로 확정하지 않는다. 품질·오검출·비용의 종합 근거로
추가 반복 우선순위를 낮췄다. M detect/960의 첫1초 TwinLite8개 실패도 원본에 남긴다.
모든8개 선별과12회 확인에서 판제 trajectory deadline은 실패했다. 따라서 이 선정은
**카메라 개발 기본값이며 전체 스택 운영 수락은 보류**다. 카메라 외 실패의 원인을
완전히 분리했다거나30분 연속 실행을 통과했다고 주장하지 않는다.

최종 실제0.50 재추론은 원본2252 selection 프레임에서 TP326/FP27/macroF1 0.267638이었다.
Capture 정답 평가는 뷰별 단일 입력이고, 실제3뷰2+1 경로는 스폰 재추론·통합 실행으로
별도 확인했다. 동기화된 측방 GT의 batch1/2 완전 동등성을 증명한 것은 아니다.
두 검토 프레임 제외2250장에서는 TP326/FP27/macroF1 0.267669였다. 저장된0.25 결과를
후필터링한 선별표의 FP28과 실제 predictor 경계 처리는 구분한다. FP27 중17개는
다른 검수 라벨의 객체와 겹쳤고10개는 미대응 박스였다. 잘못된 raw 라벨도 FP로 유지한다.
이미 이전에 열람한 heldout2474장에서는 고정0.50으로 TP899/FP34/macroF1 0.268085였다.
이 자료는 새 독립 최종 시험이 아니라 회귀 확인이며 모델·컷 재선정에 사용하지 않았다.

같은2252장의 mask ON/OFF에서 class-aware 신규 실패는 전방1·좌측1·우측0이다.
전방000323의 차량은 OFF의 car0.510이 컷 아래로 떨어지고 ON에서 같은 위치의
truck0.535가 남았다. 따라서 객체 박스 자체 소실과 라벨 소실을 구분한다.
좌측000067의 truck0.547은 ON에서 사라졌다. 반대로 전방 대응1개는 회복됐다.
**신규 라벨 누락0 조건은 통과하지 않았다.** 다만 사용자가 요청한 자차 오검출 억제
우선순위로3뷰 mask ON을 유지하고 두 실패를 이미지와 원본 예측으로 남긴다.
이를 과거 보행자 예외의 자동 적용이나 모든 후속 누락의 면제로 해석하지 않는다.

최신 원본 SHA·집계·판정은 [8개 조합 선정 증거](../../../config/baselines/evidence/camera_coco7_live_v1/broad_model_decision.json)에 모았다.

### 갤러리 한곳에서 보기

[통합 갤러리 목차](http://localhost:8768/)에서 검색·실험 종류 필터로 카메라와 LiDAR
갤러리를 모두 찾는다. [8개 조합 요약](http://localhost:8768/camera-coco7-live-v1/broad-model-summary-v1/index.html)과
[같은 RGB 모델 비교](http://localhost:8768/camera-coco7-live-v1/broad-model-spawn-inspection50-v1/comparison.html)를
최근 링크에 둔다. 각 제목의 E 번호는 재생성해도 바뀌지 않는다.
시각은 최초 목차 등록 당시 갤러리 HTML의 수정 시각을 KST로 고정한 것으로,
실험 시작 시간·bag source stamp와 구분한다. 최신 갤러리 시각순으로 정렬한다. 기존8766/8767 개별 주소는 유지한다. 재생성은 저장소에서
`python3 tools/perception/build_experiment_gallery_hub.py --artifacts ../artifacts`다.
서버 재시작은 같은 위치에서 `python3 -m http.server 8768 --bind 127.0.0.1 --directory ../artifacts`다.
로컬 artifact가 없는 다른 PC에는 원본 증거 파일도 별도로 전달해야 한다.

### 직접 실행·confidence

```bash
roslaunch asmc_camera camera_coco7.launch profile:=debug
```

```bash
rqt_image_view --clear-config /perception/camera/objects/debug/mosaic/compressed
```

좌·전방·우 YOLO 결과는 `/perception/camera/objects/debug/mosaic/compressed` 한 화면에
나란히 표시한다. 각 칸은 원본 비율을 유지하며 source stamp·age와 차체 mask를 표시한다.
뷰 간에는 최신 결과를 독립적으로 갱신하므로 하나의 동기 관측으로 소비하지 않는다.
입력이0.5초 이상 오래되면 해당 칸의 박스를 지우고 STALE로 표시한다.
합성 영상 header는 합성 시각이며 원본 관측 stamp는 각 칸에 따로 남는다.
화면은 최대10Hz, YOLO 추론 목표는 뷰별15Hz다. rqt가 없으면 합성용 구독·디코딩을
하지 않고, 기존 개별 뷰 토픽은 그대로 사용할 수 있다. YAML·모델·confidence는 변경하지 않는다.

YOLO만 확인하려면 같은 launch에 `start_camera_lane:=false start_traffic_light:=false`를
추가한다. 기본 debug는 TwinLite·YOLO·ResNet 모두 ACTIVE이며 LiDAR·판제·차량 제어는
시작하지 않는다. 차선 피팅 결과의 주행 소비는 음영 ODD로 한정한다.

컨테이너·bridge는 별도로 준비한다. planning은 세 모델 STANDBY, debug는 ACTIVE다.
차선 피팅 결과의 주행 소비는 계속 음영 ODD로 한정한다. 판제는 구간과 소비를 결정한다.
`objects_conf_thres:=0.55`로 생산자 시작 컷을 바꾸거나 YAML의 `conf_thres`를 수정한다.
현재0.40은 위의 실제 재추론으로 선정한 개발 절충값이다. 이전0.50 선정의 원본 증거는 유지한다.
0.30의 F1은 더 높지만 FP가 크게 증가한다. 판제의 메시지 소비 컷과 어댑터 추가 컷은
독립적이고, 생산자가 제거한 낮은 confidence 결과를 소비자에서 복원할 수는 없다.
3뷰 차체 mask는 기본 ON이다. 자차 오검출을 억제하는 사용자 선택을 유지하고,
새 모델의 ON/OFF 차이도 최신 증거에 남긴다. 이전 누락 예외로 신규 실패를 지우지 않는다.
기존 `perception.launch`의 운영 모델 기본값은 독립 수락 전까지 유지한다.

## 이전 두 후보 선정 기록 — 아래 결과는 보존하며 현재 기본값은 위 절을 따른다

실행·추가 개발 기준 모델은 **YOLO26m detect / 전방1280 / 좌우640×480 /
TensorRT FP16 / 2+1 batch / 뷰별15Hz / 생산자 confidence0.50**으로 결정했다.
설정은 `src/asmc_camera/config/camera/yolo26m_detect_coco7_three_view.yaml`이다.
원본 전방 영상은1280×720이며 tensor는736×1280이다. raw7종과3뷰 Ego mask를 유지한다.
L-seg/960은 비교 설정으로 보존하며, 운영 기본값 승격·PR 전체 수락과 모델 결정은 구분한다.

같은0.70 컷에서 SUNNY·FOGGY 각각3회, MORAI+TwinLite+ResNet+LiDAR/fusion+
판제/제어 preview를 비교했다. 제어는 시험 토픽만 사용했고 실제 자동주행은 하지 않았다.

| 후보 | 3뷰 처리율 범위 | 입력 수신→출력 wall p95 최댓값 | 전체 GPU peak VRAM |
|---|---:|---:|---:|
| M detect /1280 |14.75–14.95Hz|84.4ms|5322MiB|
| L seg /960 |14.45–14.90Hz|115.5ms|5428MiB|

동일0.70의 Capture selection 검수 박스 대응은 M의 TP227/FP2/macroF1 0.2000,
L의 TP236/FP4/macroF1 0.1930이다. car/truck/person/bus만 검수 GT가 있으며
작은 박스가 많다. L의 TP가 더 많다는 점도 보존한다. 낮은 컷0.50의 저장 결과
screening에서는 L300/22, M282/20으로 L의 전체 대응 수가 더 많았다.
M의 절대적인 인식 우위를 주장하지 않고, 약31ms의 통합 지연 여유·15Hz 유지·
근거리 개와 검토한 person 오검출을 함께 고려해 선택했다. 현재 운영 fusion이
전방 bbox를 소비하므로 seg mask의 후속 활용 이득을 이미 얻었다고 계산하지 않는다.

두 후보 모두 카메라 공급 gate를 통과했지만 L은 일부5초 창에서15Hz 목표의95%에
미달했다. 전체 스택은 두 후보 모두 trajectory deadline 실패를 유지한다.
L은 먼 승용차에 유리했지만, 새12/30/50m 스폰에서 가까운 Dog1을 놓쳤고 Capture의
기둥(person0.857)·차량 바퀴(person0.649) 오검출도 남았다. M은 가까운 개와 일부
원거리 트럭에 유리했다. 새로운42개 장면에서0.70 기준 기대 라벨을5장 중3장 이상
관측한 장면은 두 후보 모두21개였다. 이는 검토한 중앙 영역의 관찰이며 pixel GT
recall/AP나 실제 접근 주행의 최초 검출 시각이 아니다. 거리도 Ego 스폰 anchor 기준이다.
일부 매우 가까운 차량의 부분 관측은 두 모델 모두 놓친다. 단일 사전학습 모델로
대회 객체 전체를 수락한 것은 아니다.

confidence는 모델 결정 이후 **selection 자료에서만** 정했다.0.60의 mask 대조에서
front/000277 차량이 OFF0.602→ON0.587로 떨어졌고,0.55에서는 front/000336 차량이
OFF0.566→ON0.543으로 떨어졌다. 최종0.50은 이 두 전방 차량을 보존한다.
2252 selection 프레임의 TP/FP는0.70의227/2에서 **실제0.50 추론의281/18**로 바뀐다.
낮은 컷으로 저장한 결과를0.50 이상으로 필터링한 screening은282/20이었다.
실제 predictor의 confidence 경계 처리와 후필터의 포함 조건은 구분하며 실제 재추론을
최종 수치로 사용한다. FP18 중14건은 실제 객체와 겹치는 raw 라벨 혼동이며4건은
미대응 박스다. 모두 영상으로 개별 검토하고 라벨 혼동도 FP로 유지한다.
이 수치는 반복 프레임의 raw-label 대응이며 고유 객체 수가
아니다. confidence는 정답 확률로 보장되지 않는다. 과거의 YOLO11m FP 상한만으로
컷을 정하던 정책에서, 사용자가 요청한 조기 검출·근접 객체 보존을 우선한 절충으로
변경했으며 증가한 FP를 면제하거나 삭제하지 않는다.

0.50에서도 mask 신규 대응 실패는 좌측 차량1·우측의 일부만 보이는 person1건이다.
이전 GT287 보행자 예외와 다르므로 자동 승인하지 않는다.0.60/0.55 실패도 보존한다.
기존 heldout은 이미 과거에 열람한 자료이므로 이번에는 고정 컷의 회귀 검증으로만
사용한다. 이 점수에 맞춰 컷을 다시 조정하지 않는다.

최종0.50 설정으로 SUNNY·FOGGY 각각3회 전체 스택 preview를 다시 측정했다.
뷰별14.70–14.95Hz, 입력 수신→출력 wall p95 최댓값84.6ms, p99 최댓값94.9ms,
최대 공백197.1ms, 모델 로드부터 포함한 전체 GPU peak5717MiB(8188MiB의69.8%)였다.
6회 모두 카메라 공급·15Hz 목표 gate를 통과했으며 판제 trajectory deadline은
6회 모두 실패로 남았다. MORAI 렌더링과 사용자의 배경 작업을 포함한 해당 실행의
관측값이며, 순수 TensorRT 연산 시간이나30분 안정성 승인으로 해석하지 않는다.

최종 설정의09-21 카메라 bag·주/음영 fusion bag은 격리 master11314에서 각각1회
연결 검사를 했다. 카메라·음영 실행은 통과했고 주 bag은14.91Hz였으나 출력 공백
340ms로 공급 gate가 실패했다. 같은 구간 입력 최대 공백94ms, 해당 출력 wall 지연
271ms, 리소스 조회 소요311ms가 관측됐다. VRAM 고갈은 없었으나 지연의 원인을
GPU/OS 스케줄링으로 확정하지 않는다. 원본 실패와 `primary/gap-diagnostic.json`을 보존한다.
두 fusion 실행 모두 단일 발행자·종료 집계·offset≤50ms는 통과했다. 한 번의 smoke를
반복 매칭 수·분류 동등성 수락으로 확대하지 않는다.

`asmc-ros-noetic` 안에서 직접 실행한다. 새 launch의 기본 profile은 planning(STANDBY)다.
debug는 세 모델을 ACTIVE로 시작하고 debug 영상을 발행한다. rqt는 별도로 연다.

```bash
roslaunch asmc_camera camera_coco7.launch profile:=debug
```

```bash
rqt_image_view --clear-config /perception/camera/objects/debug/image/compressed
```

좌우는 `/perception/camera/objects/debug/left/image/compressed`,
`/perception/camera/objects/debug/right/image/compressed`다.
LiDAR는 nested 환경을 source한 별도 터미널에서 `roslaunch Main main.launch
start_yolo:=false rviz:=false`로 연결한다. 같은 master11311을 사용한다.
기존 `perception.launch`·`camera_objects.launch`의 YOLO11m 기본값은 전체 수락까지 유지한다.

### 판제에서 조절하는 범위

- **생산자 최소 컷:** 위 YAML의 `conf_thres` 또는 launch의 `objects_conf_thres`다.
  `roslaunch asmc_camera camera_coco7.launch profile:=debug objects_conf_thres:=0.55`
  처럼 명시한다. 개별 `camera_objects.launch`에서는 `conf_thres:=0.55`다.
  시작 시 검증하며 재시작해야 적용된다. 실행 중 camera private param만 바꾸는 것은
  backend hot reload가 아니다. 컷 아래에서 이미 버린 박스를 소비자가 복원할 수 없다.
- **판제의 소비 컷:** `CameraObjectBox.confidence`를 직접 사용해 필요에 따라 더
  엄격하게 조절한다. 현 비교 범위의0.50–0.70을 출발점으로 삼되 구간 정책은 판제가
  결정한다. 실제 변경값은 소비 결과와 함께 검증한다.
- **LiDAR 호환 어댑터의 추가 컷:** Main의 `camera_objects_conf_thres` 기본0.0은
  생산자가 승인한 후보를 그대로 통과시킨다. 숨은0.7 컷으로 신규 후보를 잃지 않는다.
  private `/camera_objects_yolo_bbox_adapter/conf_thres`는 캐시 파라미터로 실시간 변경한다.

```bash
rosparam set /camera_objects_yolo_bbox_adapter/conf_thres 0.65
# 생산자의 후보를 다시 모두 통과
rosparam set /camera_objects_yolo_bbox_adapter/conf_thres 0.0
```

추가 컷은 이후 수신하는 관측에 적용한다. 기존 track의 확인된 분류를 즉시 삭제하거나
과거 결과를 재발행하지 않는다. 비유한 값·범위 밖 값은 거부하고 어댑터 입력을 보류한다.
관측 header·좌표·MD5·50ms 매칭·200ms 대기·unique 기본 정책은 유지한다.

비교 증거는 artifacts의 `finalist-decision-*`, `finalist-selected-*`에 모은다.
`finalist-decision-analysis-v1.json`은 두 모델의 같은 컷 비교와 모든 반복의 실패를
함께 담는다. 비교 갤러리는 `finalist-decision-inspection07-v1/comparison.html`,
최종 컷은 `finalist-decision-inspection50-v3/comparison.html`, mask 대조는
`finalist-selected-mask-controls50-v3/index.html`, FP 전수 검토는
`finalist-selected-confidence-review50-v3/index.html`이다. 로컬 gallery server8766에서 연다.
후속 모델은 이 M-detect 기준선과 동일 입력으로 비교한다.

최종 증거와 원본 SHA 목록은
[`finalist_model_decision.json`](../../../config/baselines/evidence/camera_coco7_live_v1/finalist_model_decision.json)에 고정했다.
바깥/nested 빌드, 관련 단위·계약 시험176개와 validator6개는 통과했다.
실제 ROS 어댑터의 컷 상승·복구·무효값 보류는 XML1개 시험(실패0·오류0)으로 확인했다.
초기 `--text` 시험의 seq assertion 실패와 실행기 경로/검사 오류도 원본에 보존한다.
ROS publisher마다 seq를 새로 붙이므로 wire에서는 원본 stamp·frame_id를 검사하며,
순수 변환 시험에서는 복사한 전체 header를 검사한다. facade17개 중 기존1건 실패와
P1-C7개 중 기존1건 오류는 그대로 남아 있다. GitHub CI는 조직 billing 제한 해소 전
통과로 표시하지 않는다. PR#76은 Draft를 유지한다.

## 구현과 수락 상태

구현과 비교 시험은 완료했으며 운영 수락은 보류 중이다. 기존 기본 모델을
새 3뷰 후보로 승격하지 않는다. 과거 YOLOE vocabulary·seg·실패 증거는 유지한다.

- raw 출력: `person, dog, bicycle, car, motorcycle, bus, truck`.
  새 프로필은 `class_name == source_class_name`이며 네 그룹으로 매핑하지 않는다.
  과거 group 프로필은 별도로 보존한다. 메시지 필드·MD5는 유지한다.
- 전방·좌우의 각 Ego polygon mask를 기본 적용한다. 원본 JPEG·화질·좌표·header를
  바꾸지 않고 YOLO 입력 복사본만 가린다. TwinLite·ResNet 입력은 유지한다.
- TensorRT detect/seg 공통 backend는 정확한 engine SHA·task·head·CUDA를 검사한다.
  실패 시 CPU/PyTorch로 조용히 전환하지 않는다. seg mask 복원 비용도 측정한다.
- 원본 전방 1280×720에서 tensor 너비 640/960/1280을 비교한다.
  대응 tensor 높이는 384/544/736이다. 좌우는 640×480을 유지한다.
  batch3에서도 좌우를 확대하지 않고 padding한다.
- `target_hz_by_view`는 뷰별 목표다. 모드 서비스의 `target_hz: 0`은 이를 유지하고,
  양수 요청은 모든 뷰를 해당 주기로 통일한다. 처리 완료 결과를 복제하지 않는다.
- TwinLite 목표는 뷰별 10Hz, 입력 tensor 640×384다. YOLO와 Hz를 강제로 맞추지 않는다.
  차선 결과의 주행 소비는 음영 ODD에 한정한다.

## 시험 도구·증거

실험 조건은 [camera_coco7_live_v1.yaml](../../../config/experiments/camera_coco7_live_v1.yaml)에 둔다.
원본 증거는 상위 workspace `artifacts/camera-coco7-live-v1`, 모델은
`data/models/camera_coco7_v1`이다. 전체 checkpoint·engine을 Git에 넣지 않는다.

1. `prepare_coco7_models.py`: checkpoint/ONNX/engine SHA, 라이브러리·dynamic shape 기록.
2. `capture_coco7_actors.py`: gRPC 실제 목록 대조, owned actor 촬영·정리.
   actor 3D 박스를 검수한 영상 GT로 간주하지 않는다. 요청 z와 실제 생성 z가
   다를 수 있으며 pedestrian SetTransform은 이 버전에서 지면 상대 z다.
   공중 배치 v1/v2 실패도 보존한다. 실제 위치와 영상 검토를 함께 사용한다.
3. `coco7_scene.py`: 7종·3뷰에 객체가 있는 고정 연산 부하 장면.
   기존 actor는 유지하고 registry에 생성 성공이 기록된 시험 actor만 정리한다.
4. `evaluate_coco7_models.py`: 고정 selection/heldout 영상, raw 클래스별 일대일 IoU 0.5.
   세부 종류가 미확인인 Vehicle/Obstacle 영역은 TP·FP로 임의 판정하지 않는다.
   GT가 없는 종류는 평가 불가다. confidence sweep과 원본 예측·mask·갤러리를 보존한다.
5. `run_coco7_live_trials.py`: 모델 3개 상주, PREPARE 실제 영상 준비 확인 후 ACTIVE,
   wall 입력 수신→출력 수신 지연·뷰별 공급률·200ms 리소스 기록.
   모든 실패를 보존하고, 최소 실행 가능성과 목표 Hz 달성을 구분한다.
   최신 측정기는 모델 로드·예열부터 GPU 메모리를 수집하고 90% gate를 적용한다.
   `resources`는 ACTIVE 구간, `resources_startup_and_active`는 로드·준비 포함이다.
   이전 실행의 ACTIVE 메모리로 시작 시점 peak까지 검증했다고 해석하지 않는다.
6. `analyze_coco7_input_gaps.py`: 실패한 출력 공백과 같은 시점의 원본 카메라
   공급 공백을 연결한다. 입력 공백이 설명된 실행도 원래 수락 실패는 유지한다.
7. `analyze_coco7_mask_control.py`: 동일 RGB SHA의 전체 mask ON/OFF 결과를
   같은 GT·confidence로 비교하고 신규 누락 전부를 이미지 갤러리에 남긴다.
8. `analyze_coco7_fusion.py`: 완전한 종료 집계, 고유/재사용 관측, 트랙별
   시간적 분류 확인 중복과 빈 결과 이후의 과거 관측 사용을 검사한다.
9. `summarize_coco7_live_matrix.py`: 실험 버전·모델·해상도·주기·batch별 세 반복을
   함께 판정한다. 실패 반복을 제외하지 않으며 미완성 묶음을 수락하지 않는다.
   잘못된 코드 SHA로 중단한 실행은 `REJECTED_PROVENANCE.json`과 원본을 보존하고
   정상적인 모델 성능 표본에서 제외한다.

`near-object-gallery/index.html`은 같은 RGB의 960/1280과 PyTorch/TensorRT를
비교하는 갤러리다. 파랑은 GT, 초록은 검출이며 slider는 저장된 예측을 필터링한다.
선정 자료의 일부 가까운 차량은 960에서 검출되지만 1280에서 미검출됐다.
해상도 확대를 무조건 품질 개선으로 간주하지 않는다. `summarize_coco7_geometry.py`의
GT 픽셀 높이별 평가는 크기별 진단이며 실제 거리·정량 거리 정확도가 아니다.

heldout 평가는 `evaluate_coco7_models.py --split heldout --frozen-confidence ...`로
선정 자료에서 정한 점수만 사용한다. heldout에서 점수를 다시 탐색하지 않는다.
Capture의 현재 검수 GT에는 car/truck/person/bus만 있으므로 dog/bicycle/motorcycle을
포함한 7종 전체의 정량 수락으로 확대하지 않는다. gRPC actor의 3D 상태와 영상
검토 갤러리는 나머지 종류의 관찰 자료이며 독립적인 픽셀 GT를 대체하지 않는다.

다섯 모델 × 전방 너비 세 가지의 같은 gRPC 촬영 RGB 비교는
`actor-all-models-fixed07/comparison.html`에서 두 모델을 선택해 확인한다.
각 프로필은 지면 배치 검사를 통과한 97개 fixture를 사용한다. 배치·스폰 실패
21개도 원본에 남겨 두었으며, actor의 기대 클래스가 검출됐다는 이유로 같은
actor에 정확하게 대응됐거나 픽셀 GT recall을 통과했다고 계산하지 않는다.
갤러리는 214개 같은 RGB 표본에 박스·원본 라벨·confidence·instance mask를 표시한다.

### Mask 수락 보류

960 입력의 YOLO26m-seg, confidence 0.65, 2+1 batch, 세 뷰 15Hz는 전체 스택
세 반복에서 처리 기준을 통과했다. 그러나 같은 RGB 2,252장의 black mask ON/OFF
대조에서 전방 1·좌측 1·우측 4건의 신규 GT 누락이 나왔다. 윤곽 안쪽 4픽셀과
중성색 96을 적용한 선택적 v2 후보는 기존 사례를 회복했지만 다른 전방 1·우측 1건이
생겼고 오검출 4건으로 같은 너비의 기준선 3건을 초과했다. detect 960/0.7의
black mask에서도 좌측 1·우측 1건이 나왔다. 기존 전방 누락 승인과 별개다.

부분 장면에서 회복됐다는 이유로 v2를 기본값으로 적용하지 않는다. v1의 black
설정과 실패 원본을 유지하며 v2는 명시적 실험 프로필이다. `coco7_masks.py`의
`--mask-profile black-v1|neutral-v2`로 동일 데이터 대조를 재현한다. OFF는 원본을
수정하지 않고 빈 제외 영역을 반환한다. v2의 fill/inset과 실제 raster 윤곽은 진단에
기록된다. 선정되지 않은 설정을 heldout 결과에 맞춰 다시 조정하지 않는다.

`full-mask-controls-selection/index.html`,
`full-neutral-mask-controls-selection/index.html`,
`full-detect-mask-controls-selection/index.html`에서 모든 신규 누락을 직접 확인한다.
`yolo26m_seg_coco7_three_view.yaml`은 처리 성능 후보이며 운영 수락된 기본값이 아니다.

실측에서 GPU의 박스 필드를 하나씩 Python scalar로 읽는 비용이 컸다. 박스를
한 번에 CPU로 옮기고, TwinLite가 구독자 없는 대형 debug 영상을 직렬화하지
않도록 수정했다. 클래스·좌표·순서·source header와 GPU 추론 결과는 유지한다.
이 수정 전후 실행은 별도 증거로 보존하며 수정 전 실패를 삭제하지 않는다.
수정 후 PyTorch–TensorRT 동일 입력 270개 비교도 다시 통과했다.

## 전방 fusion 재사용 실험

`Main main.launch fusion_yolo_policy:=unique|bounded_reuse`로 비교한다.
두 방식 모두 ±50ms와 200ms 대기를 유지한다. 결과를 재사용할 때도 해당 LiDAR
관측으로 다시 투영·기하 연관한다. 같은 source stamp/epoch를 같은 track의
분류 확인·miss에 두 번 반영하지 않는다. 정상 빈 관측 이전의 검출을 되살리지 않는다.
STANDBY·세대 변경·producer 종료·전방 결과 침묵은 버퍼를 무효화한다.

fresh/reused 매칭 수를 분리한다. 총 매칭 수 증가만으로 객체 정확도 개선을 주장하지 않는다.
카메라 ID·source stamp·epoch·보정 식별자는 내부 관측에 보관하고 투영에 보정값을
명시적으로 전달한다. 이번 운영 연결은 전방 bbox뿐이다. 3뷰 mask fusion은 후속 구현이다.
legacy classification의 car 범주와 별도로 `yolo_class`에는 원래 7종 이름을 유지한다.

정상 빈 결과는 유효한 시간 관측이다. 최신 빈 결과는 이전의 비어 있지 않은 박스를
막지만, 시간 범위 안의 이전 빈 관측까지 지우지 않는다. 최초 재사용 시험에서는
모든 과거 stamp를 막아 음영 장면의 빈 관측을 대량으로 거부하는 결함이 확인됐다.
`FusionTimeMatcher.hpp::EligibleObservationStamp`와 실제 matcher 시험으로 이를
분리했다. 수정 전 실행은 `fusion-coco7-10hz-v2`에 보존하며 운영 수락으로 세지 않는다.
앞선 `fusion-coco7-10hz`는 뷰별 15Hz가 10Hz 요청을 덮어쓴 설정 오류다. 원본과
`REJECTED_PROTOCOL.json`을 보존한다. 실행기는 run-local YAML에 scalar와 뷰별 주기를
함께 고정하고 ROS의 실제 파라미터도 확인한다.

동일 영상이 여러 관측에 쓰이므로 `analyze_fusion_trace.object_state`는 trace 순서상
이미 적용된 최근 LiDAR 갱신을 연결한다. 미래의 재사용 갱신으로 과거 상태의 원인을
덮어쓰지 않는다. 기존 연속 C++ 재현기는 unique 소비 기준으로 보존하며, 그것의
통과를 reuse 정책 전체의 증명으로 확대하지 않는다. reuse의 실제 기록은 별도
`analyze_coco7_fusion.py`로 고유 증거·카운터 중복·빈 결과 barrier를 검사한다.

수정 후 10Hz 전체 반복은 `fusion-coco7-10hz-v3`에 있다. 주 bag의 unique 매칭은
404/394/400쌍, reuse는 455/455/457쌍(재사용 71/75/72쌍)이었다. 음영 bag은
unique 173/171/169쌍, reuse 173/174/167쌍이며 실제 재사용은 0쌍이었다.
음영 reuse의 세 번째 반복은 약 314ms 출력 공백으로 실패했다. 영상 입력 최대
공백은 약 92ms였으므로 센서 침묵으로 면제하지 않는다. 주 bag의 기존 세 실행에서
분류가 안정된 공통 관측 2,512개 중 새 정책에서 분류 차이 1개가 남았다.
음영의 안정된 공통 관측 1,001개에서는 차이가 없었다. 미대응 관측도 따로 기록한다.
숫자 track ID 대신 동일 LiDAR source stamp와 상호 유일한 1mm 중심 대응으로
비교했으며, 이 통계는 독립 GT의 정확도나 추적 recall이 아니다.

15Hz 대조의 주 bag은 unique 477/466/473쌍, reuse 478/478/484쌍이었다.
음영은 unique 177/178/175쌍, reuse 176/178/178쌍이며 여기서도 실제 재사용은
0쌍이었다. 안정된 공통 분류는 주 2,206개·음영 936개에서 차이가 없었지만,
주 unique 두 번째 실행에서 세 뷰 모두 307–321ms 출력 공백이 발생했다.
같은 시기 GPU 사용률과 계측 소요시간의 변화는 상관 관측이며 원인 확정이 아니다.
10/15Hz의 trace ON 재사용 검사는 총 8회 모두 카운터 중복·정책 위반 없이
완전한 종료 집계를 통과했다. 이를 trace OFF 처리율·분류 수락의 대체로 쓰지 않는다.

전체 실시간 스택의 15Hz reuse 대조 3회 중 첫 실행은 모든 모델의 공급 공백과
판제 객체 공급 deadline에서 실패했다. 나머지 두 실행의 통과로 이 실패를
제외하지 않는다. `live-finalist-reuse-v3`와 입력 공백 분석을 함께 보존한다.

`compare_coco7_fusion_trials.py`는 두 bag의 두 정책 각각 세 반복을 모두 요구한다.
실패 반복·분류 차이·미대응을 보존하고 설정 오류로 거부된 실험은 비교 입력으로
받지 않는다. 결과는 각 실험의 `policy-comparison.json`으로 남긴다.

```bash
python3 tools/perception/compare_coco7_fusion_trials.py \
  ../artifacts/camera-coco7-live-v1/fusion-coco7-10hz-v3 \
  --output /tmp/coco7-fusion-comparison.json
```

재사용은 선택적 실험 기능이며 기본 정책은 `unique`다. 총 매칭 수의 증가와
출력 지속성·분류 상태 수락은 각각 판단한다.

## Jamming 연결

`lane_boundary_adapter camera_bbox_detector.launch` 기본은 공유 카메라 전방 결과의
Msg/BBox 변환이다. 별도 YOLO를 실행하지 않는다. `detector_source:=legacy`는
과거 21% crop 생산자를 명시적으로 켠다. 기존 주행·차선 변경 튜닝값은 유지한다.
새 소비 경로는 crop 0과 동일 Ego mask를 사용하고, 박스 하단 접점을 차체가 가리면
박스를 보존한 채 UNKNOWN/UNOBSERVED로 보고한다. 이런 박스를 삭제하거나
이전 차로 소속을 근거 없이 유지하지 않는다.

## 직접 실행

컨테이너 진입·bridge와 rqt는 별도다. 모든 ROS 터미널에서 같은 outer/nested 환경과
master를 source한다. 전용 빌드의 devel 경로와 공유 workspace devel을 혼용하지 않는다.

공유 `asmc-ros-noetic`은 현재 worktree를 `/root/ws`로 연결한 고정
Ultralytics 8.4.150 / TensorRT 10.13.0.35 이미지다. 바깥 workspace를 먼저
빌드·source하고 nested workspace를 빌드했다. 실제 CUDA 연산·engine 예열·생성
메시지 MD5와 `/vehicle/state` 50Hz, 세 카메라 20Hz·camera4 10Hz 입력을 확인했다.
컨테이너 재생성 전 셸에서는 나와 다시 진입한다.

실제 팀 주행에서는 YOLO·TwinLite·ResNet·Main·판제 노드를 이 컨테이너의
서로 다른 터미널에서 실행한다. 추가 추론 컨테이너는 필요하지 않다. 같은
workspace source와 master를 사용하며 TensorRT engine은 `/data/models/camera_coco7_v1`에서
읽는다. 별도 검증 컨테이너는 재현·네트워크 격리에 사용한 시험 환경이다.
이미지 이름에 YOLOE가 포함돼 있어도 일반 YOLO detect/seg backend를 함께 제공한다.
표준 `docker/ros-noetic/Dockerfile`에도 동일 CUDA11 TensorRT·export 의존성을
`requirements-yolo-runtime.txt`로 고정한다. 팀 표준 ROS 이미지를 다시 빌드할 때
추론 의존성도 함께 설치되며 별도 컨테이너로 YOLO를 분리하지 않는다.
설치 블록은 팀 ROS 기본 이미지 위에서 기존 공식 wheel 캐시를 재사용해
실제 빌드하고 CUDA 연산·detect/seg 예열·3뷰 추론을 확인했다. upstream
`ros:noetic`부터의 전체 재빌드는2.3GB PyTorch 재다운로드 단계에서 중단했다.
새 설치 블록 검증과 전체 upstream 이미지 재빌드 완료를 구분한다.
팀원 장비에서는 해당 장비의 TensorRT/CUDA 환경에서 engine을 재생성하고 SHA를
설정한다. 4060 Ti에서 만든 engine의 다른 GPU 호환성을 전제하지 않는다.

```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source src/lidar+sensorfusion/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=localhost
```

```bash
roslaunch asmc_camera camera_objects_three_view_experiment.launch config:=/선정한/runtime.yaml
```

현재 후보의 객체 단독 확인은 아래와 같다. 후보 설정의 `target_hz_by_view`와
`front_width`, `conf_thres`를 수정하고 launch를 재시작해 비교한다. 목표 주기 서비스의
0은 파일의 뷰별 설정 유지, 양수는 모든 뷰의 주기를 같은 값으로 바꾼다.

```bash
roslaunch asmc_camera camera_objects_three_view_experiment.launch \
  config:=/root/ws/src/asmc_camera/config/camera/yolo26m_seg_coco7_three_view.yaml
```

TwinLite·신호등까지 화면을 확인하려면 앞선 객체 launch를 종료하고 실행한다.
이때 차선의 주행 소비는 계속 음영 ODD로 제한한다.

```bash
roslaunch asmc_camera perception.launch profile:=debug \
  camera_objects_config:=/root/ws/src/asmc_camera/config/camera/yolo26m_seg_coco7_three_view.yaml \
  objects_views:="[front, left, right]"
```

별도 LiDAR 터미널에서는 후보의 confidence를 어댑터에도 명시한다.

```bash
roslaunch Main main.launch start_yolo:=false rviz:=false \
  camera_objects_conf_thres:=0.65 fusion_yolo_policy:=unique
```

```bash
rqt_image_view --clear-config /perception/camera/objects/debug/image/compressed
rqt_image_view --clear-config /perception/camera/objects/debug/left/image/compressed
rqt_image_view --clear-config /perception/camera/objects/debug/right/image/compressed
```

위 실험 launch의 debug는 영상 발행이며 rqt 자동 실행이 아니다.
일반 `profile:=debug`는 객체 overlay 3개·mosaic·BEV·신호등의 6개 debug 출력이다.
`objects_debug_masks:=true`는 instance mask 영상 3개를 추가한다. 9개 영상 발행
부하는 별도 실험에서 세 실행 모두 실패했다. 6개 출력 대조는 모델 공급은
세 실행 모두 통과했지만 판제 trajectory deadline은 두 실행에서 실패했다.
debug OFF의 headless 공급 수락과 GUI 디버그 성능을 구분한다.
3뷰 raw7 공급 검사는 아래 도구를 사용한다. legacy
`verify_perception_runtime.py --camera-objects`는 전방 18Hz 계약 검사이므로
3뷰 저주기 후보 검사로 대신 사용하지 않는다.

```bash
python3 tools/perception/verify_coco7_runtime.py --seconds 20 --minimum-hz 9.5
```

9.5Hz는 **10Hz 목표 프로필**의 공급 허용치이며 LiDAR 주기와 일치해야 하는
물리적 조건이 아니다. 8/9Hz를 별도 목표로 시험할 수 있지만 기존 실패를
9.5Hz에서 낮춰 통과 처리하지 않는다. 새 관측 지연·객체 공백·fusion의 시간
대응과 분류 이력을 함께 평가한다. 낮은 주기용 합격 기준은 별도 프로필에서 정한다.

이상적인 균일 입력에서도 8Hz/9Hz 영상의 최근접 시간차 최댓값은 각각62.5ms/
55.6ms다. 현재 ±50ms matcher에서는 시간 위상에 따라 대응하지 못하는 LiDAR
관측이 생긴다. unique는 카메라 관측을 한 번만 소비하므로12Hz LiDAR 대비
8/9Hz 영상의 대응 횟수 상한은 약67%/75%다. 이는 손실 없는 이상적 입력의
계산 예시이며 실제 시험 결과가 아니다. bounded_reuse는 같은 관측의 기하
연관을 다시 검사하지만 새로운 영상으로 만들거나 허용 offset을 늘리지 않는다.
TwinLite와 YOLO의 추론 주기는 독립적이다.

전체 스택 계산 시험의 `coco7_preview_stack.launch`는 명령을 `/evaluation/ctrl_cmd*`로
격리한다. 실제 주행은 별도 gate·저속 시험이며 preview 성공으로 대체하지 않는다.
`drive_coco7_preview.py`는 이미 준비된 전체 preview를 5km/h 이하에서 확인하는
시험 도구다. preview 조향과 유효 trajectory의 속도를 사용하되 종방향 명령은
type2 속도 상한으로 변환한다. 이는 수정 없는 MPC 구동 성능 검증과 구분한다.
단일 공개 제어 발행자, 모델·영상·판제 입력 신선도, 충돌·경로 이탈·속도 상한을
감시하고 종료 후 Ego pose·gear·제어 모드를 확인한다. 실제 이동이 0.5m 미만인
명령 시도는 주행 성공으로 기록하지 않는다. 기존 actor·시나리오는 변경하지 않는다.

복원은 실제 정지·위치·roll/pitch/yaw·기어·제어 모드를 세 번 확인한다.
현재 MORAI에서는 `SetTransform`이 제어 모드를 AUTO로 바꾸고 약 1.34° yaw
차이를 만들 수 있었다. 관측된 오차를 한 번만 보정하고 모드·기어를 마지막에
복원한다. 임시 Ego pause는 명시적으로 해제하며 원래 pause 상태까지 복원했다고
주장하지 않는다. 복원 실패 시 다음 물리 시험을 중단한다. 최초 복원 실패·추가
회복 RPC는 `ego-restoration-recovery.json`에 남겨 두었다.

UDP 저속 대조에서는 `127.0.0.1` 및 Windows 호스트 목적지 모두 명령과 실제
전진이 맞지 않았다. 세 번의 작은 역방향 이동은 주행 성공으로 세지 않는다.
원래 자세·P 기어·키보드 모드로 복원했으며 기존 actor는 유지했다. UDP 송신 성공
로그는 MORAI의 제어 수신 증명이 아니다. gRPC 제어 진단은 별도 transport로
실행하고 UDP를 동시에 송신하지 않는다. 이 진단의 성공도 UDP/MPC 통합 구동
승인으로 확대하지 않는다.

gRPC API의 proto는 velocity 단위를 명시하지 않는다. 최초 진단에서 ROS의
km/h 숫자를 그대로 전달하자 실제 속도가 약3.6배로 접근했고5.5km/h guard가
중단했다. `drive-grpc-v1` 실패·복원 결과를 보존한다. 시험 gate에서만 m/s로
명시 변환한 `drive-grpc-mps-v2-r1/r2/r3`은 모두 전진·정지·복원과 세 모델의
재진입을 통과했다. 실제 전진거리는1.22–1.25m, 최고 속도1.77–1.88km/h다.
판제 STOP과 더 낮은 속도 명령은 존중했으며 실제5km/h 주행이나 UDP 수신·
원래 MPC actuator의 수락으로 확대하지 않는다. 운영 bridge 단위는 변경하지
않았다. `drive-grpc-mps-summary.json`이 세 실행과 모든 stream의 결과다.

## Sunny·foggy 재검토

날씨별 [추가 검토 요약](../../../config/baselines/evidence/camera_coco7_live_v1/weather_followup_summary.json)은
기존 전체 비교를 sunny/foggy로 분리한다. `analyze_coco7_weather.py`는 동일
RGB SHA·완전한 GT와 예측 coverage를 확인하고 원래 confidence를 유지한다.
Capture 날씨는 run 이름의 태그이며 독립적인 날씨 telemetry는 저장돼 있지 않다.
반면 새 실시간 시험은 각 suite 전후 gRPC GetWeather로 날씨를 확인했다.

별도 heldout 2,474프레임에서 selection에서 고정한 confidence를 재사용했다.
Sunny는 car/person/bus, foggy는 car/person/bus/truck의 GT가 있어 서로 다른
날씨의 macro F1을 직접 비교하지 않는다. dog/bicycle/motorcycle은 평가 불가다.

| 후보 | confidence | sunny macro F1 / FP | foggy macro F1 / FP |
|---|---:|---:|---:|
| YOLO11m /960 |0.70|0.0429 /0|0.1072 /1|
| YOLO26m detect /960 |0.70|0.0623 /0|0.1607 /2|
| YOLO26m detect /1280 |0.70|0.0662 /0|0.1869 /1|
| YOLO26m-seg /960 |0.65|0.0502 /0|0.1846 /4|

검증 GT의 픽셀 높이 중앙값은 sunny2px/foggy4px이며 sunny529/647개,
foggy2,213/3,383개가10px 미만이다. 낮은 점수를 해석할 때 이 분포를 함께
본다. 작은 GT를 평가에서 삭제해 통과로 바꾸지 않았다. sunny selection은
143개 중141개가10px 미만이고 모든 후보의0.7 검출 TP가0이었다. 이 선정
자료만으로 sunny 모델을 확정하지 않는다. 새로운 heldout 결과로 confidence를
다시 조정하거나 기존 모델의 학습용 자료로 사용하지 않는다.

L detect/1280은 foggy **selection**에서0.2479/FP6으로 고정0.7의 최고
점수였다. M detect/1280의0.2157/FP2보다 점수는 높지만 기준선 FP5를
초과했고 최적화한 전체 스택 반복·장시간 검증을 완료하지 않았다. 이를
M 후보의 heldout 점수와 직접 비교하거나 연산 한도 내 최적 모델로 확정하지 않는다.
L-seg/1280은 같은 selection에서 FP27이었다. 모델 크기만으로 승격하지 않는다.

현재 충분히 비교한 후보에서는 **detect M/1280이 bbox 품질·지연의 우선 후보,
seg M/960은 mask 활용 후보**다. 앞서960에서만 잡힌 가까운 차량 반례와
mask 신규 누락도 유지한다. 이 판단은 운영 기본값 승격이 아니다.

### 같은 주행용 컨테이너의 날씨별 전체 스택 시험

실제 `asmc-ros-noetic`에서 MORAI·TwinLite3뷰·ResNet·LiDAR/fusion·판제/제어
계산을 함께 실행했다. 2+1 batch·세 뷰15Hz·debug OFF·unique 정책으로 각
모델×날씨를3회 측정했으며, 생성한9개 fixture actor만 정리하고 원래 SUNNY로
복원했다. 공개 차량 제어 발행은 하지 않고 `/evaluation/ctrl_cmd`를 사용했다.

| 후보·날씨 | 실제 뷰별 Hz 범위 | wall p95 최댓값 | 전체 VRAM peak |
|---|---:|---:|---:|
| M detect/1280·sunny |14.85–14.95|71.7ms|5051MiB|
| M detect/1280·foggy |14.85–14.95|84.4ms|5112MiB|
| M-seg/960·sunny |14.75–14.90|93.7ms|5197MiB|
| M-seg/960·foggy |14.65–14.90|102.4ms|5205MiB|

두 후보의 모델 공급·지연·VRAM 기준은12회 모두 통과했다. **전체 스택은12회
모두 실패**했으며 판제 trajectory 최대 공백270–333ms가250ms 기준을 초과했다.
모델 성능 통과로 판제 deadline이나 이전30분 실패를 면제하지 않는다.

원본은 `weather-live-shared-v1/sunny` 및 `weather-live-shared-foggy-v2/foggy`다.
v1 driver는 측정기의 수락 실패 종료 코드1을 실행 오류로 취급해 sunny6회 후
멈췄으며 기록을 보존하고 foggy6회를 v2에서 이어갔다. 복원 직후 GetWeather가
이전 foggy를 반환한 v2 실패도 유지한다. 후속 독립 RPC의 SUNNY 확인은
`weather-restoration-recovery-v1.json`에 있다. 이 실패들은 성공으로 덮어쓰지 않는다.

## 거리·batch 비교 후속 (2026-10-09)

FLOP/s는 설정한 처리율에서 필요한 산술 연산량의 추정이다. 실제 backend
시간 순위는 메모리 접근, TensorRT tactic, 호출·전송·NMS와 seg mask 복원에
따라 달라진다. 공식 YOLO26 표는 unused branch를 제거한 NMS-free 모델의
수치다. 현재 고정한 one-to-many/NMS 경로와 같은 graph라고 간주하지 않는다.
[공식 모델 표](https://docs.ultralytics.com/models/yolo26/)

`benchmark_coco7_batches.py`로 실제 3뷰 RGB를 고정해 네 모델×세 전방
너비×세 batch 방식×3회, 총108회 측정했다. 각 회차는 예열 후40 cycle이며
MORAI는 실행 중이었다. 표는 각 회차의 cycle wall p50을3회 평균한 ms다.
세 영상 packing, model.predict의 전처리·NMS, 원본 좌표/mask 복원을 포함한다.
JPEG decode·ROS 발행·다른 모델 ACTIVE 부하는 포함하지 않는다. 순수 GPU
kernel 시간이나 실제 카메라 Hz로 해석하지 않는다. 반복 재사용한 RGB는
고정 입력 benchmark이며 운영에서 결과를 재발행한 것이 아니다.

| 모델 | 2+1/640 | 2+1/960 | 2+1/1280 | batch3/640 |
|---|---:|---:|---:|---:|
| YOLO26m detect |21.92|24.43|26.19|17.01|
| YOLO26l detect |26.60|27.88|31.47|19.03|
| YOLO26m seg |28.57|31.93|34.54|24.01|
| YOLO26l seg |31.92|36.18|40.78|26.89|

전방640의 tensor는384×640, 측방은480×640이다. batch3는 모두480×640으로
패딩해2+1보다7.14% 많은 픽셀을 처리하지만 호출 감소로16–28% 빨랐다.
960은37.84%,1280은81.58% 더 많은 픽셀을 처리한다. 960에서는 두 방식의
시간이 거의 같았고,1280에서는 batch3가31–43% 느렸다. 엔진은 기존 dynamic
FP16 engine이며 최적 shape는 batch2/480×640이다. 전용 engine을 재생성한
최적값이라고 주장하지 않는다. head/confidence0.7/차체 mask는 고정했다.

`capture_coco7_ranges.py`는7종 대표 에셋을 SUNNY/FOGGY에서20·40·60m에
정지 배치했다. 요청 거리는 Ego anchor에서 spawn anchor까지이며 실제3D
bbox 중심 거리는 각 case에 별도로 기록한다. 센서 원점으로부터의 정확한
거리가 아니다.42개 모두 높이 gate를 통과했고 원본 영상을 검토해 중앙의
객체가 보이는 것을 확인했다. Ego 이동0m·gear 동일, 생성 객체만 정리하고
gRPC 응답을 재확인해 원래 SUNNY로 복원했다.630 JPEG의 source stamp·SHA를
보존했다. 주행 중 최초 검출 거리나 전체 에셋 recall 시험은 아니다.

동일 전방5프레임을12개 모델/해상도에 confidence0.7과0.25로 각각 실제
재추론했다. 넓은 중앙 검토 영역(570,230)–(685,455)에서 예상 라벨이5프레임
중3개 이상 있었던 장면 수는 다음과 같다. 이 영역은 수동 검토한 검색 영역이며
정밀 GT box/mask가 아니다. 정량 AP·recall이나 독립적인 오검출 수락으로
해석하지 않는다. 다른 위치의 같은 라벨로 성공을 대신하지 않는다.

| 모델 | conf0.7: 640/960/1280 | conf0.25: 640/960/1280 |
|---|---:|---:|
| M detect |11/15/17|19/24/25|
| L detect |13/16/16|23/24/29|
| M seg |10/11/12|19/22/28|
| L seg |10/15/15|19/25/22|

각 분모는42장면이다.0.25는 원인 분리를 위한 대조이며 운영 confidence를
낮추거나 최종 값으로 선정한 것이 아니다. 큰 모델·해상도가 모든 사례에서
좋지 않다. 예컨대 conf0.7에서 M detect/1280은40m person을 두 날씨 모두
5/5회 검출했지만60m는0/5였다. L seg/960은 car를60m에서 두 날씨 모두
5/5 검출했다. FOGGY40m bus를 truck으로 반환하는 후보도 있다. 개·자전거의
40–60m 검출은0.25에서도 대부분 실패했다. 세부 라벨·미검출·공백 개선에는
데이터 기반 검증·추가학습이 필요하며 규모 확대만으로 해결됐다고 판단하지 않는다.

일반 품질 비교는 전방 단독 tensor를 사용한다.640 batch3에서는 전방 패딩
높이도 달라지므로 실제 정지3뷰를 넣은 별도 비교와 새 입력15Hz 시험을 구분한다.
촬영된3뷰는 같은 정지 장면의 ordinal로 연결하며 source stamp가 같은 동시
영상이라고 표시하지 않는다. 운영 메시지·모델·fusion 기본값은 바꾸지 않았다.

실제640 batch3 패딩으로 같은 정지3뷰를 넣으면 conf0.7의3/5 장면 수는
M detect11·L detect13·M seg8·L seg11이다. 전방 단독 tensor 대비 M seg는
SUNNY40m truck이5/5→0/5, FOGGY40m truck이4/5→1/5로 감소했다. 반대로
추가 검출된 사례도 보존했다. padding/batch 조건이 출력에 영향을 주므로
단독 품질 표와 batch3 운영 출력을 동일하다고 가정하지 않는다.

`asmc-ros-noetic`에서 MORAI를 유지한 C1 12회도 완료했다. TwinLite·ResNet은
로드된 STANDBY, YOLO만 ACTIVE이며 full-stack 수락 시험이 아니다.

| 640/batch3 후보 | 뷰별 평균 Hz 범위 | wall p95 최댓값 | PASS/3 | 최대 공백 |
|---|---:|---:|---:|---:|
| M detect |14.80–14.90|30.7ms|3|177ms|
| L detect |14.70–14.90|35.9ms|2|275ms|
| M seg |14.80–14.95|40.9ms|3|180ms|
| L seg |14.50–14.90|48.9ms|2|536ms|

모델 오류·중복/역행 stamp는0이며 최대 ACTIVE 전체 VRAM5567MiB다.
평균 Hz만으로10/12 PASS를12/12로 표시하지 않는다. L detect r2 전방
275ms 공백은266ms 입력 공백과 겹쳤다. L seg r3는 ACTIVE 첫1초에8개씩
출력했고 첫 출력 지연536ms, 입력 최대 공백465–494ms였다. 입력 header
stamp 차이도 같은 규모였으므로 측정 subscriber 도착 시각만의 지연과는
구분된다. 이것만으로 MORAI 렌더링·bridge·스케줄링의 원인을 확정하거나
모델 수락 실패를 면제하지 않는다. 정상 출력 사이의 간격과 첫/끝 endpoint
공백을 구분하며 원본 gate는 둘 다 유지한다.

새 측정/기존 COCO 시험28개·검출 회귀70개와 계약·문서 검사3개가 통과했다.
운영 코어·메시지를 바꾸지 않아 전체 build를 반복하지 않았다. 이전 facade와
CI 결제 제한은 별도 미해결 상태이며 이 시험으로 통과 처리하지 않는다.
[Git에 보존한 요약](../../../config/baselines/evidence/camera_coco7_live_v1/range_batch_followup_summary.json)에
42장면별 각 모델의 결과 수, 실제 거리,108회 timing,12회 live 판정과 원본
SHA가 있다. 원본 입력/출력/리소스는 `batch3-live-640-v2/trials`와
`range-batch3-live-input-gaps-v1.json`에 보존한다.

브라우저에서 객체·날씨·거리·모델을 고르고 중앙 확대를 켜 확인한다.

- [거리 비교(conf0.7)](http://localhost:8766/range-model-inspection-v2/comparison.html)
- [거리 비교(conf0.25)](http://localhost:8766/range-model-inspection-low25-v1/comparison.html)
- [실제640 batch3 비교(conf0.7)](http://localhost:8766/range-batch3-640-inspection07-v1/comparison.html)

원본은 `range-captures-v1/sunny`, `range-captures-v2/foggy`,
`range-model-inspection-v2`, `range-model-inspection-low25-v1`,
`batch-range-benchmark-v2`다. 첫 날씨 enum 비교 오류, 완료 전 registry를 읽어
FOGGY 마지막 case가 빠진 inspection-v1, batch post 타이머의 미세한 차이를
오류로 취급한 benchmark-v1, source 패키지가 생성 ROS msg를 가린 첫 live
startup 실패도 보존한다. 완료된 capture와 모델 pairing·SHA·중복 stamp를
검사하도록 보강했으며 실패를 성공 결과로 덮어쓰지 않는다.

## 이후

### 최종 회귀와 CI

실제 공유 workspace의 바깥·nested build를 사용한 network-none CPU 회귀를
`final-regression`에 보존했다. 카메라·메시지 등563개 중 기존 facade1건,
Main311개 모두 통과, LiDAR 좌표·시간 core3개 통과, 측정 도구131개 통과다.
생성 메시지를 사용한 모드/클래스/seg/batch/호환 어댑터 ROS 검사5개와
jamming fixture4개(내부36시험), 계약·문서 validator6개도 통과했다.

audit 전체51개 중 P1-C metadata authority 상태1건은 실패한다. 해당 코드·
테스트·HD맵 metadata와 facade 관련 코드가 main과 동일함을 SHA로 기록했다.
다른 대회 metadata를 변경하거나 검사 기준을 완화하지 않았다. 최초 audit
실행의 ASMC_DATA 마운트 누락2건과 CMake의 --test-dir 미지원으로 실제 시험이
없었던 실행도 보존하고, 올바른 환경·작업 위치의 수정 실행을 별도 기록했다.
jamming의 --text 종료 코드만으로 판정하지 않고 내부 unittest의 OK를 확인했다.

GitHub CI는 결제/사용 한도 제한으로 job의 첫 step부터 실행되지 못했다.
과거 HEAD의 CI 성공이나 로컬 회귀로 최종 CI 통과를 대체하지 않는다.
PR은 Draft이며 기본 모델·재사용 정책의 수락과 CI 확인이 남아 있다.

GitHub의 실제 annotation은 `The job was not started because recent account payments
have failed or your spending limit needs to be increased`이며 job의 steps는 비어 있다.
조직 owner/billing manager가 [조직 Billing](https://github.com/organizations/INHAautonav/settings/billing)의
결제 상태와 Actions budget을 확인해야 한다. 정상화 후 실패한 workflow를 재실행한다.
이 PR에서는 결제·예산·필수 검사 설정을 변경하지 않는다.
Git에 포함한 [최종 요약](../../../config/baselines/evidence/camera_coco7_live_v1/final_acceptance_summary.json)
과 baseline에서 원본 증거 경로·SHA·성공·실패를 확인한다.

### 30분 통합 실행

`live-sustained-30min`은 MORAI 렌더링을 유지한 정지 상태의 전체 스택
1800초 실행 1회다. YOLO26m-seg/전방960/2+1 batch/세 뷰15Hz에서 실제
YOLO 평균은 전방14.94·좌14.89·우14.89Hz, wall p95는91–92ms,
p99는104–106ms였다. TwinLite 뷰별 평균9.99Hz, ResNet 처리율9.94Hz,
최대 전체 VRAM5168MiB, 모델 오류0·중복/역행 stamp0이었다.

수락 결과는 **실패**다. YOLO 최대 공백409–447ms, TwinLite467–471ms,
ResNet400ms, 판제 trajectory/state/objects/control deadline 초과를 보존한다.
YOLO의250ms 초과 출력 공백은 뷰별13/12/12회이며 모두250ms 초과
카메라 수신 공백과 겹친다. 인과관계를 확정하거나 실패를 면제하지 않는다.
TwinLite 최소5초 창9.4Hz, ResNet9.2Hz도 기준에 미달한다.
ResNet의 신호등 부재에 따른 valid=false는 처리 실패와 구분한다.

카메라3종의 시작1–2분 대비 마지막1분 RSS 중앙값 증가는 각각
YOLO2.09·TwinLite5.72·ResNet0.49MiB였다. 30분만으로 메모리 누수가
없다고 확정하지 않는다. Fusion20617쌍·drop2266·종료 pending1/2를 기록했다.
screen 로그는 일부 interval이 끊겨 합계가 맞지 않았으며, master의 온전한
rosout interval358개와 종료 tail을 연결한 합계는 정확히 일치했다.
screen 원본도 보존한다. `sustained-30min-summary.json`과
`sustained-input-gaps.json`이 상세 결과다. 평균 성능만으로 기본 모델을
승격하지 않으며, 센서 전달·스케줄링 공백과 판제 deadline을 후속 해결한다.

이번은 추가학습 없이 일반 YOLO 모델·해상도·주기를 선정하는 단계다.
다음 YOLOE 장애물 확대에서는 원본 이름+네 소비 그룹을 별도로 비교한다.
단일 모델로 전 구간을 처리하는 구성을 우선하고, 부족할 때만 판제가 구간별
준비·활성화·가중치 전환을 요청한다. LiDAR 구조물·군집 개선은 재호의 인계 과제다.
