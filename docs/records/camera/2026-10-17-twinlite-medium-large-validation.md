# 2026-10-17 TwinLite Medium·Large 동일 val 비교

현재 Large 학습의 수렴 추이와 기존 Medium 운영본의 품질을 비교한다.
학습·운영 가중치·split·선정 지표는 변경하지 않는다. GPU 학습을 유지하고
별도 CPU 평가만 실행한다. Test와 17시 자료는 이번 비교에 사용하지 않는다.

## 비교 조건

`twinlite_morai_v7_large_capture_r2`의 val 1,404장을 전부 사용한다.
전방544×960·좌우480×640(H×W), 동일 RGB/GT·padding 유효 영역·EMA·batch4·
CPU FP32·기존 multiclass evaluator로 비교한다. 원본 크기로 되돌린 면적과
model-grid 면적을 섞지 않는다. F1이 아닌 segmentation IoU다.

데이터 metadata SHA는
`8a4e5fc8f654b8739ea777d6546ef92d2c6d27807d31e07039f0a07211a8a20e`,
val manifest SHA는
`3a1aaa64869e88bff4ae44dfa8badbdc20dec62d17bc5a67059a89da46f2b2c7`다.
Medium은 패키지의 `twinlite-medium-v6.pt`이며 SHA는
`a26311b3af4e92880f632509eff3c81defc1b95135f5b463c7a08e5b6e23d24b`다.
Large는 완료된 `epoch002.pt`로 고정한다. 당시 완료 epoch1–4 중 조건 균형
평균이 가장 높았던 체크포인트이며 가변적인 best 파일 경로로 식별하지 않는다.

기존 Medium은 MORAI v5 actor-negative epoch18 EMA에서 초기화한 v6 결과다.
사용자가 기억한 짧은 마지막 학습 시간만으로 전체 학습량을 설명할 수 없다.
Large는 BDD Large의 공통 tensor를 전이하고 4-class 출력부를 초기화한 새 학습이다.
데이터·초기화·모델 크기가 동시에 다르므로 품질 차이를 모델 크기만의 효과로 해석하지 않는다.

## Large 학습 추이

학습 중 GPU AMP 전체 val 기록이며, 위 CPU FP32 대조와 정밀도를 구분한다.

| Epoch | 조건 균형 두 head 평균 | Drivable | 노면 표시 macro | Train loss |
|---|---:|---:|---:|---:|
| 1 |86.9432%|98.0620%|75.8244%|0.76157|
| 2 |90.6459%|98.2320%|83.0598%|0.28539|
| 3 |90.5147%|98.2158%|82.8137%|0.14314|
| 4 |90.3894%|98.2127%|82.5661%|0.11905|

초기 큰 개선 후 epoch2–4의 검증 지표는 일시적으로 정체했고 train loss는 계속 감소했다.
현재 val에서 개선이 둔화됐다는 근거이며, 이후 개선 불가를 입증하지는 않는다.
이 몇 번의 관측만으로 LR이나 과적합의 인과관계를 확정하지 않는다.
Epoch2를 보존하고 이후 완전한 epoch의 추이를 확인한다.
최신 epoch가 최고점이나 운영 모델을 자동으로 대체하지 않는다.

## 동일 CPU FP32 평가 결과

두 평가 모두1,404장을 처리했고 정상 종료했다. 종합과 노면 표시 조건 균형은
여섯 조건을 같은 비중으로 평균하며, 개별 클래스 행은 전체 val의 픽셀을 합친 IoU다.

| 지표 | Medium v6 | Large epoch2 | 변화 |
|---|---:|---:|---:|
| 조건 균형 두 head 평균 |86.4446%|90.6452%|+4.2006%p|
| 조건 균형 노면 표시 macro |75.3907%|83.0578%|+7.6672%p|
| Drivable pooled |97.4979%|98.2748%|+0.7769%p|
| 흰차선 pooled |77.5363%|86.2628%|+8.7264%p|
| 황색차선 pooled |88.5273%|91.2058%|+2.6785%p|
| 정지선 pooled |71.7118%|85.7466%|+14.0348%p|

전방 황색차선은85.0070%→83.2787%로1.7284%p 낮아졌다.
전체 평균이 좋아도 이 약점을 생략하지 않는다. 전방 흰차선·정지선 및
측방 개선과 함께 원본 사례·음영구간 family를 확인한 뒤 배포 후보를 판단한다.
Large CPU와 학습 중 AMP의 조건 균형 평균 차이는−0.0007201%p였다.
따라서 관측된4.20%p 차이를 CPU/GPU 정밀도 차이로 설명할 수는 없다.

Large checkpoint SHA는
`134882082cb6e887be7ecdc4104a9432a30b16968bc9bd89de8fb53d920fbc8a`다.
Medium CPU 평가 loop는376.1초, Large는589.7초였으며 이 값은 CPU 검증 비용이다.
이 시간을 TensorRT 실시간 배포 지연으로 사용하지 않는다.

## 지표 해석 범위

조건 균형 지표는 SUNNY/FOGGY×11/13/15의 여섯 조건을 같은 비중으로 평균한다.
노면 표시 macro는 background를 제외한 흰차선·황색차선·정지선 평균이다.
FOGGY15 정지선은432장 전체에서 GT444픽셀로 매우 희소하며,
오른쪽 뷰의 황색차선도15,876픽셀이다. 조건별·뷰별·클래스별 수치와
pooled 지표·원본 사례를 함께 확인한다. 이를 이유로 GT나 선정 지표를 바꾸지 않는다.

같은 K-City 맵과 과거 Medium 개발에서 사용한 val이므로 비공개 독립 시험이 아니다.
현재 입력 해상도에서의 semantic 품질 비교이며 TensorRT 지연·family 역할·
곡선 연결·readiness·배포 수락은 별도 검증이다.

## 재현과 산출물

기존 `scripts/evaluate_twinlite.py`로 두 모델의 CPU FP32 전체 val을 평가한다.
`tools/perception/compare_twinlite_validation.py`는 모델·데이터 SHA, EMA·정밀도·
batch와 전체 표본 수를 검사한 뒤 두 보고서를 재사용해 표·그래프와 같은18개
RGB/GT/Medium/Large 사례를 만든다. 결과는 공통 artifacts의
`camera-seg-medium-large-compare-20261017-r1/`에 저장한다.
기존 학습 중 AMP 결과를 CPU 결과로 바꿔 기록하지 않는다.

[E367 동일 val 비교 갤러리](http://127.0.0.1:8000/camera-seg-medium-large-compare-20261017-r1/index.html)는
실험 목록과 E366 학습 현황에서 접근한다. 고정18개 사례의 RGB/GT/Medium/Large를
한 행에 표시하며 뷰별 클래스 증감·조건별 수치·수렴 그래프·전체 SHA를 함께 제공한다.
완료된 CPU 검증 컨테이너는 로그·종료 상태를 보존한 뒤 제거한다.
GPU 학습 컨테이너와 갤러리 서버는 유지한다. 운영 모델 교체는 하지 않는다.
