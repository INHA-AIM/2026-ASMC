# TwinLiteNet+ Large 추가학습 준비 조사

이 기록은 데이터 경로 정리 후의 학습 설계 자료다. Large 변환·다운로드·학습·운영 모델 교체는
아직 실행하지 않았다. 현재 기본 모델은 기존 Medium v6 TensorRT다.

## BDD100K 원본과 공식 모델 출력

[고정 공식 소스](https://github.com/chequanghuy/TwinLiteNetPlus/tree/90f1b8695ae311d5123b05f8534b2e11e42499d2)의
`model/model.py`는 공유 encoder와 drivable/lane decoder를 사용한다.
기본 `out_da`, `out_ll`은 각각2채널이다. `BDD100K.py`도 foreground/background 타깃을 만든다.

| 범위 | 종류 |
|---|---|
| 공식 TwinLiteNet+ 기본 drivable head | background / drivable |
| 공식 TwinLiteNet+ 기본 lane head | background / lane |
| BDD100K 원본 drivable 라벨 | direct / alternative / background |
| BDD100K 원본 lane category | crosswalk, double other, double white, double yellow, road curb, single other, single white, single yellow |

원본 lane 라벨에는 parallel/vertical 방향과 solid/dashed 속성도 있다.
원본이 상세 라벨을 가진다고 기본 사전학습 checkpoint가 이 종류들을 모두 출력하는 것은 아니다.
[BDD100K 정본 라벨](https://github.com/bdd100k/bdd100k/blob/master/bdd100k/label/label.py)에는
별도 stopline category가 없고, 공식 TwinLiteNet+ 기본 head에도 stopline 출력이 없다.

현재 MORAI 모델은 drivable2종 + road_marking4종(background/white_lane/yellow_lane/stopline)이다.
정지선은 세 번째 독립 head가 아니라 road_marking head의 한 클래스다.
2→3/4채널 변경은 최종 분류층 전이 문제이며 encoder/decoder 전체를 초기화해야 하는 이유가 아니다.
Large는 Medium과 내부 channel/깊이가 다르므로 공식 BDD100K Large checkpoint에서 시작하는 편이
적절하다. BDD100K를 처음부터 다시100epoch 학습하는 작업과 구분한다.

음영 family는 흰/황색 차선 구분을 사용하므로 이를 임의로 binary lane으로 줄이지 않는다.
정지선이 대회 소비 범위에서 불필요하면 후속 준비에서 background/white/yellow3종을 우선 검토한다.
현재 배포 계약을 이 조사만으로 변경하지 않는다. 유지 여부는 GT·loss·출력 계약·family 검증과
함께 결정하며, channel 추가 자체를 학습 불안정의 원인으로 단정하지 않는다.

## 실제 실행 해상도와 학습 입력

[공식 논문](https://arxiv.org/html/2403.16958v3#S4.SS1.SSS3)은640×384 입력을 사용한다.
이는 우리 전방544×960·측방480×640 입력의 최적 학습 설정을 검증한 결과가 아니다.

| 뷰 | 권장 추가학습 bucket | 현재 실행 tensor |
|---|---|---|
| 전방 | H544×W960 | B1×3×544×960 |
| 좌·우 | H480×W640 | B2×3×480×640 |

전방960 정사각형 학습을 의미하지 않는다. 원본1280×720은960×540으로 축소하고 총4px 높이
padding을 준다. 측방640×480은 축소·padding 없이 사용한다. RGB·semantic·instance·ignore에
동일 Geometry를 적용하고 categorical GT는 최근접 보간으로 처리한다.
한 training batch 안에서는 같은 shape를 사용하고, 전방/측방 bucket을 epoch에서 함께 학습한다.
추론의1+2 batch와 학습 batch는 별도다.

현재 v6의27,405개 타깃/valid-mask 경로는 복원했지만 baked target은384×640이다.
기존 mask를960으로 확대해 native 세부 정보가 복원됐다고 설명하지 않는다.
추가학습용 새 버전은 기존 native semantic/instance GT에서 각 bucket 크기로 다시 생성한다.
원본 RGB·GT는 보존하고 run 단위 평가 분할을 유지한다.

960은 얇은/먼 차선에 더 많은 픽셀을 제공할 가능성이 있지만 우월함을 보장하지 않는다.
640 공통 입력은 비용 대조 후보이며, 같은 평가 자료의 차선 IoU·연결성·family 역할/기하·공급률로
판단한다. 첫 권장은 배포 Geometry에 맞춘 두 bucket이며 일률적인960 정사각형이 아니다.

공식 README에서 Large는1.94M parameter·17.58GFLOPs, Medium은0.48M·4.63GFLOPs다.
같은 입력의 FLOPs 비율은 약3.8배지만 이를 실제 TensorRT 지연 비율로 환산하지 않는다.
추가학습 전 shape/head 전이, 이후 TensorRT와3모델 동시 부하를 확인한다.

## 후속 실행 순서

1. 공식 BDD100K Large 가중치·head·클래스 순서와 전이 가능한 tensor를 확인한다.
2. native GT에서544×960/480×640 타깃을 새 데이터 버전으로 생성한다.
3. 정지선 유지/제외와 필요한 흰/황색 구분을 결정하고 loss·평가 계약을 맞춘다.
4. 실제 GPU batch·AMP·유한 loss를 점검한 뒤 MORAI 추가학습을 시작한다.
5. 음영 SUNNY/FOGGY×11/13/15시·전방/측방 품질과 family를 평가한다.
6. TensorRT 변환·YOLO/ResNet 동시 공급을 통과한 후보만 배포에 반영한다.

현재 데이터 위치와 확인 명령은 [데이터 준비](../../operations/data-preparation.md)를 따른다.
