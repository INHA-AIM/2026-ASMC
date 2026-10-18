# 2026-10-17 · TwinLiteNet+ BDD100K Large 사전학습 추론

공식 Large를 추가학습 없이 기존 MORAI Capture 영상에 추론하고 현재 Medium v6와 비교했다.
운영 모델·TensorRT·family·메시지 계약은 유지한다. 실행은 [별도 실험 안내](../../../src/asmc_camera/docs/bdd-pretrained-experiment.md)를 따른다.

## 고정 입력과 모델

- 원본: 공식 README의 `large.pth`, Google Drive file ID `1H8P-GrOUBOaVs5LEqXBfz0dC9gguUUio`.
- 원본 SHA: `5605d1a8ce762c1b6f7d23f4c2e7ec9384bfd6e6887b040b43e1f8902a817e22`.
- 배포 SHA: `e503cbbb05177ef6e59fe19285b60314715109f95bf2a494185e3145dd53a4a6`.
- 고정 공식 source: `90f1b8695ae311d5123b05f8534b2e11e42499d2`.
- Large 1,943,911 parameters, 408개 state tensor 원본과 동일. 두 head는 각각 background/foreground 2채널.
- v6 val 1,404장 = 468개 3뷰 묶음. SUNNY/FOGGY × 11/13/15시, 뷰별 468장.
- run별 표본 수는 135–432장으로 불균등하다. 전체 평가에는 모두 사용하고, 갤러리는 조건별 12묶음 × 3뷰 = 36장씩, 총216장을 고른다.
- 각 run 내 시간 간격으로 선정했다. 인접 영상은 독립 장면으로 세지 않으며, 확인되지 않은 영상에 터널 등 조건을 추정 부여하지 않았다.

BDD는 RGB `/255`·LINEAR letterbox·padding114로 추론한다. 공식 입력384×640과
실행 후보 전방544×960·좌우480×640을 대조했다. Medium v6는 기존 자동 보간(축소 AREA)을 유지했다.
모두 PyTorch AMP로 직렬 추론했으며, 이 실험의 시간은 TensorRT 실시간 부하 비교가 아니다.

## 결과

| 모델·입력 | MORAI drivable IoU | 흰/황 차선 통합 IoU |
|---|---:|---:|
| 현재 Medium v6·전방960/측방640 | 97.26% | 74.75% |
| 공식 BDD Large·공식384×640 | 61.70% | 11.81% |
| 공식 BDD Large·전방960/측방640 | 65.74% | 7.69% |

threshold0.5, native GT·유효 영역에서 pixel intersection/union을 누적했다.
차선 GT는 흰/황 차선의 합집합이며, 정지선 GT 픽셀은 binary 비교에서 제외한다.
drivable GT는 MORAI의 객체 점유 negative 정책이다. BDD 라벨 정의와 일치한다고 가정하지 않는다.
Large는 흰/황/정지선의 개별 분류 성능을 제공하지 않는다.

현재 MORAI에 추가학습한 Medium이 직접 사용하기에 더 적합하다. BDD Large의 차선은
공식640 입력에서 상대적으로 나았으며, 해상도를 높이는 것만으로 MORAI 성능이 개선되지 않았다.
이는 학습 도메인·scale·라벨 정의가 다른 비교이므로 Large 구조가 열등하거나 추가학습 후에도
개선되지 않는다는 결론은 아니다. 다음 Large 추가학습의 초기화 후보와 실패 사례 확인에 사용한다.

## 산출물과 검사

로컬 `artifacts/camera-seg-bdd-large-20261017-r2/index.html`은 **ASMC 실험 목록 E362**에 연결했다.
날씨·시각·뷰 필터, 원본/GT/현재 모델/BDD960/BDD640 비교 이미지와 전체 지표·image/GT SHA를 제공한다.
공식 원본은 `data/models/twinlite_bdd_large_v1/`, 실행 checkpoint는 패키지 models에 둔다.

첫 r1은 두 번째 모델을 로드할 때 upstream의 namespace package에 `__file__`이 없어 실패했다.
원래 출력 폴더와 실패 사유를 보존했다. 로더가 검증된 `__path__`를 확인하도록 수정하고
다른 출처의 `model` 모듈은 계속 거부한다. r2의 세 모델 로드·전체 GPU 추론이 완료됐다.

공식 전처리 수치 일치, 실험 토픽·서비스 격리, 기본 실행 보존과 source metadata를 CPU/ROS 시험으로 확인한다.
카메라 CPU 26개, runtime 회귀 91개와 추가 namespace/provenance 검사를 통과했다.
현재 컨테이너 `/root/ws`에서 `catkin_make --pkg asmc_camera -j2 -l2`가 통과했다.
실제 ROS/GPU/renderer의 runtime 입력 연결은 독립 master에서 통과했다. 12초 시험에서
뷰별111개 확률·합성56개를 수신했고 shape·stamp·wire 오류와 운영 토픽/서비스 발행은0이었다.
첫 연결 시험은 기존 mono8 확률을32FC1로 해석한 시험기 오류로 실패했으며 이를 보존하고 수정 후 재검증했다.
결과는 같은 폴더의 `ros-smoke/report.json` 및 PR에 기록한다.
고정 RGB 재생 시험은 실제 MORAI 주행·전체 스택 공급 수락으로 표시하지 않는다.

실험은 별도 브랜치로 제공하며 PR #94의 개인 PC 데이터 경로 변경과 분리한다. 이번 작업에는
학습·수집·운영 기본값 승격·PR 병합을 포함하지 않는다.
