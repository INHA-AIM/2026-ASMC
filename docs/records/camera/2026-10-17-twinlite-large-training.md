# 2026-10-17 TwinLite Large v7 학습 시작

사용자의24시간 학습 승인 후 **17:55:14 KST**에 독립 학습 컨테이너를 시작했다.
최대 종료 시각은 **2026-10-18 17:55:14 KST**다.40epoch는 LR schedule의 상한이며
24시간 내 완료를 보장하는 수치가 아니다. 운영 Medium v6 TensorRT와 YOLO E8은 유지한다.

- [E366 학습 현황](http://127.0.0.1:8000/camera-seg-large-train-20261017-r1/index.html)
- [학습·checkpoint·재개 명령](../../../src/asmc_camera/docs/training-run.md)
- [고정 학습 recipe](../../../config/experiments/twinlite_large_training_v7.json)
- [E365 데이터 준비 기록](2026-10-17-twinlite-large-data.md)

## 데이터와 설정

`twinlite_morai_v7_large_capture_r2`: train68,390장 전체, val1,404장, test14,556장.
원본·준비 데이터는 읽기 전용이고, test는 학습 중 사용하지 않는다.
조건 보정 후 epoch당91,969회 노출,22,993 microbatch를 사용한다. 배치 계보와 기존
actor-negative4-class semantic 정책, 주차칸 선 포함, padding 제외를 유지한다.

| 항목 | 설정 |
|---|---|
| 초기 모델 | 공식 BDD100K Large, 공통394개 tensor 전이·마지막 출력부14개 초기화 |
| 입력 H×W | 전방544×960, 좌우480×640 |
| Batch | 실제4, 누적3, 유효12; BatchNorm은4 |
| 학습 | 전체 fine-tuning·AMP·AdamW·EMA |
| LR·정규화 |0.0002·weight decay0.0005 |
| Schedule |40epoch polynomial(power1.5),24시간 절대 상한 |
| 재현성 | seed20261017·고정 코드 스냅샷·데이터/모델 SHA |
| 평가 | 전이 직후와 매 완전한 epoch에서 전체 val |
| 선정 | `six_condition_macro_mean_head_iou` |
| 저장 | 매 완전한 epoch와1,500 microbatch 복구 주기 |

초기 전이 상태의 조건 균형 drivable IoU는0.6402012, road_marking foreground macro IoU는
0.0053045, 두 head 평균은0.3227528이었다.4-class 출력부를 새로 초기화한 상태이며,
기존 이진 BDD lane의 성능이나 학습 완료 성능으로 해석하지 않는다.
고정18개 val 사례의 RGB/GT/추론은 같은 표본을 epoch마다 갱신한다.

## 실행·검증 근거

학습 컨테이너는 `asmc-twinlite-large-v7`, 갤러리는 `asmc-experiment-gallery`다.
수집·MORAI·실시간 모델 추론을 시작하지 않았다. 출력은
`E:\asmc_data\twinlite\training_runs\twinlite_large_v7_20261017_r1`에 저장한다.

입력 데이터 metadata SHA:
`8a4e5fc8f654b8739ea777d6546ef92d2c6d27807d31e07039f0a07211a8a20e`.
초기 checkpoint SHA:
`e503cbbb05177ef6e59fe19285b60314715109f95bf2a494185e3145dd53a4a6`.
고정 upstream은 `90f1b8695ae311d5123b05f8534b2e11e42499d2`다.
코드 스냅샷과 전체 파일 SHA는 공통 data의
`runtime/twinlite-large-training-20261017-r1/snapshot.json`에 있다.

첫 분리 실행은 스냅샷에 camera Python 의존성이 빠져 import 단계에서 종료됐다.
모델·데이터 로드 및 optimizer update 전의 실패다. 누락 모듈을 포함한 revision2를
CPU에서 로드 검증한 뒤 실행했다. 실패 로그와 최초 snapshot은 같은 runtime 폴더에 보존했고
실패 컨테이너는 제거했다.

CPU 시험4개는 부분 epoch 재개 시 optimizer·EMA·RNG·다음 batch 복원,
완전한 epoch 이후 재개·scheduler LR 복원, deadline·중단 표시, 데이터 로드 없는 offset을
검사했다. Dropout을 사용하는 합성 모델에서 중단 없는 학습과 재개한 모델·EMA tensor가 같았다.
실제 GPU 학습에서는 유한한 loss와500회 이상 optimizer update를 확인했다.
이때 AMP 자동 축소로2회 update가 생략됐으며, 연속 오류나 OOM은 없었다.
AMP skip 수는 숨기지 않고 페이지에 계속 표시한다.

실제 첫 복구 checkpoint는31,740,242bytes로 재로딩했으며 epoch1·next_batch3·
optimizer update1·EMA update1·AdamW state300개와 고정 deadline을 확인했다.
페이지 JavaScript 구문과 실제 상태 JSON을 이용한 DOM smoke도 통과했다.
12개 상태 카드·6개 조건 표·loss/IoU SVG가 생성됐고 HTTP200을 확인했다.

학습을 끝까지 수행한 결과나 배포 수락을 이 시작 기록으로 대체하지 않는다.
최종 best checkpoint와 heldout·TensorRT·family 검증은 학습 종료 후 정리한다.
