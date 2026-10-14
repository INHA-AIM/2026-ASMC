# 카메라 장애물 학습 데이터 확보·감사·라벨 통일

> 기준일: 2026-10-09. 사용자 지정 Roboflow 22개 프로젝트와 기존 Capture GT를 대상으로 한다.
> 운영 모델·ROS 인터페이스·confidence·fusion·컨테이너 마운트는 변경하지 않는다.

최신 평가 자료는 [16종 검수 평가본](#16종-검수-평가본--2026-10-13)이다.
이전 v8 본학습은 epoch 4에서 보존했고, 배경 손실을 수정한 v9 본학습이 새로 진행 중이다.
평가본 구성 단계에서는 학습을 재시작하지 않았으며, 이후 사용자 승인으로
[배경 손실 수정·v9 본학습](#배경-손실-수정전체-본학습-재시작--2026-10-13)을 시작했다.
[v8 본학습](#v8-yolo26l-본학습--2026-10-12-실행-중)의 시작 기록은 아래에 보존한다.
[Capture 자동 수집](camera-mpc-object-collection-pilot.md)과 데이터 전처리를 완료하고,
YOLO26l 가중치 본학습을 진행 중이다. 운영 교체·PR 병합은 하지 않았다.
이전 저장 위치·로컬 teacher 비교는 [v6 작업](#로컬-teacher와-저장-버전--2026-10-11-v6)에 보존한다.

## 16종 검수 평가본 — 2026-10-13

사용자 요청으로 `data/dataset_versions/camera_objects_bbox_validation16_v1-r2`를 생성했다.
**150장·양성 322개·ignore 33개**, 모든 이미지는 목표 16종 전체의 누락 여부를 검토한
complete 평가 자료다. train 53,588장과 파일 SHA·디코딩 픽셀 SHA·source/run·split group을
대조했으며 일치 자료는 없다. 원본과 기존 split, 618장 진단 자료는 변경하지 않았다.

| 클래스 | 등장 이미지 | 양성 박스 | 소스/run 그룹 |
|---|---:|---:|---:|
| person | 22 | 22 | 3 |
| dog | 12 | 12 | 8 |
| deer | 12 | 12 | 3 |
| bicycle | 12 | 12 | 6 |
| car | 24 | 44 | 3 |
| motorcycle | 13 | 23 | 4 |
| bus | 11 | 15 | 1 |
| truck | 12 | 14 | 2 |
| bollard | 8 | 16 | 1 |
| box | 18 | 50 | 4 |
| concrete_barrier | 11 | 14 | 2 |
| steel_barricade | 13 | 13 | 3 |
| traffic_drum | 25 | 33 | 1 |
| plastic_barrier | 9 | 17 | 2 |
| traffic_cone | 12 | 12 | 1 |
| stroller | 13 | 13 | 2 |

소스는 Roboflow 45장 / AIM 46장 / Capture 59장이다. 설정 기록으로 확인한 SUNNY 53장,
FOGGY 52장과 설정 미상의 시각적 맑음 23장·안개 22장을 **구분**한다.
Roboflow 하늘에 구름이 있다는 이유로 SUNNY 또는 CLOUDY로 단정하지 않는다.
박스 숫자를 맞추려고 복제·증강하지 않았고, 한 이미지의 모든 대상 주석을 유지했다.
완전한 클래스×날씨×소스 균형이나 독립 장면 충족을 의미하지 않는다.
버스·볼라드·드럼·콘은 소스/run 그룹이 하나이며, 콘은 SUNNY뿐이다.
그룹 수는 독립 장면 수의 상한이다. 같은 교차로·배치의 연속 사진은 독립 표본이 아니다.

검수 및 누출 방지:

- 직접 대응 Roboflow 원본 박스를 유지했다. AIM dog/deer/bicycle은 기존 teacher로 후보를
  찾은 뒤 RGB와 actor inventory로 확인했으며, 사슴의 dog 제안을 deer로 수정했다.
  따라서 이 쉬운 객체 패널로 teacher의 무편향 recall을 주장하지 않는다.
- 터널 유모차는 teacher가 사람만 제안해 원본 해상도에서 adult/stroller를 직접 분리했다.
  유모차 안의 아기는 person에서 제외했다. 네 영상은 하나의 encounter다.
- 구 Capture의 WoodBox actor 13,50–56은 각각 가장 크게 보이는 RGB로 종류를 확인하고
  native instance bbox를 추가했다. dense map drum/barrier 및 잘못 배치한 대형 트럭은 제외했다.
- 이미 학습한 route60/120/180 배치는 제외했다. 미사용 초기 route12 중 나중에 재사용된
  BabyStroller3(native ID 9/90001/90002)가 보이는 영상도 제외했다.
  r1은 이 추가 검사 이전의 초안이므로 평가에 쓰지 않는다. **r2만 설정에 고정**했다.
- 캐시가 있는 train 22,914장에 pHash 근접 검사를 추가했다. 거리 ≤6의 34쌍을 모두
  비교해 같은 맵 배경이지만 객체/배치/시점이 다른 사례임을 확인했다.
  캐시 없는 최근 수집분까지 pHash 전수 검사를 했다고 보고하지 않는다.
- 실제 YOLO 파일·ignore sidecar를 재읽고, 합성 정답 예측을 넣어 평가기의 16종 GT 개수와
  ignore 제외가 일치함을 CPU로 확인했다. 모델의 성능 평가 결과가 아니다.

기존 `macro_f1`의 GT≥20 정책은 유지하고 `macro_f1_fixed16`을 추가했다.
16종 모두 complete GT가 있을 때 전체 클래스 F1을 같은 비중으로 평균한다.
소표본 목록을 함께 반환하며 이 수치를 2%p 회귀 자동 중단 기준에 바로 연결하지 않는다.
현재 구성은 16종 **개발 진단용**으로 사용할 수 있지만 최종 배포 수락/새로운 비공개 시험을
대체하지 않는다. 기존 heldout에서 이미 살펴본 자료의 이력도 유지했다.

재현 입력은 `artifacts/camera-validation16-v1/review/reviewed-records-r2.jsonl`,
도구는 `tools/perception/prepare_camera_validation16.py`, 설정은
`config/experiments/camera_bbox_validation16_v1.json`이다. evidence의 summary/audit에 SHA를 고정했다.
데이터 폴더의 `reservation.json`은 다음 train 통합에서 제외할 source/group/hash 목록이다.
일반 학습으로 잘못 쓰지 않도록 data.yaml의 train 경로는 의도적으로 유효하지 않다.

갤러리: **E307** `http://localhost:8768/camera-validation16-v1/gallery-r2/index.html`.
클래스 필터가 있으며 녹색 양성·자홍색 ignore를 표시한다. 목차에서도 날짜/E 번호로 찾을 수 있다.
기존 618장과 frozen training supervisor는 자동 교체하지 않았다. 학습 재개 판단 전
초기 기준선과 epoch 4를 이 평가본의 동일 confidence 0.40으로 비교하는 것이 다음 단계다.
이 작업에서는 새 GPU 추론·optimizer update·운영 모델 변경을 수행하지 않았다.


## 배경 손실 수정·전체 본학습 재시작 — 2026-10-13

**작은 GT·ignore·거리 분석(같은 날, 학습 유지):** `analyze_camera_small_supervision.py`로 train 53,588장의 주석 전체를 집계하고, source×view당 고정 hash 표본 최대 8장(총 581장)의 배경 grid와 ignore feature-cell 겹침을 확인했다. 추가 양성 제외 후보(원본과 960 양쪽 검사)는 짧은 변/면적 4/32에서 47,444개, 4/64에서 83,009개, 6/64에서 92,108개, 8/128에서 128,608개다. 2epoch 저장 예측에 같은 크기 필터를 사후 적용하면 FP/TP 제거가 각각 254/3, 429/6, 472/7, 574/14다. 이는 원래 GT를 고정한 출력 필터 민감도이며 해당 정책으로 재학습한 성능이 아니다. 현재 FP 976개 중 작은 것만 제거해도 상당수 FP가 남는다.

581장 표본에서 ignore 교차 footprint는 tensor 면적 기준 stride8/16/32 평균 0.0396/0.1026/0.2915%다. 이미 허용된 semantic-safe 배경에서 ignore 때문에 추가 차단된 비중은 stride8 평균 약 0.000036%p, stride16/32는 0이었다. 따라서 tiny-ignore의 cell 확장이 넓은 배경 손실 제거의 주원인이라는 가설은 현재 표본에서 지지되지 않는다. 이 수치는 assignment의 정상 양성 우선 복원을 제외한 기하 분석이며 모든 train의 실제 gradient 분포를 측정한 것이 아니다. 미확인 semantic 영역·건물 등은 원래 safe grid 바깥인 별도 문제다.

SHA가 일치하는 검수 ledger와 같은 run의 Capture 요청 직전 Ego 상태(age≤0.2s)를 연결했다. 직접 native ID가 연결된 양성 147,478개 중 약 80m 밖은 39,937개(27.1%), 120m 밖은 17,643개다. 양성 115,424개는 거리 미확인으로 남겼다. 거리는 static actor pivot과 요청 시점 Ego의 3D 거리이며 노출 시점 광학 depth·날씨별 가시거리가 아니다. 복합 actor ID를 추정 연결하거나 depth PNG encoding을 추측하지 않았다. RGB 크롭에서 안개 속 작은 물체의 식별 곤란과 같은 크기라도 명암/형상 차이가 확인되어, 일괄 거리 cutoff는 채택하지 않았다.

**판정:** 현재 학습을 계속한다. 4/32는 후속 대조 후보지만 즉시 양성 삭제·배경 강제화·ignore 범위 축소를 승인할 증거는 부족하다. 아직 수렴하지 않은 1/2epoch 저장 결과의 관찰이므로 4epoch 이후에도 같은 평가 규칙으로 비교한다. 향후 확정된 데이터/손실 변경은 새 immutable 데이터 revision과 새 run에서 현재 체크포인트를 출발점으로 사용하고 기존 optimizer 이력과 정책 경계를 명시한다. 데이터가 바뀐 것을 동일 실행의 단순 resume로 숨기지 않는다. 공식 사전학습부터 재시작은 기존 가중치의 잘못된 학습이 회복되지 않는 증거가 있을 때 판단한다.

근거와 원본 크기/3배 확대 갤러리: `artifacts/camera-small-supervision-analysis-20261013-r3/`. `summary.json`, `grid-sample.jsonl`, `evaluation-sensitivity.json`, `crop-index.json`과 실행 도구 snapshot을 보존했다. r1은 직접 run ledger SHA 불일치로 거리 미확인, r2는 SHA 고정 검수 ledger 연결, r3는 운영 차체 mask와 실제 OpenCV resize를 적용한 크롭이다. 정량 통계와 v9 학습 데이터는 변경하지 않았다.

**09:13 KST 재개:** 2epoch 평가 후 person/motorcycle의 연속 회귀로 자동 일시정지됐으나, 사용자가 멈추지 말고 학습을 계속하도록 지시했다. `--continue-on-regression`으로 회귀는 경고·평가 기록만 남기고 정지하지 않는다. `last.pt`의 완료 2epoch에서 3epoch로 재개했으며 optimizer/EMA 및 E2E loss schedule updates=2 복원을 확인했다. 데이터·학습 인자·40epoch 일정과 원래 첫 24시간 종료 시각(10월 14일 04:08 KST)은 유지한다. 각 epoch 저장·4epoch 및 이후 4epoch 간격 평가와 오류 중단은 유지한다. 실행 컨테이너는 `asmc-camera-main-v9-resume-r2`, 변경 전 상태·supervisor와 재개 SHA 근거는 본학습 산출물의 `resume-20261013T001314Z/`에 보존했다. 운영 승격 조건은 완화하지 않았다.

사용자가 전수 검사 후 짧은 재학습 대조 없이 본학습 재시작을 승인했다. `camera_objects_bbox_v9-background-r3`은 v8의 train 53,588장·양성 262,902개·ignore 261,415개를 유지한다. 기존 selection/heldout 소속과 새 16종 개발 평가본의 train 제외도 유지했다.

**전수 검사 범위:** train+기존 평가 54,829장 전부의 RGB 파일 SHA/디코딩 픽셀·크기·render 품질·모든 박스/고정 클래스 ID·실제 YOLO 라벨을 확인했다. 사용 가능한 Capture semantic/instance/BBox의 동기 파일 SHA와 원본 GT 크기·양성 actor 가시성도 재검사했다. 구조/정합성 격리 0장, 확인된 split 누출 0이다. 모든 RGB의 의미를 사람이 한 장씩 검수했다는 주장은 하지 않는다. 위험 대표 30장의 RGB/GT/배경 영역 overlay를 별도로 검토했다.

**수정 정책:** 전체 partial 이미지를 complete로 바꾸지 않았다. Capture 36,444장(검수 negative 2,000장 포함)은 semantic의 도로/차선/보도/하늘 및 정확히 연결된 정상 객체 instance 영역에 배경 BCE를 허용한다. 미확인 standing object/building/etc, ego, padding, 극소/ignore 객체 영역은 보수적으로 제외한다. 8px feature cell의 모든 픽셀이 안전해야 허용하고 stride16/32도 안전 영역을 다시 축소한다. GT 없는 partial 16,144장은 문서화된 정답 박스 내부에서 해당 클래스의 비할당 cell에만 억제 손실을 허용한다. 다른 미주석 클래스 전체를 배경으로 보지 않는다. 기존 complete Roboflow 1,000장은 일반적인 배경 손실을 유지한다.

영역별 배경 grid는 NPZ sidecar와 SHA로 보존하고 loader가 letterbox/반전과 함께 전달한다. 두 E2E head에서 ignore 우선 제외와 정상 양성 우선 적용을 유지한다. CPU 회귀 45개 통과, 실제 v9 Capture/RF/complete/negative 표본을 읽은 CPU backward에서도 배경 gradient 복원을 확인했다(합성 모델·optimizer update 0; 정확도 시험은 아님). v9의 GT 기반 safe cell 평균은 960 tensor의 47.77%이며, 실제 배경 손실 양은 head 할당·ignore에 따라 달라진다.

**본학습:** `artifacts/camera-bbox-main-v9/run-20261013-r1/`, 컨테이너 `asmc-camera-main-v9-r1`. 공식 YOLO26l 사전학습 가중치에서 새로 시작하며 기존 4epoch 가중치는 보존했다. imgsz960·batch4·유효 batch16·AMP·AdamW/LR0.001·40epoch 최대 일정·첫 24시간 budget를 유지한다. GPU 200batch 사전 점검을 통과했으며 점검 가중치는 본학습에 사용하지 않았다. 모든 유효 train 이미지를 epoch마다 사용하고 최대 2회 반복한다. 본학습 오류/유한 손실과 첫 진행 기록을 확인했다.

주 평가는 검수한 16종 150장, 기존 618장은 별도 회귀 진단이다. Confidence0.40에서 1/2/4epoch와 이후 4epoch마다 평가한다. 소표본 신규 클래스도 개발용 평균에서 누락하지 않으며 소표본 상태를 명시한다. 기존 충분한 표본 클래스의 recall 회귀 감시는 유지한다. checkpoint·optimizer·EMA·scheduler 재개와 E2E loss schedule epoch 일치를 검사한다. 평가 성능은 아직 새 학습 epoch 완료 전이므로 개선을 확정하지 않는다. 운영 모델/ROS/fusion과 PR 병합은 변경하지 않았다.

근거: `config/baselines/evidence/camera_background_spatial_v2/`. 전수 검사 manifest: `artifacts/camera-background-full-audit-20261013-r2/audit.jsonl`. v9-r1은 초기 원본 manifest 경로 불일치의 실패 초안, r2는 전수 감사 산출물, r3은 절대 경로 metadata로 고정한 본학습 버전이다. 원본 manifest와 다른 중단 Capture는 SHA가 정확히 일치하는 기존 `-rgb-verified-r1` snapshot으로 확인했으며 실패 증거도 보존했다.

[전수 감사·배경 영역 갤러리](http://localhost:8768/camera-background-full-audit-20261013-r2/index.html) · [v9 본학습 실시간](http://localhost:8768/camera-bbox-main-v9/run-20261013-r1/live.html)

## 배경 손실 점검 — 2026-10-13

CPU 실제 forward/backward로 두 E2E head를 검사했다. Partial은 `keep = foreground`가 되어 비할당 위치의 BCE gradient가 0이다. 같은 합성 입력의 complete에서는 one2many/one2one 배경 gradient L1이 48/12, partial에서는 0/0이었다. 양성 gradient는 유지됐다. 검수 negative의 배경 gradient, stride-cell ignore와 정상 양성 우선 처리도 정상이며 기존 LossTests 7개가 통과했다. 수치는 합성 입력 검사이며 실제 학습 gradient 평균이 아니다.

| 자료 | partial | complete | reviewed negative |
|---|---:|---:|---:|
| Capture | 34,441 | 3 | 2,000 |
| Roboflow | 16,118 | 1,000 | 0 |
| AIM | 24 | 0 | 0 |
| bag | 2 | 0 | 0 |

최근 Capture 30,674장은 전체 16종 검수 메타데이터가 없어서 partial이다. 코드상 complete에는 annotation_complete와 명시적 full16 accept/all_targets_annotated 기록이 모두 필요하다. 스폰 actor의 native GT 정확성과 고정 맵 객체까지 포함한 전체 주석 완전성은 다르므로 모두 complete로 바꾸지 않았다.

확인된 문제는 계획대로 구현된 image-level partial 정책이 양성 장면 대부분에서 배경·중복 억제 학습까지 제거한다는 점이다. 오검출의 유력한 기전이지만 실제 기여도와 단독 원인은 재학습 대조 없이는 확정하지 않는다. 무주석 배경 2,000장만으로 객체 주변 중복 억제를 대신할 수 있다고 가정하지 않는다.

다음 수정은 Capture 소스/run별 누락 검사 후 확실한 묶음을 새 revision에서 complete로 승인하고, 제한된 불확실 영역만 ignore로 남기는 방향이다. 검수되지 않은 전체 배경을 negative로 켜지 않는다. 이후 작은 대조로 확인한다. 이번에는 원본·v8·학습 코드·가중치를 변경하거나 학습을 재개하지 않았다. 상세 근거는 `config/baselines/evidence/camera_background_loss_v1/`와 `artifacts/camera-background-loss-audit-20261013/`에 보존했다.

## 16종 평가 후 재개 판정 — 2026-10-13 03:45 KST

사용자의 조건부 재개 요청에 따라 원본 기준선과 4epoch를 새 150장 평가본에서 실제 GPU 평가했다. Confidence 0.40, 전방1280/측방640, 기존 mask를 고정했다.

| 경로 | 기준선 TP / FP | 4epoch TP / FP | 기준선 / 4epoch 16종 macro F1 |
|---|---:|---:|---:|
| 기존 NMS-free | 99 / 18 | 282 / 9,479 | 0.3396 / 0.0677 |
| 별도 NMS 대조 | 101 / 18 | 270 / 962 | 0.3466 / 0.3886 |

새 클래스 검출은 증가했지만 중복·오검출이 많아 **변경 없는 장시간 재개는 보류**한다. NMS 대조는 다른 head 경로를 사용하며 기존 결과를 대체하지 않는다. 16종 평균은 COCO에 없는 클래스의 0점도 포함하므로 기존 7종 보존을 따로 확인해야 한다. NMS-free의 person 19/22·motorcycle 20/23은 기준선과 같지만 오검출까지 해결됐다는 뜻은 아니다.

Train의 partial 50,585장(94.4%), complete 1,003장, reviewed_negative 2,000장을 확인했다. Partial은 비할당 배경 분류 손실이 제외된다. 배경·중복 억제 부족의 원인 후보이며 인과관계는 아직 확정하지 않았다. 두 head의 E2ELoss 학습은 확인했다. 다음은 Capture/소스별 주석 완전성 및 배경 손실 노출·E2E schedule 재개 검사와 작은 대조다. 일괄 complete 전환은 하지 않는다.

학습 컨테이너는 기존 paused_regression 상태를 유지하고 optimizer update·운영 변경은 하지 않았다. 원본 예측·SHA·NMS 대조와 사례는 `artifacts/camera-validation16-resume-20261013/`, 판정은 evidence `resume-decision-20261013.json`이다.

## v8 YOLO26l 본학습 — 2026-10-12 실행 중

사용자 합의로 YOLO26l detection 하나를 먼저 본학습한다. YOLOE는 개선이 부족할 때
검토하며 운영 모델·ROS 계약·fusion 정책과 #81/#82의 미병합 상태는 유지한다.

- 데이터는 `camera_objects_bbox_v8-pretrain-r1`, train 53,588장·양성 262,902개·ignore 261,415개다.
  매 epoch 전체를 포함하고 최대 2회 반복한다. epoch 0은 60,816노출/15,204batch다.
- 공식 YOLO26l 원본 가중치에서 고정 16종으로 시작한다. 이전 회귀 가중치를 사용하지 않는다.
  이름별 기존 7종 분류 행 전이를 실제 로그에서 확인했다. 전체 26,201,016개 파라미터를 학습한다.
- imgsz960, batch4, 유효 batch16, AMP, AdamW/lr0.001/weight decay0.0005, warmup1,
  seed20261011과 v8 ignore/partial·Ego mask·증강 정책을 고정한다.
- `asmc-camera-main-v8-r1`은 공유 컨테이너와 같은 고정 이미지의 독립 학습 컨테이너다.
  data와 D 드라이브 AIM 원본은 읽기 전용, 네트워크는 격리했다. 실제 코드는 실행 폴더의
  `code/`에 복사하고 SHA를 기록했다. 학습 중 worktree 수정은 실행 코드에 반영되지 않는다.
- 시작 2026-10-12 19:43 KST, 첫 실행 종료 시한 2026-10-13 19:43 KST다.
  최대 40epoch 일정을 고정하며 시간 부족 시 완전한 epoch에서 저장한다. 다음 epoch 예상시간의
  1.2배와 평가 여유 600초가 남아야 다음 epoch를 시작한다. 완료 epoch 수는 실제 기록을 따른다.
- GPU 200batch 점검은 batch4/AMP로 통과했다. optimizer step 50회 중 초기 AMP scale 조정으로
  6회 생략, 유한 gradient 44회를 기록했다. 비유한 loss·OOM은 없었다. 점검 가중치는 폐기하고
  본학습은 원본에서 다시 시작했다. CPU 관련 검사 38개 통과, hosted CI는 별도 상태다.

본학습 실행 위치는 `artifacts/camera-bbox-main-v8/run-20261012-r1/`이다.
`supervisor.json`은 전체 진행, `main/run.json`은 25batch마다 갱신되는 실제 진행이다.
`main/fit/weights/epochN.pt`는 optimizer·scaler·EMA·실제 학습 모델·scheduler를 보존한다.
재개 시 EMA가 아닌 실제 학습 모델을 복원하고 optimizer step·EMA update 일치를 확인한다.
실제 재개 검증은 첫 epoch 완료 이후 기록되며 사전 CPU 검사만으로 완료했다고 표시하지 않는다.

중간 평가 618장은 v8 selection과 과거 파일럿의 selection/heldout 600장을 중복 제거해 구성했다.
파생 진단 manifest에서만 evaluation role을 부여하며 원본 split은 변경하지 않았다.
Train 이미지/픽셀/계보 누출 0, complete121장/partial497장이다. person204·motorcycle138 GT로
기존 회귀를 확인한다. dog/bicycle/traffic_cone/stroller GT가 없고 deer1개이므로 해당 종류의
일반화·회귀 검증 완료를 주장하지 않는다. 과거에 여러 번 본 평가 자료이며 blind test가 아니다.

1/2/4epoch 및 이후 4epoch마다 confidence0.40에서 외부 평가한다. 기존 7종 중 GT20개 이상인
클래스 recall이 원본 기준선보다 2%p 넘게 감소한 상태가 두 점검 연속이면 자동 중단한다.
Partial GT는 recall만, complete GT20개 이상 신규 클래스만 macro F1에 포함한다.
회귀 조건을 통과한 최고 점수를 `best-development.pt`로 보존한다. stock `best.pt`와 다르다.
이 결과는 본학습 후보이며 운영 교체는 TensorRT·3뷰 통합·전방 fusion·독립 MORAI 주행 시험 후다.

2026-10-12 23:16 KST에 초기 회복 가능성을 확인하기 위해 2epoch 체크포인트에서 한 차례
재개했다. 1/2epoch 평가에서 motorcycle recall은 116/138 → 90/138로 원본 126/138보다
낮아 자동 중단됐지만, person은 87/204 → 139/204, car는 18/148 → 57/148로 회복했다.
손실 NaN·OOM·재개 상태 오류는 확인되지 않았으며, batch4·누적4회·유효 batch16,
AdamW·초기 LR0.001·warmup1epoch와 고정 데이터/코드는 그대로 유지한다.
다음 평가 지점은 4epoch다. 기존 연속 회귀 중단 규칙과 최초 24시간 종료 시각
(2026-10-13 19:43 KST)은 변경하지 않았다. 이 재개는 초기 중단을 검토한 한 차례의
계속 실행이며, 이륜차 회귀·많은 오검출의 해결이나 최종 수렴을 확인한 것이 아니다.
중단 직전 상태와 판단은 실행 폴더의 `continuation-history/` 및
[재개 근거](../../../config/baselines/evidence/camera_bbox_main_v8/continuation-after-epoch2.json)에 보존했다.

실행 진입점은 [본학습 감독기](../../../tools/perception/run_camera_bbox_main.py),
[고정 설정](../../../config/experiments/camera_bbox_main_v8.json),
[학습기](../../../tools/perception/train_camera_bbox_pilot.py)다. 기존 인자를 유지하면서 학습기에
`--epoch-budget`, `--reserve-seconds`를 추가했다. 일반 YOLO CLI는 v8에 사용할 수 없다.

[실시간 진행](http://localhost:8768/camera-bbox-main-v8/run-20261012-r1/live.html)과
[평가·실패 갤러리](http://localhost:8768/camera-bbox-main-v8/run-20261012-r1/index.html)는
[전체 목차](http://localhost:8768/)에서 접근한다. GPU batch 시간은 데이터 로딩을 포함한
전체 epoch wall 시간과 다르며, 초기 예상만으로 15–20epoch 완료를 확정하지 않는다.

## v8 전처리·전체 통합 — 2026-10-12

현재 worktree는 `asmc-camera-data-audit`, 브랜치는 `feat/capture-route-collection`이다.
기존 v7과 최신 승인 Capture 12개 부분집합의 train 후보 53,411장·530,009주석을 입력으로
사용했다. 이전 8,000장 파일럿 제한과 소스별 채택 상한은 v8에 적용하지 않는다.
#81·#82는 이번 작업에서 병합하지 않는다. 운영 기본 모델·ROS 계약과 현재 마운트는 유지한다.

| 결과 | 수량 |
|---|---:|
| 유효 양성 이미지 | 51,588 |
| 검수 후 추가한 배경 이미지 | 2,000 |
| 최종 train | **53,588** |
| train 양성 / ignore / 제외 주석 | **262,902 / 261,415 / 2,358** |
| 양성이 없어 별도 보관한 후보 이미지 | 1,823 |
| selection / heldout | **113 / 1,128** — 기존 소속 유지 |
| epoch 0 노출 이미지 | 60,816 — 전체 포함, 이미지당 최대 2회 |

ignore 객체도 원본·manifest·sidecar에 보존한다. 양성이 없는 1,823장을 빈 정답으로
전환하지 않았으므로 위 train 주석 수에는 이 별도 보관본의 주석이 포함되지 않는다.
완전 가림 native box는 이전 Capture 변환부터 양성으로 내보내지 않으며 원본 GT와 부모
manifest에 남아 있다. 유모차의 확인된 아기 component만 person에서 제외하는 정책도 유지한다.

| 학습 클래스 | 원본 등장 이미지 | 유효 양성 이미지 | 반복 후 노출 |
|---|---:|---:|---:|
| person | 30,055 | 23,019 | 25,691 |
| dog | 12,324 | 7,281 | 8,868 |
| deer | 17,105 | 10,310 | 11,988 |
| bicycle | 14,665 | 9,530 | 11,214 |
| car | 8,280 | 7,517 | 8,704 |
| motorcycle | 16,874 | 9,337 | 10,868 |
| bus | 6,809 | 6,244 | 8,067 |
| truck | 5,479 | 4,882 | 6,773 |
| bollard | 14,877 | 6,743 | 8,164 |
| box | 18,554 | 9,754 | 11,238 |
| concrete_barrier | 18,653 | 11,200 | 13,014 |
| steel_barricade | 18,321 | 13,010 | 14,798 |
| traffic_drum | 12,557 | 6,036 | 7,617 |
| plastic_barrier | 19,093 | 11,197 | 12,809 |
| traffic_cone | 14,566 | 6,345 | 7,842 |
| stroller | 19,494 | 11,260 | 12,560 |

여러 클래스가 함께 있는 이미지는 각각에 집계하므로 열 합계는 전체 이미지 수와 다르다.
같은 Capture 연속 구간·증강 계보·배치의 여러 날씨를 독립 장면으로 세지 않는다.
세부 종류·출처·뷰·split·계보·크기 집계는
[coverage 증거](../../../config/baselines/evidence/camera_pretrain_v8/coverage.json)에 있다.
날씨는 54,827장 unknown, 2장 sunny이며 터널 여부도 기존 행에서 확인되지 않아 unknown이다.
배경 외형으로 날씨·터널을 추측해 커버리지를 부풀리지 않는다.

### 전처리와 supervision

[전처리 정책](../../../config/datasets/camera_pretrain_policy_v1.json)은 원본 영상 또는 960 tensor
중 한쪽이라도 짧은 변 `<2px` 또는 면적 `<16px²`이면 ignore로 지정한다. Capture 가시 instance
픽셀 `<4`도 ignore다. 경계값 자체는 통과한다. 부분 가림은 가시 instance를 감싸는 bbox를
사용하고, 낮은 픽셀 점유율만으로 얇은 자전거·철제 방벽을 제거하지 않는다.
차체 mask 후 남은 instance 픽셀로 bbox를 재계산하며 부분 겹침만으로 양성 전체를 삭제하지 않는다.
갤러리의 차량 24/12px·기타 16/8px 표시 기준은 v8 학습과 새 평가에 적용하지 않는다.

RGB 검사에서 **actor semantic GT에 없는 고정 지도 드럼·방벽**이 확인됐다.
정확한 actor GT와 16종 전체의 주석 완전성은 구분한다. 새 Capture는 확인된 양성을 유지하면서
partial로 사용한다. 명시적인 전체 16종 RGB 검수 근거가 있는 이미지에만 complete를 부여한다.

| train 이미지 상태 | 수량 | 손실 |
|---|---:|---|
| complete | 1,003 | 일반 양성·배경 손실, ignore cell의 배경 BCE 제외 |
| partial | 50,585 | assign된 양성의 분류·박스 손실, 미주석 배경 BCE 제외 |
| reviewed_negative | 2,000 | 검수한 빈 영상의 배경 BCE 유지 |

미세 ignore도 feature stride cell과의 교차로 적용한다. 정상 양성 foreground가 겹치면 양성을
우선한다. YOLO26·YOLOE detection의 one-to-many와 one-to-one 모두에 같은 처리를 연결하며
원래 assigner·E2E 가중치 스케줄·box/DFL 계산은 유지한다. 공유 Ultralytics 설치는 수정하지 않는다.

배경 후보 14,145장 중 run×view 12묶음에서 각 50장, 총 600장을 RGB로 확인했다.
문제가 없었던 5묶음은 묶음 채택하고, 지도 물체가 보인 7묶음은 확인한 개별 영상만 사용했다.
최종 2,000장은 exact 중복과 평가 계보를 다시 검사했다. GT 부재만으로 모두 배경이라고
처리하지 않았으며 검수 주체·표본·22개 예외·SHA는 `background-reviews.json`에 기록했다.

### 입력과 epoch 순서

[전용 학습 설정](../../../config/experiments/camera_bbox_pretrain_v8.json)은 `imgsz=960`,
`rect=False`, `multi_scale=0`이다. 전방1280×720은 960×540과 위아래210px padding,
측방640×480은 960×720과 위아래120px padding으로 들어간다. 카메라마다 별도 학습 표본이며
운영 2+1 batch와 다르다. 뷰가 확인된 자료만 운영 Ego mask를 적용한다.
unknown 뷰 18,357장에는 mask를 추정하지 않는다.

HSV와 수평 반전만 유지하고 Mosaic·MixUp·CopyPaste·임의 회전·이동·축소는 끈다.
양성·ignore 좌표를 동일하게 resize/padding/반전한다. 클래스별 **유효 양성 등장 이미지 수**의
역제곱근으로 최대 한 번 추가 반복하며 seed `20261011`을 고정한다. 기본 사용량은 모든 이미지
1회다. optimizer 경계 재개를 위해 epoch 길이는 유효 batch 16의 배수로 맞추고 반복 상한을
지킨다. epoch 0의 batch 내 같은 correlation group 충돌은 7건으로 기록하며 완전 분산으로
과장하지 않는다. epoch별 순서는 달라지되 총 길이는 일정하다.

### 저장·검증·후속 학습 연결

```text
data/dataset_versions/
├── camera_objects_bbox_v8-positive-stage-r1/ # 양성 준비 단계, 학습 불가
└── camera_objects_bbox_v8-pretrain-r1/       # 최종 통합본
    ├── data.yaml                            # 고정 16종 ID
    ├── manifest.jsonl                       # 원본·클래스·supervision·출처
    ├── labels/{train,val,test}/             # 양성 YOLO bbox만
    ├── supervision/{train,val,test}/        # ignore와 완전성 정보
    ├── images/{train,val,test}/             # 원본 symlink
    ├── epoch-000-order.jsonl
    ├── exclusions.jsonl                    # ignore-only 등 별도 보관
    ├── policy.json / background-reviews.json
    ├── pretrain.json / summary.json / verification.json
    └── _SUCCESS                             # 최종 metadata SHA
```

원본 이미지·이전 버전은 수정하지 않았다. 일반 CLI 차단 전의 검증본은 `camera_objects_bbox_v8-pretrain-r1-before-cli-guard`로 보존했다.
초기 픽셀 해시 형식 불일치 실패본도
`camera_objects_bbox_v8-pretrain-r1-incomplete-hash-diagnostic`으로 보존했다.
구형 v7의 dimensions+RGB 해시와 새 Capture의 RGB-only 해시를 각각 검증한 뒤 공통 해시로
통일했다. 성공 검증은 실제 YOLO·sidecar 재읽기, 모든 RGB SHA, split/계보 누출, 부모 manifest
불변성을 포함한다. 확인된 구조 오류·미해결 클래스·split 누출은 0이다.

현재 worktree에서 다음은 **읽기 전용 재검증**이다. 보고서를 지정하면 기존 파일을 덮어쓰지 않는다.

```bash
python3 tools/perception/verify_camera_pretrain.py \
  --dataset ../data/dataset_versions/camera_objects_bbox_v8-pretrain-r1 --rgb-sha
```

`prepare_camera_pretrain.py`는 고정 inventory의 부모 버전에서 새 양성 stage를 생성하고,
`prepare_camera_backgrounds.py`가 제안한 표본의 승인 기록을 `finalize_camera_pretrain.py`가
결합한다. 모두 새 output 경로를 요구한다. `report_camera_pretrain.py --dataset … --output …`로
원본/유효/반복과 계보 커버리지를 재집계할 수 있다.

후속 학습은 `train_camera_bbox_pilot.py --pretrain`과 위 v8 설정을 사용한다. 기본 학습기로
`data.yaml`에는 기본 YOLO CLI의 경로 검사를 실패시키는 보호 경로를 둔다. 실제 입력은 전용
trainer가 검증 manifest에서 읽는다. 기존 실행 진입점은 `--pretrain`이 없으면
GPU 패키지 import 전에 차단한다. 최종 `_SUCCESS`, metadata·manifest·정책 SHA, 클래스 순서와
원본 경로도 검사한다. 검증한 버전은 **Ultralytics 8.4.150**이다.
YOLOE는 기존 detection YAML과 PE trainer를 유지하며 segmentation 학습을 시작하지 않는다.

학습 컨테이너에는 workspace data와 **`/mnt/d/asmc_aim/dataset` 원본 경로의 읽기 전용 마운트**가
필요하다. 현재 `asmc-ros-noetic`에 D 드라이브가 없어 전체 loader preflight가 실패하는 것을
확인했다. 이번에 공유 마운트는 바꾸지 않았고 데이터가 빠진 상태에서 학습하도록 fallback하지
않는다. CPU 검증은 /tmp의 격리 코드와 실제 v8 5개 표본으로 수행했다.
중간 체크포인트는 `evaluate_camera_bbox_pilot.py --selection-only`로 heldout을 읽지 않고
확인하며 최종 검증에서만 heldout을 사용한다. 재개는 기존 epoch 경계 정책과 v8 metadata SHA를
묶는다. 증강 RNG의 완전한 연속성까지 보장한다고 표시하지 않는다.

로컬 손실·입력·기존 회귀 **60개 통과**, optimizer update **0**이다. hosted CI 결제/한도 차단은
로컬 통과와 구분하며 이 작업에서 hosted CI 성공을 주장하지 않는다.
precision/F1은 complete GT에서만 계산한다. selection의 충분한 GT는 person에 편중됐으며,
다수 종류는 heldout에 없거나 partial뿐이다. **학습 입력 준비 완료와 16종 배포 평가 준비 완료는
다르다.** 후속 selection 보강 없이 전체 16종 성능을 확정할 수 없다.

[E300 전처리 갤러리](http://localhost:8768/camera-coco7-live-v1/v8-pretrain-20261012/index.html)는
양성·극소 ignore·차체 mask·partial·배경과 배경 채택에서 제외한 지도 물체 예외를 보여준다.
전체 목차는 [기존 갤러리 주소](http://localhost:8768/)를 사용한다.
요약·재검증·커버리지는 [v8 증거](../../../config/baselines/evidence/camera_pretrain_v8/summary.json)에
고정하며 다음 단계는 가중치 재학습과 중간 selection 평가다.

## 현재 기준 — 2026-10-10 v2 16종

이 절과 [v2 라벨 정책](../../../config/datasets/camera_object_labels_v2.json),
[v2 소스별 대응표](../../../config/datasets/camera_object_source_labels_v2.json)가 현재 합의다.
아래 v1/2026-10-09 결과·검수·export는 당시의 증거로 보존하며 현재 분류 계약과 구분한다.
**PR #78의 데이터·샘플링은 main에 병합됐다. 현재 #80의 bbox 파일럿은 Draft이며 운영 실행은 수정하지 않는다.**
최신 결과는 [사람·이륜차 회귀 진단](#2026-10-11-사람이륜차-회귀-진단--재학습-없음)과 [16epoch 비교 결과](#16epoch-비교-결과--본학습-후보-선정-보류), 샘플링은 [bbox 파일럿](#bbox-파일럿-샘플링--2026-10-11), 전체 자료 수치·적용 정책은 [Capture actor 보강 결과](#capture-actor-연결-보강--2026-10-11-v5-capture-r1)를 먼저 읽는다.
아래 v4의 엄격한 채택 기준은 별도 보존 자료의 기준이며 v5 전체의 검수 완료를 뜻하지 않는다.
2026-10-11 추가 검토는 [teacher 대안과 RF-DETR](#2026-10-11-teacher-대안과-rf-detr-검토), 병합 범위는 [#78 main 통합](#2026-10-11-pr-78-병합-범위와-main-통합)을 따른다.
최신 후속 범위는 [bbox·AIM 세 폴더·RF-DETR 선별](#2026-10-11-후속-합의--bbox-우선aim-범위rf-detr-선별)을 먼저 확인한다.


MORAI 에셋명 / 원본 데이터 라벨 / 추론 prompt / 학습 클래스 / 소비 그룹을 별도로 보존한다.
MORAI 사용자 표기는 실제 gRPC API 이름이 아니며, 미확인 API 이름은 null로 둔다.

| MORAI 사용자 표기 | 학습 클래스 | 소비 그룹 |
|---|---|---|
| deer | deer | animal |
| bicycle | bicycle | vehicle |
| scooter + sportsbike | motorcycle | vehicle |
| bollard | bollard | obstacle |
| cargobox + woodbox | box | obstacle |
| concrete_barricade + concrete-barrier | concrete_barrier | obstacle |
| steel_barricade | steel_barricade | obstacle |
| PE drum + redbarrel + trafficbarrel + yellowbarrel | traffic_drum | obstacle |
| pe firewall | plastic_barrier | obstacle |
| color cone + rubber cone | traffic_cone | obstacle |

학습 후보 16종은 `person, dog, deer, bicycle, car, motorcycle, bus, truck,
bollard, box, concrete_barrier, steel_barricade, traffic_drum, plastic_barrier,
traffic_cone, stroller`다. 입식 전동 킥보드는 제외한다. 자전거와 steel barricade는 독립
학습 클래스로 유지한다. `box`로 합쳐도 CargoBox/WoodBox의 하위 유형은 평가에 남긴다.

실행 도구의 `canonical`은 검수한 세부 의미다. `training_map`을 거쳐 `training_class`와
`consumer_group`을 생성한다. 예: `wooden_box → box → obstacle`,
`pe_drum/traffic_barrel → traffic_drum → obstacle`. `original_label`,
`original_category_id`, `original_xyxy`, mask와 검수 근거는 유지한다.
`morai_asset_name`은 독립 확인 없이는 null이며 teacher가 반환한 이름과 혼동하지 않는다.

**소스별 확정 대응:** Katri 여섯 소스의 0=car, 1=bus, 2/3=person,
5=PE_Drum, 7=bollard; 4는 탑승자 복합 박스, 6은 dog/deer 혼합이다.
jjj의 8은 CargoBox→box가 기본 의미이나, 이번 RGB 검수에서 PE 방벽·철제·콘크리트
예외 5개 주석을 확인했다. 소스 전체의 8 대응을 바꾸지 않고 annotation ID별로 정정한다.
9는 box/concrete_barrier/steel_barricade/plastic_barrier 혼합이다.
selfcar는 **0=PE_Drum→traffic_drum, 1/2=bollard**로 Katri와 다르다.
Enzo의 barrigate/pe barrigate는 각각 concrete_barricade/pe_firewall 의미에서 새 학습
클래스로 연결한다. tooth-decay의 **Fence=steel_barricade**, Concrete_1/2=concrete_barrier,
PE는 원통 드럼으로 확인한 주석만 traffic_drum으로 연결한다. 새 RGB 검수에서 같은 PE에
사각 PE 방벽도 확인했으므로 이 소스의 PE 묶음은 승인하지 않는다. Park의 box는
WoodBox/PE_Firewall 혼합이므로 개별 분류한다.
그 밖의 animal, object/obstacle, vehicle, static/dynamic, 복합 탑승자는 이름만 바꾸지 않는다.

2026-10-10 사용자는 Roboflow 주석이 각 작성자의 의도에 맞게 정확하며, 우리 학습
분류와 다른 의미를 해석해 활용해야 한다고 재확인했다. 자료가 불량이어서 격리한 것으로
설명하지 않는다. 직접 대응은 기존 박스를 유지해 이름을 변환하고, 여러 종류가 합쳐진
클래스는 목표 종류를 구분한다. 복합 박스는 원본 기준을 보존하고 필요한 경우 구성요소를
별도로 보완한다. 이 의미 확인을 teacher가 만든 새 박스의 승인으로 확장하지 않는다.

### 검수·export의 범위 관리

v1 검수의 종류·좌표 수정은 읽을 수 있다. 다만 새 방벽류가 추가된 v2의 완전성을 자동
승인하지 않는다. 새 검수 JSON의 `review_scope: camera_objects_v2_16`이 같은 영상에서
새 대상까지 확인했음을 나타낸다. 과거 기록은 덮어쓰지 않는다.
사람이 새 박스를 추가할 때 `additions`에 새 id/canonical/xyxy를 기록하고
`human_confirmed: true`와 검수자를 남긴다. teacher 제안은 자동 승인되지 않는다.

### YOLOE-26x 검증 경로

[고정 실험 설정](../../../config/experiments/camera_yoloe_x_teacher_v1.json)에 base18,
rich21, bollard/plastic 표현의 단독·결합 대조를 기록했다. X checkpoint 전용 embedding,
PyTorch FP16 batch1, 전방1280/측방640, 3뷰 Ego mask를 사용한다.
1280 OOM은 failure.json에 보존하고 960을 명시적으로 별도 실행한다. TensorRT는 생성하지 않는다.

[Capture GT 준비기](../../../tools/perception/prepare_capture_teacher_gt.py)는
run·원본 종류·actor ID·연속 frame 구간으로 독립 RGB 검수 결과를 연결한다.
같은 숫자 actor ID의 Obstacle과 Vehicle이 공존할 수 있어 run+ID만으로 합치지 않는다.
Teacher 출력은 GT로 사용하지 않는다. actor 종류 확인만으로 영상의 누락 주석까지
검증됐다고 판단하지 않으며, 완전성 미확인 영상은 precision·teacher 승인에서 제외한다.
2026-10-10 사용자 확인에 따라 Capture GT의 기하 정확성과 극소형 객체의 평가 범위를
구분한다. 작은 박스라는 이유로 잘못된 GT로 판정하지 않는다. `Vehicle/Obstacle`의 세부
학습 종류 미확정은 별개이며 [픽셀 범위 후속](#capture-극소형-gt-평가-범위--2026-10-10)을 적용한다.

[평가·제안 도구](../../../tools/perception/camera_teacher_audit.py)는 selection에서
0.40 대조와 0.10–0.70 탐색을 수행한다. 클래스별 GT/예측 각각 20개 이상,
precision ≥95%인 후보 중 recall 최대(동률이면 높은 confidence)를 선택하고 heldout에
고정한다. heldout에서도 같은 지원 수·precision 조건을 확인해야 teacher 클래스로 승인한다.
세부 의미·16 학습 클래스·4 소비 그룹을 구분하고 사슴→dog는 animal 위치 검출로만 인정한다.

승인된 클래스만 원본 혼합 박스와 일대일 IoU ≥0.5로 연결한다. 제안 파일은
`decision: pending`, `requires_human_confirmation: true`를 남긴다.
사람+자전거 복합 박스를 한 클래스의 박스로 확정하거나 두 클래스로 복제하지 않는다.
Teacher 미검출은 빈 정답이 아니다. 모든 이름 변경·추가 박스는 사람이 최종 확인한다.

진행 순서는 명확한 자료의 v2 범위 검수 → animal/6 → 숫자9 → box/object/obstacle이다.
운영 모델·ROS 계약·차량 제어는 유지하고, 추가 수집·학습은 이번 범위 밖이다.


## 상자·방벽 혼합 라벨 보강과 Capture 활용 — 2026-10-11 v5-obstacles-r1

[고정 실험](../../../config/experiments/camera_training_data_v5_obstacle_reinforcement.json),
[선정·명시적 RGB 판정 변환 도구](../../../tools/perception/prepare_camera_obstacle_review.py),
[결과표](../../../config/baselines/perception_camera_training_data_v5_obstacles.yaml),
[파일 검증](../../../config/baselines/evidence/camera_training_data_v5_obstacles/verification.json),
[Capture 종류 증거](../../../config/baselines/evidence/camera_training_data_v5_obstacles/capture-subtype-evidence.json)를 참조한다.
[보강 갤러리](http://localhost:8768/camera-training-data-audit-v5-obstacles/index.html)에서
원본 RGB 확대 시트와 193장 이름 수정 전후·채택 판정을 볼 수 있다.

tooth-decay 96장, hl-fma_2 64장, jjj 33장의 **객체 확대 516개**를 실제 RGB로 확인했다.
이는 영상의 모든 객체·주석 완전성을 검수했다는 뜻이 아니다. 16개 확대 박스는 종류를
확정하지 않고 남겼다. 같은 영상의 다른 혼합·복합 라벨까지 해소된 경우만 반영해
**166장·731객체**를 추가했다(tooth 93장, hl-fma_2 62장, jjj 11장).
남은 27장에는 tiny subtype 외에도 탑승자 복합·dog/deer 혼합 주석이 있다.

| 학습 클래스 | 이전 전체 객체 | 현재 전체 객체 | 현재 train 객체 |
|---|---:|---:|---:|
| box | 592 | **668** | **459** |
| plastic_barrier | 548 | **585** | **469** |
| steel_barricade | 441 | **505** | **304** |
| concrete_barrier | 2,029 | **2,160** | **1,910** |
| traffic_drum | 17,901 | **18,157** | **17,737** |

`object`의 나무 무늬 상자/검정·노랑 개방형 철제 방책/원통 드럼을 각각
wooden_box→box, steel_barricade, traffic_drum으로 연결했다. tooth의 PE에서 구멍·연결부가
있는 사각 플라스틱 방벽은 plastic_barrier, 원통과 반사띠가 확인되는 객체는 traffic_drum이다.
jjj의 9에서도 콘크리트 방벽·상자·철제·PE 방벽을 주석별로 나눴다. 다른 미검토 영상이나
파일명으로 종류를 자동 전파하지 않는다. 기존 보류 제안은 새 RGB 판정 없이 승격하지 않는다.

최신 산출물은 `data/dataset_versions/camera_objects_bbox_v5-obstacles-r1`이다.
**17,631장·114,019객체**(Roboflow 17,626 + Capture 5)이며
**train/selection/heldout = 16,851/113/667**이다. 원본 박스·16종 ID·평가 라벨·분할
변경 0, 기존 채택 소실 0, exact pixel split 누출 0을 확인했다. 기존 v5의 품질 tier도 유지해
이전 대량 자료를 엄격 검수 완료 자료로 잘못 승격하지 않는다. 관련 CPU 회귀는 97개다.
원본·v4·v5-breadth와 이전 검수 기록을 보존했다.

### Capture에 장애물이 있었는데 적게 활용한 이유

현재 Capture 인벤토리의 원본 라벨은 Obstacle 7,910개, Vehicle 2,478개 외에도
car 12,690, person 5,236, truck 3,024, bus 1,628개가 있다. **Capture 전체가 종류 미상인
것은 아니다.** 다만 Obstacle의 박스·instance actor ID·semantic 대분류 색상은 세부
에셋명과 다르다. 인벤토리의 **Obstacle 7,910개 모두** source_class_name은 Obstacle,
semantic은 `[236,255,2]`로 같아 이 정보만으로 box/방벽/드럼을 확정할 수 없다.
확인한 예시 run의 manifest/state에는 IMU·GPS가 연결돼 있고 객체 모델명을 actor에
연결한 목록은 확인되지 않았다.

장애물 종류 미확정과 예전의 엄격한 영상 완전성 조건이 겹쳤고, 이번 대량 확장 도구는
Roboflow train을 대상으로 하므로 Capture는 아직 5장이다. GT 박스가 부정확하거나
Capture를 학습에 사용할 수 없다는 뜻이 아니다. RGB에서 종류를 확인해 **run + 원본
category + actor ID + 등장 구간**에 연결하면 기존 박스·mask를 활용할 수 있다.
actor ID는 run/category 사이에서 재사용되므로 ID 하나만으로 전체 run에 전파하지 않는다.

### 다음은 짧은 비교 학습

현재 규모는 **부분집합 파일럿을 시작할 수 있는 상태**다. 전체 Roboflow 혼합 주석과
Capture를 모두 정리할 때까지 기다리지 않는다. 다음 작업에서는 이 export를 고정하고
소스·클래스별 샘플링으로 bollard/drum·person/car의 압도적인 비중을 조정한다.
YOLO26l detect와 YOLOE 후보의 짧은 학습을 비교해 box·plastic/steel/concrete 방벽 개선,
기존 차량·사람·강아지 회귀, 오검출·검출 공백과 배포 지연으로 본학습 모델 하나를 정한다.
bicycle/stroller는 아직 비어 있고 독립 selection 커버리지가 부족한 종류는 성능 판단을
보류한다. 이 결손을 모든 학습 시험의 중단 조건으로 삼지는 않는다. 이번 작업은 실제
학습·추가 수집·운영 모델 변경을 수행하지 않았다.

## 대량 활용 train 확장 — 2026-10-11 v5-breadth-r1

사용자는 car에 bus/truck이 섞인 차량 세부 분류는 대회에 큰 문제가 없으며,
쉬운 수정만 유지하고 직접 대응 자료를 대량 활용하도록 결정했다. 이에 **표본 하나의
오류로 소스 전체를 보류하는 조건과 16종 주석 완전성 조건을 train 확장에서 해제**했다.
원본 박스·소스별 의미를 사용하고, 이미 독립 RGB로 확인한 종류 수정은 반영한다.
새 자료 전체를 다시 시각 검수했다고 표현하지 않는다. 신호등·표지판 등의 제외 주석은
학습 txt에서 제거한다. 제외 주석과 대상 주석이 함께 있는 영상은 대상 주석을 활용한다.

[고정 설정](../../../config/experiments/camera_training_data_v5_breadth.json),
[실행 도구](../../../tools/perception/prepare_camera_breadth_dataset.py),
[결과표](../../../config/baselines/perception_camera_training_data_v5_breadth.yaml),
[실제 YOLO 파일 검증](../../../config/baselines/evidence/camera_training_data_v5_breadth/verification.json),
[대량 활용 갤러리](http://localhost:8768/camera-training-data-audit-v5-breadth/index.html)를 참조한다.
전체 [갤러리 목차](http://localhost:8768/)에도 고정 E 번호·날짜로 등록한다.

| 수량 | v4-coverage-r1 | v5-breadth-r1 |
|---|---:|---:|
| 전체 이미지 | 5,013 | **17,465** |
| Roboflow / Capture | 5,008 / 5 | **17,460 / 5** |
| 객체 주석 | 73,030 | **113,288** |
| train | 4,233 | **16,685** |
| selection / heldout | 113 / 667 | **113 / 667** |

train에 **12,452장·40,258객체**를 추가했다. 전체 데이터는 3.48배이며 train은 3.94배다.
새 자료에는 aiv5challenge 1,767장, hl-fma_2 906장, hl-fma_3 4,746장,
jjj 1,109장, mino 2,325장, Park 800장, battery 284장, tooth-decay 353장,
detection-bzetd 161장, yuna 1장이 포함된다. 소스 약칭의 전체 이름은 결과표에 있다.

기존 5,013장은 `previous_reviewed_subset`, 새 12,452장은
`source_mapped_breadth_training`으로 manifest에 구분한다. 새 자료 중 기존 묶음 표본
실패와 관련된 6,718장, 개별 reject 기록이 있던 129장도 포함했다. 검수 결과를 삭제한
것이 아니라 사용자가 허용한 품질 정책을 적용한 것이며, 원래 보류 이유·검수 판정도 남긴다.
새 자료의 `complete_target_annotation_asserted=false`다. 미주석 대상 객체가 있을 수 있어
기존 엄격 검수 자료와 같은 품질로 설명하지 않는다. 차종 혼입을 허용하더라도 dog/deer,
box/방벽 혼합 등을 임의의 클래스에 넣지는 않는다.

| 기존 46,326장 중 아직 미포함인 Roboflow 자료 | 이미지 수 |
|---|---:|
| 종류 미확정 혼합·탑승자 복합 박스 | 14,967 |
| 빈 주석으로 대상 부재 미확인 | 5,764 |
| 제외 라벨만 있는 영상 | 3,909 |
| 중복 대표·분할 충돌 검토 | 3,231 |
| tooth-decay PE의 드럼/사각 방벽 혼합 | 524 |
| 고정 평가 그룹이므로 train에 추가하지 않은 자료 | 461 |
| 잘못된 박스 좌표 | 10 |
| 합계 | **28,866** |

위 표는 이미지별 우선 사유로 중복 없이 집계했다. 원본은 모두 유지한다. 제외 라벨만
있는 영상과 빈 주석을 자동 negative로 쓰지 않는다. tooth-decay의 PE처럼 실제로 다른
형상이 섞인 주석은 개별 확인된 변경만 반영한다. Teacher와 파일명으로 혼합 정답을
확정하지 않는다. 기존 박스 좌표·16종 ID·원본 split·평가 영상과 라벨은 변경 0이며,
실제 파일 재검증에서 17,465장·113,288객체, exact pixel split 누출 0을 확인했다.

재현 명령은 worktree에서 실행한다. 경로는 상위 workspace의 로컬 자료 영역이며
출력 디렉터리는 새 이름을 사용한다. 기존 산출물이 있으면 덮어쓰지 않는다.

```bash
PYTHONPATH=tools/perception ../data/roboflow/.venv/bin/python tools/perception/prepare_camera_breadth_dataset.py \
  --previous ../data/roboflow/curated-v4-coverage-r1/curated.jsonl \
  --reviews ../data/roboflow/visual-review-v4-coverage-r1/reviews.json \
  --labels config/datasets/camera_object_labels_v2.json \
  --config config/experiments/camera_training_data_v5_breadth.json \
  --output ../data/roboflow/curated-v5-breadth-r1-reproduced \
  --export ../data/dataset_versions/camera_objects_bbox_v5-breadth-r1-reproduced \
  --gallery ../artifacts/camera-training-data-audit-v5-breadth-reproduced
```

학습·GPU 실행·운영 모델 변경은 아직 하지 않았다. v5는 **대량 활용 파일럿에 사용할
수 있는 부분집합**이며, 독립 평가가 모든 16종을 덮는 비교 학습 준비 완료와 구분한다.
bicycle·stroller는 여전히 비어 있다. 다음 학습에서는 소스·클래스별 샘플링을 정하고,
기존 v4와 v5의 신규 장애물 개선·기존 객체 회귀·오검출을 같은 고정 평가 자료로 확인한다.
데이터 수 증가 자체를 성능 향상으로 보고하지 않는다.

## 동물·오토바이 보강과 수량 — 2026-10-11 coverage-r1

이전 엄격 검수 부분집합은 `data/dataset_versions/camera_objects_bbox_v4-coverage-r1`이다.
[고정 설정](../../../config/experiments/camera_training_data_v4_coverage.json),
[결과표](../../../config/baselines/perception_camera_training_data_v4_coverage.yaml),
[실제 YOLO 파일 검증](../../../config/baselines/evidence/camera_training_data_v4_coverage/verification.json),
[검수 갤러리](http://localhost:8768/camera-training-data-audit-v4-coverage-review/coverage-review.html)를
참조한다. 아래 mixed-r4와 원본·검수 기록은 그대로 보존했다.

| 수량의 의미 | 이미지 수 |
|---|---:|
| Roboflow 22개 자료의 로컬 확보 파일 | 46,326 |
| 그중 현재 bbox 학습용 데이터에 채택한 Roboflow | 5,008 |
| 함께 채택한 Capture | 5 |
| 현재 정리한 데이터 전체 | **5,013** |
| train / selection / heldout | **4,233 / 113 / 667** |

46,326장은 증강본·프로젝트 간 중복을 포함하며 고유 원본 장면 수가 아니다.
17개 프로젝트는 공개 버전 export, 5개는 공개 snapshot이다. 22개의 최신 버전을
모두 확보했다고 표현하지 않는다. Capture 전체 인벤토리 30,090장은 별도이며,
현재 채택된 5장과 혼동하지 않는다. 위 수량은 학습을 이미 수행한 수가 아니다.

이번에는 전체 RGB **189장**을 검토했다. animal-tohen 50장과 dog/deer 원본 박스 확대,
detection-bzetd의 bike 63장과 bike/Obstacle 확대, bicycle-man 표본 50장,
Enzo dog 100장 중 처음 24장, hl-fma others 2장이다. Enzo 나머지 76장은 검수 시트를
생성했지만 이번에 검토했다고 세지 않는다. 이름 수정 수락 105장 중 2장은 기존 평가
영상과 유사해 보류했고, 실제 **103장·454객체**를 train에 추가했다.
총 객체는 **73,030개**다. 새 객체는 motorcycle 189, person 137, dog 30, truck 29,
deer 24, car 22, traffic_drum 14, bus 7, box 2다.

- `animal`은 원본 RGB에서 dog/deer를 개별 구분했다. 작은 양성 GT와 증강 원본은 유지했다.
- detection-bzetd의 `bike`는 검토한 영상에서 탑승자 없는 오토바이·스쿠터다.
  `motorcycle`로 연결하며 다른 소스의 bike에 같은 규칙을 적용하지 않는다.
- 같은 소스 `Obstacle`은 대부분 드럼이지만 주석 120은 나무상자다. 개별 ID와
  이미지 SHA로 수정하며 이미지 간 annotation ID가 같다고 같은 객체로 합치지 않는다.
- bicycle-man의 박스는 사람+자전거 전체를 포함하며 같은 영상의 차·콘 등이 빠져 있다.
  원본 의도대로 유효한 복합 주석이지만 우리의 분리 기준에는 구성요소와 다른 대상
  주석 보완이 필요하다. 같은 박스를 person/bicycle 두 개로 복제하지 않는다.
- Enzo dog에는 사슴이 함께 들어 있고 앞쪽 주황/검정 철제 방책 주석이 없다.
  이름만 고쳐 채택하지 않았다. hl-fma `others` 두 장은 유모차가 아닌 나무상자이며,
  그중 한 장은 다른 큰 부분 상자가 미주석이라 보류했다.

[이번 보완 목록](http://localhost:8768/camera-training-data-audit-v4-coverage-review/pending-worklist.json)에
미채택 86장의 사유를 남겼다. 기존 채택 소실·split 이동·원본 좌표 변경·확인된 분할
누출은 모두 0이다. 새 자료의 이름 수정에 teacher 예측을 정답으로 사용하지 않았다.

dog/deer train과 motorcycle train은 보강됐지만 독립 selection과 bicycle/stroller는
여전히 부족하다. 전체는 14/16종이며 16종 비교 학습 준비 완료가 아니다.
볼라드·드럼 편중도 남아 있어 다음 학습 단계에서 클래스별 표본 균형을 확인해야 한다.
다음 보완은 자전거 복합 박스·동물 평가 영상의 누락 주석을 먼저 해결하고,
유모차 자료와 독립 평가 장면을 확인하는 것이다. 평가 자료를 train으로 옮기지 않는다.
학습·추가 수집·GPU 실행·운영 추론 변경은 수행하지 않았다.

## JJJ 혼합 라벨 전체 검수 — 2026-10-11 r4

이전 부분집합은 `data/dataset_versions/camera_objects_bbox_v4-mixed-r4`다.
[고정 설정](../../../config/experiments/camera_training_data_v4_jjj.json),
[결과표](../../../config/baselines/perception_camera_training_data_v4_jjj.yaml),
[실제 파일·분할·SHA 검증](../../../config/baselines/evidence/camera_training_data_v4_jjj/verification.json)을
함께 참조한다. [최신 검수 갤러리](http://localhost:8768/camera-training-data-audit-v4-jjj-review/jjj-review.html)는
[전체 실험 목차](http://localhost:8768/)에서 날짜·E 번호로 찾을 수 있다.

이번에는 JJJ의 미검수 **313장을 모두 검토**했다. 앞선 32장과 합하면 345장 전체의
검토가 끝났으며 채택 255장·개별 보완 90장이다. 신규 231장·3,898객체를 포함하고,
기존 채택 영상 1장의 원본 8번 주석도 정정했다. 총 **4,910장·72,576객체**이며
train 4,130장 / selection 113장 / heldout 667장이다.

| 학습 클래스 | 이전 train 객체 | 현재 train 객체 | 현재 전체 객체 |
|---|---:|---:|---:|
| box | 99 | 265 | 474 |
| plastic_barrier | 317 | 401 | 517 |
| steel_barricade | 85 | 219 | 420 |
| concrete_barrier | 211 | 1,171 | 1,421 |

전체 영상과 원본 annotation ID를 표시한 **9번 확대 박스 1,803개·8번 165개**를 확인했다.
열린 A형 철제 프레임, 연속된 받침이 있는 콘크리트 패널, 나무 무늬 상자,
흰색/주황색 사각 PE 방벽을 구분했다. 줄무늬만 보고 철제로 분류하지 않았다.
확인되지 않은 작은 옆면·동물 종류는 보완 목록에 남겼다. 크기 자체가 제외 조건은 아니다.

원본 8번의 개별 예외는 1628/4107→pe_firewall, 4108/6605→steel_barricade,
4508→concrete_barricade다. 이 중 6605/4508이 있는 영상은 다른 누락·복합 박스 때문에
보류 상태여서 제안 이름을 export하지 않았다. 기존 8→cargo_box 규칙을 다른 영상에
일괄 교체하거나 다른 소스의 숫자 의미를 바꾸지 않았다.

[JJJ 보완 목록](http://localhost:8768/camera-training-data-audit-v4-jjj-review/pending-worklist.json)은
90장 각각에 원본 ID·SHA·분할·사유와 다음 작업 종류를 기록한다. 사람+자전거 복합 박스는
구성요소 주석이 필요하고, 미주석 대상이 있는 영상은 해당 박스를 보완해야 한다.
일부 원본 박스는 아래쪽 반사 패딩까지 포함하여 가시 영역 기준의 좌표 수정이 필요하다.
이런 영상에서 주석을 임의로 지우거나 진짜 negative로 바꾸지 않았다.
실제 좌표 수정·박스 추가는 기존 검수 계약에 따른 별도 확인 대상으로 남긴다.
Park 30장·Tooth-yellow 13장의 이전 보완 기록도 유지했다.

실제 YOLO 이미지·라벨 파일을 다시 읽어 고정 16종 ID·좌표·객체 수·SHA를 확인했다.
기존 split 이동·채택 소실·원본 좌표 변경·채택 자료의 미해결 라벨·구조 오류·확인된
split 누출은 모두 0이다. 원본과 r3를 보존하고 새 검수 기록과 export를 별도로 만들었다.
Teacher 예측·사람 검수 여부를 추정해 승인하지 않았으며 작은 양성 GT도 유지했다.

**16종 비교 학습 준비는 아직 보류다.** dog/deer/bicycle/stroller 자료와 train의 motorcycle,
대부분 클래스의 독립 selection이 부족하다. 볼라드·드럼 편중도 남아 있다. 다음은 기존
자료의 동물·자전거·오토바이/유모차를 검수하고, 독립성이 확인된 새 그룹에서 부족한 평가
분할을 확보하는 순서다. 기존 평가 그룹을 train으로 옮기지 않는다. 신규 수집·학습·운영
추론 변경은 시작하지 않았다.

## 혼합 상자·방벽 개별 재분류 — 2026-10-11

아래는 앞선 r3 결과다. 당시 데이터 `data/dataset_versions/camera_objects_bbox_v4-mixed-r3`와
검수 증거는 보존하며, 최신 결과는 위 r4 절을 따른다.
[실험 설정](../../../config/experiments/camera_training_data_v4_mixed.json),
[결과표](../../../config/baselines/perception_camera_training_data_v4_mixed.yaml),
[실제 파일·분할·SHA 검증](../../../config/baselines/evidence/camera_training_data_v4_mixed/verification.json)을 함께 참조한다.
[수정 전후 갤러리](http://localhost:8768/camera-training-data-audit-v4-mixed-review/mixed-review.html)는
[전체 실험 목차](http://localhost:8768/)에도 날짜·고정 E 번호로 연결했다.

207장을 독립 RGB 검수해 156장·1,032객체를 추가했다. 총 **4,679장·68,678객체**이며
train 3,899장 / selection 113장 / heldout 667장이다. 기존 채택 소실·split 이동·원본
박스 좌표 변경·확인된 split 누출은 모두 0이다. 작은 양성 GT를 제외하지 않았으며
teacher 예측을 정답으로 사용하거나 사람이 검수했다고 주장하지 않는다.

| 학습 클래스 | 이전 train 객체 | 현재 train 객체 | 현재 전체 객체 |
|---|---:|---:|---:|
| box | 0 | 99 | 308 |
| plastic_barrier | 0 | 317 | 433 |
| steel_barricade | 36 | 85 | 286 |
| concrete_barrier | 134 | 211 | 461 |

**원본 클래스 묶음 전체를 승인하지 않고 개별 annotation ID를 수정했다.**

- Park `box`: 전체 112장 검수, 82장 채택. 나무상자와 흰색/주황색 PE 방벽을
  각각 `wooden_box→box`, `pe_firewall→plastic_barrier`로 연결했다. 좁은 옆면도
  드럼으로 간주하지 않았다. 원본 `bus`로 표시된 검은 덤프트럭·나무 적재함 화물트럭은
  해당 영상에서만 `truck`으로 수정했다. 다른 소스의 bus 정책은 유지한다.
- JJJ 숫자 `9`: 345장 중 0–31번 32장 검수, 24장 채택. 콘크리트·철제 방책·
  나무상자·PE 방벽을 박스별로 분리했다. 직접 대응인 `8→cargo_box→box`는 유지했다.
  생성만 한 나머지 313장은 검수 완료나 승인 수에 넣지 않았다.
- Tooth `Obstacle_Yellow`: 전체 63장 검수, 50장 채택. 나무 무늬 상자,
  노란 원통 드럼, 열린 A형 철제 방책을 각각 box/traffic_drum/steel_barricade로
  분리했다. 같은 영상의 원통 PE·콘크리트도 함께 확인했다. 사각 방벽까지 섞인
  Tooth의 전체 `PE` 묶음에 대한 일괄 승인은 유지하지 않는다.

**검수한 51장은 개별 보완 대기다.** 여러 상자·트럭을 한 박스로 묶은 경우,
차량·사람의 별도 주석이 필요한 경우, 줄무늬 패널의 종류나 극소형 하위 유형을
판별하지 못한 경우다. 작은 크기 자체가 탈락 기준은 아니며, 확인되지 않은 이름을
붙이지 않았다. 보류 기록의 제안 overrides는 export에 반영하지 않는다.
미검수 313장과 검수 후 보류 51장을 구분한다.

같은 프로젝트에서 반복 촬영한 장면을 새로운 독립 평가 자료로 취급하지 않았다.
이번 추가 자료는 기존 train 그룹이며 Enzo heldout을 train으로 옮기지 않았다.
box는 세 split에 존재하지만 나머지 클래스의 selection이 부족하고,
dog/deer/bicycle/stroller는 여전히 채택 자료가 없다. 볼라드·드럼 편중도 남아 있어
**16종 비교 학습 준비 완료는 아니다.** 다음 검수는 JJJ 나머지 → 복합·누락 박스
보완 → 동물/자전거/유모차와 독립 selection 확보 순서다.

원본·v3·v4·direct-r2·중간 mixed/r2와 검수 기록은 보존했다. mixed/r2는 확대 검수
중 추가 드럼 종류·덤프트럭 의미가 정정되기 전의 중간 결과로, 학습에 사용하지 않는다.
최종 `new-decisions-r3.json`과 합친 `visual-review-v4-mixed-r3/reviews.json`의 SHA는
검증 파일에 고정했다. 운영 모델·ROS·confidence·GPU·학습은 변경하지 않았다.

검수 도구는 기존 준비기를 확장했다. `--all-images`는 고정 50장 표본 대신 해당
소스·클래스의 전체 영상을 생성하고 `--crop-label`은 원본 annotation ID와 픽셀 크기를
표시한 확대 영상을 추가한다. **영상 생성은 자동 승인이 아니다.** 예:

```bash
python3 tools/perception/prepare_camera_direct_review.py \
  --curated "$ASMC_DATA/roboflow/curated-v4-direct-r2" \
  --bucket 'roboflow:parkminseo/final-ftuy2|box' \
  --all-images --crop-label box --output /tmp/park-review-new
```

[수정 전후 보고기](../../../tools/perception/report_camera_label_changes.py)는 저장된
검수 기록과 실제 curate 결과를 비교한다. source SHA·split·원본 좌표가 달라지면
보존됐다고 표시하지 않는다. 초록은 실제 채택, 주황은 보완 대기이며 원본 라벨과
최종 학습 라벨을 나란히 보여준다. 실제 YOLO 파일을 다시 읽어 4,679장·68,678객체를
검증했고 관련 CPU 시험 84개·문서/인지 계약 validator가 통과했다.
Hosted CI는 최종 HEAD에서 별도로 확인하며 로컬 시험 통과를 hosted 통과로 표시하지 않는다.

## 직접 대응 자료 검수와 일괄 포함 — 2026-10-10

사용자의 직접 대응 자료 포함 지시에 따라 고정 표본 RGB와 원본 박스를 확인했다.
**통과 묶음은 표본 몇 장만 남기는 대신 해당 소스·원본 클래스 자료 전체에 승인 범위를
적용했다.** 다만 같은 영상의 다른 대상 라벨, 구조·중복·고정 분할 검사도 통과해야 한다.
원본 박스·클래스는 수정하지 않고 새 export에서 16종 이름으로 대응했다.

[고정 설정](../../../config/experiments/camera_training_data_v4_direct.json),
[최신 baseline](../../../config/baselines/perception_camera_training_data_v4_direct.yaml),
[파일 재검증·표본 SHA 증거](../../../config/baselines/evidence/camera_training_data_v4_direct/verification.json)를
기준으로 한다. 로컬 데이터는 `data/dataset_versions/camera_objects_bbox_v4-direct-r2`다.
[결과 시작점](http://localhost:8768/camera-training-data-audit-v4-direct-review/index.html),
[실제 RGB 표본과 보류 사례](http://localhost:8768/camera-training-data-audit-v4-direct-review/direct-review.html),
[소스×클래스 진행표](http://localhost:8768/camera-training-data-audit-v4-direct-review/review/progress.html)를
[기존 목차](http://localhost:8768/)에서 날짜·E 번호로 찾을 수 있다.

| 자료 | 이미지 | 객체 | train / selection / heldout |
|---|---:|---:|---|
| 직전 v4 보존본 | 527 | 973 | 344 / 1 / 182 |
| 직접 대응 확대본 | **4,523** | **67,646** | **3,743 / 113 / 667** |

이번 검수 기록은 중복을 제거한 RGB 1,047장: 승인 898, 문제 확인 103, 보류 46이다.
Codex의 독립 RGB 검수이며 사람이 확인했다는 뜻으로 `human_confirmed`를 기록하지 않았다.
원본 이미지·표본 시트 SHA와 패널 번호를 결정에 연결했다. Round 1–3은 전 패널을,
Round 4는 0–55만 확인했다. 생성됐지만 확인하지 않은 나머지 패널은 검수 완료가 아니다.
전체 132개 소스×원본 클래스 묶음은 승인 14, 표본 문제 확인 13, 미완료/보류 105다.
승인 14에는 함께 등장하는 제외 라벨 `speed/lane bar` 2개가 포함되며, 이를 학습 대상으로
추가한 것은 아니다. 22개 소스의 전체 검수가 끝났다고 표시하지 않는다.

### 포함한 자료와 개별 예외

- Selfcar 0=드럼·1/2=볼라드: 세 묶음의 고정 표본이 통과했다. 구조·중복 검사 후
  2,548장을 포함했다. 서로 다른 소스의 숫자 ID를 같은 의미로 해석하지 않았다.
- Enzo: WoodBox·볼라드·오토바이/좌식 scooter·철제 바리케이드·PE 드럼 묶음과
  개별 정상 이미지를 포함해 총 666장이다. CargoBox·콘크리트·PE 방벽도 포함되지만,
  dog/deer 혼합이나 가장자리 방벽 미주석 장면은 보류했다. 전부 기존 heldout을 유지했다.
- hl-fma_3: bus 묶음은 통과했다. person/car/truck의 정상 개별 이미지를 더해 677장이다.
  일부 장면의 미주석 상자·PE 방벽·자전거 및 car로 묶인 경트럭은 개별 수정 대상으로 남겼다.
- s-workspace person은 중복 검사 후 112장, Tooth의 확인된 콘크리트·철제 방책·원통 드럼은
  134장, Battery cone은 정상 개별 이미지 40장을 포함했다. 기존 aiv5 341장과 Capture 5장도 유지했다.
- **Tooth PE 예외:** 원통 드럼 외 사각 PE 방벽이 같은 라벨에 섞여 있었다.
  `PE→traffic_drum`을 묶음 전체에 적용하지 않는다. Round 3의 248/283/327은 보류했다.
  원통으로 확인한 개별 이미지만 채택하며 방벽은 다음 재분류에서 plastic_barrier로 확인한다.
- Battery의 콘 박스는 직접 대응하지만 일부 장면에 주석 없는 적백 체크 원통 물체가 있다.
  해당 물체의 목표 종류와 누락 박스를 확인하기 전 관련 장면은 보류했다.
  원본 작성자의 콘·신호등 수집 목적에 맞는 주석과 우리 16종의 완전성을 구분한다.

### 분할·검증과 남은 병목

실제 YOLO 파일 4,523장·67,646객체의 좌표·고정 ID·이미지 SHA를 재검증했다.
채택 자료의 미해결 라벨·구조 오류·확인된 split 누출은 0이며, 기존 76,416개 입력의
split 변경과 이전 v4 채택 자료 소실도 0이다. v3/v4 원본 manifest SHA를 다시 확인했다.
작은 양성 GT와 Capture 12/3/3 run 분할을 유지했다. 표본 표시 기준은 학습 필터가 아니다.

현재 12종이 존재하지만 dog/deer/bicycle/stroller는 아직 채택 자료가 없고,
box·plastic_barrier·motorcycle은 train이 0이다. Enzo heldout을 학습으로 옮기지 않는다.
다음은 jjj 9·Park box 등 기존 train 혼합 라벨을 분리하고, animal 및 탑승자 복합 박스를
검수해 부족한 종류와 독립 평가 자료를 채우는 순서다. 볼라드 49,505객체·드럼 15,339객체가
많으므로 파일 수만 보고 준비 완료로 판단하지 않으며, 파일럿 전에 클래스·장면 균형을 정한다.
**구조적으로 사용 가능한 확대 부분집합이며 16종 비교 학습 준비 완료는 아니다.**
PR #78은 Draft로 유지한다. 추가 수집·학습·운영 모델·ROS·GPU 환경은 변경하지 않았다.

[표본 생성기](../../../tools/perception/prepare_camera_direct_review.py)는 기존 고정 seed 표본을
RGB 시트와 SHA manifest로 렌더링할 뿐 승인 결정을 만들지 않는다. 검수 기록은 로컬에
별도 보존하며 기존 `camera_data_audit.py curate --bbox-only --fixed-class-order`에
새 reviews와 이전 v4 `--split-manifest`를 전달했다. 재실행은 기존 산출물을 덮어쓰지 않도록
새 output/export 경로를 사용한다. engine·teacher는 직접 대응 자료의 채택 조건이 아니다.

## v4 자료 활용 확대 — 2026-10-10

[고정 실행 설정](../../../config/experiments/camera_training_data_v4.json),
[실행 baseline](../../../config/baselines/perception_camera_training_data_v4.yaml),
[검증 증거](../../../config/baselines/evidence/camera_training_data_v4/verification.json)는 직전 v4의 고정 보존본이다.
로컬 [결과·검수 시작점](http://localhost:8768/camera-training-data-audit-v4/index.html)과
[소스×클래스 진행표](http://localhost:8768/camera-training-data-audit-v4/review/progress.html)는
기존 [갤러리 목차](http://localhost:8768/)에도 날짜·E 번호로 연결한다.

| 자료 | 이미지 | 객체 | train / selection / heldout |
|---|---:|---:|---|
| v3 보존본 | 95 | 250 | 3 / 1 / 91 |
| v4 고정 16종 부분집합 | 527 | 973 | 344 / 1 / 182 |

v4는 실제 YOLO 파일 재읽기에서 좌표·클래스·객체 수·이미지 SHA가 일치하고,
채택 자료의 미해결 라벨·구조 오류·확인된 split 누출은 0이다.
16종 ID는 자료 유무와 관계없이 위 순서의 0–15로 고정한다. 현재 채택 자료는 9종이며,
**구조적으로 사용 가능한 부분집합이지 두 모델 비교 학습 준비 완료가 아니다.**
모든 16종이 세 split에 있는 조건을 만족하지 못하므로 PR #78은 Draft로 유지한다.
학습·추가 수집·운영 모델 변경은 수행하지 않았다.

### 실제 RGB 검수와 묶음 승인

이번에 233장의 원본 RGB/박스를 추가 확인했다. 165장 승인, 26장 문제 확인,
42장 보류다. 이는 중복·split 검사를 거치기 전 검수 판정 수이며 export 증가 수와 다르다.
aiv5 person의 고정 50장 표본 묶음을 승인했고, Enzo WoodBox·PE_Drum은
기존 수정 기록과 새 RGB 확인을 바탕으로 개별 채택했다. 명확한 원본 라벨은 teacher 검증 없이 박스를 유지하고 이름만 대응한다.
개별 수정·승인은 묶음의 실패와 별도로 처리하되 구조·중복·분할 검사는 그대로 적용한다.
표본은 SHA·scope·bbox 정책에 결합한다. 이전의 효과 없는 명칭 override는 확인 이력을
보존하면서 검증 표본으로 인정하며, 실제 종류/좌표 변경·박스 추가 표본으로 묶음을 승격하지 않는다.

- Enzo PE 방벽: 고정 50장 중 대부분 정상이다. 27번에서는 오른쪽 가장자리의 흰 PE
  방벽 주석이 빠져 있어 이 이미지만 보류했다. PE 원본 397주석(중복 전) 중 v4에는
  116주석이 채택됐으나 모두 기존 heldout이다. 방벽 자료가 전혀 없다는 뜻이 아니다.
- Enzo cargo box: 미완료 표본 9장에는 dog/deer 혼합과 왼쪽 노란 steel barricade의
  누락 주석이 함께 있어 보류했다. 기존 정상 CargoBox/WoodBox 채택 자료는 유지했다.
- aiv5 person: 표본 50장은 원본 의미·박스가 맞아 묶음을 승인했다. 같은 소스 car에는
  bus/truck이 섞여 있어 car 묶음은 승인하지 않았고 명확한 승용차 이미지만 개별 채택했다.
  이것은 작성자의 넓은 vehicle 의미와 우리 세부 종류의 차이다.
- jjj 8=CargoBox는 직접 대응 가능하지만 같은 이미지의 9에는 WoodBox·콘크리트·철제·
  PE 방벽이 함께 있다. 첫 32장 확인에서 이를 기록했다. 생성된 50장 전부를 완료로
  표시하지 않으며, 8번 박스만 골라 나머지를 누락 정답으로 만들지 않는다.

검수 UI는 원본 라벨/category ID/박스·세부 종류·학습 클래스·소비 그룹을 함께 보여준다.
수정 전 박스는 자홍 점선이며 승인/보류/문제 확인을 구분한다. JSON 다운로드 후 새
검수 버전으로 합친다. 변경 라벨·추가 박스는 사람이 확인하고, 미승인 X teacher 결과는
참고일 뿐 정답이나 삭제 근거가 아니다. 기존 원본·v3·v4 첫 실행(r1)과 검수 파일은 보존했다.

### Capture bbox와 작은 정답

`--bbox-only`는 polygon/RLE·semantic 연결 진단을 보조 segmentation 진단으로 분리한다.
RGB·박스 기하·누락 BBox 파일·세부 종류·완전성 검사는 남긴다. Capture bbox가 정확하다는
사용자 확인과 `Obstacle/Vehicle` 세부 종류 확인은 별개다. instance 연결이 없는 경우에도
사람이 RGB/원본 bbox를 독립 확인하고 `bbox_geometry_verified`·`human_confirmed`를
기록하면 연결 진단만 분리할 수 있다. 이 절차로 BBox 파일 누락이나 잘못된 좌표를 면제하지 않는다.

학습 export에는 작은 양성 정답을 제거하지 않았다. v4의 973객체에는 16/8px 미만
203개, 16/8 이상·24/12 미만 129개가 포함된다(긴 변/짧은 변, 원본 해상도 기준).
차량 24/12px·기타 16/8px 갤러리 표시와 공통 평가 범위는 학습 필터가 아니다.
1×1px 양성 보존·고정 ID·기존 v3 호환을 CPU 시험과 실제 파일 재검증으로 확인했다.

### 남은 준비 병목과 작업 순서

- box 110객체, plastic_barrier 116객체를 확보했지만 train은 0이다. Enzo heldout을
  옮기지 말고 jjj 9, Park box 등 기존 train 혼합 자료를 개별 분류·검수하는 것이 우선이다.
- motorcycle/steel_barricade/concrete_barrier/traffic_drum도 현재 heldout 위주다.
  명확한 숫자·Fence·PE 라벨 소스에서 부족한 split의 표본 검수를 먼저 이어간다.
- dog/deer/bicycle/bus/bollard/traffic_cone/stroller는 현재 채택 자료가 0이다.
  원본 확보량 0을 뜻하지 않는다. animal/6, 탑승자 복합, 명확한 bus/콘/볼라드
  소스의 진행표를 이용해 수정·승인 범위를 늘린다.
- person은 train327객체, car는 train49·heldout2, truck은 selection1이다.
  기존 7종 보존용으로도 독립 평가 자료가 더 필요하다.
- 날씨·터널·거리·뷰는 독립 근거가 없으면 unknown이다. 클래스·하위 유형·split별
  수치와 원본 픽셀 크기는 `review-progress.json`에 남긴다. 실제 GT 정확도나 충분한
  학습량을 단순 커버리지로 보장하지 않는다.

기존 평가 그룹과 Capture 12/3/3 run은 고정한다. 중복/증강/연속 장면은 같은 그룹에
두며, 새로운 독립 그룹에만 seed20261009의 80/10/10을 적용한다. 기존 자료 검수 후에도
부족한 PE_Firewall·상자·터널·FOGGY·가림·거리 조건만 추가 수집 목록으로 좁힌다.
그다음 같은 고정 bbox 데이터로 YOLO26l detect와 YOLOE detection의 짧은 비교 학습을 진행한다.

### 재현 명령

상위 workspace 데이터가 `$ASMC_DATA`이고 작업 저장소에서 실행하는 경우다.
이미 존재하는 출력 디렉터리는 덮어쓰지 않고 새 버전 경로를 사용한다.

```bash
python tools/perception/camera_data_audit.py curate \
  --inventory "$ASMC_DATA/roboflow/versioned-inventory-v1/inventory.jsonl" \
    "$ASMC_DATA/roboflow/snapshot-inventory-v2/inventory.jsonl" \
    "$ASMC_DATA/roboflow/capture-inventory-v1/inventory.jsonl" \
  --labels config/datasets/camera_object_labels_v2.json \
  --reviews "$ASMC_DATA/roboflow/visual-review-v4/reviews.json" \
  --bbox-only --fixed-class-order \
  --split-manifest "$ASMC_DATA/roboflow/curated-v3/curated.jsonl" \
  --output "$ASMC_DATA/roboflow/curated-v4" \
  --gallery /path/to/artifacts/camera-training-data-audit-v4/review \
  --export "$ASMC_DATA/dataset_versions/camera_objects_bbox_v4"
python tools/perception/camera_data_audit.py verify-export \
  --dataset "$ASMC_DATA/dataset_versions/camera_objects_bbox_v4"
```

## 작업 경계

`feat/camera-training-data-audit` / `asmc-camera-data-audit`에서 별도로 작업한다.
YOLOE 실행 검증은 PR #77 / `asmc-camera-objects`에 남긴다. 이번에는 학습·추가 수집을 하지 않는다.
이 자료는 객체 검출용이며 [TwinLite Capture 계약](../../contracts/perception/dataset-labeling-split.md)의
기존 학습 분할을 변경하지 않는다. 객체 검출 GT-v2의 18개 run은 12 train / 3 selection /
3 heldout, 30,090 view-frame 분할을 유지한다.

## 다운로드와 원본 보존

[22개 소스 목록](../../../config/datasets/roboflow_sources_v1.json)의 최신 완료 공개 버전을
처음 조회할 때 pin한다. 중단 후 다시 실행해도 번호를 바꾸지 않는다. 원본 ZIP·COCO 주석·
polygon/RLE·클래스 ID·기존 split·전처리·증강·라이선스·SHA를 `$ASMC_DATA/roboflow`에 보존한다.
이미지·ZIP·대형 manifest·인증 정보는 Git에 넣지 않는다.

공식 `roboflow` SDK 1.7.0과 CPython 3.12를 전용 venv에서 사용한다.
[공식 SDK](https://github.com/roboflow/roboflow-python)와
[다운로드 안내](https://docs.roboflow.com/datasets/create-and-upload/download-a-dataset)를 기준으로
프로젝트·버전·원본 주석을 조회한다.
[고정 requirements](../../../tools/perception/requirements-camera-data-audit.txt)를 기록했으며 ROS 환경은 수정하지 않는다.
API 키는 로컬 로그인 또는 `ROBOFLOW_API_KEY`에서 읽는다. 서명 URL·키·SDK 예외 본문은 기록하지 않는다.
공개 프로젝트에도 생성된 버전이 없을 수 있으며, 이를 삭제나 접근 거부로 해석하지 않는다.
이 경우 공식 search export의 준비 상태를 먼저 확인한다. 준비되지 않으면 공식 이미지 조회 API로
공개 Dataset에 편입된 원본과 native 주석을 받아 **버전 없는 원본 스냅샷**을 별도 보존한다.
미편입 업로드는 학습 자료로 포함하지 않는다. 검색 API의 250개 제한과 페이지 경계 중복을
검사하고, 공개 Dataset 이미지 수와 고유 ID 수가 일치하며 재조회에서도 같은 구성인지 확인한다.
새 버전·라벨·학습 작업을 만들거나 공유 설정을 변경하지 않는다. 스냅샷을 버전 1로 표시하지 않는다.

```bash
python tools/perception/download_roboflow_sources.py \
  --sources config/datasets/roboflow_sources_v1.json --root "$ASMC_DATA/roboflow"
python tools/perception/snapshot_roboflow_projects.py --root "$ASMC_DATA/roboflow" --method native
```

일반 다운로드는 동시 2개·최대 3회 시도다. 스냅샷 도구는 이 작업이 끝난 후 실행한다.
`download-report.json`은 번호 있는 버전 결과, `snapshot-report.json`은 추가 원본 확보 결과다.
22개 원본 확보와 22개 공개 버전 확보를 별도로 판정한다.

## 라벨과 완전성

[라벨 정책](../../../config/datasets/camera_object_labels_v1.json)은 원본 문자열과 정규 이름을 분리한다.
학습 후보는 person / dog / deer / bicycle / car / motorcycle / bus / truck /
electric_scooter / bollard / cargo_box / wooden_box / steel_barricade /
traffic_cone / traffic_barrel / stroller다. 네 소비 그룹은 평가용 metadata이며 운영 출력을 바꾸지 않는다.
실제로 채택된 정답이 있는 클래스만 학습 export에 포함한다.

숫자 클래스는 아래 2026-10-10 사용자 제공 대응표가 있는 **해당 프로젝트의 원본 이름**에만
적용한다. Obstacle, Vehicle, animal, scooter, bike, bicycle man의 혼합·복합 의미는
여전히 개별 확인한다. 다른 데이터셋의 숫자 ID를 COCO ID로 해석하지 않는다. 사슴의 dog 검출을 허용한 것은
사슴의 정답도 dog로 바꾸라는 승인이 아니다. 사람+자전거 박스를 둘로 복제하지 않는다.
신호등·표지판·구조물·노면 요소는 원본을 보존하고 학습 대상에서 제외하지만,
같은 영상에 있는 대상 객체의 누락 주석을 무시하지 않는다.
번호 없는 native 스냅샷의 COCO category ID는 로컬 변환용 ID다. 원래 문자열·native JSON·
image ID·파일 이름을 함께 보존하며 이를 COCO 80종 ID로 해석하지 않는다.

파일 디코딩·크기·SHA·bbox/polygon·참조 ID를 전수 검사한다. 누락된 주석 파일은 negative가 아니다.
Capture는 instance의 가시 영역 bbox와 semantic 종류 확인을 재사용하며 기존 palette 미확인 run,
미확정 subtype, 미대응 actor pixel 격리를 유지한다. RGB가 GT 생성 당시 SHA와 다르면 격리한다.
압축 RLE를 디코딩하지 못했다면 mask 검증 완료로 표시하지 않는다.

**기존 Capture `annotation_complete`만으로 16종 학습에 자동 채택하지 않는다.** 기존 GT의
검증 범위 밖에 있던 가까운 배럴·볼라드가 보이는데 주석되지 않은 반례를 확인했다.
자동 채택에는 이번 16종 전체를 포함하는 `verified_target_classes` 근거가 필요하며,
현재 범위 근거가 없는 프레임은 개별 영상 검수를 통과한 경우에만 사용한다. 과거의
자동 채택 수치는 새 범위 검증 전 중간 결과이며 최종 학습 자료가 아니다.

개별 영상에서 `pe drum`이 도로에 놓인 금속 배럴로 확인되면 `traffic_barrel`로 대응했다.
모든 drum 문자열을 자동 변경하지 않는다. `dog`로 잘못 주석된 사슴은 영상의 뿔·체형을
확인해 `deer`로 수정했다. 낮은 팔레트형 CargoBox와 X형 보강재가 있는 WoodBox를 구분한다.
콘크리트·플라스틱 방벽이 이번 16종에서 제외됐다고 충돌할 수 없는 공간이 되는 것은 아니다.
미지원 물체는 원본에 남기고 후속 학습 범위로 따로 검토한다.

## 검수·중복·분할

외부 자료는 최초에는 검수 대기다. 소스×클래스별 seed 20261009로 최대 50장을 고정한다.
표본 전부에서 대상의 완전 주석과 가시 영역 bbox를 확인한 명확한 라벨 묶음만 승격한다.
실패 표본이나 라벨 수정 표본이 있는 묶음은 일괄 승인하지 않는다. 혼합 라벨은 개별 검수·수정하거나 격리한다.
개별 검수는 이미지 SHA에 연결하며 다른 이미지에 전용하지 않는다.

동일 파일·동일 픽셀의 원본 기록은 모두 남기고 파생 데이터에서만 대표를 선택한다.
중복 주석이 충돌하면 박스 수로 우열을 결정하지 않는다. pHash는 유사 후보이며 삭제 근거가 아니다.
split 사이 유사 후보는 검수 전 export하지 않는다. 탐색이 불완전하면 누출 0으로 판정하지 않는다.
stretch 전처리로 종횡비가 달라진 경우도 pHash 후보 검사에 포함한다. 종횡비 차이만으로
후보를 버리지 않으며 `aspect_ratio_changed`로 별도 표시한다.
외부 자료는 장면 계보가 불명확하므로 프로젝트 전체를 한 그룹으로 80/10/10에 결정적으로 배정한다.
동일 이미지로 연결된 프로젝트를 먼저 합친 뒤 seed 20261009 순서와 최대 나머지 배분으로
그룹 수를 나눈다. 그룹 크기 때문에 이미지 비율은 정확히 일치하지 않을 수 있다.
Capture 평가 이미지와 겹치는 외부 프로젝트는 평가 쪽으로 제한하고 train에서 제외한다.
Capture 고정 split끼리 충돌하면 재배정하지 않고 격리한다. 이미 본 heldout은 새로운 비공개 test가 아니다.

```bash
python tools/perception/camera_data_audit.py inventory \
  --root "$ASMC_DATA/roboflow" \
  --labels config/datasets/camera_object_labels_v1.json \
  --capture-manifest /path/to/gt-v2/frames.jsonl \
  --capture-datasets "$ASMC_DATA/datasets" \
  --output "$ASMC_DATA/roboflow/inventory-v1"
python tools/perception/camera_data_audit.py curate \
  --inventory "$ASMC_DATA/roboflow/inventory-v1/inventory.jsonl" \
  --labels config/datasets/camera_object_labels_v1.json \
  --output "$ASMC_DATA/roboflow/curated-v1" \
  --gallery /path/to/artifacts/camera-training-data-audit-v1/review \
  --export "$ASMC_DATA/dataset_versions/camera_objects_bbox_v1"
python tools/perception/camera_data_audit.py verify-export \
  --dataset "$ASMC_DATA/dataset_versions/camera_objects_bbox_v1"
```

출력은 덮어쓰지 않는다. [통합 목차](http://localhost:8768/)의 데이터 감사 페이지에서
클래스·프로젝트를 검색하고 정규 라벨·가시 영역 좌표·검수 근거를 입력할 수 있다.
브라우저에서 검수 JSON을 저장하고 `curate --reviews <file>`로 새 버전을 만든다.
클릭만으로 학습 파일을 수정하지 않는다. 혼합 라벨을 추측해서 소스 전체를 승인하는 기능은 없다.
수정된 bbox의 원래 값과 검수자·근거는 `original_xyxy` / `review_record`에 남긴다.
Capture와 외부 inventory를 따로 생성한 경우 `--inventory`에 두 파일을 함께 전달할 수 있다.
CPU 작업은 2개씩 실행하며 GPU·MORAI·bridge·ROS master는 조작하지 않는다.

검수한 원본은 다시 생성하지 않는다. 번호 없는 스냅샷의 완료 archive SHA를 재실행 시
확인하고 재사용한다. native image ID를 inventory ID에 사용해 검색 순서 변화와 분리한다.
검수 거부는 오류 확정과 확인 부족을 근거 문장으로 구분하며 둘 다 학습에서는 격리한다.
날씨·거리·터널별 정확한 수량은 메타데이터 근거가 없으면 unknown으로 남긴다.

## 완료 판정과 다음 작업

baseline에서 소스별 원본 확보, 채택·격리 수, 실제 split과 클래스 커버리지를 확인한다.
날씨·터널·뷰·거리를 파일명만으로 확정하지 않고 근거가 없으면 unknown으로 표시한다.
원본 확보와 학습 준비 완료는 별도다. CargoBox/WoodBox가 없는 데이터셋은 추가 장애물 학습 준비 완료가 아니다.
미해결 라벨·split 충돌·검수 전 후보를 학습에 넣지 않는다.

파생 YOLO 자료는 원본 RGB에 대한 bbox 정답이다. 차체는 학습 대상이 아니며 raw RGB를
몰래 마스킹하지 않는다. 후속 학습에서 검증된 뷰별 Ego mask를 적용할지 결정하고,
학습·추론 전처리를 일치시켜야 한다. 진짜 polygon·Capture instance mask는 manifest의
원본 참조로 보존하며 bbox export가 segmentation 학습 자료라는 뜻은 아니다.

다음은 고정 데이터의 YOLO26l detect / YOLOE 후보 짧은 bbox 비교 학습이다.
기존 7종과 CargoBox/WoodBox의 검출·오검출·시간적 공백·통합 지연으로 본학습 모델 하나를 선정한다.
이번 작업에서 학습·운영 모델 교체를 수행했다고 설명하지 않는다.

## 실행 위치와 증거 갱신

데이터 도구는 호스트의 `asmc-camera-data-audit`에서 전용 venv로 실행한다.
현재 데이터 루트는 상위 workspace의 `data/roboflow`다. ROS 컨테이너에는 SDK를 설치하지 않는다.
`asmc-camera-objects`의 PR #77 작업과 마운트는 보존한다.

```bash
source ../data/roboflow/.venv/bin/activate
export ASMC_DATA="$(cd ../data && pwd)"
python tools/perception/report_camera_data_audit.py \
  --root "$ASMC_DATA/roboflow" \
  --curated "$ASMC_DATA/roboflow/curated-v1" \
  --dataset "$ASMC_DATA/dataset_versions/camera_objects_bbox_v1" \
  --gallery ../artifacts/camera-training-data-audit-v1 \
  --baseline config/baselines/perception_camera_training_data_audit_v1.yaml
python tools/perception/build_experiment_gallery_hub.py --artifacts ../artifacts
```

baseline은 소스별 원본/채택/격리 수, 주석 종류, SHA, 클래스 대응, 채택 클래스의 split별 객체 수와
검수 근거를 묶는다. `classes_in_all_splits`에 없는 클래스는 공정한 비교 학습 평가를 준비했다고
표시하지 않는다. `ATTRIBUTION.md`에 외부 출처와 라이선스를 함께 남긴다.
`reviewed_subset_ready`는 채택된 작은 부분집합의 구조·분할 검사이며,
`bbox_pilot_ready`는 기존 7종과 CargoBox/WoodBox의 train/selection/heldout 확보 여부다.
두 판정을 섞지 않는다. 후자가 false이면 필요한 클래스·분할의 개별 검수를 먼저 늘린다.

## 2026-10-10 결과와 아직 남은 감사

[고정 baseline](../../../config/baselines/perception_camera_training_data_audit_v1.yaml)에 소스별 표와 SHA를 기록했다.
22개 원본 확보는 완료했다. 17개는 최신 공개 버전, 5개는 발행 버전이 없는 native 스냅샷이다.
공개 버전 22개가 존재한다고 표시하지 않는다. 보존 archive 22개의 SHA를 재검증했고 총 2,911,061,824 bytes다.
외부 이미지 46,326개(증강·중복 포함)와 Capture 30,090 view-frame, 총 76,416개를 구조 검사했다.

실제 개별 검수는 286장이다. 생성한 소스×클래스 표본은 132묶음·고유 4,799장이며,
그중 검수한 표본은 236장이다. 표본 생성과 검수 완료를 혼동하지 않는다.
확인 부족·미검수·누락·혼합 주석·구조 오류·중복 충돌·cross-split 유사 후보는 학습에서 격리했다.
**전체 수동 라벨 감사는 미완료이며 검증된 부분집합만 제공한다.**

최종 경로는 `$ASMC_DATA/roboflow/curated-v2`와
`$ASMC_DATA/dataset_versions/camera_objects_bbox_v2`다. v1 및 그 이전 중간 결과는 보존하지만
`SUPERSEDED.txt`로 후속 학습 사용을 막았다. 재실행은 새 출력 버전으로 만들고 기존 디렉터리를 덮어쓰지 않는다.

| 항목 | 최종 결과 |
|---|---|
| 채택 이미지 / 객체 | 119 / 213 |
| train / selection / heldout 이미지 | 16 / 1 / 102 |
| 미확정 라벨 / 채택 자료 구조 오류 / 확인된 pixel split 누출 | 0 / 0 / 0 |
| 채택 정답 | person 12, dog 18, deer 10, car 3, bus 2, truck 1, cargo_box 62, wooden_box 52, traffic_barrel 53 |
| 전체 격리 | 76,297 이미지 기록 |
| pHash 유사 후보 | 251,062쌍; 확정 중복 수가 아니며 원본 보존 |
| 학습·운영 모델 변경 | 없음 |

종횡비를 바꾼 증강본 검사에서 2장을 추가 격리했다. 동일 픽셀 복제본의 0.5px 이하 반올림 차이는
원본 RGB 대조와 개별 수정 기록으로 통일했으며, 단순히 박스가 더 많은 주석을 선택하지 않았다.
Capture의 기존 `annotation_complete`가 가까운 배럴·볼라드까지 포함하지 않는 반례도 보존했다.
번호 없는 스냅샷의 조회·다운로드는 여러 도구 수정 단계에 걸쳐 수행했다. 원본 archive·native JSON SHA를
수집 증거로 삼고, baseline의 코드 commit/SHA는 최종 감사·export 구현을 고정한다.

**비교 학습 준비는 보류다.** 검수된 자료가 heldout에 편중됐고 모든 split에 정답이 있는 클래스는
현재 CargoBox뿐이다. WoodBox는 train 2개·heldout 50개이고 selection 정답이 없다.
기존 7종의 일부도 train 또는 평가 정답이 없다. seed·Capture split을 바꾸거나 heldout을 train으로
이동시켜 수량을 맞추지 않는다. 없는 클래스의 정답을 빈 파일로 만들지도 않는다.

다음 검수는 미확정 자료를 전부 사용하기보다 train/selection에 배정된 소스의
CargoBox·WoodBox와 기존 7종을 우선 늘린다. 혼합 `object/obstacle`, 숫자 이름,
사람+자전거 복합 박스와 주석 누락은 개별 수정하거나 계속 격리한다.
새 검수 JSON으로 v3을 만들고 `bbox_pilot_ready`를 다시 검사한 뒤 두 모델의 짧은 bbox 학습으로 넘어간다.

## 2026-10-10 후속: 원본 클래스 의미 확인과 활용 우선순위

> 이 절은 v1 대응표 당시의 이력이다. 아래의 cargo_box/wooden_box/traffic_barrel과
> 방벽 `requires_class_decision`은 현재 학습 명칭이 아니다. 현재 v2에서는 box/traffic_drum과
> concrete_barrier/plastic_barrier로 연결하며, 상단 계약과 최신 v3 결과를 적용한다.

**위 v2의 76,297개 격리는 데이터 품질 불합격 수가 아니다.** 이 중 76,291개에
`visual_review_required`가 있었다. 숫자 이름의 의미 미확인, 미검수, 주석 범위 미확인도
같은 격리 집계에 들어갔다. 사유는 서로 중복되므로 합산하지 않는다. 이름을 맞추기 어렵다는
이유로 MORAI 자료 자체를 버리지 않는다. 119장은 이전의 엄격한 문턱을 통과한 작은 부분집합이며,
사용 가능한 전체 자료량으로 해석해서는 안 된다.

사용자가 22개 자료가 MORAI임과 클래스 의미를 직접 확인했다.
[소스별 대응표](../../../config/datasets/camera_object_source_labels_v1.json)에 출처를 사용자 제공 정보로
기록했다. 앞서 RGB로 발견한 반례는 notes에 함께 남긴다. 대응 순서는 소스 규칙 → 전역 규칙이다.
숫자는 COCO category ID가 아닌 **원본 라벨 문자열**로 해석하며, native 변환 과정에서 새로 부여한
category ID를 의미 ID로 사용하지 않는다. 원본 라벨·ID·박스·polygon은 수정하지 않는다.

동일한 외부 inventory 46,326개에 대한
[의미 재분류 집계](../../../config/baselines/perception_camera_source_label_resolution_v1.json)는 다음과 같다.
이는 중복·증강을 포함한 기록 수이고, 영상 완전 주석·split 검수 통과 수가 아니다.

| 항목 | 기록 수 |
|---|---:|
| 외부 이미지 | 46,326 |
| 라벨 의미가 명확해진 대상 주석 | 140,077 |
| 이 중 기존 ambiguous에서 대상 이름으로 새로 대응된 주석 | 101,314 |
| 신호·표지 등 제외 주석 | 11,254 |
| 혼합·복합·학습 범위 결정 등 추가 판단이 필요한 주석 | 57,835 |
| 기록된 라벨에 ambiguous가 없는 이미지 | 28,284 |

마지막 행에는 제외 클래스만 있는 이미지도 포함될 수 있다. 실제 대상 누락 여부를 확인하기 전
negative 또는 학습 승인으로 사용하지 않는다. 기존 v2 export와 baseline 판정은 덮어쓰지 않았다.

### 소스별 적용과 남은 작업

Katri 공통 숫자 규칙은 아래 여섯 프로젝트에만 적용한다:
`cone-z2vga/katri-q4vkc`, `katri-2-txrqf`, `katri_kids-2-x5xnl`, `katri_kids-zblud`,
`merge-axyff`, `ssibal-sjav5`.
0→car, 1→bus, 2/3→person, 5→traffic_barrel(PE_Drum), 7→bollard다.
4는 사람+자전거 복합 박스, 6은 dog/deer 혼합으로 남긴다.

| 소스 | 활용과 남은 확인 |
|---|---|
| aiv5challenge/yeonsoo-sarang | car/person 유지. 현재 환경과 유사한 기본 클래스 회귀 자료로 우선 활용 |
| battery-detection/cone-traffic-tirxp | cone→traffic_cone. 신호등 제외, 실제 polygon 보존 |
| bicycle-man/morai-bicycle | bicycle man은 탑승자 복합 박스. person/bicycle 각 박스를 그리거나 별도 rider 클래스 합의 필요 |
| 위 Katri 여섯 소스 | 명확한 숫자 라벨을 먼저 통일. animal·탑승자 박스만 분리 검수. 서로 복제된 자료와 증강 계보 확인 |
| jjj-nx9di/katri_new_data-xkbev | 공통 0–7에 8→cargo_box 추가. 9는 concrete/steel barricade·wooden_box·PE_Firewall을 박스별 재분류 |
| dh-mzmhr/animal-tohen | 기존 animal 박스를 dog/deer로 구분. 좌표를 다시 그릴 필요가 없는 박스는 종류만 수정 |
| enzo-mdikr/my-first-project-eeree | cargo box/wood box/PE_Drum/bollard/이륜차 등을 우선 활용, speed 제외. 기존 RGB 검수에서 dog로 주석된 deer가 있어 해당 묶음은 보정 필요 |
| hl-fma/hlfma_2 | person 등 명확한 이름 활용. 실제 metadata의 object에는 WoodBox·PE_Drum·steel barricade 등이 섞여 있어 종류 재분류 |
| hl-fma/hlfma_3 | car/truck/bus/person 활용, others 2개 원본 주석은 확인 필요 |
| mino-qjf5p/vi2re | car/person 및 검수한 obstacle 박스 활용, 신호·표지 제외. obstacle 전체를 한 종류로 바꾸지 않음 |
| parkminseo/final-ftuy2 | human→person, car/bus 유지, red/green/yellow 제외. box는 WoodBox/PE_Firewall 분리 |
| s-workspace-p9mvp/morai_obstacle | person 회귀·replay용. 풍부한 사람 자료가 있으므로 대량 반복보다 장면 다양성 우선 |
| selfcar/morai_test-t3i2r | **0→traffic_barrel, 1/2→bollard**. bollard01/04 정보는 원본 이름으로 보존. Katri 숫자 규칙과 혼용 금지 |
| seonhyung-kim/morai-perception | pedestrian(사용자 표기 pedstrain)은 사람 포함 복합 유형, vehicle·animal·obstacle도 세부 종류 검수. traffic_light 제외 |
| tooth-decay-rnacu/morai_ai_mission-fhqkh | **Fence→steel_barricade**, PE→traffic_barrel, sign/TL 제외. Concrete_1/2는 concrete barricade 후보. Obstacle_Yellow는 혼합 종류 검수 |
| yolo-m5lvd/detection-bzetd | Bus/Car/Truck/Person 활용. bike 외형과 Obstacle 혼합 유형 확인. Traffic/StreetLight 제외 |
| yuna-z15io/obstcle-mi0kq | static/dynamic은 물체 종류가 아니므로 기존 박스에 종류 재라벨링 필요. 초기 파일럿 우선순위는 낮음 |

Concrete barricade와 PE_Firewall은 실제 대회 물리 장애물 후보다. 앞서 zero-shot prompt에서
구조물 오검출 때문에 뺐다는 이유로 학습에서도 영구 제외하지 않는다. 현재 16종 정책에는 없으므로
`requires_class_decision` 또는 혼합 후보로 보존한다. asset 외형·박스 기준을 검토하고
`concrete_barricade`·`pe_firewall` 같은 세부 클래스를 추가할지 다음 학습 ontology에서 확정한다.
이 상태를 검출할 필요 없는 배경으로 자동 변환하지 않는다.

### 기존 데이터 활용과 필요한 수집

1. **기존 자료를 먼저 활용한다.** Enzo·jjj의 CargoBox/WoodBox, selfcar·Katri의 PE_Drum/bollard,
   battery의 cone을 우선 검수한다. aiv5·hlfma_3·Katri의 기본 사람·차량은 회귀와 균형 유지용으로
   함께 사용한다. project 전체를 버리는 대신 명확한 라벨이 있는 이미지부터 완성한다.
2. **혼합 클래스는 박스 좌표를 재사용해 종류를 수정한다.** dog/deer, class 9, box,
   object/obstacle 순으로 teacher 초안과 RGB 검수를 결합한다. 사람이 자전거를 탄 복합 박스를
   COCO bicycle 박스로 바로 치환하거나 같은 박스를 두 클래스로 복제하지 않는다.
3. **누락된 대상은 해당 이미지에서 보완한다.** cone 전용 이미지에서도 학습 대상 person/car가
   보이는데 라벨이 없으면 일반 YOLO의 완전 지도학습에 그대로 넣지 않는다. 신호 주석 삭제는
   가능하지만 다른 대상의 누락을 해결하지는 않는다. 16종이 모두 매 이미지에 존재해야 한다는
   조건은 없다. 영상에 실제로 보이는 선택 대상이 빠지지 않았는지를 검사한다.
4. **추가 수집은 부족 조건에 한정한다.** 터널의 낮은 밝기·FOGGY, 원거리·부분 가림,
   현재 좌우 뷰와 Ego mask 경계, CargoBox/WoodBox·PE_Firewall 등 부족 종류를 우선한다.
   기존 자료로 충분한 밝은 장면 car/person/cone를 다시 대량 수집하지 않는다.
   semantic/instance GT를 함께 확보하되 semantic 종류만으로 객체별 instance mask가 생기지는 않는다.

사용자 확인으로 라벨 의미가 풀려도 영상 완전성·구조·중복·분할 검사는 유지한다. 다만 위 우선
묶음의 표본 검수를 끝내 명확한 라벨 묶음을 승격하는 방식으로 진행하며, 전체 76,416장을
하나씩 수작업으로 끝낼 때까지 기다리는 방식은 피한다. 실제 오류가 발견된 묶음은 개별 보완한다.
고정 Capture split과 이미 평가한 heldout은 그대로 둔다. 희소 클래스의 분할 부족은 같은 장면을
train/평가로 복제해 채우지 않고 다른 소스·장면의 정답을 확보한다.

### YOLO와 YOLOE 학습 연결

자료 의미 통일은 두 모델에 공통이다. 원본 실제 종류 → 정규 학습 클래스 →
person/animal/vehicle/obstacle 소비 그룹을 분리하고 정적·동적 여부를 클래스와 섞지 않는다.
**같은 bbox train/selection/heldout으로 YOLO26l detect와 YOLOE detection 파일럿을 비교**한다.
YOLOE도 오픈 vocabulary라는 이유로 dog/deer 혼합이나 누락된 박스를 자동 정답으로 만들지는 못한다.
YOLOE teacher는 새로운 장애물의 이름 초안을 만드는 보조로 사용하며, 틀린 prompt 출력은 검수한다.

Ultralytics의 [공식 YOLOE 학습 안내](https://docs.ultralytics.com/models/yoloe/#train-usage)는
동일 scale의 detection YAML에 seg checkpoint 가중치를 가져오고 `YOLOEPETrainer`를 사용하는
bbox 학습 경로를 제공한다. 단순히 seg 예측에서 masks를 버리는 것과 구분한다.
실제 파일럿 전 고정 설치 버전에서 변환·학습·export를 작은 입력으로 확인한다.
`YOLOEPESegTrainer`로 seg를 학습할 때에는 실제 instance 주석이 있는 자료를 사용하며,
bbox 사각형을 가짜 mask 정답으로 만들지 않는다. PE fine-tuning의 class-name 고정 방식과
학습 후 prompt 변경 가능성도 export 전에 검증한다.

현 배포 모델 YOLO26l detect는 유지한다. 부족한 CargoBox/WoodBox 등의 개선,
기존 7종 회귀, 터널 오검출·공백과 4060 Ti 통합 지연을 보고 본학습 모델 하나를 선택한다.
두 모델용 데이터를 처음부터 따로 수집하지 않는다.

소스 의미만 다시 집계하려면 아래 명령을 사용한다. 이 명령은 학습 승인·export를 수행하지 않는다.
새 `curate` 실행은 저장 inventory의 원본 이름을 소스 규칙으로 다시 해석한 뒤 기존 검수 기록을
적용한다. 기존 export와 원본은 그대로 두고 v3 등 새 경로를 지정한다.

```bash
python tools/perception/camera_data_audit.py source-label-summary \
  --inventory "$ASMC_DATA/roboflow/versioned-inventory-v1/inventory.jsonl" \
    "$ASMC_DATA/roboflow/snapshot-inventory-v2/inventory.jsonl" \
  --labels config/datasets/camera_object_labels_v1.json \
  --output "$ASMC_DATA/roboflow/source-label-resolution-v1.json"
```

후속 검증: CPU 시험 44개, 문서 migration·로컬 링크·perception contract/topic policy 통과.
숫자 의미의 소스 간 혼용, Fence 예외, 혼합 박스 자동 분할, 원본 좌표 변경과
이름 통일만으로 학습 승인되는 경우를 검사한다. 실제 v3 검수·학습은 아직 수행하지 않았다.

```bash
python tools/perception/camera_data_audit.py verify-export \
  --dataset "$ASMC_DATA/dataset_versions/camera_objects_bbox_v2"
```

검증: CPU 시험 37개, 문서 migration, perception contract/topic policy 검사 통과.
실제 생성 YOLO 파일을 다시 읽어 이미지 SHA·클래스 ID·좌표·객체 수·파일 구성·split을 확인했다.
검수 HTML의 JavaScript 구문과 로컬 HTTP 경로도 확인한다.
GitHub CI는 `recent account payments have failed or your spending limit needs to be increased`로
job 시작 전 차단됐다. 코드 시험 실패나 기존 facade 실패와 섞지 않는다.
PR #77의 HEAD와 runtime worktree·컨테이너 마운트는 변경하지 않았다.

## v3·X teacher 실행 결과 — 2026-10-10

[고정 수치·SHA](../../../config/baselines/evidence/camera_yoloe_x_teacher_v1/summary.json),
[전체 소스 대응 집계](../../../config/baselines/evidence/camera_yoloe_x_teacher_v1/source-label-resolution-v2.json),
[검증 기록과 코드 snapshot](../../../config/baselines/evidence/camera_yoloe_x_teacher_v1/validation.json),
[v3 baseline](../../../config/baselines/perception_camera_training_data_v3.yaml)을 함께 읽는다.

- 원본 Roboflow 46,326장·209,166개 주석에서 의미가 풀린 후보 142,103개,
  제외 11,254개, 혼합·미확정 55,809개다. v1 inventory보다 103,340개가 추가로 해석됐다.
  이는 증강·중복을 포함한 이름 대응 수이며 학습 승인 수가 아니다.
- Enzo 100개 독립 RGB 검토와 기존 Capture 일부의 새 범위 재검수로 **v3 95장·250객체**를
  생성했다. box 96(CargoBox 46/WoodBox 50), concrete_barrier 98, traffic_drum 50,
  person 3, car 2, truck 1이다. train 3 / selection 1 / heldout 91이다.
  기존 보수적 장면 그룹과 고정 Capture split을 옮겨 분포를 맞추지 않았다.
- export의 클래스 ID·원본 SHA·실제 YOLO 좌표·객체 수·파일 구성 재검증을 통과했다.
  채택 자료 미해결 라벨·구조 오류·확인된 정확 픽셀 split 누출은 각각 0이다.
  **부분집합 생성은 통과했으나 16종 파일럿 학습은 준비되지 않았다.** 클래스와
  train/selection/heldout별 정답이 부족하다. 과거 v2 119장을 새 범위로 무조건 승격하지 않았다.
- 첫 v3 이관에서 v1의 콘크리트 방벽 제외 override 98개가 남아 새 대상이 누락됐다.
  재검증에서 발견해 사용자 제공 소스 의미와 RGB 검토로 바로잡았다.
  [실패 근거](../../../config/baselines/evidence/camera_yoloe_x_teacher_v1/migration-failure-r1.json)와
  `curated-v3-rejected-r1` / `camera_objects_bbox_v3-rejected-r1`을 보존했다. 실패 버전은 학습에 쓰지 않는다.
- X checkpoint SHA는 `d08d390a08f98195f7c87807839fe4ff93a5491645fef1bc3bf0700efafdd639`다.
  공식 checkpoint·encoder에서 5개 prompt embedding을 각각 생성하고 같은 317프레임을 실제 추론했다.
  PyTorch FP16 batch1·전방1280/측방640·3뷰 mask, network-none, MORAI 종료 상태다.
  TensorRT export·OOM·silent fallback은 없었다.
- 기존 Capture selection/heldout 5,667프레임 중 20개 간격과 독립 actor 참고 영상으로
  고정한 **317프레임의 초기 진단**이다. 완전한 16종 주석을 확인한 정식 GT가 이 표본에는
  없어서 precision·전체 데이터 recall·confidence 선정·teacher 승인은 하지 않았다.
  33개 actor 종류 참고는 Codex의 독립 RGB 검토이며 사람의 최종 확인과 구분한다.
  과거부터 공개·평가된 heldout을 새 blind test로 부르지 않는다.
- 부분 GT의 confidence 0.40·IoU ≥0.5 대응에서 base18/rich21 모두 box 2/52,
  cone 87/114다. box는 0.25에서 4/52, 0.10에서도 11/52다. 이 수는 확인한 주석만의
  위치 대응 진단이며 GT 없는 배경의 precision을 만들지 않는다.
  상자 주석은 저장 영상에서 small(<32² pixels) 36개, medium 6개, large 10개다.
  작은 객체가 많은 이 표본을 전체 장애물·근거리 상자 성능으로 일반화하지 않는다.
  `doe`·`wooden crate`·`traffic drum` 추가는 이 표본의 통합 클래스 대응을 늘리지 못했다.
  plastic barrier는 base18/rich21 1/86, bollard21 2/86, plastic21/combined21 0/86이다.
  이 차이를 근거로 최종 vocabulary를 확정하지 않았다.
- 승인 teacher 클래스와 teacher가 적용한 Roboflow 라벨은 **0**이다.
  X teacher로 모든 혼합 라벨을 자동 분리할 수 있다는 가정은 지지되지 않았다.
  정답·지원 수를 갖춘 클래스만 다시 정식 selection → 고정 heldout 검증으로 연결한다.
  기존 원본 박스가 명확하면 이름 통일과 영상 검수를 먼저 활용하는 편이 낫다.

브라우저 시작점은 `http://localhost:8768/`다. 날짜·실험 번호가 있는
`camera-training-data-audit-v3/`와 `camera-yoloe-x-teacher-v1/`을 목차에서 선택한다.
teacher 비교 갤러리는 원본 GT·예측 bbox를 표시하고 대응 instance mask는 NPZ에 보존했다.
`camera-yoloe-x-teacher-v1/capture-gt-review/`는 teacher 예측 없이 독립 Capture 종류 후보와
원본 박스만 보여 준다. `actor_review.human_confirmation_pending`이 있는 후보는 미확정이다.
새 검수 페이지는 세부 의미·통합 학습 클래스·원본 category ID를 함께 보여 준다.
사람 확인 체크와 새 박스 JSON 입력을 제공하며 다운로드한 기록만 다음 export에 반영한다.

새 버전 생성 명령은 다음과 같다. 출력 경로가 이미 있으면 덮어쓰지 않는다.

```bash
python tools/perception/camera_data_audit.py curate \
  --inventory "$ASMC_DATA/roboflow/versioned-inventory-v1/inventory.jsonl" \
    "$ASMC_DATA/roboflow/snapshot-inventory-v2/inventory.jsonl" \
    "$ASMC_DATA/roboflow/capture-inventory-v1/inventory.jsonl" \
  --labels config/datasets/camera_object_labels_v2.json \
  --reviews "$ASMC_DATA/roboflow/visual-review-v3-corrected/reviews.json" \
  --output "$ASMC_DATA/roboflow/curated-v3" \
  --export "$ASMC_DATA/dataset_versions/camera_objects_bbox_v3"
python tools/perception/camera_data_audit.py verify-export \
  --dataset "$ASMC_DATA/dataset_versions/camera_objects_bbox_v3"
```

[GT 준비기](../../../tools/perception/prepare_capture_teacher_gt.py)의 `--actor-reviews`는
독립 RGB 근거·SHA·원본 종류·actor·frame 범위를, `--image-reviews`는 v2 범위의 영상 완전성
확인 기록을 받는다. `camera_teacher_audit.py infer`는 config/profile/checkpoint/embedding을
검사한다. 필요하면 `--image-prefix <호스트 데이터 경로> --image-root /data`로 읽기 경로만 변환한다.
`suggest`는 검증 report와 checkpoint·prompt·config를 맞추고 기존 혼합 박스에 한해 제안한다.

격리 GPU 환경의 재현 순서는 아래와 같다. checkpoint와 encoder는 실행 전에 준비하고
고정 SHA를 확인한다. `/audit`는 이 데이터 감사 worktree의 read-only mount,
`/prepared`는 X 전용 모델 디렉터리, `/results`는 새 실험 결과 디렉터리다.
공유 ROS 컨테이너에는 설치하거나 덮어쓰지 않는다.

```bash
export PYTHONPATH=/opt/seg-deps:/audit/src/asmc_camera/src:$PYTHONPATH
python3 /audit/tools/perception/prepare_yoloe_l.py \
  --directory /prepared --scale x --stage prompts --profile rich21 \
  --config /audit/config/experiments/camera_yoloe_x_teacher_v1.json
python3 /audit/tools/perception/camera_teacher_audit.py infer \
  --gt /data/roboflow/capture-teacher-gt-v1/diagnostic-sample.jsonl \
  --weights /prepared/yoloe-26x-seg.pt --embeddings /prepared/rich21.npz \
  --profile rich21 --front-width 1280 \
  --hood-config /audit/src/asmc_camera/config/camera/yolo_hood_mask_v1.yaml \
  --labels /audit/config/datasets/camera_object_labels_v2.json \
  --config /audit/config/experiments/camera_yoloe_x_teacher_v1.json \
  --image-prefix "$ASMC_DATA" --image-root /data \
  --output /results/rich21-repeat-1280
```

여기서 `--image-prefix`는 manifest에 기록된 **호스트의 절대 데이터 경로**다.
다섯 profile은 각각 새 embedding과 출력 경로로 실행한다. 이미 저장한 결과의 재집계에는
`camera_teacher_audit.py analyze`와 `report_camera_teacher_audit.py`를 사용한다.
전자는 checkpoint·embedding·prompt·config가 섞인 결과를 거부하고 승인 보고서에 SHA를 연결한다.
후자는 `--root`, `--gt`, `--labels`, `--config`, `--environment`, 선택적인 `--evidence`로
원본 예측에서 비교 표·갤러리·요약을 재생성하며 추론하지 않는다.
`camera_teacher_audit.py review --gt ... --labels ... --config ... --output <새 경로>`는
독립 GT 검토 화면을 재생성한다. 브라우저는 서버 파일을 쓰지 않고 확인 기록을 다운로드한다.

검증은 데이터 CPU 50개·teacher CPU 10개·기존 측정/목차 6개·카메라 검출 회귀 75개가
통과했다. 메시지가 없는 격리 환경의 첫 import 실패는 `asmc_msgs`·`asmc_camera`를
private `/tmp` workspace에서 빌드해 해결했다. 문서 migration·계약·topic policy,
v3 export 재검증·검수 HTML JavaScript 구문·갤러리 링크와 HTTP도 별도로 확인한다.
Hosted CI의 계정 결제/사용 한도 차단은 로컬 시험 결과와 구분한다.

남은 과제는 train/selection의 명확한 소스 검수 확대, Capture의 우선 객체 종류·완전성 사람 확인,
animal/6 → 9 → box/object/obstacle 순의 개별 재분류다. source 라벨 의미가 확인됐다는 이유로
새 박스·teacher 예측까지 사람이 검수했다고 기록하지 않는다. 필요한 터널·원거리·가림 자료는
이 커버리지를 본 뒤 수집하며, 이번에는 학습이나 수집을 시작하지 않았다.

## Capture 극소형 GT 평가 범위 — 2026-10-10

사용자는 Capture GT가 정확하지만 먼 객체의 박스가 점처럼 표시된다는 것을 확인했다.
원본 GT를 삭제하지 않고 평가·표시에서 극소형 객체를 `ignore`로 분리한다.
[크기 정책](../../../config/experiments/camera_capture_size_scope_v1.json)은
원본 영상 픽셀 기준 긴 변 ≥16px **그리고** 짧은 변 ≥8px를 주 평가 대상으로 둔다.
정확히 경계값이면 포함한다. 원거리 m cutoff는 추정하지 않는다. tensor 960/1280으로
후보를 바꿔도 동일 원본 영상의 대상 집합이 바뀌지 않으며 실시간 검출을 크기로 삭제하지 않는다.

이 값은 잠정 분석 범위이며 모델의 검출 가능 한계·운영 수락 기준이 아니다.
8/4px, 24/12px 대조와 전체 GT 결과를 같이 보존한다. 이미 노출된 selection/heldout을
사용한 사후 진단이므로 새로운 blind 평가나 이 결과에 맞춘 threshold 선정으로 설명하지 않는다.

- 기존 317프레임·같은 저장 예측을 CPU로 재집계했다. 추가 GPU 추론은 하지 않았다.
- 원본의 전체 종류 GT 738개 중 크기 충족 306개·극소형 432개다. 이 기하 집계에는
  종류 미확정·제외 종류도 포함된다. 아래 정량 표는 알려진 학습 대상의 부분 주석만 사용한다.
- confidence 0.40·IoU ≥0.5에서 base18/rich21의 결과는 다음과 같다.

| 학습 클래스 | 전체 GT 대응/수 | 주 평가 GT 대응/수 | 극소형 ignore |
|---|---|---|---|
| person | 46/93 | 33/34 | 59 |
| car | 20/195 | 20/37 | 158 |
| box | 2/52 | 2/29 | 23 |
| traffic_cone | 87/114 | 81/87 | 27 |

사람·차량의 기존 전체 비율은 극소형 GT의 영향을 크게 받았다. 상자는 크기 범위를
분리해도 대응 수가 낮아 추가학습·오분류 검토 대상으로 남는다. 분모가 바뀐 비율 상승을
모델 자체의 개선으로 주장하지 않는다. 세부 GT가 미확정인 항목은 전체 precision/recall과
teacher 승인에 사용하지 않는 동작을 유지한다.

주 평가 GT를 먼저 일대일 매칭하고, 미대응 예측이 ignore GT 안에 충분히 들어간 경우에만
FP 집계에서 제외한다. 기준은 `intersection / prediction_area ≥0.5`다. 작은 GT를 덮는
거대한 잘못된 박스나 배경의 작은 FP는 면제하지 않으며, 주 평가 대상의 중복 검출도 FP로 남는다.
이는 이번 도구의 ignore 규칙이며 [COCO 평가 구현](https://github.com/cocodataset/cocoapi/blob/master/PythonAPI/pycocotools/cocoeval.py)의
면적별 집계와 동일한 지표라고 표시하지 않는다.

학습용 export는 바꾸지 않았다. 일반 [YOLO TXT 형식](https://docs.ultralytics.com/datasets/detect/)
의 객체별 class/xywh 라벨을 지우는 것만으로 학습 loss가 그 영역을 ignore하지 않는다.
극소형 대상이 보이는 영상을 라벨 없는 negative로 바꾸지 않는다. 학습 시에는 원본 양성
라벨을 유지하는 대조부터 시작하고, 필요한 경우 loss ignore를 별도로 구현·검증한다.

[집계 증거](../../../config/baselines/evidence/camera_yoloe_x_teacher_v1/size-scope-v1.json)와
`http://localhost:8768/camera-capture-size-scope-v1/`에서 비교한다. 회색 ignore 박스는
기본 숨김이며 체크로 표시할 수 있다. 원본 이미지를 따로 열 수 있고 이전 갤러리는 보존한다.

```bash
python tools/perception/report_capture_size_scope.py \
  --gt "$ASMC_DATA/roboflow/capture-teacher-gt-v1/diagnostic-sample.jsonl" \
  --inference-root "$ASMC_ARTIFACTS/camera-yoloe-x-teacher-v1" \
  --config config/experiments/camera_yoloe_x_teacher_v1.json \
  --labels config/datasets/camera_object_labels_v2.json \
  --scope-config config/experiments/camera_capture_size_scope_v1.json \
  --output "$ASMC_ARTIFACTS/camera-capture-size-scope-v1"
```

출력 경로는 새 경로여야 한다. 픽셀 경계, 불량 GT 기하의 숨김 방지, 작은 배경 FP,
큰 오검출 면제 방지, 주 평가 매칭 우선과 중복 검출을 CPU 시험으로 검사한다.
후속 검증은 teacher CPU 16개·기존 데이터 CPU 50개, 문서 migration·계약, JavaScript 구문과
갤러리 HTTP 확인을 통과했다. 범위 옵션 없는 재집계는 저장된 rich21 전체 GT 결과의 11개
분석 필드와 정확히 같았다. 기존 GPU 추론·학습 export를 다시 실행하지 않았다.
`camera_teacher_audit.py analyze --scope-config <크기 정책> --scope-profile main16x8`도
같은 범위로 집계할 수 있다. 결과에는 범위·정책 SHA를 기록한다. 극소형 GT는 teacher
승인의 최소 표본 수에 포함하지 않으며, 해당 범위로 승인된 teacher를 원본의 더 작은
혼합 박스에 적용하지 않는다. 옵션이 없으면 이전 전체 GT 집계와 동일하다.
작은 미확정 종류는 ignore 영역으로만 사용한다. 주 평가 범위 안의 미확정 종류·입력 문제나
영상의 범위 확인은 계속 검증해야 하며, 극소형 제외를 이유로 해당 검증을 면제하지 않는다.

### 사용자 화면 검토 후 24/12px 대조 — 2026-10-10

사용자가 16/8px 적용 사진의 작은 박스를 보고 기준 상향을 요청했다.
[v2 정책](../../../config/experiments/camera_capture_size_scope_v2.json)은 원본 긴 변 ≥24px
**그리고** 짧은 변 ≥12px를 잠정 주 범위로 둔다. 기존 v1 정책·갤러리·전체 GT는 유지한다.
원거리 자체를 제거하는 정책은 아니다. 예시 front/000180의 truck GT 17×9px·21×12px는
ignore지만 car 34×13px와 bus 41×20px는 남는다. 이미 노출된 결과를 이용한 시각 검토용
범위 변경이며, 더 좋은 점수를 얻었다는 이유로 모델 개선·teacher 승인으로 설명하지 않는다.
작은 예측을 전역 삭제하지 않으며 실시간 검출·학습 export에도 이 제한을 적용하지 않는다.

새 갤러리는 `http://localhost:8768/camera-capture-size-scope-v2/`다. 생성 명령은 위 명령에서
`--scope-config config/experiments/camera_capture_size_scope_v2.json`과 새 출력 경로를 사용한다.
[v2 집계](../../../config/baselines/evidence/camera_yoloe_x_teacher_v1/size-scope-v2.json)는
16/8과 24/12 결과를 함께 보존한다. 제외 박스 표시는 사용자 요청에 따라 밝은 자홍색
2px 윤곽으로 변경했다. `제외 GT도 표시 (자홍색)`을 켜면 볼 수 있다.

후속 사용자 요청에 따라 같은 v2 갤러리의 **표시만** 클래스별로 구분한다.
[표시 정책](../../../config/experiments/camera_capture_display_scope_v1.json)은
car/bus/truck에 24/12px, 사람·다른 작은 객체·종류 미확정 GT에 16/8px를 적용한다.
종류 미확정 Vehicle을 임의로 car로 간주하지 않는다. 파란 박스는 표시 대상이며,
자홍색 박스는 표시에서 숨긴 GT다. 표시 대상과 평가 대상이 다를 수 있으므로 각 프레임에
표시/숨김 수와 공통 평가 대상/ignore 수를 따로 안내한다. `숨긴 GT도 표시 (자홍색)`으로
확인한다. 기존 표·평가 manifest는 공통 24/12px 그대로이며 학습 export도 바꾸지 않는다.
재생성 시 `--display-config config/experiments/camera_capture_display_scope_v1.json`을 추가한다.
출력의 `display-policy.json`·`display-scope.jsonl`은 표시만의 정책·annotation ID를 기록한다.

원본 GT 보존은 해당 영상을 학습에 채택할 때 정확한 작은 양성 박스도 함께 유지한다는
뜻이다. 모든 Capture 프레임을 무조건 학습에 사용한다는 뜻은 아니며, 학습에 넣을
장면·중복·클래스/크기별 표본 분포는 별도로 선정한다. 표시 제한을 학습 라벨 삭제로
전달하지 않는다.

### PE_Firewall 자료를 먼저 활용하는 순서

PE_Firewall의 학습 클래스는 `plastic_barrier`다. 현재 zero-shot의 낮은 대응은 원본
Roboflow 박스의 오류를 뜻하지 않는다. 고정 X 진단은 부분 GT에서 rich21 기준 전체
1/86, 16/8px 1/54, 24/12px 1/44 대응이다. 주석이 없는 상황을 포함하는 전체 recall이나
현재 운영 YOLO의 정량 수치로 사용하지 않는다.

- Enzo의 `pe barrigate` 397개 주석은 사용자 제공 의미로 직접 대응 가능하다.
- jjj의 `9` 1,803개와 Park의 `box` 391개는 각각 여러 객체 종류가 합쳐진 원본 주석이다.
  전부 PE_Firewall로 바꾸지 않고 방벽 사례를 개별 분류한다. 숫자는 해당 라벨의 전체
  주석 수로, PE_Firewall 수나 중복 제거 후 독립 표본 수가 아니다.
- 다른 `obstacle/object` 포괄 주석도 필요하면 확인하되 검증되지 않은 X 예측만으로
  plastic_barrier 정답을 자동 생성하거나 미검출 주석을 삭제하지 않는다.
- 위 자료의 PE 방벽을 장면·중복 계보로 정리하고, SUNNY/FOGGY·터널·거리·가림·세 뷰
  커버리지를 확인한다. 모르는 조건은 unknown이다. 부족한 조건만 추가 수집하며 학습과
  평가 장면을 나눈다. 현재 작업에서는 추가 수집이나 학습을 시작하지 않았다.

출처와 계수는 [방벽 자료 확인](../../../config/baselines/evidence/camera_yoloe_x_teacher_v1/plastic-barrier-coverage-v1.json)에 남긴다.

## Capture actor 연결 보강 — 2026-10-11 v5-capture-r1

**Capture만 보강했다. 샘플링 변경·추가 수집·GPU 추론·학습·운영 변경은 하지 않았다.**
기존 v5-obstacles-r1의 17,631장에 Capture train **4,867장·13,315객체**를 추가했다.
Capture는 5→**4,872장**, 전체는 **22,498장·127,334객체**다. Roboflow 17,626장은
변경하지 않았다. train은 16,851→**21,718장**, selection 113장·heldout 667장은 그대로다.
이전 채택 자료의 annotation·품질 구분과 Capture 12/3/3 run 배정을 유지한다.

| 학습 클래스 | 이번 Capture 추가 객체 | 전체 객체 |
|---|---:|---:|
| box | 802 | 1,470 |
| traffic_drum | 363 | 18,520 |
| traffic_cone | 584 | 1,372 |
| concrete_barrier | 36 | 2,196 |
| bicycle | 249 | 249 |
| dog | 106 | 136 |
| person | 2,312 | 24,566 |
| car | 6,719 | 18,676 |
| bus | 945 | 4,214 |
| truck | 1,199 | 3,229 |

`plastic_barrier` 585개·`steel_barricade` 505개·`deer` 24개는 이번에 늘지 않았다.
`stroller`는 여전히 없다. 있는 자료를 다른 종류로 바꾸어 부족 클래스를 채우지 않았다.
15종의 양성 자료가 있다는 것과 독립적인 16종 평가 준비 완료를 구분한다.

### 연결 근거와 한계

- train의 미확정 actor를 run·원본 category·actor ID·연속 frame 구간으로 나눴다.
  40프레임 초과 재등장은 별도 구간이며, run/category가 다른 같은 숫자 ID는 연결하지 않는다.
- 167구간 중 RGB 종류 검토가 가능한 72구간의 가까운/앞/뒤 영상을 확인해 **70구간**을
  연결했다. 실제 채택 영상에는 67구간·2,494개 종류 확정 annotation이 사용됐다.
  나머지 추가 객체는 이미 종류가 명확한 원본 GT다. 95개의 극소형/불명확 구간과
  두 미확정 종류는 teacher나 인접 run의 ID로 추정하지 않았다.
- `Obstacle` semantic의 공통 색만으로 상자·드럼·동물을 나누지 않았다. RGB에서 확인한
  종류를 원본 actor에 연결하고 GT bbox·mask·annotation ID를 보존했다.
- 자전거 탑승 장면은 **별도 person GT와 Vehicle bicycle GT/mask**가 있는 것을 확인했다.
  사람과 자전거를 합친 큰 박스를 두 번 복제한 자료가 아니다. 자전거의 원본 범위를 유지한다.
- foggy 5pm run의 Obstacle 15는 초기 frame 74의 RGB가 bicycle 모습이므로 drum을
  84 이후로만 연결했다. Obstacle 57 dog는 265–303만 연결하고 다른 장면이 보이는
  376까지 전파하지 않았다. 원인은 확정하지 않았으며 추가 확인 사진을 보존했다.
- 새 자료는 전방 2,096장·좌 1,251장·우 1,520장, 9개 train run에서 나왔다.
  run 이름 기준 SUNNY 4,078장·FOGGY 789장이다. 프레임별 터널 여부는 unknown이다.
  인접 프레임의 정보는 상관되므로 이를 4,867개의 독립 장면으로 해석하지 않는다.
- RGB 검토용 최소 면적 250px²는 **검토할 근거 영상 선택 기준**이며 학습 필터가 아니다.
  종류가 연결된 영상의 작은 양성 GT는 export에 남는다. 표시 정책도 학습 필터가 아니다.
- 모든 포함 영상의 16종 누락 주석을 전수 확인했다는 뜻은 아니다. 사용자 허용 v5 breadth
  정책을 유지하며 `complete_target_annotation_asserted=false`로 기록한다. 알려진 미확정
  actor가 남는 영상·빈/누락 GT·기존 기하 문제·평가 유사 영상은 새 train에 넣지 않았다.

[고정 설정](../../../config/experiments/camera_training_data_v5_capture.json),
[결과와 split별 수량](../../../config/baselines/perception_camera_training_data_v5_capture.yaml),
[actor별 원본 SHA·구간 근거](../../../config/baselines/evidence/camera_training_data_v5_capture/actor-source-evidence.json),
[실제 YOLO 파일 재검증](../../../config/baselines/evidence/camera_training_data_v5_capture/verification.json)을 연결한다.
전체 22,498장·127,334객체의 클래스 ID·기하·원본 파일 SHA를 재검증했고 pixel split 누출은 0이다.
이것은 라벨 의미 정확도나 모델 성능 개선 수치가 아니다.

갤러리: `http://localhost:8768/camera-training-data-audit-v5-capture/`.
actor RGB 15개 시트와 원본→학습 라벨 110개 비교를 기존 날짜·E 번호 목차에 연결했다.
이미지·mask·전체 manifest는 로컬 데이터 영역에 있고 Git에는 도구·설정·압축된 근거만 남긴다.

### 재현

상위 workspace에서 `ASMC_DATA`·`ASMC_ARTIFACTS`를 지정하고 감사 worktree에서 실행한다.
출력 경로가 이미 있으면 새 revision 경로를 사용한다. 원본·이전 export를 덮어쓰지 않는다.

```bash
python tools/perception/prepare_capture_training_data.py render \
  --previous "$ASMC_DATA/roboflow/curated-v5-obstacles-r1/curated.jsonl" \
  --output "$ASMC_ARTIFACTS/camera-training-data-audit-v5-capture/actor-review"
python tools/perception/prepare_capture_training_data.py bind \
  --previous "$ASMC_DATA/roboflow/curated-v5-obstacles-r1/curated.jsonl" \
  --intervals "$ASMC_ARTIFACTS/camera-training-data-audit-v5-capture/actor-review/intervals.json" \
  --decisions config/baselines/evidence/camera_training_data_v5_capture/actor-decisions.json \
  --output "$ASMC_ARTIFACTS/camera-training-data-audit-v5-capture/actor-reviews.json"
python tools/perception/prepare_capture_training_data.py export \
  --previous "$ASMC_DATA/roboflow/curated-v5-obstacles-r1/curated.jsonl" \
  --labels config/datasets/camera_object_labels_v2.json \
  --reviews "$ASMC_ARTIFACTS/camera-training-data-audit-v5-capture/actor-reviews.json" \
  --output "$ASMC_DATA/roboflow/curated-v5-capture-r1" \
  --export "$ASMC_DATA/dataset_versions/camera_objects_bbox_v5-capture-r1"
```

다음 작업에서 run·소스·클래스 분포와 고정 평가 커버리지를 확인해 학습 샘플링을 정한다.
그 뒤 동일 bbox 자료로 YOLO26l detect/YOLOE detection의 짧은 비교 학습을 한다.
이번 Capture 보강을 그 작업들의 완료나 운영 모델 교체로 설명하지 않는다.


## bbox 파일럿 샘플링 — 2026-10-11

원본 v5-capture-r1 22,498장과 split은 유지했다. 별도 파일럿은 train 8,000장과
selection/heldout 각각 300장이다. 원본의 작은 양성 GT와 이미지에 속한 모든 대상 박스를 보존한다.
단일 소스/run 최대 2,000장, Capture 최대 3,200장이고 seed는 20261011이다.
희소 클래스의 등장 이미지와 box/방벽류를 먼저 고르고, 나머지는 이미지 등장 빈도의
역제곱근 가중치로 선택한다. 증강 family와 Capture 40-frame 구간은 상관관계 집계이며 독립 장면 GT가 아니다.

[설정](../../../config/experiments/camera_bbox_pilot_v1.json),
[원본·파일럿 SHA와 분포](../../../config/baselines/evidence/camera_bbox_pilot_v1/sampling.json),
[YOLO 파일 재검증](../../../config/baselines/evidence/camera_bbox_pilot_v1/verification.json),
[샘플링 도구](../../../tools/perception/prepare_camera_bbox_pilot.py)를 연결한다.

평가본은 원래 evaluation partition 안에서만 보강했다. cross-split 유사 영상·기하 문제·
누락 원본 GT는 제외하고, 세부 이름이 확정되지 않은 박스는 원본에 남긴다.
16종 누락 검수 근거가 없는 영상은 recall-only다. 미대응 검출을 FP로 계산하지 않는다.
selection의 신규 장애물 GT 부족과 heldout의 개별 종류 소표본은 남아 있으며,
파일럿 export 성공을 전체 16종 모델 선정의 충분한 평가 근거로 설명하지 않는다.

```bash
python tools/perception/prepare_camera_bbox_pilot.py \
  --config config/experiments/camera_bbox_pilot_v1.json \
  --data-root "$ASMC_DATA" \
  --output "$ASMC_DATA/dataset_versions/camera_objects_bbox_pilot_v1-r4"
```

학습은 이 결과를 포함한 별도 브랜치에서 직렬 실행한다. 운영 모델과 ROS 설정은 변경하지 않는다.


## bbox 비교학습 — 2026-10-11 PR #80

최신 main 835d56e3와 #78의 샘플링을 포함한 `feat/camera-bbox-pilot-training`에서 작업한다.
폴더는 기존 `asmc-camera-data-audit`를 유지하며 실시간 `asmc-camera-objects` 마운트를 변경하지 않는다.
학습은 `asmc-camera-bbox-pilot` network-none 컨테이너, 고정 Ultralytics 8.4.150 / Torch 2.1.2+cu118에서 직렬 실행한다.
공유 ROS Python은 수정하지 않는다. MORAI/실시간 GPU 모델과 학습을 겹치지 않는다.

두 모델은 960·batch4·AMP에서 200 batch씩 사전 측정을 통과했다.
YOLO26l epoch 예상 528.20초, YOLOE bbox 477.03초에 20% 여유를 넣어 같은 17epoch를 배정했다.
유효 batch16, AdamW LR .001 / decay .0005, 동일 증강·seed·이미지 순서를 사용한다.
YOLOE는 detection YAML + seg checkpoint + PE trainer이며 mask head는 학습하지 않는다.
학습 클래스별 영어 표현 하나를 고정해 fused head에 반영한다. 이 결과는 임의의 새 prompt를
추가할 수 있는 원래 open-vocabulary checkpoint와 구분한다. freeze=None으로 전체 파라미터를 학습한다.

실제 GPU/배경 부하로 사전 예상보다 느려져도 모델당 3시간 예산을 늘리지 않는다.
예산 중단이면 원래 실패를 보존하고, 두 후보 모두 완료한 동일 epoch의 checkpoint만
추가 평가할 수 있다. 더 진행한 후보의 이후 업데이트는 사용하지 않는다. 비교 전 각 checkpoint의
epoch·optimizer update를 CPU로 검사하고, [CPU sampler 재현기](../../../tools/perception/verify_camera_bbox_order.py)의
hash가 실제 학습 전체/부분 순서와 일치해야 공통 prefix 순서를 인정한다. 이는 파일 순서 검증이며
증강 RNG의 비트 단위 일치를 주장하지 않는다. 최소 5epoch 미달·OOM·실패 checkpoint는 이 경로로 수락하지 않는다.
원래 `comparison.json`은 남기고 별도 `comparison-equal-epoch.json`에 비교 학습량과 이유를 기록한다.

[예산/클래스 설정](../../../config/experiments/camera_bbox_pilot_v1.json),
[사전 측정과 실패 기록](../../../config/baselines/evidence/camera_bbox_pilot_training_v1/smoke.json),
[직렬 실행기](../../../tools/perception/run_camera_bbox_comparison.py),
[학습·재개](../../../tools/perception/train_camera_bbox_pilot.py),
[평가기](../../../tools/perception/evaluate_camera_bbox_pilot.py)를 연결한다.

진행 갤러리: `http://localhost:8768/camera-bbox-pilot-v1/`. 30초마다 로컬 상태를 읽는다.
`artifacts/camera-bbox-pilot-v1/paired-r1/comparison.json`과 모델별 `run.json`이 실제 진행 기록이다.
위 절은 시작 시 배정과 검증 정책이다. 실제 종료·동일 학습량 비교와 후보 판정은 아래 최신 결과에 기록한다.

### 결과와 재개 해석

- native validation을 끄고 외부 평가로 통일했다. `results.csv`의 precision/recall/mAP 0은
  미측정 placeholder이며 실제 검출률 0이 아니다. 정확도는 외부 `evaluation.json`에서 읽는다.
- checkpoint의 `updates`는 EMA 갱신/optimizer step 시도 카운터다. 실제 AdamW state의 `step`과
  구분한다. 첫 YOLO26l의 17epoch는 EMA 8,500회·실제 AdamW 8,491회였으며 AMP에서 9회가 생략됐다.
  생략을 숨기거나 실제 update와 동일하다고 설명하지 않는다. 동일 epoch/입력 순서로 비교하면서
  각 모델의 실제 step·scaler를 함께 보고한다. 재개 시험은 optimizer state와 scaler 복구도 확인한다.
- main 학습은 사전 측정 가중치를 버리고 다시 시작한다. 200 batch checkpoint는 partial epoch라
  재개에 쓰지 않는다. 원본 epoch별 checkpoint의 optimizer·EMA·누적 업데이트를 보존한다.
- 학습 전 YOLO26 평가에는 원래 COCO80 사전학습 checkpoint를 사용한다. 16종으로 교체한
  고정 16종의 `initial.pt`는 학습 초기화 진단으로 별도 보존한다. 아래 회귀 진단에서 기존
  7종의 weight/bias가 이름별로 정확히 전이됐음을 확인했다. 이전의 “무작위 head” 설명은
  정정한다. 운영 기준선은 여전히 원본 COCO80 checkpoint이며 평가기가 원본 SHA를 확인한다.
  YOLOE의 학습 전 결과는 같은 detection 구조와 16종 text embedding을 준비한 뒤 fine-tuning
  직전에 저장한 checkpoint다. `inference.json`에 요청/실제 weight SHA와 기준선 종류를 모두 남긴다.
  두 후보의 기존 객체 회귀 gate는 현재 기준 모델인 YOLO26l/1280/.40과 비교한다. YOLOE가
  자기 사전학습 결과보다 개선됐다는 이유만으로 기존 운영 인식보다 낮은 recall을 면제하지 않는다.
  .40 고정 대조를 그대로 보고하며, 후보 판정은 selection에서 고정한 confidence의 신규 F1과
  기존 객체 recall을 사용한다. .40에서의 calibration 차이를 실제 선택 confidence의 회귀와 혼동하지 않는다.
- 재개 시 모델·데이터/config/초기 가중치 SHA·batch·전체 epoch를 검사한다. 파일 손실·중단은
  마지막 완전 epoch에서 재개하며, 중단 epoch의 동일한 증강 RNG까지 비트 단위 복구한다고 주장하지 않는다.
  기본 loader는 재생성되므로 원래 파일 순서의 연속 복구도 주장하지 않는다. 비교 후보는 fresh 실행이며,
  여러 재개 구간을 합친 순서 근거 없이 fresh 후보와 입력 순서가 같았다고 승인하지 않는다.
- 알려진 평가 한계: selection 신규 객체 box 6개, heldout animal은 deer 1개, stroller 0개다.
  공통 GT 평가와 허용된 car/bus/truck·dog/deer 상위 의미 평가를 분리한다.
- 완전 주석 검수 근거가 있는 영상만 precision/F1에 쓴다. 나머지는 annotated recall이다.
  같은 소스/run의 많은 영상은 독립 장면 증거가 아니며 별도로 표시한다.
- sparse 평가본으로 실제 최초 안정 검출·공백 ms를 만들지 않는다. 이후 연속 영상/통합 시험에서 확인한다.
- PT batch1 평가 지연은 TensorRT 3뷰/통합 지연과 다르다. 학습 중 VRAM도 배포 VRAM이 아니다.
- 고정 heldout은 이번 비교에 노출된 자료다. 본학습 후 새로운 독립 최종 시험으로 재포장하지 않는다.

본학습 후보는 신규 장애물 개선·기존 객체 회귀·오검출을 검토한 뒤 정한다.
1%p 내 동률이면 TensorRT 배포 지연/VRAM 확인 전 확정하지 않는다.
이후 부족 조건만 보강 → 한 모델 본학습 → TensorRT·3뷰 배포 → 전방 fusion/4그룹 소비 계약 →
MORAI 전체 스택 반복·30분 연속 → 팀 실행/복구 문서 순서다.
3뷰/mask fusion과 TwinLite BEV 자세 보정은 별도 후속 확장이다.


### 연속 입력 참고 평가

정확도 selection/heldout 300/300장은 그대로다. 별도 [연속 입력 고정 목록 요약](../../../config/baselines/evidence/camera_bbox_pilot_training_v1/temporal-probe.json)은
기존 evaluation run에서만 고른 120장이다. 알려진 actor가 평가 크기 범위에서 이어지고
frame index가 실제로 연속인 구간만 쓴다. 원본 박스·분할을 수정하지 않으며 confidence 선정에 사용하지 않는다.
[준비·분석기](../../../tools/perception/camera_bbox_temporal.py)는 5개 연속 저장 입력 중 3개 IoU .5 대응을
만족한 최초 window 끝 frame과 최대 누락 입력 개수를 보고한다. 이 구간 밖 최초 등장이나
실제 ms 지연·live 센서 연속성을 입증하지 않는다. 별도 GT가 없는 신규 장애물의 연속성도 평가 완료로 만들지 않는다.

[Capture RGB 범위 점검](http://localhost:8768/camera-bbox-pilot-v1/native-gt-review/)에서는
actor GT의 기하 정확성과 전체 시설물 주석 완전성을 구분했다. 작은 미확정 actor만 남았다는
이유로 Capture를 자동 full-GT로 승격하지 않는다. 반대로 실제 완전 주석 영상에서는
제외된 신호등/표지판 위치의 잘못된 box 검출을 ignore로 면제하지 않는다.

### 16epoch 비교 결과 — 본학습 후보 선정 보류

2026-10-11에 두 모델의 bbox 파일럿과 8개 학습 전/후·해상도 평가를 마쳤다.
[수치·SHA·클래스별 결과](../../../config/baselines/evidence/camera_bbox_pilot_training_v1/results.json),
[동일 입력 순서 증거](../../../config/baselines/evidence/camera_bbox_pilot_training_v1/sampler-proof-ab.json),
[실제 재개 증거](../../../config/baselines/evidence/camera_bbox_pilot_training_v1/resume-proof.json)를 연결한다.
사진은 [전체 실험 목차](http://localhost:8768/) → [bbox 파일럿](http://localhost:8768/camera-bbox-pilot-v1/)에서 확인한다.
각 모델의 `.40` 대조와 selection에서 정한 confidence의 갤러리를 별도로 제공한다.

**두 후보 모두 기존 객체 recall 회귀 기준을 넘어서 본학습 후보 확정을 보류한다.**
운영 기본 YOLO26l detect/1280/.40과 ROS 인터페이스는 변경하지 않았다.
YOLOE/1280이 아래 신규 네 종류에서 우세했지만 기존 객체 회귀를 면제하지 않는다.

- 총 8,000장, 960/AMP/batch4/nbs16, 같은 학습 정책과 입력 순서다.
- A는 17epoch를 9,394.3초에 완료했다. B는 10,802.6초에 예산 중단되어 16epoch와
  17번째의 일부만 처리했다. 총 주 학습은 20,196.9초(약 5시간 37분)다.
- 원래 17epoch 실행의 실패와 B의 부분 checkpoint를 보존한다. 품질 결과를 보기 전에 정한
  예산 중단 정책에 따라 **양쪽 epoch15.pt, 즉 완전한 16epoch**만 비교했다.
  A의 17번째와 B의 부분 17번째 업데이트는 비교에서 제외했다. 원래 동일 17epoch LR 일정은 유지했다.
- 두 실제 전체/부분 입력 hash가 CPU 재현과 일치한다. 공통 16epoch prefix도 같다.
  이는 파일 순서 증거이며 GPU 증강의 비트 단위 동일성을 뜻하지 않는다.
- 선택 checkpoint는 각 EMA 시도 8,000회, 실제 AdamW A 7,991회/B 7,990회다.
  AMP 생략 9/10회와 scaler를 공개한다. 정상 완료를 AMP 생략 0회와 혼동하지 않는다.

| 후보 / 전방 너비 | selection confidence | 신규 macro F1 | person recall | motorcycle recall | 전방 PT p50 / p95 / p99 ms |
|---|---:|---:|---:|---:|---|
| YOLO26l / 960 | .45 | 65.54% | 20/26, 76.92% | 2/138, 1.45% | 14.37 / 25.95 / 27.35 |
| YOLO26l / 1280 | .25 | 65.65% | 21/26, 80.77% | 4/138, 2.90% | 15.81 / 23.36 / 25.74 |
| YOLOE-26l bbox / 960 | .10 | 77.11% | 22/26, 84.62% | 14/138, 10.14% | 14.46 / 23.31 / 27.23 |
| YOLOE-26l bbox / 1280 | .20 | 82.61% | 21/26, 80.77% | 4/138, 2.90% | 17.34 / 25.44 / 27.91 |

표의 정확도는 고정 heldout, confidence는 selection에서만 정했다. 신규 macro는 완전 GT가
20개 이상인 **traffic_drum·concrete_barrier·box·steel_barricade 네 종류**다.
각 종류가 같은 소스의 연속/유사 장면에 편중되어 있어 대회 전체 일반화 정확도가 아니다.
선정 selection의 신규 GT 자체가 적으므로 신규 클래스 confidence 검증 완료로 표시하지 않는다.
PT 지연은 오프라인 FP16/batch1의 알려진 전방 뷰 174개 표본이다. TensorRT 2+1 batch나
MORAI·다른 모델·판제 동시 실행 지연이 아니다. 측방/unknown 뷰 지연은 증거 JSON에 별도 기록한다.

현재 YOLO26l/1280/.40 기준선의 person은 24/26(92.31%), motorcycle은 126/138(91.30%)이다.
차량 집계 GT 141개 중 138개가 motorcycle이고 car 2개·bus 1개뿐이다. 따라서 집계 하락을
승용차·버스·트럭 전체의 성능 저하로 일반화하지 않는다. animal GT도 deer 1개뿐이다.

[대표 오토바이 오분류 사진](http://localhost:8768/camera-bbox-pilot-v1/gallery/yoloe26l-after-1280/1eb600d6bcf6a934046aca250f6d788886b492b836ad1b37735ed95fee570d28.jpg)에서는
학습 전 motorcycle이 학습 후 높은 confidence의 car로 바뀐다. selection confidence의
기하만 검사한 일대일 IoU≥.5 진단에서 motorcycle→car는 A/1280 96개, B/1280 90개다.
이 진단은 위치 검출과 이름 실패를 구분하기 위한 것이며 오분류를 TP로 면제하지 않는다.
학습 motorcycle은 63장·191주석으로 단일 소스 `yolo-m5lvd/detection-bzetd`에만 있다.
그 RGB 표본에서는 실제 이륜차가 확인됐다. 소스·외형 다양성 부족, 다른 라벨의 혼입,
희소 클래스 학습·망각이 원인 후보이며, 이번 통계만으로 한 원인을 확정하지 않는다.

완전 GT 70장에서는 .40의 전체 FP가 A/1280 61개, B/1280 50개였다.
selection confidence에서는 각각 74/71개다. partial GT의 미대응 예측을 FP에 넣지 않았다.
신규 클래스 FP도 함께 있으므로 COCO만 지원하는 기준선과 전체 FP 수를 같은 지원 범위의
오검출률처럼 단순 비교하지 않는다. 원본 클래스별 수치를 증거에 남긴다.

별도 연속 Capture 120장은 29개 actor 구간/224 actor-input 쌍을 제공했다.
선정 confidence에서 안정 window가 있는 구간은 A/1280 23개, B/1280 27개였고 최대 누락은
6/4 입력이다. 저장 입력 index 기준 참고 지표이며 실제 Hz·ms·최초 시야 진입 지연이 아니다.
미지원 신규 장애물의 연속 검출을 이 숫자로 승인하지 않는다.

학습 중 전체 GPU VRAM peak는 A 7,890MiB/B 7,884MiB(8,188MiB의 약 96.4/96.3%)였다.
Windows·배경 사용을 포함한 학습 계측이며 운영의 90% 배포 여유 기준을 통과했다는 뜻이 아니다.
2개 GPU utilization 표본은 미지원으로 남겼다. OOM·silent batch 감소는 없었다.

재개는 별도 256장 train fixture에서 모델별 1epoch 후 일시정지→2epoch 완료를 실제 확인했다.
optimizer 558개 state slot·1차 moment SHA·scaler·EMA가 복구됐고,
AdamW는 A 10→26/B 7→23으로 이어졌다. YOLOE fused detection head도 유지됐다.
[검증기](../../../tools/perception/verify_camera_bbox_resume.py)는 checkpoint와 run 기록을 다시 검사한다.
main 비교는 재개 없이 실행했으며, 재개 시험을 main 입력 순서 연속성 증거로 사용하지 않는다.

로컬 관련 CPU 시험 125개, 문서·perception 계약·토픽·런타임 validator가 통과했다.
Hosted CI는 결제 제한으로 job이 시작되지 않은 기록과 구분한다. #80은 Draft이며 #78 의존성과
#77 미완료 통합 검증은 유지한다.

다음 작업은 대량 추가 수집·본학습에 앞서 **이륜차와 사람 회귀 원인 점검 및 고정 평가 자료 보강**이다.
기존 자료의 motorcycle↔car/bicycle 혼입과 복합 박스부터 확인하고, 기존 객체 유지용 자료의
소스 다양성·샘플링·학습률 변경을 통제한 작은 재파일럿을 계획한다. 신규 장애물 기준으로는
YOLOE bbox/1280을 우선 재검토할 근거가 있지만 본학습 후보로 승인하지 않았다.
원인 해결 뒤 한 모델 본학습 → TensorRT/3뷰 → 전방 fusion → 전체 스택 30분 → 팀 문서 순서다.
이번 실행에서는 추가 수집·본학습·운영 교체를 시작하지 않았다.
## 2026-10-11 PR #78 병합 범위와 main 통합

사용자 승인으로 hosted CI billing 제한 상태에서도 데이터·감사·샘플링 도구를 main에
병합한다. 미확인 자료까지 완전한 GT로 수락하거나 운영 모델을 교체하는 병합은 아니다.
채택 tier와 소스별 허용 혼입, 미완전 주석, 평가 부족과 stroller 미지원은 유지한다.
#80의 두 모델 파일럿은 별도 작업이며 이 PR에서 학습 수락을 주장하지 않는다.

#77 병합 main `c8ce3f55`를 반영했다. 공유 준비 도구는 m/l의 640/960/1280·variant
export와 X teacher의 PyTorch-only 제한을 모두 유지한다. X ONNX/engine 차단 및
전방·측방 입력 profile 경계 시험을 추가했다. 데이터·샘플링·준비 도구 CPU 시험
117개와 perception 계약/토픽/runtime validator 및 문서 migration 검사 4개가 통과했다.
#77에서 확인한 카메라 전체 306개 중 main의 기존 facade 1건 실패는 별도로 유지한다.
해당 runtime 및 launch는 이 PR에서 변경하지 않는다. hosted CI를 통과로 표시하지 않는다.

현재 v5-capture-r1의 실제 YOLO 이미지/라벨을 다시 읽어 22,498장·127,334객체,
픽셀 해시 기준 split 누출 0을 확인했다. manifest SHA는
`63cc3539294a7035d537073b77cac08b7672107b13483418dac402de2c794a3b`다.
이는 파일·좌표·ID·split 검증이며 모든 주석의 영상 의미 정확도를 보증하지 않는다.

## 2026-10-11 teacher 대안과 RF-DETR 검토

**현재 결론:** Grounding DINO를 YOLOE-X의 보완 teacher로 검증할 가치는 있지만,
두 모델의 MORAI 정확도 우열은 아직 측정하지 않았다. SAM 2는 박스/점으로 마스크를
만들고 영상에서 전파하는 모델이며 객체 이름을 결정하는 검출기가 아니다.
RF-DETR은 추가학습 가능한 별도 실시간 배포 후보다. 이 세 역할을 혼동하지 않는다.
이번에는 참고 소스와 코드·문서만 확인했으며 새 추론·자동 라벨·추가학습은 실행하지 않았다.

### 라벨 보강 이력과 원본 GT 우선

[YOLOE-X 원본 결과](../../../config/baselines/evidence/camera_yoloe_x_teacher_v1/summary.json)는
`formal_teacher_acceptance=false`, `approved_classes=[]`, `teacher_labels_applied=0`이다.
X를 시험한 사실과 X 예측을 정답으로 적용한 사실은 다르다. 현재 학습 자료는 원본 라벨의
소스별 의미 해석·RGB 검토와 Capture actor 연결로 정리했다. 이 기록은 변경하지 않았다.

- **Capture:** 정확한 bbox·instance·semantic과 actor 대응을 우선한다. coarse 종류가
  미확정인 경우 teacher는 해당 원본 박스의 이름 후보만 제안한다. 정확한 GT 박스를
  새 예측으로 대체하거나 미검출 객체를 지우지 않는다.
- **Roboflow:** 직접 대응 라벨은 이름 변환만으로 사용한다. 혼합 박스에만 teacher의
  개별 이름/분리 제안을 적용하고 원본 박스·출처·검수 이력을 보존한다.
- **bag:** RGB에서 bbox 초안 생성이 가능하다. Ego mask, 중복 장면, 작은 대상,
  미검출과 불완전 주석을 확인하며 teacher 무검출을 빈 정답으로 만들지 않는다.
- **AIM:** `/mnt/d/asmc_aim/dataset` 존재를 다시 확인했다. 기존 감사의 RGB 417,435개는
  semantic/instance/2D bbox GT 미확인 상태이며 이번에 전수 재감사한 수가 아니다.
  dynamic_obstacle 하위 lap에는 CAMERA_1~4·capture_contract가 있다. camera4 rear와
  우리 신호등 camera4를 구분한다. 파일 수와 독립 장면 수, 새 클래스 커버리지는 다르다.
  검수한 세션 표본부터 사용하고 원본·선정/평가 split과 센서 보정을 보존한다.

### Grounding DINO + SAM 2를 사용한다면

[Grounding DINO](https://github.com/IDEA-Research/GroundingDINO)는 text-conditioned bbox를,
[SAM 2](https://github.com/facebookresearch/sam2)는 prompt로 instance mask와 영상 전파를
제공한다. [공식 연결 예제](https://github.com/IDEA-Research/Grounded-SAM-2)는 detector의
box/name을 SAM 2에 공급한다. 잘못된 이름과 누락은 SAM 2만으로 해결되지 않는다.
박스 학습에서는 먼저 Grounding DINO 단독 검출과 YOLOE-X를 비교하고, 마스크·가림
검토·연속 영상 주석 전파에 실익이 있을 때 SAM 2를 추가하는 순서가 적절하다.

다음 시험은 고정 selection RGB·정확한 기존 GT와 같은 16종 대응표를 사용한다.
box의 CargoBox/WoodBox, plastic_barrier의 PE_Firewall, 철제/콘크리트 방벽, drum,
사람/이륜차·원거리·FOGGY·터널을 구분한다. 같은 이름으로 합칠 학습 클래스도 원래
하위 유형별 검출 누락을 남긴다. class/IoU 일대일 precision·recall, 중복, 누락,
검수 수정량과 처리 시간을 비교한다. GT가 없는 종류는 사례 검토로 남긴다.
클래스별 충분한 표본에서 검증한 teacher만 초안을 일괄 생성하고, 불일치·새 박스·
변경 이름은 검수한다. 두 teacher가 일치한다는 것만으로 정답 승인을 대신하지 않는다.

로컬 Grounding DINO checkpoint 예제와 GD1.5/1.6/DINO-X cloud API 예제는 구분한다.
후자 API를 무료 로컬 checkpoint로 가정하지 않는다. 8GB GPU에서는 큰 detector와
SAM 2를 순차 실행하거나 작은 묶음으로 처리하는 후보가 적절하지만 OOM·속도는 미측정이다.
영상 전파는 새로 등장하는 객체를 놓칠 수 있어 주기적 재검출·검수가 필요하다.

### RF-DETR 고정 소스와 시스템 적합성

공식 [roboflow/rf-detr](https://github.com/roboflow/rf-detr)를 상위 workspace의
`external/baselines/RF-DETR`에 shallow clone했다. 원본 develop checkout SHA는
`eca736acab9f6fe93f2cbc82a1d9f6edfb53f0ea`, pyproject는 `1.12.0.dev`다.
확인 시점 [stable 1.11.2](https://github.com/roboflow/rf-detr/releases/tag/1.11.2)와
개발 소스를 구분한다. vendor/submodule·ROS 의존성으로 추가하지 않았고 checkout은 깨끗하다.
설치·가중치 다운로드·학습·4060 Ti GPU 측정은 하지 않았다.

공식 README의 COCO detection 비교는 다음과 같다. **Roboflow 자체 측정의 T4,
TensorRT FP16, batch1, 서로 다른 기본 해상도**이며 우리 전체 스택 wall latency가 아니다.

| 후보 | 기본 tensor | COCO AP50:95 | GPU latency ms | 배포 파라미터 M |
|---|---|---:|---:|---:|
| RF-DETR S | 512×512 | 53.0 | 3.5 | 32.1 |
| RF-DETR M | 576×576 | 54.7 | 4.4 | 33.7 |
| RF-DETR L | 704×704 | 56.5 | 6.8 | 33.9 |
| YOLO26 L (공식 비교행) | 640×640 | 54.1 | 5.7 | 25.3 |

이 표로 MORAI 인식률·우리 1280 입력 지연·3뷰15Hz 달성을 예측하지 않는다.
서로 다른 모델군의 S/M/L 이름도 같은 크기를 뜻하지 않는다. 모델 탐색을 추가한다면
**RF-DETR S/M의 배포 비용을 먼저 선별**하고 L은 품질 대조로 남기는 것이 현재 4060 Ti
자원과 일정에 맞는 검토 순서다. 이 판단은 시험 우선순위이지 후보 선정 결과가 아니다.

읽은 연결 지점과 필요한 검증은 다음과 같다.

- `src/rfdetr/config.py`, `detr.py`: DINOv2 기반 pretrained closed-set 검출이다.
  text prompt로 MORAI 신규 이름을 추가하는 모델이 아니며 동일 16종 bbox 추가학습이 필요하다.
  COCO sparse category slots와 실제 80종 개수를 혼동하지 않는다.
- `detr.py:predict`, `models/postprocess.py`: RGB·ImageNet normalize와 기본 square resize다.
  develop의 `(H,W)` 인자는 patch×window 나눗셈 제약이 있다. batch를 지원하지만
  현재 YOLO의 종횡비 보존 letterbox와 같은 전처리가 아니다. 원본 좌표 복원과
  camera calibration 계약을 유지하고 같은 RGB·작은 객체에서 실제 품질을 확인한다.
- `docs/exports/onnx.md`, `tensorrt.md`: ONNX/TRT export·dynamic batch 경로가 있다.
  raw normalized cxcywh와 logits를 올바르게 decode하고 COCO/custom background slot,
  출력 이름·sigmoid·top-k·클래스 ID를 검증한다. NMS-free를 중복 검출 없음으로 해석하지 않는다.
  실제 전방1280/960·좌우640, batch2+1/3은 engine shape·출력 대응·지연 시험이 필요하다.
- `docs/learn/train/dataset-formats.md`: COCO 및 YOLO 형식 학습 경로가 있어 기존
  bbox 데이터 재사용 후보로 적합하다. ID/누락 주석·split 검증은 그대로 필요하다.
- `pyproject.toml`: develop은 Python>=3.10·Torch>=2.2·Transformers>=5.1을 요구한다.
  기존 Noetic Python3.8/Torch2.1.2를 덮어쓰지 않는다. 준비/학습 환경은 격리하고
  실제 배포는 `asmc-ros-noetic`의 TensorRT backend 등 호환 경로를 검증한다.
- 코드와 Apache-designated N/S/M/L은 Apache-2.0이다. Plus A/F/P/XL/2XL detection은
  PML-1.0으로 구분한다. 다른 모델군의 라이선스까지 일괄 Apache로 안내하지 않는다.

### 다음 작업의 우선순위

#77은 `c8ce3f55`, #78은 `9092491a`로 main에 병합했다. 현재 #80에 두 병합을 반영하되
파일럿의 person/motorcycle 회귀와 기존 통합 보류를 유지한다. #80은 병합하지 않았다.

1. 기존 파일럿의 사람·이륜차 회귀와 평가/샘플링을 먼저 점검한다. 새 모델을 쓰는 것만으로
   미완전 라벨·소스 편중·forgetting을 해결했다고 가정하지 않는다.
2. 같은 selection 표본에서 YOLOE-X 대 Grounding DINO의 신규 장애물 초안 품질을 비교한다.
   성능이 입증된 클래스와 부족 조건만 bag/AIM/Capture 의미 보강에 적용한다.
3. 정리된 공통 bbox 데이터로 기존 두 후보의 작은 재파일럿을 우선한다. RF-DETR은
   같은 평가본의 사전학습·TRT 자원 선별이 유망할 때만 짧은 비교학습 후보로 추가한다.
   이번 분석을 새 GPU 실행·전체 AIM 라벨링·세 번째 본학습의 완료로 표시하지 않는다.

main 동기화·문서 보강 후 현재 브랜치의 데이터/파일럿/준비 도구 CPU 시험 130개와
문서 migration·perception 계약 검사를 통과했다. 기존 실험 launch/default와 메시지는
변경하지 않았다. 공유 ROS의 Python3.8.10·Torch2.1.2+cu118도 읽기 전용으로 확인했다.

## 2026-10-11 후속 합의 — bbox 우선·AIM 범위·RF-DETR 선별

이 절은 앞 절의 후속 순서를 구체화한다. **작은 재학습 전에 RF-DETR의 사전학습
검출 품질과 3뷰 배포 비용부터 선별**한다. 운영 기본값은 그대로이고 새 GPU 시험이나
추가학습을 이번 문서 정리에서 실행한 것은 아니다.

### YOLOE bbox에 mask는 필수가 아니다

[공식 detection fine-tuning](https://docs.ultralytics.com/models/yoloe/#train-usage)은
같은 scale의 `yoloe-26l.yaml`에 `yoloe-26l-seg.pt`를 불러오고 `YOLOEPETrainer`를
사용한다. class+bbox GT로 학습할 수 있다. 우리 기존 파일럿은 이 경로를 사용했고,
`train_camera_bbox_pilot.py`가 detect task와 segmentation head 부재를 검사한다.
이름이 seg인 원본 checkpoint를 사용했다는 이유로 mask GT가 필요한 것은 아니다.
seg 모델 자체를 mask 출력까지 학습하는 경우에는 실제 instance mask/polygon 정답이
필요하다. bbox를 가짜 mask로 바꾸지 않는다. SAM 2는 현재 bbox 준비의 필수 항목에서
제외하고, 필요해지면 mask 주석이나 영상 전파 보조로 별도 검증한다. 이미 있는
Capture instance/semantic GT는 보존한다. PE 학습 후 고정 클래스 head와 임의 prompt를
바꿀 수 있는 원본 open-vocabulary 모델도 구분한다.

### AIM 대상은 세 폴더로 제한

대상 root는 `/mnt/d/asmc_aim/dataset`이며 사용자가 지정한 다음 폴더만 살핀다.
기존 417,435개 전체 RGB 집계를 이 세 폴더의 학습 가능 수로 재사용하지 않는다.

| 폴더 | 현재 처리 정책 |
|---|---|
| `static_010101` | 정적 장애물·클래스별 대표 세션 표본부터 감사 |
| `dynamic_obstacle` | 사람·이륜차 포함 동적 객체와 원거리·가림 표본 우선 |
| `tunnel_static_obstacle` | NAS 다운로드 진행 중. 전송이 끝난 세션만 별도 확인 후 감사 |

2026-10-11에 세 경로의 존재만 확인했다. tunnel 폴더가 존재하거나 원본에서 복사된
`.NAS_TRANSFER_COMPLETE.json`이 있다는 사실만으로 로컬 다운로드 완료를 판정하지 않는다.
복사 중인 파일은 hash/label/export 대상으로 열지 않는다. 전송 완료 세션의 파일 수·
manifest/크기·SHA를 확인하고 이후 고정 snapshot으로 사용한다. NAS 전송은 변경하거나
중지하지 않는다. 세 폴더 밖 lap·rear camera4는 이번 자동 학습 후보에 확대하지 않는다.

### RF-DETR M/L과 YOLO26x의 자원 판단

현재 고정 RF-DETR 소스의 `RFDETRMediumConfig`와 `RFDETRLargeConfig`는
DINOv2-windowed-small, hidden256, decoder4, patch16, windows2와 300 queries를
공유한다. 기본 입력/positional encoding 및 가중치는 다르다. M=576²/L=704²의
공식 지연 차이를 같은 960/1280 tensor의 모델 구조 차이로 단정하지 않는다.
현재 L checkpoint는 `rf-detr-large-2026.pth`다. 과거의 deprecated
DINOv2-base `rf-detr-large.pth`와 혼용하지 않는다.

[Ultralytics detection 표](https://docs.ultralytics.com/models/yolo26/#performance-metrics)의
640 기준 FLOPs는 YOLO26l=86.8B, YOLO26x=194.4B로 약 2.24배다. 이는 같은 입력의
연산 규모 대조이며 4060 Ti 지연·VRAM이 정확히 2.24배라는 뜻은 아니다. X는 우선
오프라인 품질 상한/teacher 후보로 두고, 실시간 3뷰 후보보다 낮은 실행 우선순위를 둔다.
RF-DETR-L은 YOLO26x처럼 큰 scale이라는 이유만으로 제외하지 않는다.

### 재학습 전에 수행할 제한된 비교

1. 사람·이륜차 회귀 원인 점검과 기존 선정 GT의 유효 범위를 확인한다.
2. YOLO26l 현재 기준선 대 RF-DETR M/L detection의 **기존 COCO 7종**을 비교한다.
   같은 RGB·3뷰 mask·원본 좌표와 기존 selection/heldout 분할을 쓴다. pretrained
   RF-DETR로 신규 box/PE_Firewall을 지원한다고 평가하거나 신규 16종 우열을 확정하지 않는다.
3. RF 기본 입력 M576²/L704²는 빠른 선별용으로 확인한다. 실배포 비교는 전방
   544×960/736×1280, 좌우480×640에서 M/L을 실제 추론·export한다. shape 제약과
   resize/letterbox 정책을 명시하고 원본 시야·좌표 복원을 검증한다. 기본 square와
   rectangle의 품질 변화도 남긴다. static shape별 engine과 좌우batch2+전방1을 우선한다.
4. PT/ONNX/TRT bbox·class·confidence 대응을 확인한 뒤 같은 FP16 TRT 조건의 지연을
   비교한다. PT RF-DETR와 TRT YOLO의 숫자로 모델 우열을 결정하지 않는다. 15Hz 입력에서
   전처리·decode·batch 대기·후처리 포함 p50/p95/p99·공백·VRAM을 기록한다.
5. 품질·자원이 명백히 열세인 조합은 이유를 남기고 통합 반복을 생략한다. 유망 후보만
   MORAI+TwinLite+ResNet+LiDAR/fusion+판제 preview에서 검사한다. 기존 지연/공급과
   VRAM90% 기준, 현재 운영 baseline은 유지한다. RF-L 또는 YOLO-X의 부담 여부는
   이 실측 전에는 미확정이다. X는 M/L에서 의미 있는 부족이 남을 때 제한 대조한다.
6. 그다음 YOLOE-X/Grounding DINO의 초안 품질 검증·자료 보강과 작은 재학습으로
   연결한다. RF 후보가 선별을 통과하면 같은 고정 bbox 데이터의 짧은 학습 비교에
   포함할 수 있다. COCO 기본 성능 선별만으로 신규 MORAI 장애물 학습 후 우열을 선언하지 않는다.

측정용 환경/engine 준비와 실제 `asmc-ros-noetic`의 3뷰 운영 연결은 분리해 검증한다.
RF-DETR의 최신 develop을 공유 Python에 설치하지 않는다. checkpoint·라이브러리 버전·
preprocessing·engine SHA를 고정하고 실패·OOM·생략 사유를 보존한다. 이번 추가에서는
학습 데이터·모델·ROS 메시지·실행 기본값과 GPU 실행을 변경하지 않았다.

## 2026-10-11 사람·이륜차 회귀 진단 — 재학습 없음

**회귀는 실제이며 첫 epoch부터 나타난다. 클래스 ID 오류·전이 head 전체 초기화로
설명할 수 없다.** #80은 Draft를 유지하며 본학습·운영 교체는 보류한다.
다음 작업은 재학습에 앞선 사전학습 모델 비교다. 이 절은 위의 “작은 재파일럿 먼저”라는
과거 후속 순서를 대체한다.

진입점은 [diagnose_camera_bbox_regression.py](../../../tools/perception/diagnose_camera_bbox_regression.py),
조건은 [camera_bbox_regression_v1.json](../../../config/experiments/camera_bbox_regression_v1.json),
요약은 [회귀 baseline](../../../config/baselines/perception_camera_bbox_regression_v1.json)이다.
원본·split·v5/pilot export·기존 예측·가중치와 운영 설정은 그대로 보존했다.
새 학습·teacher 라벨링·추가 수집·TensorRT 생성은 수행하지 않았다.

### 평가 경로와 전이 확인

- 기존 before/after × 960/1280의 8개 예측 캐시를 재사용했다. 데이터/config/model SHA,
  RGB SHA, 고정 16종 ID·canonical/text 순서, mask SHA, 뷰와 입력 크기·예측 멤버십을 확인했다.
  unknown 뷰에는 임의 차체 mask를 적용하지 않는다. cached evaluator SHA와 이번 도구 SHA는
  별도로 보존하며 동일 코드로 생성됐다고 표시하지 않는다.
- confidence **.40 고정 대조**와 기존 selection 선정값을 분리했다. heldout을 보고
  confidence를 재선정하지 않았다. 중간 epoch는 .40 고정으로만 추론했다.
- 클래스 일치 일대일 IoU≥.5와 이름을 무시한 기하 대응을 분리한다. 진단 범주는
  정확 검출 → 다른 이름의 IoU≥.5 → .10≤confidence<임계값 → IoU .1–.5 위치 오류 →
  저장 예측 범위에서 위치 미검출 순이다. `.10` 미만 예측은 저장돼 있지 않아
  완전 미검출과 구별하지 못한다. 클래스가 맞는 대응을 먼저 고정하고 잔여 기하 대응도
  일대일로 할당하므로 동일 예측을 여러 GT의 성공으로 세지 않는다.
- 부분 GT에서는 미대응 검출을 FP로 세지 않는다. 차량 24/12px·나머지16/8px 평가 범위와
  모든 원본 GT 결과를 각각 보존했다. 작은 GT나 mask 표시 설정을 학습 export에 전달하지 않았다.
- Ultralytics8.4.150의 이름별 전이를 실제 tensor로 확인했다. YOLO26l의 기존 7종에 대해
  `cv3`·`one2one_cv3`의 3개 scale weight/bias **84항목 모두 원본과 FP16 완전 일치**다.
  person/motorcycle 각각12항목도 같으며 shape가 호환되는 나머지1080tensor도 같다.
  이전 문서·도구 설명의 “고정16종은 무작위 head” 주장은 정정했다. 원본 COCO80 기준선과
  고정16종 초기화 결과는 계속 구분한다.
- YOLOE의 초기 모델은 실제 `YOLOEDetect`, fused=True, canonical/text 16종 순서가 맞다.
  원본 seg와 직접 비교할 수 있는1080tensor는 같고 fused terminal convolution은 shape·표현이
  달라 원본 seg의 클래스 행과 직접 동일 비교하지 않는다. 초기 motorcycle137/138 검출도
  확인했으므로 처음부터 motorcycle 이름이 car에 연결된 상태는 아니다. text encoder의
  중간 embedding을 저장된 terminal convolution만으로 비트 단위 복원했다고 주장하지 않는다.

### 동일 .40에서의 학습 중 변화

전방1280·측방640, 같은 mask, scoped heldout 기준이다. epoch0은 학습 직전 `initial.pt`이며
epoch1/4/8/16은 각각 `epoch0/3/7/15.pt`다. 부분17epoch는 배제했다.

| 모델 | 완료 epoch | person 정확 TP/26 | motorcycle 정확 TP/138 | motorcycle 기하 대응/138 | 그중 car 오분류 |
|---|---:|---:|---:|---:|---:|
| YOLO26l | 0 | 24 | 126 | 129 | 0 |
| YOLO26l | 1 | 13 | 0 | 5 | 3 |
| YOLO26l | 4 | 14 | 5 | 60 | 52 |
| YOLO26l | 8 | 18 | 0 | 85 | 82 |
| YOLO26l | 16 | 20 | 4 | 90 | 86 |
| YOLOE bbox | 0 | 24 | 137 | 137 | 0 |
| YOLOE bbox | 1 | 2 | 0 | 40 | 40 |
| YOLOE bbox | 4 | 13 | 3 | 45 | 17 |
| YOLOE bbox | 8 | 18 | 3 | 80 | 77 |
| YOLOE bbox | 16 | 19 | 2 | 86 | 84 |

기하 대응은 정확한 이름의 성공을 뜻하지 않는다. YOLO26l epoch0의 다른 이름3개는
person이며 epoch4에는 traffic_drum3개도 있다. YOLOE epoch4에는 person25개도 있다.

기존 선정 confidence에서는 YOLO26l1280/.25와 YOLOE1280/.20가 모두
person21/26·motorcycle4/138이다. .40 결과와 섞지 않는다. 960의 .40에서는 두 모델의
person20/26, motorcycle은 각각3/138·9/138이다. 해상도만 낮춰 회귀가 해결되지는 않는다.

학습 전 정확 TP였던 객체만 추린 **1280/.40 heldout 신규 누락**은 다음과 같다.

| 모델·클래스 | 신규 누락 | 다른 클래스 | 낮은 confidence | 저장 범위에서 위치 미검출 |
|---|---:|---:|---:|---:|
| YOLO26l person | 4 | 0 | 2 | 2 |
| YOLO26l motorcycle | 122 | 85 | 17 | 20 |
| YOLOE person | 6 | 0 | 3 | 3 |
| YOLOE motorcycle | 135 | 84 | 13 | 38 |

YOLOE person은 신규 누락6개와 새 성공1개가 함께 있어 합계가24→19다.
신규 누락은 모델별 학습 전 결과와 비교하며, 운영 수락의 공통 기준선은 원본YOLO26l1280/.40로 유지한다.

GPU 재평가는 MORAI·실시간 추론이 없는 상태의 전용 network-none 컨테이너에서
8개 누락 checkpoint만 직렬 수행했다. 총149.75초이며 3600초 제한 이내다.
최종16epoch는 저장 결과를 재사용했다. 이 시간과 PT batch1 지연을 전체 스택 지연으로 사용하지 않는다.

기존 loss/LR·optimizer/EMA/AMP 기록도 연결했다. 1epoch/16epoch의 실제 AdamW step은
YOLO26l493/7991, YOLOE492/7990이며 EMA update는 두 후보500/8000이다.
전체 AMP 생략9/10회를 보존했지만 이것만으로 회귀의 원인이라고 확정하지 않는다.
두 모델 cls loss는 첫 epoch2.059/1.626에서16epoch.556/.577로 감소했다.
학습 loss 감소는 heldout recall 보존의 증거가 아니다. native validation의0은 비활성화 placeholder다.

### 라벨·다양성 검토와 별도 수정 후보

- 이륜차 train **63장·191박스 전부**의 RGB와 원본→학습 이름, 모든 이륜차 crop을 확인했다.
  원본 `bike`가 motorcycle로 대응하며 대부분 스포츠바이크·스쿠터다. 일괄 car 오변환은
  관찰되지 않았다. 작은 crop 일부의 세부 subtype을 독립 확인했다고 표시하지 않는다.
- 191개 중 long-side<32px는103개, medium84개, large4개다. 단일
  `yolo-m5lvd/detection-bzetd` 소스63장이고, family63개가 독립 장면63개를 뜻하지 않는다.
  시각적으로 같은 배치·교차로·형상이 반복된다.
- 사람 train2759장·9261박스는15 source/run에 있으나 unknown 뷰1970장이다.
  min-side<8px는2838박스이며 작은 양성 GT는 그대로 보존했다. 크기만으로
  가림 상태를 추정하지 않는다. 사람 수가 많다는 사실만으로 충분한 다양성·주석 완전성을 보장하지 않는다.
- 소스별 고정 seed round-robin으로 추가200장을 검토했다. hl-fma와 detection-bzetd의
  자전거 탑승자 영상에서 **보이는 자전거에 별도 bicycle 박스가 없는 사례6장**, 이륜차 학습의
  `cd1eb8d57b6e…`에서 다수 사람·이륜차 누락1장을 확인했다. 원본 annotation_complete=false
  상태도 함께 보존한다. 이들은 새로 깨끗한 GT로 승인하지 않았다.
- 수정 후보는 `visual-review.json`의7개 image SHA와 `issues`에 모았다. 자전거 component
  박스 추가와 사람의 visible extent 확인, 누락 대상 보완이 필요하다. 확정할 수 없는 새 박스를
  임의 생성하지 않았으며 **실제 라벨 변경0개**다. car/bus/truck 혼입 전수 정정으로 확대하지 않는다.
- train8000장 중 annotation_complete=true는752장이다. 나머지는 완전성이 입증되지 않았다는
  뜻이며 모두 잘못됐다는 의미는 아니다. 다만 stock detector 학습에서 보이는 미주석 객체가
  배경으로 취급될 수 있는 위험은 위의 실제 누락 사례로 확인했다.
- 신규 회귀 영상99장 전체의 원본 RGB·GT를 검토했다. Enzo의 motorcycle은 실제 검정/빨강
  스쿠터를 가리킨다. 이 평가 사례에서 확정된 클래스명 정정은 없다. selection에는 motorcycle GT가
  없고 heldout138개는 모두 Enzo 소스다. 같은 배치의 반복 영상이므로138개 독립 장면으로 해석하지 않는다.
  사람 heldout도 주로 한 FOGGY Capture run에 편중된다. 다른 환경 일반화는 추가 대조가 필요하다.

시각 검수자는 Codex이고 human_confirmed=false다. 전체 이미지/주석 crop을 본 사실과
모든 대상의 exhaustiveness를 인증하는 것은 구분한다. v5와 원본을 덮어쓰지 않았다.

### 원인 판정과 다음 독립 대조

| 판정 | 내용 |
|---|---|
| 확인됨 | ID/text 순서·초기7종 전이·평가 SHA/mask 일치, 첫 epoch 회귀, motorcycle→car, 일부 누락 주석, 소스/장면 편중 |
| 근거 있는 가설 | 희소·유사 장면과 불완전 주석이 기존 표현 망각/분류 혼동을 유도했을 가능성; full fine-tuning·초기 학습률·증강의 영향 |
| 추가 대조 필요 | 샘플링·LR·head/backbone 업데이트·증강 각각의 인과 기여, 미주석 영역 보완 효과, 다른 독립 이륜차 장면 성능 |

다음 재학습에서는 우선 확인된 component/누락 주석을 보완하고 기존 객체 replay의 장면
다양성을 확보한다. 그다음 샘플링과 LR을 한 번에 바꾸지 말고 각각 대조한다. 더 강한 평활화나
confidence 하향만으로 분류 회귀를 해결했다고 판단하지 않는다. 이번에는 이 대조 학습을 실행하지 않았다.

### 재현·갤러리·수락 상태

[회귀 비교 갤러리](http://localhost:8768/camera-bbox-regression-v1/)에는 epoch 곡선,
32개 검수 시트·191개 이륜차 crop, before TP→after FN 전체1137개 대조를 보존했다.
1137은 모델·해상도·threshold별 반복 대조 수이며 고유 실패 영상은99장이다.
선정 confidence 대조도 학습 전·후에 같은 임계값을 적용했다. 초안에서 before .40과
after 선정값을 섞었던1138개 대조는 수정 사유와 함께 로컬
`intermediate-threshold-mismatch.tar.gz`에 보존하며 최종 집계에 사용하지 않는다.
[전체 목차](http://localhost:8768/)의 날짜·고정 E 번호로 연결한다.
원본 예측·새 epoch 출력·검수 기록은 `artifacts/camera-bbox-regression-v1/`이며,
Git에는 [요약 증거](../../../config/baselines/evidence/camera_bbox_regression_v1/)만 기록한다.

```bash
python tools/perception/diagnose_camera_bbox_regression.py audit --root /absolute/path/to/parent-workspace
python tools/perception/diagnose_camera_bbox_regression.py weights --root /absolute/path/to/parent-workspace
python tools/perception/diagnose_camera_bbox_regression.py report --root /absolute/path/to/parent-workspace
```

`audit/report`는 기존 오프라인 가상환경, `weights`는 고정 Ultralytics8.4.150 환경에서 실행한다.
`epochs`는 동일 환경에서 `--gpu-idle-confirmed`를 명시해야 하며 기존 출력 폴더를 재사용하지 않는다.
MORAI·실시간 추론 종료 확인 없이 이 인자를 사용하지 않는다. 기존 캐시를 지우지 말고 새
`--output`을 지정해 재현하며, 중단/실패 출력은 완전한 epoch 결과로 집계하지 않는다.

관련 CPU140개와 문서 migration·perception 계약/토픽/runtime 검사를 통과했다.
Hosted CI 결제 한도 차단과 기존 facade 실패는 별도 사항이며 이 진단이 해소했다고 주장하지 않는다.
추론/ROS 코드가 바뀌지 않아 운영 빌드·전체 스택 수락을 다시 수행한 것으로 표시하지 않는다.

**당시 계획:** RF-DETR L2026·704·batch3를 현재 YOLO26l과 먼저 비교한다. 아래 최종 진단에서
RF를 이번 재학습 후보에 추가하지 않기로 결정했다. 다음은 검증 teacher 초안·자료 보강 →
작은 재학습 비교. AIM은 합의한3폴더만 사용하며 다운로드 중인 터널 폴더는 완료 확인 전 제외한다.
이번 진단 완료는 recall 회복·본학습 후보 선정·운영 승격·#80 병합 승인이 아니다.

### RF-DETR 사전학습 비교 — 실행 경계

설정의 단일 기준은 `config/experiments/camera_rfdetr_selection_v1.json`이다.
공식 소스는 `external/baselines/RF-DETR`의
`eca736acab9f6fe93f2cbc82a1d9f6edfb53f0ea`이며 L 가중치는
`rf-detr-large-2026.pth`이다. 구형 `rf-detr-large.pth`와 혼용하지 않는다.
원본 checkout은 수정하지 않고 별도 snapshot·Python≥3.10 환경에서 SDK/ONNX를 준비한다.
ROS Python3.8의 패키지는 유지하며 실제 추론은 `asmc-ros-noetic`에서 SDK 없이
TensorRT10.13.0.35를 사용한다. 기존 worktree·마운트·운영 launch 기본값은 유지한다.

- 준비: `tools/perception/prepare_rfdetr_candidate.py`의 `onnx` → `engine`.
  checkpoint·ONNX·engine SHA와 실제 shape·batch profile을 보존한다.
- parity: `evaluate_rfdetr_candidate.py infer --kind rfdetr-pt`가 만든 batch1/2/3
  reference를 `check_rfdetr_parity.py --kind onnx|tensorrt`로 검사한다.
  클래스별 일대일 IoU≥0.98·confidence 차이≤0.02와 임계값 경계 이동을 구분한다.
- 품질: 같은 evaluator의 `yolo-trt`와 `rfdetr-trt` 결과를 `analyze`로 집계한다.
  기존 파일럿의 YOLO head와 현재 운영 engine의 head가 다르므로 예전 before 캐시를
  현재 운영 기준선으로 재사용하지 않는다. confidence0.40 기준선을 유지하고 RF confidence는
  selection에서만 선정한다. heldout에서 재튜닝하지 않는다.
- 비교: `report_rfdetr_candidate.py`는 원본 GT/운영 YOLO/RF 결과와 모든 변경 목록을
  보존한다. 미리보기 수만 제한하며 작은 GT나 원본 데이터를 삭제하지 않는다.
- 통합: 기존 `run_coco7_live_trials.py`를 사용한다. 날씨 반복은
  `run_coco7_weather_matrix.py --no-spawn --seconds 60 --repeats 3`으로 기존 actor를
  유지하고 모델별 batch를 적용한다. 기존 날씨는 복원하며 공개 제어를 발행하지 않는다.

RF는 전체 시야를 square resize하고 RGB·ImageNet 정규화를 사용한다. YOLO letterbox와
동일한 전처리라고 표현하지 않는다. 공식 sigmoid/global top300을 **먼저** 선택한 뒤 sparse
COCO category ID 1/2/3/4/6/8/18을 원본7종으로 제한한다. 추가 NMS나 소비 그룹 매핑은 없다.
원본 좌표 복원과 실제 batch1/2/3을 검사하며 중단 뷰를 기다리지 않는다.

현재 heldout의 오토바이는 단일 소스에 편중되고 일부 클래스의 GT는 매우 적다.
완전 GT≥20인 클래스의 macro F1과 주석된 객체 recall을 구분하고, 부분 GT의 미대응
검출을 FP로 세지 않는다. 이 자료 통과를 COCO7 전체의 독립 장면 수락으로 해석하지 않는다.
오프라인 group 추론 시간은 MORAI·다른 모델이 실행되는 input→output 지연이 아니다.

기존 카메라 worker·PREPARE·세대 무효화·Ego mask·메시지 MD5·전방 unique fusion은 유지한다.
실험 launch는 `camera_rfdetr_experiment.launch`이며 SHA가 고정된 생성 runtime 설정을
`camera_objects_config:=...`로 명시해야 한다. 기본 파일을 바꿔 운영 모델을 교체하지 않는다.
학습·teacher 라벨링·추가 수집은 이 비교에 포함하지 않는다. 완료 여부와 실제 측정치는
`config/baselines/perception_camera_rfdetr_selection_v1.json` 및 연결된 증거를 기준으로 확인한다.

### RF-DETR L704 첫 실행 결과 — 후보 확정 보류 (2026-10-11)

공식 L2026 checkpoint의 MD5와 SHA를 확인하고, 기존 split 그대로 selection300장·heldout300장과
temporal 자료를 합친 **고유675장**에 실제 재추론했다. TensorRT10.13.0.35 engine은 별도 환경에서
만들었으며 `asmc-ros-noetic`의 Python3.8에서 RF SDK 없이 기존 ROS factory로 로드·예열했다.
STANDBY 시험은 오류0·발행0으로 통과했다. 이는 ACTIVE 처리율·MORAI 통합 수락 시험이 아니다.

| confidence0.40 / 같은 RGB·mask·평가 범위 | 현재 YOLO26l | RF L704 PT / TensorRT 진단 |
|---|---:|---:|
| heldout person TP/GT | 24/26 | 22/26 |
| heldout motorcycle TP/GT | 127/138 | 136/138 |
| selection person TP/GT | 92/107 | 94/107 |
| selection bus TP/GT | 22/27 | 17/27 |
| selection car TP/GT | 29/38 | 29/38 |

사람 heldout recall은7.69%p 감소하여2%p 기준을 넘는다. 오토바이는6.52%p 늘었지만 단일
Enzo 소스에 편중돼 있다. 완전 GT가 충분한 heldout macro F1은 사실상 오토바이 한 클래스의
값이므로7종 전체 우위로 설명하지 않는다. selection에서 선정한 RF confidence0.60도 heldout
사람20/26·오토바이119/138로 회귀 기준을 통과하지 못했다. heldout을 보고 임계값을 다시 바꾸지 않았다.

수치 변환 검증은 **FP32 PT(TF32 OFF)→ONNX 통과, FP32 TensorRT 진단 통과, FP16 후보 미달**이다.
plain FP16·TF32 OFF·정규화 연산 보호·decoder 보호 등6개 engine과 실패를 모두 보존했다.
최종 혼합 정밀도는 backbone FP16, transformer/head/box 연산 FP32다. 첫12장만 검사하면
confidence0.40/0.60/0.70에서 통과했으나, 클래스·뷰를 보강한16개 batch군(실제 batch1/2/3)에서
동일 박스1건의 IoU0.9683·confidence 차이0.0152가 남았다. IoU0.98 기준을 면제하지 않았다.
원본을 확인하면 사슴을 person으로 오검출한 박스다. 모델 오분류와 FP16 경계 차이를 구분한다.
confidence0.10에서 발생한 추가 수치 차이도 이전 증거에 보존한다.

[최신 비교·실패 갤러리](http://localhost:8768/camera-rfdetr-selection-v1/final-analysis-v1/)는
원본 GT/운영 YOLO/RF를 나란히 표시하고, 수치 미달 박스의 확대 영상과 전체 변경41장을 연결한다.
미리보기46장만 제한하며 전체 예측675장은 보존한다. [전체 목차](http://localhost:8768/)에는 고정
E 번호·등록 시각을 붙였다. 핵심 지표·수치 검증·ROS smoke는
[요약 증거](../../../config/baselines/evidence/camera_rfdetr_selection_v1/)에 있다.

**첫 오프라인 판정은 현재 YOLO 유지·RF 학습 후보 선정 보류**였다. 이후 MORAI 재실행 뒤 아래
통합 진단으로 이번 후보 제외를 확정했다. 수치 일치 선행조건이 실패했으므로 최종3회 반복과
M·다른 해상도·X/XL 선별은 진행하지 않았다. 변환 문제를 모델 크기 문제로 간주해 다음 모델로
자동 이동하지 않는다. 저장 영상의 offline group 시간은
ROS 통합 지연도, 서로 다른 batch 스케줄의 공정한 속도 비교도 아니다.

운영 default와 weight·confidence·ROS MD5·unique fusion은 유지한다. RF 실험 뒤 같은 master에서
기본 YOLO launch를 실행해도 backend가 남지 않도록 `model_kind`만 초기화한 뒤 config를
로드한다. RF/YOLOE 설정은 명시한 종류를 사용하고, 없는 설정은 node의 YOLO 기본값을 쓴다.
외부에서 주입하는 trace 등 다른 파라미터는 지우지 않는다.
공통 카메라 검출75개·runtime31개와 RF 전용16개 CPU 시험, ROS overlay 빌드를 통과했다.
Hosted CI 결제 차단과 기존 facade 실패는 해소된 것으로 표시하지 않는다. #80은 Draft를 유지한다.

### RF 통합 진단 완료 — 이번 재학습 후보에는 추가하지 않음 (2026-10-11)

MORAI·TwinLite3뷰·BEV/차선·ResNet·LiDAR/전방 unique fusion·실제 전역 경로의 판제/제어
preview를 함께 실행했다. 실제 runtime은 `asmc-ros-noetic`이며 GUI/debug/trace OFF,
3뷰 Ego mask ON, confidence0.40이다. SUNNY는 RF→YOLO, FOGGY는 YOLO→RF로 각60초,
FOGGY YOLO 대조60초를 추가하여 **완전한 측정 창5회**를 보존했다. 주행 시험이나 최종
날씨별3회 반복 수락이 아니라 후보 중단 판단을 위한 진단이다.

| 날씨·실행 | 모델 | 전방/좌/우 Hz | 전방 p50/p95/p99 ms | 세 뷰 최대 공백 ms | 전체 VRAM peak MiB |
|---|---|---|---|---:|---:|
| SUNNY | YOLO26l·1280·2+1 | 14.93/14.92/14.88 | 43.4/64.3/77.1 | 182.0 | 4,342 |
| SUNNY | RF L·704·batch3 | 10.12/10.10/10.13 | 126.2/172.3/200.8 | 218.9 | 4,342 |
| FOGGY 최초 | YOLO26l·1280·2+1 | 14.82/14.72/14.77 | 44.1/68.8/76.3 | 352.2 | 4,207 |
| FOGGY | RF L·704·batch3 | 11.35/11.32/11.35 | 113.2/158.8/171.3 | 179.7 | 4,045 |
| FOGGY 대조 | YOLO26l·1280·2+1 | 14.93/14.93/14.87 | 42.1/57.7/68.9 | 196.1 | 4,129 |

위 지연은 **동일 source stamp의 입력 callback 수신→결과 수신 wall 지연**이다. 모델 단독
순수 추론 시간도, 센서 취득부터의 전체 지연도 아니다. bridge는 receive stamp를 생성하므로
header 간격만으로 MORAI 렌더링의 원인을 확정하지 않는다. 실행별·뷰별 percentile을 유지하며
p50의 평균을 전체 중앙값으로 쓰지 않는다. peak는 모델 준비와 ACTIVE를 포함한 전체 GPU
메모리로, WSL에서 제공되지 않는 프로세스별 VRAM을 추정하지 않는다.

RF는 평균10–11Hz여도 SUNNY의5초 창 최저9.0Hz, p95 172–174ms, 일부 p99>200ms였다.
FOGGY도 p95 158–159ms로150ms를 넘었다. 최소9.5Hz와 목표15Hz를 구분하며 평균10Hz라는
이유로 수락하지 않는다. 입력은 약20Hz로 공급됐고 오류·중복·역행 stamp는0이다.
VRAM은 약4.0–4.3GiB로90% 상한보다 여유가 있어 메모리가 주된 제한은 아니다.

마지막 누적 stage 진단에서 RF의3장 묶음은 전처리62.5ms·engine/전송23.9ms·decode6.8ms였다.
객체 노드 CPU 중앙값도 SUNNY에서 RF148.5% 대 YOLO67.2%(한 코어100%)였다. GPU 사용률은
RF47% 대 YOLO55%로, GPU 포화만으로 원인을 설명할 수 없다. CPU 전처리와 혼합 정밀도
경로의 비용이 남아 있다. 이 stage 값은 실행 전체의 wall percentile이나 모델별 동일 batch의
공정한 순수 추론 비교가 아니다. OpenCV 스레드 수만 바꾼 CPU 대조로는 명확한 개선을 확인하지
못했으며 운영 전처리·추론 설정을 바꾸지 않았다. 이번 결과를 RF 구조 자체의 최종 성능 한계로
일반화하지 않는다.

FOGGY 최초 YOLO는 측정49초 부근에 전방 입력253.6ms·좌282.7ms·우254.6ms·추가309.6ms의
수신 공백과 모든 카메라 출력의330–385ms 공백이 함께 나타나 **실패**다. CPU 전처리 probe가
첫 FOGGY matrix와 겹쳤으므로 해당 실행은 탐색 진단으로 명시하며 특정 원인으로 단정하지
않는다. 별도 대조 실행은 통과했지만 최초 실패를 지우거나 자동 면제하지 않았다.
두 모델 모두 전역 planner/control preview deadline을 충족했다. 다만 기존 차선 소비자의
`wrong_geometry_config_hash` 거부와 현재 화면에서 신호등 유효 검출 부재는 별도 미해결 조건이다.
전역 planner 준비·출력 통과를 음영 차선 소비나 전체 주행 수락으로 표현하지 않는다.

초기 환경 실패도 남겼다. 첫 실행은 nested `Msg` source 누락, 다음은 기존 underlay의 planner
실행 파일 누락으로 ACTIVE 전에 종료됐다. 전용 overlay에 동일 소스의 `asmc_msgs`와 planner를
빌드했고, 오래된 생성 header 문제를 해결한 뒤 위 측정을 수행했다. 팀원의 원본 소스·공유 설치·
마운트는 변경하지 않았다. 소스/실행 파일 SHA, 실제 source 순서와 실패 경로는
[통합 요약 증거](../../../config/baselines/evidence/camera_rfdetr_selection_v1/integrated-screen-summary.json)에 있다.
측정 actor 생성0·공개 제어 발행0이며 날씨는 원래 CLOUDY로 복원했다. 측정 노드만 정리하고
사용자의 bridge·Ego·기존 actor·MORAI는 유지했다.

**최종 판단:** 현재 YOLO26l을 유지하고 RF를 이번 작은 재학습 비교 후보에 추가하지 않는다.
오토바이127→136/138의 이득은 유용하지만 사람24→22/26·버스22→17/27 손실과 통합 지연,
FP16 엄격 parity 미달을 감수할 만큼 전반적인 이득을 입증하지 못했다. 특히 사람 누락 두 건은
15×50px·8×28px의 원거리 객체로 RF confidence0.297·0.187이었다. 전방704와1280의 해상도
차이도 영향을 줄 수 있으므로 모델 자체의 우열로 확정하지 않는다. heldout을 보고 threshold를
낮춰 재선정하지 않았다.

일정을 줄이기 위해 M·640·직사각960·X/XL과 최종12회 수락 반복은 생략한다. 기존 실험 backend와
증거를 남겨 추후 parity·전처리 개선 후 다시 비교할 수 있다. 지금은 검증된 teacher 초안·필요
자료 보강과 사람/이륜차 누락 주석·다양성·replay 문제를 반영한 **YOLO26l/YOLOE bbox 작은
재학습 비교**로 돌아간다. 이번 단계에서는 학습·teacher 라벨 변경·운영 교체를 수행하지 않았다.

최신 E218 갤러리는 통합 수치와 같은 GT ID로 연결한 변경 설명을 함께 제공한다. 이전 갤러리 일부의
변경 설명이 TP 우선 목록 순서로 잘못 짝지어져 `annotation_index` 기준으로 수정했다.
675장 품질 집계는 변하지 않았으며 이전 페이지·수정 안내·원본 HTML을 보존했다.

### 요청에 따른 RF-DETR M576 추가 대조 — 통합 완료·YOLO 유지 (2026-10-11)

L 비교 뒤 사용자가 M의 비용도 확인하도록 요청하여 공식 `rf-detr-medium.pth`를 추가했다.
이전 M 생략 판단과 L 실행 증거는 유지한다. 공식 MD5를 확인한 checkpoint의 SHA는
`749ff6071828aaffac63e204c4f4135ed3d6cdae4d702e086c360edc3b5768c8`이며 입력576×576,
batch1/2/3, L과 같은 backbone FP16·decoder/head FP32 정책을 사용한다.
고정 소스에서 M과 **새 L2026**의 모델 설정을 비교하면 positional encoding 크기36/44와
기본 해상도576/704가 다르다. 서로 다른 가중치를 사용하며, 이름만으로 YOLO의 M/L처럼
backbone 크기가 크게 달라진다고 가정하지 않는다.

같은 전방1280×720·좌우640×480 RGB를 사용한 CPU 전처리만의 대조는 다음과 같다.
`asmc-ros-noetic` Python3.8·OpenCV5.0.0·20 threads에서 예열10회 후30회 측정했다.
resize·RGB 정규화·CHW·3장 stack을 포함하고 JPEG decode·mask·모델·GPU는 제외한다.

| native 입력 | 전처리 p50 / p95 / p99 ms |
|---|---:|
| L704 | 15.72 / 16.95 / 19.67 |
| M576 | 10.89 / 12.97 / 13.25 |

M의 전처리 중앙값은 약31% 줄었다. 이 값은 유휴 CPU 대조이며, 앞선 L 통합 실행의
누적 stage 평균62.5ms나 입력→출력 지연과 동일한 지표가 아니다. 이 결과만으로 M의
통합 처리율 또는 YOLO 대비 지연 우열을 확정하지 않는다.

같은 고유675장에 실제 PT/ONNX/TensorRT 검증을 수행했다. confidence0.40의 TP/GT는
PT와 TensorRT에서 같았다. heldout 사람은 YOLO24/26·M22/26, 오토바이는127/138·135/138이다.
selection 버스는22/27·18/27이다. M의 confidence는 selection에서0.50으로 선정했으며 heldout을
보고 다시 조정하지 않았다. 사람 회귀 기준을 충족하지 못했고, 오토바이 소스 편중과 나머지
클래스의 부족 표본 때문에7종 전체 우위로 해석하지 않는다.

FP32 ONNX parity는 통과했으나 혼합 FP16 TensorRT는 **두 객체**에서 엄격 기준을 넘었다.
작은 오토바이 박스 IoU0.9644와 사람 confidence 차이0.0318이며, 임계값별 중복 기록을
별도 객체로 세지 않는다. 미대응 결과는 없지만 IoU≥0.98·confidence 차이≤0.02를 면제하지
않았다. 따라서 이후 통합 측정도 비용 진단이며 운영 수락이 아니다.

MORAI 재실행 뒤 동일한 C6 통합 부하에서 SUNNY M→YOLO, FOGGY YOLO→M을 각60초
측정하고 FOGGY YOLO 대조60초·센서/bridge만의 대조60초를 추가했다. 총5개 통합 창과
1개 센서 창이며 최종 날씨별3회 반복·주행 수락은 아니다. 실제 추론 컨테이너는
`asmc-ros-noetic`이다. GUI/debug/trace OFF, 3뷰 mask ON과 confidence0.40을 유지했다.

| 날씨·실행 | 모델 | 전방/좌/우 Hz | 전방 p50/p95/p99 ms | 세 뷰 최대 공백 ms | 전체 VRAM peak MiB |
|---|---|---|---|---:|---:|
| SUNNY | YOLO26l1280 | 14.93/14.93/14.87 | 38.5/51.4/58.4 | 173.7 | 4,079 |
| SUNNY | RF M576 | 14.95/14.93/14.92 | 69.1/109.2/121.9 | 185.8 | 3,856 |
| FOGGY 최초·실패 | YOLO26l1280 | 14.90/14.88/14.80 | 38.1/51.0/59.2 | 321.0 | 4,014 |
| FOGGY | RF M576 | 14.97/14.92/14.83 | 55.9/105.9/124.3 | 169.5 | 3,826 |
| FOGGY 대조·실패 | YOLO26l1280 | 14.90/14.92/14.80 | 38.2/57.0/64.7 | 434.5 | 4,093 |

표의 값은 동일 source의 callback 입력 수신→결과 수신 wall 지연이며 순수 GPU 시간이나
센서 취득부터의 지연이 아니다. M은 두 날씨에서 모든 뷰의 최소9.5Hz·p95≤150ms·
p99≤200ms·최대 공백≤250ms를 충족했다. 15Hz 목표 대비 평균14.83–14.97Hz이며
모든 창에서 엄격히15Hz를 충족했다고 표시하지 않는다.
오류·중복·역행 stamp는0이며 전역 planner/control preview deadline도 통과했다.
M의 전방 지연은 YOLO보다 p50 약18–31ms·p95 약49–58ms 크지만 **M을 연산 부족만으로
제외할 근거는 없다.** 전체 메모리는 두 모델 모두90% 상한보다 여유가 있다.

M의 마지막 누적 stage 평균은 SUNNY에서 전처리28.2ms·engine/전송13.5ms·decode5.8ms,
FOGGY에서26.6/13.0/5.6ms였다. 평균 실제 batch는2.76/2.65로, 중단/도착 차이에 따라
1/2/3을 처리한다. 서로 다른 batch 구성인 YOLO의 stage 평균과 순수 GPU 우열을 비교하지
않는다. 객체 노드 CPU 중앙값은 M137.6/132.2% 대 YOLO60.3/60.0%였다(한 코어100%).
M도 CPU 전처리·후처리 비용이 남는다. 이전 L 통합 수치와는 측정 세션·배경 부하가 다르므로
L→M의 통합 지연 개선량을 해상도만의 인과 효과로 계산하지 않는다.

FOGGY YOLO 두 실행은 모든 카메라 모델의 출력 공백으로 **실패**이며 대조도 면제하지
않았다. 두 번째 실행54.2초 부근에는 입력 전방254.8ms·좌252.1ms·우229.0ms·추가296.6ms가
함께 끊겼다. 센서/bridge만 실행한 별도60초에서도 전방258.2ms·좌258.1ms·추가258.7ms,
우231.0ms가 관측됐다. 따라서 이 입력 공백은 YOLO 추론 없이도 재현되지만 MORAI·네트워크·
bridge 중 원인 위치는 미확정이다. bridge receive stamp를 센서 취득 stamp로 해석하지 않는다.
M의 두 창 통과를 장시간 입력 안정성이나 기존 차선 소비자의 geometry hash 문제 해결로
확대하지 않는다. 원래 날씨CLOUDY를 모두 복원했고 측정 노드만 정리했다. actor 생성과
공개 차량 제어 발행은0이며 사용자의 bridge·Ego·MORAI는 유지했다.

**판정:** M576은 실행 가능한 예비 후보지만, 현재 사람/버스 손실과 엄격 FP16 parity 미달을
감수할 전반적 검출 이득은 입증하지 못했다. 이번 작은 재학습 비교에는 RF를 추가하지 않고
YOLO26l/YOLOE bbox에 집중한다. 이는 RF 학습 후 성능의 결론이 아니며, 이륜차 문제가
남거나 향후 품질 우위가 확인되면 보존한 M backend로 재검토할 수 있다.
[최신 M 비교 갤러리](http://localhost:8768/camera-rfdetr-selection-v1/m-followup-final-v2/)와
[M 요약 증거](../../../config/baselines/evidence/camera_rfdetr_selection_v1/m576-followup.json)에
원본 예측·수치 미달·전처리 범위를 연결했다. 운영 YOLO·confidence·fusion과 학습 자료는 변경하지 않았다.

### 이 비교 이후 남은 카메라 작업

1. 회귀 진단에서 확인한 누락 주석 수정 후보를 별도 검수 기록에 적용하고, 사람·이륜차의
   장면 다양성·replay와 selection 평가 부족을 보완한다. 기존16종 ID·split·원본 GT는 유지한다.
2. YOLOE-X/Grounding DINO 초안을 Capture의 독립 GT로 비교한 뒤, 검증된 종류만 혼합
   Roboflow·bag·AIM 자료 보강에 사용한다. bbox 작업에는 SAM2가 필수가 아니다. AIM은
   `static_010101`, `dynamic_obstacle`, `tunnel_static_obstacle`로 제한하고 마지막 폴더는
   다운로드 완료 확인 후 사용한다. teacher 미검출을 빈 정답으로 바꾸지 않는다.
3. 같은 고정 bbox 자료로 YOLO26l detection/YOLOE detection을 작게 재학습한다. 사람·이륜차
   회귀가 해소되지 않으면 본학습 후보를 선정하지 않는다. 데이터가 없는 stroller 등은 미지원으로
   남긴다. 학습 조건의 영향은 독립 대조하며 지금 샘플링/LR의 인과관계를 확정하지 않는다.
4. 선정한 한 모델을 본학습하고 TensorRT·전방 해상도·confidence를 검증한다. 원본 이름과
   소비용4그룹을 구분하고 현재7종 인터페이스에서 확대되는 계약을 판제/LiDAR와 확인한다.
5. 전방 bbox fusion·모드 전환·실행/복구 안내와 전체 스택30분 시험을 마친다. 위 FOGGY 입력
   공백과 차선 geometry hash 거부는 통합 수락 전에 별도로 해결한다. 3뷰/mask fusion과
   TwinLite BEV 자세 보정은 후속 확장으로 유지한다. #80은 결과·제한을 보존한 Draft다.


## 로컬 teacher와 저장 버전 — 2026-10-11 v6

이번 작업은 `feat/camera-teacher-data-expansion`에서 진행한다. #80의 진단 기준과 원본을
이어받으며 운영 YOLO26l·ROS 메시지·fusion·차량 제어를 변경하지 않는다. 학습과 새
MORAI 수집은 수행하지 않는다. v5와 파일럿 체크포인트는 그대로 보존한다.

### 저장 위치와 버전 선택

상위 workspace의 `data/dataset_versions/camera_objects_versions.json`이 단일 버전 목록이다.
`current_adopted`는 **학습 자료의 채택 버전**이며 운영 가중치 선택이나 학습 완료를 뜻하지
않는다. 각 항목에 실제 경로, manifest/정책 SHA, 검증 결과와 부모 통합 기록을 연결한다.
과거 버전에서 부모 정보가 없으면 null로 표시하며 추정하지 않는다.

```text
data/
├── roboflow/raw/<workspace>/<project>/<version>/export/  # 원본 보존
├── roboflow/curated-v5-capture-r1/                       # 이전 22,498장
├── roboflow/teacher-v6-inventory-r1/                     # 기존 자료·AIM 현황
├── roboflow/teacher-v6-independent-actor-reviews-r1/     # 독립 RGB 확인
├── roboflow/teacher-v6-gt-r4/                            # teacher 검증 정답
├── roboflow/teacher-v6-aim-screen-r3/                    # AIM 대표 프레임 큐
├── roboflow/teacher-v6-bag-screen-r1/                    # 장애물 bag 대표 RGB
├── roboflow/teacher-v6-runs/<model>-<input>-<revision>/   # 원본 예측·binding·실패
├── roboflow/teacher-v6-mixed-individual-reviews-r2/      # 이름 수정 근거
├── roboflow/curated-v6-teacher-r4/                       # 채택·보류·부모 SHA
├── dataset_versions/camera_objects_bbox_v6-teacher-r4/ # 실제 YOLO export
└── models/teacher_v6/                                   # 공개 B/L 가중치·별도 의존성
```

Capture 원본은 `data/capture_runs`, `data/capture_batches`와 기존 변환된 `data/datasets`
등의 실제 manifest 경로를 사용한다. 폴더 이름만으로 새 원본 경로를 만들거나 이동하지 않는다.
AIM 원본은 `/mnt/d/asmc_aim/dataset/{static_010101,dynamic_obstacle,tunnel_static_obstacle}`만
읽는다. 전용 컨테이너에서는 `/aim`을 read-only mount하며 같은 절대 원본 경로의 별칭도 제공한다.

export의 이미지는 **원본을 가리키는 심볼릭 링크**, labels는 새 YOLO bbox 파일이다.
export 폴더만 복사하면 원본 RGB는 따라가지 않는다. 원본 삭제·이동은 링크를 끊는다.
이식할 때는 manifest의 원본도 함께 보존하거나 별도 복사본과 SHA 검증을 준비해야 한다.
`val=selection`, `test=heldout`이다. 클래스 정책 v2와 데이터 단계 v6는 독립 버전이고,
`r`은 수정본이다. 기존 산출물을 덮어쓰지 않으며 새 판단에는 새 revision을 사용한다.

### 공식 로컬 모델과 실행 범위

[고정 설정](../../../config/experiments/camera_local_teacher_v6.json)을 모든 입력에 사용한다.
YOLOE-X는 기존 checkpoint 전용 combined21 embedding을 확인한다. B는
`IDEA-Research/grounding-dino-base`, L은 **OpenMMLab MM Grounding DINO Swin-L**의
`grounding_dino_swin-l_pretrain_obj365_goldg-34dcdc53.pth`다. 원래 GroundingDINO의
T/B 목록과 MM의 L을 같은 구현으로 표현하지 않는다. 미완전 학습 표시의 pretrain_all은
사용하지 않는다. Pro 1.5/1.6 API·이미지 외부 전송은 사용하지 않는다.

공식 참고 checkout은 상위 `external/baselines/{GroundingDINO,mmdetection}`에 있으며
vendor/submodule/ROS 의존성이 아니다. 원본 소스는 수정하지 않았다. 실제 SHA·라이선스·
가중치 pin은 [v6 증거](../../../config/baselines/evidence/camera_local_teacher_v6/)에 기록한다.

[준비기](../../../tools/perception/prepare_local_teachers.py)는 공식 파일의 revision/LFS SHA를
검증하고 끊긴 공개 다운로드를 Range로 재개한다. 같은 파일의 동시 쓰기를 잠금으로 막는다.
설치는 `data/models/teacher_v6/python-deps`와 전용 `asmc-camera-teacher-v6`에 한정한다.
공유 `asmc-ros-noetic` 패키지를 덮어쓰지 않는다. 준비 완료 후 컨테이너의 bridge를 분리하고
HF/Transformers offline 플래그로 추론한다. ROS master나 공개 제어 토픽을 사용하지 않는다.

B/L은 공식 resize(짧은 변 800·긴 변 최대 1333), batch1로 시작한다. L은 추론에 불필요한
학습용 gradient-checkpoint wrapper를 끄고, MM의 3-D 문장 attention mask를 지원하는
BERT eager 경로를 사용한다. 가중치·문장 mask·모델 구조는 바꾸지 않는다. 초기 fairscale/
SDPA 호환 실패는 로그에 보존한다. OOM은 별도 실패로 기록하고 640/1024 대비를 명시한다.
조용히 CPU·작은 모델·축소 입력으로 전환하지 않는다.

### 검증 정답과 채택 근거를 구분한다

Capture는 원본의 정확한 bbox 기하와 actor 종류를 별도로 확인한다. run·원본 category·
actor ID·연속 frame 구간의 앞/뒤 RGB를 검토한다. **같은 actor ID가 구간 안에서 다른 물체로
바뀐 사례가 있어 ID만으로 종류를 전파하지 않는다.** 새 35개 구간은 상자·콘·PE 방벽·차량의
독립 RGB 기록이며 이전 v5나 최종 모델 평가 정답을 수정하지 않는다.

teacher 검증의 selection/heldout은 whole Capture run 기준의 별도 역할이다. 각 행의
`dataset_split`에 원래 12/3/3 분할을 보존한다. 이 자료를 새로 숨겨진 최종 모델 시험이라고
설명하지 않는다. 종류 미해결 객체가 평가 범위에 있으면 full-image precision에서 제외한다.
크기 범위 16/8px는 이번 teacher 평가에만 적용하며 작은 정확한 학습 GT는 그대로 둔다.

새 박스용 전체 영상 정밀도와 **기존 정확한 박스의 이름 보조**를 별도로 집계한다.
후자는 이름 무시 일대일 IoU≥0.5로 원래 박스에 연결된 예측만 평가하며, 미대응 예측을 새
정답으로 채택하지 않는다. 잘못된 `concrete barrier` 이름을 RGB에서 WoodBox로 확인한
경우 그 주석만 box로 바꾼다. concrete barrier 전체를 box로 바꾸는 alias는 만들지 않는다.
원본 phrase·confidence·native bbox·원본 주석·수정 근거를 유지한다.

selection에서 confidence를 고르고 heldout에서는 고정한다. precision≥95%, GT/예측
각20개와 독립 장면3개가 부족하면 자동 승인을 보류한다. 불완전 GT의 미대응 검출을 FP로
세지 않는다. 이륜차·deer 등 GT가 적은 종류를 소표본 결과로 지원 완료 표시하지 않는다.

### 큐·검수·export·재개

[진입점](../../../tools/perception/camera_teacher_expansion.py)의 명령은 다음과 같다.

| 명령 | 결과 |
|---|---|
| inventory | Roboflow/Capture 채택·혼합 라벨과 AIM 다운로드/actor 현황 |
| prepare-gt | 독립 actor RGB 근거와 원본 bbox를 사용한 teacher 검증 입력 |
| samples | train의 source×원본 클래스별 seed 고정 표본 |
| aim-queue | 완료된 세션의 원본 frame association과 3뷰 대표 입력 |
| bag-queue | 명시적으로 지정한 장애물 녹화의 1초 간격 RGB |
| infer | 원본 phrase·bbox·SHA·실패·처리 시간; 중단 후 재개 |
| compare | 고정 0.4와 confidence 탐색·고정 heldout·두 승인 범위 |
| export | 독립 검수/검증된 묶음을 반영한 고정16종 YOLO와 재읽기 검증 |
| registry | 단일 데이터 버전 목록·중간 revision 대체 상태 |
| coverage | 클래스/하위 유형/뷰/날씨/분할별 보강량·부모 보존 검사 |
| comparison-gallery | 같은 RGB의 독립 GT/원본 주석과 세 teacher 비교 |
| admission-gallery | 원본 이름·teacher 오류와 채택 라벨 전후 비교 |
| gallery | GT·원본 teacher 제안 비교 페이지 |

AIM actor 목록은 **보일 가능성**의 근거이며 영상 GT가 아니다. 원본 association에서
camera 파일을 읽고 단순 파일 정렬과 frame index를 연결하지 않는다. rear는 제외한다.
`training_visible=false` 등 원본 생산자 플래그도 보존한다. 로컬 파일 수가 맞지 않는 터널
세션은 재확인 전 보류한다. download 중인 폴더 전체를 완료했다고 간주하지 않는다.
대표 프레임은 시작·중간·끝을 넓게 보되, 인접 사진 수를 독립 장면 수로 부풀리지 않는다.

첫 8시간은 진행 집계 시점이며 강제 종료 시간이 아니다. 실행 큐의 predictions.jsonl에
완료된 행을 flush하고 이미지·모델·prompt·설정 binding을 검사해 재개한다. 끊긴 마지막
JSON append는 원본 복사본을 남기고 복구하며 중간 행 손상은 거부한다. 이미 같은 RGB를
추론한 결과는 `--reuse`로 모델/전처리/픽셀 SHA를 확인해 계보와 함께 재사용할 수 있다.
새 실행이라고 세지 않으며 기존 예측은 덮어쓰지 않는다.

묶음 채택은 검증된 종류에만 source/조건 cohort를 고정하고 최대50장 seed 표본≥95%를
확인한 뒤 수행한다. 실패 표본과 복합 박스는 개별 수정 또는 보류한다. teacher 미검출을
negative로 만들지 않는다. 직접 대응 라벨은 teacher 승인이 필요 없고 기존 박스를 유지한다.
새 revision은 기존 evaluation·채택 v5를 보존하며 새 후보의 exact/pHash 분할 충돌을 검사한다.

### 완료 결과 — 채택 v6-r4

현재 채택본은 **22,631장·127,645객체**, train **21,851장**, selection **113장**,
heldout **667장**이다. v5의22,498장·127,334객체와 기존 평가 분할을 모두 보존했다.
새로운 **133장·311객체**는 Roboflow 혼합100장, AIM22장, bag11장이다.
bag11장에는 WoodBox 양성2장과 전체 RGB를 검토한 구조물-only negative9장이 포함된다.
정확한 작은 양성 GT는 학습 export에서 유지했다. 16종 ID는 고정이며 stroller 자료는0이다.

| 보강 학습 클래스 | 새로 포함된 이미지 | 객체 증가 |
|---|---:|---:|
| person / car | 12 / 12 | 67 / 24 |
| dog / deer | 6 / 40 | 7 / 44 |
| bicycle / motorcycle | 3 / 6 | 3 / 6 |
| bus / truck | 2 / 2 | 2 / 3 |
| box | 44 | 52 |
| steel_barricade / concrete_barrier | 31 / 4 | 31 / 6 |
| traffic_drum / plastic_barrier | 49 / 5 | 59 / 6 |
| bollard | 1 | 1 |

한 이미지에 여러 클래스가 있으므로 클래스별 이미지 수의 합은133이 아니다.
추가된 이미지 수는 독립 장면 수가 아니다. [전체 커버리지와 원본 보존 검사](../../../config/baselines/evidence/camera_local_teacher_v6/class-coverage.json)를 함께 확인한다.
최종 YOLO 파일 재읽기·이미지 SHA·클래스 순서 검증은22,631장·127,645객체를 통과했고
확인된 exact-pixel split 누출은0이다. 근접 장면의 계보 독립성이 모두 증명됐다는 뜻은 아니다.

v6-r1은 Roboflow100장을 더한 중간 부모다. r2의 일부 AIM 하위 유형 메타데이터를 바로잡았고,
r3의 원점 재생성 대신 **r1을 누적 부모로 한 r4**를 채택했다. 이로써 새로운 미채택 평가
후보 때문에 기존 승인2장이 사라지지 않게 했다. r2/r3는 원본·증거로 보존하며 단일 버전
목록에서 `superseded_intermediate`로 표시한다. 현재 경로는 항상 `current_adopted`로 확인한다.

AIM80장과 bag27장은 개별 RGB 검수했다. 그중 **AIM58장·bag16장**은 보호된 평가 입력과
pHash 거리≤6인 후보라 학습 채택을 보류했다. 이는 확정 중복 판정이 아니다. 원본과 수정한
라벨·대응 후보를 보존했으며, 영상/계보를 확인하기 전 분할 제한을 면제하지 않는다.
별도로 bag 검토39장 중12장은 회색 컨테이너 또는 먼 물체의 종류·전체 주석 확인이 남아 있다.

**사용자 확인:** 20260912 터널 입구의 주황 프레임·검정/노랑 사선무늬 대상은 객체 목록에
없는 고정 구조물이다. 해당 대상의 barrel/box/barrier 예측을 학습 양성으로 사용하지 않는다.
실제 WoodBox의 `concrete barrier` 예측은 독립 RGB 검수로 그 박스만 `box`로 수정했다.
컨테이너·다른 구조물을 같은 종류라고 자동 확정하거나 다른 실제 방벽까지 제외하지 않는다.

### Teacher 비교 결과와 사용 가능한 범위

세 모델은 **같은 독립 Capture GT1,653장·14개 run 그룹**에서 비교했다. 범위 안의 전체
객체 종류가 확인된 영상은1,254장이다. 나머지는 부분 GT로 처리해 미대응 검출을 FP로
세지 않았다. 원래 모델의 최종 heldout과 teacher 확인 목적의 역할을 구분한다.

아래는 confidence0.40에서 **box 원본 이름들을 학습 box로 매핑한** 결과다.
이름을 무시한 기하 대응은 별도이며 아래 TP로 섞지 않았다.

| Teacher | box TP / 예측 / GT | precision | recall |
|---|---:|---:|---:|
| YOLOE-26x | 28 / 33 / 305 | 84.85% | 9.18% |
| Grounding DINO-B | 15 / 16 / 305 | 93.75% | 4.92% |
| MM Grounding DINO-L | 56 / 56 / 305 | 100% | 18.36% |

L은 상자 검출에서 이득이 있었지만 recall이 낮다. PE 방벽은 평가 가능한GT102개 중
YOLOE-X의 정상 이름 대응이1개, B/L은0개였다. **대형 teacher 하나로 모든 미지원 장애물을
자동 정답화할 수 있다는 근거는 없다.** 잘못된 이름의 박스는 사람이/RGB가 종류를 확인한
경우에만 재활용하며, 전역 alias로 바꾸지 않는다.

selection에서 고른 threshold를 고정한 heldout까지 **표본·precision 조건을 통과한 후보**는
X person0.60, B person0.30, L person0.35·bicycle0.45다. precision≥95%와 양쪽 GT/예측20개를
확인했지만, **3개 이상의 독립 장면 근거는 아직 확정하지 못했다.** 반복 녹화 run 수를
독립 장면 수로 간주하지 않도록 r5 보고기에서 자동 승인 gate를 보강했다. 따라서 세 teacher
모두 현재 `approved=false`이며, 후보 threshold는 개별 검수 보조용이다. 새 소스 cohort
검수와 독립 장면 확인을 통과하기 전 자동 라벨을 확정하지 않는다.
**이 범위는 배포 confidence나 모델 지원 수락을 뜻하지 않는다.**

원래 정확한 박스에 연결해 **이름만 제안하는 조건부 통계 후보**는 더 넓다. X는 person/bicycle/
car/truck/traffic_drum/traffic_cone, B/L은 person/bicycle/bus/traffic_cone가 표본·precision 조건을
통과했다. 이 후보도 독립 장면 확인 전 자동 승인은 보류한다.
조건부 이름 precision을 전체 영상 precision으로 사용하면 안 된다. 상자·steel·plastic 등
이번 보강은 자동 묶음 승인 없이 **개별 RGB 검수**로 채택했다. 모든 변경의 근거·원래 박스·
teacher phrase와 actor prior를 남겼다.

AIM의 actor 목록에 dog가 있는데 실제 RGB에는 자전거 탑승자가 보이는 사례도 있었다.
actor metadata는 후보 선정용이며 종류 GT로 무조건 전파하지 않는다. 원본 생산자의
`training_visible=false`·3D/atomicity 미검증 표시를 그대로 두고, 별도로 확인한 2D bbox만
사용한다. 이 보강으로 원본3D GT가 검증된 것이 아니다.

### 실제 처리 범위와 재현

AIM 세 폴더의 snapshot은 **163,269장·187세션**이다. 로컬 파일 수가 일치한 세션에서
대표 **5,328장·148세션**을 X로 선별했다. 별도로 명시한 장애물 bag의 **500장**을 처리했다.
객체 없는 주행 전체를 전수 추론하지 않았고, 모든 원본을 재라벨링했다고 표시하지 않는다.
선별된231장의 같은 RGB는 B/L/X로 추가 대조했다. 모델·실행 SHA와 실패·재사용 기록은
[실행 증거](../../../config/baselines/evidence/camera_local_teacher_v6/runs.json)에 있다.
L은 기본800/1333·FP16·batch1에서 완료했으며 OOM 축소 대조는 필요하지 않았다.
초기 L-GT 실행에는 실행기 소스 snapshot이 없어 모델/설정 binding까지만 주장한다.
후속 focused 실행에는 코드와 패키지 pin을 기록했다. 이전 예측 재사용 시간을 새 처리율로
설명하지 않는다. 이 시간은 offline teacher 처리이며 ROS source-to-output 지연이 아니다.

상위 workspace에서 저장된 검수 결정을 재생성하는 예시는 다음과 같다. 기존 산출물 경로를
덮어쓰지 않고 별도의 `reproduced` 폴더를 사용한다. image_facts/export에는 numpy·OpenCV·
Pillow가 필요하므로 기존 전용 환경에서 실행한다.

```bash
python3 asmc-camera-data-audit/tools/perception/camera_teacher_expansion.py export \
  --curated data/roboflow/curated-v6-teacher-r1/curated.jsonl \
  --reviews data/roboflow/teacher-v6-admission-r4/reviews.json \
  --additional data/roboflow/teacher-v6-admission-r4/frames.jsonl \
  --output data/roboflow/curated-v6-reproduced \
  --dataset data/dataset_versions/camera_objects_bbox_v6-reproduced
```

검수 결정·manifest는 로컬 데이터에 있고 Git에는 SHA·요약·도구만 있다. 다른 PC에서 이
명령을 실행하려면 원본 이미지와 입력 검수 파일도 함께 옮겨야 한다. 단일 버전 목록의
부모 SHA·[입력 pin](../../../config/baselines/evidence/camera_local_teacher_v6/input-pins.json)을 확인한다.

[브라우저 실험 목차](http://localhost:8768/)에서 날짜/E번호가 붙은 teacher 같은 RGB 비교,
AIM/bag 세 모델 비교, **v6 bbox labels before / after**를 연다. localhost는 현재 개발 PC의
로컬 서버이며 팀 공유 호스팅이 아니다. 페이지는 상위 `artifacts/camera-teacher-v6`에 있다.
원본 예측과 개별 검수 contact sheet도 보존했다.

다음 우선순위는 보류74장의 실제 계보/근접 영상 검토, 부족한 plastic_barrier·box·이륜차의
유효 독립 장면 확보, 기존 person/vehicle 회귀 방지 샘플링이다. stroller는 여전히 미지원이다.
이번 v6은 구조적으로 검증된 부분집합이며 학습·운영 모델 교체나 전체 클래스 지원 수락이 아니다.

### PE Firewall 후속 진단 — 2026-10-11

사용자의 "프롬프트 문제인가 / 보강량이 너무 적다"는 피드백으로 **기존 자료만** 추가 대조했다.
운영 모델·기존21종 설정·v6-r4 라벨은 변경하지 않았다. 결과와 입력/실행 스크립트 SHA는
[PE 진단 증거](../../../config/baselines/evidence/camera_local_teacher_v6/pe-prompt-diagnosis.json)에 있다.

selection의 PE12장과 PE가 없는 대조12장을 고정했다. 평가 범위 안 PE GT는35개이며
반복 관측을 포함한다. 같은 Capture run이므로 독립 장면 검증이나 새 heldout이 아니다.

| 방법 | confidence | PE 일대일 TP / GT | PE 없는 대조 중 검출된 이미지 |
|---|---:|---:|---:|
| DINO-L 기존21종, 정상 plastic_barrier 이름 | 0.40 | 0 / 35 | 0 / 12 |
| DINO-L 단일 `plastic barrier` | 0.40 | 8 / 35 | 2 / 12 |
| YOLOE-X visual prompt | 0.40 | 6 / 35 | 0 / 12 |
| DINO-L 단일 `plastic barrier` | 0.10 | 27 / 35 | 12 / 12 |

기존21종 DINO-L도0.10에서 **이름 무시 기하 대응26/35**, 정상 plastic 이름15/35였다.
즉 위치 제안을 전혀 못 만드는 상태는 아니다. 다른 이름·낮은 confidence·다중 프롬프트
문맥이 함께 작용한다. `plastic road barrier`, `orange road barrier`, `water filled barrier`,
`water-filled traffic barrier`, `plastic jersey barrier`, `traffic barricade`, `road barrier`도
각각 단독 실제 추론했다. 이름을 구체화하면 항상 개선되는 것은 아니었다. threshold만
낮추면 대조 배경 오검출이 크게 증가한다. 위 대조 수는 FP 박스 수나 precision이 아니다.

[공식 YOLOE visual prompt](https://docs.ultralytics.com/models/yoloe/#predict-usage)는
단어 대신 참조 RGB의 bbox를 사용한다. 이번에는 대상24장 밖의 같은 run에 있는 orange PE
박스1개를 예시로 주고 X에서 실제 실행했다. 출력 `object0`의 의미는 그 참조 PE이며,
학습 클래스 `plastic_barrier`와 연결할 수 있다. 아직 다른 run/날씨 일반화나 자동 보강
승인으로 확대하지 않는다. SAM·추가 학습·새 다운로드·TensorRT 생성은 필요하지 않았다.

보강133장은 대량 라벨링 완료로 볼 수 없다. 기존 자료에는 이미 plastic_barrier591객체가
있고, Enzo의 직접 대응 PE142장은 **이름이 모호해서가 아니라** 주석 완전성/RGB 검수가
미완료여서 보류됐다. 이번 고정 표본12장에서 PE·drum의 직접 대응과 원본 박스 활용
가능성을 확인했지만, 아직 최대50장 cohort 검수 완료나142장 채택을 뜻하지 않는다.

자료 활용 우선순위는 다음과 같이 유지/구체화한다.

1. Enzo 등 명확한 PE/box/drum 라벨은 기존 박스를 보존하고 source×class 묶음 표본 검수로
   채택한다. 이 경로에 open-vocabulary teacher의 자동 승인 기준을 요구하지 않는다.
2. 혼합 클래스는 독립적으로 실제 종류를 확인한 actor/연속 등장 구간·시각적으로 같은
   묶음부터 검수한다. 잘못된 teacher 이름은 위치 제안과 분리해 개별/묶음 근거로 수정한다.
3. pHash 보류74장은 실제 RGB·계보로 확인한다. 비슷한 터널 배경만으로 확정 중복 판정하지
   않고, 실제 평가 누출을 확인한 후보만 제외한다. 기존 frozen split은 유지한다.
4. 신규 bbox는 단일 종류별 DINO와 YOLOE visual prompt를 라벨 초안 후보로 활용한다.
   누락된 다른 대상 객체·잘못된 박스·미검출을 검토하며, 예측을 무조건 정답으로 쓰지 않는다.

브라우저에서 `artifacts/camera-teacher-v6/pe-prompt-diagnosis-r1/index.html`의 같은 RGB,
원본21종/단일 text/visual prompt 결과와 Enzo12장 표본을 비교할 수 있다.

## 기존 자료 채택 확대 — 2026-10-11 v7

**현재 채택본은 `camera_objects_bbox_v7-adoption-r2`, 23,264장·129,630객체다.**
부모 v6-r4의22,631장·127,645객체와 모든 split/group을 그대로 보존했다.
[집계·보존 증거](../../../config/baselines/evidence/camera_data_expansion_v7/summary.json),
[실제 YOLO 파일 재검증](../../../config/baselines/evidence/camera_data_expansion_v7/verification.json),
[채택기](../../../tools/perception/camera_data_expansion_v7.py),
[새 채택 예외 정책](../../../config/datasets/camera_object_adoption_v7.json)을 참조한다.
운영 추론·ROS 인터페이스·bridge·스폰·주행·녹화·학습은 이번에 변경/실행하지 않았다.

| 구분 | v6-r4 | v7-r2 | 증가 |
|---|---:|---:|---:|
| 전체 이미지 | 22,631 | 23,264 | 633 |
| 전체 대상 객체 | 127,645 | 129,630 | 1,985 |
| train 이미지 | 21,851 | 22,023 | 172 |
| selection 이미지 | 113 | 113 | 0 |
| heldout 이미지 | 667 | 1,128 | 461 |

### 직접 대응 → 혼합 재분류 → 기존 teacher 검수 재사용

Enzo의 의미가 직접 대응하는 pending463장에 대해 **소스×원본 클래스별 seed20261009,
최대50장, 고유268장**의 실제 RGB를 검수했다. 포함된 PE 방벽·상자·드럼·콘크리트 방벽의
원본 박스를 활용했다. 기존 중복/대표본 제한2장은 유지하고 **461장·1,325객체**를 추가했다.
이 소스 전체는 이전부터 heldout이다. 따라서 PE142장의 증가도 **평가 보강**이며 train 증가로
설명하지 않는다. 이번에 본 heldout은 새로 숨겨진 최종 시험이 아니다. 묶음 표본 통과와
모든 개별 영상의 완전 GT 확인도 구분해 `review_basis=source_class_sample`을 남긴다.

혼합 라벨은 실제 RGB와 annotation ID가 표시된 확대 영상을 확인해 **mino76장,
tooth92장, jjj2장**을 train에 추가했다. Mino의 `obstacle`은 cargo/wood box·drum·
콘크리트 방벽으로 나누고, tooth의 `Obstacle_Yellow`는 WoodBox와 노란 drum으로 나눴다.
**tooth의 같은 `PE` 라벨에 원통 드럼과 사각 PE_Firewall이 공존**하므로 새 묶음 일괄
승인을 차단하고 주석별로 traffic_drum/plastic_barrier를 부여한다. v2 원본 규칙과 과거
채택본은 덮어쓰지 않는다. jjj9의 미확정 흰색 방벽4주석은 PE_Firewall로 확인했다.
모든 원본 박스와 작은 정확한 양성 GT를 유지했고, v7에서 새 annotation ID는 만들지 않았다.

기존 AIM/bag 유사 장면 보류 중 부족 종류13장을 대상으로 **모든 대응35쌍**의 RGB를
다시 확인했다. 배경은 같지만 배치 물체가 다른 AIM 상자1장·철제 방벽1장은 복원했다.
같은 배치 또는 독립성 미확인인 나머지11장은 유지했다. 다른 근접 후보도 미검수 상태를
유지하며 전체74장 중72장은 여전히 보류다. 21개 부족 후보 전체700쌍의 검수가 끝났다는
뜻이 아니다. pHash만으로 실제 누출을 확정하거나 한 쌍의 통과로 나머지를 면제하지 않는다.
기존 teacher 원본·개별 검수 결과를 재사용했고 새 추론·미승인 teacher 자동 채택은 없다.

| 대상 클래스 | 추가 train 등장 이미지 | 추가 heldout 등장 이미지 | 전체 객체 증가 |
|---|---:|---:|---:|
| box | 58 | 319 | 459 |
| plastic_barrier | 18 | 142 | 303 |
| concrete_barrier | 105 | 222 | 535 |
| steel_barricade | 2 | 0 | 2 |
| traffic_drum | 126 | 236 | 679 |
| bollard | 2 | 0 | 7 |

한 이미지에 여러 종류가 있어 클래스별 이미지 수를 더하면633이 되지 않는다.
수량 증가를 독립 장면 증가로 해석하지 않는다. 소스/run 그룹 수와 검증된 독립 장면 수를
분리하며 현재 독립 장면 수는 미확인이다. 확인하지 않은 날씨·터널·뷰는 unknown으로 남긴다.

천으로 덮인 물체·여러 물체를 합친 박스·tiny PE 종류 불명29개 검토 사례는 보류 기록에
남겼다. jjj의 탑승자 복합·dog/deer 혼합과 기존 숫자9 미완료19장도 이름만 바꿔 채택하지
않았다. 이는 원본 라벨이 불량하다는 뜻이 아니라 목표16종으로 분리하는 확인이 남았다는
뜻이다. 동일 픽셀·같은 split group 충돌은 검수로 면제할 수 없고, 새 후보의 모든 반대
split pHash≤6 대응을 확인한다. 이번 채택본의 exact-pixel split 누출은0이며, 모든 장면의
계보 독립성이 입증됐다는 뜻은 아니다.

### 저장·재현·갤러리

```text
data/roboflow/
  expansion-v7-reviews-r1/            # 고정 표본·개별 RGB 수정·보류 근거
  expansion-v7-reviews-r2/            # 유사 장면 35쌍 재검토, 2장 복원
  curated-v7-adoption-r1/             # 631장 추가한 중간 부모, 보존
  curated-v7-adoption-r2/             # 최종 통합 기록·summary·coverage
data/dataset_versions/
  camera_objects_bbox_v7-adoption-r2/ # data.yaml·manifest·검증·image symlink·YOLO labels
  camera_objects_versions.json       # current_adopted와 부모 SHA/검증 상태
artifacts/camera-data-expansion-v7/   # 원본 검수 시트·확대·전후·유사 장면 비교
```

상위 workspace의 data/artifacts이며 실행 worktree 내부의 별도 원본이 아니다.
이미지는 원본을 가리키는 symlink다. 원본·이전 revision을 삭제하지 않는다.
Git에는 도구·정책·요약·문서만 포함한다. 클래스 규칙 v2와 데이터 v7은 별도 버전이다.

재생성은 저장된 검수 패키지를 사용해 **새 경로**로 실행한다. 예:

```bash
python3 tools/perception/camera_data_expansion_v7.py \
  --parent ../data/roboflow/curated-v6-teacher-r4/curated.jsonl \
  --reviews ../data/roboflow/expansion-v7-reviews-r1/reviews.json \
  --output /tmp/asmc-v7-r1-reproduce \
  --dataset /tmp/asmc-v7-r1-dataset
```

r2는 r1의 curated를 부모로 사용하고 `expansion-v7-reviews-r2/reviews.json` 및
`--near-reviews ../data/roboflow/expansion-v7-reviews-r2/near-reviews.json`을 지정한다.
부모·이미지 SHA, 고정 표본 구성, source/class 검수, 기존 채택 불변,16종 ID, 원본 박스,
정확한 작은 GT, 실제 파일·정규 좌표·이미지 SHA와 split 누출을 검사한다.

[갤러리 전체 목차](http://localhost:8768/index.html),
[v7 요약·검수 진입점](http://localhost:8768/camera-data-expansion-v7/index.html),
[633장 원본/변환 비교](http://localhost:8768/camera-data-expansion-v7/final-adopted/index.html)를
제공한다. E 번호와 등록 시각은 기존 목차에 붙이며 실험 시작 시각으로 해석하지 않는다.

CPU 검사는 기존 데이터 전용 `../data/roboflow/.venv/bin/python`으로 실행한다.
호스트 기본 Python의 SciPy 미설치 오류는 코드 실패와 구분한다. 관련 데이터/teacher/갤러리
회귀, perception 계약과 문서 검사를 실행하고 로컬 결과를 증거에 남긴다. Hosted CI의
결제 차단/네트워크 미확인은 통과로 표시하지 않는다. PR #81은 Draft를 유지한다.

### 다음 구현으로 넘길 부족 조건

train 등장 이미지는 PE 방벽172장·철제 방벽178장·사슴61장·이륜차69장·자전거230장,
상자1,109장이다. stroller는0장이다. 평가 selection의 클래스 커버리지가 여전히 부족하며
이번 증가가 학습 준비·회귀 해결을 뜻하지 않는다. 기존 Capture12/3/3과 평가 분할을
train으로 옮기지 않는다.

[MPC 수집 파일럿](camera-mpc-object-collection-pilot.md)에 최신 등장 이미지 기반
**장애물10+희소 객체2 배분**, bridge GT/추가 LiDAR 수신, Object 파싱 한계, 시각·actor
연결, 지면 높이·기준점 검증, raw→Capture 전환과 SUNNY/FOGGY13시 파일럿을 명시했다.
파일럿 통과 후 전체 경로의11/13/15시6회 주행으로 확장한다. 수집 후 샘플링·비교 재학습으로
이어가며 이번에는 자동 수집을 시작하지 않는다.


## Capture 자동 수집 구현 — 2026-10-12

PR #80은 main에 병합했다(`c862ff07`). 기존 v7 자료는 보존했다.
[Capture 수집 인계](camera-mpc-object-collection-pilot.md)에 현재 실행 경로·저장 구조·20/30km/h 시험과
남은 조건을 모았다. 정차 native GT 검증 부분집합은 9장·57객체다. 검토 전용 주행과
수락된 학습 자료를 구분하며 전체 경로 여섯 조건 완료로 표시하지 않는다.
이번 변경은 별도 `feat/capture-route-collection`에서 진행한다. 학습·운영 기본 모델 변경은 없다.

## 2026-10-13 — 크기·배경·거리 분석과 전체 재분할 후보

사용자 요청에 따라 **새 정책 적용 전 분석만** 수행했다. 진행 중인 v9 학습, 원본 GT,
학습 손실과 운영 설정은 변경하지 않았다. 사용자는 새 분할을 위해 필요하면 원본
사전학습 가중치에서 다시 시작하는 것을 허용했다. 현재 체크포인트를 새 test에 평가해
독립 시험이라고 설명해서는 안 된다.

도구는 `tools/perception/review_camera_supervision_policy.py`다. `base`, `scan`,
`partition`, `resplit`, `panels`, `refine`, `evaluate`, `report` 단계를 제공한다.
산출물은 상위 workspace의 `artifacts/camera-supervision-policy-review-20261013-r2/`다.
r1은 빈 양성 TXT 직렬화 검사 실패를 보존한 미완료 실행이며, r2는 이를 수정했다.
scan의 느린 색상 비교는 정확한 RGB palette histogram으로 교체하고 기존 로그에서
재개했다. GPU 추론·optimizer update·실제 데이터 정책 적용은 수행하지 않았다.

### 1. 4px/64px² 양성 후보의 영향

원본과 square960 letterbox 모두에서 짧은 변≥4px **그리고** 면적≥64px²인 현재 양성을
유지한다. 경계값은 통과한다. 기존 ignore를 양성으로 승격하거나 배경으로 바꾸지 않는다.
현재 전체 양성262,902개 중83,009개(31.6%), Capture양성158,573개 중60,024개(37.9%)가
양성 제외 후보다. 학습 GT를 실제로 수정하지 않은 반사실적 집계다.

거리 연결이 가능한 현재 양성147,478개에서:

| 거리 근사 | 양성 | 제외 후보 | 비율 |
| --- | ---: | ---: | ---: |
| 0–20m | 31,005 | 255 | 0.8% |
| 20–40m | 32,962 | 3,938 | 11.9% |
| 40–60m | 25,525 | 9,260 | 36.3% |
| 60m 이상 | 57,986 | 42,555 | 73.4% |

60m 미만에서도15.0%가 제외된다. 따라서 크기 기준을60m 거리 cutoff나 안개 가시거리와
동일하게 취급하지 않는다. 같은 run의 SHA 고정 ledger, 정확한 native actor ID,
요청 시각 Ego age≤0.2초만 연결했다. 객체 기준점–Ego 3D거리이며 노출 시각의
optical depth가 아니다. 거리 미상115,424개는 추정하지 않았다.

### 2. 배경 확보와 손실 영역 확대 가능량

사용자는 일반/auto Capture의 상당수가 차선용 무스폰 주행이며, comp에서는 미션 바깥에
장애물이 없다고 설명했다. 메타데이터의 `capture_backend_requested=auto`는 호출 방식이라
그 값만으로 무스폰을 확정하지 않는다. run 종류와 semantic GT를 함께 사용했다.

60,764개 RGB/GT 경로를 검사했다. 무스폰 후보4개 run의14,145장 전부와 comp의5,679장이
차체 밖 대상 semantic 픽셀0이었다. 총19,824장은 **RGB 묶음 검수용 배경 후보**이며
자동 채택이 아니다. BBox 파일 부재와 teacher 미검출을 배경 근거로 사용하지 않았다.
현재 분할·현재 train 중복을 제외한 신규 train 후보는15,436장이다. 전체 재분할안을
따르면 신규 배경 후보의 역할은 train12,679장·selection5,145장으로 달라진다.

현재 Capture train36,444장에 대해 건물·도로 경계·노면 표식·교통 표지판·신호등의
semantic 영역을 추가하는 후보를 계산했다. 기존 ignore와 새 크기 ignore, 정상 양성
박스의 위치를 고려했다. `standing_object`·`etc`·포괄 Obstacle은 자동 배경에 넣지 않았다.
배경 면적은 보수적인 stride8 cell의 기하 집계이며 실제 assignment/gradient 크기는 아니다.
padding과 차체를 제외한 분모와 전체960 tensor 분모를 모두 저장했다.
추가 배경 cell2,732,080개가 가능하지만, 이 수정만으로 오검출이 해결된다고 결론내리지 않는다.
새 배경 이미지를 모두 학습에 추가해야 한다는 뜻도 아니다.

raw-only 자료의 split은 기존18개 Capture run 분할을 따른다. 현재 학습에 이미 포함된
자료의 영역 집계는 실제 active manifest 소속을 따른다. 두 기준의 차이를 숨기지 않는다.
날씨는 명시된 수집 기록16개 run에서 복원하고 근거 SHA를 남겼다. 기록에 없는 조건이나
터널 여부는 unknown이다. 구름/밝기만 보고 날씨를 추정하지 않았다.

### 3. 기존 예측의 거리별 recall과 오검출

기존150장 평가본의 baseline·1/2/4epoch 저장 예측을 confidence0.40으로 재집계했다.
원래 GT 범위와4px/64px² 후보 범위를 함께 보존했다. 새 GT 범위에서 점수가 높아지는
것을 재학습 개선으로 해석하지 않는다. 원래 범위의 FP는18/1,620/976/505개다.
4epoch 후보 범위에서는492개이지만 이는 ignore 범위 변경에 따른 평가 효과다.

전체322개 GT 중 거리 연결은34개뿐이다(0–20m23개·20–40m7개·40–60m4개).
그중 여러 종류는 기존 COCO 모델의 미지원 종류라 전체를 기존7종 성능으로 설명하지 않는다.
거리 미상288개는 별도로 표시한다. 미대응 예측에는 정답 거리가 없으므로 FP에
임의의 거리를 부여하지 않는다. 해당 평가본으로 전체16종60m 검출 성능을 수락할 수 없다.
Capture FP의 semantic 박스 중심점도 집계했지만 박스 전체의 배경 판정은 아니다.

### val/test 동일 기준과 새 전체 분할

기존 평가본에 같은 크기 후보를 적용한 별도 export, 완전 주석만 사용한 균형 진단 묶음,
기존 train과의 ID/파일/픽셀/그룹 격리 검사를 만들었다. 기존16종 selection이 기존 test의
Enzo 프로젝트와 comp_sample_b_seg05 run을 이미 사용했으므로 기존 test 전체를 새로운
독립 heldout으로 표시할 수 없다. 기존 소속과 원본 결과는 보존한다.

사용자 승인에 따른 **전체 재분할 후보**는 다음과 같다. 현재 학습에는 적용하지 않았다.

| 역할 후보 | 이미지 | 16종 양성 존재 | ignore만 남아 학습 양성으로 넣지 않을 후보 |
| --- | ---: | --- | ---: |
| train | 38,846 | 모두 있음 | 455 |
| selection | 5,144 | 모두 있음 | 1,028 |
| heldout | 10,943 | 모두 있음 | 419 |

원본 데이터와150장 검수본을 ID별로 결합해54,933장의 후보를 만들었다. 같은 스폰 layout의
날씨 반복, 같은 comp 시나리오 가족, 관련 숫자 라벨 Katri 소스를 그룹으로 유지한다.
train최소60%, val/test최소5%·최대30%, train종류별100박스·val/test종류별20박스 이상을
조건으로 평가 자료량을 최소화한 그룹 배정이다. 정확한80/10/10보다 누출 방지를 우선했다.
출처가 불명확한 모든 유사 장면의 계보까지 확인했다는 의미는 아니다.

전체 평가 후보에서 클래스·날씨·뷰·연속 구간을 고려한 검수 패널은 val131장·test96장이다.
동일 correlation group 최대2장으로 제한한다. val 일부 클래스는20GT 미만이며 소표본이다.
모든 공존 주석과 ignore를 유지하며 박스를 버리거나 같은 박스를 복제해 균형을 맞추지 않는다.
두 패널 중 partial은 각각128장·71장이다. 따라서 **최종 평가 준비 완료가 아니며** 전체16종
누락 검수를 통과하기 전 precision/F1에 사용하지 않는다. 그룹당2장 제한 없는 이전
148장·89장 패널도 비교용으로 보존했다.

현재 학습은 새 test의 일부를 이미 보았다. 새 분할을 사용하면 원본 사전학습 가중치에서
새 run으로 시작해야 한다. 과거 사람의 검토/개발 노출 이력은 새 학습 시작으로 없어지지 않는다.
다음 작업은 평가 패널의 image-wide16종 검수와 배경 검수, 실제 supervision 계약 검증 후
새 학습 데이터 버전을 확정하는 것이다. 학습 재시작은 이번 분석에서 수행하지 않았다.

갤러리: `http://localhost:8768/camera-supervision-policy-review-20261013-r2/index.html`.
정책·거리·배경·새 분할·클래스별 분포 JSON과 원본 overlay를 같은 페이지에 연결했다.


## 2026-10-13 · v10 재분할·전처리 확정 및 사전학습부터 재시작

현재 채택 데이터는 `data/dataset_versions/camera_objects_bbox_v10-resplit-r2`다. `r1`은 legacy bbox-only 검수 영상의 semantic SHA 미존재를 발견해 준비에 실패한 산출물이며 `_SUCCESS`가 없고 학습에 사용하지 않았다. r2는 검수 완료 bbox와 선택적 segmentation 연결을 구분한다. 원본, 이전 버전, 실패 증거를 보존했다.

- Train **39,391장·양성 148,778개·ignore 230,546개**. Val **212장·475개 양성**, test **116장·445개 양성**.
- 같은 Capture 객체 배치의 날씨 반복, 연속 구간, 확인된 Roboflow 증강/복제 계보를 분할 단위로 묶었다. 기존 53,588장의 train을 그대로 유지하지 않은 이유는 평가 독립성을 위한 전체 그룹 재분할이다. ignore-only train 후보 455장은 보관하고, 기존 2,000장 외 검수된 배경 1,000장을 추가했다.
- 평가 reserve pool은 selection 5,144장 / heldout 10,943장으로 유지한다. 이번 주 평가본에 뽑히지 않은 예약 영상을 train으로 되돌리지 않는다. 과거 학습/검토 이력이 있는 이미지가 포함되므로 새로운 비공개 시험이라고 표현하지 않는다. 새 가중치는 원본 pretrained에서 다시 시작한다.
- Train의 완전 주석 1,408장, partial 34,983장, 검수된 negative 3,000장이다. partial은 확인된 박스 자체가 틀렸다는 뜻이 아니라 **전체 16종의 누락 없음까지 보증되지 않았다**는 뜻이다. 확인된 semantic 배경과 기존 클래스 한정 손실 범위를 사용하며 나머지를 일괄 배경 정답으로 만들지 않는다.

| 학습 클래스 | Train 등장 이미지 | 양성 박스 | Epoch 0 노출 이미지 | Val GT | Test GT |
|---|---:|---:|---:|---:|---:|
| `person` | 12,501 | 30,213 | 13,589 | 116 | 63 |
| `dog` | 4,468 | 4,536 | 4,859 | 22 | 21 |
| `deer` | 4,629 | 4,738 | 5,122 | 22 | 24 |
| `bicycle` | 5,871 | 6,108 | 6,509 | 22 | 22 |
| `car` | 5,467 | 11,449 | 5,486 | 39 | 35 |
| `motorcycle` | 4,862 | 5,178 | 5,395 | 21 | 43 |
| `bus` | 5,212 | 7,185 | 5,519 | 22 | 22 |
| `truck` | 3,871 | 7,214 | 4,300 | 20 | 29 |
| `bollard` | 4,617 | 30,166 | 4,827 | 21 | 20 |
| `box` | 4,639 | 4,950 | 5,051 | 27 | 22 |
| `concrete_barrier` | 6,151 | 7,536 | 6,688 | 22 | 22 |
| `steel_barricade` | 5,417 | 5,590 | 5,967 | 20 | 23 |
| `traffic_drum` | 4,088 | 13,755 | 4,480 | 26 | 23 |
| `plastic_barrier` | 3,842 | 4,262 | 4,518 | 26 | 27 |
| `traffic_cone` | 2,713 | 3,006 | 3,529 | 27 | 21 |
| `stroller` | 2,892 | 2,892 | 3,688 | 22 | 28 |

### 균형 판단

사람 등장 이미지가 콘의 약 4.6배다. 볼라드는 30,166박스지만 등장 이미지는 4,617장이므로 박스 수로 과도하게 반복하지 않는다. 모든 유효 학습 이미지를 매 epoch 최소 한 번 사용하고, 역제곱근 빈도 가중치로 희소 종류를 이미지당 최대 2회 노출한다. Epoch 0은 41,728회 노출이며, 반복 후 사람/콘 노출 차이는 약 3.9배다. 흔한 클래스 제거로 균형을 맞추지 않았다. 각 클래스 최소 2,713장과 신규 16종을 확보했으나 장면 독립성과 실전 인식 성능을 보장하는 수치는 아니다.

Val/test는 종류별 GT가 각각 20개 이상이다. 355개 후보 패널을 Codex가 RGB·원본 GT와 대조했고 애매한 사례는 원본 크기로 확인했다. Val 14장/test 13장은 누락된 고정 드럼·방벽, 비정상 스폰, amodal 박스, 약 90도 증강 등의 이유로 주 평가에서 보류했다. 기존 GT를 teacher 결과로 덮어쓰지 않았다. 많은 종류가 1~2개 배치/소스 그룹에 의존하므로 일반화·통계적 회귀 판단에는 제한이 있다.

### 전처리·배경 정책

- 16종과 train/val/test 모두 **원본 영상 또는 square 960 tensor에서 짧은 변 <4px OR 면적 <64px²**이면 ignore. 경계값은 유지한다. Capture 가시 instance 픽셀 <4개도 ignore다. 거리 60m cutoff는 적용하지 않는다.
- 전방 1280×720→960×540, 측방 640×480→960×720 후 각각 960×960 padding. 원본 기준도 확인하므로 측방 영상을 확대해 원래 극소 객체가 양성으로 되살아나지 않는다. class별 갤러리 24/12px·16/8px 제한은 학습/평가에 사용하지 않는다.
- 뷰가 확인된 21,488개 train 이미지에는 Ego mask가 모두 적용된다. 미확인 Roboflow 17,903장은 추정 mask를 적용하지 않는다. 전체 평가에서도 같은 mask SHA를 검사한다.
- 정확한 작은 실제 객체를 negative로 바꾸지 않는다. Ignore와 positive가 겹치면 정상 positive를 우선한다. 부분 가림의 보이는 instance 박스와 원본 하위 유형은 보존한다. 유모차 안 아기는 별도 person 양성이 아니다.
- 기존 안전 semantic 배경에 `building, road_edge, road_sign, traffic_sign, traffic_light`를 추가했다. `etc`, `standing_object`, 포괄 `Obstacle`은 승인하지 않았다. 총 1,814,205개 안전 grid cell이 추가됐으며 이는 전체 이미지 수나 독립 배경 장면 수가 아니다.
- 무스폰 Capture라도 지도 고정 드럼·방벽이 보여 이미지 전체를 background로 승인하지 않았다. 추가 negative는 기존 source×view 50장 검수에서 통과한 묶음만 사용하고, 제외 사례와 중복을 제거했다. 3,000장은 train의 약 7.6%다.
- 색상/수평반전만 유지, mosaic/mixup/회전/이동/축소는 OFF. 양성·ignore·안전 배경 grid를 같은 변환으로 이동한다.

### 검증과 실행

실제 39,719장 RGB SHA·디코딩 픽셀 SHA, 원본/YOLO 라벨·sidecar·고정 ID·좌표·분할/계보를 검사했고 확인된 충돌은 0이다. 18,072개 Roboflow train 후보와 최종 평가 328장의 perceptual hash 비교에서 나온 19쌍은 전부 영상으로 확인했으며 동일 도로의 다른 차량 배치/위치여서 중복으로 확정하지 않았다. 이 검사가 모든 미기록 계보의 독립성을 증명하지는 않는다.

고정 학습 컨테이너에서 실제 로더로 source/view/완전성/16종을 포함한 122개 입력을 검사했다. 실제 960 image·bbox·ignore·safe grid·반전·collate와 전체 순서/최대 2회/재현성을 통과했다. 두 활성 검출 head의 background/ignore gradient, partial/negative, 양성 우선 및 기존 complete 손실 일치 CPU 시험 23개가 통과했다. 관련 정책/평가/분할 회귀 39개도 통과했다. 데이터 준비 중 optimizer update는 하지 않았다.

기존 v9는 full E8까지 계산 후 기존 평가본과 회귀 진단본으로 평가·SHA를 보존한다. 감독 PID1은 host Docker STOP 신호로 `Tsl`, 계산 자식은 `Rl`인 것을 확인했다. 같은 namespace 안의 init STOP 요청만으로는 정지되지 않아 상태를 확인하고 host 신호로 보정했다. 기존 frozen 코드는 수정하지 않았다.

새 실행은 `artifacts/camera-bbox-main-v10/run-20261013-r1`에 고정했다. `transition.json`이 현재 상태의 근거다. `_EVALUATED`와 새 데이터/검증 SHA가 모두 통과할 때만 기존 컨테이너를 종료하고 새 컨테이너 `asmc-camera-main-v10-r1`을 시작한다. 공식 `data/models/camera_coco7_v1/yolo26l.pt`에서 시작하며 v9 model/EMA/optimizer를 재개하지 않는다. Baseline과 200-batch smoke 후 smoke 가중치를 버리고 본학습한다. 960 / AMP / AdamW LR0.001 / batch4→2→1 / 유효16 / 최대40epoch·첫24시간이다. 초기 회귀는 사용자 승인대로 경고만 남기며 자동 중단하지 않고, 오류·예산 경계는 유지한다.

중간 선택에는 새 val 212장만 사용한다. 이전 618장은 새 train과 겹칠 수 있어 새 선택 기준에서 제거하고 v9 역사 자료로만 보존했다. Test 116장은 중간 평가에 넣지 않는다. 운영 모델·ROS/fusion·PR 병합 상태는 변경하지 않았다.

- [분포·검수 갤러리](http://localhost:8768/camera-dataset-v10-review-20261013-r1/index.html)
- [새 학습 상태](http://localhost:8768/camera-bbox-main-v10/run-20261013-r1/live.html)
- [실험 목차](http://localhost:8768/index.html)
- 설정: `config/datasets/camera_pretrain_policy_v3.json`, `config/experiments/camera_bbox_main_v10.json`.
- 도구: `prepare_camera_pretrain_v10.py`, `report_camera_pretrain_v10.py`, `finish_camera_epoch_boundary.py`, `transition_camera_pretrained.py`.
- 로컬 상세 증거: `artifacts/camera-dataset-v10-review-20261013-r1/`; Git 요약: `config/baselines/evidence/camera_pretrain_v10/`.
- Hosted CI는 기존 결제 한도 차단과 구분하며 이번에 실행 성공했다고 보고하지 않는다.


## 2026-10-13 · v9 E8 완료 평가와 warm-start 판단

v9 E8 저장/평가는 16:19 KST에 완료됐고, 기존 승인에 따른 자동 전환으로 v10이 16:20 KST부터 baseline/smoke 후 본학습을 시작했다. 사용자의 E8 선행 검토 요청을 확인한 뒤 v10은 500 batch 지점에서 Docker pause로 동결했다. 완료 epoch는 없었으며 초기화 방식은 원본 pretrained다. 평가가 끝난 뒤 이 방식을 유지해 unpause한다.

같은 150장·confidence0.40에서 전체16 macro F1은 E1 22.41%, E2 38.28%, E4 49.85%, E8 61.13%였다. 신규9종(소표본 포함) macro F1은 33.11→50.48→57.57→66.27%, 기존 기준 FP는1620→976→505→332로 개선됐다. 그러나 motorcycle recall은 원본20/23→E4 14/23→E8 7/23, person은 원본19/22→E8 17/22로 회귀가 남았다. 신규 클래스 대부분 GT20개 미만이므로 점수는 개발 진단이며 배포 수락이 아니다.

기존 예측을 재사용해 4px/64px²로 재집계하면 E8 fixed16 F1은61.97%, FP309이고 motorcycle recall은 동일30.4%다. 원래 잘 잡던 충분히 큰 motorcycle→car, person→box/deer 사례를 원본 영상으로 확인했다. 단순 극소 GT 필터만으로 회귀가 해소된다고 보지 않는다. 이 재집계는 기존150장의 주석 완전성을 새로 보증하는 결과가 아니다.

중요: 새 val212장 중207장, test116장 모두가 v9 train과 ID/fileSHA/pixelSHA 기준 정확히 겹친다. E8를 v10 초기 가중치로 쓰면 새 분할의 평가 독립성을 잃는다. 이어 학습은 기술적으로 가능하고 최종 성능 향상을 배제할 수 없으나, 이 경우 학습 이력 전체와 독립인 평가본을 다시 마련해야 한다. E8는 보존하고 공식 pretrained 시작을 유지한다. 새 시작 자체가 회귀 해소를 보장하지 않으며 예정된1/2/4epoch 평가로 확인한다. 초기 회귀 자동 중단은 사용자 요청대로 비활성 상태를 유지한다.

[평가·비교 갤러리](http://localhost:8768/camera-epoch8-decision-20261013-r1/index.html) · 로컬 원본 assessment.json/decision.json 보존. 새 GPU 재추론·threshold 재선정·가중치 변경은 하지 않았다.


## 2026-10-13 · 본학습 live 화면 개편

`report_camera_bbox_main.py`와 `templates/camera_training_live.html`이 동일한 `live.html` 주소에 반응형 현황 화면을 생성한다. 학습 프로세스·frozen code·가중치는 바꾸지 않는다. 실제 dataset summary와 supervisor/main JSON을 사용하며 v8 수량을 하드코딩하지 않는다.

현재 epoch/전체 일정 진행률, 최근 batch 기준 epoch ETA, 중단 시간 포함 경과 시간, 처리율, 학습 프로세스 최대 예약/할당 VRAM, 평가 F1 추이·클래스별 recall/precision/F1·회귀 경고를 표시한다. 브라우저가 관측한 loss는 경로별 localStorage에 최대480개를 보존하며 첫 접속 이전 이력을 꾸며 표시하지 않는다. 15초 갱신, 3분 갱신 중단 경고, 평가/일시정지 상태에서는 ETA 숨김. 시각은 Asia/Seoul로 고정한다.

데스크톱1440px/모바일430px headless Chrome 렌더링을 확인했다. 처음 epoch, epoch 경계, E4에서 E5 재개, E8 종료, main 기록 없음 5개 진행률 시나리오를 실제 JS로 확인했다. 테스트는 /tmp 임시 문서이며 생산 페이지에는 합성 평가 자료를 넣지 않았다. GPU 가속을 끈 별도 브라우저 프로필을 사용했다.

실행: `python3 tools/perception/report_camera_bbox_main.py --run <run> --dataset <dataset>`


## 2026-10-13 · ASMC 실험 목록 개편

[ASMC 실험 목록](http://localhost:8768/index.html)의 제목과 탐색 구조를 정리했다. 최근 본학습·현재 데이터 검수·이전 학습 평가·운영 모델 선정 근거를 상단에 연결하며, 기본은 원본 폴더별 작업 묶음이다. 분야·등록 날짜·검색·정렬·전체 페이지 보기와 페이지 나누기를 제공한다. 검색 조건은 URL에 보존한다.

기존 E 번호·최초 등록 시각·상세 주소는 유지한다. 등록 시각은 실험 시작 시각이 아니며, 과거 실패와 보류 기록도 펼쳐볼 수 있다. `build_experiment_gallery_hub.py`가 `templates/experiment_catalogue.html`과 함께 HTML·목록 JSON·링크 검사 보고서를 생성한다. v9 종료 갤러리의 누락된 상위 목차를 복구했으며 보관된 원본 갤러리는 수정하지 않았다. 학습 실행 설정과 가중치는 변경하지 않는다.

재생성: `python3 tools/perception/build_experiment_gallery_hub.py --artifacts ../artifacts`


## 2026-10-13 18:50 KST · v10 E2 차량 회귀 진단

학습은 중단하지 않았다. E2 selection212/conf0.40에서 car0/39, bus6/22, truck11/20; baseline은21/39,12/22,7/20. 초기화 직후 차량 TP와 예측 수는 baseline과 같고, E1/E2 car는 저장된 conf≥0.10에도 출력0이다. 이름 무관 일대일IoU≥0.5 대응에서 car는 truck11/bus6/motorcycle7/미대응15. 마스크와 겹치는 GT는 car2/39, bus0/22, truck1/20이므로 직접 마스킹만으로 설명되지 않는다. 학습 전후 동일 mask/input SHA다.

Train car11,449 양성 중11,400개가 Roboflow(99.6%), Capture49개. 최근 Capture는 bus3,984/7,185, truck5,373/7,214다. 과거 aiv5 car의 bus/truck 혼입은 알려져 있으나 현재 전체 혼입률은 미확정이다. 고정 source별 최대4개 car박스22개를 RGB 확인했으며 극소/가림은 확정하지 않았다. 혼입과 소스·외형 분포 차이, 초기 적응/손실 영향은 원인 후보로 구분한다. car 미출력을 초기 적응으로 자동 면제하지 않으며 예정 E4 점검에서 회복 여부를 확인한다.

[진단·학습 전후 갤러리](http://localhost:8768/camera-v10-vehicle-diagnostic-20261013-r1/index.html). 저장 예측 CPU 재집계만 수행했고 라벨·split·weights·frozen 학습 코드는 유지했다.


## 2026-10-13 · comp 차량 감소 원인과 지도 고정 드럼 확인

car 99.6% Roboflow의 직접적인 데이터 구성 원인을 추적했다. `resplit_group()`이 comp_sample 전체 날씨/주행을 한 계보로 묶고 solver가 selection으로 예약했으며 comp_sample_b만 train이다. v9 train comp_sample에는 car 양성3,500개가 있었다. 새4px/64px² 정책의 예약 selection comp_sample은3,715장/car양성2,155·ignore3,606개이고, 실제 채택 평가 패널은112장/car양성39개다. Capture train car는 comp_sample_b41장·49양성만 남았다. 데이터 부재가 아니라 누출 방지 분할에서 source×class train 보존을 충분히 검토하지 못한 편중이다. 양성 수만으로 분할 품질을 설명하지 않는다. 현재 데이터/학습을 변경하지 않았고, 예약 평가본을 몰래 train에 돌리지 않는다.

고정 드럼: train route_pp_30_20261012T032742_843 front001028/001050/001070의 톨게이트 전 오른쪽 드럼을 RGB/semantic/저장 grid로 확인했다. semantic standing_object이며 partial/known_class_background=false, 오른쪽 standing_object와 교차하는8px셀605/427/472 중 safe background는 모두0이다. 확인 표본은 배경 오검출 손실로 억제하지 않으나 양성 학습은 누락된다. 모든 소스나 reviewed_negative 전체에 대한 보장은 아니다. 지도 고정 여부와 무관하게 실제 traffic_drum 외형은 같은 목표 클래스 양성으로 보강하는 것이 일관적이다. standing_object 전체 변환은 금지한다. 일부 val/test 후보에는 고정 드럼 누락 보류가 이미 기록돼 있다. 상세 근거는 기존 차량 진단 갤러리의 추가 추적과 JSON을 참조한다.

## 2026-10-13 · 출처별 재감사와 고주로 지도 고정 드럼 보강

사용자 지시대로 v10 실행을 중단하거나 활성 데이터·split·frozen 학습 코드를 수정하지 않았다. E4 평가는 기존 실행에서 진행한다. 이번 결과는 데이터 구성 문제를 확인한 것이며 단독 회귀 원인으로 확정한 것은 아니다.

- Train car 11,449박스 중 Capture49/Roboflow11,400. RF 상위 소스는 aiv5 3,643, hlfma_3 3,463, jjj2,037, mino1,912. RF의 정확한 차종 에셋 커버리지는 미확인이다. car 클래스명을 서로 다른 차량 모델 수로 세지 않는다.
- 예약 comp_sample3,715장/car2,155양성의 native GT: Sedan1,362/SUV384/Wagon93/MPV73/Vehicle224/unknown19. 실제 API 모델명은 미확인이나 여러 차체 종류는 확인된다. comp_sample 전체를 selection 계보로 묶은 결과가 현재 Capture car 부족의 직접 원인이다.
- bus7,185 중 Capture3,995/RF3,190. Capture Universe2016 3,984개라 같은 에셋·배치 반복에 집중한다. truck7,214 중 Capture5,374/RF1,840; HD65/Porter2/Xcient P520/P540/Mixer의 5종5,373개를 확인했다. 최대 단일 소스 양성 비중은 bollard87.7%, traffic_drum60.6%로 다른 종류의 편중도 함께 남긴다.
- 소스 보존 분할 진단: comp_sample→train, comp_sample_b→selection, heldout고정, Capture차량별 train≥1,000 및 평가클래스별≥20은 selection dog 제약으로 불가능. dog≥10으로 완화한 **소표본 진단안만** 저장했다. train후보41,578장/car13,212개 중 Capture2,155, selection후보2,412장/dog10개. 미승인·미export이며 실제 평가 RGB 검수와 독립 dog 평가 보강이 필요하다. 현재 실행이 본 영상이 새 selection에 포함되므로 채택 시 공식 사전가중치에서 새 실행해야 한다. 새로운 무결점 heldout이라고 주장하지 않는다.

고정 드럼 라벨은 `data/roboflow/fixed-map-drums-reviewed-r2/`에 **immutable patch**로 저장했다. 기존 주석·분할·원본을 유지하고 ID로 병합하는 자료이며 단독 data.yaml은 제공하지 않는다. r1과 원본 제안도 보존했다.

- GPS구역·standing_object semantic·RGB색/형상으로 882장/5,179개 제안. standing_object 전체를 drum으로 변환하지 않는다. 고정 instance ID는 개체 구별에 쓸 수 없어 서로 이어진 영역은 인위적으로 나누지 않는다.
- 소스별 최대50장, 20소스/637crop을 Codex가 RGB로 확인했다(사람 검수 승인으로 표시하지 않음). ego를 동일하게 적용한 component 정합성을 재검사하고 연결·가림·모호한 색상 영역을 보류했다.
- 채택 **861장/3,888박스**, 현재train444장/+1,891; selection예약120장/+715; heldout예약297장/+1,282. 보류1,291제안. 원래 partial 이미지는 partial이며 모든 목표 클래스 완전성을 주장하지 않는다. 12px 제한은 이번 묶음 제안의 RGB 종류 확인 기준이고 학습4px/64px² 정책이나 기존 작은 GT를 변경하지 않는다.
- 잘못된 reviewed_negative6장을 RGB로 확인했다. 1장은 partial+양성으로 보강,5장은 `quarantine-removals.json`에 배경 채택 철회로 기록했다. 실행 중 데이터에는 미적용이다.
- 전체 reviewed_negative3,000장 semantic전수 검사: 기존target픽셀0,standing_object존재2,945장(16픽셀이상연결영역2,900장),SHA불일치0. 이는 대부분 기둥 등일 수 있어 오류2,945장으로 해석하지 않는다. 하지만 semantic에 일반actor종류가 없다는 근거만으로 full negative를 주장할 수 없다. 향후 배경 제안기는 standing_object가 있으면 RGB검수/partial경로로 넘기도록 보완했다. 기존 partial3표본의 드럼 safe background0 결과와 reviewed_negative누락6건을 구분한다.

CPU 신규6시험, v10기존8시험 통과. 실제 patch YOLO/sidecar/원본SHA/기존주석/분할계보/링크 재검증 통과. 추가 spatial-loss 회귀는 host 데이터venv에 torch가 없어 실행불가였으며 통과로 표시하지 않는다(이번 변경에 loss 수정 없음). 현재 학습의 GPU를 사용하지 않았다. 새 수집은 시작하지 않았다.

[출처·차종·드럼 보강 갤러리](http://localhost:8768/camera-source-coverage-v10-20261013-r1/index.html). 기존 ASMC 실험 목록에 날짜/E번호로 연결한다. 추가 수집은 우선 comp차량 재활용과 미사용자료 확인 후, 미확인 차체/차종·독립 dog 평가·희소 각도/거리/FOGGY에 한정한다. 모든 대회 차종이 이미 있다는 주장은 하지 않는다.

재현(상위 workspace에서, output은 새 revision을 사용):

```bash
PY=data/roboflow/.venv/bin/python
TOOLS=asmc-camera-data-audit/tools/perception
$PY $TOOLS/audit_camera_source_coverage.py --dataset data/dataset_versions/camera_objects_bbox_v10-resplit-r2 --output artifacts/camera-source-coverage-v10-20261013-r2
$PY $TOOLS/audit_camera_negative_semantic.py --dataset data/dataset_versions/camera_objects_bbox_v10-resplit-r2 --output artifacts/camera-source-coverage-v10-20261013-r2/negative-semantic-audit.json
$PY $TOOLS/prepare_fixed_map_drums.py --dataset data/dataset_versions/camera_objects_bbox_v10-resplit-r2 --output data/roboflow/fixed-map-drums-v2
# RGB검수 ledger를 고정한 뒤 export_fixed_map_drums.py의 --review로 전달한다.
```

## 2026-10-13 · v11 Capture 차량 복원 준비와 E4 판단

사용자가 Capture 차량 복원·데이터 재구성을 승인했고, 기존 학습을 교체할지 여부는 E4 후 판단하도록 했다. 실행 중 v10 데이터·가중치·설정은 수정하지 않았다. E4 자동 평가 뒤 기존 supervisor가 다음 epoch를 진행하는 상태를 유지했다. 새 학습 실행이나 PR 병합은 하지 않는다.

새 데이터는 `data/dataset_versions/camera_objects_bbox_v11-capture-restored-r2/`이며, `_SUCCESS`와 별도 verification/loader proof가 모두 있을 때만 구조 검증 완료다. r1은 오래된 `provenance.review_record`와 새 top-level review 형식 차이로 export가 중단됐고 `_FAILED.json`으로 보존한다. r2는 기존 evidence 또는 image SHA와 full16 scope가 확인된 경우에만 검수 기록을 연결한다. 단순히 complete 플래그만 승격하지 않는다.

- comp_sample 계보를 train으로 복원하고 comp_sample_b·Park·지정 AIM 계보를 selection으로 예약했다. heldout의 116장과 정답은 그대로 유지했다. 동일 run/계보를 잘라 train/val에 나누지 않는다.
- 기존 comp 중 양성 또는 유효한 배경 학습이 가능한 2,913장을 train에 복원한다. ignore-only는 빈 정답으로 사용하지 않는다.
- train 41,345장/양성154,611개/ignore234,208개. car는11,449→13,212개, 그중Capture49→2,155개. source별 RF/Capture 분포를 별도 보고한다.
- 새 train에 map-fixed traffic_drum 2,309박스를 병합했다. 기존 train 보강 외에 이번에 복원한 comp 계보의 보강분도 포함한다. 원본과 이전 patch는 유지한다.
- 이전 negative 중 standing_object가 있는 2,939장은 partial spatial-background-only로 전환한다. 이 영역을 drum으로 일괄 라벨링하지 않으며 기존 loss/loader의 safe semantic grid만 사용한다. 명확한 full negative55장, 잘못된negative5장제외/1장양성보강. 새로운 손실 수식은 추가하지 않는다.
- 16종 ID·4px/64px²(원본과960)·Ego mask·기존 양성/ignore·이미지당최대2회 반복은 유지한다. 모든유효train을 매epoch 최소1회 사용한다.
- selection: 기존complete100장 + 개별RGB검수complete94장 + partial11장 =205장. 추가 후보113장 중8장은 중복/불명확한 주석으로 보류한다. partial은 recall용이며 미대응검출을 FP로 세지 않는다. dog10개 및 일부차량의 표본/독립장면 부족은 해결된 것으로 표시하지 않는다.
- 새 selection47장은 현재v10 train에 포함됐던 자료다. 이어학습하면 이 자료는 개발진단용이며 독립평가로 주장할 수 없다. 공식 사전학습 가중치에서 새로 학습해야 새 selection의 train독립성을 확보할 수 있다. 기존heldout을 대신 반복선정용으로 사용하지 않는다.

E4는 기존selection212/conf0.40에서 car0/39, bus3/22, truck10/20, person99/116, motorcycle20/21이다. E2→E4 신규9종macroF1 0.42990→0.44616, fixed16macroF1 0.32786→0.33387. 차량회귀가 남아 있고 새selection의 노출문제가 있으므로 **새v11 + 공식사전가중치 시작을 권장**한다. 이 결과만으로 이어학습이 절대회복불가라고 주장하지 않는다. 실제교체는 사용자의 다음결정까지 보류한다.

관련 도구: `prepare_camera_capture_restoration.py`가 기존v10 export를 재사용해 구성하고 `verify_camera_capture_restoration.py`가 원본/heldout/예약계보/반복상한/배경전달을 독립검사한다. GPU를 숨긴 기존 frozen trainer의 spatial-loss CPU시험7개가 통과했다(optimizer update 없음). 준비후 같은 frozen loader로 새 데이터 표본과 batch연결을 확인한다. [복원·분포·E4 갤러리](http://localhost:8768/camera-capture-restoration-v11-20261013-r1/index.html).

### 2026-10-13 — v9·v10 회고: 새 학습 전 검토 (실행 설정 변경 없음)

[E329 보충 회고](http://localhost:8768/camera-capture-restoration-v11-20261013-r1/retrospective.html)에 저장 결과·입력 SHA·권고를 연결했다. 새 학습은 시작하지 않았고 현재 v10 실행은 유지했다. 아래 권고는 적용 완료 항목과 구분한다.

- **클래스 수 자체를 원인으로 확정하지 않는다.** v9도 동일 16종이며 car는 사전19/44→E8 38/44로 증가했다. 그러나 FP1→101, F1 59.38%→41.53%로 정밀도가 악화됐다. 새 클래스 추가 중 기존 표현이 약해질 가능성과 라벨/소스 편중을 구분한다. 초기화 직후 차량 결과는 사전 가중치와 일치했으므로 기존 클래스가 모두 무작위 초기화됐다는 설명은 배제한다.
- **전체 점수 상승이 회귀 해결을 뜻하지 않는다.** v9 신규9종 macro F1은 E1 33.11%→E8 66.27%지만 motorcycle은 E4 14/23→E8 7/23(사전20/23). v10 E4 person은 사전과 같은99/116이지만 FP19→469, F1 84.62%→28.95%. 다음 선정은 recall 외에 precision/F1·FP와 클래스별 혼동을 함께 확인해야 한다.
- v9의150장과 v10의212장은 평가 구성·크기 기준이 다르다. 두 총점으로 버전 우열 또는4px/64px²의 인과효과를 주장하지 않는다. v9 E8에 새 크기 기준만 적용한 재집계는 fixed16 F1 +0.83%p이고 motorcycle recall은 그대로다.
- 확인된 데이터 문제는 comp_sample의 평가 예약으로 Capture train car가49개만 남은 것과 map-fixed drum 미주석/잘못된 negative다. v11 복원 이후 Capture car2,155/13,212개(16.31%)이며, 모든 출처·차종 다양성이 충족된 것으로 표시하지 않는다.
- **새 버전도 partial 비중이 높다:** 39,839/41,345장(96.36%). 이는 배경 손실이 전혀 없다는 뜻이 아니다. safe semantic cell 및 확인된 클래스 박스 내부 감독이 이미 있고, 실제 학습 gradient의 출처·클래스 비중은 아직 측정하지 않았다. complete로 확정 가능한 검수 묶음과 안전 배경을 우선 보완하되 partial 전체를 complete로 올리지 않는다. standing_object를 일괄 background로 처리하지 않는다.
- 차량 분류를 소비 단계에서 vehicle로 묶어도, 학습에서 같은 외형을 car/bus/truck으로 상충되게 감독하는 효과가 없어지지는 않는다. 사용자 승인한 대량 활용은 유지하되, 새 학습 전 주요 직접 대응 소스와 Capture 복원 자료에서 명백한 차량 오분류·누락·복합박스에 한정한 점검을 권장한다. 전체 오분류율은 아직 측정하지 않았다.
- 유지 권고: 16종·공식 사전 가중치·3뷰 Ego mask·4px/64px² ignore·양성 우선·계보 분할·전체 데이터 활용. 실제 v9/v10의960/batch4/nbs16/AMP/AdamW lr0=.001/warmup1/약한 증강은 동일하며, 학습률이나 해상도가 주원인이라는 대조 증거가 없어 우선 유지한다. 복원 후에도 붕괴가 지속될 때만 낮은 LR 또는 단계적 freeze를 별도 한 변수 대조로 검토한다.
- 감시 권고(미구현): 초기화 및E1/2/4/이후2epoch마다 기존 클래스 TP/FP/precision/F1·신규 macro·기하 대응/이름 혼동·감독 셀/gradient 통계를 기록한다. 단일 초반 점검으로 즉시 탈락시키지 않되 지속적인 클래스 소실을 총점으로 면제하지 않는다. 새 selection의 complete GT dog10/bus14/truck12와 독립 장면 부족을 명시하고, heldout을 반복 선정용으로 전용하지 않는다.

공식 [Ultralytics fine-tuning 문서](https://docs.ultralytics.com/guides/finetuning-guide/)의 이름별 가중치 전이와 사전학습 활용을 참고하되 실제 고정8.4.150 실행 증거를 우선한다. [ICCV2017 incremental detection](https://arxiv.org/abs/1708.06977)은 망각 가능성과 지식 보존의 필요성을 뒷받침하지만, 기존 자료도 함께 쓰는 본 프로젝트의 개별 회귀 원인을 증명하지는 않는다. 이번 검토에서 distillation·새 손실·학습률 변경은 구현하지 않았다.

### 2026-10-13 — v11 학습 전 차량 라벨·확실한 배경 보완

복원 r2를 보존하고 `camera_objects_bbox_v11-preflight-r3`의 개별 검수, 이어 `camera_objects_bbox_v11-preflight-r4`의 상충 중복 정리를 수행했다. 최종 준비본은 **r4**, metadata SHA `54d46da76f15d74e3938f96dd2c578ec3bd4a076df985a562b125082e8a7abc6`다. 새 학습·기존 학습 설정 변경·운영 교체·PR 병합은 하지 않았다. `camera_objects_versions.json`의 prepared_candidate만 갱신하며 current_training/current_adopted는 유지한다.

- Train **41,345장·양성154,537개·ignore234,208개**. 양성 감소74개는 같은 물체에 중복된 학습 주석을 제외한 결과다. 원본 주석·좌표는 manifest에 남긴다. selection205/heldout116의 이미지·정답은 부모와 완전히 동일하다.
- 차량 후보216주석을 개별 검토해 car→bus19개, car→plastic_barrier1개를 수정했다. 후속 교차검사에서 JJJ73장의 서로 다른 클래스 IoU≥0.95 중복74쌍을 모두 시각 검토하고 올바른 박스 하나씩 유지했다. PE 방벽에 붙은 traffic_drum2개도 plastic_barrier로 수정했다. RF train의 해당 상충 쌍은74→0이다. 전체 미검수 라벨의 오류율 또는 중복이 모두 해결됐다는 주장은 하지 않는다.
- 배경 후보96장 중86장을 원본 RGB의16종 부재와 고정 semantic의 대상 픽셀 부재로 확인했다. 모호한 맵 구조물10장은 partial 유지. Capture 차량24장 중22장은 full16 주석 완전성을 확인하고 complete로 승격했으며, 미주석 맵 drum이 있는2장은 partial 유지했다. 표본 결과를 다른 이미지에 자동 전파하지 않았다.
- Train 감독 상태는 complete1,473 / reviewed_negative141 / partial39,731이다. partial은 기존 safe semantic grid·확인된 양성 감독을 유지한다. 새로운 background loss 수식·완화된 ignore 정책은 추가하지 않았다. 미검수 standing_object 영역을 일괄 배경으로 만들지 않는다.
- `prepare_camera_preflight_revision.py`는 원본 SHA·개별 annotation identity를 고정하고 train 변경만 허용한다. 중복 제외는 다른 positive keeper와 IoU≥0.95·검수된 목표 이름 일치를 요구한다. 독립 사본과 RGB symlink를 사용해 부모 파일을 수정하지 않는다.
- 다음 실행 설정 `config/experiments/camera_bbox_main_v11.json`은 공식 사전학습 재시작,16종/960/AMP/AdamW/기존 증강·Ego mask·4px64를 유지한다. E1/2/4 이후2epoch 간격 평가를 준비했다. 충분한 complete GT가 있는 기존 클래스의 precision5%p 또는 F1 2%p 초과 하락은 체크포인트 우선 선정을 보류한다. 이는 학습 자동중단과 별개이며 초반 한 번의 하락으로 학습을 종료하지 않는다. 기존 recall pause 정책은 별도로 남아 있다.
- complete selection dog10/bus14/truck12와 장면 다양성 한계는 남는다. 부분 GT에 FP를 부여하거나 heldout을 반복 선정에 사용하지 않는다. v10이 새 selection47장을 이미 본 이력이 있어 새 config는 원본 사전학습 시작만 허용한다.

[**E331 · 차량·배경 학습 전 준비 갤러리**](http://localhost:8768/camera-v11-preflight-20261013-r1/index.html)에 수정·유지·보류 사례와 원본/수정 검수 기록, 구조 검사, CPU loader 증거를 연결했다. 전수 시각 검수나 성능 개선 검증을 완료한 것은 아니다. 학습 개선 효과는 다음 실행에서 확인한다.

최종 검증: 관련 CPU 회귀19개, frozen 양 head spatial-loss CPU시험7개, r3 실제 loader4사례와 r4 loader2사례(2×3×960×960), 전수 YOLO/sidecar/hash/계보 검사 통과. GPU/optimizer update는0이다. 원본 RGB 전체 SHA 재계산은 하지 않았으며 변경한 이미지의 SHA와 기존 고정 provenance를 검증했다. 저장한 요약 증거는 `config/baselines/evidence/camera_v11_preflight/`다.

#### 밴/버스 경계 재검토

car→bus로 바꾼19주석의 원본 검수 패널을 재확인했다. 승객용 쏠라티 계열로 보이는 미니버스와 대형 coach로 판단해 수정은 유지한다. 정확한 MORAI API 에셋명·좌석 수는 확인하지 않았으므로 외형 추정과 확정 정보를 구분한다. [현대 공식 쏠라티](https://www.hyundai.com/kr/ko/c/products/bus/solati)의 승객용15/16인승 버스 분류는 이 경계의 참고이며 COCO가 모든 밴을 bus로 정한다는 근거는 아니다. 승용 MPV/일반 미니밴은 car를 유지하며 높은 차체·측창 유무만으로 bus 일괄 변환하지 않는다. 데이터 수정 없이 검토 기록 `artifacts/camera-v11-preflight-20261013-r1/van-bus-review.json`을 추가했다. 미검수 영상 전체에 동일 분류가 일관되게 적용됐다는 의미는 아니다.

### 2026-10-13 · v10 종료 → v11 새 본학습 시작

사용자의 명시적 요청으로 `asmc-camera-main-v10-r1`을 종료했다. 완전한 epoch5(`epoch4.pt`) 체크포인트와 마지막 정식 epoch4 평가를 보존했다. 종료 grace 이후 컨테이너 exit137이며 중간 작업을 새 학습에 사용하지 않는다. 종료 기록과 checkpoint SHA는 v10 run의 `operator-closeout-v11/result.json`에 있다.

새 실행은 `artifacts/camera-bbox-main-v11/run-20261013-r1/`, 컨테이너 `asmc-camera-main-v11-r1`이다. 데이터는 `camera_objects_bbox_v11-preflight-r4`, 초기 가중치는 공식 YOLO26l detect(`9fe3c544f2b19bebad7ea41e76d7ad3d88b7c2f10d11d24430c5311f6b32db26`)이며 v10 가중치/optimizer resume는 하지 않는다. 코드·설정은 실행별 복사본으로 고정했다. 원본 데이터/AIM은 읽기 전용, network none, 기존 고정 이미지·Ultralytics 환경을 사용한다.

- selection205장(complete194/partial11)만 평가한다. legacy 평가본이나 heldout116장은 중간 평가에 합치지 않는다. 평가 구성의 train SHA/pixel/계보 누출0을 확인했다. `prepare_evaluation`은 legacy가 생략되면 selection만 준비하도록 기존 경로를 확장했다.
- 960·batch4부터200batch 사전 점검, AMP/AdamW/lr0=.001/nbs16/40epoch schedule/첫24시간 예산. 점검 가중치는 버리고 본학습은 공식 원본에서 다시 시작한다. 모든41,345장 사용, epoch0 노출43,904장, 최대2회 반복.
- E1/2/4 이후2epoch 평가. `--continue-on-regression`으로 기존 사용자 의도대로 회귀만으로 학습을 자동 중단하지 않는다. 기존 클래스 precision/F1·recall 악화는 기록하고 좋은 후보 선정 조건은 유지한다. OOM/비유한 손실/기타 오류와 시간 예산 중단은 유지한다.
- 데이터 목록의 current_training만 v11로 갱신했고 current_adopted 및 운영 모델은 유지한다. PR 병합·운영 교체는 없다.

[**v11 실시간 학습 현황**](http://localhost:8768/camera-bbox-main-v11/run-20261013-r1/live.html)에서 진행률·손실·클래스별 지표를 확인한다. [실험 목록](http://localhost:8768/)의 최근 본학습에도 연결한다. 실제 점검/시작 증거는 실행 폴더의 `supervisor.json`, `smoke-b4/run.json`, `main/run.json`을 따른다.

시작 확인:200batch/batch4 점검 통과 후 원본 가중치에서 main1epoch 시작. 초기 본학습75batch 이상, 유한gradient와 오류0을 확인했다. AMP 초기 skip은 별도 카운터에 보존하며 이후 증가를 감시한다. 점검 epoch 추정3,036초(약51분)이며 실제 epoch 완료 전 추정이다. `launch-verification.json`에 원본 가중치 SHA·resume 없음·manifest SHA·초기 손실을 기록했다.

### 2026-10-14 · v11 E14 종료 후 loss·LR 검토와 별도 분기

사용자 요청에 따라 원본 v11은 완전한 E14까지 학습·평가한 뒤 보존한다. 기존 supervisor만 `SIGSTOP`으로 대기시켰고 `--stop-after-epoch 14`인 trainer는 계속 실행한다. `operator-closeout/_EVALUATED`는 체크포인트·평가 기록의 SHA 검증 뒤에만 생성한다. 이 표식만으로 시각 검토 완료를 뜻하지 않는다. 기존 frozen trainer/데이터/loss는 수정하지 않는다.

[loss·LR 및 실패 장면 검토](http://localhost:8768/camera-v11-epoch14-review-20261014-r1/index.html)에 실제 학습 CSV와 고정 selection205장의 그래프·GT/E8/E12/E14 비교를 연결한다. E14 평가 전에는 pending으로 표시한다. heldout116장은 사용하지 않는다.

- 원래 LR은 E8 .00082675 → E10 .00077725 → E12 .00072775이며 40epoch 선형 스케줄이다. 첫 epoch 이후 warmup은 종료돼 있다. 이 값이 부적절하다는 인과관계는 아직 확인하지 않았다.
- 실제 기준은 `E2ELoss`의 one-to-many/one-to-one 가중합이며, CSV의 loss items는 one-to-one 값이다. box gain7.5/cls.5/auxiliary1.5, 원래 target-score 정규화와 assignment 설정을 유지한다. partial/ignore BCE만 두 head에 동일하게 연결한다. 기존 가중치나 loss schedule이 전부 재초기화됐다는 근거는 없다.
- 기존 frozen 코드의 CPU loss 회귀17개 통과. 정상 양성 우선, ignore/partial 배경 제외, 확인된 semantic 배경·클래스 내부 감독을 검사했다. 모든 실제 라벨·semantic이 정확하다는 증명으로 확대하지 않는다.
- E8→E12에서 놓친 버스5개는 Park 소스의 가까운 대형 coach 장면이며, 트럭3개는 상자 뒤 적재 트럭의 연속 장면이다. 작은 GT만의 문제나 전부 car로 이름이 바뀐 현상은 아니다. 상자 여러 개를 하나로 묶는 장면도 있다. 같은 소스의 연속 장면·bus/truck 소표본 한계를 유지한다.

시각 검토 후 첫 분기는 E8의 학습 모델·AdamW 상태·EMA를 복원하고 LR만 낮춘다. E9 첫 LR .0002, E10 약 .0001938이며 기존 선형 감쇠 형태와 40epoch head 가중치 일정을 유지한다. `run_camera_lr_branch.py`는 명시적인 검토 gate, 체크포인트·데이터·설정 SHA와 E14 평가 완료를 요구한다. 원본 E8→E10의 optimizer/EMA/head 시작 상태 및 입력 순서를 대조하며, 같은 selection/conf0.40으로 분기 E9/E10을 평가한다. 정확한 augmentation RNG 연속성은 주장하지 않는다. 이 문단은 실행 계획이며 실제 시작/완료는 별도 `branch.json`에 따른다. 운영 모델·데이터·PR 병합에는 변화가 없다.

#### E14 평가 후 최신 결정 — 기존 설정 E16까지 관찰, LR 분기 보류

사용자가 일시 악화 뒤 회복 가능성을 고려해 기존 설정 유지도 검토하도록 요청했다. E14를 완료·평가했고, fixed16 F1은 E12 **47.85868% → E14 47.85836%**로 사실상 동일하다. person F1 39.60→42.64, bus33.33→38.10, box63.41→70.27은 개선됐지만 truck34.78→31.58, car44.44→43.24는 악화됐다. 이는 local minimum 탈출의 증거가 아니며 뚜렷한 전체 반등도 아니다.

한 번 더 같은 조건의 관찰 구간을 확보하기 위해 **E14 optimizer/EMA/scheduler를 복원해 E16까지만** 기존 설정으로 재개했다. 원본 PID1 supervisor는 계속 대기 상태이며 별도 bounded trainer와 `operator-closeout-e16/` 감시기가 E16에서 평가·정지한다. E14 기록은 `operator-closeout/`에 그대로 남는다. 두 감시 경로를 혼동하거나 PID1을 `SIGCONT`해 중복 실행하지 않는다. 데이터·loss·confidence·LR 감쇠 설정은 변경하지 않았다. LR 분기는 아직 시작하지 않았다.

같은 selection205/conf0.40에서 사전학습 가중치의 기존7종 F1은 person82.57%, dog86.96%, bicycle66.67%, car80.00%, motorcycle57.14%, bus75.00%, truck47.06%; 7종 macro70.77%다. complete GT만 F1에 사용하며 dog10/bus14/truck12개는 소표본이다. COCO 공식 벤치마크 수치가 아니다. 초기 가중치 SHA는 위 공식 가중치와 일치한다.

새 기록은 검토 갤러리의 `decision-at14.json`, `run-record-at14.json`, `pretrained-seven-class-baseline.json`, `failure-geometry.json`에 있다. 상자 뒤 적재 차량으로 보이는 대상의 주석이 빠졌을 가능성이 있는 Park1장도 `evaluation-review-followup.json`에 별도 표시했다. 동결 GT는 변경하지 않았으며 해당 이미지에서는 초기/E8/E12의 truck 예측이 없어 기존 FP를 변경하지 않는다. 이 관찰을 모든 평가 라벨의 오류로 확대하지 않는다.


### 2026-10-14 · v11 val 크기 정책·출처 전수 감사

사용자 요청으로 현재 고정 selection205장을 읽기 전용으로 감사했다. 도구는 `tools/perception/audit_camera_validation_scope.py`, 증거는 `artifacts/camera-v11-val-audit-20261014-r1/`에 보존했다. [전체 val 검수 갤러리](http://localhost:8768/camera-v11-val-audit-20261014-r1/index.html)는 출처·클래스·뷰 필터와 ignore 표시 전환을 제공한다. 학습·라벨·분할·고정 평가본·운영 설정은 변경하지 않았다.

- 원본 및 square960 letterbox에서 짧은 변4px 미만 또는 면적64px² 미만이면 ignore이며 경계값은 통과한다. Capture instance 가시 픽셀4개 미만도 ignore다. 거리 컷은 없다. frozen policy 함수로 양성359개를 재계산해 위반0, ignore800개, excluded2개를 확인했다. 실제 YOLO 라벨·평가기 GT 수·부모 주석·원본 SHA 불일치0, train과 SHA/픽셀해시/분할 그룹 중복0이다. heldout116장은 예측 없이 주석 크기만 검사해 양성445개 위반0을 확인했다. 이는 모든 이미지의 주석 완전성을 시각 검수했다는 뜻은 아니다.
- 전체 출처는 Capture158(77.07%), Roboflow36(17.56%), AIM11(5.37%)이다. Capture는 전부 `route_pp_30_20261011T161630_349` 한 run, Roboflow는 전부 `parkminseo/final-ftuy2` 한 프로젝트다. car22/bus16/truck14/box21 양성은 모두 이 Roboflow 소스다. F1에 쓰는 complete GT는 각각20/14/12/18개다.
- Capture158장 중138장은 같은 뷰의 ±5 Capture index 안에 다른 선정 이미지가 있다. 자전거19개는 actor1개, 드럼20개는 actor2개다. index 간격을 초로 해석하거나 기록된 correlation group을 독립 장면 수로 간주하지 않는다. 날씨195장, 터널 여부194장이 unknown이라 환경 균형도 입증되지 않았다.
- 현재 E14 결과 갤러리는 오류 수 상위32장(Capture30/Roboflow2), E8→E12 실패 비교는14장(Roboflow11/Capture3)이다. 대표성 있는 무작위 표본이 아니다. 이전 갤러리는 ignore 박스도 표시하므로 작은 박스가 보인다는 사실이 양성 기준 누락을 뜻하지 않는다. 8×8 또는4×16 박스는 기준을 통과하며 작은 예측 박스에도 별도 표시 컷은 없다.

결론: 크기 정책 누락보다 출처·actor·연속 장면 편중이 확인됐다. 현재 평가본은 동일 장면의 체크포인트 비교를 위해 보존하되, 다양한 대회 조건의 성능 또는 LR의 인과관계를 확정하는 근거로는 부족하다. 다음에는 학습에 사용되지 않은 출처/run/actor/장면 그룹에서 차량·상자와 다른 조건을 보강한 별도 평가본을 검수하고 동일 후보 체크포인트를 재평가한다. 이번 감사에서 신규 평가본 구성이나 데이터 재분할은 실행하지 않았다.


#### v11 val205 전체 최고/최신 체크포인트 시각 비교

사용자 요청으로 [전체 비교 갤러리](http://localhost:8768/camera-v11-val-checkpoints-20261014-r1/index.html)를 추가했다. 생성 시점의 최신 완성 체크포인트는 E14(`epoch13.pt`), 평가된 v11 체크포인트 중 fixed16 macro F1 최고는 E8(`epoch7.pt`)이다. 같은 selection205장·confidence0.40에서 각각47.85836%/52.56640%다. GPU FP16 저장 예측을 재사용했으며 CPU/GPU 재추론이나 학습 중단은 하지 않았다. 최신이라는 표시는 이 갤러리의 고정 생성 시점 기준이다.

`build_camera_checkpoint_comparison.py`는 frozen evaluator로 전205장 membership·영상 SHA·checkpoint SHA·mask·설정 일치와 F1 재집계를 확인한다. 최고는 이번 v11에서 평가된 epoch들 사이의 전체16종 macro 기준으로, 배포 수락 또는 회귀 없는 후보 선정과 다르다. 전205장에 GT/최고/최신615개 패널을 생성하고 별도 모델 보기·클래스/출처/뷰 필터·확대를 제공한다. 실제 추론과 같은 차체 mask를 적용하며 GT ignore는 그림에서 숨긴다. partial 영상의 미대응 예측을 FP로 표시하지 않는다. provenance.json과 comparison.json에 선택 근거·원본 예측 SHA·클래스/프레임 집계를 보존했다.


#### 기존 설정 학습 계속 — E16 자동 재개 연결

전체 val205의 E8/E14 결과를 확인한 사용자가 계속 학습을 승인했다. 데이터·loss·LR·optimizer·EMA·960/batch4를 유지한다. 진행 중인 E15→E16 trainer와 평가 감시기는 그대로 두고, `continue_after_epoch16.py`가 E16 평가 완료 marker/SHA·프로세스 종료·완전한 checkpoint를 확인한 후 frozen supervisor로 재진입한다. PID1은 계속 suspended이며 SIGCONT하지 않는다. 중복 controller는 flock으로 차단한다. 기존40epoch 일정·첫24시간 deadline·2epoch 평가를 유지하고 회귀 지표만으로 자동 정지하지 않는다. 오류·OOM·비유한 손실·시간 예산 정지는 유지한다.

대기 상태와 승인·검증 기록은 run의 `operator-continue-after-e16/`에 있다. E16 이후 실제 재개 여부는 이 폴더의 started.json/state.json과 main/run.json으로 확인한다. 이번 설정 시점에는 E15가 계속 실행 중이며 이후 재개를 이미 실행했다고 표시하지 않는다. LR 분기·운영 모델 교체는 하지 않았다.


#### 2026-10-14 · E20 학습 중 E19 전체 val CPU 진단

E16 평가 후 기존 supervisor로 정상 재개했고 E18 평가 후 E20까지 학습 중이다. 데이터·LR·loss·운영 모델을 변경하지 않았다. 요청 시점의 최신 완성 checkpoint E19(epoch18.pt)를 CPU2스레드·낮은 우선순위·FP32로 val205장 전수 추론했다(약99초). frozen 평가기·4/64 GT·confidence.40·전방1280/측방640·3뷰mask를 유지했다. 결과는 run의 operator-epoch019-cpu/에 있으며 정식 GPU FP16 epoch 기록에는 합치지 않았다.

[전체 E19 결과 및 E8 비교](http://localhost:8768/camera-v11-val-epoch19-20261014-r1/index.html?mode=latest): E8 GPU fixed16 F1 52.5664%, E18 GPU47.9745%, E19 CPU47.8470%. E19 complete GT 기준 bus5/14·truck3/12, box13/18·drum20/20·bollard21/21이다. 여러 중복 박스가 같은 위치에 그려져 시각적으로는 몇 개의 정상 박스만 보이는 사례가 있다. #86은 TP3/FP17인데20개 예측이 주로 같은 세 위치에 겹친다.

저장 예측의 동일 클래스 greedy 중복 억제 진단에서 E19 IoU>.9는 F1 66.31%, IoU>.7는73.48%(complete FP581→94)다. E8은 같은 .7 진단에서74.21%, E18은72.95%다. 원래 평가·예측을 보존하며 이 값을 정식 배포 성능 또는 기존 지표 수정으로 취급하지 않는다. 실제 inference/export head와 NMS 연결, 이웃 객체 억제·통합 지연은 아직 검증하지 않았다. checkpoint 저장 상태의 model/head end2end=False, one2one head 존재와 evaluator nms=False→설치 predictor의 end2end=True 연결을 확인했으므로 학습된 head와 배포 head 대응을 별도 확인해야 한다. 이 관찰만으로 원인이 확정되거나 학습을 중단해야 한다고 판단하지 않았다.

갤러리는 최고기록 E8 FP16/최신요청 E19 FP32 차이를 명시한다. 615개 GT/예측 패널과 headless Chrome 단독205장 렌더링을 확인했다. 보강 검증 전에는 운영 승격하지 않는다.


### 2026-10-14 · 과거 전체 출력 중복 재평가와 loss/LR 판단 갱신

사용자 요청으로 [과거 본학습·파일럿 재평가](http://localhost:8768/camera-post-nms-reevaluation-20261014-r1/index.html)를 추가했다. v8/v9/v10/v11·YOLO26/YOLOE 파일럿과 pretrained/initial 기준선을 포함한42개 저장 추론 묶음을 처리했다(E20 포함). 원래 중복을 FP로 센 것은 정상 평가이며 기존 evaluation.json/supervisor.json을 고치지 않았다. `same_class_nms_v1`: confidence.40, 같은 클래스의 IoU>.7만 낮은 confidence부터 억제(동점 원본 순서), 서로 다른 클래스는 유지한다. GT/ignore와 일대일 IoU>=.5 대응·partial 처리도 유지한다. 이 출력 후처리 정책을 적용한 별도 지표를 추가했으며 운영 backend/export는 변경하지 않았다.

각 원본 평가의 TP/FP 계산용 수치를 먼저 재현하고, 동일 evaluator/manifest SHA에 연결해 다시 집계했다. v8/v9/v10/v11 live.html은 원본/중복 억제 선택, 두 곡선·클래스 수치·후처리 기준선을 표시한다. 원본 점수를 보존하고 정식 epoch 지표만 live에 넣었다(E19 CPU는 별도 결과표). 45초 간격의 CPU 보고 감시기가 새 평가 파일을 읽어 최대8시간 자동 갱신한다. PID/상태는 `watch-process.json`, `watch-state.json`; 학습 supervisor와 별개다.

동일 v11 selection205에서 원본→NMS16 F1: E8 52.57→74.21%, E12 47.86→70.93%, E14 47.86→71.16%, E16 47.74→71.82%, E18 47.97→72.95%, E20 48.60→72.69%. E19 CPU는47.85→73.48%로 GPU와 정밀도를 구분한다. E20 complete FP562→101, E8은533→121이며 두 epoch에서 TP 감소0이다. 모든 과거 실행을 합치면 NMS로 TP가 줄어든 사례도42건(반복 실험·분할 포함)이 있으므로 무조건 정확도가 개선된다고 설명하지 않는다. `tp-loss-cases.json`에 보존한다. v11 E4 motorcycle1개도 감소했다.

E8→E20 NMS F1은 person77.48→78.18, motorcycle87.18→81.08, car75.00→64.00, bus66.67→41.67, truck58.33→31.58, box70.27→76.47, concrete_barrier44.44→75.86, stroller72.73→79.17이다. 전체는 E8보다1.52%p 낮고 E12 이후 일부 회복했지만 차량 하위 클래스 누락은 여전하다. v9 E8 NMS77.36%, v10 E4 44.36%는 서로 다른 GT manifest에서 얻은 값이므로 v11과 직접 우열을 매기지 않는다.

원인 점검: 실제 deepcopy된 backend에서 nms=False는 one2one/end2end=True/cv2제거를, nms=True는 one2many/end2end=False를 선택한다. checkpoint에 저장된 end2end=False만으로 잘못된 head가 평가됐다고 판단하면 안 된다. 양 head 각120개 tensor 중102개가 initial→E19에서 변경됐다. 같은3장면의 NMS-free 출력 수20/17/3과 one2many+NMS 출력3/3/3을 확인했다. nms=True는 head 자체도 바꾸므로 이번 post-NMS 재집계와 동등하다고 설명하지 않는다. `head-audit-cpu.json`에 설치 소스·출력 근거를 보존한다.

학습 data의 complete1473/partial39731/reviewed_negative141(총41345), known_class_background=True15809를 확인했다. partial96.1%에서 미확인·ignore 영역의 배경 BCE가 제한돼 중복·오검출 억제 신호가 약해질 수 있다는 가설은 남지만, semantic 안전 배경과 클래스 내부 배경 감독이 있으므로 배경 손실 전체가 꺼졌다고 설명하지 않는다. LR/가중치 배분을 원인으로 확정할 독립 대조는 없다. 화면에서 같은 위치에 겹친 박스와 다른 클래스 혼동·진짜 배경 FP를 구분한다. 학습은 계속하며 loss/LR·배포 모델·fusion은 바꾸지 않았다.

검증: NMS 단위시험5개, deterministic100박스 torchvision batched_nms 대조, 전42실험 원본 수치 재현, 실제 live Chrome 렌더링 통과. 부분 GT/ignore/TP 손실 추적 및 고정 source SHA를 유지한다. 저장된 후보(conf>=.1,max_det300)에 대한 사후 처리이므로 native 전체 후보에서 NMS를 하는 실험과 다르며, TensorRT/실시간 검증은 후속 작업이다.

## 2026-10-14 · E345/E346 · v11 E8/E22 배포 시험 인계

이 절이 이번 배포 시험의 시작점이다. 이전 v8~v11 학습·실패 원본은 보존한다.
운영 COCO7 기본값은 유지하며, 새 16종 모델은 별도의 실험 launch로 실행한다.

### 확정한 분류 계약

| 소비 그룹 | 원본 학습 클래스 |
|---|---|
| vehicle | car, bus, truck |
| dynamic_obstacle | person, dog, deer, bicycle, stroller |
| static_obstacle | motorcycle, bollard, box, concrete_barrier, steel_barricade, traffic_drum, plastic_barrier, traffic_cone |

사용자가 SportBike의 정적 특성과 자전거·유모차가 사람과 함께 움직일 수 있음을 확인했다.
이 그룹은 대회 ODD에서의 이동 가능성/사용 목적이며, 실제 운동 상태가 아니다.
CameraObjectBox.class_name에는 그룹, source_class_name에는 원본 16종 이름을 넣는다.
원본 confidence·source header·좌표·카메라 ID를 유지한다. 미등록 이름은 static으로
대체하지 않는다. 16종 가중치의 ID 순서가 다르면 로딩을 거부한다.

Main은 `camera_class_profile:=mission16_v1`에서만 새 정규화를 사용한다. BBox wire에는
원본 이름을 유지하며 Main이 같은 대응으로 그룹을 계산한다. `classification`과
`yolo_class`가 그룹/원본 이름을 각각 보존하고 `is_dynamic`·속도는 기존 LiDAR 추적
결과를 유지한다. 카메라 confidence는 원본 카메라 토픽에 남으며 confidence 필드가
없는 기존 fusion 메시지에 임의의 값을 덧붙이지 않는다.

planning_inputs의 최종 DetectedObject는 숫자형 vehicle/pedestrian/unknown만 지원한다.
공개 MD5를 유지하기 위해 vehicle은 vehicle 타입, 원본 person은 기존 pedestrian 타입,
나머지 두 obstacle 그룹은 일반 unknown 타입으로 전달하되 객체·기하·운동 상태를 버리지 않는다. 원본/그룹 문자열이 필요한
판제 검토는 `/lidar/tracked_objects_map` 및 validated tracked companion을 함께 사용한다.
unknown 숫자 타입을 '객체 없음' 또는 '정적'으로 해석하면 안 된다.

### 학습 종료와 후보 고정

승인된 구현 시작 시 E22 저장·평가는 끝났고 감독기가 다음 구간을 시작한 상태였다.
감독기 PID1422를 SIGSTOP하고 그 뒤 학습 worker PID2120을 SIGINT했다. 기존 PID1도
정지 상태를 유지한다. 후속 부분 epoch는 후보로 사용하지 않는다. 재개하려면 새 승인과
명시적 pinned checkpoint가 필요하며 두 감독기를 SIGCONT하지 않는다.

| 후보 | 파일 | SHA-256 |
|---|---|---|
| E8 | epoch7.pt | 6d5f4f0645b855bf66f0b0fa9d86b690d7659027e419d3b894d40c046ff89f70 |
| E22 | epoch21.pt | f4d7c82b92bc15926caf712c92c9f936cb497f8639c959a6fdfc165b278f4928 |

두 파일의 내부 epoch·EMA·optimizer를 확인했다. LR 일정은 원래40epoch 그대로다.
중단 전 상태와 결과는 `artifacts/camera-bbox-main-v11/run-20261013-r1/operator-closeout-e22/`에 보존한다.

### 같은 val205의 고정 조건 비교

Confidence0.40, 원본 클래스별 NMS IoU>0.70, 일대일 IoU>=0.5, 원래 positive/ignore와
partial 처리 유지. 그룹 후 NMS는 하지 않는다. 원본 raw 집계를 재현한 뒤 재집계했다.

| 후보 | 16종 macro F1 | 3그룹 macro F1 |
|---|---:|---:|
| E8 | 74.21% | 74.96% |
| E22 | 71.19% | 70.38% |

GT-only 우선 후보는 E8이다. 이 val은 반복 사용한 개발 평가본이며 독립 시험이 아니다.
표본이 적은 bus/truck과 소스 편중을 숨기지 않는다. E346 갤러리와 전체 JSON은
`artifacts/camera-deployment-e8-e22-20261014-r1/`에 있다.

### 실행 경로·동일성 검사

기존 batched backend에 opt-in `head_profile: mission16_one2one_nms`를 추가했다.
원본 체크포인트에서 predictor가 head를 선택하기 전에 `.fuse()`하면 one2one을
제거할 수 있어 이 경로에서는 사전 fuse를 하지 않는다. 설치 패키지를 수정하지 않았다.
`nms=False` one2one 출력에 명시적 same-class NMS를 적용한다. legacy one2many COCO7는 유지한다.

E8 engine은 TensorRT10.13.0.35 FP16 tactic, dynamic batch1~3으로 생성했다.
engine binding은 FP32이며 FP16 연산 engine과 입력 dtype을 혼동하지 않는다.
48장 PyTorch FP32 원본 대조에서99개 대응 IoU>=0.98·confidence 차이<=0.02를 통과했다.
confidence0.40 경계에서 미대응1개는 별도 기록했다. FP16 PyTorch 비교의25개 차이/실패도
보존하며 FP16 좌표 반올림과 NMS 선택 때문에 기존 저장 예측과 완전히 동일하다고 주장하지 않는다.
검증 파일은 `/data/runtime/camera-mission16-v1/engine-parity*.json`이다.

### 단계 상태와 다음 판단

- 완료: 학습 중단, E8/E22 고정, val205 비교 갤러리, 세 그룹 opt-in 구현, engine 생성.
- 완료: 카메라 detection CPU/ROS79시험, fusion class policy8시험, 격리 overlay 빌드.
- 완료: 두 후보×primary/shadow×2회 bag 비교(8회), 선정 E8 TensorRT×두 bag×2회 검증(4회).
- 미승격: MORAI 렌더링과 전체 ACTIVE 스택·실제 주행·30분 연속 수락.
- 후속: confidence/NMS 조정. 차량 자체 누락이 남을 때만 COCO·자료 보완/재학습 검토.

주 bag48.9초와 음영 bag24.8초 모두 전방·좌우·LiDAR를 포함한다. 시험은 독립11314
master, unique·50ms·200ms와 단일BBox 발행자를 유지한다. Main의 legacy control stub은
시험 토픽으로 remap한다. bag에는 독립적인 연속 객체 GT가 없어 전체 recall이나
최초 안정 검출의 정답 기준 점수를 만들지 않는다. 동일 stamp 결과 갤러리로 비교한다.
모든 rate/gap 실패를 보존하며 짧은 시작 공백도 자동 면제하지 않는다.

실행본은 기존 마운트를 바꾸지 않은 `/data/runtime/camera-mission16-v1` 격리 overlay다.
`asmc-ros-noetic`에서 nested/devel/setup.bash를 source하면 변경된 camera/Main을 사용한다.
`camera_mission16.launch`는 3뷰15Hz·전방1280·2+1·Ego mask ON이며 lane/resnet은 기본OFF다.
전체 스택에서는 기존 담당 launch로 해당 모델을 활성화하며 같은 노드를 중복 실행하지 않는다.
운영 COCO7 복구는 기존 workspace의 setup과 camera_coco7.launch를 사용한다.

### 최종 후보와 bag 결과 — 2026-10-14

실시간 주행 시험 후보로 **E8**을 선정한다. val205의 3그룹 macro F1이 E22보다
4.58%p 높고, 16종 macro F1도 3.02%p 높다. 두 bag은 독립 GT가 없어 정량 품질 순위를
추가하지 않는다. 일부 터널 상자는 E22만 검출하는 반례가 있어 갤러리에 보존했다.

버스·트럭의 동일 조건 비교는 아래와 같다. 완전 주석 GT는 각각14개·12개이며,
학습 전후의 이름 F1이다. 전체 차량 일반화 성능을 확정하는 표본으로 해석하지 않는다.

| 모델 | bus F1 | truck F1 |
|---|---:|---:|
| 공식 사전학습 YOLO26l | 78.26% | 47.06% |
| v11 E8 | 66.67% | 58.33% |
| v11 E22 | 28.57% | 35.29% |

v11의 트럭 최고값은 E6/E8 동률이다. 버스는 사전학습 대비 회귀가 남아 있다.
그룹화로 차량 자체의 누락을 해결했다고 주장하지 않는다.

TensorRT E8의 4회 bag 실행은 모두 기존 처리 기준을 통과했다. 아래는 **실행별
통계의 최소~최대**이며, p50의 평균이나 모든 실행을 합친 percentile이 아니다.
지연은 입력 수신→출력 wall 시간이다.

| 뷰 | 새 출력 Hz | p50 ms | p95 ms | p99 ms | 최대 공백 ms |
|---|---:|---:|---:|---:|---:|
| 전방 | 14.97 | 33.35–34.05 | 42.39–45.43 | 47.43–53.16 | 130.77–134.46 |
| 좌측 | 14.93–14.97 | 34.48–35.78 | 42.61–44.30 | 50.07–54.51 | 137.28–145.49 |
| 우측 | 14.81–14.91 | 33.63–35.05 | 41.21–44.78 | 50.81–54.42 | 137.26–173.27 |

완전한5초 창 최소는 전방/좌측14.8Hz, 우측14.6Hz다. 9.5Hz 최소 조건을 통과했으나
모든 창이 정확히15Hz였다고 표시하지 않는다. 전체 GPU peak1966/8188MiB(24.01%),
중복·역행 stamp0, 공개 제어 메시지0, BBox 단일 발행자를 확인했다.
TwinLite·ResNet은 로드된 STANDBY였으므로 전체 ACTIVE와 MORAI 렌더링 수락을 대신하지 않는다.

PyTorch 후보 비교에서는 E8 shadow-r1 최대255.5ms, E22 primary-r1/r2와 shadow-r2의
최대251.1–270.2ms 공백 실패가 있었다. 원본 report와 실패 판정을 유지한다.
TensorRT 전방 fusion은 primary matched531/533, dropped25/23, shadow matched193/192,
dropped0/0이다. drop·종료 시 pending을 감추지 않으며 서로 다른 처리 완료 프레임 수를
검출 품질 개선으로 해석하지 않는다. unique, offset±50ms, wait200ms는 유지했다.

최종 로컬 확인: detection79, runtime31, fusion policy8, planning group conversion2,
group evaluation3시험 통과. 최종 overlay 빌드·launch 파라미터 해석 통과.
공개 msg/srv와 MD5 정의는 변경하지 않았다. Hosted CI는 이번 로컬 시험과 별개이며
결제 제한을 해소하거나 CI 성공으로 표시하지 않았다. 기존 PR은 병합하지 않았다.

### 직접 실행 — 실험 프로필

기존 카메라 YOLO 생산자를 종료하고, `asmc-ros-noetic` 터미널에서 실행한다.
bridge·MORAI 연결은 기존 설정을 사용한다. 이 launch는 차량 제어를 실행하지 않는다.

```bash
docker exec -it asmc-ros-noetic bash
source /data/runtime/camera-mission16-v1/nested/devel/setup.bash
roslaunch asmc_camera camera_mission16.launch profile:=debug objects_conf_thres:=0.40
```

별도 터미널의 좌/전방/우 시각화:

```bash
docker exec -it asmc-ros-noetic bash
source /data/runtime/camera-mission16-v1/nested/devel/setup.bash
rosrun rqt_image_view rqt_image_view --clear-config /perception/camera/objects/debug/mosaic/compressed
```

Confidence는 launch 인자로 설정하며 변경 시험에서는 detector를 재시작한다. NMS0.70은
YAML의 `nms_iou`로 고정했다. 지금은 값을 바꿔 수락한 것이 아니므로 별도 저장 영상 대조가 필요하다.
기존 Main을 사용한다면 같은 overlay에서 `start_yolo:=false camera_class_profile:=mission16_v1`을
지정해야 한다. legacy Main을 그대로 둔 채 카메라 그룹만 바꾸지 않는다. adapter cutoff는
기본0.0으로 카메라에서 수락한 confidence를 재차0.7로 자르지 않는다.
숫자 판제 타입은 person의 pedestrian 호환을 유지하고, 그 외 장애물의 unknown 타입에도
기하와 속도를 보존한다. 원본16종과 세 그룹 문자열은 tracked companion에서 확인한다.

실제 주행에서 버스·차량 누락과 터널 상자·배경 오검출을 먼저 확인한 뒤
confidence/NMS 대조로 이어간다. 운영 기본값 승격, 전체 ACTIVE 스택 및30분 연속 시험은 남아 있다.

실험 목록: http://localhost:8768/ · 후보 비교: http://localhost:8768/camera-deployment-e8-e22-20261014-r1/
Bag 비교: http://localhost:8768/camera-deployment-e8-e22-20261014-r1/bag-review/
목록 재생성 시 신규 페이지 링크는 정상이며, 과거 보존 갤러리의 부모 링크2개 누락은 기존 증거로 기록한다.
