# TwinLite Large 운영 전환 계획

2026-10-17 사용자 결정: 현재 Large 학습을 8epoch까지 평가·저장한 뒤 종료하고,
검증된 Large를 기본 운영 모델로 전환한다. 2026-10-18 Large v7 E8의 품질·TensorRT·음영 정차12회 및10분 시험을 완료했다.
FOGGY10분은 통과했으나 SUNNY1회의 source freshness 저하3건으로 기본 전환을 보류했다.
후속 사용자 결정으로 알려진 제한을 보존한 채 Large E8을 기본으로 전환했다.
Medium v6 TensorRT는 명시적 복구 설정으로 남긴다. 자동 수락 실패를 통과로 바꾸지 않는다.
2026-10-18 학습은 완전한8epoch에서 정상 종료했고 이후 optimizer update는0이다.
선정 지표 최고점도8epoch(조건 균형 두 head IoU91.0968%)이므로 후보를E8로 고정했다.
원본 SHA는 `4029a2892966d5f1b3e0e2991211214395935015e5062ec9e497b4ff70c142c0`이다.
현재 전환 상태는 [E8 배포 검증 기록](../../../docs/records/camera/2026-10-18-twinlite-large-deployment.md)을 따른다.
아래 절차 자체는 기본 모델 교체 완료를 의미하지 않는다.

## 1. 학습 종료와 후보 고정

- 실행 `twinlite_large_v7_20261017_r1`을 유지한다. 기존 40epoch polynomial LR 일정,
  optimizer, batch, 데이터와 24시간 절대 상한은 변경하지 않는다.
- 외부 종료 감시기가 원자적으로 완성된 `epoch008.pt`를 확인하면 기존 `STOP` 요청을
  생성한다. 이 파일은 전체 val 평가 이후에만 생성되며, 학습기는 latest/best/history 저장을
  마친 뒤 기존 정상 종료 절차를 수행한다. 실행 중인 frozen 학습 코드는 변경하지 않는다.
- `stop-policy.json`에 감시 상태와 완료 epoch, 목표 이후 optimizer update 수를 기록한다.
  정확한 경계 종료 여부는 결과로 확인한다. 예외적으로 다음 epoch의 update가 발생해도
  부분 checkpoint는 후보에 포함하지 않고 완전한 `epoch008.pt`를 보존한다.
- 종료 후 worker 종료·GPU 해제·체크포인트 내부 epoch/EMA/optimizer/scheduler 및 SHA를 확인한다.
  마지막 고정 사례 생성이 STOP으로 중단되었다면 완료된 checkpoint로 오프라인 재생성한다.
- 매 epoch의 val1,404 결과를 비교해 1–8epoch 중 가장 좋은 완료 EMA와 8epoch를 최종 대조한다.
  최종 최고는8epoch다. 마지막 epoch라는 이유로 자동 선정한 것은 아니다.

## 2. 품질 판정

- 같은 val·해상도·전처리·valid mask에서 Medium v6, Large 최고점, Large8을 비교한다.
  이미 수행한 Medium 및 Large2 CPU FP32 전체 val 결과는 재사용한다.
- 기존 선택 지표인 SUNNY/FOGGY×11/13/15시 균형 평균 IoU를 유지한다. Drivable과
  흰차선·황색차선·정지선, 뷰·조건별 값, 얇은 차선·터널·근거리 실패를 함께 확인한다.
- Large2의 pooled 개선과 별개로 전방 황색 차선은 Medium85.01%→Large83.28%였다.
  이 회귀의 실제 위치와 family 영향, sample support를 확인한 뒤 후보를 확정한다.
- 장애물의 실제 semantic 실루엣은 drivable=0의 유효 음성 정답이다. BBox 전체를 지우거나
  장애물 픽셀을 ignore하지 않는다. 정지선·주차칸 포함 기존 semantic 정책도 유지한다.
- 후보를 val에서 고정한 뒤 test14,556장에서 Medium과 선정 Large를 비교한다.
  17시4,098장은 별도 일반화 결과로 분리한다. Test 결과를 보고 threshold/epoch를 다시
  고르지 않는다. 실패 시 개선 작업과 평가 사용 이력을 기록하고 전환을 보류한다.

## 3. Large TensorRT 배포본

- 학습용 `twinlite-managed-checkpoint-1.0.0`을 운영 지원 schema로 명시적으로 패키징한다.
  선정 EMA tensor·buffer·task spec·Large config·upstream·원본 epoch/SHA를 유지하고
  optimizer·중복 학습 상태는 배포본에서 제외한다. 원본 학습 파일은 보존한다.
- 축소 전후 모든 tensor와 동일 입력 출력을 대조한다. 경로만 바꾸어 기존 Medium 로더에
  넣지 않는다. `prepare_twinlite_tensorrt.py`의 Medium 기본값과 파일명을 일반화하고
  `model_config=large`를 exporter·validator·runtime에 일치시킨다.
- GPU 학습 종료 후 현재 지원 환경에서 FP16 engine을 만든다.
  전방 B1:1×3×544×960, 좌우 B2:2×3×480×640이며 측방 B1도 지원한다.
  Output은 drivable2 + road_marking4 및 현재의 축약 확률/class/confidence 계약을 유지한다.
- PyTorch→ONNX→TensorRT 유한값·head·shape·class 순서를 검증한다.
  확률 차이 p99≤0.02, class ID 일치≥99.5%, 충분한 GT의 클래스/뷰별 IoU 하락≤0.5%p를
  기준으로 하고 임계값 주변 및 얇은 차선 차이는 갤러리로 확인한다.
- 패키지 `models/lane_segmentation/`에 Large 배포 checkpoint·전방/측방 engine·전용
  manifest를 추가한다. Medium 파일·manifest·복구 프로필을 유지하고 SHA/환경 불일치에서
  명시적으로 실패한다. 자동 PyTorch fallback은 사용하지 않는다.

## 4. Family 및 통합 실행

- Geometry·보정·BEV 미터 격자·family 피팅/유효성·freshness350ms를 고정한다.
  같은 RGB·pose에서 model-grid→투영→family까지 재생해 역할·기하·observed/inferred·
  유효 공급을 비교한다. 모델 출력이 바뀌므로 family의 숫자 완전 일치를 요구하지 않는다.
- YOLO E8 및 ResNet과 함께 목표15/15/10Hz로 측정한다. 현재의 세 모델 상시 ACTIVE 기본
  정책을 유지하고, lane 소비 품질은 음영 ODD에서 판단한다.
- 저장 영상으로 먼저 점검한 뒤 MORAI SUNNY/FOGGY13시 각60초×3회 교차 비교한다.
  최종 구성은 FOGGY13시·전체 GUI·Main preview·가능한 기존 LiDAR 부하에서10분 확인한다.
  사용자 제어를 방해하지 않고 시험 rqt/ROS 생산자를 한 세트만 실행·정리한다.
- 수락: 완전한5초 창 YOLO/TwinLite/유효 family≥10Hz, ResNet≥9.5Hz,
  모델 지연 p95≤150ms·p99≤200ms, 모델/family 발행·수신 공백≤250ms,
  음영 정상 ACTIVE readiness 저하0, VRAM≤90%, OOM·중복/역행 stamp·큐 누적0.
  15Hz 목표 달성은 별도로 기록한다. 기존에도 남았던 family 공백은 면제하지 않고
  Medium 대비 변화와 전체 수락 여부를 구분한다.

## 5. 기본 전환과 인계

- 품질·family·부하 검증을 통과한 Large를 `lane_runtime.yaml` 정본으로 연결한다.
  PyTorch 복구는 선정 Large용으로 맞추고 Medium v6 TensorRT 복구 경로를 별도로 둔다.
- `camera.launch`·`view.launch` 사용법, 공개 메시지·토픽·모드 계약, YOLO/ResNet은 유지한다.
  Family 소비가 필요한 구간의 판단과 정책은 계속 판제 책임이다.
- 직접 주행 시각화 명령, Medium 복구 명령, E번호 결과/실패 갤러리와 실행 계약을 갱신한다.
  사용자 주행 확인 후 PR을 정리한다. 기본 전환 결과는 위 배포 기록을 따르며 이번 작업에서 PR은 병합하지 않는다.
- 수락 실패 시 Medium 운영을 유지하고 Large의 문제를 기록한다. 추가 수집·재학습은
  실패 원인을 확인한 별도 작업으로 결정한다.

관련: [학습 실행](training-run.md),
[Medium/Large 동일 val 실측](../../../docs/records/camera/2026-10-17-twinlite-medium-large-validation.md),
[E366 현황](http://127.0.0.1:8000/camera-seg-large-train-20261017-r1/index.html).
