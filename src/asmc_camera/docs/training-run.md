# TwinLite Large 장기학습·현황 확인

2026-10-17 사용자 승인으로 v7 전체 train의 Large 추가학습을 시작했다.
현재 실행 ID는 `twinlite_large_v7_20261017_r1`이며, 운영 Medium v6 TensorRT를 자동 교체하지 않는다.

- [E366 학습 현황](http://127.0.0.1:8000/camera-seg-large-train-20261017-r1/index.html)
- [실제 학습 recipe](../../../config/experiments/twinlite_large_training_v7.json)
- [데이터 준비·split·원본 경로](training-data.md)
- [이번 실행 기록](../../../docs/records/camera/2026-10-17-twinlite-large-training.md)
- [Medium 운영본과 동일 val 비교](../../../docs/records/camera/2026-10-17-twinlite-medium-large-validation.md)
- [8epoch 종료·Large 운영 전환 계획](large-deployment-plan.md)

2026-10-17 후속 결정으로 **8epoch 전체 val 평가·저장 후 종료**를 예약한다.
LR 계산의40epoch 일정과24시간 절대 상한은 유지한다. 별도 종료 감시기가
`epoch008.pt`의 원자적 저장을 확인해 기존 STOP 절차를 요청하며, 실행 중인 코드는 변경하지 않는다.
`stop-policy.json`과 종료 결과에서 완료 epoch 및 목표 이후 추가 update 여부를 확인한다.
아래24시간 안내는 안전 상한이며 현재 실행의 목표는8epoch다.
감시 컨테이너는 `asmc-twinlite-epoch8-stop`이며 GPU를 사용하지 않는다. 감시 코드도
학습 snapshot 옆 `epoch8-stop-guard.py`로 고정했다. E366에서 `armed`는 예약 활성,
`stop_requested`는 정상 종료 요청, `stopped_at_target`은 목표 경계 종료 확인을 뜻한다.
`guard_error`/`needs_review`는 확인 필요 상태다. 원래 학습 컨테이너의 자동 재시작은 설정하지 않는다.

## 설정과 평가

전방544×960·좌우480×640(H×W), 실제 batch4, gradient accumulation3으로 유효 batch12를
사용한다. BatchNorm 통계의 batch는4다. AMP·AdamW·LR0.0002·weight decay0.0005·EMA를
사용하며,40epoch 고정 polynomial schedule(power1.5)을24시간 상한 안에서 실행한다.
Seed20261017과 입력 순서를 고정하고, 모든 train68,390장을 사용한다. 부족한 촬영 조건의
제한된 반복을 포함하면 epoch당91,969회·22,993 microbatch다.

BDD Large의 공통394개 tensor를 전이하고 `out_ll`14개 tensor만4-class로 새로 초기화했다.
전이 직후 전체 val 결과는 `initial-validation.json`이며, 원래 BDD의 이진 lane 평가와 다르다.
매 완전한 epoch마다 전체 val1,404장을 평가한다. SUNNY/FOGGY×11/13/15시의 두 head IoU를
같은 비중으로 평균한 `six_condition_macro_mean_head_iou`로 best를 선택한다.
흰차선·황색차선·정지선·drivable의 값과 pooled/per-view 지표도 보존한다.
Test14,556장은 학습 중 사용하지 않고 최종 후보 평가 단계에 남긴다.

일시적인 val 하락만으로 자동 종료하지 않는다. 유한하지 않은 loss, 반복되는 AMP skip 등
실제 오류는 원인과 마지막 복구 checkpoint를 남기고 중단한다. 데이터·정답·운영 모델은 변경하지 않는다.

## 실행과 저장

학습은 `asmc-twinlite-large-v7` 컨테이너에서 터미널과 독립적으로 실행한다.
원본 RGB·준비 데이터·코드는 read-only mount이며, 출력은 E드라이브에 저장한다.
실행 당시 코드는 공통 data의 `runtime/twinlite-large-training-20261017-r1/code/`에 고정했다.
스냅샷 전체 SHA는 옆의 `snapshot.json`에 있다. 이후 작업 checkout의 수정이 실행을 바꾸지 않는다.

```text
E:\asmc_data\twinlite\training_runs\twinlite_large_v7_20261017_r1\
├── limit.json                 # 최초 시작·절대 종료 시각
├── config.json                # 실제 인자·설치 버전·모델/데이터/코드 SHA
├── initial.pt                 # 전이 직후, optimizer update 전
├── initial-validation.json
├── latest-progress.pt         # 부분 epoch 복구용, next_batch 명시
├── epoch001.pt ...            # 완전한 epoch별 모델·EMA·optimizer·scheduler
├── latest.pt
├── best_mean.pt
├── best_drivable.pt
├── best_road_marking.pt
├── history.json
└── result.json                # 종료 시에 생성
```

첫 optimizer update 뒤와1,500개 microbatch마다 복구 checkpoint를 원자적으로 교체한다.
부분 epoch는 best 선정에 쓰지 않는다. 모델·EMA·optimizer·scheduler·AMP scaler·RNG·
loss 누적·다음 batch 위치를 함께 저장하고 같은 data SHA/인자에서 재개한다.
Accumulation 중간이 아닌 optimizer 경계에서 저장하며 남은 데이터만 읽는다.

이번 종료 상한은 **2026-10-18 17:55:14 KST**다. 재개해도 `limit.json`의 절대 종료 시각을
연장하지 않는다. 다음 완전한 epoch를 넣을 시간이 부족하면 경계에서 저장하고 종료한다.
시간 상한이나 명시적 정지 요청이 epoch 중 발생하면 부분 checkpoint로 보존한다.

## 직접 확인·정지·재개

```bash
docker logs --tail 20 -f asmc-twinlite-large-v7
docker stats --no-stream asmc-twinlite-large-v7
# 정지할 때: 다음 optimizer 경계에서 복구 저장 후 종료
docker exec asmc-twinlite-large-v7 touch /runs/twinlite_large_v7_20261017_r1/STOP
```

재개는 같은 frozen code·이미지·mount에서 기존 인자에 다음 옵션을 추가해 새 컨테이너로 실행한다.
정지 요청 파일이 있으면 먼저 제거한다. 원래 신규 실행 명령의 `docker start`는 resume 인자를
추가하지 못하므로 사용하지 않는다.

```bash
python3 tools/perception/run_twinlite_training.py \
  --data-root /prepared --dataset-version twinlite_morai_v7_large_capture_r2 \
  --init-checkpoint /work/src/asmc_camera/models/lane_segmentation/twinlite-large-bdd100k.pt \
  --twinlite-root /opt/baselines/TwinLiteNetPlus \
  --output /runs/twinlite_large_v7_20261017_r1 --monitor /monitor \
  --resume /runs/twinlite_large_v7_20261017_r1/latest-progress.pt \
  --epochs 40 --max-hours 24 --batch-size 4 --accumulate 3 --num-workers 2 \
  --checkpoint-batches 1500 --lr 0.0002 --weight-decay 0.0005 --seed 20261017
```

`/work`는 frozen code, `/prepared`는 `E:\asmc_data\twinlite`, `/capture_raw`는
`E:\morai_dataset\SensorData`, `/runs`는 E드라이브의 `training_runs`를 가리킨다.
원본과 `/work`는 read-only, `/runs`와 `/monitor`는 쓰기 가능으로 연결한다.
`ASMC_TWINLITE_SOURCE_ROOTS`는 `{"capture_raw":"/capture_raw"}`로 지정한다.

## 모니터링

E366은5초 간격으로 진행률·경과 시간·epoch 내 남은 시간·loss·LR·optimizer update/AMP skip·
GPU 예약 메모리를 표시한다. Loss와 전체 val의 조건 균형 IoU를 따로 그리고,
계열 표시 전환과 점의 hover 값을 제공한다. 고정18개 사례는 초기 상태와 각epoch 후에
같은 RGB/GT/추론으로 갱신한다.90초 이상 갱신이 없으면 입력·프로세스 확인이 필요하다고 표시한다.
페이지는 `asmc-experiment-gallery`의 localhost8000으로 제공한다. Rqt나 추론 노드를 추가하지 않는다.
