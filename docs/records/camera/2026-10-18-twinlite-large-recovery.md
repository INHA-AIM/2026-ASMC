# TwinLite Large 원본 경로 복구·FOGGY15 구성 확인

## 중단 상태와 복구 기준

`twinlite_large_v7_20261017_r1`은6epoch를 완료한 뒤7epoch 도중 RGB 읽기 실패로 종료했다.
OOM이나 loss 오류가 아니라 `/capture_raw/CAMERA_1/capture_pivot6_20261012_r1_000001_Intensity.png`
누락이다. 원래 경로에서 전방 RGB24,653장이 보이지 않았으며, 동일 이름의 파일은
`D:\morai dataset\shahn03\SensorData\CAMERA_1`에 모두 있었다. 중단 파일의 SHA는
원래 manifest의 `9f194d8fb9a0fbf9508ab795d8d2b5888c62990bf577e5942792be2ab70aff84`와 같았다.

사용자가 E드라이브로 다시 복사하겠다고 하여 원래 root를 유지한다. 복원 완료와 SHA를
검증하기 전에는 재개하지 않는다. RGB·정답·split을 제외하거나 새로 섞어 오류를 우회하지 않는다.
원본 경로는 `E:\morai_dataset\SensorData`, 준비본은
`E:\asmc_data\twinlite\dataset_versions\twinlite_morai_v7_large_capture_r2`다.

`latest-progress.pt` CPU 로딩에서7epoch partial, 다음 batch15000, 완료6epoch,
optimizer update50969, EMA update50969, optimizer state300개, scheduler last_epoch6을 확인했다.
오류 직전까지의 약579개 batch는 checkpoint 이후였으므로 다시 처리해야 한다.
재개 시 원래40epoch LR 일정·batch4·누적3·seed·24시간 절대 상한과8epoch 종료 목표를 유지한다.
중단된 worker와 monitor의 오류·로그는 E366의 `recovery-20261018-r1/`에 보존한다.
재개 여부는 해당 폴더의 `recovery-status.json`과 E366 현재 상태를 따른다.

## 복원 검증과 실제 재개

사용자 복사 후 전방 전체28,106장의 SHA(22,464,615,669bytes)를 원래 manifest와
대조해 불일치0을 확인했다. Dataset metadata와 train/val/test manifest SHA도 모두
고정 값과 같았다. 중단 샘플을 frozen 학습 로더로 CPU에서 다시 읽어3×544×960 RGB,
두 head 정답544×960과 유효518,400픽셀을 확인했으며 optimizer update는 하지 않았다.

2026-10-18 00:23KST경 기존 `latest-progress.pt`에서 재개했다. 복구 checkpoint SHA는
`8d7a929f00a88c8740e4b159f1def479fe13db52cc77efb3cd928a384fea5d4e`다.
00:25KST 확인에서7epoch·batch15515·optimizer update51140으로 진행했으며,
loss0.10158·LR0.0001567323·누적 AMP skip21로 오류가 없었다. 실패한 컨테이너는
`asmc-twinlite-large-v7-failed-20261018-r1`, 현재 재개 컨테이너는 기존 이름
`asmc-twinlite-large-v7`이다. 종료 감시기 `asmc-twinlite-epoch8-stop`도 다시 armed 상태다.
경로를 D로 바꾸거나 데이터를 제외하지 않았다. 기존 절대 종료 시각도 유지한다.

## 실제 train 구성

2026-10-18 manifest 재집계 결과다. 실제68,390장 모두 매 epoch 한 번 이상 사용하며,
확정된 여섯 조건만 최대2회 반복해 부족분을 보충한다. 반복 후91,969회다.

| 조건 | 이미지 | 이미지 비율 | Epoch 노출 | 노출 비율 |
|---|---:|---:|---:|---:|
| SUNNY11 |6,534|9.55%|11,784|12.81%|
| SUNNY13 |6,663|9.74%|11,784|12.81%|
| SUNNY15 |11,784|17.23%|11,784|12.81%|
| FOGGY11 |6,439|9.42%|11,784|12.81%|
| FOGGY13 |6,486|9.48%|11,784|12.81%|
| FOGGY15 |2,565|3.75%|5,130|5.58%|
| SUNNY·시간 미확인 |8,310|12.15%|8,310|9.04%|
| 날씨·시간 미확인 |19,609|28.67%|19,609|21.32%|

FOGGY15는 `route_pp_30_20261012T010508_040` 한 run이며 세 뷰의 반복 관측이다.
원본 수와 독립 장면 수는 다르다. 최대2회 정책 때문에 FOGGY15 노출은 다른 확정 조건의
약43.5%에 그친다. 조건이 완전히 균등하다고 설명하지 않는다. 미확인40.82%를
임의로 날씨/시간에 배분하지 않는다. 확정17시 자료는 기존 합의대로 test에 남아 있다.

## FOGGY15 노면 표시 IoU

최신 완료6epoch의 수치다.

| 클래스 | FOGGY15 IoU |
|---|---:|
| 흰차선 |83.74%|
| 황색차선 |89.22%|
| 정지선 |0.81%|
| 세 클래스 평균 |57.93%|

해당 val은432장이지만 정지선 GT 총444픽셀, 양성115장으로 양성 이미지당 평균3.86픽셀이다.
Packed target115장을 직접 다시 읽으면87장(75.7%)이1–3픽셀이고, 최대도47픽셀이다.
가장 많은 표본의 RGB/GT 위치를 대조하면 원거리 쪽에 해당했다. 모든 픽셀의 실제 미터
거리를 측정한 결과는 아니지만 극소 원거리 support가 주된 평가 제약이라는 근거다.
반면 FOGGY15 train은 정지선509,965픽셀·양성1,248장이다. train에서 정지선을 아예 빼서
생긴 결과가 아니다. 흰/황 차선의 단순 평균은86.48%이지만 이는 원인 설명용 보조값이며,
현재3클래스 지표를 대체하거나 정지선의 실패를 면제하지 않는다.

FOGGY15 train의 적은 관측량과 단일 run 편중은 확인되었으나, 이것만으로 낮은 IoU의
인과관계를 확정하지 않는다. 평가 정지선의 극소 support와 train/val 장면 차이를 함께
검토해야 한다. Family는 흰/황 차선 근거를 사용하므로 이 평균을 family 성능으로 읽지 않는다.
이번 재개에서는 데이터·손실·비율·평가 기준을 바꾸지 않는다.

6epoch의 전체 조건 균형 두 head 평균은90.7472%로, 이전2epoch 최고90.6459%를 넘었다.
여전히8epoch 전체 평가까지 확인하고 완료 checkpoint 중 후보를 고른다.

관련: [학습 실행·재개](../../../src/asmc_camera/docs/training-run.md),
[E366 실시간 현황](http://127.0.0.1:8000/camera-seg-large-train-20261017-r1/index.html).
