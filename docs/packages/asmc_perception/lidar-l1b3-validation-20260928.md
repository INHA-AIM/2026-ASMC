# L1-B3: YOLO 도착 지연과 터널 보호 매칭 격리 평가

**상태: 과거 v1 기록 보존, L1-B3-G 정정 대상, 운영 연결 보류.** 원본 LiDAR·bridge·판제,
ROS topic/MD5, HD맵, 모델 weight는 변경하지 않았다.
[기계 판독 기준](../../../config/baselines/lidar_l1b3_20260928.yaml)과
[재실행 방법](../../../tools/lidar/camera_sync/README.md)이 정본이다.
손재호의 `6435af1`에서 `TunnelFilterRos.cpp`는 YOLO 결과를
스캔 처리 시 즉시 최근접 stamp로 선택하고 50ms 이내만 인정한다.
200ms까지 기다리는 객체 클래스 퓨전과는 별도 경로다.

## 2026-09-28 평가기 검수 정정

아래 표와 SHA는 `f419836` 당시 산출물을 보존한 기록이다. 현재 평가기는
FIFO 결정 순서를 직접 검증하며, 기존 Parser와 같이 LiDAR stamp를 밀리초로
절삭한 별도 matching stamp를 사용한다. 원본 ns는 그대로 보존한다.
양수 YOLO stamp의 중복/역행은 원본 코드처럼 수용하면서 진단하며,
동일 offset이면 timestamp 정렬상 앞쪽 frame을 선택한다.
정정 재생에서는 즉시 1,149, 25ms 1,453(+304), boxes 82, empty 1,371,
timeout 1,109로 바뀌었다. 과거 표의 수치를 새 결과로 덮어쓰지 않는다.

이전 스케줄러는 최종 정렬로 FIFO 위반을 숨길 수 있었다. 실제 bag의 25ms
비교에서는 scan 간격이 충분하여 해당 역전이 확인되지 않았지만, burst 합성
입력에서 재현되므로 이전 일반적인 FIFO 수락 주장은 철회한다.

`sweep.q[0]`은 **기준 지면 평면 ±0.15m / 30m 이내의 잔여점**이다.
기존 평가기가 이를 `wall_residual`로 잘못 이름 붙였다. 아래의 19/0점은
벽 잔여점 감소 근거가 아니며, 벽 제거 효과 해석을 철회한다. 참고 지표 자체와
원본 아티팩트는 보존한다. 실제 필터 비교·AIM 자료 적합성은 L1-B3-G에서
별도로 기록한다.

## 입력·재현성

동일한 26r1 정차 bag 1개(2,562 scan, `/YOLO/BBox` 3,197개)를
bag **수신 순서**대로 두 번 읽었다. 원본 메시지 SHA, 각 scan의
`PointCloud2` 점 수와 frame, 관측/수신 정수 ns를 기록했다.
장면은 시작점, 고주로 A423, 음영 A126 진입/중간, 복귀점의 5개
정차 위치이며 텔레포트 전후의 transition 225 scan은 별도다.
source, bag, 기존 130ms 참고 sweep 및 모든 산출물 SHA는
manifest로 고정했다. 저장한 두 실행의 `decisions.jsonl`,
`summary.json`, `manifest.json`은 각각 byte-for-byte 일치한다.
미래 도착 YOLO 선택은 0건이다.

후보는 scan이 도착했을 때 유효한 box 배열(0개도 **유효 빈 검출**)이
있으면 즉시 결정한다. 없으면 최대 25ms 기다리며, 먼저 도착한
50ms 이내 stamp 결과로 결정한다. 대기열은 2 scan, scan 순서를
유지한다. timeout·out-of-window·잘못된 stamp·OFF를 빈 검출로
바꾸지 않는다. 원본 영상의 촬영 시각은 bag에 따로 없으므로 여기서
말하는 YOLO `header.stamp`는 원본 이미지의 bridge 수신 시각이다.
광학 노출 시각으로 승격하지 않는다.

| 장면 | scan | 즉시 매칭 | 25ms 후보 매칭 | 추가 매칭 |
|---|---:|---:|---:|---:|
| 시작 | 475 | 260 | 310 | 50 |
| 고주로 A423 | 467 | 203 | 267 | 64 |
| 음영 A126 진입 | 463 | 183 | 241 | 58 |
| 음영 A126 중간 | 461 | 188 | 229 | 41 |
| 복귀 | 471 | 208 | 268 | 60 |
| transition | 225 | 106 | 137 | 31 |
| **전체** | **2,562** | **1,148** | **1,452** | **304** |

후보가 매칭한 1,452개 중 box가 있는 결과는 82개,
유효하지만 **box가 없는 결과는 1,370개**다. 두 음영 정차
장면의 추가 매칭 99개는 모두 유효 빈 검출이다. timeout
1,110개는 그 결정 시점까지 도착한 유효 YOLO stamp가 50ms
조건 밖이었다. queue overflow와 YOLO 잘못된 stamp는 실제
bag에서 0건이다. YOLO 결과 수신-minus-source-stamp는
p50 **34.0ms**, p95 **54.0ms**, p99 **63.0ms**;
LiDAR는 p50 **0.82ms**다. 이는 bag 수신과 stamped input의
차이이며 순수 모델 추론 시간은 아니다.

대기 상한 민감도(같은 causal replay, 독립 후보):
0ms 1,148, 10ms 1,258, 25ms 1,452, 50ms 2,046,
130ms 2,562 matched scan. 130ms가 많이 맞는 것은 미래
데이터를 가져온 증거가 아니라 **출력을 그만큼 늦춰서** 받을
수 있었기 때문이다. 운영 지연 예산과 다른 센서 부하를
검토하지 않고 130ms를 적용하지 않는다. 25ms 후보 대기
p50/p95/p99는 **13.0/25/25ms**다.

## 벽 잔여·객체 보존 판단의 경계

기존 130ms 오프라인 도구는 기다리는 동안 GPS/IMU까지
더 반영해 자세가 달라질 수 있다. 이번 후보는 그 결과를
실제 25ms 변환 결과로 가장하지 않았다. 같은 scan의
130ms sweep에서 `baseline`(보호 이용)과
`camera_missing`(보호 없음 + ambiguous fallback)이 남긴
기준 지면 평면 주변 잔여점·cluster·처리 시간을 **조건부 참고값**으로만
JSONL에 덧붙였다. 터널 진입 추가 매칭 58 scan의
missing-minus-protected 지면 평면 주변 잔여 p50은 19점, 터널 중간
41 scan은 0점이다. 그러나 실제 후보가 이 geometry를
재현했다고 주장할 수 없다. 130ms sweep의 필터 처리시간
참고 p50/p95/p99는 **5.83/7.78/9.66ms**이며 25ms
대기·YOLO·동시 GPU 부하를 포함하지 않는다.

49/50/51ms stamp와 0/10/25/26/50/130/200ms 도착
경계, 영점·역행·중복, 유효 빈 검출, OFF, queue overflow,
미래 자료 사용 금지는 7개 독립 합성 시험으로 확인했다.
손재호 고정 source의 기존 4개 CTest suite도 container
재생성 뒤 누락된 **test binary만 다시 빌드**해 통과했다.
그중 tunnel core의 보호점/벽 처리 시험은 원본 구현
회귀로 유효하지만, 동일 스캔을 새로운 25ms 선택으로
터널 파이프라인에 넣는 end-to-end 시험은 아직 없다.
벽 근접 차량·보행자·낮은 장애물의 실제 보호 효과 및
연속 주행 검출/추적 정확도도 미수락이다.

**판정:** 25ms 대기는 매칭 상태를 개선했지만 절반 가까운
스캔이 여전히 out-of-window다. 이 단계에서 운영에
연결하거나 터널 벽 문제의 근본 해결이라고 판단하지
않는다. 다음 기하 실험에서는 각 스캔의 점군·자세·보정
행렬을 **수신 시점에 고정**하고, 실제 선택 YOLO box로
같은 보호점 계산과 필터를 양쪽에서 실행해야 한다.
실제 보호점/잔여점/cluster 및 벽 근접 객체 보존,
종단 지연을 비교한다. 지속 YOLO 누락의 fail-open
벽 잔여는 매칭 대기만으로 해결되지 않으므로 기하
필터와 별도 원인으로 둔다.

## 이후 독립 모델 비교

사용자 검토 후 현재 YOLO11m 기준선과 YOLO11n/s,
YOLO26n/s/m의 COCO pretrained detection을 같은
이미지·해상도·batch 1·정밀도·threshold로 비교한다.
YOLO26 검출 weight와 COCO 80 클래스는
[Ultralytics 공식 문서](https://docs.ultralytics.com/models/yolo26/)에
명시돼 있다. 같은 class ID라도 box/confidence가 같다고
가정하지 않는다. 현 Ultralytics/PyTorch 호환성, weight
SHA, `person/car/truck/bus` 매핑, 원본 이미지 좌표와
source stamp를 먼저 확인한다. 모델 단독 속도보다
TwinLite·BEV·신호등·LiDAR 동시 실행의 입력→출력
p50/p95/p99, 보호 매칭률, 누락률, GPU/CPU 자원을
채택 기준으로 사용한다. 터널·가림·원거리·보닛
오검출을 따로 채점하고 COCO 밖 일반 장애물은
LiDAR가 보존한다. FP16/TensorRT와 threshold 조정은
모델 크기 비교 이후 별도 변수다. 이 모델 비교는
이번 L1-B3 승격 결정을 대신하지 않는다.
