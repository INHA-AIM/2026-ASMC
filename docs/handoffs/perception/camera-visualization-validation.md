# 객체 시각화·차선 공급 최적화 검증 — 2026-10-15

실행 정본은 [카메라 실행·관측 계약](../../contracts/perception/camera-runtime-v1.md)이다.
PR #85에서 E8/사전학습 선택, 16종→세 그룹, confidence0.40·NMS0.50, 3뷰 mask와 원본 header를 유지한다.
YOLO/TwinLite 목표15Hz·ResNet10Hz, 시각화 목표10Hz다. 학습·수집·LiDAR 연결·판제 정책·PR 병합은 수행하지 않는다.

## 실행 경로

객체 시각화의 기본 경로는 네 노드의 JPEG 중계에서 `camera_object_view` 한 노드의 직접 합성으로 바뀐다.
기존 bbox·관측 결과를 사용하며 추론 생산자를 만들지 않는다.

```text
3뷰 원본 JPEG + CameraObjectObservation
    → 정확한 camera/stamp 대응 → 뷰별 overlay
    → 좌/전방/우 합성 → JPEG 한 번 인코딩
```

- JPEG 캐시는 뷰별8프레임·500ms다. 렌더링 worker는 하나이고 최신 관측만 보관한다.
- 같은 뷰/stamp의 렌더링을 재사용하며 기본 합성 경로에는 중간 JPEG가 없다. 개별 뷰 토픽은 구독할 때만 추가 인코딩한다.
- session·generation·카메라·stamp·보정·모델·설정을 구분한다. 상태 세대와 다른 관측, 이전 session과 동일 세대의 상충 설정은 표시하지 않는다.
- 정상 빈 결과는 새 영상으로 갱신한다. 무효 결과·STANDBY·재시작은 이전 박스를 지우고, 오래되거나 중단된 뷰는 빈 WAITING/STALE 패널로 표시한다.
- 합성 header는 디버그 화면의 발행 시각이다. 각 패널의 source stamp를 따로 표시하며 동기 3뷰 관측으로 설명하지 않는다.
- 기존 개별/합성 토픽과 `view.launch models:=objects` 사용법을 유지한다. 명시적 legacy/raw/filter 진단은 기존 렌더러를 유지한다.

화면 구독자가 없으면 큰 영상 구독과 변환을 중단한다. TwinLite·BEV·ResNet 시각화에도 수요에 따른 구독을 적용했다.
BEV 비교 화면을 실제 구독하지 않으면 raw 영상과 융합 영상의 불필요한 구독이 생기지 않는다.
입력 구독의 연결·해제는 wall time으로 확인하며 모델별 독립 프로세스는 유지한다.

## Readiness 진단과 추가 계산 최적화

기존 `/perception/camera/diagnostics`에 `camera_semantic_perception/readiness` 상태를 추가했다.
입력 준비, family 유효성, source age, 수신 후 경과 시간, generation·cutoff와 실패 이유를 같은 snapshot에서 기록한다.
객체 시각화의 codec·캐시·뷰별 렌더링 수는 기존 `/perception/camera/objects/diagnostics`의 `camera_objects/view`에 추가했다.
공개 메시지·서비스 필드와 MD5는 변경하지 않았다.

초기 GUI 실행은 모델과 family가 유효해도 family source age가350ms를 넘겨 readiness가 꺼졌다.
JPEG 중계를 제거한 뒤에도 실패가 남아 다음의 불필요한 처리를 계측하고 줄였다.

- 차선용 세 뷰의 작은 모델 격자를 큰 디버그 확률 영상보다 먼저 발행한다. 배열·header·계산 값은 그대로다.
- 의미 영상의 색상 이미지를 LUT로 생성한다. CPU 대조와 회귀에서 원래 픽셀과 정확히 일치했다.
- LUT 연산의20스레드 실행은 wall5–6ms/CPU24–26ms, 1스레드는 wall/CPU 약7ms였다. TwinLite 시각화 프로세스만1스레드로 제한했다.
- fitting cProfile에서 확인한 반복 평균·B-spline 기저·트랙 정렬/보간을 재사용했다. 파라미터·선택 규칙·차선 알고리즘은 유지했다.
- 기저 캐시는64개·행렬당256KiB 상한이며 호출자에는 독립된 writable 배열을 반환한다. 키 크기도 제한한다.

확인된 수치 회귀에는 기존/신규 기저·평균 예측·스칼라/벡터 보간의 정확한 일치, 캐시 오염 방지와 상한이 포함된다.
시각화 스레드 설정의 정본 SHA를 갱신하고 이전 SHA·변경 이유를 보존했다. 이전 실험의 SHA와 결과는 수정하지 않았다.

## 측정과 수락

최종 교차 반복·GUI 대조·동일 영상 비교를 완료했다. 30분 연속 시험의 최종 수락은 완료 결과로 갱신한다.

기존 기준은 계측만 추가한827a53dd 경로이고 신규는 같은 모델·입력·후처리의 최적화 경로다.
MORAI와 세 모델 및 기존 Main preview를 켠다. Main 제어는 `/b0/discarded_ctrl_cmd*`로 격리한다.
GUI 전체 조건은 세 모델의 디버그 출력에 sink를 연결하고 객체 합성 화면에 실제 rqt 하나를 연 조건이다.
BEV 비교 화면·추가 rqt 창 여러 개와 공동 자율주행 수락을 의미하지 않는다.

- SUNNY/FOGGY 각3회·60초에서 기존/신규 순서를 교차한다. GUI OFF와 객체 화면만 켠 대조도 날씨별1회 남긴다.
- 반복 이후 최종 구성으로 SUNNY 전체 GUI30분을 연속 실행한다. 개인 문서·브라우저 작업을 종료하지 않는다.
- 모델 지연은 압축 입력 callback→발행의 monotonic wall 시간이다. Family는 같은 source stamp의 TwinLite callback→family 구독 수신이며 별도다.
- 반복 통합 p50/p95/p99는 원시 프레임을 합쳐 계산한다. 실행별 중앙값의 평균이 아니다.
- YOLO/TwinLite 완전한5초 창≥10Hz, ResNet≥9.5Hz, 모델 p95≤150ms·p99≤200ms, 공백≤250ms, 전체 VRAM≤90%를 검사한다.
- 유효 family 신규 공급≥10Hz·공백≤250ms, 기존350ms freshness와 readiness 순간 저하0을 요구한다. 목표15Hz의95% 달성은 별도다.
- OOM·지속 큐 증가·중복/역행 stamp·silent fallback을 허용하지 않는다. 센서 공급 저하를 자동 면제하지 않는다.

원본 실패·부분 실행·cProfile 진단은 별도로 남긴다. cProfile 실행의 지연을 성능 수락 값으로 사용하지 않는다.
JPEG 호출 계측은 양쪽 경로에 같은 도구를 적용한다. 정상/실패 화면은 이미 수신한 JPEG를 보관했다가 측정 종료 후 저장한다.
실패 화면은 readiness 저하 시점의 가장 최근 디버그 화면이며 정밀 동기 GT로 설명하지 않는다.
장시간 RSS에는 입력/출력 probe의 원시 측정 기록도 포함된다. 캐시 크기·pending·교체 수를 함께 검사해 생산 큐 증가와 구분한다.

### 전체 GUI 교차 반복 결과

아래 CPU는 camera/debug/Main preview ROS 프로세스의 합계다. 100%는 한 코어이며 rqt는
이 합계 대신 별도 WSL 전체 CPU에 포함된다. Windows 애플리케이션 CPU/RAM은 측정할 수 없다.
RSS 합계는 공유 메모리를 중복 셀 수 있고 전체 GPU에는 MORAI·Windows 배경 부하가 포함된다.
누락된 CPU 샘플은0으로 채우지 않는다.

| 구성 | 날씨 | CPU 평균 % | RSS peak MiB | 전체 VRAM peak % | 유효 family Hz 평균 | readiness false snapshot |
|---|---|---:|---:|---:|---:|---:|
| 기존4노드 | SUNNY | 500.7 | 4,121.3 | 58.92 | 10.88 | 28 |
| 신규1노드 | SUNNY | 430.1 | 3,921.4 | 59.00 | 14.56 | 0 |
| 기존4노드 | FOGGY | 498.0 | 4,092.9 | 58.90 | 11.07 | 24 |
| 신규1노드 | FOGGY | 429.6 | 3,886.4 | 58.92 | 14.67 | 0 |

각 행은60초3회다. false snapshot 수를 독립적인 실패 사건 수로 설명하지 않는다.
CPU 합계는 SUNNY14.1%·FOGGY13.7% 감소했다. 여러 최적화가 함께 적용된 최종 대조이므로
각 변경이 개별적으로 이 수치를 만들었다고 주장하지 않는다.

| 구성 | 날씨 | Family p50 / p95 / p99 ms |
|---|---|---|
| 기존 | SUNNY | 197.3 / 260.4 / 288.4 |
| 신규 | SUNNY | 128.0 / 186.2 / 210.3 |
| 기존 | FOGGY | 193.3 / 255.0 / 286.7 |
| 신규 | FOGGY | 129.4 / 189.7 / 215.1 |

신규 모델 callback→발행 지연은 다음과 같다. Family 지연에 모델용150/200ms 기준을 적용하지 않는다.

| 출력 | SUNNY p50 / p95 / p99 ms | FOGGY p50 / p95 / p99 ms |
|---|---|---|
| YOLO 전방 | 46.3 / 75.0 / 85.1 | 46.8 / 75.1 / 86.2 |
| YOLO 좌 | 46.2 / 75.1 / 85.9 | 47.0 / 75.9 / 87.0 |
| YOLO 우 | 46.1 / 74.8 / 86.1 | 47.1 / 75.1 / 85.5 |
| TwinLite 전방 | 59.2 / 97.0 / 110.4 | 58.2 / 100.0 / 113.7 |
| TwinLite 좌 | 61.0 / 101.6 / 113.6 | 60.4 / 103.3 / 116.2 |
| TwinLite 우 | 61.6 / 101.6 / 114.3 | 60.7 / 103.5 / 115.9 |
| ResNet camera4 | 62.1 / 94.3 / 104.8 | 57.9 / 91.2 / 96.9 |

신규6회와 GUI OFF/객체만 대조4회 모두 최소 공급·모델 지연·공백·readiness 기준을 통과했다.
유효 family의 최저5초 창은 전체 GUI에서13.6–14.6Hz였다. 목표15Hz의95%인14.25Hz를
모든 창에서 달성한 것은 아니며 최소10Hz 통과와 구분한다.
SUNNY-r1의 객체 codec 호출은 기존 decode4,247/encode3,222회에서 신규1,717/594회로 줄었다.
신규 내부 snapshot에서 합성 encode/publish585/585, 개별 encode0, 오류0과 뷰별 캐시 상한8을 확인했다.
CPU/ROS 시험은 같은 뷰·stamp 중복 decode0과 중간 encode0을 확인한다.

첫 FOGGY 신규 회차의 자원 sampler는 종료된 노드의 XMLRPC를60초 기다려1개 샘플만 얻었다.
이 회차의 CPU는 unknown이며0%로 해석하지 않는다. 원본을 보존하고 XMLRPC timeout250ms·
실패 조회5초 backoff와 실제 실행 노드 목록을 적용했다. 같은 조건을 자원 coverage 때문에만
한 번 재측정해120개 샘플을 확보했다. 최종 표와 pooled 지연에는 이 재측정 회차를 사용한다.
80% 이상 샘플·유효 CPU/GPU90% 이상·샘플 공백3초 이하를 요구하며 성능 열세 회차를 교체하지 않는다.

### 출력 동등성과 로컬 검사

같은 저장 RGB에서 객체180/180·차선 class-ID180/180·신호등60/60 대응이 일치했다.
신호등 confidence 차이도0이었다. 렌더러는 기존 overlay/mosaic 함수를 사용하고 원본 영상은 변경하지 않는다.
CPU 회귀728개와 계약 회귀35개, 문서·정본 설정 SHA·토픽 정책 검사를 통과했다.
생성된 ROS 메시지 fixture와 최신 hosted CI 상태는 최종 검증 기록에서 로컬 검사와 구분한다.

## 결과·재현·후속

- 원본: `data/runtime/camera-visualization-v1/results/`, 기존 진단은 `camera-structure-v1/results/visualization-baseline-diagnostic/`.
- 갤러리: `artifacts/camera-visualization-20261015-r1/`, ASMC 실험 목록의 날짜·E번호로 연결한다.
- 집계: `summarize_camera_visualization.py --matrix final-matrix-r2`를 사용한다. 실패와 최종 후보를 구분한다.
- 같은 저장 RGB에서 기존/신규 bbox·차선 class-ID·신호등 출력을 대응한다. 파라미터나 confidence를 다시 선정하지 않는다.

`asmc-ros-noetic`의 기존 카메라·시각화 launch를 종료한 뒤 다음을 실행한다.

```bash
source /data/runtime/camera-visualization-v1/devel/setup.bash
roslaunch asmc_camera camera.launch mode:=active
```

별도 터미널:

```bash
source /data/runtime/camera-visualization-v1/devel/setup.bash
roslaunch asmc_camera view.launch models:=objects
```

또 다른 터미널:

```bash
source /data/runtime/camera-visualization-v1/devel/setup.bash
rosrun rqt_image_view rqt_image_view --clear-config /perception/camera/objects/debug/mosaic/compressed
```

기본 STANDBY, PREPARE/ACTIVE·기존 alias, 사전학습 선택은 실행 계약을 따른다.
수락 실패가 남으면 기본 전환과 PR 준비 완료를 보류한다. 병합은 이번 범위가 아니다.
다음 작업은 `asmc_lidar`의 카메라별 관측 수신·버퍼·보정·투영 경계 인계다.
