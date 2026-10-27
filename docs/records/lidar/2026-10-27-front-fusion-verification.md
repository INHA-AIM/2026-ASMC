# F1 전방 camera–LiDAR fusion 검증 — 2026-10-27

**실제 native matching 식별·진단, 원본 RGB 갤러리, 두 연결 결함 수정까지 완료했다.**
좌우 matching·원본 clock 대응·전체 공급 수락을 완료한 것은 아니다. 특히 정차 기록의
473.47ms 유효 결과 공백은 250ms 기준 실패로 남긴다.

## 기준과 변경 범위

- main `936663152de770795edd925e11c115d84b8dfb00`(PR #117)을 확인하고 시작했다. PR #115·#116을 포함한다.
- branch `feat/camera-lidar-fusion-verification`; run `camera-lidar-f1-20261027-r1`.
- [등록 manifest](../../../config/baselines/camera_lidar_f1_v1.json)와
  [작은 결과 요약](../../../config/baselines/camera_lidar_f1_result_v1.json)에 소스·설정·입력 SHA를 연결한다.
- 등록 당시 D 여유 약47GiB에 대해 사용자가 **이번 F1 동시 기록만** 저장 예외를 승인했다.
  전체 storage policy·LAN2LAN·센서/네트워크 정본은 변경하지 않았다.
- 모델·전처리·추적·속도·HD맵 필터·판제 정책·공용 message/service/MD5는 유지했다.
- matching ±50ms, wait 정책200ms, unique 관측, IoU0.25, 분류 초기5/변경3/미스3은 유지했다.

## 수정한 연결 결함

| 재현된 문제 | 수정 | 실패/통과 근거 |
|---|---|---|
| `LidarToImage`가 잘못된 context를 거부해도 결과를 무시하고 빈 투영으로 IOUFusion을 실행. 기하 실패가 미검출/miss로 바뀜 | 투영 실패 scan을 명시적으로 drop. 이미지 consume·miss·근거 시각 갱신 없이 객체 기하/분류 보존 | `projection-defect-before.log` 실패 → `FusionContract.InvalidProjectionIsNotAnEmptyCameraObservation` 통과 |
| Current 검사가 투영 이전뿐이라, 점군 투영 중 바뀌거나 폐기된 관측이 분류 적용까지 갈 수 있음 | 투영 완료 후 분류 적용 직전에 receiver Current 재검사. 실패 시 역사 scan을 대기로 돌려 다음 호출에서 재선택 | `epoch-race-before.log` 실패 → `FusionContract.RecheckCameraEpochAfterProjectionBeforeApplying` 통과 |

첫 수정은 `b7ae0d7d`, 두 번째는 `da9c182f`다. 새 epoch나 IMU를 오래된 scan에 덧씌우지 않는다.
두 번째 검사는 세대뿐 아니라 receiver가 더 이상 현재 관측으로 인정하지 않는 경우에도 적용된다.
실제 터널 기록에서 `observation_changed_during_projection` 1건을 대기로 처리했다. 이 사유만으로
해당1건을 생산자 재시작이라고 단정하지 않는다.

기존 27개에 진단3개·결함2개를 더한 front fusion32개가 통과했다. ±50ms 경계, 대기 만료,
unique/분류 유지·session 전환은 기존 fixture도 함께 유지한다. 잘못된 geometry를 정상 빈 결과로
취급하지 않는 것이 이번 동작 변경이며, matching 알고리즘이나 threshold를 개선한 것은 아니다.

## 실제 선택을 보여주는 진단

`trace.fusion`은 현재 호출 scan과 실제 처리한 과거 pending scan을 구분한다. 실제 camera session,
generation, observation ID, calibration/model/config ID, 영상 header·wire seq, signed 시간차·대기,
원본 bbox index·실제 IoU·분류 before/after·신규 근거 반영을 기록한다.

기존 debug snapshot은 같은 serializer를 사용하고 웹에 전방 실제 matching / 좌우 수신만을 구분한다.
1Hz diagnostics는 debug OFF에서도 동작한다. 최근100scan 집계, 호출당64결정/128association
제한과 생략 수를 노출하며 점군 복사나 새 영상 인코딩 경로를 추가하지 않았다.
선택 객체가 이번 snapshot에서 매칭되지 않았으면 이전 관측을 방금 매칭된 것처럼 표시하지 않는다.
[현행 해석과 실행](../../../src/asmc_lidar/docs/fusion-verification.md)을 따른다.

HD맵 보호는 실제 fresh 양성 camera vehicle/dynamic 근거일 때만 적용하는 기존 정책 그대로다.
LiDAR 운동 판정이나 과거 class 유지로 보호하는 로직을 추가하지 않았다. 큰 guardrail cluster와
인접 차량의 IoU 한계는 원본 RGB/군집 투영을 함께 남겼으며, 모든 오매칭을 해결했다고 보지 않는다.

## 동시 기록과 실제 실행

각60초, 총240초로 사용자 허용10분 이내다. 카메라3모델·격리 LiDAR debug·기존 웹을 사용했다.
ROS `ctrl_cmd` 송신자가 없는지 확인하고, 기존 P 경로 추종기를 이용한 gRPC 시험 제어만 실행했다.
기존 사용자 actor는 건드리지 않고 정차/FOGGY용 차량3개만 생성·제거했다.
가감속 시험 최고3.87m/s·횡편차0.13m, 터널 최고4.12m/s·횡편차0.21m였다.
각 시험은 차량 위치·gear·control mode·날씨/시간을 복원했고 failure0이었다.

| 조건 | matching된 scan 수 | 완전5초 창 최소 유효 trace Hz | 처리 p95 ms | header→trace수신 p95 ms | 유효 trace 최대공백 ms |
|---|---:|---:|---:|---:|---:|
| SUNNY 정차 | 637 | 10.6 | 6.22 | 8.78 | 473.47 |
| SUNNY 가감속·조향 | 630 | 11.6 | 4.22 | 6.36 | 139.42 |
| SUNNY 터널 | 624 | 11.4 | 12.87 | 15.92 | 122.64 |
| FOGGY 정차 | 631 | 11.8 | 5.82 | 8.23 | 121.07 |

- 표의 수신 지표는 bag의 trace 소비 시각이다. 판제 토픽 수신 지연이나 촬영→발행 지연으로 바꾸어 해석하지 않는다.
- SUNNY 정차 raw 입력 최대공백310.57ms와 scan3343의 `vehicle speed is stale for LiDAR scan`을
  확인했다. 이 scan의 무효 처리를 포함한 유효 trace 공백은473.47ms였다. 단순히 fusion 계산이
  473ms 정지했다고 해석하지 않는다. 원천 네트워크/CAN 공급 공백의 근본 원인은 이 기록만으로 확정하지 않았다.
- 나머지 세 조건의 유효 trace 최대공백은139.42/122.64/121.07ms였다. 평균 또는 최소 창 Hz만으로
  정차 실패를 면제하지 않는다. 전체 주행 공급 수락은 아직 아니다.
- 터널712scan 모두 map=`gps_missing`이면서 local 처리와 전방 투영을 유지했다. 마지막 GPS를 재사용하지 않았다.
- wait 정책200ms는 다음 scan 호출에서 재평가한다. 실제 관측한 match 대기는 정차 최대310.05ms,
  터널203.13ms였다. 200ms를 OS 실행 deadline으로 보장하지 않으며 FIFO/스케줄링 개선은 후속이다.

원본 RGB는 camera topic/header stamp/sequence의 정확한 대응만 사용했다. 실제 native 전방 cluster
bbox와 detector bbox, 좌우의 과거 scan `base_link` 8코너를 원본 영상에 표시했다. 좌우에 별도
matching을 구현하지 않았고 시간이 다른 코너를 ego motion으로 정렬했다고 표시하지 않는다.
전방1280×720·좌우640×480 보정 ID/크기 일치, 기존54 투영 오차<1e-8, 누락/중복 역변환
negative control을 확인했다. 순수 수치 일치이며 독립 GT에 대한 정확도 증명이 아니다.

카메라 source clock은 `morai_sensor`, LiDAR는 `velodyne_device_hour`다. 같은 bridge session이어도
직접 차감하지 않았다. 기록 시작 가장자리 등 exact timing 대응이 없는 사례는 별도로 표시했다.
회귀에서 다른 카메라/sequence·중복 대응·session·clock·역행을 정상 대응으로 인정하지 않는다.

## 진단 비용과 출력 동등성

같은 정차 bag을 OFF/ON 각각3회 교차 replay했다. 전체712 처리 scan의 객체·순서·OBB·추적·분류·
필터 결과 SHA는6회 모두 `607f6f66239bead768c10bbdbe5c0c543e478b207a885f20d22e3ee35d253bcd`로 같았다.

- 처리 p95 OFF: 4.778/4.746/4.828ms; ON: 4.768/4.910/4.647ms.
- 같은 repeat의 최대 증가0.164ms: ≤2ms 기준 통과.
- peak RSS: OFF159768/159896/160300KiB, ON159748/159760/159784KiB.
- 100scan bounded 집계·결정/association 상한 회귀 통과. 이 짧은 실험을 장기 메모리 leak 부재 증명으로 확대하지 않는다.
- replay는 기록 도착 순서와 native 런타임을 사용한다. 실제 thread 대기·ROS 발행·web 직렬화 비용을
  이 수치에 모두 포함했다고 주장하지 않는다. 웹은 기존 sampled 경로·HTTP/JS 회귀로 별도 검사한다.

## 빌드·계약·ROS 검사

- 현재 컨테이너 `catkin_make` 전체 재빌드와 offline probe 빌드 통과.
- `.github/ci/lidar.sh`: 필수 ctest35개 모두 통과(53.89초), 카메라 수신·격리 LiDAR node·monitor
  ROS 시험, 공용 LiDAR→판제 어댑터와 boundary 회귀, 웹 JavaScript68개 통과.
- front fusion32개, C0/F1 작은8개(54투영 포함), F1 Python3개, 공용 메시지 직렬화5개 통과.
- 메시지/토픽 계약·정본화·문서 링크/실행 예제·CI coverage 검사 통과. 새 F1 검사를 기존 CI에
  등록하고 필수 front 검사 최소 수를32로 올렸다. 별도 대용량 bag CI job은 추가하지 않았다.
- 최초 로컬 전체 검사에서 CI가 별도 생성하는 `planning_gps_recovery_test`가 없어서 중단됐다.
  해당 실행파일을 생성한 뒤 동일 전체 검사를 통과했다. 생략으로 처리하지 않았다.
- 시험 소유 camera/LiDAR/view는 종료했고 기존 bridge·master는 유지했다. 기존 사용자 roslaunch
  부모 프로세스는 종료하지 않았다. 갤러리 HTTP200 확인. Hosted CI는 PR 게시 후 별도 결과다.

## 근거와 실패 보존

D 정본 `artifacts/camera-lidar-f1-20261027-r1`에 다음 자료가 있다.

- `stationary/straight/tunnel/foggy/signals.bag`, `drive.jsonl`, `report.json`.
- `stationary-result`, `straight-analysis`, `tunnel-result`, `foggy-result`: 최종 요약·갤러리·SQLite/결정 기록.
- `stationary-result/replay-*`:6회 동일 결과·자원·실행 로그.
- `index.html`·`result-summary.json`: 최종 목록.
- `projection-defect-before.log`, `epoch-race-before.log`, `epoch-fixed-tests.log`: 수정 전 실패와32개 통과.
- `workspace-build.log`, `projection-regression.log`, `lidar-regressions-r2.log`: 현재 빌드·회귀 근거.

기존 실험 서버에서 `http://localhost:8000/camera-lidar-f1-20261027-r1/index.html`을 연다.
초기 분석의 ROS callerid bytes 직렬화 실패, probe의 receipt metadata 누락, `/usr/bin/time` 미설치
실패 출력은 덮어쓰지 않았다. 현재 도구는 callerid 문자열 정규화, 기록된 receipt 검증, 기존 독립
resource helper를 사용한다. 최종 결과는 위 명시한 폴더만 따른다.
등록 manifest의 예전 일반/터널 bag은 C0 참조 목록이며 이번 실시간4개 bag과 섞지 않았다.

## 완료 경계와 다음 한 단위

다음은 **뷰별 비차단 3뷰 matching 설계**다. F1의 실제 observation ID·역사 scan context·미사용/
분류 유지/새 근거 구분을 지키며, 느린 전방을 기다리는 FIFO가 좌우까지 막지 않게 설계한다.
여러 카메라가 같은 객체를 본 것을 독립적인 시간 확인 횟수로 세지 않는 규칙을 먼저 확정한다.

안승현은 보정·시각·카메라 연결 fixture를 지원하고, 손재호는 matching·추적·속도·GPS 없는
터널 구조물 처리 알고리즘을 담당한다. 원본 clock 매핑·ego motion·deskew는 별도 근거가 필요하다.
판제 행동 코드와 메시지 소비 경계는 이번 F1에서 변경하지 않았다.
