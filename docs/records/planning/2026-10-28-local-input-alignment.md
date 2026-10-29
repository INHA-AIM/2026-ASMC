# LOCAL-2A — 입력 정렬과 흰선–터널 벽 간격 측정

2026-10-28, PR #126의 `feat/shadow-local-footprint`에서 시행했다.
LOCAL-2A의 입력 분리·상대 시각 정렬·후진 관측 표시와 저속 반복 검증을 완료했다.
**무장애25km/h, 장애물 회피·복귀와 운영 기본 전환은 아직 수락하지 않았다.**
다음 단위는 [LOCAL-2B](2026-10-28-local-timing-and-dr-pilot.md#제안하는-후속-단위)다.

## 기준선과 변경 위치

- 비교 기준: `6f416d46`의 LOCAL-1 실행. 기존 S2/S3·drive18/19/20 실패 자료는 보존한다.
- `58649789`: 독립 입력 선택·한 번의 상대 변환·상태 수신 분리·측정 도구.
- `2e792813`: 제어 판단과 진단이 동일한 ROS clock 표본을 사용하도록 수정.
- `a703498b`: 저속에서 단 한 번의 상태 수신 대기·진단·시험 포트 격리.
- NORMAL/map planner와 MPC 계산 코어, 기존 BBox 음영 방식의 행동·기본 launch는 변경하지 않았다.
  새 local만 실행하고 기존 제어 생산자가 함께 있으면 자신의 제어를 거부한다.

| 위치 | 이번 변경 |
|---|---|
| `asmc_planner/src/local/motion.hpp` | signed 속도, 보간 가능한 상대 이동 이력, 최대100ms 제한 예측 |
| `asmc_planner/src/local/node.cpp` | 최신 완료 camera bundle과 최신 LiDAR를 독립 선택, 계산 중 상태 callback 차단 제거, 세대 재검사 |
| `asmc_planner/src/local/planner.*` | source grid를 복사·재표본화하지 않는 좌표 조회, 이미 반영한 자차 이동 거리 중복 차감 제거 |
| `asmc_controller/src/local/node.cpp` | 계획 기준시각→제어시각 정렬, 원본 scene freshness 유지, 저속 한 틱 대기 |
| `asmc_camera/scripts/semantic_space_trial.py` | 계산용 semantic 결과의 추가10Hz 제한 제거. 최신 완료 입력을 bounded worker로 처리 |
| planner dashboard·offline gallery | 계산과 같은 raster 변환, 후진에도 유효 관측 표시, 입력별 지연·실제 명령 발행 여부 |
| `tools/perception/measure_tunnel_clearance.py` | offline 흰선/수직면 대조·3뷰 원본 RGB 재투영. 운영 벽 인식기를 추가하지 않음 |

## 발견한 원인과 처리

### 늦게 오는 입력과 불필요한 join

drive03의 실제 주행 구간 집계:

| 항목 | p50 | p95 |
|---|---:|---:|
| 카메라 묶음 source→planner 수신 | 137.0ms | 179.0ms |
| LiDAR source→planner 수신 | 8.2ms | 11.4ms |
| 선택한 LiDAR source가 family source보다 새로운 정도 | 116.7ms | 206.7ms |
| 계획 시점 camera source age | 173.8ms | 227.6ms |
| 계획 시점 LiDAR source age | 44.8ms | 97.4ms |
| family callback 대기 | 0.058ms | 0.101ms |
| LiDAR callback 대기 | 0.061ms | 0.088ms |

여기서 source는 현재 ROS header 기준이다. 촬영 원본 clock과의 정확한 네트워크 지연을
측정했다는 뜻이 아니다. 두 결과의 도착이 가까워도 원본 시각은 다르므로, 동일한50ms
창 안의 LiDAR만 요구하면 사용 가능한 장면을 버리거나 오래된 LiDAR를 선택한다.
**웹 영상을 경유하는 경로가 아니며 callback 대기도 주 병목이 아니었다.**

현재 정책은 다음과 같다.

1. probability·coverage·family·projection context의 동일 source 묶음은 유지한다.
   그중 최신 완료 묶음과 최신 LiDAR 관측을 독립적으로 고른다.
2. signed 속도+IMU 상대 이력으로 각 source를 계획 시각에 한 번 정렬한다. 지도/GPS는
   쓰지 않는다. LiDAR 최신 결과가 INVALID이면 과거 정상 결과로 숨기지 않는다.
3. grid는 원본 raster로 유지하고 footprint 조회 좌표를 역변환한다. 삼각함수는 footprint
   조회마다 한 번 계산하며40,000셀을 변환·복사하지 않는다. 정렬 p95는 약0.20ms다.
4. `LocalTrajectory.header`는 정확한 정수 ns의 계획 기준시각, `scene_stamp`는 원본
   camera 시각이다. 제어는 둘을 구분하며350ms freshness를 새 header로 갱신하지 않는다.
5. 자차 이동만 정렬한다. 객체 자체의 운동을 해결했다고 표시하지 않는다. 나이×
   `max(5m/s, 유효 절대속도)`의 불확실성 여유를 사용하지만 이는 실험용 상한 가정이다.
   동적 객체의 완전한 충돌 예측으로 수락하지 않았다.

카메라–LiDAR **fusion의 ±50ms matching 계약은 변경하지 않았다.** 이 변경은 local
planner가 이미 생성된 서로 다른 종류의 관측을 소비하는 경계에만 적용한다.

### 상태 callback과 제한 예측

같은 drive20 bag에서 사용 가능한 IMU 상태의 source 간격은 p95 53.23ms,
p99 79.80ms, 최대82.98ms였다.20Hz tick913개 중52개는 최신 상태가50ms보다 오래됐다.
원래50ms 예측 제한은 정상 센서 cadence에서도 실패할 수 있었다.

- 계산 밖의 짧은 snapshot 잠금과 `AsyncSpinner(2)`로 상태 수신을 분리한다.
  동시에 두 Plan을 실행하지 않고, 처리 중 reset/session 변경이면 결과를 폐기한다.
- 상대 이동 예측은100ms 이내로 제한한다. 상태 수신150ms·관측350ms 한계는 유지한다.
- drive02에서 상태 age100.718ms 때문에 **한 번의 full-brake**가 발생했다.
  직전100ms 안에 정상 계산했고5km/h 이하이며 입력·경로가 유효한 경우에만 다음
  상태를 한 틱 기다린다. 새 명령이나 새 timestamp를 발행하지 않으며 추가 예측도 하지 않는다.
- 한 번 기다린 뒤에도 복구하지 않으면 기존 무효 처리를 따른다. selector의250ms
  watchdog도 유지한다. 진단은 `motion_sample_pending`, `command_published=false`다.
- drive03에서는 같은 급제동이 재발하지 않았지만100.7ms 조건 자체도 다시 발생하지 않았다.
  강제 입력 공백·한 틱 대기·지속 중단은 별도의 실제 ROS 회귀에서 검증했다.

후진은 signed 상대 이력을 유지하므로 화면의 관측이 사라지지 않는다. 전진 전용 planner가
자동 후진을 제어하는 것은 아니며 `reverse_control_not_supported`로 구분한다.

## 반복 replay와 주행 결과

아래 replay는 같은 drive20 bag의 **native 주행 표본42.687초**만 비교한 것이다.
준비·시작·종료의 무효 표본을 삭제하지 않고 별도 전체 `summary.json`에 남겼다.

| 구성 | 유효 계획 Hz | 유효 계획 최대 공백 | 계획 계산 p95 | 무효 계획 |
|---|---:|---:|---:|---|
| 변경 전 | 9.488 | 376.2ms | 28.67ms | scene_stale4 |
| 변경 후 replay1 | 9.956 | 170.1ms | 30.55ms | 0 |
| 변경 후 replay2 | 9.956 | 176.0ms | 30.48ms | 0 |
| 변경 후 replay3 | 9.933 | 187.6ms | 31.84ms | 0 |

replay2 제어에는 `local_source_stale`1회가 있었다. `/clock`이 판단과 report 사이에
바뀌어 당시 진단만으로 원인이 모호해진 결함은 동일 clock 표본 사용으로 수정했다.
이를 입력 결함이 모두 사라졌다고 집계하지 않는다. 전체 replay에는 준비 단계의
335~364ms 발행 공백이 남으며 위 표와 혼동하지 않는다. 계획 계산량은 기준선보다
약2~3ms 증가했고, 개선점은 불필요한 입력 폐기와 공급 공백 감소다.

실제 MORAI에서는 객체 없는 터널을35m씩3회 주행했다. 시험 상한5km/h였지만 기존
근거리 가림 보조 정책이 목표를3km/h로 제한했다.25km/h 또는 전체 터널 수락이 아니다.

| run | 이동/접촉 | 주행 중 계획 Hz / 최대 공백 | 급제동 tick | 본차로 절대오차 p95 |
|---|---|---|---:|---|
| drive01 | 35.06m / 0 | 10.009 / 143.8ms | 0 | 전체4.96cm,10초 이후4.92cm |
| drive02 | 35.01m / 0 | 9.994 / 142.9ms | 1 | 전체1.14m,10초 이후4.19cm |
| drive03 | 35.01m / 0 | 10.012 / 154.1ms | 0 | 전체1.17m,10초 이후3.62cm |

drive02/03은 시작점을25m 전진시킨 곡선 구간이라 본차로에서 약1.43m 벗어난 상태로
시작했다. 전체 오차를 감추지 않고 초기 복귀와 정렬 이후를 나눴다. 위 오차는 **관측한
lane family 대비 내부 일관성**이며 독립 GT 차로 정확도가 아니다. 실제 속도 중앙값은
약2.97km/h, 순간 최대3.73~3.97km/h였다. 계획 p95 약29.7~30.3ms, MPC 전체 기록
p95 약11.0~17.3ms였다.3회 모두 native contact0·시험 복구 오류0이었다.

## 흰선과 실제 수직 벽 사이 여유

### 측정 방법

- GPS 무효 구간의 원본 LiDAR를 **해당 scan의 trace 변환**으로 실제 base와 수평화
  좌표에 올린다. 같은 session의 상대 이동 이력으로 family source 시각에 정렬한다.
- IMU 보정된 family 중 좌/우 외곽 role1/3의 **연속 OBSERVED 구간만** 사용한다.
  inferred·관측 공백을 연결해서 흰선을 만들지 않는다. 영상–scan 차이는35ms 이내다.
- 지면 위0.25~1.8m 점에서 전방3/5/7/9/11/13m의2m 창별 수직면을 찾는다.
  최소30점·전방 지지1.5m·높이 지지0.5m·3개 높이 구간을 요구해 기둥 하나와
  수평 선반을 벽으로 채택하지 않는다. 이 offline 검사는 planner 입력이 아니다.
- 흰선에서 바깥 수직면까지 차량 y방향 간격을 기록한다. 곡선의 법선 방향 최단거리와
  완전히 같지 않다. 원본 카메라 보정 ID를 검사하고 생산자의 투영 코어로3뷰 RGB에
  흰선(빨강)·벽 하단 지상점(청록)을 재투영해 확인했다.

### 수치

표의 중앙값과 p05–p95는 겹치는 scan/거리 창을 집계한 분포다. 독립 표본 신뢰구간이 아니다.

| 자료/구간 | 유효 scan / 거리 창 | 왼쪽 간격 중앙값 [p05–p95] | 오른쪽 간격 중앙값 [p05–p95] |
|---|---:|---|---|
| 기존 drive20 | 363 / 2,898 | 1.41m [1.16–1.75] | 2.20m [2.10–2.29] |
| 새 drive01, 같은 시작 | 375 / 2,988 | 1.42m [1.16–1.75] | 2.20m [2.10–2.29] |
| 새 drive02, 시작점+25m | 426 / 3,327 | 1.86m [1.68–2.13] | 2.13m [2.05–2.27] |
| 새 drive03, +25m 재시험 | 456 / 3,586 | 1.87m [1.68–2.13] | 2.13m [2.05–2.26] |

이 자료의 공간 범위는 서로 겹치는 두35m 구간으로 약60m 수준이며 터널 전체 입구·출구를
보장하지 않는다. 수직면 적합 잔차 p95의 창별 분포는 p95 약11.7~11.8cm다. 도장선
피팅·센서 보정·곡률·벽면 형상이 포함된 수치이므로 mm/cm 단위의 정확한 물리 폭으로
단정하지 않는다. 오른쪽13m 등 수직 지지가 없는 곳은 측정 불가로 남겼다.

**좌측 여유가 구간에 따라 달라 고정 대칭 폭을 채택하지 않는다.** 측정된 벽까지의 공간에
턱·비노면·개별 장애물이 있을 수 있다. 이를 주행가능 노면으로 자동 채우거나 semantic의
장애물 뒤 검은 영역을 지우지 않는다. 후속 통로 사전값은 구간별 자료와 실제 통과 시험을
거쳐 사용하며, 이번에는 운영 ROI·벽 제거·차체 마진에 새 상수를 넣지 않았다.

## 자료 위치와 재현

정본은 `D:\morai dataset\shahn03\artifacts`다. 아래 ID의 입력 SHA·설정·source revision/
dirty patch와 runtime 바이너리 근거를 보존한다. 과거 run은 덮어쓰지 않았다.

| run ID | 내용 |
|---|---|
| `local2a-20261028-audit01` | 이전/중간 replay, drive20 측정. 최종 그림은 `wall-drive20-verified/index.html` |
| `local2a-20261028-audit02` | drive18/19에 원본 점군/trace가 없어 벽 측정 불가였다는 근거 |
| `local2a-20261028-audit03` | 최종 replay1~3, `evaluation.json`, 집계 스크립트·센서 cadence·최종 검사·웹 |
| `local2a-20261028-drive01` | 실제 같은 시작점 주행, bag·판단·벽 측정 |
| `local2a-20261028-drive02` | +25m 구간,100.7ms 상태 공백의 급제동1회 보존 |
| `local2a-20261028-drive03` | 수정 후 재주행, bag·판단·벽 측정 |

새 주행 각 폴더의 `wall-clearance/index.html`에서 RGB·점군을 보고,
`measurements.jsonl`에서 scan/family stamp·좌우·전방거리·gap·점 지지·잔차를 읽는다.
`following-analysis.json`과 `drive-interval-evaluation.json`은 초기 복귀/실제 주행 집계다.
`trial-result.json`의 `closed_loop_acceptance:false`는 LOCAL-1 전체 수락을 뜻하지 않도록
유지했다. audit01의 중간 투영 그림 오류도 최종 그림과 구분하여 남겼다.

등록한 **새 run** 안에서 측정을 다시 실행한다. 아래 출력 하위 폴더는 존재하면 실패한다.

```bash
source /root/ws/devel/setup.bash
python3 /root/ws/tools/perception/measure_tunnel_clearance.py \
  /artifacts/local2a-20261028-drive03/sensors_0.bag \
  /artifacts/NEW_REGISTERED_RUN/wall-clearance \
  --camera-config /root/ws/src/asmc_camera/config/cameras_v3.yaml
```

컨테이너에는 재빌드한 local planner/controller와 최신 semantic 공급·웹 코드를 적용했다.
현재 `roslaunch asmc_planner local.launch`의 **제어 OFF preview**로 준비했으며
`http://localhost:8088/web/index_v2.html`의 LOCAL 실험 보기에서 확인한다. 기존 NORMAL/
BBox 제어와 selector는 가동하지 않았고 `/ctrl_cmd` 발행자0을 확인했다. 사용자 세션 이후의
실행 상태를 보장하는 것은 아니다. 제어 시험은 전용 반복 실행기의 소유권 검사를 거친다.

## 검증과 다음 LOCAL-2B

로컬 검사: native local core, 독립 ROS 입력 정렬(10/25km/h 합성·후진·무효 최신 입력·
session 전환), 제어 시각/한 틱 대기/중단/소유권, RenderWorker4개, 벽 측정3개,
dashboard JS/실제 브라우저, NORMAL AVOID·로터리·도로 제약·충돌 관련7개 회귀를 통과했다.
실제 브라우저의 깨진 문자·JS 오류가 없고 canvas가 화면 위에서 표시됨을 확인했다.
Hosted CI는 아직 실행하지 않았다. 외부 bag을 CI 필수 의존성으로 추가하지 않았다.

LOCAL-2B는 아래 한계부터 순서대로 해결한다.

1. 근거리 bonnet 가림을 일괄 비노면으로 취급하는 문제와3km/h 보조 제한을 분리한다.
   관측하지 않은 바닥을 벽 폭만으로 자유공간으로 만들지 않는다.
2. 우측 본차로 reference의 support와3개 경계 공통 support를 분리한다. 실제 노면·
   통로 유효 범위를 기준으로 후보를 만들고3/5선 중 하나를 고르는 정책은 도입하지 않는다.
3. strict 실패 후 보조 `Plan()` 재호출의 중복 scene/후보 계산을 제거한다.
4. 빈 터널5→10→15→20→25km/h에서 이력 정렬·추종·제동·관측거리 제한을 각각 확인한다.
   이번 합성25km/h 입력 시험을 실제 고속 주행으로 계산하지 않는다.
5. 벽/천장 오인 OBB의 노이즈 대응은 LOCAL-2C, 단일 정적 회피·복귀는2D로 분리한다.
   이번에 실제 객체를 제거하거나 NORMAL의 안전 검사를 완화하지 않았다.
