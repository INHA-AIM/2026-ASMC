# LOCAL-2H — 10월29일 1차 비교 체크포인트

이 문서는 `feat/shadow-local-footprint`에서 2026-10-29에 수행한 LOCAL 보완과 실제 시험을 기록한다.
최신 main 반영·배포·push·PR·병합은 이번 단위에 포함하지 않는다. 다음 요청에서 localization 방식과
비교할 수 있는 체크포인트이며 운영 음영 방식의 승격이 아니다. 기준 revision은 `430effff`
(`archive/local2h-baseline-20261029`)다. 각 실제 실행의 정본 SHA는 해당 `runtime-provenance.json`이다.

## 결론과 사용 범위

현재 선택한 설정은 10km/h 상한, 기존 차체 외곽·30cm 최소 여유·350ms freshness·실측 상태 기반
속도 계획을 유지한다. FOGGY13에서 빈 터널, NORMAL→LOCAL→NORMAL, 주차 차량, 횡단 보행자,
얇은 철제 장애물, 단일 방호벽을 실제로 주행했다. 대표 최신8회는 모두 거리 한도에 도달했으나
짧은 제동·정지는 남는다. 모든 장면 무정지, 250ms 유효 경로 공급, 충분한 실제 이격을 수락한 결과가 아니다.

- 자동 모니터: <http://localhost:8088/web/index_v2.html>. `LOCAL 경로·노면·fusion` 또는 자동 보기.
- 실험 목록: <http://localhost:8000/> → **LOCAL-2H 연속성·기준선·검증 길이**.
- 정본 자료: `D:\morai dataset\shahn03\artifacts\local2h-20261029-validation01`.
- 하위 `driveNN`은 각각 별도 등록된 run이다. 실패·준비 구간·잘못된 후보도 보존한다.

## 원인 분리와 수정

### family 수신과 노면 fallback

영상 context/노면 raster가 family fitting보다 먼저 도착한다. 무조건 최신 노면을 고르면 정상적인
family 대기에도 fallback과5km/h 제한이 반복됐다. 현재 선택은 동일 stamp의 완전한 묶음을
우선하고 정상 fitting 수신 간격을 고려한다. fitting이 실제로 멎어 source age250ms를 넘으면
최신 유효 노면으로 전환한다.350ms 한계를 늘리지 않으며 최신 명시적 plane/config 무효를
과거 정상 family로 가리지 않는다. 선택 source stamp의 역행도 금지한다.

`drive17`은 planner에만 family 수신을2초 차단했다. 실제30메시지가 빠진 동안 비교 창에서
유효 계획25개(노면20/family5), 정상 MPC50개, 무효 명령0이었다.100.246m에 도달했다.
이는 수신 누락 통제시험이며 plane 자체 무효나 모든 실제 인지 실패까지 해결했다는 뜻은 아니다.
`family-gap-proof01.json`, `alignment05/06.log`가 근거다.

### 기준선이 짧은 경우와 검증에서 잘리는 경우

`reference-vs-validation01.json`, `short-prefix-stages01.json`은 기준선 길이·탐색 한계·선택 길이를
분리한다. 대표8회에서 기준선8m 미만은 주차 차량 run의8계획뿐이었다. 대부분 기준선18–20m,
차체 앞길이를 차감한 탐색 약15.8m가 있어도 OBB·노면 내부 가림·근거리 검증에서3–4m로 줄었다.
기준선이 길다는 이유로 검증된 경로도 길다고 표시하지 않는다. debug 후보는12개 제한이 있어
화면에 없는 후보를 미생성으로 단정하지 않는다.

복구 목표4→6m도 비교했다.4m 이후는 기존 strict 노면·차체·OBB sweep을 그대로 적용했다.
새 미관측 허용이나 속도 상향은 없다. `drive23/24`의 같은 정지 person 배치에서6m 후보는
복구 경로106개 중1개만4.1m를 넘었고, 실제 정지 시간은5.47초 대2.14초였다. 따라서
**6m 확대는 채택하지 않고 기본4m를 유지**한다. 경로 끝점 숫자를 늘리는 것만으로 해결되지 않았다.

### 후보와 속도 계획

기하상 우승 후보가 실제 가속/조향 상태의 속도 계획에서 실패할 때 새로 검증 가능한 저속 복구를
다시 검사한다. 도로 연결 성분의 중심과 본차로만으로는 같은 쪽만 탐색하는 사례가 있어,
양쪽 안쪽 가장자리도 bounded 후보에 포함한다. 차체/OBB 충돌·외곽 노면은 계속 veto다.
가까운 측방 OBB를 실제 선택 sweep이 좁게 지나는 경우만5km/h maneuver 상한을 적용한다.
YOLO 검출 자체를 저속 조건으로 사용하지 않는다. 복구3km/h/이미 보조5km/h인 경우 중복 계산을 생략한다.

### 정적 의미와 객체 나이

공용 LiDAR velocity/motion/분류를 덮어쓰지 않았다. 기존 semantic static 기능도 기본OFF다.
LOCAL의 planning-only 후보는 최신 고정 형상6표본·300ms 근거가 있을 때만 과거 시각 오차
외곽을5→0이 아닌 **2m/s**로 제한한다. 유효 절대속도는 하한으로 보존하며 vehicle/dynamic 및
이동 가능한 원본 클래스는 배제한다. 움직임·형상 변화·근거 만료 시 즉시 해제한다.
이는 정적 확정이나 무효 속도의0 대체가 아니다. 원래 polygon과 나이 불확실성 외곽을 웹에서
따로 표시한다. 단독 적용 `drive14`는 지속 정지를 해결하지 못해 그 결과도 보존한다.

### 실험 실행의 결함

MORAI P warmup도 UDP의 D 필드로 덮어써져 준비 중 약2.9km/h 움직이던 문제를 찾았다.
시험 전용 transport gate가 준비 중 실제 actuator를 brake hold하고, release 후 새 명령만 보낸다.
운영 bridge는 변경하지 않았다. 이후 출발 전 속도 약0.00002km/h로 배치를 고정했다.
`drive11` 이후는 실제 환경을 FOGGY13으로 확인하고10초마다 readback한다. 초기SUNNY/Cloudy
관련 탐색은 공식 FOGGY 대조와 섞지 않는다.

## 대표 실제 결과

아래 정지 시간은 실제 속도 기록의 stationary episode이며 invalid command 개수와 다르다.
모든 run의 contact 신호와 SDK box overlap 중단은 없었다. SDK box는 mesh 충돌 정본이 아니다.

| run | 장면 | 진행/소요 | 실제 정지 | 계획 p95 | 유효 경로 최대 공백 |
|---|---|---:|---:|---:|---:|
|15|drum 재배치|60.16m /38.12s|0.54s|44.2ms|303ms|
|16|같은 drum|60.26m /42.54s|1.52s|53.3ms|296ms|
|17|빈 터널+family2초 누락|100.25m /40.15s|0|33.1ms|201ms|
|18|NORMAL↔LOCAL 통합|235.92m /73.90s|0|43.3ms|208ms|
|19|주차 vehicle|60.22m /53.15s|3.65s|44.6ms|900ms|
|20|횡단 person|60.11m /31.21s|0|43.9ms|305ms|
|21|얇은 steel|60.09m /37.43s|0|44.0ms|206ms|
|22|단일 barrier|60.01m /36.39s|0|44.3ms|203ms|

통합 run18의 전체 최고40.36km/h는 NORMAL 구간이다. LOCAL 상한10을 올린 결과가 아니다.
SDK 최소 간격은15/16에서14.6/22.6cm,19에서24.4cm였으므로 모든 실제 여유가30cm 이상이라고
주장하지 않는다. 인지 polygon·추종·SDK box 차이가 남는다. 유효 경로 공백과 실제 명령 공급은
별도 기록한다. 주차 차량의900ms는 유효 경로 공백이며 같은 길이의 UDP 침묵을 뜻하지 않는다.

## 채택하지 않은 후보

- `a22def38`의 이전 경로0.8m 진행량 slack: run10이25초 지속 정지. `03339290`에서 되돌렸다.
- 미래 실행 여유0.5→0.3초: run13이18.49m에서25초 정지. knob을 제거하고0.5초를 유지한다.
-6m 복구 목표: 위 동일 배치에서 이득 미확인. 시험 옵션만 남기고 기본4m.
- CAN 조향으로 정지 초기 곡률 보완: 일부 오프라인 기하 가능성만 확인. 기본OFF, 폐루프 수락 아님.

## 추가 요청: 경로 유지와 단계별5개 기준선 대조

`path_selection_trial=1`은 **현재 sweep을 통과한 일반 경로8m 이상**에서만 유지 우선권을 준다.
실제 상태의 속도 계획·정지 여유0.5m를 확인하며 짧은 복구 경로에는 적용하지 않는다.
새 장애물이 생기면 즉시 폐기한다. 과거 유효성을 연장하거나 경고/freshness를 무시하지 않는다.

`path_selection_trial=2`는 같은 native quintic·곡률·조향·sweep·속도 코어로5개 기준선을 먼저
대조한다. 본차로 기준0/±반차로/±한차로이며 도로 경계를 의미하지 않는다. 현재 위치에서
중간 기준선에30cm 이내로 도달했을 때만 본차로 방향으로 다음 단계를 선택한다. 막힌 기준선을
강제로 따르지 않으며 유효한 긴 후보가 없으면 현재 자유 경로 탐색/복구로 내려간다.
두 정책은 이번 비교용이며 기본0이다. 실제 run25 이후 결과는 실험 status와 추가 결과표에서
확정한다. 기존 카메라5선 follower나 제어기를 중복 시작하지 않는다.

## 수정 범위·검사·다음 main 반영

- `src/asmc_planner/src/local`: 입력 선택, 후보/복구/속도 연계, 객체 나이 가설, 시험 정책.
- `src/asmc_planner/tests`: 같은 C++ 코어의 회귀와 실패 fixture.
- `tools/scen_runner`: 시험 gate·날씨 확인·family 누락 주입·native 연속성 집계.
- `src/asmc_planner/web`: 기존 판제 콘솔의 reference·불확실성·준비 hold 표시.
- `tools/perception`: 기존 ASMC 실험 목록의 이번 결과 연결.
- camera/LiDAR 모델·공용 메시지 MD5·기존 NORMAL 행동·controller 모델은 변경하지 않았다.

`ci01/planning.log`는 현재 branch의 committed planning CI를 독립 master에서 실행한 결과다.
NORMAL 회피·로터리·고주로·충돌, LOCAL alignment·경계·인계·소유권·웹 검사를 통과했다.
후속 시험 정책은 `build14/15.log`, `test14/15.log`에서 native 검사를 추가했다. 다른 채팅이 수정한
CI WIP를 이번 변경으로 stage하거나 검사 통과 근거로 사용하지 않았다.

실행은 LOCAL 컨테이너에서 bridge를 공유하되 camera/lidar/autonomy는 해당 컨테이너 한 세트다.

```bash
roslaunch asmc_camera camera.launch
roslaunch asmc_lidar lidar.launch
roslaunch asmc_planner autonomy_no_bridges_v2.launch jamming_planner:=local
```

별도 `local.launch`를 동시에 시작하지 않는다. 기본 jamming 방식은 camera 그대로다.
자율시험은 전용 master·전용 raw bridge·physical lease와 빈 UDP/웹 포트를 확인하며, 사용자
수동11311과 다른 localization 컨테이너는 수정하지 않는다.8088 observer는 run 종료 후 옛
경로를 비우고 종료 상태만 유지한다. 재접속/새 run 연결은 `monitor-browser01`에 검증했다.

원복은 `local_planner_config:=/artifacts/local2h-20261029-validation01/baseline.yaml`로 신규
geometry padding/근접 maneuver 후보를 끄거나, 별도 checkout에서 baseline430effff를 빌드한다.
현재 worktree의 다른 채팅 CI WIP에 reset을 적용하지 않는다. 실패 raw bag·SHA는 고치지 않는다.
정확한 실행 명령·profile/scene은 각 run의 `config.json`, `runtime-provenance.json`과 부모
`reproduce-run.py`에 보존한다.25km/h 달성 대신 현재 검증 길이·연속성과 다음 단일 개선의
근거를 제출한다. **지속 정지, 짧은 가림 검증, 이격과 공급 전체 수락은 아직 남은 한계**다.

## 최종 비교와 배포 요청으로의 연결

기존 main은 origin/main fad73f82이며 이미 LOCAL의 조상이다. 별도 배포 브랜치를 준비하되
원본 실험 branch와 CI 미커밋 파일을 보존한다. 다음 표는 동일 FOGGY13 주차 차량 배치의
폐루프 비교이며 카메라/추적 잡음까지 동일한 녹화 입력인 것은 아니다.

| 정책 | run | 결과 | 실제 정지 | 판단 |
|---|---|---|---:|---|
| 현재 자유 경로 |19|60.22m/53.15s|3.65s|기준선 유지|
| 현재 검증된 긴 경로 유지 우선 |25|60.21m/48.96s|3.51s|효과 작음, 기본OFF|
| 단계별5개 가이드 |26|8.41m 후25초 지속 정지|25.06s|채택 안 함|
| 전방 무게중심+5개 가이드 |27|60.05m/61.93s|6.83s|통과했으나 우월성 없음|
|3뷰 무게중심+5개 가이드 |28|60.33m/51.62s|3.53s|한 배치 차이로 front/all 확정하지 않음|

run26은 카메라가 벽 방향을 보는 자세에서 broad 비노면이 커져 semantic reference 자체를
만들지 못했다. 작은 잡음만의 실패가 아니다. 전방-only는 추가 추론/warp 없이 원래 front
투영 결과를 재사용하며 **방향 선호만** 비교한다. 충돌 검증용3뷰 raster와 LiDAR는 유지한다.
현재 사용 기본은 자유 경로, guidance 비교0, front 전용OFF다.

작은 노면 잡음 guard는 실험OFF다.8–16m,3셀 이하의 고립된 비노면만 대상으로 하고
350ms 이내 이전 실제 clear-road가 필요하다. 지속 비노면·unknown 경계·근접 LiDAR는 veto,
원본 raster는 수정하지 않는다. run29 빈 터널80.15m, run30 steel60.03m를 통과했으나
두 run의 실제 적용 셀 수가0이면 잡음 억제 효과를 입증한 결과가 아니다. 합성 회귀만으로
기본ON하지 않는다. 이 두 run은 후처리와 일부 겹쳐 순수 성능 대조로 채택하지 않는다.

사용자 후속 요청으로 저속 회복 목표를3→5km/h로 변경한다. 이는 강제 최저 속도가 아니다.
공간·조향·실제 감속·충돌 검사는 여전히 더 낮은 속도나 정지를 요구할 수 있다. 최종 배포
검증은 별도 run31 이후와 배포 안내에 기록하며 기존3km/h 결과와 혼동하지 않는다.
LiDAR global fitting0.10m 후보도 main 기본값을 바꾸지 않도록 배포 기본에서 제외한다.
