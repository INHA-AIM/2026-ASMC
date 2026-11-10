# LOCAL-2B: 경로 길이·선택·속도 안정화와 실패 갤러리

LOCAL-2A 이후 PR #126의 GPS 독립 실험 경로에서 진행했다. 운영 NORMAL/map 및
기존 BBox 음영 제어기는 수정·동시 실행하지 않았다. 목표25km/h는 유지하지만 실제
허용속도는 관측 범위와 제동 조건으로 제한한다. **빈 터널·일부 구간의 결과이며 전체
터널/장애물 회피/25km/h 수락이 아니다.**

- 시작 revision: `53d30883`. 경로·속도: `bc463f00`, 입력 교체 경계: `1bd58131`.
- 근거: D 정본 `artifacts/local2b-20261028-audit01`, `local2b-20261028-drive01`~`drive18`.
- 각 run은 입력 SHA·소스 patch/미추적 소스·실행 바이너리/설정 SHA·원본 bag·판단·제어·
  simulator native 접촉·원복 결과를 보존한다. drive08은 중복 제어 발행자 때문에 시작 전
  차단됐다. 이를 통과로 세지 않는다. D→G SensorData는 변경하지 않았다.
- [ASMC 실험 목록](http://localhost:8000/)의 **LOCAL-2B 주행·실패 검수** 바로가기,
  [대조 보고서](http://localhost:8000/local2b-20261028-audit01/campaign/index.html)에서
  시도별 그래프와 실패 RGB/semantic/경로를 본다.

## 차선이 보이는데 경로가 짧아지는 이유

1. 이전에는3경계 공통 support 끝에서 차량 앞부분을 또 빼므로, 보조 왼쪽 선이 짧거나
   전방 family 근거가 줄면 우측 본차로 경로도 불필요하게 짧아졌다. 이제 role2/3 우측
   본차로 reference support와 물리적으로 검증된 grid/OBB sweep을 분리했다.
2. 근거리 coverage128/0은 카메라 자차 가림·미관측이다. 격자 중심이 실제 점유 차체
   끝보다1~4cm 벗어나도 이를 미래 미관측 충돌로 판단했다. 점유 차체 경계에는 반 셀
   양자화만 허용하며, 미래 차체는 실제 관측 근거를 계속 요구한다.
3. frame 간 현재 차체 바로 앞은 과거에는 보였지만 지금 가려질 수 있다. 앞서 관측된
   노면을 상대 이동으로 정렬해 이 근거리만 보조한다. 이력 없는 초기 시험은 기존3km/h
   회복 상태로 시작한다. **이 시작 구간을 완전 관측·고속 수락으로 세지 않는다.**
4. drive05의 plan83, drive11의 plan80에서는 속도별 추가 semantic 여유 안에 들어온
   자차 가림 가장자리의 비노면 셀 때문에 약15m 경로가0.3~0.4m로 줄었다. 차체+최소
   10cm+격자 반 대각선은 hard 검사로 유지하고, 그 바깥 속도별 추가 여유는 비용으로
   변경했다. 실제 비노면이 hard 범위에 있거나 LiDAR 객체가 있으면 계속 차단한다.
5. drive05 plan49에는 빈 터널인데 LiDAR가 긴 띠 모양 객체를 발행했다. track4710/4711/
   4715/4720은 길이4.18~7.27m, 높이0.124~0.214m, 중심 z 약−0.40~−0.43m였다.
   이 프레임의 기하는 천장보다는 노면 잔여점 가능성을 지지한다. 원본 점군/전처리 대조가
   필요하며, planner에서 이를 임의 삭제하지 않았다. 바로 다음 scan은 filtered-empty다.
6. source가279ms 된 묶음은 계산28ms와 다음 제어 tick을 거치면350ms를 넘을 수 있다.
   현재 계산 시작은250ms 이하의 최신 완료 묶음으로 제한한다. 오래된 source를 새
   header로 갱신하지 않는다. producer 지연 자체가 길면 대기는 실패로 남는다.

차선의 가시거리와 차체 전체가 통과할 수 있는 거리는 다르다.20m grid라도 차량 앞부분
약3.845m, source→계획 사이 이동, 최소 여유를 적용하면 실제 후륜축 경로는 대체로
14~16m다. 이를25m로 외삽하지 않았다.25km/h에서1.5m/s² 순수 제동거리만 약16.1m이고
실행 지연·양의 가속·jerk까지 포함해야 하므로, 현재 범위에서25km/h를 유지하는 것은
이 시험의 정지거리 계약과 맞지 않는다.

## 구현과 책임

```text
src/asmc_planner/src/local/
  planner.cpp/.hpp       reference·sweep·후보 연속성·속도 계획
  road_memory.hpp       근거리 실제 노면의 제한된 과거 근거
  motion.hpp            상대 시각 이력 조회(새 위치 추정기 아님)
  node.cpp              입력 snapshot·bounded 대기·진단
src/asmc_controller/src/local/node.cpp
  기존 MPC 연결·명령 교체·local 가속 한계
src/asmc_{planner,controller}/config/local_*.yaml
  local 실험 설정/복구
src/asmc_planner/web/
  dashboard_v2.js, local_scene_renderer.js
tools/scen_runner/ (아래 설명)
```

공용 메시지/MD5·추론 모델·family 피팅·LiDAR 전처리/추적/fusion·NORMAL 알고리즘은
변경하지 않았다. 추가 ROS 노드나 별도 벽 RANSAC도 만들지 않았다.

### 관측 노면 이력과 경로 연속성

- 최대50관측·5초·6m·yaw 차0.2rad. 근거리 원본 x−1.2~6.5m, |y|≤2.5m만 조회한다.
- coverage255·확률≥204인 실제 노면을5×5셀로 erosion한 근거만 사용한다. 새로운
  비노면이 과거 노면보다 우선하며, 현재 관측된 비노면을 덮어쓰지 않는다. LiDAR hull도
  항상 별도로 검사한다. 관측 나이는 원본 시각에서 계산한다.
- session/config/시각/상태 reset에서 이력을 비운다. debug의 청회색은 과거 실제 노면이고
  현재 프레임의 새로운 확률 예측이 아니다. 이동 객체가 가림으로 들어오는 경우까지
  수락한 근거는 아니며 동적 시험이 남는다.
- 이전 경로를 현재 상대 좌표에 정렬하고 전체 sweep을 다시 검사한다. 비슷한 비용·진전이면
  유지하여 후보 교체를 줄인다. 검사하지 않은 과거 경로를 그대로 재발행하지 않는다.
- raster index는 strict/회복 계산에서 한 번만 만든다. family 추론을 웹을 통해 받지 않는다.

### 속도와 제어

- 기존 SpeedProfiler를 재사용하되, 내려간 cap의 순간 target과 실제 jerk 적분 상태를
  구분하여 다음 step에 실제 적분 상태를 Commit한다. 공유 NORMAL 구현은 그대로다.
- 현재 양의 가속에서1.5m/s² 제동까지의 jerk 구간을 적분한다. 순항 상한은 양의 가속
  envelope와0.35m/s 추종 여유로 안정화한다. 이0.35는 높이·폭 값이 아니다.
- local reference와 MPC solve의 양의 가속 상한은1m/s²다. NORMAL의 pedal 정규화
  파라미터를 바꾸지 않고 기존 MPC의 solve별 제한을 사용한다.
- 새 명령 계산은 source≤350ms·상태 예측≤100ms를 지킨다.114ms IMU cadence나
  새 계획이 한 tick 늦는 경우 마지막 정상 solve 이후≤150ms만 **발행 없이** 기다린다.
  기존250ms selector watchdog을 유지하며, 상태 source/수신150ms 초과·지속 중단·
  새 session/무효 geometry는 기존 무효 처리다. 신호를 새롭게 만들거나 정지 정책을
  전부 해제한 것이 아니다.

## 반복시험

아래 집계는 native 주행 구간만 사용한다. 준비 단계·회복·누락·중복 실행 차단은 원본에
남긴다. `trial-result.closed_loop_acceptance=false`는 전체 LOCAL-1 수락을 아직 하지
않았다는 뜻이다. 일부 빈 터널 통과와 모순되지 않는다.

| 시험 | 설정 / 실측 최고 km/h | 거리 | native 접촉 | 급제동 명령 tick | family 횡오차 p95 | 계산 p95 |
|---|---:|---:|---:|---:|---:|---:|
| drive05 | 20 / 20.12 | 65.4m | 0 | 33 | 6.71cm | 33.68ms |
| drive12 | 20 / 19.37 | 65.3m | 0 | 0 | 7.34cm | 41.52ms |
| drive13 | 20 / 19.43 | 65.0m | 0 | 0 | 7.19cm | 32.29ms |
| drive14 | 25 / 19.33 | 65.2m | 0 | 0 | 7.86cm | 31.07ms |
| drive16 | 15 / 15.93 | 65.1m | 0 | 2 | 7.26cm | 30.38ms |
| drive17 | 20 / 19.13 | 65.0m | 0 | 0 | 7.30cm | 31.78ms |
| drive18 | 25 / 19.16 | 65.3m | 0 | 0 | 7.12cm | 30.38ms |

최종 drive16/17/18의 같은 위치10m에서 경로 교체량 p95는 각각1.09/1.06/1.10cm다.
source/state 대기 중에는 명령을 새로 만들지 않는다. 최대 명령 공백은 각각99.97/100.02/100.32ms다.

**미해결:** drive16 plan57/scan175238에 노면 높이의 긴 OBB4개(track5860~5863)가
나와 경로15.42→2.95→15.43m가 됐다. 높이0.129~0.204m, 길이4.34~7.11m,
base 중심z−0.376~−0.381m다.2tick 급제동은 한 차례 실패이며 제거하지 않았다.
이때 live trace와 재실행 모두 터널 추가 필터는 ran=false였다. 터널 추가 벽 처리의
직접 실패로 단정할 수 없으며 기본 지면 분리·수평화·순간 자세를 함께 조사해야 한다.


횡오차는 family 우측 본차로에 대한 내부 일관성이고 독립 측량 GT 오차가 아니다.
**정속 수락도 미완료다.** 급제동0은 속도 진동0을 뜻하지 않는다. drive17의10~13초에는
유효 경로와 `ok` 제어 상태에서도 약19→12→18km/h 변화가 있었다. 이 구간 brake는
최대 약0.05이며, 공간 단축/인지 누락과 분리해 속도 프로파일·MPC의 가감속 응답을
다음 local 단위에서 대조해야 한다. 과거 근거와 hard/soft 여유의 동적 장애물 수락도 남는다.
목표25를 설정한 시험도 현재 여유에 따라 실제18~19km/h대로 제한된다. 원래 초기20km/h
실패 drive05의 급제동33tick과 이후 실패를 함께 공개한다.

## 시각화·재현

`tools/scen_runner/scen_runner/records/`에 다음을 둔다.

- `local_gallery.py`: source stamp가 정확히 일치하는 RGB/semantic 위 후보·선택 경로·
  차체 sweep·OBB. 실패 제어의 실제 plan sequence와 가장 짧은 실패 경로를 우선 선택한다.
- `local_campaign.py`: 실측 속도·경로 길이·탐색 한계·속도 상한·급제동의 인터랙티브 대조.
  미완료 run을 조용히 빼지 않고 준비 실패도 별도 기록한다.
- `analyze_local_path_stability.py`: bag의 exact ego_at_plan으로 직전 경로를 현재 기준계에
  옮긴 뒤 같은5/10m 위치에서 경로 교체량을 측정한다. native 주행 구간·같은 session만 비교한다.
- 기존 `analyze_local_following.py`: family 기준 횡오차와 실제 속도·제어 사유.

과거 동결 run은 수정하지 않는다. LOCAL-2A 갤러리/대조 산출물은 새 audit01에 저장했다.
`tools/perception/build_experiment_gallery_hub.py`는 기존 번호를 보존하고 **로컬 경로·주행·
실패 검수** 분류와 바로가기를 추가한다. 판제 웹8088과 실험 목록8000은 역할이 다르다.

```bash
# 컨테이너: 수동 주행 관찰용 계산/제어 preview, 자동 제어 기본OFF
source /root/ws/devel/setup.bash
roslaunch asmc_planner local.launch
# 기존 semantic_space 노드가 없을 때만 별도 실행
# roslaunch /root/ws/tools/scen_runner/launch/semantic_space.launch
```

기록 도구는 원본 bag 읽기와 새로운 등록 run의 출력으로 실행한다.

```bash
PYTHONPATH=tools/scen_runner python3 -m scen_runner.records.local_gallery /artifacts/<run>
PYTHONPATH=tools/scen_runner python3 -m scen_runner.records.analyze_local_path_stability /artifacts/<run>
PYTHONPATH=tools/scen_runner python3 -m scen_runner.records.local_campaign /artifacts/<new-audit>/campaign /artifacts/<trial1> /artifacts/<trial2>
```

## 검사·복구·남은 일

- C++ local 코어: reference 분리, 좌표 왕복, 실제 노면 이력/새 비노면 우선/만료/reset,
  OBB 우선, hard/soft 여유, 정지거리, sweep 회귀.
- 독립 master: source 만료 뒤 계산 금지·명령 재발행 없음·지속 중단·기존 ownership,
  IMU100ms 예측 한계·역행·재시작·최신INVALID 보존·signed 후진 관측 유지.
- 기존 controller reference/timing/stop continuity 및 웹 motion/local/map 분리 회귀.
- 실험 목록의 LOCAL-2B 새 링크·17회 주행 선택/그래프·인계 다운로드는 HTTP200/JS오류0.
  별도 기존 학습/연구2개 run의 정적 링크30개 누락은 보고서에 보존했고 동결 자료를 수정하지 않았다.
- hosted CI는 이번 로컬 작업에서 실행하지 않았다. 실제 검사 로그와 SHA는 audit01에 둔다.

내부 rollback은 `causal_road_memory:false`로 이력 보조를 끄는 비교와 시작 revision checkout이다.
`speed_cap_kph`는 시험 속도 상한이며 학습 모델이나 NORMAL 운영 설정을 바꾸지 않는다.
선호 여유를 비용으로 처리한 의미를 단순 임계값 튜닝으로 숨기지 않는다.

다음 작은 단위는 **가감속 시 노면 잔여 OBB/가림 경계와 단일 장애물의 구분**이다. 원본 점군
→지면 분리→군집→OBB와 IMU 시각을 같은 scan으로 대조하고, 얇은 실제 장애물이 제거되지
않는 대조 fixture를 만든다. 빈 터널 성공만으로 box/drum/barrier 회피나 터널 전체를 수락하지
않는다. 벽 간격은 LOCAL-2A의 측정 자료를 사용하되 고정 통로폭으로 바로 적용하지 않는다.

## 인지 담당 인계

[인지 실패 인계 갤러리](http://localhost:8000/local2b-20261028-perception-handoff01/index.html)와
D:`artifacts/local2b-20261028-perception-handoff01`에 원본 bag4개·정확한 plan/scan·
기록 trace·IMU·소스/설정·재현 명령을 보관했다. drive16/05는 LiDAR 잔여 OBB,
drive11은 카메라 노면 가장자리와 planner 추가 여유의 상호작용, drive15는 입력 교체
시각 사례다. 후자 둘은 LOCAL-2B에서 경계 처리를 수정했고 인지 자체 결함으로 확정하지 않았다.
카메라 family 자체가 크게 튀었다는 별도 결함은 이 시험에서 입증하지 않았다.

전달용 폴더 전체 약801MiB(0.782GiB). SHA256SUMS와 source_bags_sha_match 검사를 포함한다.
정본 원본을 삭제·옮기지 않았고 담당자에게 자동 전송하지 않았다.
