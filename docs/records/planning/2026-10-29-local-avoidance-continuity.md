# LOCAL-2E 전환·장애물 회피 연속성

LOCAL-2D 결과 `776b549e` 이후 사용자 승인으로 반복 시험한다. 기존 일반 판제와
기본 camera 방식은 유지한다. 이 worktree의 속도·IMU **상대측위**와 현재 semantic,
family, LiDAR local 관측을 사용한다. 별도 `research/shadow-localization`의 GPS anchor·
HD맵 DR 구현을 병합하거나 그 worktree/컨테이너를 수정하지 않는다.

## 원인과 변경 경계

- `asmc_controller/src/local/node.cpp`: 준비 경로는 모드 전환 때 유효하더라도 다음
  50ms timer 위상에서 만료될 수 있었다. 실제 인계 시 첫 계산을 실행하고 timer를
  다시50ms에 맞춘다. NORMAL 중 두 번째 MPC를 계산하지 않는다.350ms 원본 freshness,
 100ms 상태 예측, selector250ms watchdog과 소유권 검사는 유지한다. 기존 저속 복구의
 550ms degraded grace는 유지하되 정상 freshness로 계산하지 않는다.
- `asmc_planner/src/local/reference_prior.hpp`, `semantic_reference.hpp`, `node.cpp`:
  family 소실 때 노면의 보이는 폭만으로 기준을 다시 만들면 약1.7m 튈 수 있었다.
  마지막 정상 family의 방향 선호를 기존 상대측위로 정렬해 최대2초/6m/0.2rad까지
  사용한다. source 중복은 나이를 갱신하지 않는다. 현재 노면 조각 안에서만 목표를
  고르며, 과거 선은 통과 가능 공간·현재 family 검출·물리 벽이 아니다. 현재 raster,
  LiDAR hull, 차체 sweep은 독립적으로 검사한다. 세대 변경은 이력 초기화다.
- `asmc_planner/src/local/planner.cpp`: 기존 복구는 경로가 거의 없을 때만 실행됐다.
  유효하지만 현재 속도에서 정지하기에 짧은 경로도 긴 저속 대안을 먼저 탐색한다.
  알려진 비노면/OBB를 유지한 swept 후보를 우선하고, 별도 near-field recovery는
  3km/h·원인 표시·terminal speed 검사를 유지한다. 복구 중 실제 측정 OBB와의
  겹침은 비용만 주고 허용하지 않고 후보를 거부한다. 물리 외곽2.160835×4.635m와
  기본10cm 여유는 줄이지 않았다.
- `tools/scen_runner`: 원태의 owned fixture·lease를 재사용한다. 실제 제어는 독립
  master, 전용 bridge/camera/LiDAR/판제 한 세트다. 다른 lease는 기다리며 종료하지
  않는다. 접촉은 native 표본과 CollisionData를 별도로 기록한다.(-1,0)은 무접촉
  sentinel이며 새 실제 접촉을 성공으로 세지 않는다. bag은 LZ4 무손실·60초 분할이다.

`semantic_reference_fallback`은 현재 기본 OFF이며 등록한 실험 profile에서만 켠다.
저속/미관측 보조의 통과를 정규10km/h 회피 수락이나 독립 위치 정확도로 표현하지 않는다.

## 재현 검사

새 실제 ROS 회귀 `test_local_entry_phase.py`는 idle tick 직후 source age323ms의
모드 전환을6회 재현한다. 첫 명령2–6ms, 이미 만료된380ms 관측은 거부한다.
기존 future·역행·소유권·handoff 검사도 유지한다. 단위 검사에는 source-aged reference
이동/만료/중복·unknown/현재 장애물 보존과 짧은 prefix의 저속 재탐색을 포함한다.

## 단계별 주행 근거

정본은 D `artifacts/local2e-20261029-*`다. 각 run의 registration/runtime-provenance,
원본bag, 제어/경로 전체 수신과 native 상태를 보존한다. 과거 run을 수정하지 않는다.

- drive01: 우측 plastic barrier,10km/h,60m 접촉0, full-brake6. 개선 전 실패 근거다.
- drive02: 좌측 plastic barrier,10km/h,60m 접촉0, full-brake0.
- drive03: 우측 plastic barrier,10km/h,60m 접촉0, full-brake7. 추가 복구 분기를 보완했다.
- drive04: NORMAL→LOCAL→NORMAL235m 접촉0, full-brake0. 첫LOCAL명령8.35ms,
  첫NORMAL25.39ms. active family 상대오차p95 5.78cm, 최대50.78cm로 독립GT가 아니다.
- drive05: 보완 후 우측 plastic barrier10km/h,60m 접촉0, full-brake0.
  계산 critical p95 44.38ms, 유효 경로 최대 공백138.8ms, 최종 명령 공백108.2ms.
- drive06: 우측 plastic barrier12km/h,60m 접촉0. 거의 정지 후 회복한 구간도 보존한다.
- drive07: Steel_Barricade10km/h,60m native/UDP 접촉0, full-brake0.

일반 구간 최고속도와 LOCAL 상한을 혼동하지 않는다. 통합 시험의NORMAL 속도는 기존
정책이며 전체 최고40km/h는 LOCAL10km/h 성능이 아니다. 회피 중 본차로에서 떨어지는
것은 의도한 동작이므로 통과 후 복귀와 빈 터널 추종을 구분한다.

## 늦게 확인한 복귀 비용 결함

80m 우측 시험 drive16은 접촉·full brake 없이 진행했어도 본차로에서 약1.9m 떨어진
이전 회피 경로를 계속 유지했다. family의 본차로 근거와 독립 native 고정 경로가
같은 방향을 가리켰다. 원인은 이전 경로 비용0.20이 본차로0.04보다5배 크고 전체
계획 범위에 일정하게 적용되던 구조다. `7beb57f6`은 이전 경로의 영향을
`exp(-max(0,x)/3m)`로 줄여 가까운 부분의 연속성은 유지하고 먼 부분에서 본차로
복귀가 우세하게 만든다. analytic/lattice/rechecked 후보의 공통 비용과 탐색 heuristic,
복구의 이력 비용에 같은 감소를 적용했다. 충돌·곡률·조향 변화율 검사는 유지한다.
과거 우회 경로를 넣은 고정 native 시험은 도로가 비었을 때 다시 본차로를 선택하며,
모든 선택 구간이 현재 full-body sweep를 통과하는지 검사한다.

목표점도 노면 조각의 가장자리에 놓지 않는다. 현재 관측한 구간에 차체 폭이 들어가는
지점으로 clamping한다. side 후보는 각 transition마다 번갈아 평가해 직진12개가
analytic 예산을 먼저 소진하지 않도록 했다. 예산40ms와 동일한 기하/비용 검사는
유지하며 trace에 각 후보의 lateral target과 transition을 남긴다.

## 수동 검수와 복구

이 worktree를 연결한 `asmc-local2c-20261028`의 새 Bash는 공유11311과 자기 빌드를
사용한다. 공용 bridge, 이 컨테이너의 camera/LiDAR, autonomy 한 세트만 실행한다.
기본 camera 방식과 아래 명시적 LOCAL 실험을 동시에 시작하지 않는다.

```bash
roslaunch asmc_planner autonomy_no_bridges_v2.launch jamming_planner:=local local_planner_config:=/artifacts/local2d-20261029-validation02/semantic-reference-trial.yaml
```

위 profile은 등록·동결한 실험 입력이며 모델/지도 정본을 대체하지 않는다. 현재
실험 profile의 true인 항목은 semantic 방향 reference fallback이다. 기본 패키지
설정은 false이므로 기본 LOCAL 명령만으로 실험 후보를 켰다고 해석하지 않는다.
기존 방식 복구는 다른 autonomy를 종료한 뒤 기본 launch(`jamming_planner:=camera`)를
사용한다. 동시 제어·만료 관측·무효 geometry를 허용하는 복구는 하지 않는다.

결과·원본RGB/노면/경로·이전 실패는
[LOCAL-2E 실험 검수](http://localhost:8000/local2e-20261029-validation01/index.html)에서 본다.

## 최종 결과와 남은 한계

19회 실제 시험에서 총2,061.35m를 주행했고 각 run이 설정 거리에 도달했다. 관측된
native 접촉과 기록한 실제 UDP 접촉은0이다. 이를19회 정규 회피 수락으로 표시하지
않는다. 초반 실패,12km/h의 transient braking,3km/h 복구와 미관측 보조도 보존한다.
기본속도10km/h를 유지하며15km/h 확대·운영 기본 승격은 하지 않았다.

| 최종 소스 시험 | 결과 | full-brake(최종 wire) | 유효 경로 최대 공백 | 계산 critical p95 |
|---|---|---:|---:|---:|
| drive17 · 우측 barrier10km/h,80m | 회피·복귀, 접촉0 | 0 | 160.0ms | 46.0ms |
| drive18 · barrier+반대편drum10km/h,60m | 연속 회피·복귀, 접촉0 | 0 | 166.3ms | 43.4ms |
| drive19 · NORMAL→LOCAL→NORMAL235m | 양방향 인계·빈 터널, 접촉0 | 0 | 164.8ms(LOCAL 활성만) | 43.7ms |

이 세 run의 움직이기 시작한 뒤 연속 정지(≤0.36km/h)는 관측되지 않았다. native
표본 약10Hz의 측정 한계이며 정지 가능성이 절대 없다는 보장은 아니다. 이전 drive06의
약0.83초 정지와 drive01/03/08/09/12의 제동은 삭제하지 않았다.

복귀는 실제 native 차량 위치를 고정 경로의 **선분**에 독립 투영해 확인했다.
마지막2초의 최대 거리: drive17 0.026m, drive18 0.026m. 빈 터널 drive19의 활성구간
p95는0.077m다. 지도/paint의 survey GT가 아니라 live family와 독립적인 기준이다.
초기 nearest-vertex 결과는25cm 샘플 양자화를 포함해 약12cm 바닥값이 있었다.
그 JSON도 그대로 남기고 `native-segment-review.json`을 현재 지표로 사용한다.
중간 선분 점의 오차0 회귀로 양자화 문제를 검출한다. native pose·고정 경로는
평가에만 사용했고 LOCAL 제어 입력으로 넣지 않았다.

진입 source age323ms와4ms 미래 상태를 포함한 실제 ROS 검사에서 첫 명령은약2–6ms,
미래 상태로 계산0·이미380ms인 관측 거부를 확인했다. 실제 drive15의 진입/복귀는
10.9/45.5ms, 양방향 첫150ms STOP0이었다. drive14의 wire STOP3회는 음영 인계가
아니라 초기 selector UNKNOWN 중 AUTO를 켠 시험기 readiness race였다. 시험기는
신선한 실제 wire 명령까지 기다리도록 수정했고 drive15/19에서는 전체 wire STOP0이다.

12km/h는 아직 미수락이다. drive12는 접촉 없이 진행했지만 terminal braking 위반
2명령과60m 시점 복귀 지연이 남았다.4m 복구 경로를 넘어 제동 궤적이 나가는 경우
그 경로를 유효하게 꾸미거나 source freshness를 늘리지 않았다. 다음 속도 확장은
이 고정 사례의 조기 회피·복귀와 실제 제동 거리를 확인한 뒤 진행한다.

최종 소스 planning CI 그룹(일반8개+LOCAL native/실제ROS/ownership/handoff/clock),
추가 readiness·collision·공급 계측, gallery/catalog와 카메라 IMU 투영 검사가 통과했다.
결과 로그는 등록 run 안에 보존하며 hosted CI 성공으로 표현하지 않는다. shared11311
대화형 환경·자기 worktree 빌드와 단일 실행 경계도 마감 시 확인했다.

### 적용 범위

- 새/변경 계산: `asmc_planner/src/local`와 `asmc_controller/src/local`.
- 실행/평가/재현: `tools/scen_runner`, 갤러리 목록은 `tools/perception`.
- 이번 단위에서 LiDAR 검출·tracking·fusion, 공용 메시지/MD5, 모델·engine,
  일반 판제 행동·고정 충돌 여유·GPS validity 정책은 변경하지 않았다.
- 별도 localization worktree는 보호했다. 이번 상대측위를 global DR/HD맵 위치측위와
  같은 방식으로 부르거나 두 컨테이너를 섞어 성능을 집계하지 않는다.
- semantic 방향 후보는 기본 OFF다. 명시한 실험 profile에서 검수하며,
  저속 복구/미관측 보조를 포함한 현재 결과를 일반 운용 수락으로 확대하지 않는다.

후속 우선순위는 **12km/h terminal braking 고정 사례에서 초기에 충분한 회피 공간을
선택하는지 확인하고, 복구에 의존하는 구간을 줄이는 것**이다. 현재10km/h 후보와
별도 localization 방식은 같은 배치·입력 한계·실제 복귀 지표로 비교할 수 있다.

최종 drive19에는 실제 GPS0 표본214개와 정상 좌표183개가 기록됐다. LOCAL은 GPS를
구독하지 않고, 진입/복귀 mode와 일반 구간만 기존 경계를 사용한다. 시각화·CI를
실제 제어 경로에 넣지 않았다. 임시 시험 노드만 정리하고 다른 worktree는 유지했다.
