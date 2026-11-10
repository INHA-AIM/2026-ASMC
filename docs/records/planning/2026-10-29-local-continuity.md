# LOCAL-2D 공급·전환·폐루프 검증

후속 반복 개선은 [LOCAL-2E 기록](2026-10-29-local-avoidance-continuity.md)을 따른다.
아래 LOCAL-2D 당시의 실패·수락 보류는 역사적 근거로 보존한다.

기준 main은 `7bb4d9a1`(PR131/132/133), LOCAL 리베이스 완료는 `de33475d`다.
백업과 초기 검사는 [준비 기록](2026-10-29-local-rebase.md)에 보존한다.
이 문서는 그 이후의 실행 정리·수락 판단 정본이다. 운영 `camera` 기본값과 다른
worktree는 변경하지 않으며, 이번 작업에서 push·강제 push·병합은 하지 않는다.

## 구현 위치와 책임

| 위치 | 변경 |
|---|---|
| `asmc_controller/src/local/ownership_watch.hpp` | master 조회를 전용 스레드로 분리. 1초·200ms·동시1개, 캐시1.25초 만료, 실패/중복 시 발행 차단 |
| `asmc_controller/src/local/node.cpp` | 타이머는 캐시만 조회. 실제 명령 이력·미래 관측 거부 유지. solve 후에도 원본350ms freshness 검사 |
| `asmc_planner/src/local/node.cpp` | 입력 참조·유한 이력·세대만 짧게 복사. 정렬/노면 기억/계획/상세 JSON은 잠금 밖. 계산 상태 단일 소유·발행 세대 재검사 |
| `asmc_planner/src/local/planner.{cpp,hpp}` | Scene 이동 전달로 추가 전체 복사 제거. 이전 경로도 현재 노면/OBB/운동학으로 검사 |
| `asmc_planner/src/Planning/GpsModeSource.*` | LOCAL 선택에서만 기존 진입30m 전 경로 준비. 실제 JAMMING 조건 유지 |
| `asmc_planner/src/Planning/ControlSelector.*` | 준비/이전 실제 명령 유효 시 새 소스 명령을 최대100ms 기다림. 옛 명령 재발행/시간 갱신 없음 |
| `asmc_planner/src/local/preparation_limit.hpp`, `PlanningPort.*`, `V2Cycle.hpp` | LOCAL 준비 구간에서 기존 NORMAL 속도 계획에 상한 전달. 기존 곡률·제동·jerk 계산을 재사용. camera 선택·다른 구간 비적용 |
| `asmc_camera/launch/semantic_space.launch` | 작은 행렬의 BLAS worker를1개로 제한. 원본 projection context·IMU 보정·해상도·모델은 유지 |
| `tools/scen_runner` | 독립 master/소유 PID·UDP 검사, 실제 입력 준비 후 AUTO, 모든 경로/명령 수신 시각·RSS·모드/준비/selector 기록 |

350ms 관측 freshness,100ms 상태 전개,250ms selector watchdog을 늘리지 않았다.
275ms 된 완전한 묶음에는 남은 시간에 맞춰 탐색 예산을 줄이고, source age300ms를
넘겨 계산이 끝난 결과는 발행하지 않는다. 제어20Hz 여유50ms를 남기는 정책이며
과거 결과 재발행이나 새 stamp 부여가 아니다. 이전282.88ms는 **계획 판단 기록 간격**이다.
실제 명령 공백과 혼동하지 않는다.

## 시험 환경과 원인 분리

수동은 공유11311, 자동시험은 독립 master다.11348이 이미 열린 경우11400부터
빈 포트를 선택한다. 포트 선택 후 시작 경쟁도 실제 master PID로 검출한다.
`drive06`은 사용자가 연 dashboard와11348 시작이 겹쳐 **차량 변경 전 차단**됐다.
상대 master/dashboard를 종료하지 않고 다음 run에서11400을 사용했다.

LOCAL 전용 컨테이너는 `asmc-local2c-20261028`, `/root/ws`는
`/workspace/2026-ASMC-work`다. 컨테이너 CPU2개 제한에서는 노면 투영이87–104ms,
도착한 노면은 이미 약300ms가 지난 상태여서 경로를 준비하지 못했다. 같은 계산의
단독 시험은 중앙값 약14ms였다. 이 컨테이너만 CPU6개로 바꾸고 BLAS1 worker를
사용하자 정상 구간 투영은19–24ms, source age는약84–139ms가 됐다.
두 변경을 동시에 적용한 실시간 결과이므로 BLAS만의 개선량으로 주장하지 않는다.
다른 컨테이너의 quota·마운트·Bash·엔진 포인터는 변경하지 않았다.

## 실제 정지 원인과 수정

- `drive02`: 시험 도구가 노면 생산자를 시작하지 않은 준비 결함. 실행 경로에 연결하고
  실제 유효 경로가 오기 전에는 AUTO로 전환하지 않도록 수정했다.
- `drive03`: CPU 제한 아래 투영 지연.350ms를 늘리지 않고 위 환경에서 재측정했다.
- `drive04`: 빈 터널60.17m, 접촉0, 중앙 속도9.94km/h, family 횡오차p95 0.117m.
  초기 저속 복구36계획이 포함돼 전체 단위 수락으로 집계하지 않는다.
- `drive05`: NORMAL이약30.90km/h로 LOCAL에 인계됐다. 검증된 짧은 공간에서 정지할
  수 없어 LOCAL이 거부했다. 준비 중 NORMAL의 기존 속도 프로파일에 LOCAL 상한을
  전달한 `drive07`에서는 진입 속도9.77km/h였다. 일반 속도 정책은 유지한다.
- 본차로에 사용하지 않는 반대편 외곽선의 접힘이 전체 scene을 거부했다. 그 보조선만
  제외하고, 본차로 중앙/우측선의 비유한·뒤접힘 검사는 유지했다.
- 진입부의 producer `too_few_observed_roles`/`insufficient_local_multi_role_support`는
  실제 `valid=false`·경계0개다. 입력 지연이나3뷰 fusion 계산 공백과 구분한다.
- barrier 앞에서는 새 family의 근거리 support가5.64m부터 시작해
  `family_disconnected_support`로 정지했다. 물리 충돌 여부와 방향 reference의 근거
  범위를 분리해야 하는 사례이며 원본 bag과 실패 gallery를 보존한다.

## 노면 방향 reference 후보

`asmc_planner/src/local/semantic_reference.hpp`는 family가 무효/접힘/근거리 지원
부족일 때 **현재 노면에서 방향 선호만** 만드는 실험 후보다. 기본 설정은 OFF다.
전용 run의 `semantic-reference-trial.yaml`로만 대조한다.

- producer의 family를 valid로 바꾸거나 옛 family를 새 관측으로 쓰지 않는다.
- 같은 시각의 IMU projection·노면·local OBB·속도/IMU 정렬을 사용한다. GPS/정답 pose
  입력이나 새 노면 투영/재추론을 추가하지 않는다.
- 관측된 연속 노면 조각에서 우측1.75m의 약한 선호를 구한다. 한 차로 폭의 lateral
  선호 범위는 먼 측방 패치를 방향으로 오인하지 않기 위한 조건이며 물리 ROI가 아니다.
- 노면은 수정하지 않는다. reference의 빈 구간 연결이 통과 가능 공간을 생성하지
  않는다는 회귀를 포함하며, 실제 경로는 기존 차체 sweep·OBB·미관측·운동학 검사를 받는다.
- 후보가 사용되면 최대5km/h, 기존 near-field recovery는3km/h다. 각각 사유를 기록한다.
  family가 정상화되면 기존 본차로 reference/10km/h 상한을 사용한다.
- source/raw family 유효성·reference 출처를 trace에 남긴다. 이 후보의 통과와 정규
  10km/h 회피 수락은 구분한다.

`drive08` 고정 bag에서 유효 계획71→196으로 늘었으나 저속 복구116개가 포함됐다.
`drive09`는 왼쪽 단일 barrier를 접촉 없이60.15m 통과·복귀했다.
`drive10`은235.48m NORMAL→LOCAL→NORMAL 통과, 진입/복귀 첫 명령15.2/40.6ms,
두 전환 직후150ms의 full STOP0이었다. 이는 첫 성공이며 반복 수락은 별도 기록한다.

## 검증과 재현 자료

결과 정본은 D `artifacts/local2d-20261029-*`다. 각 run은 입력/설정 SHA·소스 revision과
실행파일 SHA를 남긴다. 초기 코드·수정 코드·저장된 LiDAR 객체와 새 LiDAR 재처리를
구분한다. `validation01`의 최초 OFF/ON6회와 `validation02`의 수정 후 검사를 섞지 않는다.

최근100계획 요약 외 전체 trace는 bounded 비동기 writer에 저장한다. 전체 delivery로
경로·MPC·최종 명령 공백을 구하고,1Hz 진단 수를 공급률로 사용하지 않는다. 유효 계획이
사라진 채 시험이 끝난 경우도 `driving_valid_plan_max_silence_ms`에 포함한다.
초기/종료 공백과 실제 주행 구간은 둘 다 보존한다. 메모리 증가와 warmup도 구분한다.

현재 local CI planning 그룹의 핵심8검사, LOCAL 실제 ROS 입력/소유권/전환 검사와
지연 master200ms 검사가 통과했다. 추가 후보와 최종 설정에 대한 검사·반복 주행은
완료 표에 연결하며, hosted CI 성공으로 표현하지 않는다.

일반 실행은 [명령 안내](../../operations/commands.md#구성-요소별-local-음영-주행-선택)를 따른다.
기본 `jamming_planner:=camera`와 별도 LOCAL 선택을 혼동하지 않는다.

## 최종 경계 보완

경로가 첫 odometry보다 먼저 오면 같은 frame인데도 버리던 결함을 수정했다
(`18f19a1b`). 시험도 plan-first 순서를 강제한다. 다른 frame·역행·미래 입력
거부는 유지하며, sleep만 늘리는 방식으로 실패를 덮지 않았다.

MPC의50ms 격자 제약만으로는25ms 뒤 실제 인계 명령을 제한하지 못했다.
LOCAL은 실제 발행 이력으로 마지막 조향을 제한하고 다음 MPC도 그 실제 명령으로
시작한다(`91dc0123`). 양방향 selector의 첫 인계 명령에는 canonical controller
YAML의 최소 변화율을 보수적 경계로 적용한다(`3afbe0df`). 일반 후속 명령·페달·
기본 camera 선택은 바꾸지 않는다. 새 명령이 없으면 옛 명령을 재발행하지 않는다.

## 최종 측정과 수락 판정

[ASMC 실험 목록의 LOCAL-2D 검수](http://localhost:8000/local2d-20261029-validation02/index.html)에
실제 경로·RGB·노면·실패와 전체 수치를 연결했다. 기본 OFF와 실험 profile을 구분한다.

| 항목 | 측정 | 판단 |
|---|---|---|
| 입력 잠금 p95, OFF/ON6회 | 0.058–0.063ms | 2ms 이내 |
| 계산 임계 경로 p95, 고정 bag | OFF28.52–29.45ms / ON27.10–28.31ms | 관측 비교에서 악화 없음. 음수 개선량을 보장하지 않음 |
| 상세 직렬화 p95 | OFF0.041–0.043ms / ON0.29–0.32ms | 2ms 이내 |
| 유효 경로 최대 수신 공백, 고정 bag6회 | 143.2–208.0ms | 250ms 이내. 초기/종료 무효는 별도 집계 |
| 원래257.1ms 재현 | 초기 run에 보존, 수정 후6회 모두 위 범위 | 실패를 삭제하지 않음 |
| 새 runtime 전환 `drive23/25/26` | 각235m 접촉0, 첫 LOCAL명령28.7–50.0ms, 첫 NORMAL명령14.2–33.4ms | 제어 공급·조향 경계 확인 |
| 인계 첫 조향 변화율 | 약0.10rad/s | 기존 최대1rad/s보다 보수적. 수신 시각 오차 별도 |
| 빈 터널 family 상대 횡오차 | 일부 최대0.70–1.26m | **최대0.40m 조건 미달**. 독립 GT 오차 아님 |
| 오른쪽 회피 `drive15/17` | 접촉 없이 통과했으나 유효 경로 공백307.8/287.2ms | **250ms 조건 미달** |
| 좌/우 각3회 | `9/13/14`, `15/16/17` 거리60m·접촉0·복귀 관찰 | 3km/h/미관측 보조 포함, 정규 회피 수락 대체 불가 |
| 최종 전환 정지 명령 | `drive23/25`0, `drive26`2회 | **0 STOP 조건 미달** |
| 메모리/기록 | native RSS는 warmup 후 유한 cache에서 plateau, journal drop0 | 장시간 누수 부재를 일반화하지 않음 |

`drive26` 준비 경로 source age는 진입 약313ms였다. 첫 LOCAL tick에서363ms가
되어 `local_source_stale`로 두 차례 full brake를 발행했고 다음 신선한 경로에서
복귀했다. 실제 완전 정차·접촉은 없었지만 전환 수락 실패다. readiness의 시각 여유와
첫 계산/제어 위상을 맞춰야 한다. stamp 갱신이나350ms 연장은 사용하지 않는다.

`drive24`는 기록기 short write(40051B 요청,308B 기록)로 중단됐다. C/D 여유는 약
98/80GiB였고 별도16MiB 쓰기·읽기 SHA 검사는 통과했다. 원인은 확정하지 않았다.
원본 bag은10,434개 메시지 뒤 읽기 오류가 있으며 재색인/덮어쓰지 않았다. 갤러리는
읽을 수 있는 prefix와 오류를 표시한다. 이후 기존 시나리오 수집과 동일한 LZ4 무손실
기록으로 `drive25/26` 두 번 정상 종료했다. 압축이 근본 I/O 결함을 해결했다고 주장하지 않는다.

`drive21/22`는 다른 worktree의 simulator lease를 확인해 차량 변경 전 차단했다.
이후 시험기는 최대300초 소유권 대기를 사용했으며 다른 실행을 종료하지 않았다.

로컬 planning CI 그룹 전체(기존8개 + 실제 ROS LOCAL/소유권/인계/시각/이력), camera
공용 투영12개, 공급·reference 출처/미발행 제동 집계2개, gallery4개와 catalog4개가
통과했다. 기존 pre-refactor145검사 보존 검사도 통과했다. hosted CI는 실행하지 않았다.

**LOCAL-2D 전체 수락은 보류다.** 공급 리팩토링·명확한 수신/인계 결함 수정은 유지하되
노면 reference 후보는 기본 OFF, 운영 기본 `camera`는 유지한다. 실험 재현은 등록된
`validation02/semantic-reference-trial.yaml`을 `local_planner_config`에 명시한다.
일반 수동 명령은 `jamming_planner:=local` 하나이며 후보가 자동 켜지지는 않는다.

다음 작은 단위는 **LOCAL-2E: 준비 경로 시각 여유·첫 계산 위상과 reference 전환
안정화**다. `drive26` 진입, `drive15/17` 짧은 prefix를 고정 회귀로 사용한다.
정지하지 않은 통과나3km/h 복구만으로 완료하지 않고, 두 reference의 실제 지지와
유효 경로 공백·본차로 오차를 함께 확인한다. 새 모델·추적기는 도입하지 않는다.

작업 말미 원격 main은 `2e961209`(PR134)로 바뀌었으나 차이는 CI 전용이다.
검증 runtime 기준 `7bb4d9a1`은 유지했고 PR 게시 전 최신 CI 통합을 따로 확인해야
한다. 이번 단위에서 push·병합·운영 승격은 하지 않았다.
