# D1: Legacy 행동 선택권 / 계산 자산 경계 분석

> **상태:** D1-1 분석 산출물. **코드를 수정하지 않는다.**
>
> **목적:** `frenet_planner.cpp`에서 "Frenet이 스스로 행동을 결정하는 코드"와
> "그 행동을 수행하기 위해 계산하는 코드"를 완전히 분류한다. 실제 코드 경계
> 정리는 D1-2, 자동 전환 제거는 각 모드의 MORAI 검증 통과 후에 한다
> ([fsm-frenet-interface-v2.md](../../contracts/planning/fsm-frenet-v2.md) 10.5절).
>
> **대상 파일:** `src/frenet_planner/src/frenet_planner/frenet_planner.cpp`
> (행 번호는 커밋 `fa530f9` 기준)
>
> **선행 계약:** [fsm-frenet-interface-v2.md](../../contracts/planning/fsm-frenet-v2.md)
> 10절(legacy migration policy), 12.3절(D1/D2 엄격한 범위)

---

## 0. 요약: D0에서 예상한 것과 실제의 차이

계약 문서 10.2절은 "분리 지점 5개가 `if (use_legacy_behavior)` 블록 끝부분에 모여
있다"고 적었다. 이 서술은 **행동 결정 지점에 대해서는 맞지만, legacy gating
전체로는 불완전하다.**

실제로 `use_legacy_behavior`는 **19개 지점**에서 참조되며 성격이 4종류로 나뉜다.

| 그룹 | 지점 수 | 성격 | D1에서의 취급 |
|---|---|---|---|
| **A** | 5 | 행동 결정 (`cmd.mode` 확정 + COMMIT 승격) | 분리 대상 — FSM으로 이관 |
| **B** | 7 | MERGE 실행 컨텍스트 | **보존** — v2 MERGE가 재사용 |
| **C** | 3 | AVOID 실행 컨텍스트 | **보존** — v2 AVOID가 재사용 |
| **D** | 3 | 진단 로그 전용 | 보존 (무해) |

(A의 5개 = `cmd.mode` 대입 4곳 + `merge_.commit_pending` 승격 1곳. 1.1/1.2절에
전부 열거한다. `use_legacy_behavior` 참조 19곳 중 나머지는 선언 1곳,
블록 진입 1곳, 외부 STOP 경로 4곳이다.)

즉 `use_legacy_behavior`를 기계적으로 제거하면 **B/C 그룹의 검증된 실행 로직이
함께 사라진다.** 계약 문서 10.5절이 "모드 단위로 자동 전환만 제거"라고 한 이유다.

### 0.1 "행동 결정 / 계산 자산" 2분류로는 부족하다

D1-1 초안은 700~1006행을 통째로 "계산 자산"으로 분류했다. **이 분류는 위험하다.**
그 구간에는 순수 계산만 있는 것이 아니라, **Frenet이 스스로 대상을 고르는 policy**가
함께 들어 있다.

```cpp
// AVOID: planner가 회피할 장애물을 스스로 선택하고 context를 활성화 (737~769행)
if (FindStaticAvoidanceTarget(...)) {
    avoidance_.active = true;
    avoidance_.obstacle_id = target.id;      // ← 대상 선택 권한
    avoidance_.phase = AvoidPhase::TRACKING;
}

// FOLLOW: planner가 leader를 스스로 획득하고, 더 가까운 차로 자동 handoff (828~858행)
if (nearest_gap + leader_switch_margin < current_gap) {
    following_.leader_id = nearest_leader.id;  // ← 주행 중 대상 교체
}
if (!following_.active && nearest_found) {
    following_.active = true;
    following_.leader_id = leader.id;          // ← 대상 선택 권한
}

// MERGE: zone 진입을 감지해 스스로 활성화 (888~900행)
merge_.active = true;
```

참고로 코드 자신이 이것을 policy라고 부른다 — MERGE 활성화 로그가
`"[FrenetPlanner-MERGE] policy activated: ..."`다(896행). 즉 이 구간이 계산이 아니라
정책이라는 것은 원저자도 인지하고 있었다.

**v2 계약과 정면으로 충돌한다.** 6.1절대로 FOLLOW/AVOID의 대상은
`BehaviorCommand.primary_object_id`가 authoritative다. FSM이
`FOLLOW object_id = 15`를 줬는데 Frenet이 내부에서 17번으로 바꾸면 command
contract가 깨지고, FSM은 자기가 지시한 대상을 추종 중이라고 오인한다.

따라서 분류를 **4단계**로 세분한다.

| 분류 | 의미 | v2에서 |
|---|---|---|
| **A. Behavior selection** | 어떤 maneuver를 할지 결정 | **FSM으로 이관** |
| **B. Legacy acquisition / orchestration** | 어떤 대상을 고를지, 언제 활성화할지 결정 (v1 전용 policy) | **v2에서 사용 금지** — FSM command가 authoritative |
| **C. Reusable planning computation** | 주어진 대상/모드로 수치를 계산 | **재사용** |
| **D. Execution context** | 이미 확정된 기동을 수행 | **재사용** |

구체 예시:

| 코드 | 분류 |
|---|---|
| `cmd.mode = AVOID/MERGE/FOLLOWING/LANE_KEEPING` | **A** |
| `merge_.commit_pending = true` (내부 EXECUTE 승격) | **A** |
| `FindStaticAvoidanceTarget` → `obstacle_id` 자동 선택 | **B** |
| `FindLeader` → nearest leader 자동 획득 / handoff | **B** |
| merge zone 감지 → `merge_.active = true` | **B** |
| 지정된 object ID를 Frenet s,d로 투영 | **C** |
| 장애물 폭으로 avoid offset 계산 | **C** |
| gap / TTC 계산 (`FindRoundaboutGap` 등) | **C** |
| OBB collision (SAT) | **C** |
| candidate generation / cost / selection | **C** |
| COMMIT 잠금 궤적 실행 (566~638행) | **D** |
| 저속 fallback 우회 (`near_any_merge`) | **D** |
| TRACKING 충돌 필터 예외 (1176~1185행) | **D** |

> **B에 대한 주의:** `FindLeader` / `FindStaticAvoidanceTarget`의 **탐색 수학 자체를
> 버린다는 뜻이 아니다.** 함수 안의 차선 판정·거리 계산·투영은 C로 재사용할 수 있다.
> 금지되는 것은 **"Frenet이 알아서 누구를 따라갈지/피할지 고르는 policy"**를 v2
> 경로에서 그대로 쓰는 것이다. D5/D6에서 이 함수들을 호출할 때는 반드시
> `primary_object_id`로 지정된 대상에 한정해야 한다.

전체 참조 위치: 475, 542, 544, 547, 551, 559, 566, 641, 653, 656, 660, 666, 699,
1088, 1176, 1313, 1478, 1496, 1498행.
(475=선언, 641/660/666/1088=외부 STOP 경로, 699=legacy 블록 진입)

---

## 1. 그룹 A — 행동 결정 (분리 대상)

`cmd.mode`를 확정하는 지점. 이것이 FSM으로 넘어갈 **행동 선택권**이다.

### 1.1 우선순위 체인 (1008~1063행)

```
if      (begin_avoidance || AVOID 진행중)  → cmd.mode = AVOID        (1009행)
else if (merge_.active)                    → cmd.mode = MERGE        (1024행)
else if (following_.active)                → cmd.mode = FOLLOWING    (1053행)
else                                       → cmd.mode = LANE_KEEPING (1062행)
```

**AVOID > MERGE > FOLLOW > KEEP** 고정 우선순위다. v2에서는 이 우선순위 판단
자체가 FSM으로 넘어간다 — Frenet은 `maneuver_type`을 받아 그대로 수행한다.

정확한 대입 위치:

| 행 | 대입 | 조건 |
|---|---|---|
| 1009 | `cmd.mode = AVOID` | `begin_avoidance \|\| (avoidance_.active && phase != TRACKING)` |
| 1024 | `cmd.mode = MERGE` | `merge_.active` |
| 1053 | `cmd.mode = FOLLOWING` | `following_.active` |
| 1062 | `cmd.mode = LANE_KEEPING` | 위 셋 모두 false (else 분기) |

### 1.2 내부 COMMIT 승격 (997행)

```cpp
merge_.commit_pending = true;
```

`gap.confirmed && gap.safe && merge_.safe_cycles >= active_confirm_cycles`가
만족되면 planner가 스스로 COMMIT을 예약한다. v2에서는 이 trigger가 FSM의
`EXECUTE`로 이동한다 (계약 문서 5.4절).

### 1.3 전환 조건을 만드는 코드는 계산 자산이다

legacy 블록(699~1063행)은 실제로 네 구획으로 나뉜다.

| 행 범위 | 구획 | 분류 |
|---|---|---|
| 700~736 | AVOID context 갱신 (저장된 장애물 위치 추적, phase 전이) | **C/D** |
| **737~769** | **AVOID 신규 대상 자동 획득** (`FindStaticAvoidanceTarget` → `obstacle_id` 선택) | **B** |
| 770~829 | FOLLOW leader 유지 / dropout grace | **C/D** |
| **826~858** | **FOLLOW leader 자동 handoff + 신규 획득** (`FindLeader` → `leader_id` 선택) | **B** |
| 861~887 | MERGE zone window 판정 (`roundabout_window` / `highway_window`) | **C** |
| **888~900** | **MERGE 자동 활성화** (`merge_.active = true`, 900행) | **B** |
| 901~1006 | gap 평가 + phase 전이 (`FindRoundaboutGap` / `FindHighwayMergeGap`) | **C** (단 **997행 COMMIT 승격은 A**) |
| **1008~1063** | **우선순위 체인 — `cmd.mode` 확정** | **A** |

| 전환 | 조건 변수 | 조건을 채우는 구획 |
|---|---|---|
| → AVOID | `begin_avoidance`, `avoidance_.active/phase` | 700~769 |
| → MERGE | `merge_.active` | 861~1006 |
| → FOLLOWING | `following_.active` | 770~860 |
| COMMIT 승격 | `gap.confirmed/safe`, `merge_.safe_cycles` | 861~1006 |

**핵심:** 조건을 만드는 코드(오른쪽)는 **계산 자산**이고, 그 조건으로 mode를 정하는
코드(1.1절 4개 대입 + 1.2절)만 **행동 결정**이다. D1-2는 이 둘의 경계만 코드에
명시한다.

### 1.4 결정 블록(1008~1063)은 순수 함수에 가깝다 — D1-2가 안전한 이유

이 56행 블록을 실제로 검사한 결과:

| 검사 | 결과 |
|---|---|
| 계산 함수 호출 (`Find*` / `Project*` / `Generate*` / `Filter*` / `Evaluate*` / `Select*`) | **0건** |
| planner 상태 변경 (`avoidance_` / `following_` / `merge_`에 대입) | **0건** — 읽기만 함 |
| 하는 일 | context 상태를 읽어 `cmd`(및 지역변수 `avoid_offset`, `leader_s` 등)에 쓰기 |

즉 이 블록은 **"상태를 읽어 명령 구조체를 채우는" 단일 책임**만 갖는다. 부수효과가
없으므로 D1-2에서 별도 함수로 추출할 때 동작이 바뀔 위험이 낮다.

> **예외 하나:** 1.2절의 COMMIT 승격(997행)은 이 블록 **밖**(MERGE 계산 구획 안)에
> 있고 `merge_.commit_pending`을 **변경**한다. 즉 행동 결정 중 유일하게 부수효과가
> 있는 지점이며, 계산 자산 코드와 물리적으로 섞여 있다. D1-2에서 이 한 줄만은
> 단순 추출이 불가능하므로 별도로 다룬다.

---

## 2. 그룹 B — MERGE 실행 컨텍스트 (보존)

행동 결정 **이전과 이후**에 흩어져 있으며, `cmd.mode = MERGE` 결정과는 별개로
MERGE를 실제로 수행하는 데 필요한 코드다.

| 행 | 내용 | 왜 보존인가 |
|---|---|---|
| 542~546 | `context_highway_zone` / `context_highway_checkpoint` 조회 | 현재 어느 checkpoint를 향하는지. v2 MERGE도 동일하게 필요 |
| 547~553 | `context_conflict_s` / `context_stop_buffer` | roundabout/highway별 conflict 좌표. 계약과 무관한 지도 정보 |
| 559 | `near_any_merge` | 저속 fallback 우회 판정 (2.2절) |
| 566~638 | **COMMIT 잠금 궤적 실행** | 2.1절 |
| 1313~1320 | `legacy_merge` 출력 파라미터 | conflict gap / completion distance / cross speed |

### 2.1 566행 블록은 컨텍스트 조회가 아니라 MERGE 실행 본체다

```
COMMIT 잠금 상태 (merge_.gap_locked && phase == COMMIT)
  → 남은 궤적을 ego 위치 기준으로 잘라냄        (RemainingPath)
  → CTRV+OBB로 재검증                          (CartesianTrajectoryClear)
      → 정지선 전 차단 : WAIT 복귀, 잠금 해제
      → 정지선 후 차단 : COLLISION_BLOCKED (비상정지 요청)
      → 통과           : 그대로 out_path 반환하고 조기 return
  → highway checkpoint 도달 시 다음 checkpoint로 전진
```

v2 MERGE(D8)는 이 로직을 **그대로 재사용**한다. 바뀌는 것은 잠금에 들어가는
trigger(FSM EXECUTE)뿐이다. 이 블록을 건드리면 `711d536`에서 MORAI로 검증한
회전교차로/고속도로 주행이 깨진다.

### 2.2 559행 `near_any_merge`를 지우면 데드락이 발생한다

저속(`|ego.v| < 0.5`) fallback을 우회하는 조건이다. 주석에 이유가 있다.

> 회전교차로에서는 저속이어도 아래의 MERGE FSM을 반드시 거친다. 여기서 조기
> 반환하면 WAIT/COMMIT 잠금보다 먼저 매 cycle 전체 도로가 빌 때까지 검사하게
> 되어 conflict point 앞에서 두 번째 정지와 데드락이 발생한다.

v2 MERGE도 저속으로 conflict point에 접근하므로 동일한 우회가 필요하다.

---

## 3. 그룹 C — AVOID 실행 컨텍스트 (보존)

| 행 | 내용 | 왜 보존인가 |
|---|---|---|
| 653~658 | `tracking_near` / `hold_avoid_target` | 저속에서 회피 목표 d 유지. 없으면 정지 근처 자기잠금 |
| 666 | 저속 fallback 속도 상한 분기 | legacy는 무제한, external은 `resolved_target_speed` |
| 1176~1185 | **TRACKING 장애물 충돌 필터 제외** | 3.1절 |

### 3.1 1176행 블록의 의미

TRACKING(회피를 아직 시작하지 않은 구간)에서는 잠금된 장애물 **하나만** 충돌
필터에서 제외한다. 주석의 이유:

> 8초 충돌 lookahead가 먼 정적 장애물까지 미리 중앙 경로를 전부 제거하면
> stop/go가 발생하므로, 잠금된 해당 장애물만 이 구간의 충돌 필터에서 제외한다.
> 다른 장애물은 계속 필터링되고, 25m 진입 시 잠금 장애물도 즉시 다시 포함된다.

**행동 결정이 아니라 회피 기동의 실행 세부사항이다.** v2 AVOID(D6)의 PREPARE
단계에서 같은 문제가 발생하므로 동일 처리가 필요하다.

---

## 4. 그룹 D — 진단 로그 (무해)

| 행 | 내용 |
|---|---|
| 1478, 1496, 1498 | 충돌 차단 진단 로그에서 `following_.leader_id` 출력 |

`use_legacy_behavior && following_.active`로 가드되어 external 경로에서는 `-1`이
찍힌다. v2에서 FOLLOW context가 채워지면 가드를 조정하면 되고, 지금 손댈 이유는
없다.

---

## 5. 분리 대상이 아닌 것 (오분류 주의)

### 5.1 STOP APPROACH의 `cmd.mode = LANE_KEEPING` (1134행)

```cpp
if (stop_.phase == StopExecutionPhase::APPROACH) {
    ...
    cmd.target_speed = std::min(cmd.target_speed, v_safe);
    cmd.mode = LANE_KEEPING;   // ← 분리 대상 아님
}
```

행동 결정이 아니라 **후보 생성기 위임**이다. STOP의 APPROACH 구간에서는 exact-stop
quintic 대신 velocity-keeping 생성기를 쓰라는 내부 구현이며, 외부에 보고되는
`last_mode_`는 STOP으로 유지된다:

```cpp
last_mode_ = external_stop_mode ? STOP : cmd.mode;
```

**C4에서 완성·검증한 STOP core에 속하므로 그대로 보존한다.**

### 5.2 `cmd.mode` 대입을 일괄 검색해 처리하면 안 되는 이유

성격이 셋으로 갈린다.

| 행 | 대입 | 성격 |
|---|---|---|
| 1009 | `cmd.mode = AVOID` | 행동 결정 (분리) |
| 1024 | `cmd.mode = MERGE` | 행동 결정 (분리) |
| 1053 | `cmd.mode = FOLLOWING` | 행동 결정 (분리) |
| 1062 | `cmd.mode = LANE_KEEPING` | 행동 결정 (분리) |
| 1068 | `cmd = *external_command` | v2 경로 진입점 (유지) |
| 1134 | `cmd.mode = LANE_KEEPING` | STOP APPROACH 내부 위임 (**보존**) |

---

## 6. v2 외부 command가 채워야 할 입력 목록

legacy 경로가 계산해서 `cmd`에 채우던 값 중 v2에서 **누가 채우는가**를 분류한다.
이것이 D2(PlannerCommand 정리)와 D3(변환)의 입력 명세가 된다.

### 6.1 FSM이 주는 값 (BehaviorCommand에서 옴)

| `PlannerCommand` 필드 | `BehaviorCommand` 출처 |
|---|---|
| `mode` | `maneuver_type` + `direction` (계약 2.2절) |
| `has_target_speed` / `target_speed` | `desired_speed_mps` (계약 3.5절) |
| `stop_position` | `target_pose_map` → RefLine 투영 |
| `leader_id` | `primary_object_id` (FOLLOW) |
| **(AVOID 대상 ID — 필드 없음)** | `primary_object_id` (AVOID) — **6.4절 참고** |
| `time_gap` / `min_gap` | `desired_time_gap_s` / `minimum_gap_m` (`use_custom_follow_gap=true`) |
| `merge_type` | `merge_type` |
| `merge_sa_id` / `merge_sb_id` | `preferred_front_object_id` / `preferred_rear_object_id` (선택) |

### 6.2 Frenet이 계산해야 하는 값 (FSM이 주지 않음)

| 필드 | 무엇으로 계산 | 단계 |
|---|---|---|
| `leader_s` / `leader_speed` / `leader_accel` | `primary_object_id`로 객체 조회 → Frenet 투영 | D5 |
| `avoidance_d_offset` | 장애물 폭 + 차량 폭 + 안전 마진 | D6 |
| `merge_conflict_s` / `merge_stop_s` | `merge_type` + 지도 설정(`params.yaml`) | D8 |
| `merge_entry_time` | gap selector | D8 |
| `merge_gap_safe` / `merge_committed` / `merge_crossing` | MergeContext 상태 | D8 |
| `gap_gain` | config 고정값 | D5 |
| `target_speed` (최종) | 곡률/충돌 제약 적용 후 | D4 |

**6.2가 D2의 핵심 근거다.** 이 값들은 "명령"이 아니라 "계산 결과"인데 지금 같은
구조체에 들어 있다. 계약 문서 12.3절이 지적한 입력/결과 혼재가 이 목록이다.

### 6.3 v2에서 쓰지 않는 값

| 필드 | 이유 |
|---|---|
| `has_max_speed` / `max_speed` | `BehaviorCommand`에 대응 필드 없음 (계약 3.5절) |
| `target_lane` | `direction`으로 대체. `ResolveLateralOffset`이 mode로 판단 |

---

### 6.4 AVOID 대상 ID를 받을 필드가 없었다 — D2에서 해소됨

`BehaviorCommand`는 AVOID의 회피 대상을 `primary_object_id`로 준다(계약 3.3절).
그런데 **D1 시점의 `PlannerCommand`에는 이를 받을 canonical 필드가 없었다.**

```cpp
// D1 시점 global.hpp
double avoidance_d_offset = 0.0;  // AVOID 전용이지만 "계산된 횡offset"이지 대상 ID가 아니다
int leader_id = -1;               // FOLLOWING 전용으로 문서화되어 있음
```

즉 AVOID 대상을 **외부에서 지정할 방법이 없고**, `FindStaticAvoidanceTarget`이
자동 선택하는 경로밖에 없었다(0.1절 B 분류).

**D2의 결정: (a) 공통 필드**

```cpp
int target_object_id = -1;   // FOLLOW 선행차 / AVOID 회피 장애물 공용
```

`BehaviorCommand`가 둘 다 `primary_object_id` 하나로 주므로 1:1 대응이
자연스럽고 변환 계층의 분기가 줄어든다. 기존 `leader_id`는 이 필드로 대체되었다
(커밋 `e64faf5`).

**D1에서 기록해야 했던 사실은 이것이다:**

> AVOID 대상 identity는 v2에서 FSM이 소유한다. D6에서
> `FindStaticAvoidanceTarget`의 **자동 선택 로직을 그대로 재사용하면 계약 위반**이며,
> 반드시 `primary_object_id`로 지정된 장애물에 한정해야 한다.

이 사실을 D1에 남기지 않으면, D6에서 기존 자동 선택 경로를 무심코 이어붙일 위험이
크다. 필드는 D2에서 생겼지만, **그 필드를 authoritative로 취급하는 것**은 D6의
책임이다.

---

## 7. D1-2 이후 작업 (이 문서 범위 밖)

D1-1은 분류까지다. 다음 조각:

- **D1-2**: 그룹 A(행동 결정)를 별도 함수로 추출해 경계를 코드에 명시.
  **동작은 바꾸지 않는다** — 순수 리팩터링이며 회귀 테스트로 확인한다.
  1.4절에서 확인한 대로 1008~1063 블록은 부수효과가 없어 추출이 안전하고,
  997행 COMMIT 승격 한 줄만 예외로 별도 처리한다.
- **D2**: `PlannerCommand`를 6.1(입력) / 6.2(계산 결과)로 분리.
- **D5/D6/D8**: 각 모드가 MORAI 검증을 통과한 뒤 해당 그룹 A 조각만 제거
  (계약 문서 10.5절).

### 7.1 D1-2에서 지키는 불변식

```
legacy 경로(3인자 Plan)의 동작이 리팩터링 전후로 동일하다
  → frenet_planner 테스트 9개 전부 통과
  → 특히 command_keep_plan_test / command_stop_plan_test 회귀 없음
```

B/C/D 분류(0.1절)는 D1-2에서도 **건드리지 않는다.** 이 문서의 목적은 그 경계를 다음
작업자(및 나중의 나)가 실수로 넘지 않게 하는 것이다.

### 7.2 B 분류(legacy acquisition policy)의 처리 시점

B는 D1-2에서 **제거하지도, 사용하지도 않는다.** legacy 경로에서는 그대로 동작해야
하고(불변식 7.1), v2 경로는 애초에 이 코드를 타지 않는다(`cmd = *external_command`).

실제 처리는 각 모드 구현 시점이다.

| 단계 | B 처리 |
|---|---|
| D5 FOLLOW | `primary_object_id`로 지정된 대상만 사용. 자동 handoff/획득 경로를 v2에서 호출하지 않음 |
| D6 AVOID | `primary_object_id`로 지정된 장애물만 사용 (6.4절). 자동 선택 금지 |
| D8 MERGE | `maneuver_type=MERGE` 수신 시에만 활성화. zone 자동 감지로 활성화하지 않음 |
| D10 | legacy 경로 삭제와 함께 B 코드 제거 |

각 단계에서 B를 우회할 때, 그 안의 **C 성분(탐색 수학·투영·거리 계산)은 재사용
가능**하다는 점을 잊지 말아야 한다. 버리는 것은 "누구를 고를지 정하는 권한"뿐이다.
