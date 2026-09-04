# FSM ↔ Frenet 인터페이스 v2 계약 (D0)

> **상태:** 계약 고정 문서. 이 문서는 `BehaviorCommand` / `FrenetFeedback` 기반
> v2 인터페이스의 유일한 기준(source of truth)이다. D1 이후의 모든 커밋은 이
> 문서에 정의된 계약을 구현하며, 계약을 바꾸려면 코드보다 이 문서를 먼저 고친다.
>
> **범위:** 이 커밋(D0)은 문서만 추가한다. production 알고리즘 코드는 한 줄도
> 바꾸지 않는다.
>
> **수정 대상 경계:** 이 프로젝트에서 수정하는 것은 **Frenet Planner와 MPC 코드뿐이다.**
> `BehaviorCommand.msg` / `FrenetFeedback.msg`는 **FSM 개발자가 정의한 원본을 그대로
> 사용하며 Frenet 측에서 수정하지 않는다.** FSM 패키지 코드도 수정하지 않는다.
> 11절의 논의 항목은 FSM 개발자에게 확인을 요청하는 목록이며, 합의되지 않으면
> 메시지는 원본 그대로 유지하고 Frenet 측 구현으로 흡수한다.
>
> **선행 문서:** [planning-behavior-handoff.md](./planning-behavior-handoff.md)는
> `BehaviorContext`/`PlanFeedback` 기반 v1 규약이며 legacy reference다. 두 규약은
> cutover 전까지 병행 운영된다(10절).

---

## 0. 왜 v2인가

v1(`BehaviorContext` / `PlanFeedback`)은 "이번 사이클에 어떤 모드로 주행할지"만
전달하는 **stateless 명령**이었다. 그래서 다음을 표현할 수 없다.

- 하나의 차선변경을 "준비 → 실행 → 완료"라는 **하나의 단위**로 묶는 것
- Frenet이 "후보는 만들었지만 아직 실행하면 안 된다"고 답하는 것
- FSM이 "이제 실행해도 된다"고 **허가**를 주는 것
- 이미 시작한 기동을 취소할 수 있는지 여부를 되돌려 받는 것

v2의 핵심 전환은 두 가지다.

1. **`command_id`는 피드백용 일련번호가 아니라 maneuver transaction의 identity다.**
   한 번의 기동이 시작해서 완료·실패·취소로 끝날 때까지 같은 ID를 유지한다.
2. **`execution_request=EXECUTE`는 "실행하라"가 아니라 "COMMIT을 허가한다"이다.**
   Frenet은 EXECUTE를 받아도 현재 상황으로 안전성을 **재검증**한 뒤에만 COMMIT한다.

---

## 1. Canonical messages

### 1.1 소유권

| 메시지 | 소유 패키지 | 경로 |
|---|---|---|
| `BehaviorCommand` | `asmc_msgs` | `src/asmc_msgs/msg/BehaviorCommand.msg` |
| `FrenetFeedback` | `asmc_msgs` | `src/asmc_msgs/msg/FrenetFeedback.msg` |

**두 메시지를 다른 패키지에 복사하지 않는다.** ROS에서 `asmc_msgs/BehaviorCommand`와
`behavior_planner/BehaviorCommand`는 내용이 같아도 서로 다른 타입이며, 같은 토픽에
연결되지 않는다.

현재 저장소에 남아 있는 `src/behavior_planner/msg/BehaviorContext.msg`,
`src/behavior_planner/msg/PlanFeedback.msg`는 **빌드되지 않는 죽은 파일**이다
(`src/behavior_planner/CMakeLists.txt`는 `UrbanFeatureDebug.msg`만 등록하고,
`behavior_node.cpp`는 `asmc_msgs::BehaviorContext`를 publish한다). 혼동을 막기 위해
D3에서 함께 삭제한다.

### 1.2 토픽 (D3에서 개설)

| 방향 | v1 (유지) | v2 (신규) |
|---|---|---|
| Behavior → Frenet | `/behavior/context` | `/behavior/command_v2` |
| Frenet → Behavior | `/planning/feedback` | `/planning/frenet_feedback_v2` |

ROS1은 한 토픽에 두 메시지 타입을 실을 수 없다. cutover 전까지 두 계통을 **병행**
운영하여, 현재 MORAI에서 검증된 KEEP/STOP 주행을 깨지 않는다.

### 1.3 좌표계·단위

- `header.frame_id = "map"` (양방향 고정)
- 속도 `m/s`, 거리 `m`, 시간 ROS Time
- 무효값 sentinel: 객체 ID `-1`, 실수 `-1.0`, 문자열 `""`, 목표 pose `has_target_pose=false`
- **예외:** `current_d`, `target_d`는 실제로 음수가 유효하므로 `-1.0`을 무효값으로
  쓰지 않는다. 유효성은 각각 `frenet_state_valid`, `target_d_valid`로 판단한다.

---

### 1.4 모든 모드가 공통으로 채우는 최소 필드

D0에서 확정한다. 어떤 모드든, 성공이든 실패든 **이 세 필드는 항상 유효한 값이어야
한다.** FSM이 피드백을 신뢰할 수 있는 최소 계약이다.

| 필드 | 규칙 |
|---|---|
| `command_id` | 처리 중인 `BehaviorCommand.command_id`를 그대로 echo. 처리 중인 명령이 없으면 `0` |
| `active_maneuver` | 2.3절의 역매핑 결과. 활성 maneuver가 없으면 `MANEUVER_NONE`(255) |
| `execution_state` | 5.5절의 전이에 따른 현재 상태. 처리 중인 명령이 없으면 `IDLE` |

여기에 실무상 함께 묶이는 두 필드를 더한다.

| 필드 | 규칙 |
|---|---|
| `command_accepted` | 명령의 형식·필수 필드가 정상인지. 궤적 생성 성공 여부와 **무관** |
| `result_code` | 정상이면 `RESULT_NONE`, 아니면 구체 사유 (3절 / 8절) |

`command_id` echo가 특히 중요하다. FSM은 "현재 `command_id`와 일치하는 피드백만
FSM에 반영"하므로, echo가 틀리면 피드백 전체가 무시된다.

나머지 필드(gap, progress, leader, candidate count 등)는 해당 모드를 구현하는
단계에서 채우며, 그때까지는 msg가 정의한 sentinel을 쓴다(9.4절).

---

## 2. Enum 매핑 (ROS ↔ 내부)

**이 절은 v2에서 가장 사고 위험이 큰 부분이다.** 두 enum은 번호 체계가 완전히
다르므로 `static_cast`가 절대 금지된다.

### 2.1 두 체계

| 의미 | `BehaviorCommand.maneuver_type` | 내부 `BehaviorState` |
|---|---|---|
| 차선 유지 | `KEEP` = 0 | `LANE_KEEPING` = 0 |
| 선행차 추종 | `FOLLOW` = 1 | `FOLLOWING` = 1 |
| 합류 | `MERGE` = **2** | `MERGE` = **8** |
| 차선 변경 | `LANE_CHANGE` = **3** | `LANE_CHANGE_LEFT` = **2** / `LANE_CHANGE_RIGHT` = **3** |
| 회피 | `AVOID` = **4** | `AVOID` = **7** |
| 정지 | `STOP` = **5** | `STOP` = **9** |
| 긴급정지 | `EMERGENCY_STOP` = **6** | `EMERGENCY` = **10** |
| (내부 전용) | — | `INTERSECTION_WAIT`=4, `TURN_LEFT`=5, `TURN_RIGHT`=6 |

내부 enum 순서는 `src/frenet_planner/src/frenet_planner/global/global.hpp`의
선언 순서에서 온다.

> **`static_cast<BehaviorState>(msg.maneuver_type)`는 금지한다.**
> 이 캐스트를 하면 `MERGE`(2)가 `LANE_CHANGE_LEFT`로, `AVOID`(4)가
> `INTERSECTION_WAIT`로, `STOP`(5)이 `TURN_LEFT`로 실행된다.

### 2.2 정방향 (command → internal)

`LANE_CHANGE`만 1:2 매핑이며 `direction`으로 갈린다.

```
KEEP            → LANE_KEEPING
FOLLOW          → FOLLOWING
MERGE           → MERGE
LANE_CHANGE + DIRECTION_LEFT   → LANE_CHANGE_LEFT
LANE_CHANGE + DIRECTION_RIGHT  → LANE_CHANGE_RIGHT
LANE_CHANGE + DIRECTION_NONE   → INVALID_COMMAND (3절 참조)
AVOID           → AVOID
STOP            → STOP
EMERGENCY_STOP  → EMERGENCY
그 외 값        → INVALID_COMMAND
```

### 2.3 역방향 (internal → feedback)

`FrenetFeedback.active_maneuver`는 `BehaviorCommand.maneuver_type`과 **같은 숫자
체계**를 쓴다. 따라서 별도의 역매핑 함수가 필요하다.

```
LANE_KEEPING                        → KEEP
FOLLOWING                           → FOLLOW
MERGE                               → MERGE
LANE_CHANGE_LEFT, LANE_CHANGE_RIGHT → LANE_CHANGE   (2:1 축약)
AVOID                               → AVOID
STOP                                → STOP
EMERGENCY                           → EMERGENCY_STOP
INTERSECTION_WAIT, TURN_LEFT, TURN_RIGHT → MANEUVER_NONE (255)
활성 maneuver 없음                   → MANEUVER_NONE (255)
```

좌/우 정보는 `active_maneuver`에서 사라진다. FSM은 자신이 보낸 `direction`을
알고 있고 `command_id`로 대응 관계를 확인할 수 있으므로 손실이 없다.

### 2.4 필수 회귀 테스트 (D3)

- 정방향: 7개 maneuver × 유효 direction 조합이 모두 의도한 내부 값이 되는지
- 역방향: 내부 11개 값이 모두 의도한 ROS 값 또는 `MANEUVER_NONE`이 되는지
- **round-trip**: `LANE_CHANGE`를 제외한 6개는 command → internal → feedback이
  원래 값으로 돌아와야 한다
- **negative**: `static_cast`와 결과가 다름을 명시적으로 확인 (MERGE/AVOID/STOP)

---

## 3. Validation matrix

v2 adapter의 첫 관문. 통과하지 못하면 planner를 호출하지 않는다.

### 3.1 공통 (모든 maneuver)

| 항목 | 조건 | 실패 시 `result_code` |
|---|---|---|
| `header.frame_id` | `== "map"` | `INVALID_COMMAND` |
| `command_id` | `!= 0` | `INVALID_COMMAND` |
| `valid_until` | `!= 0` (`isZero()` 아님) | `INVALID_COMMAND` |
| `valid_until` | `now <= valid_until` | `STALE_COMMAND` |
| `maneuver_type` | 정의된 값 (0~6) | `INVALID_COMMAND` |
| `execution_request` | 정의된 값 (0~2) | `INVALID_COMMAND` |
| `direction` | 정의된 값 (0~2) | `INVALID_COMMAND` |
| `merge_type` | 정의된 값 (0~2) | `INVALID_COMMAND` |
| `desired_speed_mps` | `isfinite` 이고 (`== -1.0` 또는 `>= 0.0`) | `INVALID_COMMAND` |
| 객체 ID 3종 | `>= -1` | `INVALID_COMMAND` |

`valid_until` 검사는 `STALE_COMMAND`로 **별도 구분**한다. 명령 자체는 정상인데
전달이 늦은 것이므로 FSM이 재전송으로 회복할 수 있는 상황이기 때문이다.

### 3.2 FOLLOW gap 필드

| `use_custom_follow_gap` | `desired_time_gap_s` | `minimum_gap_m` |
|---|---|---|
| `false` | `== -1.0` | `== -1.0` |
| `true` | `isfinite` 이고 `> 0.0` | `isfinite` 이고 `> 0.0` |

### 3.3 maneuver별

| maneuver | `execution_request` | `direction` | `has_target_pose` | 객체 ID | `merge_type` |
|---|---|---|---|---|---|
| `KEEP` | `EXECUTE` | `NONE` | `false` | 전부 `-1` | `MERGE_NONE` |
| `FOLLOW` | `EXECUTE` | `NONE` | `false` | `primary >= 0`, 나머지 `-1` | `MERGE_NONE` |
| `STOP` | `EXECUTE` | `NONE` | **`true`** | 전부 `-1` | `MERGE_NONE` |
| `EMERGENCY_STOP` | `EXECUTE` | `NONE` | `false` | 전부 `-1` | `MERGE_NONE` |
| `LANE_CHANGE` | `PREPARE`/`EXECUTE`/`CANCEL` | **`LEFT` 또는 `RIGHT`** | `true` | 전부 `-1` | `MERGE_NONE` |
| `AVOID` | `PREPARE`/`EXECUTE`/`CANCEL` | `NONE`/`LEFT`/`RIGHT` | 선택 | `primary >= 0`, 나머지 `-1` | `MERGE_NONE` |
| `MERGE` | `PREPARE`/`EXECUTE`/`CANCEL` | `NONE`/`LEFT`/`RIGHT` | **미정 — 11절 Q1** | `primary=-1`, front/rear 선택 | `ROUNDABOUT`/`HIGHWAY` |

- `LANE_CHANGE`에서 `DIRECTION_NONE`은 거부한다. 어느 쪽으로 갈지가 없으면
  내부 enum을 결정할 수 없다(2.2절).
- `MERGE`가 아닌 모든 maneuver는 `merge_type == MERGE_NONE`이어야 한다.
- `MERGE`는 `merge_type != MERGE_NONE`이어야 한다.

### 3.4 target pose 검증 (`has_target_pose=true`일 때)

1. `position.x/y/z` 모두 `isfinite`
2. quaternion 4성분 모두 `isfinite`, 노름이 1에서 허용오차 이내
3. reference line 투영 성공 (`ProjectPointToS`) — 실패 시 `TARGET_PROJECTION_FAILED`
4. **heading 검증**: 목표 pose의 yaw와 투영점의 reference yaw 차이가 임계값 이내

heading 검증(4)은 신규 항목이다. 현재 투영 방어는 `s_hint` +
`projection_search_window`(100 m) + `max_lateral_offset`(5 m)뿐이라, 왕복 구간이
5 m 이내로 나란히 지나가면 **진행 방향이 반대인 구간에 목표가 붙을 수 있다.**
초기 임계값은 넉넉하게 잡고(30°~45°) MORAI에서 검증 후 조인다.

이 검증은 D3의 adapter/projection 계층에서 추가한다. **C4에서 완성한 STOP core
(`StopContext`, APPROACH/BRAKING/HOLD)는 수정하지 않는다.**

### 3.5 `desired_speed_mps` 매핑

`BehaviorCommand`의 속도 필드는 하나뿐이고, 내부 `PlannerCommand`는 둘
(`has_target_speed`/`target_speed`, `has_max_speed`/`max_speed`)이다.

```
desired_speed_mps < 0.0  →  has_target_speed = false          (Frenet 기본 목표속도)
desired_speed_mps >= 0.0 →  has_target_speed = true
                            target_speed     = desired_speed_mps
항상                     →  has_max_speed    = false
```

`has_max_speed`/`max_speed`는 v2 계약에 대응 필드가 없으므로 **v2 경로에서는
쓰지 않는다.** 새 계약에 없는 필드를 억지로 메시지에 추가하지 않는다. 두 필드는
legacy/internal 경로를 위해 `PlannerCommand`에 그대로 남긴다.

곡률·충돌·가속도에 의한 추가 감속은 planner가 계속 적용하며, 그 결과는
`FrenetFeedback.applied_speed_mps`로 보고한다(8.4절).

---

## 4. Command identity

### 4.1 정책

`BehaviorCommand.msg`의 정의를 그대로 따른다.

> 한 행동이 시작해서 완료·실패·취소될 때까지 유지되는 ID.
> 같은 행동을 반복 발행할 때는 동일 ID 사용.
> 행동 종류나 목표가 변경되면 새로운 ID 사용.

따라서 판정은 다음 네 가지다.

| 상황 | 처리 |
|---|---|
| 새 `command_id` | 새 maneuver transaction 시작 |
| 같은 ID + `PREPARE` → `EXECUTE` | 정상 진행 (동일 transaction) |
| 같은 ID + `EXECUTE` → `CANCEL` | 취소 요청 (7절) |
| 같은 ID인데 immutable 필드가 변경됨 | **`INVALID_COMMAND`** — 기존 context는 보존 |

마지막 항목이 중요하다. 같은 ID로 목표가 바뀌면 "이 transaction이 무엇이었는지"가
무너진다. 이때 기존 context를 리셋하면 진행 중이던 기동이 조용히 다른 기동으로
바뀌므로, **거부하되 기존 상태는 유지**한다.

### 4.2 Immutable 필드 (fingerprint 구성)

같은 `command_id` 안에서 바뀌면 안 되는 필드 — 총 13개.

```
maneuver_type
direction
merge_type
source_link_id
target_link_id
has_target_pose
target_pose_map            (has_target_pose=true일 때만 비교)
desired_speed_mps
primary_object_id
preferred_front_object_id
preferred_rear_object_id
use_custom_follow_gap
desired_time_gap_s
minimum_gap_m
```

### 4.3 Mutable 필드 (identity에서 제외)

```
header.stamp        매 발행마다 갱신됨
valid_until         매 발행마다 갱신됨
execution_request   PREPARE → EXECUTE → CANCEL 전이가 정상 동작
reason              디버그 문자열, 판단에 사용하지 않음
```

**`execution_request`를 fingerprint에서 반드시 제외해야** `PREPARE`에서
`EXECUTE`로의 정상 전이가 같은 transaction으로 이어진다.

### 4.4 부동소수 비교

`desired_speed_mps`, `desired_time_gap_s`, `minimum_gap_m`, `target_pose_map`은
실수이므로 정확한 동등 비교를 하지 않는다. 허용오차 기반으로 비교하며, 기본값:

- 속도·gap: `1e-6`
- pose position: `1e-3` m
- pose orientation: `1e-3` (quaternion 성분)

FSM이 같은 값을 재발행할 때 부동소수 표현이 동일할 것으로 가정하지 않기 위함이다.

### 4.5 필수 회귀 테스트 (D3)

```
PREPARE  id=42  target A   →  EXECUTE id=42  target A   ⇒ 동일 identity (통과)
PREPARE  id=42  target A   →  EXECUTE id=42  target B   ⇒ INVALID_COMMAND
         id=42             →           id=43           ⇒ 새 transaction
같은 ID, header.stamp/valid_until/reason만 변경          ⇒ 동일 identity (통과)
```

---

## 5. PREPARE / EXECUTE / CANCEL

### 5.1 의미

| 요청 | 의미 | Frenet이 하는 일 | Frenet이 하지 않는 일 |
|---|---|---|---|
| `PREPARE` | 준비 지시 | 목표 투영, gap 탐색, 후보 생성, 안전성 검증 | **궤적 COMMIT 금지** |
| `EXECUTE` | **COMMIT 허가** | 현재 상황으로 재검증 후 COMMIT | 무조건 실행 |
| `CANCEL` | 취소 요청 | 가능하면 취소, 불가능하면 거부 사유 회신 | 이미 COMMIT된 것을 되돌림 |

`KEEP` / `FOLLOW` / `STOP` / `EMERGENCY_STOP`은 항상 `EXECUTE`이므로 이 handshake를
거치지 않는다. `MERGE` / `LANE_CHANGE` / `AVOID`만 `PREPARE → EXECUTE` 흐름을 탄다.

### 5.2 PREPARE에서 무엇을 publish하는가

이것이 v2에서 기존 개발과 가장 크게 다른 지점이다.

```
"요청 maneuver의 후보 궤적"   ≠   "이번 사이클에 MPC로 보낼 궤적"
```

`MERGE + PREPARE`에서 후보 생성과 gap 판정이 모두 성공해도, 그 MERGE 궤적을
MPC로 보내면 안 된다. 그러면 FSM의 EXECUTE 허가 없이 기동이 시작된다.

PREPARE 중 MPC로는 **현재 안전한 유지 궤적**(KEEP 또는 holding)을 계속 보낸다.

피드백은 다음과 같이 나뉜다.

| 필드 | PREPARE 성공 시 값 | 의미 |
|---|---|---|
| `command_accepted` | `true` | 명령을 받아들였다 |
| `plan_valid` | `true` | 요청 maneuver의 새 유효 후보를 만들었다 |
| `execution_state` | `READY` | EXECUTE를 보내도 좋다 |
| `trajectory_source` | `TRAJECTORY_NEW` 또는 `REUSED` | **현재 MPC로 보낸 것은 유지 궤적** |

`plan_valid`와 `trajectory_source`가 서로 다른 축이라는 점이 핵심이다(8.3절).

### 5.3 EXECUTE에서 prepared candidate를 믿지 않는다

```
t = 10.00  PREPARE  → gap safe 판정
t = 10.30  EXECUTE
```

이 0.3초 사이에 후방 차량이 접근할 수 있다. 따라서 EXECUTE는 저장된 후보를 그대로
실행하는 것이 아니라:

```
EXECUTE 수신
  → 현재 객체 snapshot으로 안전성 재검증
  → 여전히 안전  ⇒ COMMIT, execution_state = COMMITTED
  → 안전하지 않음 ⇒ COMMIT 안 함, execution_state = WAITING
                    (command_accepted=true, plan_valid=false)
```

EXECUTE가 안전하지 않다고 판단되어도 이는 **실패가 아니다.** FSM은 계속 EXECUTE를
재발행할 수 있고, 조건이 회복되면 COMMIT된다. `FAILED`로 보고하지 않는다.

### 5.4 MERGE의 COMMIT 소유권 이전

기존 MERGE 자산(`MergeContext`, APPROACH/WAIT/COMMIT/CROSS/CLEAR, gap/TTC/locked
path)은 **버리지 않는다.** 알고리즘은 그대로 두고 COMMIT의 **trigger만** 옮긴다.

```
[현재]  내부 gap-safe 판정 + safe_confirm_cycles  →  commit_pending  →  COMMIT
[v2]    내부 gap-safe 판정 + safe_confirm_cycles  →  READY (COMMIT 안 함)
        FSM의 EXECUTE 수신 + 재검증 통과          →  commit_pending  →  COMMIT
```

즉 `commit_pending`을 세우는 조건에 "EXECUTE 허가"가 추가되는 것이지, gap 판정
로직 자체를 다시 만들지 않는다. 이 작업은 M0~M4에서 수행한다.

### 5.5 execution_state 전이

```
IDLE
 └─ 새 command 수신
     ├─ KEEP/FOLLOW/STOP/E-STOP  → EXECUTING → COMPLETED / FAILED
     └─ MERGE/LANE_CHANGE/AVOID
         PREPARING
          ├─ 후보/gap 미확보     → WAITING  ⇄ PREPARING
          └─ 안전 후보 확보      → READY
              └─ EXECUTE + 재검증 통과 → COMMITTED → EXECUTING → COMPLETED
              └─ EXECUTE + 재검증 실패 → WAITING
         (어느 단계에서든 CANCEL → 7절, 명령 결함 → FAILED)
```

---

## 6. Terminal latch

### 6.1 문제

20 Hz로 피드백을 보내는 중 `COMPLETED`를 한 tick만 내고 즉시 `IDLE`로 돌아가면,
FSM이 그 한 프레임을 놓쳤을 때 기동이 끝났다는 사실을 영영 알 수 없다.

### 6.2 정책

`COMPLETED` 또는 `FAILED`에 도달하면, **새 `command_id`가 올 때까지** 그 terminal
상태를 계속 보고한다.

```
command_id=20  execution_state=COMPLETED
command_id=20  execution_state=COMPLETED
command_id=20  execution_state=COMPLETED     ← 같은 ID가 재발행되어도 유지
...
command_id=21 수신
command_id=21  execution_state=PREPARING     ← 새 ID에서만 latch 해제
```

latch 중에도 `trajectory_source`는 계속 실제 값을 보고한다. 기동은 끝났지만 차량은
계속 주행하므로 궤적 출력이 멈추면 안 된다.

### 6.3 해제 조건

- 새 `command_id` 수신 (유일한 정상 해제)
- `EMERGENCY_STOP` 수신 (7.3절 preemption)

같은 ID의 재발행, `execution_request` 변경으로는 해제되지 않는다.

---

## 7. Command preemption과 CANCEL

### 7.1 CANCEL 정책

| 현재 `execution_state` | CANCEL 수신 시 |
|---|---|
| `PREPARING` / `WAITING` / `READY` | 취소 성공 → `IDLE`, context 정리 |
| `COMMITTED` / `EXECUTING` | **거부** → `result_code = TOO_LATE_TO_CANCEL`, 잠긴 궤적 계속 실행 |
| `COMPLETED` / `FAILED` | 무시 (이미 종료) |

`TOO_LATE_TO_CANCEL`을 회신할 때도 `command_accepted = true`다. 명령 자체는
정상적으로 수신·해석되었고, 다만 취소가 불가능한 시점이라는 뜻이다.

### 7.2 COMMITTED 중 새 command

`COMMITTED` / `EXECUTING` 상태에서 **다른** `command_id`의 일반 maneuver가 오면
`TOO_LATE_TO_CANCEL`과 동일하게 처리한다. 교차로 한가운데서 기동을 바꾸는 것이
가장 위험하기 때문이다.

### 7.3 EMERGENCY_STOP만 preempt 가능

`EMERGENCY_STOP`은 `execution_state`와 무관하게 **언제든** 기존 transaction을
중단시키고 즉시 실행된다. 잠긴 MERGE 궤적도 폐기한다.

```
COMMITTED (MERGE, id=30)
  + EMERGENCY_STOP (id=31) 수신
  → id=30 을 FAILED로 종료
  → merge locked path 폐기
  → id=31 즉시 실행
```

---

## 8. EMERGENCY_STOP / trajectory reuse / feedback 축

### 8.1 EMERGENCY_STOP 정지 목표의 소유권

**현재 코드의 가정이 새 계약과 충돌한다.**

`path_generator.cpp`의 `case EMERGENCY:`는 "FSM이 emergency 상황에 맞는
`stop_position`을 설정해준다고 가정"하고 `GenerateStoppingCandidates(start,
cmd.stop_position, cfg)`를 호출한다. 그러나 `BehaviorCommand`는
`EMERGENCY_STOP`에서 `has_target_pose = false`(목표점 없이 즉시 긴급정지)라고
명시한다. 그대로 연결하면 `stop_position`이 NaN인 채로 후보 생성기에 들어간다.

**결정: 정지 목표 계산의 소유권은 Frenet에 있다.**

```
EMERGENCY_STOP 수신
  → 후보 최적화 파이프라인 생략
  → 현재 ego 속도/상태 기준으로 즉시 감속 궤적을 직접 생성
  → 모든 샘플에서 v >= 0
  → 마지막 샘플 v = 0
```

후보 생성 → 곡률 필터 → 충돌 필터 파이프라인을 태우면 "긴급정지인데 후보가 0개라
아무것도 못 보냄"이 발생할 수 있다. 긴급정지에서 이는 허용되지 않는 실패 방향이다.

구현은 C4에서 이미 검증한 **BRAKING fail-safe 감속 궤적 생성 로직을 별도
generator로 재사용**한다(`frenet_planner.cpp`의 STOP BRAKING 실패 경로). 새 수학을
만들지 않는다. 이 작업은 **D4**에서 수행한다.

### 8.2 대상 객체 소실 (`TARGET_OBJECT_LOST`)

`FOLLOW`의 `primary_object_id`나 `AVOID`의 회피 대상이 관측에서 사라진 경우다.
perception dropout과 실제 소실을 구분해야 한다.

기존 core에는 이미 `FollowingContext.missing_cycles`와
`following_cfg_.dropout_grace_cycles` 기반의 유예 로직이 있다. v2에서도 이를
그대로 사용하며, 판정은 다음과 같다.

| 상황 | `result_code` | `execution_state` |
|---|---|---|
| 유예 사이클 이내의 일시적 미관측 | `RESULT_NONE` | 유지 (`EXECUTING`) |
| 유예 사이클 초과 | `TARGET_OBJECT_LOST` | `FAILED` |
| 관측되었으나 차선/방향/범위를 벗어남 | `TARGET_OBJECT_LOST` | `FAILED` |

`TARGET_OBJECT_LOST`는 명령 결함이 아니므로 `command_accepted = true`다. FSM이
새 `command_id`로 다른 maneuver를 지시하는 것이 정상 회복 경로다.

이 정책은 F3(FOLLOW dropout) / A0~A2(AVOID)에서 구현한다.

### 8.3 `plan_valid`와 `trajectory_source`는 별개의 축

| 필드 | 질문 |
|---|---|
| `plan_valid` | 이번 사이클에 **요청 maneuver용** 새 유효 후보를 만들었는가 |
| `trajectory_source` | 이번 사이클에 **실제로 MPC로 무엇을 보냈는가** |

| 상황 | `plan_valid` | `trajectory_source` |
|---|---|---|
| 정상 신규 궤적 | `true` | `TRAJECTORY_NEW` |
| transient 실패 + 이전 궤적 재사용 | `false` | `TRAJECTORY_REUSED` |
| MERGE 잠긴 궤적 실행 중 | `false` | `TRAJECTORY_LOCKED` |
| 명령 결함 / 충돌 차단 | `false` | `TRAJECTORY_SAFE_STOP` |
| 보낼 궤적이 없음 | `false` | `TRAJECTORY_NONE` |

이 구분은 C2에서 만든 fail-safe 정책(`immediate_stop` vs 재사용)을 메시지 수준에서
명시화한 것이다.

### 8.4 Trajectory reuse ownership — **D4에서 반드시 수정**

**현재 코드에는 command 경계 검사가 없다.** `frenet_planner_node.cpp`의 재사용
분기는 다음이 전부다.

```cpp
if (has_last_valid_trajectory_ && plan_fail_streak_ <= kMaxTransientFailCycles) {
    asmc_msgs::Trajectory reused = last_valid_trajectory_;
    ...
}
```

`command_id`도, 캐시를 만든 명령이 무엇이었는지도 비교하지 않는다. trajectory
conversion 실패 경로도 동일하다.

C2가 막은 것은 `INVALID_COMMAND` / `UNSUPPORTED_BEHAVIOR` / `COLLISION_BLOCKED`
등에서의 재사용이다. **정상적인 새 명령에서 transient failure가 난 경우의 명령
경계는 막지 않는다.** 따라서 다음이 현재 가능하다.

```
command_id=10  KEEP  → 유효 KEEP 궤적이 캐시에 저장됨
command_id=11  STOP  → 명령 자체는 정상
                     → planning cycle에서 TRANSIENT_GENERATION_FAILURE
                     → has_last_valid_trajectory_ == true
                     ⇒ command 10의 KEEP 궤적이 재사용됨  ← STOP 명령 중 KEEP 주행
```

**v2에서는 재사용을 동일 `command_id`로 한정한다.** 캐시를 execution manager가
소유하며, 최소한 다음을 만족해야 재사용한다.

```
cached.command_id  == current.command_id
cached.fingerprint == current.fingerprint
현재 요청이 CANCEL이 아님
명령이 stale하지 않음
실패 사유가 TRANSIENT_GENERATION_FAILURE
plan_fail_streak <= kMaxTransientFailCycles
```

캐시 구조:

```cpp
struct CachedTrajectory {
    bool valid = false;
    uint32_t command_id = 0;
    CommandFingerprint fingerprint;
    ros::Time generated_at;
    asmc_msgs::Trajectory trajectory;
};
```

`command_id`가 바뀌면 캐시는 **즉시 무효화**되며, 그 사이클의 실패는 재사용이
아니라 safe stop으로 처리된다.

---

## 9. PlannerCycleReport

### 9.1 왜 필요한가

현재 `Plan()`은 `bool` + `CartesianPath`만 반환하고, 진단값은
`last_mode()` / `last_target_speed()` / `last_failure_reason()` 세 개의 getter로만
나온다. 새 `FrenetFeedback`은 20개 이상의 필드를 요구하므로 이 인터페이스로는
채울 수 없다.

### 9.2 원칙

**node가 planner 상태를 추측하지 않는다.**

```
[금지]  node: "mode==MERGE니까 gap_evaluated는 아마 true겠지"
[정상]  planner: 실제로 gap 평가를 수행 → report.gap_evaluated = true
        node:    report 값을 그대로 FrenetFeedback에 복사
```

### 9.3 구조

```cpp
struct PlannerCycleReport {
    bool command_accepted = false;
    bool plan_valid = false;
    ExecutionState execution_state = ExecutionState::IDLE;

    double maneuver_progress = -1.0;

    int active_leader_id = -1;
    int selected_front_object_id = -1;
    int selected_rear_object_id = -1;

    bool gap_evaluated = false;
    bool gap_safe = false;
    double estimated_entry_time_s = -1.0;

    bool frenet_state_valid = false;
    double current_s = 0.0;
    double current_d = 0.0;

    bool target_d_valid = false;
    double target_d = 0.0;

    double applied_speed_mps = -1.0;

    uint32_t generated_candidate_count = 0;
    uint32_t valid_candidate_count = 0;
    double selected_cost = -1.0;

    int blocking_object_id = -1;
};
```

### 9.4 단계적 채우기

모든 필드를 한 번에 완성하려 하지 않는다. **계산 가능한 것부터** 채우고, 나머지는
해당 모드를 구현할 때 함께 채운다. 12.1절의 "모드 구현 = 경로 생성 + Feedback 한
세트" 원칙이 이 표에 그대로 적용된다.

| 단계 | 채우는 필드 |
|---|---|
| **D4** (KEEP/STOP/E-STOP) | `command_accepted`, `plan_valid`, `execution_state`, `current_s`, `current_d`, `frenet_state_valid`, `applied_speed_mps`, `generated_candidate_count`, `valid_candidate_count`, `selected_cost`, `blocking_object_id` |
| **D5** (FOLLOW) | `active_leader_id` |
| **D6** (AVOID) | `maneuver_progress`, `target_d`, `target_d_valid` |
| **D7** (LANE_CHANGE) | (D6 필드 재사용 — `direction` 반영된 `target_d`) |
| **D8** (MERGE) | `selected_front_object_id`, `selected_rear_object_id`, `gap_evaluated`, `gap_safe`, `estimated_entry_time_s` |
| **D9** | 전체 일관성 검증 + sentinel 사용 감사 |

아직 채우지 않은 필드는 **`FrenetFeedback.msg`가 정의한 sentinel을 정확히** 사용한다
(객체 ID `-1`, 실수 `-1.0`, bool `false`, `active_maneuver`는 `MANEUVER_NONE=255`).
임의의 0이나 추측값을 넣지 않는다.

---

## 10. Legacy migration policy

### 10.1 D1의 역할: 제거가 아니라 "행동 선택권 분리"

D1은 legacy 코드를 삭제하는 단계가 **아니다.** 어느 부분이 FSM으로 넘겨야 할
**행동 선택권**이고 어느 부분이 계속 쓸 **계산 자산**인지 경계를 짓는 단계다.

핵심 원칙:

> **행동 선택권만 FSM으로 넘긴다. 계산 자산은 전부 보존한다.**

### 10.2 분리 대상 (행동 선택권 → FSM)

`PlanInternal`의 `use_legacy_behavior` 분기 안에서 "지금 어떤 모드로 주행할지를
planner가 스스로 결정하는" 부분이다.

`frenet_planner.cpp`의 해당 지점(행 번호는 D0 작성 시점 기준):

| 분리 대상 | 현재 위치 |
|---|---|
| 정적 장애물 발견 → AVOID 자동 전환 | `cmd.mode = AVOID` (약 1009행), 선행 조건은 `FindStaticAvoidanceTarget` |
| merge zone 진입 → MERGE 자동 전환 | `cmd.mode = MERGE` (약 1024행), 선행 조건은 `roundabout_window` / `highway_window` |
| leader 획득 → FOLLOW 자동 전환 | `cmd.mode = FOLLOWING` (약 1053행), 선행 조건은 `FindLeader` / `following_.active` |
| 그 외 → KEEP 자동 폴백 | `cmd.mode = LANE_KEEPING` (약 1062행) |
| 조건 만족 → 내부 EXECUTE 승격 (COMMIT 자동화) | `merge_.commit_pending = true` (약 997행) — 5.4절 |

이 다섯 지점이 `if (use_legacy_behavior) { ... }` 블록의 끝부분에 모여 있고, v2
경로는 `cmd = *external_command;`로 이 결정 전체를 건너뛴다. 즉 **분리 지점은 이미
구조적으로 한 곳에 모여 있다** — D1은 이 경계를 문서화하고 v2 경로가 어떤 필드를
채워야 하는지 확정하는 작업이다.

> STOP APPROACH 단계의 `cmd.mode = LANE_KEEPING`(약 1134행)은 분리 대상이 **아니다.**
> 이것은 행동 선택이 아니라 "STOP의 APPROACH 구간에서는 velocity-keeping 후보
> 생성기를 쓴다"는 내부 구현 위임이며, 외부에 보고되는 `last_mode_`는 STOP으로
> 유지된다. C4의 STOP core에 속하므로 그대로 보존한다.

### 10.3 보존 대상 (계산 자산 → 그대로 사용)

이것들은 v2에서도 계속 쓴다. 다시 만들지 않는다.

| 보존 자산 | 쓰이는 단계 |
|---|---|
| `FollowingContext` (leader 추적, handoff, dropout grace) | D5 |
| `AvoidanceContext` (TRACKING/SHIFT/PASS, 방향 잠금) | D6 |
| `MergeContext` (APPROACH/WAIT/COMMIT/CROSS/CLEAR, locked path) | D8 |
| `StopContext` (APPROACH/BRAKING/HOLD) — C4 완성분 | D4 |
| candidate generation (`ResolveManeuver` 및 각 generator) | D4~D8 |
| collision check (SAT / OBB) | D4~D8 |
| object prediction (CTRV, yaw-rate 추정) | D5~D8 |
| gap check (`FindRoundaboutGap` / `FindHighwayMergeGap`) | D8 |
| cost calculation (`EvaluateCosts`, `SelectBestPath`) | D4~D8 |

### 10.4 지금 지우지 않는 것

다음은 **D8이 MORAI 검증을 통과할 때까지 그대로 둔다.**

- 3인자 `Plan(ego, obstacles, out_path)` — legacy UDP 진입점
- `PlanInternal`의 `use_legacy_behavior` 분기
- `frenet_planner_udp_legacy` 실행파일과 `src/main.cpp`

**이유:** 커밋 `711d536`("라이다 고주로, 회전교차로 검증한거. 백파일 2개 땀")에서
회전교차로 및 고속도로 MERGE를 MORAI로 검증하고 백파일까지 확보했다. **그 검증된
동작의 유일한 실행 경로가 legacy 간이 FSM 안에 있다.** D8이 완성되기 전에 제거하면
과거의 성공을 재현할 수단이 사라진다.

### 10.5 제거 순서

모드 단위로, **v2 구현이 MORAI 검증을 통과한 뒤 해당 조각만** 제거한다.

```
D5 FOLLOW  MORAI 검증 통과  →  legacy FOLLOW  자동 전환 제거
D6 AVOID   MORAI 검증 통과  →  legacy AVOID   자동 전환 제거
D8 MERGE   MORAI 검증 통과  →  legacy MERGE   자동 전환 + 내부 COMMIT 승격 제거

D10에서 한 커밋으로 (atomic cutover):
  3인자 Plan()
  use_legacy_behavior 분기
  frenet_planner_udp_legacy + src/main.cpp
  /behavior/context, /planning/feedback 구독·발행
  BehaviorContext.msg, PlanFeedback.msg
```

### 10.6 병행 운영 중 불변식

- v1 경로(`/behavior/context` → `/planning/feedback`)는 D10 cutover까지 **동작을 유지**한다
- v2 경로는 **추가**될 뿐 v1의 코드 경로를 바꾸지 않는다
- 두 런타임을 동시에 launch하지 않는다 (기존 UDP/ROS 런타임 규칙과 동일)
- 각 모드의 legacy 자동 전환을 제거할 때, 그 모드의 **계산 자산은 건드리지 않는다**

---

## 11. Pending decisions (FSM 개발자 합의 필요)

아래 항목은 **논의용 제안이며, 합의되지 않으면 원본 `.msg`를 그대로 유지한다.**
D3은 FSM 개발자가 준 원문을 그대로 추가하고, Frenet 측에서 메시지를 수정하지 않는다.

제안하는 enum 값은 원본 `FrenetFeedback.msg`에서 현재 마지막으로 사용된
`TOO_LATE_TO_CANCEL=9` 다음 번호를 이어받은 것이다.

| # | 항목 | 제안 | 영향 |
|---|---|---|---|
| **Q1** | `MERGE`에서 `has_target_pose=true`가 필수인가 권장인가 | 문서는 "일반적으로 true". 필수면 3.3절 표를 확정, 권장이면 `target_link_id` + 기존 merge zone 설정으로 대체 가능한지 확인 필요 | 3.3절 validation matrix |
| **Q2** | `result_code`에 `INPUT_DATA_STALE=10` 추가 | 현재 node는 vehicle / objects / behavior stale을 각각 구분해 fail-safe 처리하는데, `FrenetFeedback`에는 `STALE_COMMAND`(= `valid_until` 만료)밖에 없다. 입력 데이터 stale을 명령 만료로 보고하면 FSM이 잘못된 회복 동작을 한다 | `FrenetFeedback` enum 1개 추가 |
| **Q3** | `result_code`에 `MANEUVER_NOT_IMPLEMENTED=11` 추가 | 개발 기간 중 v2가 아직 열지 않은 maneuver를 받았을 때 필요. `INVALID_COMMAND`는 명령이 정상인데 거짓말이 되고, 조용한 KEEP 폴백은 더 위험하다. **cutover 후에는 발생 금지**임을 문서에 명시 | `FrenetFeedback` enum 1개 추가 |
| **Q4** | terminal latch 정책 (6절) 동의 여부 | `COMPLETED`/`FAILED`를 새 `command_id`까지 유지 | FSM 수신측 해석 |
| **Q5** | preemption 정책 (7절) 동의 여부 | `COMMITTED` 중 일반 새 command 금지, `EMERGENCY_STOP`만 preempt | FSM 발행측 정책 |
| **Q6** | `EMERGENCY_STOP` 정지 목표를 Frenet이 자체 계산 (8.1절) 동의 여부 | 메시지 변경이 아니라 책임 분계 확인 | 없음 (내부 구현) |

Q2/Q3가 승인되지 않을 경우의 대안은 **v2 인터페이스를 해당 maneuver가 모두 구현될
때까지 production cutover하지 않는 것**이다. 개발 중 v2 토픽에서는 미구현
maneuver를 거부하되, FSM과 실제 연결 전에 전부 구현한다.

---

## 12. 개발 순서 (D0~D10)

### 12.1 핵심 원칙: 모드 구현 = 경로 생성 + Feedback 한 세트

Feedback을 마지막에 몰아서 만들지 않는다. 각 모드를 뚫을 때 **Command 수신 →
Trajectory 생성 → Feedback 발행**까지를 하나의 완료 단위로 본다.

```
FSM BehaviorCommand
        ↓
Frenet command 수신
        ↓
해당 모드 처리
        ↓
Trajectory 생성
        ↓
FrenetFeedback 발행
        ↓
FSM이 결과 확인
```

따라서 예를 들어 "FOLLOW 개발 완료"의 기준은 `FOLLOW 경로가 생성된다`가 아니라
다음까지 되어야 한다.

```
FOLLOW command 수신
  → FOLLOW trajectory 생성
  → active_maneuver = FOLLOW
  → command_id echo
  → execution_state 보고
```

이 방식의 이점은 **병렬 통합**이다. Frenet이 전부 만든 뒤 마지막에 FSM이 붙는
구조가 아니라, 한 모드가 끝나면 FSM 담당자가 그 모드부터 바로 붙여볼 수 있다.

### 12.2 단계별 계획

| 단계 | Frenet | Feedback | FSM 협업 가능 상태 | 기존 주행 영향 |
|---|---|---|---|---|
| **D0** | 인터페이스/enum 계약 확정 (본 문서) | 기본 계약 확정 | 메시지 계약 확인 | 없음 |
| **D1** | Legacy 간이 FSM 분리 준비 (10절) | — | — | 없음 |
| **D2** | Canonical `PlannerCommand` 구축 | — | — | 없음 |
| **D3** | `BehaviorCommand` → `PlannerCommand` 변환 | `command_id` 보존 경로 확보 | Command 수신 테스트 | 없음 |
| **D4** | KEEP / STOP / EMERGENCY_STOP | 동시 구현 | 기본 E2E | KEEP baseline 유지 |
| **D5** | FOLLOW | 동시 구현 | FOLLOW 연동 | |
| **D6** | AVOID | 동시 구현 | PREPARE/EXECUTE handshake 검증 | |
| **D7** | LANE_CHANGE | 동시 구현 | 방향 + handshake | |
| **D8** | MERGE | 동시 구현 | 최종 복합 모드 | |
| **D9** | 공통 예외 처리 | 통합 정리 | 인터페이스 안정화 | |
| **D10** | MORAI E2E 통합 검증 | 전체 검증 | 전체 통합 | cutover |

### 12.3 모드 개발 순서의 근거

enum 선언 순서(KEEP/FOLLOW/MERGE/LANE_CHANGE/AVOID/STOP/EMERGENCY_STOP)와 **개발
순서는 별개다.** 개발은 의존성이 적은 것부터 간다.

```
KEEP  →  STOP / EMERGENCY_STOP  →  FOLLOW  →  AVOID  →  LANE_CHANGE  →  MERGE
```

- **KEEP 먼저**: 현재 MORAI에서 검증된 baseline이라 회귀 기준이 된다.
- **STOP / EMERGENCY_STOP**: C4에서 STOP core가 이미 완성되어 있고, handshake가
  필요 없는 단순 `EXECUTE` 모드다.
- **FOLLOW**: 객체 resolve와 Frenet 투영이 처음 들어온다. 아직 handshake는 없다.
- **AVOID가 LANE_CHANGE보다 먼저**: AVOID에서 `PREPARE → READY → EXECUTE`
  handshake 구조를 한 번 검증해두면, LANE_CHANGE는 그 패턴을 거의 그대로 재사용할
  수 있다. LANE_CHANGE는 여기에 `direction`(LEFT/RIGHT)만 추가된다.
- **MERGE 마지막**: MERGE는 앞 단계 산출물을 거의 다 가져간다.

  | MERGE가 쓰는 것 | 어디서 만들어지는가 |
  |---|---|
  | 전방 객체 / gap | D5 FOLLOW |
  | candidate + collision 재검증 | D6 AVOID |
  | lateral transition + PREPARE/EXECUTE | D7 LANE_CHANGE |
  | conflict zone / TTC | D8 (MERGE 고유) |

### 12.4 단계별 완료 기준

각 단계는 아래를 만족해야 다음으로 넘어간다.

**D4 — KEEP / STOP / EMERGENCY_STOP**

```
BehaviorCommand(maneuver=KEEP, command_id=10)
  → KEEP trajectory
  → FrenetFeedback(command_id=10, active_maneuver=KEEP, execution_state=EXECUTING)
```

STOP은 C4의 `StopContext`(APPROACH/BRAKING/HOLD)에 연결하고, EMERGENCY_STOP은
8.1절의 직접 감속 궤적으로 처리한다. 이 단계가 끝나면 FSM 담당자에게
"KEEP / STOP / EMERGENCY_STOP은 topic으로 직접 쏴서 테스트 가능"이라고 알릴 수 있다.

**D5 — FOLLOW**

```
정상        : active_maneuver=FOLLOW, execution_state=EXECUTING
대상 소실   : active_maneuver=FOLLOW, execution_state=FAILED,
              result_code=TARGET_OBJECT_LOST
```

> **중요:** 대상 객체를 잃었을 때 **planner가 임의로 KEEP으로 전환하지 않는다.**
> Frenet은 "FOLLOW 수행 실패"까지만 보고하고, `FOLLOW → KEEP` 결정은 FSM이 한다.
> 이것이 v2에서 행동 선택권이 FSM에 있다는 원칙의 구체적 적용이다.

**D6 — AVOID (handshake 최초 적용)**

```
FSM: AVOID / PREPARE / command_id=21
  → candidate 생성 → collision 검사 → safe candidate 확보
  → Feedback(command_id=21, active_maneuver=AVOID, execution_state=READY)

FSM: AVOID / EXECUTE / command_id=21
  → candidate 재검증 (5.3절)
  → trajectory commit
  → Feedback(execution_state=COMMITTED → EXECUTING)
```

**D7 — LANE_CHANGE**

D6 패턴 재사용 + `direction`. 2.2절의 1:2 매핑
(`LANE_CHANGE`+`DIRECTION_LEFT` → `LANE_CHANGE_LEFT`)이 실제로 동작하는 첫 단계다.

```
PREPARE(direction=LEFT) → target lane candidate → feasibility → READY
EXECUTE                 → recheck → commit → EXECUTING → COMPLETED
```

**D8 — MERGE**

5.4절대로 기존 MERGE 자산을 유지하고 COMMIT trigger만 FSM의 EXECUTE로 옮긴다.

```
PREPARE : merge zone → conflict point → gap/TTC → prediction → candidate → READY
EXECUTE : gap 재검증 → collision 재검증 → commit → EXECUTING → COMPLETED
```

**D9 — 공통 예외 처리 정리**

Feedback을 처음 만드는 단계가 아니다. D4~D8에서 모드별로 이미 만들어진 것을
**통합 정리**한다.

- `execution_state` / `result_code` 사용 일관성 검증
- stale command (`valid_until` 만료) 처리 통일
- 잘못된 enum 값 처리 통일
- `command_id` mismatch 처리
- CANCEL 처리 (7절)
- 중복 command 처리
- terminal latch (6절) 최종 검증
- 실패 fallback 경로 통일 (8.3/8.4절)

**D10 — MORAI E2E**

FSM + Frenet + MPC + MORAI 전체 통합. 이 시점에 legacy atomic removal을 수행한다
(10.5절).

### 12.5 execution_state 사용 진도

`FrenetFeedback.msg`가 정의한 8개 상태(`IDLE`~`FAILED`)는 **원본 그대로 유지**한다.
단계별로 Frenet이 실제로 보고하는 부분집합만 늘어난다.

| 단계 | 실제 보고하는 상태 |
|---|---|
| D4 | `IDLE`, `EXECUTING`, `COMPLETED`, `FAILED` |
| D5 | + (변화 없음 — FOLLOW도 handshake 없음) |
| D6 | + `PREPARING`, `WAITING`, `READY`, `COMMITTED` |
| D7~D8 | 8개 전체 |
| D9 | 전체 + 일관성 검증 |

아직 보고하지 않는 상태를 FSM이 기다리게 하지 않도록, 각 단계 완료 시 "이 모드에서
나올 수 있는 상태 목록"을 FSM 담당자에게 함께 전달한다.

### 12.6 파일 배치

```
src/frenet_planner/src/ros/
├─ behavior_command_adapter.hpp / .cpp      (D3: 변환 + validation + enum mapping)
└─ command_execution_manager.hpp / .cpp     (D3~D4: transaction + 캐시 소유권)

src/frenet_planner/tests/
├─ maneuver_enum_mapping_test.cpp           (D3: 2.4절 회귀)
├─ behavior_command_adapter_test.cpp        (D3: 3절/4절 회귀)
└─ command_execution_manager_test.cpp       (D4: 6절/7절/8.4절 회귀)
```

execution/handshake는 planning 수학이 아니라 **transaction 상태**이므로 planner
core가 아닌 ROS/FSM 경계 계층에 둔다. `PlannerCommand`는 D2에서 planning 입력으로
정리하며, transaction 필드(`execution_request` 등)를 어디에 둘지는 D2에서 확정한다.
