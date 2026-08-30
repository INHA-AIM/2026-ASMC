# morai_msgs beta_drive 정렬 규약

> **최종 수정:** 2026-07-25  
> **상태:** 통합 **시작 전** 확정 규칙  
> **관련:** [morai-msgs-versions.md](./morai-msgs-versions.md) · [integration-plan.md](./integration-plan.md) · [대회 ROS 실측](../../docs/competition/morai-competition-ros-topics.md)

## 1. 원칙 (한 줄)

**`2026-ASMC`의 catkin 빌드·런타임·대회 당일 PC는 항상 `beta_drive` msg 기준.**  
레거시 코드를 이식한 직후에도, **그 PR을 merge하기 전에** beta 필드로 맞춘다.

시뮬레이터는 26.R1 h3를 쓸 수 있지만, **코드가 참조하는 msg 정의는 beta_drive 단일**이다.

---

## 2. ASMC에서의 morai_msgs 위치

| 항목 | 값 |
|------|-----|
| **공식 위치** | `src/MORAI-ROS_morai_msgs` (git **submodule**) |
| **브랜치** | **`beta_drive`만** |
| **upstream** | https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs |
| **금지** | 패키지 안·워크스페이스 안 `morai_msgs` 복사본, symlink를 submodule 대신 영구 사용 |

### 참조용 clone (ASMC 밖, git 미포함)

필요 시 [MORAI-ROS_morai_msgs](https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs)를 **원하는 경로에** 두 브랜치를 각각 clone합니다.

| 브랜치 | 용도 |
|--------|------|
| `beta_drive` | **대회 기준** — diff·검증의 정본 |
| `26.R1` | **시뮬 26.R1 h3 참조** — beta와 diff 비교용 |

```bash
# 예: 두 clone 경로에서 msg diff
diff -ru <26.R1-clone>/msg <beta_drive-clone>/msg
```

---

## 3. 통합 단계별 beta 정렬 (코드 이식 시)

통합은 `integrate/*` PR로 진행하되, **각 PR merge 조건에 beta 정렬을 포함**한다.

### 3.1 이식 직후 (integrate PR 안에서)

| 순서 | 작업 |
|------|------|
| 1 | 레거시에서 코드만 복사 (`src/main` 등 **제외**) |
| 2 | `morai_msgs` vendored 폴더는 **복사하지 않음** — submodule만 사용 |
| 3 | 아래 [필드 매핑](#4-필드-매핑-26r1--beta_drive) 적용 |
| 4 | `./scripts/build_ws.sh` 통과 |
| 5 | PR 설명에 “beta_drive 정렬 완료” 체크 |

### 3.2 merge 후 (팀원 일상 작업)

| 규칙 | 내용 |
|------|------|
| 새 ROS 노드·스크립트 | `beta_drive` msg 필드만 사용 |
| `26.R1` 필드명 금지 | `front_steer`, `front_steer_angle`, `rear_steer` 등 **커밋 금지** |
| submodule 업데이트 | 통합 리드만. 팀원은 `git submodule update --init` |
| 인터페이스 변경 | `src/interfaces/` + 3파트 합의 |

### 3.3 PR 체크리스트 (모든 ROS 관련 PR)

- [ ] `src/MORAI-ROS_morai_msgs`가 submodule `beta_drive`인가
- [ ] `grep -r front_steer` / `rear_steer_angle` 등 26.R1 전용 필드가 없는가
- [ ] `CtrlCmd` publish 시 `steering` 사용 (`longlCmdType` 1), UDP 송신 패킷 `ctrl_mode` 2 확인
- [ ] `./scripts/build_ws.sh` 통과

---

## 4. 필드 매핑 (26.R1 → beta_drive)

통합·수정 시 아래로 **일괄 치환**한다. 어댑터 레이어는 만들지 않는다 (단일 기준 유지).

| 용도 | ❌ 26.R1 (사용 금지) | ✅ beta_drive |
|------|---------------------|---------------|
| 제어 출력 | `cmd.front_steer`, `cmd.rear_steer` | `cmd.steering` |
| 차량 상태 | `msg.front_steer_angle`, `msg.rear_steer_angle` | `msg.wheel_angle` |
| 차량 상태 | `msg.angular_velocity` (Vector3) | 제거됨 — 필요 시 다른 토픽·유도 |
| 대회 전용 | — | `Competition.msg` 등 (판정 연동 시) |

### 파트별 통합 시 예상 작업

| 출처 | beta 상태 | 통합 PR에서 할 일 |
|------|-----------|-------------------|
| `mpc_ws` | ✅ 이미 beta | 그대로 이식 |
| `aim_ws` LBC·bridge | ❌ 26.R1 필드 | `EgoVehicleStatus`, subscriber 코드 수정 |
| `aim_scenario_runner` | ❌ 26.R1 | gRPC·상태 읽기 필드 수정 |
| `morai-3d-detection` live | △ 데이터 무관 | `morai_3d_live.py`만 beta 필드로 수정 |

---

## 5. 신호등·토픽 (beta 정렬과 별개)

- **msg:** `beta_drive`에도 `IntscnTL`, `TrafficLight`, **`GetTrafficLightStatus`** 등 **정의 있음**
- **대회 런타임:** Set만 · Get 안 나옴 → [실측](../../docs/competition/morai-competition-ros-topics.md)
- **26.R1.h3:** Get 발행 → **GT 라벨 세션에만** 사용 (일상 1순위 아님)
- **노이즈:** 대회 SIM (1순위)
- collector: 기본=대회 토픽 · TL Get은 26 profile optional
- `/Lamps_topic` 사용 안 함

---

## 6. submodule 등록 (통합 리드, planning-v1 PR)

통합 시작 시 최초 1회:

```bash
cd "$ASMC"
# 기존 symlink 제거 후
git submodule add -b beta_drive \
  https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs.git \
  src/MORAI-ROS_morai_msgs
git submodule update --init --recursive
```

팀원 clone 후:

```bash
git clone --recurse-submodules https://github.com/INHAautonav/2026-ASMC.git
# 또는
git submodule update --init --recursive
```

---

## 7. 시뮬 26.R1 h3와의 관계

| 계층 | 버전 |
|------|------|
| MORAI SIM 실행 파일 | 26.R1 h2~h3 (팀원 PC) |
| ROS msg 정의 (빌드) | **`beta_drive` only** |
| 로컬 diff 참고 | upstream `26.R1` 브랜치 clone |

시뮬이 26.R1이어도, **대회와 동일한 msg로 빌드**하면 당일 PC와 정합성이 높아진다.  
필드 불일치로 런타임 오류가 나면 **코드를 beta에 맞추고**, msg를 26.R1로 내리지 않는다.

---

## 8. 담당

| 역할 | 담당 |
|------|------|
| submodule·beta 정책 | 안승현 (통합 리드) |
| planning beta 검증 | 정윤태·양서준 |
| LBC beta 마이그레이션 | 안승현 |
| scenario beta 마이그레이션 | 장원태 |
| perception live | 손재호 |
