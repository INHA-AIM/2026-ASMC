# morai_msgs beta_drive 정렬 규약

> **문서 역할:** Canonical until competition port — 현재 submodule과 msg 정렬 규칙
> **버전 조사:** [morai-msgs-versions.md](../records/bridge/morai-msgs-versions.md)
> **문서 허브:** [README.md](../README.md)
> **현재 상태:** submodule 등록·공용 message 정렬 완료, competition port 전까지 유지
> **관련:** [버전 조사](../records/bridge/morai-msgs-versions.md) ·
> [과거 integration 기록](../records/archive/wave1-wave2-plan.md) ·
> [운영 profile](../operations/README.md)

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

## 3. beta 정렬 gate (코드 이식·추가 시)

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

### 레거시 출처별 port 참고

아래 표는 초기 이식 출처를 식별하기 위한 기록이다. 현재 코드의 완료 상태나 담당 경로는
[ownership](../onboarding/ownership.md)과 각 package README를 따른다.

| 출처 | beta 상태 | 통합 PR에서 할 일 |
|------|-----------|-------------------|
| `mpc_ws` | ✅ 이미 beta | 그대로 이식 |
| `aim_ws` LBC·bridge | ❌ 26.R1 필드 | `EgoVehicleStatus`, subscriber 코드 수정 |
| `aim_scenario_runner` | ❌ 26.R1 | gRPC·상태 읽기 필드 수정 |
| `morai-3d-detection` live | △ 데이터 무관 | `morai_3d_live.py`만 beta 필드로 수정 |

---

## 5. 신호등·토픽 (beta 정렬과 별개)

- **msg:** `beta_drive`에도 `IntscnTL`, `TrafficLight`, **`GetTrafficLightStatus`** 등 **정의 있음**
- **대회 런타임:** 허용된 UDP와 팀 내부 정규화 topic만 사용한다. 현재 profile은
  [운영 가이드](../operations/README.md)를 따른다.
- **26.R1.h3:** Get 발행 → **GT 라벨 세션에만** 사용 (일상 1순위 아님)
- **노이즈:** 대회 SIM (1순위)
- collector: 기본=대회 토픽 · TL Get은 26 profile optional
- `/Lamps_topic` 사용 안 함

---

## 6. submodule 사용

submodule은 이미 등록되어 있으므로 다시 `git submodule add`하지 않는다. 새 clone 또는 pin
동기화에는 다음만 사용한다.

```bash
git clone --recurse-submodules https://github.com/INHAautonav/2026-ASMC.git
# 기존 clone
git submodule update --init --recursive
```

현재 pin을 확인한다.

```bash
git submodule status src/MORAI-ROS_morai_msgs
```

정본 pin은 `45c6baf148f2327f4c9fefd48262f25bdfe4b567`이다. pin 변경은 전체 consumer
build와 필드 diff를 포함한 별도 PR로만 수행한다.

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

## 8. Ownership

submodule과 공용 wire schema는 통합 lead가 주관하고, 변경되는 producer·consumer owner가 함께
리뷰한다. 현재 담당자와 경로는 [ownership](../onboarding/ownership.md)이 정본이다.
