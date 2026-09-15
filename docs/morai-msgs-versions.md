# MORAI morai_msgs 버전 매핑

> **문서 역할:** Version Reference — 조사 시점의 26.R1/대회 msg 차이 기록
> **주의:** 실제 대회판 이식 시 재실측하고 [morai-msgs-beta-policy.md](./morai-msgs-beta-policy.md)를 갱신한다.
> **문서 허브:** [README.md](./README.md)
> **최종 수정:** 2026-09-15
> 통합 시 **단일 submodule** 기준을 정하고, 레거시 코드는 필드 차이를 인지해야 합니다.
> **대회 ROS UI 실측:** [docs/competition/morai-competition-ros-topics.md](../../docs/competition/morai-competition-ros-topics.md)

## 1. upstream 브랜치·태그 정리

저장소: [MORAI-Autonomous/MORAI-ROS_morai_msgs](https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs)

| Git ref | 커밋 (요약) | MORAI SIM 대응 | msg 수 | 비고 |
|---------|-------------|----------------|--------|------|
| **`beta_drive`** | `45c6baf` Competition msg add | **대회 공식** (K-City Competition License) | **101** | 주최측 공지 브랜치 |
| **`26.R1`** (브랜치) | `4c9be6f` add angular_velocity | **26.R1** (h2·h3 포함) | 91 | 시뮬 최신 일반 라인 |
| **`main`** | `ccbf951` Merge 26.R1 | = `26.R1` 반영 후 기본 브랜치 | 91 | 앞으로 일반 릴리스 기준 |
| **`26.r1`** (태그) | `98c83d1` Merge PR #19 | 26.R1 머지 시점 | 91 | `26.R1` 브랜치와 **주석·포맷** 차이 있음 |
| **`24.r2`** (태그) | `d1ce163` | **24.R2** | 91 | 구버전 (TL msg 일부만) |
| **`master`** (구 clone) | `a561daa` (mkdocs) | **26.R1 계열** | 91 | 공지 전에 clone한 폴더 — **pre-26 아님** |

### `26.R1` 브랜치 vs `main`

- **`26.R1`**: MORAI SIM **26.R1** 릴리스용 msg 스냅샷 (개발 브랜치).
- **`main`**: `26.R1`을 merge한 **기본 브랜치**. msg 내용은 `26.R1`과 동일 계열.
- **`26.r1` 태그**: 예전 `master`에 26.R1을 merge한 커밋 — 브랜치 대비 **주석이 제거된** 버전이 섞여 있을 수 있음.

### `beta_drive` vs `26.R1`

`beta_drive`는 `26.R1`에서 파생한 것이 **아닙니다**. 별도 라인에서 대회 전용 msg가 추가·변경되었습니다.

**`beta_drive`에만 있는 msg (10종)**

`Competition`, `EgoNoisyStatus`, `EgoDetailStatus`, `AttachmentDeviceState`, `GripperState`, `MSITCustomMessage`, `ENU`, `RPY`, `RobotPose`, `XYZ`

**필드가 실제로 다른 shared msg (2026-07-26 재검증: 6종)**

| msg | `26.R1` (h3 일반) | `beta_drive` (대회) | 팀 영향 |
|-----|-------------------|---------------------|---------|
| **`CtrlCmd`** | `front_steer`, `rear_steer` (rad) | **`steering`** 단일 | **제어 필수** — `mpc_ws`는 beta(`steering`) |
| **`EgoVehicleStatus`** | `front_steer_angle`, `rear_steer_angle`, **`angular_velocity`**, lane CTE 필드 | **`wheel_angle`** (lane CTE·angular_velocity **없음**) | **ego/수집/LBC** |
| `ManipulatorControl` | `GripperStatus` 등 | 일부 필드 축소 | 대회 무관 |
| `MultiEgoSetting` | turn_signal·speed 배열 등 | 축소 | 낮음 |
| `NpcGhostInfo` | brake/steer/speed/signal | 축소 | 낮음 |
| `RobotOutput` | — | `is_active_manipulator_brake` 추가 | 낮음 |

**동일(수집·인지에 중요):** `ObjectStatus`/`ObjectStatusList`, `GetTrafficLightStatus`, `SetTrafficLight`, `IntscnTL`, `GPSMessage`, `CollisionData` 필드 동일.
단 `SyncModeCtrlCmd`·`WaitForTickResponse`는 **중첩 `CtrlCmd`/`EgoVehicleStatus`** 를 품으므로 sync 제어·tick 응답은 간접적으로 깨질 수 있음.

`mpc_ws`는 `cmd.steering` → **beta_drive와 일치**.

### 2026-09-09 direct ROS 재실측

26.R1.H3가 rosbridge로 보낸 `/Ego_topic`에는 `angular_velocity`가 포함돼 beta_drive
`EgoVehicleStatus` 변환이 거부됐다. 반면 공통 스키마인 `/Object_topic`은 약 39~40 Hz로
정상 수신됐고 NPC·보행자 ID도 보존됐다. 즉 같은 ASMC workspace에서 direct ROS를 선택하면
토픽별로 부분 호환되는 상태가 되므로 팀 기본 경로로 삼지 않는다. 26.R1.H3와 대회판 모두
UDP bridge가 beta 내부 계약으로 정규화한다. direct ROS/rosbridge GT 비교는 2026-09-15
종료했으며 `rosbridge.sh`는 당시 실패 조건 재현용으로만 남긴다.

### ROS msg 차이와 UDP bridge 공용 범위

ROS의 `.msg` 필드/토픽 차이는 direct ROS serialization 계약의 차이다. UDP bridge는
와이어의 binary packet을 직접 해석한 뒤 팀의 `beta_drive` ROS topic으로 바꾸므로,
포트와 binary layout이 같으면 26.R1.H3와 대회판이 하나의 구현을 공유할 수 있다.

- Camera/GPS/IMU는 센서 UDP 포맷을, 3D LiDAR는 Velodyne protocol을 해석한다.
- Ego는 현재 181/229/245-byte 패킷 변형을 길이로 구분한다.
- ROS 출력은 버전과 무관하게 현재 `beta_drive` canonical topic으로 고정한다.
- `udp_bridge.sh --simulator 26r1|competition`은 구현을 복제하지 않고 provenance와
  향후 호환 파라미터 분기만 관리한다.

다만 26.R1은 front/rear steer 추가와 UDP 예제 갱신이 공지됐으므로, 대회판에서
수신 header/패킷 길이와 23-byte CtrlCmd 반응은 실차로 gate한다. 차이가 확인되면
별도 bridge 복제가 아니라 공용 parser의 패킷 변형과 simulator profile 파라미터로 추가한다.
공식 근거는 MORAI [26.R1 센서 통신 프로토콜](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/26.R1/-35)과
[26.R1 release note](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/26.R1/morai-sim-drive-26-r1-0)를 따른다.

### 시뮬 선택 (2026-08-03)

| 순위 | SIM | 권장 역할 |
|------|-----|-----------|
| **Phase 0 기본** | **26.R1.h3** | 차선·drivable·수집·GT (대회 제약으로 일상은 여기) |
| **게이트** | **대회 beta** | 노이즈·토픽 공백·당일 |
| — | 24.R2 | 비권장 |

스택·저장 스키마는 **`beta_drive`**. 26 ego/ctrl만 정규화. 토픽 완전 정합 포기.

---

## 2. 신호등 (Traffic Light) — msg vs 런타임 토픽

### .msg 정의

**`beta_drive`와 `26.R1` 모두** 아래 TL 관련 msg가 **존재**합니다.

- `IntscnTL`, `TrafficLight`, `GetTrafficLightStatus`, `MoraiTLInfo`, `MoraiTLIndex`, `Lamps`, `SetTrafficLight`

→ “대회 브랜치에 신호등 **msg가 없다**”는 말은 **사실이 아님**.
→ 이슈는 **런타임에 토픽이 비거나 맵별로 다름** 쪽에 가깝습니다.

### 런타임 (팀 검증)

| 토픽/서비스 | KATRI / 개발 | 비고 |
|-------------|--------------|------|
| `/Lamps_topic` | **비어 있음** | LBC에서 사용 안 함 |
| `/IntscnTL_topic`, `/GetTrafficLightStatus` 등 | 맵·시뮬 설정에 따라 가변 | LBC가 가용 토픽 자동 구독 |
| `/Service_MoraiEventCmd` | **turnSignal** 폴링 | 좌회전 의도 → BEV 신호등 색 보정 |

### 대회 MORAI ROS UI 실측 (2026-07-25)

**정본:** [docs/competition/morai-competition-ros-topics.md](../../docs/competition/morai-competition-ros-topics.md)

대회 버전 ROS Topic 패널에 **노출된 전부:**

| 방향 | Topic / Service | Type |
|------|-----------------|------|
| → | `/Ego_topic`, `/Competition_topic` | `EgoVehicleStatus` |
| → | `/Object_topic` | `ObjectStatusList` |
| → | `/CollisionData` | `CollisionData` |
| → | `/tf` | `tf2_msgs/TFMessage` |
| ← | `/ScenarioLoad` | `ScenarioLoad` |
| ← | `/SetTrafficLight` | `SetTrafficLight` |
| srv | `/Service_MoraiEventCmd` | `MoraiEventCmdSrv` |

Bridge: `127.0.0.1:9090` · Target Frame **50 Hz**.
**없음:** GetTrafficLight / IntscnTL / Lamps / Camera·LiDAR·GPS·IMU ROS · `/ctrl_cmd`(이 패널 기준).

→ “대회에 Get이 없다” = **런타임 사실**. `.msg` 부재가 아님.
→ **기본 SIM = 대회**. **GT 라벨 세션만 26.R1.h3**.
→ `rostopic list` 대기: [morai-competition-ros-topics.md](../../docs/competition/morai-competition-ros-topics.md) §7.

**통합 시:** 코드·일상 = `beta_drive` + **대회 SIM**. 26은 privileged 수집 전용.

---

## 3. 팀 레포별 실측 기준 (로컬 clone 검증)

| 레포 | 담당 | 실측 일치 | 사용자 기억 | 검증 결과 |
|------|------|-----------|-------------|-----------|
| **`mpc_ws`** | 정윤태·양서준 | **`beta_drive` 100%** (101 msg, 내용 0 diff) | beta_drive | ✅ 일치 |
| **`aim_ws` `va_seunghyun`** | 안승현 | **`26.R1` 필드 + 주석 제거** (91 msg). beta와 **6개 msg** 필드 차이 | 26 이전 → h2 업데이트 | △ **26.R1 계열** (jang과 동일 필드, 주석만 다름). `CtrlCmd`: `front_steer`/`rear_steer`. LBC는 `EgoVehicleStatus.front_steer_angle` 사용 |
| **`aim_ws` `jang`** | 장원태 | **`26.R1` 브랜치 100%** (91 msg, 0 diff) | 26.r1.h3 | ✅ **26.R1 = h3 sim msg 라인** |
| **`aim_ws` `ochang`** | 이강태 | va와 유사 stripped, beta와 3개 diff | 대회 무관 | ⚠️ 구버전·통합 제외 |
| **`morai-3d-detection`** | 손재호 | msg 폴더 비어 있음. `morai_3d_live.py`만 `EgoVehicleStatus` 등 import | h2~h3 | △ **오프라인 데이터** 위주 — msg 빌드 의존 낮음. live는 `26.R1` 필드명 가정 가능 |
| **`2026-ASMC`** (현재) | 통합 | **`beta_drive` @ `45c6baf`** (submodule) | — | ✅ 통합 완료 |

### 참조용 clone (ASMC 밖)

[MORAI-ROS_morai_msgs](https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs)를 원하는 경로에 브랜치별로 clone합니다.

| 브랜치 | 커밋 | 용도 |
|--------|------|------|
| `beta_drive` | `45c6baf` | **대회 기준** — ASMC 빌드·당일 PC |
| `26.R1` | `4c9be6f` | **시뮬 26.R1 h3 참조** — beta와 diff 비교 |

```bash
diff -ru <26.R1-clone>/msg <beta_drive-clone>/msg
```

**정렬 규약 (통합 후 필수):** [morai-msgs-beta-policy.md](./morai-msgs-beta-policy.md)

## 4. 통합 시 beta 정렬

통합 **시작 전** 확정된 규칙은 **[morai-msgs-beta-policy.md](./morai-msgs-beta-policy.md)** 를 따릅니다.

요약:
- ASMC catkin·대회 = **`beta_drive` submodule 단일**
- 레거시(26.R1 필드) 이식 PR = **merge 전 beta 필드로 수정**
- upstream `26.R1` 브랜치 clone은 **diff 참고용만**

### morai-3d-detection

- 학습 데이터는 이미 수집됨 → msg 변경 영향 **낮음**
- `morai_3d_live.py` 재연동 시 `EgoVehicleStatus` 필드명만 beta에 맞게 수정

## 5. 버전 선택 요약

| 용도 | 권장 |
|------|------|
| **일상·ASMC·당일** | 시뮬 **대회 beta** + submodule `beta_drive` |
| **GT 라벨만** | 시뮬 **26.R1.h3** (필요할 때만) + ego/ctrl 정규화 |
| **24.R2** | 비권장 |
| **신호등** | 대회=Set/TL-Net · Get 라벨은 26 세션 |
| **레거시** | upstream `26.R1` clone |
