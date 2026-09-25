# 저장소 레이아웃·파트 격리

> **문서 역할:** Active Reference — 실제 코드 위치와 ownership 요약
> **문서 허브:** [README.md](../README.md)
> **최종 수정:** 2026-09-21
> 담당 경로·검증: [팀 ownership](../onboarding/ownership.md)
> Git·PR 규칙: [협업 규칙](../governance/README.md)

현재 표는 실제 경로를 나타낸다. HD맵은 `asmc_hd_map`으로 통합됐고, 나머지 팀 소유 공용
코드는 `asmc_msgs`, `asmc_bridge`, `asmc_perception` 경계로 순차 수렴시킨다. 단계별 삭제
gate는 [저장소 통합 로드맵](./consolidation-roadmap.md)을 따른다. planning 소유
패키지는 해당 로드맵의 일방적 이동 대상이 아니다.

## 1. 최상위 구조

```
2026-ASMC/
├── docs/                 # 팀 문서
│   ├── contracts/        # 동결된 사람이 읽는 interface 계약
│   ├── packages/         # package 개발·검증 가이드
│   └── records/          # baseline·실험·검수 기록
├── docker/
│   ├── ros-noetic/       # ROS catkin 개발 (+ libyaml-cpp)
│   └── perception-train/ # PyTorch 학습
├── src/                  # catkin 패키지 + 논리 README
├── tools/                # scenario runner, grpc, read-only audit (non-catkin)
│   └── audit/            # package/topic/message/sensor/bag/HD Map baseline 생성
├── scripts/              # bridge, build, docker 헬퍼
├── config/               # simulator 설정과 기계 판독 baseline/contract
│   ├── baselines/        # runtime·perception 이관과 producer parity 기준
│   └── contracts/        # HD Map·perception message/topic contract와 registry
├── mgeo_toolkit/         # 기존 HD map bake 도구
├── R_KR_PG_KATRI/        # 기존 KATRI 맵 자산; K-City 대회 데이터는 $ASMC_DATA
├── CONTRIBUTING.md
└── README.md
```

**Git에 넣지 않음:** `build/`, `devel/`, `data/`, `checkpoints/`, `.bag`, `*.pth`

P1-A 당시 snapshot은 [runtime-baseline-v1.md](../records/baselines/runtime-v1.md)에 보존한다.
현재 HEAD는 승인된 후속 변경으로 v1과 다르므로 `build_runtime_baseline.py --check`를 일상
회귀에 넣지 않는다. 다음 repository-wide freeze는 v1을 덮어쓰지 않고 새 version으로 만든다.
현재 contract/product는 아래 명령으로 검증한다. bag과 현장 원본은 `$ASMC_DATA`에서 읽되
저장소로 복사하지 않는다.

```bash
python3 tools/audit/validate_p1b_contract.py
python3 tools/audit/validate_perception_contract.py
python3 src/asmc_hd_map/scripts/build_competition_metadata.py --check
PYTHONPATH=src/asmc_hd_map/src python3 src/asmc_hd_map/scripts/build_drivable_area_input_audit.py --check
PYTHONPATH=src/asmc_hd_map/src python3 src/asmc_hd_map/scripts/build_compact_drivable_map.py --repo-root "$PWD" --check
# ROS Noetic container, publisher 실행 후
rosrun asmc_hd_map verify_competition_map_topics.py
roslaunch asmc_hd_map competition_map.launch local_pose_source:=ego_status
rosrun asmc_hd_map verify_local_map_topics.py  # 별도 개발 smoke: 테스트 map XY를 1회 발행
```

## 2. catkin·코드 패키지

ROS 패키지와 standalone 프로젝트는 **`src/<모듈명>/`** 바로 아래 둔다. 논리 그룹은 `src/<그룹>/README.md`만 둔다.

### planning_control (✅ PR #1·#4)

| 패키지 | 경로 | 담당 | GitHub |
|--------|------|------|--------|
| Behavior planner | `src/behavior_planner/` | 정윤태·양서준 | @yuntae12-sudo · @yangseojun |
| MPC (`mpc_controller_node`, ROS+core) | `src/mpc_controller/` | 정윤태·양서준 | @yuntae12-sudo · @yangseojun |
| Frenet planner (`frenet_planner_node`, ROS+core) | `src/frenet_planner/` | 이강태 | @kante2 |
| MORAI bridge | `src/asmc_bridge/` | 이강태·안승현 | @kante2 · @ahnsh03 |

현재 개발 경계는 [Planning/Control guide](../packages/planning-control/README.md), 실행 흐름은
[운영 규약](../operations/planning-control.md)을 따른다. v1 설계와 standalone UDP 기준선은
각각 [legacy handoff](../handoffs/planning/behavior-legacy.md)와
[archive](../archive/planning/standalone-udp-baseline.md)에 보존한다.

### bridge · HD Map

| 경로 | 역할 | 담당 |
|---|---|---|
| `src/asmc_bridge/` | Network/Sensor UDP, VehicleState, 공동 launch | 이강태·안승현; planning consumer 공동 리뷰 |
| `src/asmc_hd_map/` | reviewed HD Map과 runtime | 안승현 |

현재 bridge 구조와 Phase 3 이관 결과는 [MORAI bridge guide](../packages/asmc_bridge/README.md)를
따른다. P3-F에서 하위 node와 공동 launch를 `src/asmc_bridge/`로 물리 통합했다.

### LBC teacher BEV research

| 경로 | 담당 | GitHub |
|------|------|--------|
| `tools/research/lbc_teacher_bev/` | privileged GT/teacher BEV renderer·scenario debug | @ahnsh03 |
| `scripts/rosbridge.sh`, `config/morai_rosbridge.env` | 종료된 direct ROS 실험 보존 | @ahnsh03 |
| `mgeo_toolkit/`, `R_KR_PG_KATRI/` | 안승현 | @ahnsh03 |

### perception · scenario (P4-C6 camera/BEV/traffic 통합 완료)

| 경로 | 담당 | GitHub |
|------|------|--------|
| `src/asmc_perception/` | camera/BEV/lane/traffic runtime·support·config·model + 분리된 offline/training | @ahnsh03 주관, perception·planning 공동 리뷰 |
| `src/lane_boundary_adapter/` | camera lane typed 출력을 검토하는 read-only planning adapter | 통합·판제 공동 리뷰 |
| `src/lidar+sensorfusion/` | 독립 nested LiDAR·camera fusion workspace | @sonshiny·통합 공동 리뷰 |
| `src/data_collection/` | recorder·bag profile/replay, Capture GT·mask·dataset의 독립 package | @ahnsh03 |
| `tools/aim_scenario_runner/` | 장원태 | @wkddnjsxo |
| `tools/grpc_inha_univ/` | 장원태 | @wkddnjsxo |

현재 package 경계, 본선 입력 규칙과 Phase 4 이관 gate는
[Perception 개발·통합 가이드](../packages/asmc_perception/README.md)를 따른다. P4-C6에서
camera/BEV/lane/traffic ROS node, runtime support/config/model과 가치 있는 학습·offline audit를
`asmc_perception` 안의 명시적 경계로 통합했다. 실행·참조가 없는 camera 3D 연구와 기존 호환
package는 Git history를 남기고 제거했다.
`data_collection`과 `lidar+sensorfusion`은 통합 대상이 아닌 독립 경계다.

### interfaces (⬜ 합의 중)

| 경로 | 내용 | 주관 |
|------|------|------|
| `src/interfaces/README.md` | τ·BEV 초안 — [architecture-overview.md](./system-overview.md) §5 | @ahnsh03 (3파트 합의) |

### 공통

| 패키지 | 경로 | GitHub |
|--------|------|--------|
| ASMC 공용 메시지 | `src/asmc_msgs` (`1.0.0`, P1-B frozen contract) | 팀 공용, 계약 변경 시 3파트 리뷰 |
| morai_msgs | `src/MORAI-ROS_morai_msgs` (submodule, `beta_drive`) | @ahnsh03 (핀·브랜치 변경) |

## 3. 수정 권한 매트릭스

| 경로 | @yuntae12-sudo<br>정윤태 | @yangseojun<br>양서준 | @kante2<br>이강태 | @ahnsh03<br>안승현 | @wkddnjsxo<br>장원태 | @sonshiny<br>손재호 |
|------|--------|--------|--------|--------|--------|--------|
| `src/behavior_planner/` | ✅ | ✅ | 👀 | 👀 | — | — |
| `src/mpc_controller/` | ✅ | ✅ | 👀 | 👀 | — | — |
| `src/frenet_planner/` | 👀 | 👀 | ✅ | 👀 | — | — |
| `src/asmc_bridge/` | 👀 | 👀 | ✅ | ✅ | — | 👀 |
| `tools/research/lbc_teacher_bev/` | — | — | — | ✅ | — | — |
| `scripts/rosbridge.sh` | 👀 | 👀 | 👀 | ✅ | — | — |
| `scripts/udp_bridge.sh` · `src/asmc_bridge/` | 👀 | 👀 | ✅ | ✅ | — | 👀 |
| `tools/aim_scenario_runner/` | — | — | — | 👀 | ✅ | 👀 |
| `tools/grpc_inha_univ/` | — | — | — | 👀 | ✅ | — |
| `src/asmc_perception/` | 👀 | 👀 | 👀 | ✅ 주관 | ✅ traffic 협의 | ✅ LiDAR interface 협의 |
| `src/lane_boundary_adapter/` | ✅ 합의 | ✅ 합의 | 👀 | ✅ 주관 | — | 👀 |
| `src/data_collection/` | — | — | — | ✅ | 👀 | 👀 |
| `src/asmc_hd_map/` | 👀 | 👀 | 👀 | ✅ 주관 | ✅ 관측 자료 협의 | 👀 |
| `docker/` | 👀 | 👀 | 👀 | ✅ | 👀 | ✅ train |
| `docs/` | 👀 | 👀 | 👀 | ✅ | 👀 | 👀 |
| `src/interfaces/` | ✅ 합의 | ✅ 합의 | ✅ 합의 | ✅ 주관 | ✅ 합의 | ✅ 합의 |

✅ 주 수정 · 👀 리뷰·합의 · — 원칙적 미수정

CODEOWNERS 자동 리뷰: [.github/CODEOWNERS](../../.github/CODEOWNERS)

## 4. 레거시 upstream (본 repo 밖)

본 repo에 포함되지 않습니다. diff·이력 확인이 필요할 때 **각자 원하는 경로에 clone**합니다.

| 종류 | GitHub | 용도 |
|------|--------|------|
| planning (구 `mpc_ws`) | [yuntae12-sudo/mpc_ws](https://github.com/yuntae12-sudo/mpc_ws) | 이식 전 참조 |
| learning·scenario (구 `aim_ws`) | [kante2/aim_ws](https://github.com/kante2/aim_ws) | 브랜치별 레거시 |
| perception | [sonshiny/morai-3d-detection](https://github.com/sonshiny/morai-3d-detection) | 이식 전 참조 |
| morai_msgs (26.R1) | [MORAI-ROS_morai_msgs](https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs) `26.R1` | beta diff 참고 |

**금지:** 외부 베이스라인·레거시 코드를 **본 repo에 통째 복사** (이식 PR로만 반영).  
매핑 상세: [integration-plan.md](../archive/integration/wave1-wave2-plan.md)

## 5. 아키텍처 → 경로

| 모듈 | ASMC 경로 |
|------|-----------|
| Behavior / Frenet / MPC | `src/behavior_planner/`, `frenet_planner/`, `mpc_controller/` (v2 ROS 연결) |
| Camera lane/drivable/traffic | `src/asmc_perception/` |
| LiDAR/3D perception | `src/lidar+sensorfusion/` |
| 공통 BEV | camera v3는 `src/asmc_perception/`; 전역 map/LiDAR 결합 규격은 합의 중 |
| Capture·dataset | `src/data_collection/`, `scripts/*perception*`, `scripts/*twinlite*` |
| LBC GT/teacher BEV | `tools/research/lbc_teacher_bev/` |
| Scenario / gRPC | `tools/aim_scenario_runner/`, `tools/grpc_inha_univ/` |
| Bridge | `scripts/udp_bridge.sh` (현행 UDP) · `scripts/rosbridge.sh` (종료된 direct ROS 실험 재현용) |

담당과 검증 책임: [팀 ownership](../onboarding/ownership.md)
