# 통합 계획·진행 상황

> **문서 역할:** Historical Record — Wave 1/2 이식 당시의 계획과 결과
> **Archive 허브:** [README.md](README.md)
> **현재 우선순위:** [시스템 구조](../../architecture/system-overview.md) · [통합 로드맵](../../architecture/consolidation-roadmap.md) · [Perception guide](../../../src/asmc_camera/docs/integration.md)
> **주의:** 이 문서의 날짜·다음 작업을 현재 일정으로 사용하지 않는다.
> **문서 허브:** [README.md](../../README.md)
> **통합 리드:** 안승현
> **최종 수정:** 2026-07-25
> 당시 현황 원문: 상위 개인 workspace `docs/competition/team-status.md` (팀 clone 미포함) ·
> 현재 Git 규칙: [협업 가이드](../../onboarding/collaboration.md)

## 1. 전략 (Phase 0 / 1)

| 구간 | 내용 |
|------|------|
| **Wave 1 (완료)** | PR #1–#4 — mpc/behavior, LBC, scenario, Det3D v1 이식 |
| **Phase 0 (~8/7)** | 개인 레포에서 기능 완성; ASMC = 문서·인프라·Loc-Aid/bake/eval 스텁 |
| **Wave 2 (8/7~)** | traffic_runner, va bridge, mpc 최신, Det3D, Ahn Loc-Aid/bake → ASMC |

**하지 않는 것:** 베이스라인 통째 복사, privileged 배포 경로, ochang 이식, 빨간 main WIP.

## 2. Wave 1 체크리스트 (완료)

| 단계 | 내용 | 상태 |
|------|------|------|
| 0 | `docs/`, Docker, `scripts/`, CONTRIBUTING, **beta 정렬 규약** | ✅ |
| 1 | `mpc_ws` → `src/{mpc_controller,planner,integration_launch}` | ✅ PR #1 merged |
| 2 | `morai_msgs` submodule (`beta_drive`) | ✅ PR #1 |
| 3 | `aim_ws` learning/LBC/bridge | ✅ PR #2 merged |
| 4 | `aim_scenario_runner` (jang) | ✅ PR #3 merged |
| 5 | `morai-3d-detection` (jaeho) | ✅ PR #3 merged |
| **1b** | **planning-v2:** `behavior_planner` + bridge + 주행 버그픽스 | ✅ PR #4 merged |
| **1c** | **planning UDP-only:** standalone Frenet + MPC, Behavior 미연동 | ✅ `feature/yuntae-planner-controller` |
| 6 | `interfaces/` τ·BEV 확정 | ⬜ [architecture-overview.md](../../architecture/system-overview.md) §5 |
| — | jang LBC gRPC multi-TL | ⬜ **추후** D-09 검증 후 판단 (통합 차단 아님) |
| — | morai-3d-detection 시나리오 ROS 연동 (`adb1056`) | ⬜ **추후** — scenario runner `/dataset_control` publisher 선행 필요 |
| — | ochang `gps_logger` | ❌ **미사용** — 이식하지 않음 |

## 3. Wave 2 체크리스트 (8/7~)

| ID | 내용 | Upstream | 상태 |
|----|------|----------|------|
| W2-01 | `morai_udp_bridge` + `morai_sensor_bridge` 포팅 · `udp_bridge.sh` | `aim_ws-va` | ✅ 복사 (필드/포트 개선은 후속) |
| W2-02 | traffic_runner TL → ASMC perception | `traffic_runner` | ⬜ |
| W2-03 | mpc_ws 최신 제어 동기화 | `mpc_ws` `e1c0d84`+ | ⬜ |
| W2-04 | Det3D 최신 (`67dbaf0`+) | `morai-3d-detection` | ⬜ |
| W2-05 | Loc-Aid / bake / BEV / 차선 등 인지 후보 | ASMC `drivable_bev` | ⬜ 스텁·실험 |
| W2-06 | UDP vs ROS+bridge 팀 합의 반영 | 회의 | ⬜ |
| W2-07 | interfaces tracks/TL/pose 확정 | 3파트 | ⬜ |

## 4. Upstream HEAD (2026-07-25)

| upstream | 브랜치 | HEAD | 비고 |
|----------|--------|------|------|
| mpc_ws | main | `e1c0d84` | 활성 |
| morai-3d-detection | main | `67dbaf0` | 활성 |
| aim_ws | **va** | `6534544` | 이강태 활성 |
| traffic_runner | main | `6f0f544` | 장원태 활성 |
| 2026-summer-VIP2 | main | `32cdd94` | 안승현 참조 |
| aim_ws | jang | `0bb16cb` | 동결 |
| aim_ws | ochang | `2b6eb4b` | **Deprecated** |
| aim_ws | va_seunghyun | `1ae08b7` | 참조 |

당시 clone inventory는 상위 개인 workspace `external/team/README.md`에 있으며 팀 repository에는
포함하지 않는다. 필요한 upstream commit은 위 표에 보존했다.

## 5. Wave 1 요약 (유지)

planning-v2 · LBC · scenario · Det3D v1 — 아래 내용은 당시 이식 기록으로 유효하다.
현재 전체 운영: [운영 가이드](../../operations/README.md)
인지 eval: [perception-agentic-loop.md](agentic-loop.md)

> 이 절은 ROS 기반 이전 구현과 이후 standalone 전환 당시의 기록이다. standalone 내용은
> [historical UDP baseline](standalone-udp-baseline.md), 현재 실행은
> [Planning/Control 운영 규약](../../operations/planning-control.md)을 따른다. 당시 upstream
> 기준은 `465833e`다.

**신규 패키지:** `src/behavior_planner/` — MGeo 기반 상위 의사결정, `/behavior/context` publish

## 6. 다음 작업

1. Phase 0: 각자 개인 레포 기능 완성 + 안승현 Loc-Aid/bake
2. 회의: UDP-only vs bridge+ROS
3. 8/7~ Wave 2 이식 순서: **bridge → interfaces → Det3D/TL → Loc-Aid/bake → mpc 정렬**
