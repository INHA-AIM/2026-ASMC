# 통합 계획·진행 상황

> **통합 리드:** 안승현  
> **최종 수정:** 2026-07-08

## 1. 전략

1. **환경·규약·Docker** 통일 (본 문서 시점)
2. **폴더 격리** 후 모듈별 `integrate/*` PR
3. 레거시 **GitHub upstream**은 읽기 전용 참조 — 통합 후에도 diff용 (본 repo 미포함)

**하지 않는 것**
- 외부 베이스라인·논문 코드를 ASMC에 복사
- aim_ws 3브랜치 통째 merge
- `main`에 개인별 전체 복사본
- **26.R1 필드명 그대로 merge** — 이식 PR마다 [beta 정렬](./morai-msgs-beta-policy.md) 선행

## 2. 진행 체크리스트

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
| 6 | `interfaces/` τ·BEV 확정 | ⬜ [architecture-overview.md](./architecture-overview.md) §5 |
| — | jang LBC gRPC multi-TL | ⬜ **추후** D-09 검증 후 판단 (통합 차단 아님) |
| — | morai-3d-detection 시나리오 ROS 연동 (`adb1056`) | ⬜ **추후** — scenario runner `/dataset_control` publisher 선행 필요 |
| — | ochang `gps_logger` | ❌ **미사용** — 이식하지 않음 |

## 3. 레거시 upstream 동기화 (2026-07-08 기준)

아래는 **GitHub upstream HEAD**입니다. 필요 시 각자 clone한 경로에서 `git pull`합니다.

| upstream | 브랜치 | HEAD | 통합 이후 신규 | ASMC 반영 |
|----------|--------|------|----------------|-----------|
| [yuntae12-sudo/mpc_ws](https://github.com/yuntae12-sudo/mpc_ws) | `main` | `72e2a37` (07-08) | **3커밋** — behavior_planner, bridge, 주행 버그픽스 | ✅ PR #4 |
| [sonshiny/morai-3d-detection](https://github.com/sonshiny/morai-3d-detection) | `main` | `adb1056` (07-08) | **1커밋** — `/dataset_control` ROS 연동 | ⬜ defer |
| [kante2/aim_ws](https://github.com/kante2/aim_ws) | `va_seunghyun` | `1ae08b7` (07-05) | 없음 | ✅ 동기화 |
| [kante2/aim_ws](https://github.com/kante2/aim_ws) | `jang` | `e6b282e` (07-06) | 없음 | ✅ 동기화 |
| [kante2/aim_ws](https://github.com/kante2/aim_ws) | `ochang` | `2b6eb4b` (06-23) | 없음 | ❌ skip (`gps_logger` 미사용) |

```bash
# 예: mpc_ws를 별도 clone해 두었다면
cd <mpc_ws-clone> && git pull origin main
```

## 4. 레거시 → ASMC 매핑

| 출처 (GitHub) | ASMC | 상태 |
|---------------|------|------|
| [yuntae12-sudo/mpc_ws](https://github.com/yuntae12-sudo/mpc_ws) | `mpc_controller`, `planner`, `behavior_planner`, `integration_launch` | ✅ v1 + v2 (PR #1·#4) |
| [kante2/aim_ws](https://github.com/kante2/aim_ws) `va_seunghyun` | `learning_by_cheating`, bridge, mgeo | ✅ main |
| [kante2/aim_ws](https://github.com/kante2/aim_ws) `jang` | `tools/aim_scenario_runner`, `tools/grpc_inha_univ` | ✅ main |
| [kante2/aim_ws](https://github.com/kante2/aim_ws) `ochang` | — | ❌ skip (`gps_logger` 미사용) |
| [sonshiny/morai-3d-detection](https://github.com/sonshiny/morai-3d-detection) | `src/perception/morai_3d_detection` | ✅ v1; ROS 시나리오 연동 defer |

## 5. planning-v2 요약 (mpc_ws `72e2a37`)

> 이 절은 ROS 기반 이전 구현 기록이다. 현재 정본은
> [planning-control-udp.md](./planning-control-udp.md)이며 upstream 기준은 `465833e`다.

**신규 패키지:** `src/behavior_planner/` — MGeo 기반 상위 의사결정, `/behavior/context` publish

**planner 변경:**
- FSM 제거, `behavior_bridge.cpp` — BehaviorContext → PlannerCommand
- `/planner/plan_feedback` publish
- `path_route1.txt` (route_links_1 기준, 5,299 wp) — launch에서 `$(find mpc_node)` 주입

**mpc 변경:**
- `/frenet_planner/trajectory` 구독 + CSV fallback
- 저속·곡률 버그픽스 (`72e2a37`)
- `curve_lookahead_m` 파라미터 추가

**integration.launch:** behavior_planner → planner → mpc 순 include

**아키텍처 상세:** [planning-behavior-handoff.md](./planning-behavior-handoff.md) (mpc_ws 핸드오프 문서 이식)

**빌드 의존성 추가:** `libyaml-cpp-dev` (Dockerfile 반영)

## 6. LBC 신호등 — 통합 전 확인 (요약)

| 구분 | va_seunghyun | jang |
|------|--------------|------|
| `morai_adapters.py` | 동일 | 동일 (차용, 미수정) |
| `lbc_bev_collector` | ROS 토픽만 | ROS 토픽만 (save 경로 등 소규모 diff) |
| `lbc_bev_visualizer` | ROS, `ego_facing_only` | **gRPC 폴링** + ROS 병합, scenario 연동 |

**통합 기준선 (확정 2026-07-07):** LBC·학습 GT는 **va_seunghyun**만 이식. jang의 gRPC multi-TL visualizer는 **추후 D-09 검증 후** cherry-pick 여부 판단.

## 7. 미결정 (D-08)

KATRI 확장 맵 레이어를 K-city 2025로 제공받을 수 있는지 **MORAI 디스코드 문의 중** (답변 대기).

| 시나리오 | 대응 |
|----------|------|
| A. 제공됨 | `data/`·MapService를 2025 기준으로 전환 |
| B. 미제공 | KATRI 보조 + 2025 link/node 병행 (현행) |

## 8. 추후 검토 (D-09) LBC 신호등 — jang gRPC

**상태:** 통합 **차단 안 함**. va 기준으로 진행, jang LBC는 별도 검증 후 업데이트 판단.

검증 절차: `spike/tl-bev-validation` — [sim-verification-checklist.md](./sim-verification-checklist.md) §4

## 9. 추후 검토 (D-10) perception 시나리오 ROS 연동

**upstream:** `morai-3d-detection` `adb1056` — collector가 `/dataset_control`, `/dataset_status` 토픽 수신

**갭:** `tools/aim_scenario_runner`에 publisher 없음 → `run_collect.sh`도 `/home/jang/aim_ws` 하드코딩

**조치:** 장원태(jang) + 손재호(jaeho)가 scenario runner에 publisher 추가 후 일괄 PR

## 10. 통합 실행 순서

| 순서 | 브랜치 | 작업 | 완료 조건 |
|------|--------|------|-----------|
| **0** | — | 문서·Docker·beta 규약 | ✅ |
| **1** | `integrate/planning-v1` | morai_msgs submodule, mpc yaml 경로 | ✅ merged |
| **2** | `integrate/learning` | va_seunghyun LBC/bridge | ✅ merged |
| **3** | `integrate/scenario-perception` | scenario + 3D detection | ✅ merged |
| **1b** | `integrate/planning-v2` | behavior_planner + bridge + bugfix | ✅ PR #4 merged |
| **4** | `integrate/interfaces` | τ·BEV·토픽명 합의 | ⬜ |
| **5** | 팀별 `feature/*` | planning UDP, perception 추론 노드 등 | ASMC 위에서 개발 |

**이식하지 않음:** `ochang` 브랜치 통째(·`gps_logger` 포함), `aim_ws/src/main` 레거시, 외부 베이스라인 복사, jang `lbc_bev_visualizer` gRPC 경로(추후 판단).

## 11. 다음 작업 (통합 리드)

1. **시뮬 검증** — [sim-verification-checklist.md](./sim-verification-checklist.md) (merge 후 전원)
2. **팀원 일상 개발** — 경로·주의: [team-dev-guide.md](./team-dev-guide.md)
3. **`integrate/interfaces`** — architecture §5 τ·BEV 확정
4. D-10: jang+jaeho scenario↔collector ROS 연동
5. 팀별 `feature/*` — UDP, 추론 노드 등
