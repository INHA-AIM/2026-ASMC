# AIM Scenario Runner (ASMC)

jang `aim_scenario_runner` — MORAI gRPC 시나리오 자동화. **LBC route BEV visualizer는 기본 비활성** (va ROS LBC 기준, jang gRPC TL은 추후 D-09).

## 의존성 (repo 내)

| 경로 | 용도 |
|------|------|
| `tools/grpc_inha_univ/` | MORAI gRPC proto·API |
| `R_KR_PG_KATRI/` | MGeo 맵 JSON |
| `src/MORAI-ROS_morai_msgs` | submodule `beta_drive` |

## 실행

```bash
# MORAI gRPC 서버 활성 (시뮬 UI)
# config/runtime.yaml 의 grpc.host — WSL에서는 보통 Windows 호스트 IP
#   ip route | grep default | awk '{print $3}'

./tools/aim_scenario_runner/run.sh
# 또는
./tools/aim_scenario_runner/run.sh urban random_route_drive
```

머신별 override: `config/local_override.yaml` (예시: `local_override.yaml.example`)

## route BEV visualizer (선택)

`config/urban_scenarios.yaml`에서 `route_bev_visualizer_enabled: true` 시 va `learning_by_cheating` visualizer를 subprocess로 띄움.  
**대회·통합 기본값은 false.** gRPC multi-TL 검증(D-09) 전에는 켜지 않는 것을 권장.

## 시뮬 검증

[docs/operations/sim-verification-checklist.md](../../docs/operations/sim-verification-checklist.md) §4 — **장원태**
