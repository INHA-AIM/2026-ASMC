# tools/

catkin이 아닌 **Python·gRPC·시나리오** 도구.

| 경로 | 담당 | 엔트리 |
|------|------|--------|
| `aim_scenario_runner/` | 장원태 | `run.sh`, `scenario_launcher.py` |
| `grpc_inha_univ/` | 장원태 | `run_mode.sh`, `src/api/morai_sim_client.py` |
| `audit/` | 통합 회귀·계약 기준 | `build_runtime_baseline.py --write\|--check`, `validate_p1b_contract.py` |

설정: `aim_scenario_runner/config/runtime.yaml` (gRPC host)  
가이드: [docs/team-dev-guide.md](../docs/team-dev-guide.md) §5  
검증: [docs/sim-verification-checklist.md](../docs/sim-verification-checklist.md) §4

```bash
./tools/aim_scenario_runner/run.sh
```

P1-A runtime baseline은 ROS CLI 없이 ROS1 bag v2 index를 읽는다. local message MD5는 generated
header를 우선 사용하고, 아직 build되지 않은 `asmc_msgs`에는 검증된 ROS1 source 계산기를
사용한다. 다른 package의 generated header가 없는 clean workspace에서는 먼저 catkin build를 수행한다.

```bash
python3 tools/audit/build_runtime_baseline.py --write
python3 tools/audit/build_runtime_baseline.py --check
python3 tools/audit/validate_p1b_contract.py
python3 -m unittest tools/audit/test_p1b_contract.py
# ROS Noetic workspace를 build/source한 환경에서 실행
python3 -m unittest src/asmc_msgs/test/test_message_serialization.py
```
