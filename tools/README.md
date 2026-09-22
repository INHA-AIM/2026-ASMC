# tools/

catkin이 아닌 **Python·gRPC·시나리오** 도구.

| 경로 | 담당 | 엔트리 |
|------|------|--------|
| `aim_scenario_runner/` | 장원태 | `run.sh`, `scenario_launcher.py` |
| `grpc_inha_univ/` | 장원태 | `run_mode.sh`, `src/api/morai_sim_client.py` |
| `audit/` | 통합 회귀·계약 기준 | `build_runtime_baseline.py --write\|--check`, `validate_p1b_contract.py`, `check_documentation_migration.py` |

설정: `aim_scenario_runner/config/runtime.yaml` (gRPC host)  
담당과 리뷰 경계: [팀 ownership](../docs/onboarding/ownership.md)
검증: [운영 가이드 §7](../docs/operations/README.md#7-변경-범위별-시뮬-검증)

```bash
./tools/aim_scenario_runner/run.sh
```

P1-A runtime baseline generator는 ROS CLI 없이 ROS1 bag v2 index를 읽는다. local message MD5는 generated
header를 우선 사용하고, 아직 build되지 않은 `asmc_msgs`에는 검증된 ROS1 source 계산기를
사용한다. 다른 package의 generated header가 없는 clean workspace에서는 먼저 catkin build를 수행한다.

커밋된 `runtime_baseline_v1`은 content commit `73c0a8c`의 frozen snapshot이다. 현재 HEAD에는
승인된 HD Map/message 후속 변경이 있으므로 아래 v1 `--write`를 실행해 덮어쓰지 않는다.
v1 byte parity 검사는 해당 commit을 별도 worktree에서 checkout했을 때만 의미가 있다. 다음
repository freeze는 output 이름과 baseline ID를 v2로 올리는 별도 변경으로 수행한다.

```bash
python3 tools/audit/validate_p1b_contract.py
python3 -m unittest tools/audit/test_p1b_contract.py
python3 tools/audit/check_documentation_migration.py
# D1 이전 commit 이력이 있는 clone에서 원문 inventory와 byte 이동까지 확인
python3 tools/audit/check_documentation_migration.py --with-history
# ROS Noetic workspace를 build/source한 환경에서 실행
python3 -m unittest src/asmc_msgs/test/test_message_serialization.py
```
