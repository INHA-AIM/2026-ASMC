# tools/

catkin이 아닌 **Python·gRPC·시나리오** 도구.

| 경로 | 담당 | 엔트리 |
|------|------|--------|
| `aim_scenario_runner/` | 장원태 | `run.sh`, `scenario_launcher.py` |
| `grpc_inha_univ/` | 장원태 | `run_mode.sh`, `src/api/morai_sim_client.py` |
| `scen_runner/` | 장원태 | `run_bridge.sh`·`run_perception.sh`·`run_planning.sh`·`run_scenario.sh`, 독립 `run_validation.sh` |
| `audit/` | 통합 회귀·계약 기준 | `build_runtime_baseline.py`, `build_perception_runtime_baseline.py`, `validate_p1b_contract.py`, `validate_perception_contract.py`, `validate_perception_runtime_acceptance.py`, `validate_bridge_profiles.py` |
| `bridge/` | MORAI release template에서 transport별 import 생성 | `render_morai_network_profiles.py --write\|--check` |
| `research/lbc_teacher_bev/` | privileged MORAI state 기반 GT/teacher BEV 연구 도구 | `scripts/visualize_lbc_bev.sh`, `scripts/collect_lbc_bev.sh` |

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

`asmc_hd_map` publisher는 v1 전체 snapshot 대신
[`config/baselines/hd_map_runtime_pins_v1.yaml`](../config/baselines/hd_map_runtime_pins_v1.yaml)의
`hd_map`·`competition_inputs.metadata_bundle` pin을 읽는다. 검수된 HD맵이나 metadata를 바꾸면
`python3 src/asmc_hd_map/scripts/build_runtime_pins.py --write`로 같은 PR에서 갱신하고 `--check`로 확인한다.

```bash
python3 tools/audit/validate_p1b_contract.py
python3 tools/audit/validate_perception_contract.py
python3 tools/audit/validate_perception_topic_policy.py
python3 tools/audit/validate_perception_runtime_acceptance.py
python3 tools/audit/validate_bridge_profiles.py
python3 -m unittest tools/audit/test_bridge_profiles.py
python3 -m unittest tools/audit/test_p1b_contract.py
python3 -m unittest tools/audit/test_perception_contract.py
python3 -m unittest tools/audit/test_perception_topic_policy.py
python3 -m unittest tools/audit/test_perception_runtime_acceptance.py
python3 tools/audit/check_documentation_migration.py
python3 -m unittest tools/audit/test_perception_runtime_baseline.py
# D1 이전 commit 이력이 있는 clone에서 원문 inventory와 byte 이동까지 확인
python3 tools/audit/check_documentation_migration.py --with-history
# ROS Noetic workspace를 build/source한 환경에서 실행
python3 -m unittest src/asmc_msgs/test/test_message_serialization.py
```

`validate_perception_runtime_acceptance.py`는 source·launch·판제 consumer 경로뿐 아니라 P5-D/E
handoff 상태, 과거 호환 launch의 신호등 중복 방지 설정, representative lane-family 근거와 운영
container·시각화 진입점이 계약에서 벗어나지 않았는지 ROS 없이 검사한다. 실행 중
rate·stamp·frame·publisher/subscriber 검사는 perception을
먼저 실행한 뒤 저장소 루트의 `./scripts/verify_perception.sh`로 수행한다. verifier 자체가 인지
노드를 대신 시작하지 않으므로, 실패가 숨겨지거나 중복 producer가 생기지 않는다.

`perception_runtime_baseline_v1`도 P4-A 입력 commit `1bd8ec9`의 구조 snapshot이다.
`build_perception_runtime_baseline.py --check`는 그 commit을 별도 worktree에서 재현할 때만
사용한다. 현재 HEAD에서는 정본 package closeout과 충돌하므로 frozen manifest의 schema·핵심
값만 `test_perception_runtime_baseline.py`로 검사한다.

현재 실행 인자/명령 문서는 `python3 tools/audit/check_runtime_documentation.py`로 검사합니다.
launch 인자가 바뀌면 `--write`로 자동 목록을 갱신합니다. 현재 통합 판제의 traffic_light 기본true와
외부 카메라 사용 시 false 명시를 과거 P5 호환 기본값과 구분합니다.
