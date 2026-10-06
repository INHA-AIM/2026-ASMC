# Configuration index

런타임 설정 정본의 진입점이다. 새 파일을 top-level에 추가하지 말고 소유 영역에 둔다.

| 경로 | 내용 | 변경 단계 |
|---|---|---|
| [`bridge/`](./bridge/) | P3-A bridge·transport 기계 판독 계약 | Phase 3 |
| [`morai/`](./morai/) | MORAI network import와 `ASMC_sensor_set_*` | Phase 3 |
| [`contracts/`](./contracts/) | ROS message/topic 계약과 `asmc_msgs` 소유 registry | Phase 1·4 |
| [`baselines/`](./baselines/) | P1 저장소 snapshot과 P4 인지 runtime 이관 baseline | Phase 1·4 |
| `perception_target_policy_*.json` | dataset contract·data collection이 공유하는 target 정책 정본 | consumer 동시 변경 전까지 유지 |
| `twinlite_*.json` | 재현 가능한 perception 실험 표본 정보 | 학습·문서 consumer 공유 |
| `docs/` | 문서 이동 검증 manifest | 문서 governance |

MORAI 설정을 변경한 뒤에는 다음을 실행한다.

```bash
python3 tools/bridge/render_morai_network_profiles.py --write
python3 tools/audit/validate_bridge_profiles.py
python3 -m unittest tools/audit/test_bridge_profiles.py
```

`generated/` 파일을 직접 고치면 검증이 실패한다. simulator UI export는 release별 `templates/`에
반영한 뒤 재생성한다. top-level perception JSON은 `asmc_camera`, `data_collection`,
dataset contract가 함께 쓰는 저장소 공통 정책이므로 현재 경로를 유지한다. 이동하려면
모든 consumer와 문서 링크를 같은 PR에서 바꾸고 재현 검증을 통과해야 한다.

P4-A 인지 기준선은 통합 전 분산 package의 topic/type/frame/stamp, model SHA,
bag profile과 oracle 금지선을 기록한 **동결 snapshot**이다. 현재 실행 경로는
`src/asmc_camera/`이다. snapshot 내용 테스트와 현재 계약·parity 검증은 아래처럼
실행한다.

```bash
python3 -m unittest tools/audit/test_perception_runtime_baseline.py
python3 tools/audit/validate_perception_contract.py
python3 -m unittest tools/audit/test_perception_contract.py
python3 tools/audit/validate_perception_topic_policy.py
python3 -m unittest tools/audit/test_perception_topic_policy.py
python3 tools/audit/validate_perception_runtime_acceptance.py
python3 -m unittest tools/audit/test_perception_runtime_acceptance.py
python3 tools/audit/validate_traffic_light_parity.py --check-artifacts
python3 -m unittest tools/audit/test_traffic_light_parity_report.py
```

`build_perception_runtime_baseline.py --check`는 구 package가 존재했던 input commit
`1bd8ec9`를 **별도 worktree로 checkout했을 때만** P4-A를 재생성하는 역사 도구다.
현재 tree에서 실행하면 이식 후 삭제된 경로를 의도대로 누락으로 판정하므로 현행
검증 명령으로 사용하지 않는다.

`perception_traffic_light_parity_v1.yaml`은 대표 bag을 기존·canonical producer에 같은 순서로
공급해 String/typed/source-header/stale/recovery parity를 통과한 P4-C2 근거다. bag은
복사하지 않고 `$ASMC_DATA` 상대 경로·크기·SHA만 보존한다.

P4-C3 consumer binding은 별도 topic을 다시 만들지 않는다. 기계 정본의
`consumer_bindings.behavior_planner`와 `src/behavior_planner/src/config/behavior_params.yaml`이
typed 우선·legacy fallback·invalid/stale fail-safe 규칙을 함께 고정하며
`validate_perception_contract.py`가 두 launch의 canonical producer 경로까지 검사한다.

P5-A topic policy는 이름에 `debug`·`grid`가 있어도 fusion/lane의 필수 내부 입력인 토픽과
실제 opt-in 후보를 구분한다. `perception_topic_policy_v1.yaml`을 먼저 갱신하지 않고 runtime
publisher, subscriber 또는 result bag profile을 제거하지 않는다.

P5-C 실행 수락 정본은
[`contracts/perception_runtime_acceptance_v1.yaml`](./contracts/perception_runtime_acceptance_v1.yaml)이다.
P5-A의 정적 inventory와 달리 현재 ROS graph의 node, topic type, 유일 producer, 필수 subscriber,
rate, source stamp, frame과 기본-OFF topic 부재를 측정한다. 신호등·판제·시각화·임시 lane adapter는
함께 실행할 때만 명시적인 verifier 옵션으로 gate에 포함한다. 이 계약은 기존 topic을 rename하지
않고 현재 연결을 회귀 기준으로 고정한다. lane 정본은 shared
`/perception/lane/boundary_family`이며 legacy `boundary_observations`는 기본-OFF 호환 경로다.
같은 파일의 `phase_status`, `consumer_handoff`, `operational_closeout`이 P5-D/E까지 고정한다.
신호등 producer 소유권·typed fail-closed 정책, lane-family 정본, planning launch 기본값, 컨테이너
경계와 rqt/RViz 진입점은 별도 임의 설정이 아니라 이 계약과
[closeout 기록](../docs/records/baselines/perception-p5de-closeout-v1.md)을 함께 따른다.
