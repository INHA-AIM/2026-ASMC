# Configuration index

런타임 설정 정본의 진입점이다. 새 파일을 top-level에 추가하지 말고 소유 영역에 둔다.

| 경로 | 내용 | 변경 단계 |
|---|---|---|
| [`bridge/`](./bridge/) | P3-A bridge·transport 기계 판독 계약 | Phase 3 |
| [`morai/`](./morai/) | MORAI network import와 `ASMC_sensor_set_*` | Phase 3 |
| [`contracts/`](./contracts/) | ROS message/topic 계약과 `asmc_msgs` 소유 registry | Phase 1·4 |
| [`baselines/`](./baselines/) | P1 저장소 snapshot과 P4 인지 runtime 이관 baseline | Phase 1·4 |
| `perception_target_policy_*.json` | perception target 정책의 현행 호환 경로 | Phase 4에서 이동 검토 |
| `twinlite_*.json` | perception 실험 표본 | Phase 4에서 이동 검토 |
| `docs/` | 문서 이동 검증 manifest | 문서 governance |

MORAI 설정을 변경한 뒤에는 다음을 실행한다.

```bash
python3 tools/bridge/render_morai_network_profiles.py --write
python3 tools/audit/validate_bridge_profiles.py
python3 -m unittest tools/audit/test_bridge_profiles.py
```

`generated/` 파일을 직접 고치면 검증이 실패한다. simulator UI export는 release별 `templates/`에
반영한 뒤 재생성한다. Phase 4 소유 top-level perception 파일은 consumer 경로를 함께 바꾸는
별도 PR 전까지 그대로 둔다.

P4-A 인지 기준선은 현재 분산 package를 이동하지 않고 topic/type/frame/stamp, model SHA,
bag profile과 oracle 금지선을 기록한다.

```bash
python3 tools/audit/build_perception_runtime_baseline.py --check
python3 -m unittest tools/audit/test_perception_runtime_baseline.py
python3 tools/audit/validate_perception_contract.py
python3 -m unittest tools/audit/test_perception_contract.py
python3 tools/audit/validate_traffic_light_parity.py --check-artifacts
python3 -m unittest tools/audit/test_traffic_light_parity_report.py
```

`perception_traffic_light_parity_v1.yaml`은 대표 bag을 기존·canonical producer에 같은 순서로
공급해 String/typed/source-header/stale/recovery parity를 통과한 P4-C2 근거다. bag은
복사하지 않고 `$ASMC_DATA` 상대 경로·크기·SHA만 보존한다.

P4-C3 consumer binding은 별도 topic을 다시 만들지 않는다. 기계 정본의
`consumer_bindings.behavior_planner`와 `src/behavior_planner/src/config/behavior_params.yaml`이
typed 우선·legacy fallback·invalid/stale fail-safe 규칙을 함께 고정하며
`validate_perception_contract.py`가 두 launch의 canonical producer 경로까지 검사한다.
