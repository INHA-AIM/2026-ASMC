# asmc_planner

Single production package: Input/Parser → PlannerContext → BehaviorDecision →
ExecutionRequest → FrenetResult → Execution update → PlanningResult → ROS output.
Internal handoff uses C++ structures and direct calls. Behavior runs at 10 Hz;
Planning runs at 20 Hz. ROS topic and parameter contracts remain unchanged,
except `UrbanFeatureDebug` is now `asmc_msgs/UrbanFeatureDebug`.

```bash
roslaunch asmc_planner autonomy_no_bridges_v2.launch objects_source:=lidar traffic_light:=false
```

`behavior_planner` and `frenet_planner` packages have been removed. Their C++
namespaces and parameter namespaces remain for source/parameter compatibility.
Do not rename them as part of ownership cleanup.

## Regression assets

- `tests/behavior`: six registered Behavior tests and dashboard test.
- `tests/frenet`: 35 registered Frenet tests, fixtures and explicit v2 ROS test.
- `tests/equivalence`: frozen lifecycle/request/FSM comparisons and maneuver gate.
- `tests/acceptance`: isolated Behavior decision-loop helper (no control outputs).
- `tools`: test publishers and offline shadow/visualization tools.
- `scripts`, `web`: dashboard and calibration tools.

Known baseline: registered planner 39/41, AVOID 111 assertion failures;
`fsm_command_binding_test` retains its original missing-main link failure.
Explicit v2 ROS regression: 113/177. Assertions are unchanged by migration.

HD-map input paths now point to `config/behavior`. Historical hashes, map
provenance and frozen interface documents retain their recorded original paths.

Map source JSON values are unchanged. Their original CRLF bytes are retained so
frozen provenance SHA256 values remain valid. `asmc_hd_map.asset_paths` maps
historical recorded source identities to current assets without rewriting
immutable map products, manifests or reports.

Observer-only dashboard (does not start another planner):

```bash
roslaunch asmc_planner dashboard.launch state_topic:=/planning/vehicle/state
```
