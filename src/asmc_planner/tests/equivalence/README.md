# Frozen equivalence references

These baseline implementations were retained from the completed Phase 5–7
validation harnesses. They are linked only into explicit offline gates, never
into production libraries. Only include lookup paths changed during migration.

| Target | Preserved comparison |
|---|---|
| planner_lifecycle_equivalence | 200,000 acceptance transitions + 1,008 combinations |
| planner_request_equivalence | 200,000 request transactions, serialization and command IDs |
| planner_maneuver_equivalence | 200,000 maneuver transitions |
| planner_fsm_equivalence | 200,000 FSM transitions |
| asmc_internal_flow_equivalence | direct internal call boundary |

All gates are explicit executables. They do not alter the registered 41-test
planner baseline. `phase5_legacy_*` and `planner_baseline_fsm.hpp` are frozen
regression fixtures, not compatibility/runtime implementations.
