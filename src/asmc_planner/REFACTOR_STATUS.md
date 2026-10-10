# Planner Refactor Autonomous Run Report

## 1. Last completed phase

Phase 8 single-package integration COMPLETE. Phase 7 was restored and its full
build, registered test set and explicit ROS regression revalidated first.
The rejected inter-package composition draft was removed.

## 2. Final structure

All planner implementations and candidate execution are in `src/asmc_planner`:
Global; Parser and traffic input; Behavior context/feature/rule/decision/map;
Execution acceptance, request transaction and lifecycle; Frenet math/maneuvers;
Planning contexts/coordinator/production runtime; Output; Visualization.
The candidate is `asmc_planner/asmc_planner_node`, launched by
`asmc_planner/launch/planner_v2.launch`.

## 3. Ownership before/after

The two legacy packages previously owned runtime implementations. Required
implementations now belong to asmc_planner. Behavior produces semantic intent;
Execution retains separate requested and accepted IDs and owns active lifecycle;
Frenet calculates paths; Planning preserves cycle validation/fallback; ROS
adapters keep external I/O. Two existing steady timer cadences remain independent.
No new clock or policy was introduced.

## 4. Files moved/split

111 additional authoritative files migrated (on top of Phase 1–7 work).
104 match the prior bodies after include relocation and compatibility class
renaming are removed; seven adapter files have the reviewed direct-call wiring,
feedback snapshot synchronization or result-view changes. Existing v1/v2
orchestration differs only by result-view wrappers and relocated includes.
The new production main and runtime are owned by asmc_planner. Migration mapping
and audit records: /tmp/phase8_single_mapping.json and
/tmp/phase8_single_body_audit.json on the host.

## 5. Compatibility retained

Legacy executables, launch files, ROS message definitions, header paths and
library target names remain. Library shims contain no algorithm implementation.
Both legacy packages depend on asmc_planner, with no dependency on each other.
asmc_planner has no build/runtime dependency on either legacy planner package.
Legacy test source and configuration contents are unchanged.

External topic names, enum values and serialized message definitions remain.
The legacy debug datatype is generated inside asmc_planner from its unchanged
schema; generated headers compare byte-for-byte. Existing generated ROS value
structs are retained in command/trajectory interfaces to preserve contracts;
there is no internal ROS transport requirement. Complete ROS-type decoupling is
not claimed.

## 6. Build result

Full workspace PASS after Phase 7 restoration and after single-package Phase 8.
Runnable Behavior gtest targets rebuilt. Candidate ELF dependencies contain
only asmc_planner-owned planner libraries, not legacy planner libraries.
Three key production units also compile with legacy include directories removed
and only input-contract headers plus the package's own generated debug header.

Logs in container: /tmp/phase7_restored_build.log,
/tmp/phase8_single_final_build.log, /tmp/phase8_behavior_tests_build.log.

## 7. Registered tests before/after

All 41 registered planner tests: 39 PASS / 2 FAIL -> 39 PASS / 2 FAIL.
The exact 111 AVOID failing assertions and missing-main failure origin match.
Log: /tmp/phase8_single_registered.log (container).
No existing test was changed to obtain this result.

## 8. ROS regression before/after

Restored Phase 7: 113/177. Legacy compatibility runtime: 113/177.
Final candidate run without diagnostic observers: 113/177.
Both final runs retain exactly the same 64 failing labels.
Logs: /tmp/phase8_single_legacy_v2.log and
/tmp/phase8_single_candidate_unobserved_v2.log (container).

Earlier candidate trials returned 112/177 and 111/177 with additional
window-sensitive checks. Diagnostics showed the blocked request in WAITING
(state 2), source NEW, with no COMMIT; a separate trace showed CANCEL actually
emitting IDLE and repeated-CANCEL IDLE at 50 ms intervals. Another extra check
was the previously documented 60 ms direct-EXECUTE feedback-count assertion.
The initial extra blocked assertion was not reproduced in subsequent runs.
The final unmodified test run matches the baseline exactly. These observations
support feedback-window/transport variability; no assertion, timeout, production
clock or policy was changed to obtain the final result.

## 9. Remaining known baseline failures

fsm_command_binding_test: existing missing-main/link failure, absent executable.
command_avoid_target_test: existing identical 111 assertions.
v2 ROS: existing identical 64 checks, chiefly the known AVOID fixture failures.

## 10. New regressions and equivalence

No genuine semantic regression found; all final gates preserve the baseline.
Acceptance: 200,000 transitions + 1,008 combinations PASS.
Request transaction: 200,000 serialized outputs/ID comparisons PASS.
Maneuver transitions: 200,000 + 560 combinations PASS.
Original FSM: 200,000 cycles PASS.
New internal direct-call harness: 200,000 cycles PASS, including wire-vs-direct
input bytes, requested/accepted IDs, lifecycle, locks and terminal state.

Synthetic ROS scheduling test (no MORAI): Behavior 9.997 Hz; feedback and
trajectory 19.998 Hz. Measured window: 40 commands, 80 feedbacks, 80 valid
trajectories. The command ROS input was disconnected in a temporary test-only
config; internal intent was still accepted and planned. There was no internal
feedback ROS subscriber. Each production output had exactly one publisher.
Both standard legacy executables rejected startup while the candidate ran.
Result: /tmp/phase8_scheduling_result.json (container).

## 11. git diff --check

PASS after restoration, integration and final documentation.

## 12. git status

Uncommitted planner refactor remains on fix/yuntae-planner-refactor. Migrated
legacy .cpp implementations were replaced by the authoritative asmc_planner
sources; compatibility headers/packages remain. No staging, commit or push.
All 18 protected preexisting untracked files retain their original SHA-256.
No unrelated source, legacy test, legacy configuration or message was changed.
Status record: /tmp/phase8_single_git_status.txt (host).

## 13. Remaining cleanup

Legacy deletion is deferred. Retain compatibility/reference packages, their
schemas and fixtures. Namespace aliases and ROS-generated compatibility value
types also remain. Candidate resource data is copied from unchanged references;
only package lookup paths changed. Do not launch the old planner pair with the
candidate. The inert Phase 1 scaffold remains available separately.

## 14. GO / HOLD for MORAI validation

GO for controlled MORAI validation of the candidate single-package path.
MORAI has not been run. This is validation readiness, not a claim of completed
MORAI driving verification. No commit or push performed.

### Explicit requested architecture answers

- asmc_planner alone contains the integrated production path: YES.
- Runtime dependency between behavior_planner and frenet_planner: NONE.
- Internal Behavior -> Execution -> Frenet uses structs/direct calls: YES;
  existing generated ROS value structs remain for serialization compatibility.
- Legacy planner packages are compatibility/reference only: YES.
- MORAI validation: GO; not executed.
