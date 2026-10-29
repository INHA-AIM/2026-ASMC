# GPS anchor + existing map planner experiment

This directory is the opt-in localization route. The independent LOCAL
planner remains in `src/local`; neither route is made the production default.

```text
Competition CAN + IMU quaternion -> local/node (odometry_only, one integrator)
                              |
GPS + IMU + odometry -> position_node -> anchor -> fixed map reference
LiDAR tracks_local -> aligned scan OBB cache ---^          |
                                                  LocalTrajectory
                                                        |
                                             existing LOCAL MPC
                                                        |
                                                ControlSelector
```

`anchor.hpp` owns the map-to-relative-pose transform, not another integrator.
`input_alignment.hpp` pairs source-stamped poses with raw IMU attitude using
`sampled_motion.hpp`; it performs no additional integration. `input_epoch.hpp` guards
LiDAR producer/generation/configuration identity. `position_reference.cpp` reads
the existing Frenet reference and map containment; `static_planner.cpp` reuses
native AVOID and shared local footprint/time parameterization for the later M3.
`position_node.cpp` is the ROS boundary and research trace owner.

No estimated pose is fed back into `/gps`, LiDAR map validity, or camera inference.
Tunnel wall/ceiling preprocessing therefore continues to use real GPS validity.
The planner caches each accepted local scan once, samples one reference, computes
one speed profile, and runs the selected MPC. A bounded handoff window also prepares the incoming MPC,
seeded from the last applied steering and measured acceleration; coordinate-dependent
solver state is reset. Only the selector publishes the final command. Native MORAI actor poses are evaluation only.

| Milestone | Evidence and remaining work |
|---|---|
| M0: architecture and isolation | Separate branch/worktree; no default launch change; common physical simulator lease required. |
| M1: port and baseline | Historical r01–r06 frozen at the pre-rebase lineage; 5/10 km/h trials and ideal static-core tests. These do not accept the new source. |
| R0: main integration | Pinned main `7bb4d9a1` integrated with all 114 research commits preserved. Pre-rebase `06a6c260`, rebased `b8f4c618`; backup bundle and one-to-one commit map: workspace `.storage/manifests/localization-integration-20261029-r1`. Later changes in other worktrees are separate. |
| R1: new runtime | Fresh build/devel, GPU runtime `asmc-shadow-localization`; `/root/ws` resolves to `2026-ASMC-shadow-localization`. Interactive master 11311; automated master 11371; offline CPU 2 / 6GiB builder. |
| M2: empty-scene tracking | Nine measured equal-cap viewport cases pass, including whole-timeline re-audit. Longer GPS-valid starts fail the strict steady-speed gate; faster NORMAL entry remains unaccepted. The new approach cap passes isolated ROS tests; new physical trials are refused by unresolved native contact -100. |
| M3: static bypass and return | Scoped 1845-start exploration: three centered physical passes at C++ a8d185a7, two with short stops. New sequential-target candidate passes bounded model A/B and 17 scoped suites; physical left/right/staggered acceptance is pending. E536 contains actual results; E537 contains the corrected model and withdrawn earlier model claims. |
| M4: comparison | Compare with independent LOCAL route on the same fixtures and failure accounting; select one for intensive tuning. |
| M5: dynamic actors | Validate timing, motion uncertainty, and map projection; never infer static from missing speed. |
| M6: deployment | Reproducible source/model/config/runtime bundle, rollback, review and explicit promotion. |

Experiment results live in registered D runs. Existing frozen runs and gallery IDs
must not be overwritten. The offline container has no external network; it cannot
control the shared simulator. A live environment needs renewed producer and lease
checks, plus actual checkout-owned perception source/model/config and ROS MD5 evidence.

M2 operating policy: warm up for at least 1s and 10 aligned valid fixes; departure
requires a fresh valid plan and GPS anchor (10s deadline). After GPS loss, validate
recovery for 1s against DR within 0.25m / 2deg, then blend for 1s. A larger jump,
producer/configuration change or time epoch reset invalidates the path. Reanchor
only after measured speed is at most 0.1m/s and the same warmup has completed.
Raw body OBB corners receive aligned IMU roll/pitch once per accepted scan. The
relative odometry's yaw-only orientation is not used as a tilt measurement.

`config/shadow_localization.yaml` is the experimental acceleration/jerk ramp. It is
loaded only by the position launch/runner; no production pedal calibration or
independent LOCAL tuning is changed. Its new ramp is a candidate until live gates
pass. `tools/scen_runner/test_localization_ros.py` uses synthetic inputs on an
isolated master; `--integrated` additionally tests the two controllers and sole
selector. It never launches a bridge. The real GPS is 5Hz, so the ten-fix initial
warmup normally takes at least 1.8s; its buffer retains enough fixes to meet both
the time and count requirements. Recovery validation remains one second.
The validated target is interpolated continuously between GPS fixes, with progress
frozen once the existing 300ms GPS freshness window expires. Historical transforms
keep their original target; later recoveries cannot change past poses. Acceptance
requires an uninterrupted GPS→DR→verification→blend→GPS sequence in the same
motion/anchor epoch. A stopped reanchor is recorded as a failure of that sequence.

The full pipeline can select this route with `jamming_planner:=localization` in
`autonomy_no_bridges_v2.launch`. Its default `main` still selects the existing
camera BBox route. Localization selects only its motion, position and MPC nodes;
the old BBox nodes are absent. NORMAL keeps the anchor warm; near the shadow region the next reference and MPC
are prepared. JAMMING suspends NORMAL MPC except during a bounded return handoff.
The main planner remains available to supply fresh plans for the return handoff.
Only `asmc_control_selector` publishes the final command. A validated fresh GPS
anchor and two fresh normal plans are both required to return to NORMAL; the
main route's four-second deadline cannot bypass this experimental anchor gate.

The default is the M2 reference follower with OBB/map stopping checks.
`shadow_static_avoidance:=true` enables the explicit M3 preview. Its native
candidate search and final sweep now share the conservative exterior envelope.
Rest requires four distinct scans spanning at least 0.5s. In odometry coordinates,
the union of their observed XY hull bounds may exceed the largest individual
observed extent by at most 15cm in Euclidean distance. This accepts partial faces
of a fixed volume without requiring a fitted centre or axis ordering to stay
fixed; uniform translation remains unconfirmed. It changes the internal rest
evidence only, never the original velocity flag or observed collision geometry.
Known dynamics, people and dogs remain outside static acceptance.
When native PREPARE lacks a moving path, the fixed-reference approach uses a
speed justified by verified full-body map/OBB clearance and stopping space.
It does not force an early 3km/h reduction that can invalidate a paint-time-bounded bypass.
Committed-path failures and unsupported dynamics retain stops. This preview is
pending closed-loop obstacle acceptance and does not establish dynamic acceptance.
Shared-container setup and commands are in
[planning/control operations](../../../../docs/operations/planning-control.md).

[Rebase and M2 checkpoint](../../../../docs/records/planning/2026-10-29-aimdrive-m2.md)
records the source boundary, new analysis, and current live acceptance status.

Runtime naming migration: operational `shadow_map` source/launch and the old
external-repository-named option are replaced by `shadow_localization` and
`jamming_planner:=localization`. Old options are rejected. Frozen experiment IDs,
gallery numbers, commit SHAs and external implementation attribution are immutable.
The original external source informed the IMU/CAN integration approach; it is not
the current runtime's name.

Automatic tests use `run_localization_trial.py` on a private master. The runner
acquires the common physical lease, records native truth for evaluation only,
starts its own raw bridge, and restores the initial ego/control/environment.
`--gps-outage` is a private ROS fault fixture: original GPS coordinates select
the outage interval; packets become explicit no-fix there. Simulator sensor files
and shared ROS inputs are unchanged. Natural GPS reception and injected outages
are reported separately; zone-driven mode switching alone is not DR acceptance.

Current localization motion measures relative heading from the raw valid IMU
quaternion, unwrapped inside each motion epoch. Gyro-rate integration remains
the independent LOCAL default and an explicit A/B choice. The measured heading
prevents rate bias accumulating into lateral drift; it does not introduce GPS
or native actor truth into the relative pose. Heading jumps invalidate the epoch.
Controller handoff seeding happens after input/frame/reference resets, immediately
before solving. A real MPC regression covers both the first frame and a frame
restart in the same handoff epoch.

The next measured Molit candidate uses `velocity_turn_gain_m2ps=1.54` divided by
mean adjacent speed (floor 1m/s) for the incremental heading term in the same
integrator. It is an empirical turn correction, not a variable physical lever
arm. Acceleration at constant heading creates no extra translation. The previous
constant 1.4m model remains available with gain zero; generic LOCAL defaults to
gain/offset zero. Pre-loss GNSS/IMU/allowed-speed fits at 5/10km/h selected the gain;
15km/h pre-loss and recorded recovery are separate diagnostic checks. Failure
records informed model selection; the nine fresh viewport-cohort cases validate
that measured configuration, without establishing obstacle or other-version acceptance.
The calibration JSON retains both models and input SHAs. No timing, speed-scale,
fixed-heading, or safety-threshold adjustment accompanies this candidate.

Current measured static checkpoint (MolitComp03, 15km/h cap): C++ candidate
`a8d185a7` completed three centered WoodBox bypass/return trials with native
minimum clearances 0.830/0.849/0.993m. Two trials include 1.569/0.917s complete
stops; this is bounded M3 completion, not uninterrupted driving. An inherited
native contact `-100` aborted the following start. The runner now requires a
fresh half-second clear contact window before departure and preserves unknown
contacts. Left/right/staggered physical cases remain pending.

Full-timeline evaluator schema 2 latches the change-only mode over every native
pose and pairs only an already received decision no older than 150ms. A separate
append-only audit of 76 existing trials retains 58 accepted and revokes none;
the latest three static passes have 98.70–100% valid moving decisions, not a
claim that every decision was valid. Gallery E536 shows failures and stops too:
`localization-integration-20261029-r01/static-review-r1/index.html`.

Localization now prepares on the curved upstream global route with braking
distance for the configured shadow cap. The episode retains its preparation
distance while slowing, preventing its own speed cap from oscillating. NORMAL
MPC uses the existing acceleration/jerk limits and releases that additional cap
outside a valid approach. Invalid/stale/wrong-frame states cannot lift it. Main
defaults to zero for this extra cap. Numeric and real isolated ROS tests pass;
physical NORMAL-speed entry remains unverified. Runner `--normal-cap 30` tests a
separate NORMAL cap, `--normal-cap 0` preserves the production policy, and the
default retains the equal-cap historical M2 condition. Native mode-specific
guards and evaluation cannot mask shadow overspeed with the NORMAL ceiling.

The ROS-free `localization_static_scenarios` diagnostic covers center,
left/right offset and staggered boxes at a 15km/h requested cap in a bounded
kinematic bicycle rollout. Staggered boxes exposed a forced return toward a second
obstacle. The static-only wrapper may now replace its target in PASS/RETURN
only after the whole current observed target hull is behind the rear bumper
plus 1m and the local margin, and a different positively stationary target is
ahead. Missing old observations cannot retire a target. PREPARE starts from
the current measured state, retaining every old/new OBB and all road/paint/
collision checks; no path is relabelled and at most one native search runs.
An initial ideal point-replacement rollout was invalid because it could jump
to a remote reference at a target change; its completion claims are withdrawn.
The corrected rollout integrates velocity and bounded steering instead of
copying a path point into vehicle pose. Invalid updates apply jerk-bounded
braking with held steering and remain counted. All four synthetic cases now
complete with >0.674m sampled body clearance, no complete stop and one native
call per update; the staggered case has one invalid update (99.706% valid).
The same binary/plant with target replacement disabled stops for 3.05s in the
staggered case; single-box results are unchanged. A remote fixed reference
cannot replace PREPARE maintenance unless its first point is within 0.35m,
matching the existing native path join bound. These are synthetic geometry
checks, not physical 15km/h acceptance or an MPC/IMU performance result. The real
simulator currently reports an unresolved native `-100` contact before departure;
the clean-start gate blocks further physical acceptance without ignoring it.

## Automatic experiment monitor

Keep `http://localhost:8089/web/index_v2.html` open. The automatic trial runner
enables this observer by default; Auto selects the active experiment, or choose
**Localization map 후보·선택**. `localization_observer.py` runs a persistent
read-only dashboard on the trial's explicitly selected private master. The
autonomy child intentionally retains `dashboard:=false` to avoid duplicate HTTP
servers. Manual interactive Bash still uses shared 11311. The observer checks
master URI/PID, ROS clock mode, worktree, branch/revision and backend/helper SHA;
an occupied mismatched port is left untouched.

Campaign weather is now user-pinned to **Foggy, 13:00**. The runner checks native
readback and records requested/applied conditions. Cleanup retains Foggy instead
of restoring a former Cloudy snapshot; original snapshots remain protected as
evidence. Earlier Sunny trials are a separate condition and do not qualify Foggy
performance. Raw recording retains every topic with a bounded 1GiB buffer,
4MiB chunks and lossless LZ4; prior 64MiB overflow trials are kept as failed
recording evidence, never relabelled complete.

The page shows run/trial, source WIP status, lifecycle heartbeat, anchor readiness,
GPS/DR/recovery, current rejection reasons and available handoff diagnostics.
Blue/red curves are feasible/rejected native candidates, cyan is Localization's
prepared input, and gold follows the available selector diagnostic to the
current controller's fresh input. A replay is explicitly labelled as geometry
review without actuation. Selector diagnostics are sampled, not a hard real-time
assertion of control receipt. Per-candidate rejection explanations and native
ground truth are not supplied by this channel. Native map paths retain map
coordinates; observed local OBBs receive the existing aligned transform once.

Geometry is bounded to 5Hz, 24 representative candidates × 64 points, at most
1024 selected points and 512 observed OBBs. Trial rollover, terminal/restoring
states, source/receipt expiry and retired motion epochs clear current geometry.
Candidate encoding time/bytes are separate from planner `processing_ms` in the
research trace. Monitoring settings, source patch, debug and experiment heartbeat
are recorded with the trial, alongside all four raw camera streams, LiDAR,
GPS/IMU/CAN. HTTP/browser overhead is not a qualified control latency benchmark.
Raw bag finalization completes before releasing the physical lease. Replays on
an existing private master restore their clock, speed-caller and trial parameters;
physical launches explicitly bind the speed caller to `/morai_udp_bridge`.

Registered run `localization-obstacles-20261029-r01` contains browser evidence:
`observer-bag-15-r02` traverses GPS/DR/recovery with up to 117 generated and 13
feasible candidates; only 24 are drawn. `observer-physical-empty-5-r06` moved 40m,
showed fresh GPS→DR geometry on late browser connection/reload, then cleared all
vehicle/path geometry on completion. Its inherited `-100` contact remains in raw
records and `driving_acceptance_eligible=false`: this is monitor acceptance,
not collision-free driving acceptance. Standard driving trials retain the clean
contact gate. The optional observer-only contact diagnostic is restricted to an
empty corridor, ≤5km/h, ≤50m and ≤60s; it cannot qualify an obstacle campaign.


## Observed-motion M3 candidate (2026-10-29)

The experimental `shadow_dynamic_obstacles` and `shadow_lane_rules` launch
arguments both default to false, independently of the monitor. The automatic
runner records their exact values with `--dynamic-obstacles` and
`--shadow-lane-rules`; both require the explicit static M3 preview. Dynamic
acceptance and promotion remain pending.

A private planner copy estimates velocity from distinct, ego-compensated hull
observations, not native truth or missing absolute velocity. At least four
samples spanning 0.3s, stable extent, bounded fit residual and a finite speed
are required. Pending motion keeps the checked stopping prefix. Original
observations and source times remain intact; prediction advances once to the
ego timestamp. The final time-parametrized trajectory receives the same
map/body and moving-OBB checks before COMMIT. Native speed bounds survive shared
retiming. A late dynamic conflict can use one checked terminal-braking witness;
there is at most one native search per update, with additional speed-profile
work explicitly counted. Invalid geometry never becomes a crawl permission.

Only a selected, valid pose inside the existing configured shadow zone may
exempt paint, with a three-second exit tail. Physical road containment is
always hard. Slow, positively observed dynamic targets may then use the road
bypass candidates while retaining their dynamic predictions. Main and ordinary
LOCAL policy remain unchanged. This tail is a permission timer, not evidence
that the whole vehicle has returned home within the competition deadline;
that physical exit metric still needs a qualified simulator trial.

`localization_static_scenarios 15 --dynamic-cases --calibrated-plant` exercises
four static and nine dynamic/mixed fixtures in a bounded synthetic plant.
`--rotated-anchor` checks a nonidentity map/odometry transform. The 0.844 steering
gain and 0.15s delay do not turn this pursuit model into the real MPC or MORAI.
`--reduced-growth` is an offline sensitivity comparison (0.1 to 0.05m/s
prediction-margin growth); no live YAML uses that reduction. Raw decision logs,
failed fixtures, controller-independent body checks and source patches are
preserved in registered run `localization-obstacles-20261029-r01`. Speed is not
promoted above 15km/h while these gates remain incomplete.

Current bounded-model checkpoint: 11/13 fixture gates pass with the unchanged
0.1m/s growth setting; slow-leader overtaking and adjacent oncoming validity
remain incomplete. This is not a canonical blackout-driving acceptance. User
restarted MolitComp03 and reloaded sensor/network; read-only native checks then
showed only self ID `Ego`, with the old `-100` gone. Physical checks can resume
under the unchanged clean-start gate. C++ core/LOCAL/road regressions, input and
five LiDAR tunnel/ground suites, seven isolated ROS recovery/handoff/selector
suites, launch graphs and dashboard contracts pass at this checkpoint.


Standstill boundary follow-up: measured CAN can briefly report -0.004m/s while
stopping. Localization already allowed reverse values down to -0.05m/s, but
feeding them unchanged into a forward native profile could create a negative
trajectory sample and invalidate the plan. The planning-only boundary now uses
zero inside that existing allowance. Signed odometry, raw trace and the reverse
rejection limit remain unchanged. Pure input/three core suites and the real
isolated ROS trajectory contract pass; this does not by itself resolve the
right/staggered physical failures. Native target-loss, road, collision and
command failures now have distinct candidate diagnostic reasons.


Motion-evidence comparison candidate: a producer DYNAMIC bit cannot alone veto
stable fixed-volume support. The wrapper still needs >=0.5s of distinct hulls,
bounded union displacement, and (when the bit disagrees) a stable fitted speed
<=0.1m/s. Translating hulls never pass this test, and original source estimates
are not mutated or promoted to measured zero. The disagreement count is retained
in trace/debug. Person/dog motion remains in the explicit dynamic preview. This
candidate targets the recorded fixed-box KF motion chatter and requires new
Foggy scene validation before promotion.
