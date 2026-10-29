# Localization integration checkpoint — 2026-10-29

> 아래는 순차 개선의 과거 기록이다. 최신 실행·선정·main 통합은
> [비교 체크포인트](2026-10-29-localization-comparison.md)를 따른다.

Main `7bb4d9a19f7ea278b496655d75b772fb30e3af17` is pinned. All 114 research
commits from `06a6c260` were preserved by `--rebase-merges`, producing `b8f4c618`
before this integration. The archive ref is
`archive/shadow-localization-pre-rebase-20261029-06a6c260`. Workspace-private
`.storage/manifests/localization-integration-20261029-r1/` retains the verified
bundle, one-to-one commit map, range-diff and conflict rationale. Neither the
primary checkout's E3 WIP nor independent LOCAL work was cherry-picked.

Operational names are `research/shadow-localization`,
`2026-ASMC-shadow-localization`, `asmc-shadow-localization`,
`asmc-shadow-localization-build`, `shadow_planner:=localization`,
`shadow_localization.launch`, `/planning/localization/*`. Historical source
attribution, frozen run IDs, gallery numbers and original SHAs remain intact.
The worktree was moved with Git repair; its nested MORAI message checkout keeps
its original revision. Current build/devel are fresh and separate from every
other checkout. The old runtime is retained as `*-previous`, never used as the
new executable baseline.

The [runtime profile](../../../tools/scen_runner/config/localization_runtime.profile.json)
and [manual commands](../../operations/planning-control.md) describe dual shell
profiles. Fresh interactive Bash uses shared 11311 without a manual source step.
Noninteractive automated execution explicitly sources its isolated profile.
Only roscore/raw bridge are shared in manual operation. Camera E3, lane, LiDAR
fusion, planning and control execute from the selected worktree. GPU, 2GiB shm,
engine compatibility and checkout paths were inspected in the new container.
Latest main LiDAR code is unchanged; E3 selection and highway default are kept.

Smooth handoff is opt-in with Localization. It warms the next reference/MPC
near the region, validates epoch, producer, state/path timestamps and command,
then commits the selection. Incoming MPC seeds physical steering/acceleration,
not coordinate-dependent solver state. Default main selector/GPS policies are
unchanged. Preparation timeout, invalid input stops and 250ms watchdog remain.

Fresh Release workspace build passed. Regression evidence in registered D run
`localization-integration-20261029-r01` includes 42 input-boundary checks,
14 ideal static-core checks, default GPS/selector/recovery ROS tests, the new
handoff ROS test (old epoch, stale candidate, delayed candidate, return and
watchdog), nine selected main Frenet/startup/highway tests and five LiDAR
tunnel/ground tests. Launch graph and assessment/departure policy tests passed
inside the ROS container. Host-only attempts lacked ROS/pyproj and are not
reported as passed verification.

The first measured empty-scene 5km/h integrated trial (`empty-5-r03`) completed
NORMAL→JAMMING→NORMAL, with no contact or restore error. Neither handoff inserted
braking; active command gap was below 60ms. Every native pose was independently
projected against the native fixed Frenet reference and vehicle map envelope.
Its assessment passes every gate except GPS/DR/recovery: GPS continued to arrive,
so region-driven switching alone does not establish dead-reckoning acceptance.
This trial used the recorded integration WIP, not a released immutable source.

Private GPS-fault trials exposed a real simultaneous IMU/Ego input gap of about
0.54s and subsequent restart overspeed. Preserve the stops and failure records;
these runs are not acceptance. Shared primary bridge/control were observed
running later; process start evidence places them after those failed trials,
so the failures cannot be attributed to that later shared execution. Further
physical tests pause while that execution owns the simulator. Automatic session
preflight and runtime checks now reject live foreign bridge/control owners even
if an advisory physical lease is free. Other nodes are never killed to obtain
access. Recorded-input replay continues on a separate master without a bridge.

M3 work remains gated behind M2: authoritative LiDAR corners receive aligned
IMU tilt once; static evidence requires distinct ego-compensated observations.
Reference, static bypass and return share a strict map/OBB swept-footprint check.
Real semantic raster constraints are preserved when supplied; missing rasters
are not manufactured free space. The unneeded KEEP lattice is avoided.
The 15km/h ideal symmetric and asymmetric-road bypass/return tests now pass
with the same conservative 2.160835m exterior in native candidate generation
and final verification. The former mismatch selected a road-edge candidate
that final validation correctly rejected. No physical envelope is shrunk and
normal main keeps its original YAML. Native search already evaluates both sides,
so no duplicate side-search loop is needed. Source/anchor epochs clear static
history; trace reports actual native-search and timing calls. The static path
remains disabled by default pending closed-loop acceptance. No blanket
clearance reduction, blind creep, dynamic acceptance or higher-speed promotion
has been made from the single 5km/h result.

The 5km/h recorded replay completed with 1,509 DR decisions, 19 recovery
verification and 21 recovery blend decisions; no control publisher existed.
This is open-loop evidence, not replacement for repeated physical tests.

Next gates: repeat closed-loop empty-scene
entry/exit at 5/10/15km/h with exclusive physical ownership; validated native
static alternatives and return; then owned obstacle fixtures and measured margin
comparisons. Preserve native collision/map/unknown-input boundaries throughout.

## Recovery and repeated-trial follow-up

The simulator restart restored the existing MolitComp03 gRPC service; no R1
switch or shared-container restart was needed. Physical trials start their own
camera/LiDAR only after acquiring both aliases of the shared lease and release
those processes before releasing ownership. The active workspace run registry
is checked before every append-only trial. A busy lease waits without changing
the scene or starting GPU inference.

`empty-5-outage-r06` and `orientation-5-outage-r02` reached the end without
contact but failed recovery: a first raw GPS innovation just above 25cm caused
the correct stop under the old immediate gate. IMU orientation alone did not
fix this. A GPS/odometry rigid-heading fit was also rejected because it made
the recorded recovery innovation worse. Neither result is promoted.

Localization now removes the full canonical GPS mount `(0.35,0,1.25)` using
source-aligned IMU attitude. Recovery retains the DR transform while collecting
the specified one-second candidate; only a candidate within 25cm/2deg can start
the one-second blend. Gross raw position jumps of at least 50cm, heading jumps,
unstable validation and invalid inputs stop. No recovered sample moves the
anchor before validation. Shared Anchor consumers keep their former policy
unless this research option is enabled. Raw and candidate innovations are
recorded separately. This policy is exercised by numeric and real ROS tests.

The first complete acceptance `window-cap-5-outage-r01`, source `3cc90d02`,
passes all M2 gates: lateral p95 0.2371m, maximum 0.4647m, actual speed maximum
5.2353km/h and active command gap maximum 67.7ms. Both handoffs have brake zero
and retain moving speed; GPS/DR/verification/blend/reconnected states are all
observed. `window-cap-5-outage-r02` also passes. The first accepted result and
preserved failures are [gallery E457](http://localhost:8000/localization-integration-20261029-r01/recovery-review-r1/index.html).
Its innovation plot holds the latest raw value through the GPS outage; it does
not indicate that new GPS fixes arrived during that interval.

The third repeat stopped inside the blackout. Exact bag inspection found no
IMU/Ego forward gap over 150ms: a **0.637ms reordered speed header** triggered
the old reset-on-any-regression callback and discarded the anchor. This differs
from the separately preserved 0.54s input-gap failures. The bounded reorder
fix discards up to 100ms old input without consuming it or refreshing freshness;
larger regressions and actual gaps still invalidate the epoch. Default LOCAL
tolerance remains zero. A real callback regression covers the observed reorder,
large clock regression and forward gap. Replaying the same failed bag keeps one
motion epoch and removes all 248 former `motion_epoch_changed` decisions. The
recorded vehicle had already stopped, so this replay is not closed-loop or GPS
recovery acceptance. New repeated trials run from `cbf8c679` before escalation.

The restart-speed failure is preserved separately as `window-5-outage-r01`.
Localization uses the existing solver acceleration-ceiling API with measured
speed/acceleration; a real MPC test verifies that an accelerating reference
cannot publish throttle above the configured cap. Default main/LOCAL calls
retain the original result. No global pedal, jerk, margin or watchdog change
was used to obtain the first acceptance.

M3 evaluation records native states of owned fixtures strictly for assessment.
Intentional bypass lateral displacement is reported rather than judged by the
M2 centerline tracking gate. Static acceptance instead requires physical map
containment, native clearance, PASS/RETURN evidence and measured final return,
alongside the existing contact, control, input and recovery checks. Static
preview and higher speed remain unaccepted until those physical trials pass.

## Measured reference-point model and continuous recovery

The allowed forward speed and measured IMU heading describe different rigid
reference points during turning. Three **pre-loss GNSS/IMU/speed** fits yielded
1.394, 1.509 and 1.389m; rounded median 1.4m is the Localization-only reference
offset. One exact heading-dependent translation is added inside the existing
integrator, including its short extrapolation. Independent LOCAL retains zero.
The input SHAs and limited MolitComp03 scope are in
`config/localization_motion_calibration.json`. The earlier inspected failure used
as a parameter holdout is not a blind model-selection test. New physical runs
provide prospective evidence. Neither native actor truth nor an additional
integrator supplies the estimate. 26.R1 remains unverified.

Prospective point-model 5km/h runs reduced lateral p95 to about 9cm. An unrelated
NORMAL startup overspeed is preserved: it occurred before JAMMING and before
Localization selected control. The agent's same-speed comparison fixture now
sets the optional NORMAL solver acceleration ceiling. Its default is disabled;
manual main and normal-road production speed policy are unchanged. Each trial
records the private fixture cap explicitly. These tests do not establish an
unmeasured 30→15km/h manual handoff.

The `blend-point` cohort passed 5km/h three times, 10km/h three times and its
first 15km/h run. The second 15km/h run failed recovery despite passing contact,
map containment and tracking gates. A valid 24.36cm candidate started blending;
the next 5Hz GPS fix was compared with the previous keyframe instead of the
already-progressing transform. `cd4f97f5` interpolates that validated target at
the observation time and at intervening path times. Correction freezes after
300ms without a fresh fix; it cannot read future GPS or rewrite older targets.
The 25cm/2deg candidate gate, 50cm gross-jump stop, motion epoch handling and
250ms control watchdog are unchanged. Four additional numeric cases cover
interpolation, the next-fix gate, loss freezing and immutable history.

The recorded old failed drive contains its original emergency stop and motion
reset. Its replay removes the former early blend rejection but **cannot accept
continuous recovery** from that stopped motion. `7bd64e3a` strengthens both replay
and physical assessment to require an uninterrupted, adequately sampled sequence
in one motion/anchor epoch. The original replay summary is preserved with a
separate continuity correction; earlier seven successful `blend-point` runs also
pass the stronger continuity check. No failed result was relabelled successful.

Fresh continuous-blend verification passes 67 numeric input checks, all three
LOCAL/Localization core suites, 18 real ROS boundary checks, ten integrated
controller/selector checks, launch graph and assessment tests. The first fresh
5km/h physical run passes all gates: lateral p95 8.72cm, maximum 16.30cm,
maximum command gap 68.3ms; both handoffs retain motion. The remaining fresh
5/10/15km/h repeats and offset cases are still open at this checkpoint.
Physical obstacle bypass/return and any default promotion remain gated on them.

## Speed-dependent turn candidate (not yet physical acceptance)

The continuous-blend cohort passed all three 5km/h and all three 10km/h runs.
The supplementary full-timeline dwell audit finds zero mid-run stopped time in
all six. Its first 15km/h run fails **before blending**: the full candidate is
25.35cm. It is retained as a failed run; the 25cm gate is unchanged. GPS delay,
speed interpolation and road-pitch counterfactuals give insufficient improvement
and were not applied. A fixed yaw/scale fit is not promoted either.

Pre-loss GNSS/IMU/allowed-speed data instead identify a repeatable turn residual.
A one-parameter inverse-speed incremental term fits 5km/h gains 1.519/1.560 and
10km/h gain 1.543m²/s. Joint fit 1.53934 is rounded to 1.54, with speed floor
1m/s. Neither recovery samples nor native actors choose this parameter.
Separate 15km/h pre-loss residual RMS falls from about 12.6cm to 6.6cm. Recorded
reconstruction reduces the latest failed recovery candidate from about 26cm to
11cm; this is a diagnostic reconstruction, not a new successful closed loop.

This replaces the incremental heading coefficient with gain/mean-speed, within
the existing single integrator. It does not add a derivative-of-gain translation,
change measured control speed, change IMU pose, or claim a moving physical GPS
mount. Numerical tests cover the circular path, finite standstill limit,
acceleration without heading change, and matching short extrapolation. Generic
LOCAL keeps gain zero; gain zero plus constant offset 1.4 reproduces the old model.
The calibration JSON preserves the former model and all fit input SHAs. Model
selection used inspected failures, so fresh physical repeats must establish
acceptance before static trials or speed/default promotion.

The new turn model subsequently passes all eleven `turn-model` M2 cases:
5/10/15km/h three times each and ±0.2m/±2deg starts. 15km/h lateral p95 is
8.47/8.61/8.62cm and no mid-run stop occurs. The first longer GPS-valid start
(station 1815) completes without contact, map departure, stop or recovery fault,
but measured speed error p95 is 0.553km/h against the unchanged 0.5km/h gate.
The desired shadow reference remains 15km/h while measured speed sometimes
stays around 14.3–14.5. This is not reported as complete acceptance for arbitrary
GPS starts. A Localization-only MPC speed-cost override of 24 (base 8) is the
next candidate; the acceleration ceiling, pedal mapping and physical limits are
unchanged. Shared main and independent LOCAL tuning remain their former defaults.
Runtime parameter snapshots accompany subsequent trials. Repetition with this
controller change remains required before M3.

The speed-cost-24 diagnostic starts at stations 1815 and 1835 both pass. The
1815 speed-error p95 improves from 0.553 to 0.449km/h without a mid-run stop.
The following repeat cohort passes 5km/h, 10km/h three times and its first
15km/h run, then stops after a second 15km/h failure. Its GPS/DR recovery itself
passes, but a **29.32cm single GPS innovation before the blackout** triggers the
old immediate tracking gate, causing a stopped reanchor. The failed run remains
failed despite later completion; mid-run dwell and speed gates expose it.

Localization now also applies the bounded candidate policy to a tracking
outlier while GPS remains available: retain the validated transform, verify for
one second, then blend only a <=25cm/2deg candidate. >=50cm raw or >2deg heading
jumps still stop immediately; persistent >25cm candidates still stop. Unverified
samples cannot refresh the 120s/250m budget: history snapshots retain the last
validated fix's time and travel distance. Generic immediate-gate consumers keep
their opt-out policy. The input suite has 76 checks; 22 real ROS checks cover
isolated tracking noise, persistent and gross jumps, stopped reanchor and the
existing LiDAR/time/OBB boundaries. The failed bag replay removes the original
GPS rejection but contains its recorded braking/epoch reset, so it is not a new
closed-loop acceptance. Fresh physical repetition is required for this change.

## Real producer gap and reversible viewport cohort

The tracking-window cohort passes its first 15km/h diagnostic, 5km/h baseline,
10km/h three times and a second 15km/h run. The third 15km/h run fails despite a
GPS recovery candidate of only 8.4cm: original IMU source time and kernel receive
time both have a 207ms gap. Publication adds about 2ms at that boundary. Motion
epoch invalidation and the ensuing stop are therefore retained; no bridge
threading, timestamp substitution, 150ms integration bound or 250ms watchdog
relaxation is justified by this recording. Native simulator poses also briefly
repeat. This is evidence of a simulator producer/update hitch, not proof of a
receiver-side bottleneck or an algorithmic recovery pass.

The main simulator viewport was 1202x1000. A leased experiment sets only its
main rendering viewport to 800x666 through the supported SDK, then restores
1202x1000 before releasing the lease. Sensor/network save files are unchanged.
Every new trial decodes one fresh image from each camera before departure and
checks the canonical 1280x720 front and 640x480 left/right/extra dimensions.
The supported rendering API is used only by the scenario fixture, never the
localization estimator or operational launch. A smaller viewport is a test
condition, not a claim that simulator input faults can no longer occur.

All nine fresh `viewport-800` M2 cases pass: 5km/h once, 10/15km/h three times,
and both 15km/h (+0.2m,+2deg)/(-0.2m,-2deg) starts. The three 15km/h lateral
p95 values are 8.567/8.557/8.557cm, speed-error p95 0.342/0.409/0.303km/h,
maximum command gaps 65.6/69.3/65.8ms and reference p95 about 2.2ms. Mid-run
stopped time is zero. Both handoffs, continuous same-epoch GPS/DR recovery,
physical map containment and receipt/bag correspondence pass. This cohort uses
NORMAL and shadow trial speed caps equal to each requested speed. Production
NORMAL-speed-to-shadow-speed approach remains a separate integration check.

The shared/manual shell check resolves all three packages to this checkout,
selects master 11311 without an extra source command, and matches shared raw
Ego/IMU/GPS message MD5s. Automatic trials use master 11371 and own all inference,
planner, control and temporary raw bridge processes under the physical lease.
Runtime snapshot v3 records actual GPU/mount/build/profile and model SHAs.
Longer GPS-valid starts are the next checks; no physical obstacle result or
manual/default promotion is implied by this checkpoint. All earlier failures
and gallery E504 remain unchanged.

## M3 first physical failures and targeted corrections

The longer 1815 start with the viewport profile completes without contact, stop,
map departure, handoff or recovery fault, but full-timeline speed-error p95 is
0.561km/h. A separate NORMAL-only A/B retains this failure honestly: main
0.603km/h versus localization 0.605km/h, both above 0.5km/h; lateral p95 is
6.29/7.55cm and command maximum gap 57.0/55.5ms. These single comparisons do not
establish broad equivalence or complete arbitrary-start acceptance. M3 therefore
uses the independently accepted nine-case 1845 corridor as an exploratory
boundary, while the longer-start NORMAL speed item stays open.

`static-center-5-r01` stops before the real WoodBox, with no contact and minimum
native OBB clearance 0.607m. It fails bypass, completion, recovery and dwell gates;
reference processing p95 is 84.9ms. The original observation's missing absolute
velocity flag was retained even after positive multi-scan rest classification,
so future collision margins continued growing. `ef2ce135` adds internal
`stationary_observed` evidence only after the existing distinct ego-compensated
history succeeds. Raw velocity availability and ROS messages remain unchanged.
An unknown/static label without that evidence still receives growing margins;
dynamic classification overrides it. The 0.45m native base margin is unchanged.
Localization limits its cooperative search slice to 25ms; other callers retain
their configured 80ms default. Trace adds rest/candidate counts and rejection
reason. Three core suites, twelve main regression suites and six ideal
5/10/15km/h road variants pass with real missing-velocity flags and nonzero
margin growth; no synthetic free raster is introduced into the real route.

Offline diagnostics distinguish generation, road and collision rejection.
An optional environment switch adds counterfactual road-versus-paint diagnosis
only during replay; it cannot authorize a path and is absent in the live
container. Recorded 5km/h input with requested 5km/h has no feasible native
candidate; 983 evaluated rejections are paint-only. With requested 15km/h the
same recorded input has feasible candidates and reaches SHIFT in replay.
This is open-loop feasibility, not a physical pass or a timing benchmark.
Paint deadlines, physical road containment and collision checks were not relaxed.

The first real 15km/h diagnostic reaches SHIFT but then a committed replan fails
before the obstacle. Full braking is followed by about -0.07m/s Competition
velocity, which resets the forward-only integrator repeatedly and loses the DR
anchor. This case has no >150ms raw IMU gap at that boundary: the largest local
packet gaps are about 80ms. It must not be conflated with the earlier 207ms
producer-gap failure. Recorded CAN velocity, kernel timing, motion epochs and
original failed bag are preserved in the registered run.

`12d405e5` tracks signed brake rollback up to 0.15m/s only in Localization;
generic LOCAL retains its 0.05m/s rejection and zero clamp. The existing planning
boundary still refuses a reverse trajectory below -0.05m/s, and larger reverse,
clock regression and genuine source gaps retain epoch invalidation. This keeps
centimetres of actual rollback in relative pose instead of inventing zero motion
or renewing a stale timestamp. PREPARE also removes its unconditional 3km/h
approach cap: the existing verified path/jerk/braking-distance profile determines
the permissible speed and includes a stop. New numeric checks cover signed
rollback, larger reverse rejection and unchanged generic behavior. Existing
LOCAL and input/collision/core suites pass; a fresh 15km/h physical repeat is
running. No obstacle success or launch-default promotion is claimed yet.

Gallery E505, `motion-review-r3/index.html`, contains the nine successful M2
cases, the failed longer start and the first physical 5km/h failure with a native
body/box plot. Its root registration preserves all 504 existing identifiers and
the shared HTML outside embedded catalogue JSON. An unpublished partial r2 build
is retained after its plot-variable failure. E504 and all original runs remain
unchanged. Subsequent 15km/h results require a new append-only checkpoint.

### Centered static checkpoint and full-timeline audit

`a8d185a7` replaces fitted-centre/size rest consistency with already leveled
observed-hull support in odometry, once per distinct scan. At least four scans
over 0.5s are required. Union excess beyond the largest observed extent must
remain within the former 0.15m displacement tolerance; translating unknown
objects and explicit dynamic/person/dog inputs are not declared stationary.
Authoritative collision geometry, missing-velocity flags, base margin and
LiDAR wall/ceiling/ground preprocessing remain unchanged. Partial-face and
translating-object core regressions pass.

At that C++ checkpoint, `static-center-15-diagnostic-r03` and
`support-static-center-15-r01/r02` completed physical bypass, return before exit
and continuous GPS recovery. Native OBB minimum clearances are respectively
0.8303/0.8488/0.9934m; longest stopped intervals are 1.5692/0.9166/0s, and maximum
control gaps 69.1/67.5/73.1ms. The first two are bounded-stop M3 passes, not
no-stop results. A preceding r11 failure stopped for 12s and remains a failure.
The next `support-static-center-15-r03` aborted at its first native sample with
contact `-100`, already present in the original world snapshot. It does not
count as a successful repeat or an observed new obstacle impact. `1de19dbd`
adds a fresh clear-contact startup gate, unit tested; it never clears or ignores
unknown IDs. Left/right/staggered physical tests have not run.

An evaluation defect was found: change-only `/planning/mode` was paired within
150ms, restricting the old coverage ratio to transition neighborhoods. Commit
`2c9b5036` carries mode forward at every native sample and pairs decisions
causally within the unchanged 150ms bound. Existing native contact, stop,
tracking, speed and command-gap metrics already used the full timeline. The
append-only `full-mode-timeline-audit-r1.json` recomputes 76 trials, retains all
58 former passes and revokes none; original files are unchanged. Latest three
static passes have valid-moving ratios 98.81/98.70/100%, pairing 100%; invalid
and stopped intervals remain counted. Gallery E536 includes the corrected
coverage, all three physical static passes and preserved failure/startup cases:
http://localhost:8000/localization-integration-20261029-r01/static-review-r1/index.html
Registration preserved the other 535 identifiers, existing catalogue rows and
shared HTML outside its JSON. HTTP and an actual body/obstacle plot were checked.

### Curved NORMAL approach before the shadow boundary

An opt-in approach cap prepares from the upstream route, reserving deceleration
distance at 1.5m/s² plus response distance. Its maximum episode window is retained
as speed decreases so the cap cannot release itself before entry. Main uses
zero for both new parameters; only Localization forwards its shadow speed cap.
The existing NORMAL MPC applies the minimum of any validation cap and active
approach cap through its current acceleration/jerk/steering limits. No second
integrator, inference pipeline or path lattice is introduced. Stale, non-map or
nonfinite states cannot release a conservative active approach. Trace reports
the effective cap and preparation receipt age.

Fresh build `normal-approach-build-r1.log`, 11 CTest suites (including real ROS
approach, handoff, restart seed, selector, GPS recovery, generic LOCAL and
Localization cores), 15 assessment tests and four startup-policy tests pass.
Full launch graphs keep main's additional cap zero and match Localization's
requested cap. New interactive Bash resolves this worktree's camera/LiDAR/
planner with shared master 11311 without extra source commands. Automated ROS
tests use isolated masters; current common bridge/other worktree nodes are not
stopped. At the next ownership check, private master 11348 had live raw bridge,
mode and control publishers, so physical work stayed idle.

Runner `--normal-cap` explicitly separates NORMAL and shadow trial conditions.
Omitting it preserves the historic equal-cap M2 test; `0` uses the checked-out
production ceiling. Unknown mode stops a trial; a faster NORMAL ceiling never
permits shadow overspeed. Physical mixed-speed acceptance remains pending.
Launch defaults remain shadow 5km/h and static avoidance disabled until a
validated operating condition is selected. No source changes in main's
LiDAR package or simulator presets accompany this checkpoint.

### Native contact diagnostic and staggered obstacle continuation

Once master 11348 released physical ownership, the new build attempted
`approach-build-m2-15-r01`. Its startup gate observed 99 native queries over
10.046s with `-100`, stopped before inference/control departure, and restored
the world and viewport without cleanup errors. No native motion samples or
new bag were created, so assessment correctly cannot run; the batch recorded
that error and did not proceed to different fixtures. The earlier clean-contact
gate is now exercised in a real simulator refusal, not only unit tests.

Read-only inspection of installed `Assembly-CSharp.dll` SHA
`697e89e113febed8ac63ec328f49d8906c16ef3e7aebfb99c920992917b62eb6`
traced gRPC `collisionObjects` through `Utils::GetBasicActorState` to the actual
trigger dictionary and `ProjectManager::GetUniqueID(GameObject)`. That method
returns -100 for a null/destroyed object or an instance absent from its ID map;
the client mapping preserves the numeric string. This identifies an unresolved
contact reference, not a registered actor ID. It does not identify the current
physical collider or establish that the scene is contact-free. The installed
DLL was not patched or replaced, and its before/after SHA matches. Raw IL and
native contact queries are preserved in D; no unknown ID is filtered away.

The independent `static-dwell-root-cause-r1.json` keeps all stopped samples of
the last three completed physical static trials. Both dwell cases occur in
SHIFT with transient native candidate loss/rest-evidence variation; the motion
epoch stays 1, confirming that signed rollback no longer resets this anchor.
This is a causal trace review, not proof of one unique physical root cause.

A new bridge-free ideal tracking diagnostic found a separate multi-object
limitation: center and ±0.75m offset boxes completed, while a second box at
80m after the first at 60m blocked a forced centerline return at x=66.194m.
The wrapper now retires a completely passed observed target only in PASS or
RETURN when another confirmed static target is ahead. Every local hull point
must lie behind the rear bumper plus 1m and the unchanged local margin. Both
objects remain collision inputs; a fresh PREPARE starts from the current pose,
and the original native path is never reclassified as successful. This is
research static behavior only; no main maneuver policy was changed.

The new `localization_static_scenarios_regression` checks four cases at 15km/h,
including target replacement only for the staggered case, complete PASS/RETURN,
actual sampled body clearance and a maximum of one native call per update.
All complete with minimum sampled clearance 0.699m or more, no complete stop,
and no duplicate native search. It is ideal trajectory following, not simulated
vehicle/MPC/IMU acceptance. Probe minimum clearance is reported separately:
rejected future probes must not be mislabeled as executed physical clearance.
Four scoped core/LOCAL/input/multi-object suites pass after rebuilding both
position nodes and the shared research core. Five unchanged main LiDAR tunnel,
ceiling/ground/preprocessing suites and ten integrated isolated ROS ownership,
handoff, actual MPC and duplicate-owner boundary checks also pass.
The new physical entry/multi-object candidate remains unaccepted while the
native contact reference is unresolved. Static and speed defaults remain gated.

### Correction to the initial synthetic rollout

The first ideal rollout and its apparent four-case completion above are
superseded. Reviewing its unpublished visualization revealed that copying
`trajectory[1]` into ego pose could jump laterally to a remote fixed reference
after target replacement. This invalidates those ideal completion/clearance
claims; it does not alter the real simulator results in E536. Both unpublished
gallery attempts and all r1–r6 diagnostic failures are preserved.

The wrapper now applies the existing native join bound to WAITING_REFERENCE:
a fixed guide whose first point is farther than 0.35m cannot replace checked
PREPARE maintenance. The corrected offline plant integrates a kinematic
bicycle with 50ms steps, pursuit lookahead max(2m, 0.8×speed), curvature/lateral
acceleration and steering slew limits. It does not reset pose or yaw from the
reference. An invalid decision holds steering and applies jerk-bounded braking,
counts that update and keeps all actual model body/road checks active. Three
seconds stopped still fails, and strict completion also requires ≥95% valid
updates. This plant is a declared simplified diagnostic, not the actual MPC
or MORAI vehicle response.

The same binary/plant A/B, `static-chain-15-r8.log` versus
`static-chain-baseline-15-r1.log`, changes only the constructor's explicit
sequential-target switch. Both complete center/left/right cases identically,
minimum model body clearance 0.6743m or more. The baseline staggered case stops
3.05s at x=74.616m with 111 invalid updates and fails strict completion. The
sequential candidate completes that case with one invalid braking update,
99.7059% valid updates, no complete stop and one native search per update.
The constructor switch exists for reproducible ablation, not a new ROS topic
or a change to main's default maneuver policy. Physical multi-object acceptance
remains pending. All failed strict runs retain their nonzero exit and raw logs.

Final scoped CTest rerun passes 17/17 suites in 40.37s, including the corrected
bounded model, real ROS approach/handoff/seed/motion boundaries and unchanged
LiDAR tunnel/preprocessing/ground suites. Python assessment/startup checks are
19/19, with ten separate integrated ROS ownership/MPC boundary assertions.
The corrected continuous-model gallery is E537:
http://localhost:8000/localization-integration-20261029-r01/offline-static-review-r3/index.html
Its same-model baseline failure, one invalid candidate update and withdrawn
point-replacement claims are explicit. The previous 536 identifiers and pages
were preserved; HTTP and the rendered figure were checked. E536 remains the
separate real simulator checkpoint. Fresh physical acceptance and higher-speed/
dynamic trials remain pending; no native-contact or stopping gate was bypassed.
