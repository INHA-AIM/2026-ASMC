# Recorded contact-clearance regression

MORAI FOGGY11 drum trial `shadow-stop-fix-20261028-r15` stopped during a lane change.
The binary ROS fixtures preserve the exact `bbox.lane_stamp == family.header.stamp` pair.
`manifest.json` fixes their SHA and producer revision. They contain geometry, no images.

The native probe calls the runtime generator and unchanged road/curvature/footprint checks.
Before the quintic start-slope correction, all candidates at this state were rejected.
With the corrected heading basis, an adjacent-lane candidate has road and contact clearance.
This is a candidate-feasibility regression, not proof of actual tracking or independent GT accuracy.
The source bag and original attempt remain immutable in D storage.

`near-start-*` is the exact r31 family/result pair. Its original result reported
zero support because the corrected lane begins at1.11392m. `--support-replay`
uses the same contiguous-support function as the classifier; the expected end
is19.4756m and the unchanged road/vehicle/obstacle checks find a path. This is
not a driving-success or independent geometry-GT fixture. Interior gaps and
near starts beyond one0.25m sampling interval remain rejected.

`short-end-*` preserves r34's exact input, where the shared observed support ends
at14.764m. The former fixed15m requirement produced no road reference. Runtime
now generates only to the existing grid point whose entire inflated vehicle is
inside the paired support. It does not extrapolate to15m or relax road/curvature
checks. `short-end-provenance.json` preserves the source and original failure.

`turn-blocked-*` is a known unsuccessful r45 state after moving the plastic
barrier farther into the tunnel. Its box has real local LiDAR support, yet the
target-lane candidates cannot jointly satisfy road and obstacle constraints
during the turn. This is input for the next planner/controller comparison,
not a regression requirement that this state must always remain rejected.
The family/result stamps are paired exactly; the nearby live diagnostic was
not atomic with those messages. No source images or simulator GT are embedded.
The later15m completion fallback passes the same geometry guards for this one
input after the original147 candidates fail. Its subsequent r48 driving trial
still stops later; this fixture is not closed-loop driving acceptance.
