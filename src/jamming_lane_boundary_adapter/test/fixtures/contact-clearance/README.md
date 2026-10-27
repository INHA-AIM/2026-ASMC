# Recorded contact-clearance regression

MORAI FOGGY11 drum trial `shadow-stop-fix-20261028-r15` stopped during a lane change.
The binary ROS fixtures preserve the exact `bbox.lane_stamp == family.header.stamp` pair.
`manifest.json` fixes their SHA and producer revision. They contain geometry, no images.

The native probe calls the runtime generator and unchanged road/curvature/footprint checks.
Before the quintic start-slope correction, all candidates at this state were rejected.
With the corrected heading basis, an adjacent-lane candidate has road and contact clearance.
This is a candidate-feasibility regression, not proof of actual tracking or independent GT accuracy.
The source bag and original attempt remain immutable in D storage.
