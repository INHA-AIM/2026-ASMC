#!/usr/bin/env python3
"""Build results/TRACK_BOX_STATE_AUDIT.json from the verified audit workflow output (work/audit/audit_structured.json)
plus the orchestrator's curated per-item summary (every statement below is backed by a verified finding)."""
import json, sys

sys.dont_write_bytecode = True
RR = "/home/autonav/2026AIM/Lidar_experimments/obb_quality_temporal_20261018_170239"
P = "src/lidar+sensorfusion/src/Main/src/"

SUMMARY = {
    "base_commit": "916e4d9efb38f9066aa4b079df05d16ef011c063",
    "profile_used_by_experiment": "Main/main_integration.launch (enable_common_geometry=true, lidar_level_roll=true, planner_udp_enabled:=false)",
    "per_track_box_state": {
        "obb_size_temporal_filter": "NONE. f32_OBB_Width/Height/ZMin/ZMax and OBB center are recomputed every scan from the current cluster points "
                                    "(writeOBBToObject callers: Objectinfo_for_km, RecomputeGeometry, stabilizeMatchedOBB; " + P + "Lidar/Tracking/Tracking.cpp:203-223,538,390,259)",
        "obb_yaw_temporal_filter": "stabilizeMatchedOBB (Tracking.cpp:227-261): matched tracks only, skipped after a miss or for births; "
                                   "predicted = previous yaw re-expressed by AlignTrackingFrame (FB mode); if raw aspect W/H >= 1.2 then "
                                   "yaw = pred + 0.25*remainder(raw-pred, pi) else yaw = pred (open loop); OBB then re-fitted from current points at that yaw "
                                   "WITHOUT long-axis re-normalisation, so W<H (V2 length<width, yaw along short side) is possible.",
        "obb_center_temporal_filter": "NONE. V2 center = per-scan OBB extent midpoint (FrameBoundaryPublisher.cpp:117). KF tracks the point-mean centroid f32_CoreX/Y only (Kalman.cpp:141-206).",
        "state_carried_for_matched_tracks": "explicit field copy in CompareCentroids (Tracking.cpp:707-739): PrevCoreX/Y, KF state/P, b_KfInitialized, KfInitCount, Id, Age+1, "
                                            "MissCount=0, Static/DynamicCount, PrevE/N, PrevYaw, tracking_frame_*; class via fusion_state::Copy; OBB yaw only as EMA prior. "
                                            "A new LiDARObjectInfo field is NOT carried for matched tracks unless copied explicitly; it IS carried wholesale for missed tracks (Tracking.cpp:916).",
        "missed_tracks": "wholesale copy, predict-only, kept while MissCount<25; stale OBB center/size/corners, yaw re-expressed in FB mode only; never published (Publishable, FrameBoundaryPublisher.cpp:44-46).",
        "track_ids": "global monotonic counter s32_NewTrackId++ (3 birth sites), never reset/reused in a process (Global.cpp:984-985, FrameFilter.cpp:40,130, Tracking.cpp:886).",
        "struct_serialisation": "LiDARObjectInfo is never memcpy-ed or sent raw; UDP/ROS outputs are built field by field (PlannerObjectSender.cpp:28-31,62-63; FrameBoundaryPublisher.cpp:13-24).",
    },
    "obb_geometry_consumers": {
        "association_cost": "ShapeAgreement(prev W/H, current raw W/H) in the Hungarian cost (Tracking.cpp:631-633,657-673): history OBB W/H (stabilised for matched tracks) feeds association.",
        "merged_track_retire": "ShapeAgreement(prev, current STABILISED) >= 0.28 and distance <= 0.45 m (Tracking.cpp:856-877).",
        "duplicate_merge": "ConsolidateCurrentDetections: raw OBB center distance <= 0.60 m AND body AABB IoU >= 0.35, z ignored, before association (Tracking.cpp:395-441, FrameFilter.cpp:26).",
        "motion_classifier": "OBB_Corners_Body of MissCount==0 objects -> map AABB -> displacement speed / warm-up certify (MotionClassifier.cpp:36-45,384-394,622-628); runs inside LidarProcess before publish.",
        "v2_output": "BuildV2 center=f32_OBB_Center*, long_axis=(cos,sin)(f32_OBB_YawRad), corners=OBB_Corners_Body, length=Width, width=Height, height=ZMax-ZMin (FrameBoundaryPublisher.cpp:93-134); "
                     "called TWICE per scan in kBoth (local+map) -> a stateful stabiliser must not be stepped inside BuildV2.",
        "v1_and_legacy": "V1 Publish has its own duplicated loop; PlannerObjectAdapter/UDP/legacy ROS read the same f32_OBB_* fields through MakePlannerObjects (PlannerObjectAdapter.cpp:139-193).",
        "fusion": "FusionLiDAR::UpdateAABB overwrites f32_CoreX/Y/Z and Min/Max after LidarProcess, never OBB fields (FusionLiDAR.cpp:125-132).",
    },
    "experiment_design_constraints_derived": [
        "An output-only stabiliser must NOT write f32_OBB_* of `object` before LidarProcess.cpp:309-311 (history copy) or before MotionClassifier.Update (LidarProcess.cpp:281): that would change association (ShapeAgreement, retire), motion evidence and the existing yaw EMA chain.",
        "Its state must be stepped exactly once per processed scan (not inside BuildV2, which runs twice in kBoth) and keyed by the monotonic track id.",
        "The evaluator must canonicalise boxes (long = max(l,w), short = min(l,w), axis yaw of the long side) for both GT and prediction because V2 length<width occurs.",
        "V2 yaw is a plain atan2 (range (-pi,pi]) and can jump by ~pi for the same rectangle; all yaw metrics must be pi-periodic (axis) metrics.",
    ],
}


def main():
    d = json.load(open(RR + "/work/audit/audit_structured.json"))
    keys = ["roi_preproc", "clustering", "merge_obb", "track_box_state", "lifecycle", "output_frames"]
    kof = lambda a: next((k for k in keys if a.startswith(k)), a)
    A = {kof(a["area"]): a for a in d["audits"]}
    V = {kof(v["area"]): v for v in d["verifies"]}
    areas = {}
    counts = {"findings": 0, "confirmed": 0, "corrected": 0, "refuted": 0, "missed_facts_added": 0, "call_edges": 0}
    for k in keys:
        a, v = A[k], V[k]
        st = {x["index"]: x for x in v["verdicts"]}
        fs = []
        for i, f in enumerate(a["findings"]):
            s = st.get(i, {})
            counts["findings"] += 1
            counts[s.get("status", "refuted")] = counts.get(s.get("status", "refuted"), 0) + 1
            if s.get("status") == "refuted":
                continue
            claim = s.get("corrected_claim") if s.get("status") == "corrected" and s.get("corrected_claim") else f["claim"]
            ev = s.get("corrected_evidence") if s.get("status") == "corrected" and s.get("corrected_evidence") else f["evidence"]
            fs.append({"item": f["item"], "status": s.get("status"), "claim": claim, "evidence": ev,
                       "experiment_risk": f["experiment_risk"], "confidence": f["confidence"],
                       "verifier_note": s.get("note")})
        counts["missed_facts_added"] += len(v["missed_facts"])
        counts["call_edges"] += len(a["call_edges"])
        areas[k] = {"findings": fs, "missed_facts_added_by_verifier": v["missed_facts"], "call_edges": a["call_edges"],
                    "open_questions": a["open_questions"]}
    out = {"generated_by": "tools/build_audit_json.py from the read-only multi-agent audit (6 auditors + 6 adversarial verifiers, workflow wf_6e417218-429)",
           "worktree": RR + "/repo", "counts": counts, "summary": SUMMARY, "areas": areas}
    json.dump(out, open(RR + "/results/TRACK_BOX_STATE_AUDIT.json", "w"), indent=1, ensure_ascii=False)
    print(json.dumps(counts))


if __name__ == "__main__":
    main()
