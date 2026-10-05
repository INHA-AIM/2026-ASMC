"""Known point labels score preservation and final objects, never filter labels as GT."""
import numpy as np


def score(mode, truth, pre_wall_labels, points=None, object_id=3):
    truth = np.asarray(truth)
    pre = np.asarray(pre_wall_labels)
    labels = np.asarray(mode["labels"])
    clusters = np.asarray(mode["cluster_id"])
    if not (len(truth) == len(pre) == len(labels) == len(clusters)):
        raise ValueError("point-index lineage mismatch")
    # The wall-stage denominator excludes points already removed upstream.
    target = (truth == object_id) & (pre == 0)
    wall = (truth == 1) & (pre == 0)
    count = int(target.sum())
    candidates = []
    for ci, obj in enumerate(mode["objects"]):
        member = clusters == ci
        intersection = int((member & target).sum())
        if not intersection:
            continue
        completeness = intersection / count if count else None
        purity = intersection / int(member.sum())
        containment = None
        if points is not None:
            xyz = np.asarray(points)[target, :3]
            center = np.asarray(obj["center"])
            yaw = obj["yaw"]
            axis = np.array([[np.cos(yaw), np.sin(yaw), 0],
                             [-np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
            local = (xyz - center) @ axis.T
            half = np.asarray(obj["dimensions"]) / 2
            # L-shape OBB is the pinned implementation; numerical edge tolerance only.
            containment = float(np.mean(np.all(np.abs(local) <= half + 1e-4, axis=1)))
        candidates.append({"cluster_index": ci, "removed": obj["removed"],
                           "completeness": completeness, "purity": purity,
                           "obb_contains_visible_target_fraction": containment,
                           "obb_center": obj["center"], "obb_dimensions": obj["dimensions"],
                           "removal_flags": obj.get("flags", {})})
    survivors = [x for x in candidates if not x["removed"]]
    best = max(survivors, key=lambda x: (x["completeness"], x["purity"]), default=None)
    return {"original_target_points": int((truth == object_id).sum()),
            "wall_stage_target_points": count,
            "target_removed_upstream": int(((truth == object_id) & (pre != 0)).sum()),
            "target_kept": int((target & (labels == 0)).sum()),
            "target_wall_removed": int((target & (labels == 3)).sum()),
            "target_preservation": int((target & (labels == 0)).sum()) / count if count else None,
            "wall_stage_wall_points": int(wall.sum()),
            "wall_points_kept": int((wall & (labels == 0)).sum()),
            "final_target": best, "target_clusters": candidates,
            "accepted_target": bool(best and best["completeness"] >= .8 and best["purity"] >= .9)}


def promotion_gate(scores, baseline_successes):
    """Fixed synthetic gates; absence of evidence cannot make a candidate pass."""
    required = ("car", "person", "low_unknown")
    reasons = []
    for kind in required:
        case = scores.get(kind)
        if case is None:
            reasons.append(kind + ":missing_required_case")
            continue
        candidate, missing = case["geometry"], case["missing_reference"]
        if candidate["target_preservation"] is None or candidate["target_preservation"] < .95:
            reasons.append(kind + ":preservation_below_95pct")
        if not candidate["accepted_target"]:
            reasons.append(kind + ":final_completeness_or_purity_failed")
        before = missing["wall_points_kept"]
        if before <= 0 or candidate["wall_points_kept"] > .5 * before:
            reasons.append(kind + ":wall_reduction_below_50pct_or_no_reference")
    if baseline_successes:
        reasons.append("baseline_success_regression")
    return {"passed": not reasons, "reasons": reasons}
