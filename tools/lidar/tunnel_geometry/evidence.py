"""Camera receipt, semantic evidence, and actual point protection are distinct."""
import math

SUPPORTED = frozenset(("car", "truck", "bus", "person", "pedestrian"))


def camera_evidence(decision):
    """Describe the frozen matching decision without treating empty as free space."""
    selected = decision.get("selected_yolo_index") is not None
    enabled = decision.get("protection_enabled", True)
    reason = decision.get("reason", "missing_yolo")
    boxes = decision.get("boxes", [])
    supported = sum(str(b.get("class", "")).lower() in SUPPORTED for b in boxes)
    valid = [all(math.isfinite(b.get(k, 0)) for k in ("xmin", "ymin", "xmax", "ymax")) and
             b.get("xmax", 0) > b.get("xmin", 0) and b.get("ymax", 0) > b.get("ymin", 0) for b in boxes]
    invalid_boxes = sum(not x for x in valid)
    valid_supported = sum(ok and str(b.get("class", "")).lower() in SUPPORTED for b, ok in zip(boxes, valid))
    if selected and decision.get("selected_yolo_arrival_ns", 0) > decision.get("decision_ns", 0):
        raise ValueError("camera evidence arrived after the decision")
    if not enabled or reason == "protection_off":
        state = "protection_off"
    elif reason == "invalid_scan_stamp" or (selected and
            decision.get("selected_yolo_stamp_ns", 0) <= 0):
        state = "invalid_stamp"
    elif selected:
        state = "valid_boxes" if valid_supported else "invalid_boxes" if supported else "unsupported_only" if boxes else "valid_empty"
    elif decision.get("unmatched_cause") == "outside_50ms":
        state = "outside_time_window"
    elif decision.get("invalid_yolo_count_before_decision", 0) and \
            decision.get("nearest_arrived_stamp_offset_ns") is None:
        state = "invalid_stamp"
    else:
        state = "timeout" if reason == "timeout" else "missing"
    return {"state": state, "message_selected": selected, "protection_enabled": enabled,
            "box_count": len(boxes), "supported_box_count": supported,
            "valid_supported_box_count": valid_supported,
            "invalid_box_count": invalid_boxes, "matching_reason": reason,
            "absence_is_free_space": False}
