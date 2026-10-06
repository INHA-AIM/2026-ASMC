"""Ordered integer-box comparison, independent of ROS and GPU libraries."""


def boundary_only(old, new, candidates, tolerance=0.005):
    """Allow only one-sided near-threshold insertions or deletions.

    A coordinate change, reorder, or mixed deletion+insertion is a failure,
    even if all affected confidences are close to the decision boundary.
    """
    near = {tuple(row[:5]) for row in candidates if abs(row[5] - 0.7) <= tolerance}
    if len(old) == len(new):
        return [] if old == new else None
    shorter, longer = (old, new) if len(old) < len(new) else (new, old)
    side = "new_only" if len(old) < len(new) else "old_only"
    index, removed = 0, []
    for box in longer:
        if index < len(shorter) and box == shorter[index]:
            index += 1
        elif box in near and box not in shorter:
            removed.append((side, box))
        else:
            return None
    return removed if index == len(shorter) else None
