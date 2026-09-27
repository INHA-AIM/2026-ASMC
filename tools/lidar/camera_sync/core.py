"""Causal, arrival-ordered YOLO/LiDAR matching; no ROS runtime dependency."""
from collections import deque
from dataclasses import dataclass
from typing import Optional

MAX_OFFSET_NS = 50_000_000
MAX_WAIT_NS = 25_000_000
MAX_PENDING = 2


@dataclass(frozen=True)
class Event:
    kind: str
    arrival_ns: int
    stamp_ns: int
    source_index: int
    box_count: int = 0
    payload_digest: str = ""


def _stamp_reason(event, previous_stamp):
    if event.stamp_ns <= 0:
        return "zero_stamp"
    if previous_stamp is not None and event.stamp_ns < previous_stamp:
        return "regressive_stamp"
    if previous_stamp is not None and event.stamp_ns == previous_stamp:
        return "duplicate_stamp"
    return None


def evaluate(events, wait_ns=0, protection_enabled=True, max_pending=MAX_PENDING):
    """Return one decision per scan. Events must retain bag arrival order.

    At the first available valid YOLO result, a pending scan is decided; the
    matcher never waits for a potentially closer future result. Deadlines are
    processed before later-arriving events, but an event exactly on a deadline
    is admitted before timeout. Output order is scan order.
    """
    if wait_ns < 0 or max_pending < 1:
        raise ValueError("invalid wait or queue bound")
    pending = deque()
    history = deque()
    decisions = []
    previous_arrival = None
    previous_yolo_stamp = None
    invalid_yolo = []
    seen_yolo = 0
    last_source_index = -1

    def choose(scan, cutoff_ns):
        choices = [y for y in history
                   if abs(y.stamp_ns - scan.stamp_ns) <= MAX_OFFSET_NS
                   and y.arrival_ns <= cutoff_ns]
        if not choices:
            return None
        return min(choices, key=lambda y: (abs(y.stamp_ns - scan.stamp_ns),
                                            y.source_index))

    def finish(scan, reason, chosen=None, at=None):
        at = scan.arrival_ns if at is None else at
        if chosen is not None and chosen.arrival_ns > at:
            raise AssertionError("future YOLO result selected")
        eligible_history = [y for y in history if y.arrival_ns <= at]
        nearest_ns = min((abs(y.stamp_ns - scan.stamp_ns)
                          for y in eligible_history), default=None)
        unmatched_cause = None
        if chosen is None and reason not in ("protection_off", "invalid_scan_stamp"):
            unmatched_cause = ("not_yet_arrived" if nearest_ns is None else
                               "outside_50ms" if nearest_ns > MAX_OFFSET_NS else
                               "scan_order_blocked")
        decisions.append({
            "source_index": scan.source_index,
            "scan_stamp_ns": scan.stamp_ns,
            "scan_arrival_ns": scan.arrival_ns,
            "decision_ns": at,
            "wait_ns": at - scan.arrival_ns,
            "selected_yolo_index": chosen.source_index if chosen else None,
            "selected_yolo_stamp_ns": chosen.stamp_ns if chosen else None,
            "selected_yolo_arrival_ns": chosen.arrival_ns if chosen else None,
            "yolo_offset_ns": abs(chosen.stamp_ns - scan.stamp_ns) if chosen else None,
            "yolo_box_count": chosen.box_count if chosen else None,
            "reason": reason,
            "unmatched_cause": unmatched_cause,
            "nearest_arrived_stamp_offset_ns": nearest_ns,
            "invalid_yolo_count_before_decision": sum(
                item["arrival_ns"] <= at for item in invalid_yolo),
            "protection_enabled": protection_enabled,
            "scan_digest": scan.payload_digest,
            "selected_yolo_digest": chosen.payload_digest if chosen else None,
        })

    def eligible_or_waiting():
        while pending:
            scan = pending[0]
            chosen = choose(scan, now)
            if chosen is None:
                break
            pending.popleft()
            finish(scan, "matched_boxes" if chosen.box_count else "matched_empty",
                   chosen, now)

    def expire(before, include_equal):
        while pending and (pending[0].arrival_ns + wait_ns < before or
                           (include_equal and pending[0].arrival_ns + wait_ns == before)):
            scan = pending.popleft()
            at = scan.arrival_ns + wait_ns
            finish(scan, "timeout" if seen_yolo else "missing_yolo", at=at)

    for event in events:
        if event.kind not in ("scan", "yolo"):
            continue
        if previous_arrival is not None and event.arrival_ns < previous_arrival:
            raise ValueError("bag arrival timestamps regressed")
        if event.source_index <= last_source_index:
            raise ValueError("source index must increase")
        last_source_index = event.source_index
        previous_arrival = event.arrival_ns
        now = event.arrival_ns
        expire(now, False)
        # The pinned filter keeps only 64 YOLO frames, in a two-second stamp
        # horizon. Retaining less here would change matching eligibility.
        if event.kind == "yolo":
            seen_yolo += 1
            reason = _stamp_reason(event, previous_yolo_stamp)
            if reason:
                invalid_yolo.append({"source_index": event.source_index,
                                     "arrival_ns": event.arrival_ns,
                                     "stamp_ns": event.stamp_ns,
                                     "reason": reason})
            else:
                previous_yolo_stamp = event.stamp_ns
                history.append(event)
                if len(history) > 64:
                    history.popleft()
                while history and event.stamp_ns - history[0].stamp_ns > 2_000_000_000:
                    history.popleft()
                eligible_or_waiting()
        else:
            stamp_reason = _stamp_reason(event, None)
            if stamp_reason:
                finish(event, "invalid_scan_stamp")
            elif not protection_enabled:
                finish(event, "protection_off")
            else:
                chosen = choose(event, now)
                if chosen is not None:
                    finish(event, "matched_boxes" if chosen.box_count else "matched_empty",
                           chosen, now)
                else:
                    if len(pending) >= max_pending:
                        oldest = pending.popleft()
                        finish(oldest, "queue_overflow", at=now)
                    pending.append(event)
        expire(now, True)
    if pending:
        final = max(pending[-1].arrival_ns + wait_ns, previous_arrival or 0)
        expire(final, True)
    decisions.sort(key=lambda row: row["source_index"])
    return decisions, invalid_yolo
