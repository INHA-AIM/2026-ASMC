"""Causal FIFO replay of Son 6435af1's camera history and stamp matching."""
from dataclasses import dataclass
from collections import deque
from itertools import groupby

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
    effective_stamp_ns: int = None

    @property
    def match_stamp_ns(self):
        return self.stamp_ns if self.effective_stamp_ns is None else self.effective_stamp_ns


def evaluate(events, wait_ns=0, protection_enabled=True, max_pending=MAX_PENDING):
    """Decisions are emitted in scan order, never sorted after processing.

    Original OnYolo accepts duplicate/out-of-order positive stamps, inserts by
    stamp (upper_bound), and retains 64 frames within two seconds of the newest.
    Equal stamp-distance ties select the earlier stamp, then first insertion.
    Such stamps are diagnosed, not silently rejected. Invalid scan stamps still
    go through FIFO. At a deadline all same-time bag events are admitted in
    source order before timeout; zero-wait scans decide at their arrival event.
    Overflow emits the oldest scan and immediately rechecks the next head.
    """
    if wait_ns < 0 or max_pending < 1:
        raise ValueError("invalid wait or queue bound")
    events = [e for e in events if e.kind in ("scan", "yolo")]
    for a, b in zip(events, events[1:]):
        if b.arrival_ns < a.arrival_ns or b.source_index <= a.source_index:
            raise ValueError("arrival and source order must not regress")
    history, pending, decisions, diagnostics = [], deque(), [], []
    previous_yolo_stamp = None

    def choose(scan, at):
        candidates = [y for y in history if y.arrival_ns <= at and
                      abs(y.match_stamp_ns - scan.match_stamp_ns) <= MAX_OFFSET_NS]
        return min(candidates, key=lambda y: (abs(y.match_stamp_ns - scan.match_stamp_ns),
                   y.match_stamp_ns, y.source_index), default=None)

    def finish(scan, reason, at, chosen=None):
        nearest = min((abs(y.match_stamp_ns - scan.match_stamp_ns) for y in history
                       if y.arrival_ns <= at), default=None)
        if chosen and chosen.arrival_ns > at:
            raise AssertionError("future camera data selected")
        if decisions and (scan.source_index <= decisions[-1]["source_index"] or
                          at < decisions[-1]["decision_ns"]):
            raise AssertionError("decision order reversed")
        unmatched = None
        if chosen is None and reason not in ("protection_off", "invalid_scan_stamp"):
            unmatched = ("not_yet_arrived" if nearest is None else
                         "outside_50ms" if nearest > MAX_OFFSET_NS else "scan_order_blocked")
        decisions.append({
            "source_index": scan.source_index, "scan_stamp_ns": scan.stamp_ns,
            "scan_effective_stamp_ns": scan.match_stamp_ns,
            "scan_arrival_ns": scan.arrival_ns, "decision_ns": at,
            "decision_sequence": len(decisions), "wait_ns": at - scan.arrival_ns,
            "selected_yolo_index": chosen.source_index if chosen else None,
            "selected_yolo_stamp_ns": chosen.stamp_ns if chosen else None,
            "selected_yolo_arrival_ns": chosen.arrival_ns if chosen else None,
            "yolo_offset_ns": abs(chosen.match_stamp_ns - scan.match_stamp_ns) if chosen else None,
            "yolo_box_count": chosen.box_count if chosen else None, "reason": reason,
            "unmatched_cause": unmatched, "nearest_arrived_stamp_offset_ns": nearest,
            "invalid_yolo_count_before_decision": sum(d["arrival_ns"] <= at for d in diagnostics),
            "protection_enabled": protection_enabled, "scan_digest": scan.payload_digest,
            "selected_yolo_digest": chosen.payload_digest if chosen else None,
        })

    def drain(at, expire_equal=False):
        while pending:
            scan = pending[0]
            chosen = choose(scan, at)
            reason = None
            if scan.stamp_ns <= 0 or scan.match_stamp_ns <= 0:
                reason = "invalid_scan_stamp"
            elif not protection_enabled:
                reason = "protection_off"
            elif chosen:
                reason = "matched_boxes" if chosen.box_count else "matched_empty"
            elif expire_equal and scan.arrival_ns + wait_ns <= at:
                reason = "timeout" if history else "missing_yolo"
            if reason is None:
                break
            pending.popleft()
            finish(scan, reason, at, chosen if reason.startswith("matched") else None)

    for now, group in groupby(events, key=lambda e: e.arrival_ns):
        while pending and pending[0].arrival_ns + wait_ns < now:
            drain(pending[0].arrival_ns + wait_ns, True)
        for event in group:
            if event.kind == "yolo":
                reason = ("zero_stamp" if event.stamp_ns <= 0 else
                          "regressive_stamp" if previous_yolo_stamp is not None and
                          event.stamp_ns < previous_yolo_stamp else
                          "duplicate_stamp" if previous_yolo_stamp == event.stamp_ns else None)
                if reason:
                    diagnostics.append({"source_index": event.source_index,
                        "arrival_ns": now, "stamp_ns": event.stamp_ns,
                        "reason": reason, "accepted": event.stamp_ns > 0})
                if event.stamp_ns > 0:
                    previous_yolo_stamp = event.stamp_ns
                    history.append(event)
                    history.sort(key=lambda y: (y.match_stamp_ns, y.source_index))
                    newest = history[-1].match_stamp_ns
                    history = [y for y in history if newest - y.match_stamp_ns <= 2_000_000_000][-64:]
                    drain(now)
            else:
                if len(pending) >= max_pending:
                    finish(pending.popleft(), "queue_overflow", now)
                    drain(now)
                pending.append(event)
                drain(now, expire_equal=wait_ns == 0)
        drain(now, True)
    while pending:
        drain(pending[0].arrival_ns + wait_ns, True)
    return decisions, diagnostics
