import unittest

from core import Event, evaluate, MAX_OFFSET_NS


MS = 1_000_000
BASE = 1_000_000_000


def scan(index=1, arrival=0, stamp=0):
    return Event("scan", BASE + arrival * MS, BASE + stamp * MS, index,
                 payload_digest="unchanged-cloud")


def yolo(index=0, arrival=0, stamp=0, boxes=1):
    return Event("yolo", BASE + arrival * MS, BASE + stamp * MS, index,
                 box_count=boxes, payload_digest="unchanged-yolo")


class MatcherTest(unittest.TestCase):
    def test_stamp_boundaries(self):
        for offset in (49, 50, 51):
            result, _ = evaluate([yolo(stamp=offset), scan()], 0)
            self.assertEqual(result[0]["reason"] == "matched_boxes", offset <= 50)

    def test_arrival_boundaries(self):
        for delay in (0, 10, 25, 26, 50, 130, 200):
            events = [scan()]
            if delay:
                events.append(yolo(index=2, arrival=delay, stamp=0))
            else:
                events = [yolo(), scan()]
            base, _ = evaluate(events, 0)
            trial, _ = evaluate(events, 25 * MS)
            self.assertEqual(base[0]["reason"] == "matched_boxes", delay == 0)
            self.assertEqual(trial[0]["reason"] == "matched_boxes", delay <= 25)
            self.assertLessEqual(trial[0]["wait_ns"], 25 * MS)

    def test_valid_empty_is_not_missing(self):
        result, _ = evaluate([yolo(boxes=0), scan()], 25 * MS)
        self.assertEqual(result[0]["reason"], "matched_empty")
        self.assertEqual(result[0]["wait_ns"], 0)

    def test_invalid_and_off(self):
        events = [Event("yolo", BASE, 0, 0),
                  yolo(index=1, arrival=1, stamp=1),
                  yolo(index=2, arrival=2, stamp=1),
                  yolo(index=3, arrival=3, stamp=0),
                  scan(index=4, arrival=4, stamp=1)]
        result, invalid = evaluate(events, 0)
        self.assertEqual([x["reason"] for x in invalid],
                         ["zero_stamp", "duplicate_stamp", "regressive_stamp"])
        self.assertEqual([x["accepted"] for x in invalid], [False, True, True])
        self.assertEqual(result[0]["selected_yolo_index"], 1)
        off, _ = evaluate(events, 25 * MS, False)
        self.assertEqual(off[0]["reason"], "protection_off")

    def test_timeout_queue_and_no_future(self):
        events = [scan(1), scan(2, 1, 1), scan(3, 2, 2),
                  yolo(4, 10, 0), scan(5, 26, 26)]
        result, _ = evaluate(events, 25 * MS)
        self.assertEqual(result[0]["reason"], "queue_overflow")
        self.assertEqual(result[1]["reason"], "matched_boxes")
        for row in result:
            if row["selected_yolo_arrival_ns"] is not None:
                self.assertLessEqual(row["selected_yolo_arrival_ns"], row["decision_ns"])

    def test_first_available_not_future_closest(self):
        result, _ = evaluate([scan(), yolo(2, 10, 40), yolo(3, 20, 0)], 25 * MS)
        self.assertEqual(result[0]["selected_yolo_index"], 2)
        self.assertEqual(result[0]["wait_ns"], 10 * MS)

    def test_repeat_deterministic(self):
        events = [scan(), yolo(2, 10, 10)]
        self.assertEqual(evaluate(events, 25 * MS), evaluate(events, 25 * MS))

    def test_head_blocks_later_matched_scan(self):
        result, _ = evaluate([scan(0), yolo(1, 1, 60), scan(2, 10, 10)], 25 * MS)
        self.assertEqual([r["decision_ns"] for r in result], [BASE+25*MS]*2)
        self.assertEqual([r["reason"] for r in result], ["timeout", "matched_boxes"])

    def test_earlier_stamp_wins_offset_tie(self):
        result, _ = evaluate([yolo(0, 0, 60), yolo(1, 1, 40), scan(2, 2, 50)], 0)
        self.assertEqual(result[0]["selected_yolo_index"], 1)

    def test_deadline_event_after_same_time_scan(self):
        result, _ = evaluate([scan(0), scan(1, 25, 25), yolo(2, 25, 0)], 25 * MS)
        self.assertEqual(result[0]["reason"], "matched_boxes")
        self.assertEqual(result[0]["wait_ns"], 25 * MS)

    def test_positive_old_stamp_accepted_but_evicted(self):
        events = [yolo(0, 0, 3000), yolo(1, 1, 0), scan(2, 2, 0)]
        result, diagnostics = evaluate(events, 0)
        self.assertTrue(diagnostics[0]["accepted"])
        self.assertEqual(result[0]["reason"], "timeout")

    def test_raw_nanoseconds_preserved_legacy_stamp_separate(self):
        e = Event("scan", BASE, BASE+999999, 1, effective_stamp_ns=BASE)
        result, _ = evaluate([yolo(0, 0, -50), e], 0)
        self.assertEqual(result[0]["scan_stamp_ns"], BASE+999999)
        self.assertEqual(result[0]["yolo_offset_ns"], 50*MS)

    def test_invalid_scan_waits_behind_head(self):
        result, _ = evaluate([scan(0), Event("scan", BASE+MS, 0, 1)], 25*MS)
        self.assertEqual([r["decision_ns"] for r in result], [BASE+25*MS]*2)
        self.assertEqual(result[1]["reason"], "invalid_scan_stamp")


if __name__ == "__main__":
    unittest.main()
