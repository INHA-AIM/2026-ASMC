import copy
import unittest

from core import GpsGate, select_output


N = 1_000_000_000


def fix(t, x=0., y=0., **kw):
    row = dict(arrival_ns=t, stamp_ns=t, clock="ros_receive", status=1,
               latitude=37.0, longitude=127.0, altitude=28.0, xy_m=(x, y))
    row.update(kw)
    return row


def snapshot(t, objects=True, pose=True):
    item = {"id": 42, "classification": "vehicle", "valid": True,
            "base": {"center": [5., 1., 0.], "corners": [[1., 2., 3.]] * 8},
            "map": {"center": [12., 8., 0.], "corners": [[9., 8., 7.]] * 8} if pose else None,
            "absolute_velocity_base": None, "absolute_velocity_map": None,
            "map_reason": "valid" if pose else "pose_missing"}
    return {"input_id": "s1", "stamp_ns": t, "valid": True, "reason": "valid",
            "pose_reason": "valid" if pose else "pose_missing",
            "objects": [item] if objects else []}


def select(row, gate, t, **kw):
    return select_output(row, gate, processing_ns=t, detector_ran=True,
                         pose_trusted=True, vehicle_state_ready=True,
                         trajectory_ready=True, **kw)


class TransitionContract(unittest.TestCase):
    def ready(self):
        gate = GpsGate()
        for t in (N, N + 200_000_000, N + 400_000_000):
            gate.feed(**fix(t, x=(t-N)/1e9))
        self.assertEqual(gate.state, "ready")
        return gate

    def test_hysteresis_local_identity_and_frames(self):
        gate = self.ready()
        row = snapshot(N + 400_000_000)
        first = select(row, gate, N + 400_000_000)
        self.assertTrue(first["planner_ready"])
        self.assertEqual(first["map"]["frame"], "map")
        self.assertEqual(first["local"]["frame"], "base_link")
        self.assertEqual(first["map"]["objects"][0]["id"], 42)
        self.assertEqual(first["stamp_ns"], row["stamp_ns"])
        bad_t = N + 600_000_000
        gate.feed(**fix(bad_t, latitude=0., longitude=0.))
        shadow = select(snapshot(bad_t), gate, bad_t)
        self.assertEqual(shadow["gps"]["state"], "invalid")
        self.assertEqual(shadow["local"]["objects"][0]["id"], 42)
        self.assertEqual(shadow["map"]["status"], "gps_invalid")
        self.assertEqual(shadow["map"]["objects"], [])
        self.assertFalse(shadow["planner_ready"])
        for i, t in enumerate((N+800_000_000, 2*N, 2*N+200_000_000)):
            gate.feed(**fix(t, x=0.8+i*.2))
            self.assertEqual(gate.state, "ready" if i == 2 else "recovering")
        recovered = select(snapshot(2*N+200_000_000), gate, 2*N+200_000_000)
        self.assertEqual(recovered["map"]["objects"][0]["id"], 42)

    def test_stale_and_old_stamp(self):
        gate = self.ready()
        self.assertEqual(gate.tick(2*N)["state"], "stale")
        self.assertEqual(select(snapshot(2*N), gate, 2*N)["map"]["status"], "gps_stale")
        with self.assertRaises(ValueError):
            gate.feed(**fix(2*N-1))
        gate.feed(**fix(3*N, stamp_ns=2*N))
        self.assertEqual(gate.reason, "old_gps_stamp")

    def test_bad_stamp_clock_numeric_and_status(self):
        cases = [dict(status=0), dict(latitude=float("nan")), dict(latitude=0., longitude=0.),
                 dict(latitude=91.), dict(clock="sim"), dict(stamp_ns=0),
                 dict(stamp_ns=N+500_000_001)]
        for delta in cases:
            gate = self.ready()
            gate.feed(**fix(N+500_000_000, **delta))
            self.assertNotEqual(gate.state, "ready", delta)
        gate = self.ready()
        gate.feed(**fix(N+400_000_000))
        self.assertEqual(gate.reason, "stamp_invalid")
        gate.feed(**fix(N+600_000_000, stamp_ns=N+300_000_000))
        self.assertEqual(gate.reason, "stamp_invalid")
        with self.assertRaises(ValueError):
            gate.feed(**fix(N+200_000_000))

    def test_jump_latched_until_independent_reanchor(self):
        gate = self.ready()
        t = N + 600_000_000
        self.assertEqual(gate.feed(**fix(t, x=200.))["state"], "jump_rejected")
        self.assertEqual(gate.feed(**fix(t+200_000_000, x=201.))["state"], "jump_rejected")
        bad_anchor = {"source": "gps", "valid": True, "stamp_ns": t+400_000_000, "xy_m": [202., 0.]}
        self.assertEqual(gate.feed(**fix(t+400_000_000, x=202., trusted_reanchor=bad_anchor))["state"],
                         "jump_rejected")
        self.assertEqual(gate.feed(**fix(t+500_000_000, x=202.5))["state"], "jump_rejected")
        good_anchor = dict(bad_anchor, source="independent_odom", stamp_ns=t+600_000_000, xy_m=[203., 0.])
        self.assertEqual(gate.feed(**fix(t+600_000_000, x=203., trusted_reanchor=good_anchor))["state"],
                         "recovering")
        for dt, x in ((800_000_000, 203.2), (1_000_000_000, 203.4)):
            gate.feed(**fix(t+dt, x=x))
        self.assertEqual(gate.state, "ready")
        # A fresh gate must not eventually accept the same rejected point just
        # because elapsed time made a growing speed threshold permissive.
        latched = self.ready()
        latched.feed(**fix(t, x=200.))
        self.assertEqual(latched.feed(**fix(t+20*N, x=200.))["state"], "jump_rejected")

    def test_pose_vehicle_trajectory_and_future_health_separate(self):
        gate = self.ready()
        t = N+400_000_000
        row = snapshot(t)
        for changes, expected in [({"pose_trusted": False}, "pose_unavailable"),
                                  ({"vehicle_state_ready": False}, "vehicle_state_unavailable"),
                                  ({"trajectory_ready": False}, "trajectory_unavailable")]:
            args = dict(processing_ns=t, detector_ran=True, pose_trusted=True,
                        vehicle_state_ready=True, trajectory_ready=True)
            args.update(changes)
            out = select_output(row, gate, **args)
            self.assertEqual(out["planner_reason"], expected)
        gate.feed(**fix(t+200_000_000, x=.6))
        self.assertEqual(select(row, gate, t+200_000_000)["map"]["status"],
                         "gps_evidence_after_observation")
        row["pose_reason"] = "pose_gap_too_large"
        self.assertEqual(select(snapshot(t+200_000_000, pose=False), gate, t+200_000_000)["map"]["status"],
                         "pose_unavailable")

    def test_missing_unassessed_empty_and_invalid_distinct(self):
        gate = self.ready()
        t = N+400_000_000
        missing = select(None, gate, t)
        self.assertEqual(missing["local"]["status"], "sensor_missing")
        row = snapshot(t, objects=False)
        self.assertEqual(select(row, gate, t)["local"]["status"], "observed_empty")
        not_run = select_output(row, gate, processing_ns=t, detector_ran=False,
                                pose_trusted=True, vehicle_state_ready=True, trajectory_ready=True)
        self.assertEqual(not_run["local"]["status"], "unassessed_detector_not_run")
        bad = copy.deepcopy(row)
        bad["valid"], bad["reason"] = False, "duplicate_stamp"
        self.assertEqual(select(bad, gate, t)["local"]["status"],
                         "invalid_observation:duplicate_stamp")
        bad_stamp = copy.deepcopy(row)
        bad_stamp["stamp_ns"] = 0
        self.assertEqual(select(bad_stamp, gate, t)["local"]["status"],
                         "invalid_observation:stamp_invalid")
        self.assertFalse(not_run["planner_ready"])


if __name__ == "__main__":
    unittest.main()
