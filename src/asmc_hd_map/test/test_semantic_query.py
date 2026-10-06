import json
from types import SimpleNamespace as Obj
import unittest

from asmc_hd_map.semantic_query import SemanticMapIndex


def layer(**fields):
    return Obj(build_id="build", map_id="map", geometry_sha256="sha", complete=True,
               header=Obj(frame_id="map"), **fields)


def mission(id, start, end, payload, provisional=False, signal=False):
    return Obj(id=id, start_link_id="link", end_link_id="link", link_ids=["link"], route_station_start_m=start, route_station_end_m=end,
               status=1 if provisional else 2, STATUS_PROVISIONAL=1,
               mission_type=3 if signal else 4, TYPE_TRAFFIC_LIGHT=3,
               rule_payload_json=json.dumps(payload))


class SemanticQueryTest(unittest.TestCase):
    def setUp(self):
        self.meta = layer(operational=True)
        self.speed = mission("speed", 0, 100, {"route_speed_limits": [
            {"link_id": "link", "effective_limit_kph": 60., "mgeo_max_speed_kph": 70.,
             "competition_cap_kph": 60., "route_station_start_m": 0., "route_station_end_m": 100.}
        ]})
        self.signal = mission("signal", 90, 90, {"boundary_only": True,
            "traffic_light_id": "TL-1", "hd_map_stopline_ids": ["stop"]}, signal=True)
        self.provisional = mission("provisional", 0, 100, {}, provisional=True)
        self.partial = mission("obstacle", 25, 50, {})
        self.activation = mission("activation", 0, 100, {}, signal=True)
        self.stop = Obj(id="stop", polyline=[Obj(x=90., y=-2.), Obj(x=90., y=2.)])
        self.layers = {"missions": layer(zones=[self.speed, self.signal, self.provisional, self.partial, self.activation]),
                       "markings": layer(markings=[self.stop])}

    def test_selective_subscription_does_not_require_raster_or_lanes(self):
        index = SemanticMapIndex(self.meta, {"missions": self.layers["missions"]})
        self.assertEqual(index.speed_limit("link")["effective_limit_kph"], 60.)
        self.assertIsNone(index.speed_limit("off-route"))
        signal = index.signals_ahead(80., 15.)[0]
        self.assertEqual(signal["traffic_light_id"], "TL-1")
        self.assertEqual(signal["distance_m"], 10.)
        self.assertEqual(signal["stopline_ids"], ["stop"])
        self.assertEqual(signal["stoplines"], [])
        with self.assertRaises(ValueError):
            index.spatial((0, 0), 0, layers=["lanes"])

    def test_signal_join_includes_boundary_excludes_passed(self):
        index = SemanticMapIndex(self.meta, self.layers)
        self.assertEqual(index.signals_ahead(90., 0.)[0]["stoplines"], [self.stop])
        self.assertEqual(index.signals_ahead(90.01), [])
        self.assertEqual(index.signals_ahead(0., 89.), [])

    def test_station_restricts_partial_link_and_rejects_provisional(self):
        index = SemanticMapIndex(self.meta, self.layers)
        self.assertEqual([m.id for m in index.active_missions("link", 30.)], ["speed", "obstacle", "activation"])
        self.assertEqual([m.id for m in index.active_missions("link", 50.)], ["speed", "activation"])
        self.assertEqual(index.active_missions("unknown", 30.), [])

    def test_station_rules_and_unbounded_transition_query(self):
        index = SemanticMapIndex(self.meta, self.layers)
        self.assertEqual([m.id for m in index.missions_at_station(30)], ["speed", "obstacle", "activation"])
        self.assertIsNone(index.speed_at_station(100))
        self.assertEqual(index.speed_at_station(99)["effective_limit_kph"], 60)
        boundaries = index.mission_boundaries(80, ahead_m=300, behind_m=60)
        self.assertTrue(any(b["boundary_type"] == "point" and b["mission"].id == "signal" for b in boundaries))
        self.assertTrue(any(b["boundary_type"] == "exit" and b["distance_m"] == -30 for b in boundaries))
        self.assertFalse(any(b["mission"].id == "provisional" for b in boundaries))

    def test_closed_loop_boundaries_and_signals_cross_seam(self):
        index = SemanticMapIndex(self.meta, self.layers)
        index.signals.append(mission("near", 98, 98, {"boundary_only": True, "traffic_light_id": "TL-2"}, signal=True))
        index.payloads["near"] = {"traffic_light_id": "TL-2"}
        self.assertEqual([s["distance_m"] for s in index.signals_ahead(95, 100, route_length_m=100)], [3, 95])
        boundaries = index.mission_boundaries(95, ahead_m=40, behind_m=10, route_length_m=100)
        self.assertEqual([b["distance_m"] for b in boundaries], sorted(b["distance_m"] for b in boundaries))
        entering = next(b for b in boundaries if b["mission"].id == "obstacle" and b["boundary_type"] == "enter")
        self.assertEqual(entering["distance_m"], 30)
        self.assertEqual(entering["route_station_m"], 25)
        start = index.mission_boundaries(5, ahead_m=20, behind_m=10, route_length_m=100)
        self.assertTrue(any(b["boundary_type"] == "exit" and b["distance_m"] == -5 for b in start))

    def test_identity_drift_is_rejected(self):
        self.layers["markings"].geometry_sha256 = "other"
        with self.assertRaises(ValueError):
            SemanticMapIndex(self.meta, self.layers)
        self.layers["missions"].build_id = "older"
        with self.assertRaises(ValueError):
            SemanticMapIndex(self.meta, {"missions": self.layers["missions"]})

    def test_spatial_returns_original_objects_in_selected_layer(self):
        index = SemanticMapIndex(self.meta, self.layers)
        self.assertEqual(index.spatial((0, 0), 0, layers=["markings"]), {"markings": [self.stop]})
        self.assertEqual(index.spatial((0, 0), 3.14159265359, layers=["markings"]), {"markings": []})

    def test_unknown_source_speed_and_invalid_station_are_rejected(self):
        self.speed.rule_payload_json = json.dumps({"route_speed_limits": [{
            "link_id": "link", "effective_limit_kph": float("nan")}]})
        with self.assertRaises(ValueError):
            SemanticMapIndex(self.meta, self.layers)
        index = SemanticMapIndex(self.meta, {})
        with self.assertRaises(ValueError):
            index.signals_ahead(float("nan"))


if __name__ == "__main__":
    unittest.main()
