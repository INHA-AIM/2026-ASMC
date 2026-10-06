import json
from types import SimpleNamespace as Obj
import unittest

from asmc_hd_map.semantic_query import SemanticMapIndex


def layer(**fields):
    return Obj(build_id="build", map_id="map", geometry_sha256="sha", complete=True,
               header=Obj(frame_id="map"), **fields)


def mission(id, start, end, payload, provisional=False, signal=False):
    return Obj(id=id, link_ids=["link"], route_station_start_m=start, route_station_end_m=end,
               status=1 if provisional else 2, STATUS_PROVISIONAL=1,
               mission_type=3 if signal else 4, TYPE_TRAFFIC_LIGHT=3,
               rule_payload_json=json.dumps(payload))


class SemanticQueryTest(unittest.TestCase):
    def setUp(self):
        self.meta = layer(operational=True)
        self.speed = mission("speed", 0, 100, {"route_speed_limits": [
            {"link_id": "link", "effective_limit_kph": 60., "mgeo_max_speed_kph": 70.,
             "competition_cap_kph": 60.}
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
