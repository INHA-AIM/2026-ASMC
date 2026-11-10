import unittest
from localization_fixtures import NAMES,fixture_definition
from scen_runner.runtime.validation_episode import EpisodeRoute
from scen_runner.morai.signal_trial_coordinates import Coordinates

class FixtureTests(unittest.TestCase):
    def test_static_vehicle_assets_do_not_spawn_npcs(self):
        route=EpisodeRoute();coordinates=Coordinates([302595.,4124145.])
        self.assertNotIn('slow_lead',NAMES)
        for name,model in [('parked_vehicle','OBJ_Kia_K5'),('parked_truck','OBJ_Hyundai_HD65'),('angled_vehicle','OBJ_Kia_K5')]:
            record=fixture_definition(name,route,coordinates)['baseline'][0]['record']
            self.assertEqual(record['kind'],3);self.assertEqual(record['definition']['type'],'static')
            self.assertEqual(record['model'],model);self.assertEqual(record['target_kmh'],0)
    def test_crossings_have_opposite_routes(self):
        route=EpisodeRoute();coordinates=Coordinates([302595.,4124145.])
        def record(name):return fixture_definition(name,route,coordinates)['baseline'][0]['record']['definition']
        left,right=record('crossing_left'),record('crossing_right')
        self.assertEqual(left['start'],right['end']);self.assertEqual(left['end'],right['start'])
        self.assertGreater(left['cfg']['walk_speed_kmh'],0)

if __name__=='__main__':unittest.main()
