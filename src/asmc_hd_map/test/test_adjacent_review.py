"""Approved adjacent scope, preserved route geometry and actual paint joins."""
import importlib.util
import hashlib
import json
from pathlib import Path
import unittest
import tempfile
import numpy as np
from asmc_hd_map.runtime_bundle import geometry_sha256
from asmc_hd_map.map_overlay import feature_paint_segments,SegmentLayer

ROOT=Path(__file__).resolve().parents[3]
PKG=ROOT/'src/asmc_hd_map'

class AdjacentReviewTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.map=json.loads((PKG/'reports/kcity2025_route_curation_20260917/route_curated_candidate_map.json').read_text())
        cls.new=[f for f in cls.map['features'] if f['attributes'].get('reviewed_adjacent_group_id')]

    def test_all_preexisting_route_geometry_is_unchanged(self):
        original=[f for f in self.map['features'] if not f['attributes'].get('reviewed_adjacent_group_id')]
        self.assertEqual(len(original),128)
        self.assertEqual(geometry_sha256({'features':original}),
                         '01ec671fe402baf339f8f6bcfb83a97213d560f7311961c5f072a47d654bcfbf')

    def test_only_three_reviewed_links_receive_six_markings(self):
        self.assertEqual(len(self.new),6)
        ids={v for f in self.new for v in f['topology']['current_link_ids']}
        self.assertEqual(ids,{'A2256W000029','A2256W000175','A2256W000406'})
        inferred=[f for f in self.new if f['validation']['inferred_missing_paint']]
        self.assertEqual(len(inferred),2)
        self.assertEqual(inferred[0]['topology']['current_link_ids'],['A2256W000029'])
        self.assertIn('nominal_width_offset',inferred[0]['provenance']['transform'])

    def test_dash_endpoints_and_phase_join_the_existing_final_paint(self):
        for f in self.new:
            suffix=f['validation']['join_feature_suffix']
            if not suffix:continue
            target=next(q for q in self.map['features'] if q['id'].endswith(suffix))
            np.testing.assert_allclose(f['geometry']['coordinates'][-1],target['geometry']['coordinates'][0],atol=1e-9)
            a=f['attributes']
            if a['pattern']=='solid':continue
            self.assertEqual((a['dash_length_m'],a['dash_gap_m']),(3,3))
            p=np.asarray(f['geometry']['coordinates']);length=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1).sum()
            self.assertAlmostEqual((length+a['dash_phase_m'])%6,0,places=8)

    def test_a029_merge_is_solid_and_continuous_with_broken_predecessor(self):
        broken=next(f for f in self.new if f['id'].endswith('a029_left_missing_broken'))
        solid=next(f for f in self.new if f['id'].endswith('a029_left_missing_broken:merge_solid'))
        self.assertEqual(solid['attributes']['pattern'],'solid')
        self.assertEqual(solid['validation']['pattern_reference_ids'],['B219BS011115','B219BS011118'])
        np.testing.assert_allclose(solid['geometry']['coordinates'][-1],broken['geometry']['coordinates'][0],atol=1e-9)
        self.assertLess(solid['geometry']['coordinates'][-1][1],solid['geometry']['coordinates'][0][1])

    def test_three_lane_centers_are_in_surface_but_unreviewed_a010_is_not_selected(self):
        product=json.loads((PKG/'config/kcity2025_compact_drivable_map.json').read_text())
        self.assertEqual(product['selection']['explicit_lane_groups']['high_speed_preentry_029_175_406'],
                         ['A2256W000029','A2256W000175','A2256W000406'])
        self.assertNotIn('A2256W000010',product['selection']['selected_lane_link_ids'])
        with np.load(PKG/'config/kcity2025_compact_drivable_grid.npz') as data:
            o=data['origin_xy'];res=float(data['resolution_m']);grid=data['occupancy']
            for x in (65.824,69.385,72.369):
                i,j=np.rint((np.array([x,300])-o)/res).astype(int)
                self.assertEqual(grid[j,i],0)

    def test_added_roads_are_connected_to_existing_a411_a418(self):
        import cv2
        links={r['idx']:r for r in json.loads((ROOT/'src/asmc_planner/config/behavior/link_set.json').read_text())}
        with np.load(PKG/'config/kcity2025_compact_drivable_grid.npz') as data:
            _,labels=cv2.connectedComponents((data['occupancy']==0).astype(np.uint8))
            components=[]
            for suffix in ['029','175','406','411','418']:
                points=links['A2256W000'+suffix]['points'];xy=np.asarray(points[len(points)//2][:2])
                i,j=np.rint((xy-data['origin_xy'])/float(data['resolution_m'])).astype(int)
                components.append(int(labels[j,i]))
            self.assertGreater(components[0],0)
            self.assertEqual(len(set(components)),1)
            # Check the actual seam locally as well; a connection elsewhere on
            # the course must not mask a gap between the new and old surfaces.
            origin=data['origin_xy'];res=float(data['resolution_m'])
            lo=np.rint((np.array([60,248])-origin)/res).astype(int)
            hi=np.rint((np.array([78,265])-origin)/res).astype(int)
            _,seam=cv2.connectedComponents((data['occupancy'][lo[1]:hi[1]+1,lo[0]:hi[0]+1]==0).astype(np.uint8))
            joined=[]
            for suffix,y in [('029',262),('175',262),('406',262),('411',250),('418',250)]:
                points=np.asarray(links['A2256W000'+suffix]['points'])
                xy=points[np.argmin(abs(points[:,1]-y)),:2]
                i,j=np.rint((xy-origin)/res).astype(int)-lo
                joined.append(int(seam[j,i]))
            self.assertGreater(joined[0],0)
            self.assertEqual(len(set(joined)),1)

    def test_existing_roads_are_preserved_except_reviewed_tollgate_islands(self):
        product=json.loads((PKG/'config/kcity2025_compact_drivable_map.json').read_text())
        original=[a for a in product['drivable_areas'] if not a['id'].startswith('drivable:reviewed_h5:')]
        self.assertEqual(len(original),102)
        approved={'drivable:A2256W000153','drivable:A2256W000432',
                  'drivable:A2256W000451','drivable:corridor_fill:002'}
        self.assertEqual({a['id'] for a in original if a.get('excluded_island_ids')},approved)
        preserved=[a for a in original if a['id'] not in approved]
        self.assertEqual(len(preserved),98)
        # Digest computed from the pre-island product; keep every other area
        # byte-for-byte stable rather than re-pinning all 102 changed records.
        digest=hashlib.sha256(json.dumps(preserved,sort_keys=True,separators=(',',':')).encode()).hexdigest()
        self.assertEqual(digest,'f07bb27e9d94ef5d5c9c5a2dfdb86bcb7eaa55e77a2bedde5b2e2d16ea455cf2')

    def test_shared_paint_renderer_excludes_centerlines_and_retains_metric_gaps(self):
        f={'feature_type':'centerline','geometry':{'coordinates':[[0,0],[10,0]]}}
        self.assertEqual(len(feature_paint_segments(f)),0)
        f.update(feature_type='lane_marking',attributes={'pattern':'broken','dash_length_m':3,'dash_gap_m':3})
        np.testing.assert_allclose(feature_paint_segments(f),[[[0,0],[3,0]],[[6,0],[9,0]]])

    def test_rviz_comparison_does_not_publish_empty_line_lists(self):
        try:
            import rospy
        except ImportError:
            self.skipTest('Noetic ROS Python is required for marker publication regression')
        spec=importlib.util.spec_from_file_location('reviewed_overlay',PKG/'scripts/candidate_map_overlay_node.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        class Publisher:
            def get_num_connections(self):return 1
            def publish(self,msg):self.msg=msg
        p=Publisher();node=module.CandidateMapOverlayNode.__new__(module.CandidateMapOverlayNode)
        node.publishers={'test':p};node.radius_m=80
        empty=SegmentLayer(np.zeros((0,2,2)))
        node._publish_comparison_source('test',{('white','solid'):empty},empty,[0,0,0],rospy.Time(1),'test',(1,1,1,1))
        self.assertEqual(len(p.msg.markers),1)
        self.assertEqual(p.msg.markers[0].action,p.msg.markers[0].DELETEALL)

    def test_web_loads_only_sha_matched_physical_paint(self):
        try:
            import rospy
        except ImportError:
            self.skipTest('Noetic ROS Python is required for web loader regression')
        source=ROOT/'src/asmc_lidar/scripts/asmc_lidar_monitor_node.py'
        spec=importlib.util.spec_from_file_location('reviewed_web_overlay',source)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        args=[str(PKG/'config/kcity2025_compact_drivable_map.json'),
              str(ROOT/'src/asmc_planner/config/behavior/global_info.json'),
              str(ROOT/'src/asmc_lidar/config/lidar_pipeline.yaml')]
        lane=str(PKG/'reports/kcity2025_route_curation_20260917/route_curated_candidate_map.json')
        good=module.load_drivable_overlay(*args,lane)
        self.assertTrue(good['available']);self.assertTrue(good['lanes_available'])
        self.assertEqual(len(good['lane_markings']),134)
        self.assertTrue(all('segments' in row and 'centerline' not in row for row in good['lane_markings']))
        with tempfile.NamedTemporaryFile(mode='w') as f:
            f.write('{}');f.flush()
            bad=module.load_drivable_overlay(*args,f.name)
        self.assertTrue(bad['available']);self.assertFalse(bad['lanes_available'])
        self.assertEqual(bad['lane_markings'],[])
        self.assertIn('SHA',bad['lane_reason'])

if __name__=='__main__':unittest.main()
