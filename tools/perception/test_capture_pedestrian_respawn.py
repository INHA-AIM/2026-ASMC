import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import rotate_capture_with_pedestrian_respawn as target


class RespawnTest(unittest.TestCase):
    def test_new_identity_is_not_approved_and_original_is_unchanged(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);src=root/'original.jsonl';out=root/'rotation'
            base=dict(created=True,removed=False,geometry_gate=True,placement_approved=True,slot=1,api_name='Deer1',
                      target_class='deer',actor_type='pedestrian',yaw_deg=300,location={'x':1,'y':2,'z':28},
                      id='old-deer',native_ids=['7'],placement_review={'approved':True},route_surface_z=28)
            obs=dict(base,slot=2,api_name='WoodBox',target_class='box',actor_type='obstacle',id='old-box')
            original=''.join(json.dumps(r)+'\n' for r in [base,obs]);src.write_text(original)
            def fake_run(cmd,check):
                tool=Path(cmd[1]).name
                if tool=='rotate_capture_route_layout.py':
                    Path(cmd[cmd.index('--output')+1]).write_text(Path(cmd[cmd.index('--ledger')+1]).read_text())
                elif tool=='capture_route_layout.py' and cmd[2]=='create':
                    p=Path(cmd[cmd.index('--ledger')+1]);rows=[json.loads(l) for l in p.read_text().splitlines()]
                    for r in rows:r.update(id='new-deer',native_ids=['99'],created=True)
                    p.write_text(''.join(json.dumps(r)+'\n' for r in rows))
            with patch('sys.argv',['tool','--ledger',str(src),'--output-dir',str(out)]),patch.object(target.subprocess,'run',side_effect=fake_run):
                target.main()
            self.assertEqual(src.read_text(),original)
            new=[json.loads(l) for l in (out/'combined.jsonl').read_text().splitlines()][0]
            self.assertEqual(new['yaw_deg'],0)
            self.assertEqual(new['id'],'new-deer')
            self.assertFalse(new['placement_approved'])
            self.assertNotIn('placement_review',new)
            self.assertEqual(new['recreated_from_actor_id'],'old-deer')

    def test_unreviewed_obstacle_stops_before_any_mutation(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);src=root/'original.jsonl'
            src.write_text(json.dumps(dict(created=True,geometry_gate=True,placement_approved=False,
                                         actor_type='obstacle',slot=1))+'\n')
            with patch('sys.argv',['tool','--ledger',str(src),'--output-dir',str(root/'out')]),patch.object(target.subprocess,'run') as run:
                with self.assertRaisesRegex(ValueError,'RGB approval'):target.main()
                run.assert_not_called()

if __name__=='__main__':unittest.main()
