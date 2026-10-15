"""Cleanup must retire registered aliases before forgetting their ownership."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock,patch

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scen_runner.runtime.episode_actors import Section


def body(aid,kind=3):
    return S(actor_info=S(id=S(value=aid),object_type=kind))


class ActorCleanupTest(unittest.TestCase):
    def section(self):
        link=Mock(actor_scopes={});link.states.return_value=[]
        with patch('scen_runner.runtime.episode_actors.SceneLink',return_value=link):
            section=Section('static',42,'own-scope',{},lambda:False,lambda *_:None)
        return section,link

    def test_scoped_static_is_deleted_even_if_its_numeric_id_is_missing(self):
        section,link=self.section();section.register('own-alias',3,'Bollard01',native='1')
        link.states.return_value=[]
        link.scoped_statics.side_effect=[[body('own-alias')],[]]
        section.cleanup()
        link.destroy.assert_called_once_with('own-alias',object_type=3)
        self.assertFalse(section.actors);link.disconnect.assert_called_once()
        self.assertEqual(link.states.call_count,2)

    def test_acknowledged_deletion_does_not_forget_a_still_visible_alias(self):
        section,link=self.section();actor=section.register('own-alias',3,'Bollard01',native='1')
        link.destroy.return_value=True;link.scoped_statics.return_value=[body('own-alias')]
        with patch('scen_runner.runtime.episode_actors.time.monotonic',side_effect=[0.,2.]):
            with self.assertRaisesRegex(RuntimeError,'객체 삭제 미확인'):section.cleanup()
        self.assertEqual(section.actors,[actor]);link.disconnect.assert_not_called()
        link.scoped_statics.return_value=[];section.cleanup()
        self.assertFalse(section.actors);link.disconnect.assert_called_once()

    def test_late_cache_update_is_verified_and_unrelated_aliases_are_preserved(self):
        section,link=self.section();section.register('own-alias',3,'Bollard01',native='1')
        other=body('other-alias')
        link.scoped_statics.side_effect=[[body('own-alias'),other],[body('own-alias'),other],[other]]
        with patch('scen_runner.runtime.episode_actors.time.sleep'):section.cleanup()
        link.destroy.assert_called_once_with('own-alias',object_type=3)
        self.assertFalse(section.actors)

    def test_global_fallback_and_reused_numeric_ids_do_not_delete_another_object(self):
        section,link=self.section();section.register('retired-alias',3,'Bollard01',native='1')
        # An unregistered client scope can return unrelated global numeric IDs.
        link.scoped_statics.return_value=[body('1'),body('foreign-alias')]
        link.states.return_value=[body('1')]
        section.cleanup()
        link.destroy.assert_not_called();self.assertFalse(section.actors)

    def test_alias_removal_does_not_confirm_deletion_while_native_body_remains(self):
        section,link=self.section();actor=section.register('own-alias',3,'Bollard01',native='1')
        link.scoped_statics.side_effect=[[body('own-alias')],[]]
        link.states.return_value=[body('1')];link.destroy.return_value=True
        with patch('scen_runner.runtime.episode_actors.time.monotonic',side_effect=[0.,2.]):
            with self.assertRaisesRegex(RuntimeError,'객체 삭제 미확인'):section.cleanup()
        self.assertEqual(section.actors,[actor]);link.disconnect.assert_not_called()

    def test_wrong_actor_type_at_a_reused_id_is_not_destroyed(self):
        section,link=self.section();section.register('1',1,'K5')
        link.states.return_value=[body('1',3)]
        section.cleanup()
        link.destroy.assert_not_called();self.assertFalse(section.actors)

    def test_partial_rpc_failure_retains_each_registration_scope_for_retry(self):
        section,link=self.section()
        section.register('1',1,'K5');section.register('2',1,'K5')
        link.actor_scopes={'1':'spawn-one','2':'spawn-two'}
        link.states.return_value=[body('1',1),body('2',1)]
        def destroy(aid,**kwargs):
            if aid=='2':raise ConnectionError('lost channel')
            link.actor_scopes.pop(aid);return True
        link.destroy.side_effect=destroy
        with self.assertRaises(ConnectionError):section.cleanup()
        self.assertEqual(link.actor_scopes,{'1':'spawn-one','2':'spawn-two'})
        self.assertEqual(len(section.actors),2);link.disconnect.assert_not_called()


if __name__=='__main__':unittest.main()
