"""Recording preserves RPC return/error, timeout, and request identity."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scen_runner.config import load_config
from scen_runner.morai.episode_link import SceneLink
from scen_runner.morai.rpc_observation import ObservedStub,observe_link


class RpcObservationTest(unittest.TestCase):
    def setUp(self):
        SceneLink(load_config())  # Make SDK generated messages importable.
        from proto.morai.common.object_info_pb2 import ObjectInfo
        from proto.morai.common.result_pb2 import Result
        self.request=ObjectInfo(object_type=1,client_key='our_scope');self.request.id.value='24'
        self.Result=Result

    def test_records_zero_status_and_forwards_original_request_timeout_and_reply(self):
        original=Mock();reply=self.Result(status=0,description='NOT_CREATE_OBJECT')
        original.DestroyActor.return_value=reply
        records=[];stub=ObservedStub(original,records.append,'actor')
        self.assertIs(stub.DestroyActor(self.request,timeout=5),reply)
        original.DestroyActor.assert_called_once_with(self.request,timeout=5)
        self.assertEqual(records[0]['request']['id']['value'],'24')
        self.assertEqual(records[0]['request']['client_key'],'our_scope')
        self.assertEqual(records[0]['response']['status'],0)
        self.assertEqual(records[0]['response']['description'],'NOT_CREATE_OBJECT')
        self.assertEqual(records[0]['timeout_s'],5)

    def test_rpc_exception_is_logged_and_same_exception_is_raised(self):
        original=Mock();error=RuntimeError('deadline')
        original.DestroyActor.side_effect=error
        records=[];stub=ObservedStub(original,records.append,'actor')
        with self.assertRaises(RuntimeError) as raised:stub.DestroyActor(self.request,timeout=5)
        self.assertIs(raised.exception,error)
        self.assertEqual(records[0]['rpc_error'],'deadline')
        self.assertNotIn('response',records[0])

    def test_reads_pass_through_and_installation_does_not_duplicate_records(self):
        original=Mock();scenario=Mock();link=Mock()
        adapter=S(_actor_stub=original,_scenario_stub=scenario);link._require.return_value=adapter
        records=[];observe_link(link,records.append);observe_link(link,records.append)
        adapter._actor_stub.GetAllActorsState(self.request,timeout=3)
        original.GetAllActorsState.assert_called_once_with(self.request,timeout=3)
        self.assertFalse(records)
        original.DestroyActor.return_value=self.Result(status=1)
        adapter._actor_stub.DestroyActor(self.request,timeout=5)
        self.assertEqual(len(records),1)


if __name__=='__main__':unittest.main()
