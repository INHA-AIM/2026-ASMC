import io
from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scen_runner.records.control_binding import Reader


class TrajectorySchemaTest(unittest.TestCase):
    def trajectory(self):
        from asmc_msgs.msg import Trajectory,TrajectoryPoint
        path=Trajectory();path.header.frame_id='map';path.valid=True;path.dt=.1
        path.execution_policy=path.POLICY_STATIC_CONTACT_PROGRESS
        point=TrajectoryPoint();point.x=12.5;point.velocity=.5;path.points=[point]
        return path

    @staticmethod
    def serialized(message):
        stream=io.BytesIO();message.serialize(stream);return stream.getvalue()

    def test_current_schema_preserves_policy_and_exact_packet_boundary(self):
        path=self.trajectory();raw=self.serialized(path);reader=Reader(raw+b'\x01')
        decoded=reader.message(type(path))
        self.assertEqual(decoded.execution_policy,path.execution_policy)
        self.assertEqual(decoded.points[0].x,12.5)
        self.assertEqual(reader.at,len(raw));self.assertTrue(reader.value('?'))

    def test_old_trajectory_without_policy_is_read_as_normal_and_keeps_boundary(self):
        path=self.trajectory();raw=self.serialized(path)
        frame_width=struct.unpack_from('<I',raw,12)[0];policy_offset=16+frame_width+2
        old=raw[:policy_offset]+raw[policy_offset+1:]
        reader=Reader(old+b'\x01');decoded=reader.message(type(path),legacy_trajectory=True)
        self.assertEqual(decoded.execution_policy,path.POLICY_NORMAL)
        self.assertEqual(decoded.points[0].x,12.5)
        self.assertEqual(reader.at,len(old));self.assertTrue(reader.value('?'))


if __name__=='__main__':unittest.main()
