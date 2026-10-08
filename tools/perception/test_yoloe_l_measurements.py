import unittest
from yoloe_l_measurements import view_gate, first_report
from b0_measurements import output_report


class GatesTest(unittest.TestCase):
    def fixture(self):
        rows=[dict(arrival=.025+i*.1,stamp_ns=i+1,valid=True,source_age_ms=50,wall_latency_ms=45,input_anchor='unique') for i in range(100)]
        return output_report(rows,0,10,expected_seconds=10)

    def test_ten_hz_is_feasible_but_not_twenty(self):
        r=view_gate(self.fixture());self.assertTrue(r['passed']);self.assertFalse(r['twenty_hz_met']);self.assertFalse(r['operational_18hz_met'])

    def test_missing_window_duplicate_invalid_and_latency_fail(self):
        for update in [dict(five_second_hz=[]),dict(complete=False),dict(duplicate_stamps=1),dict(valid_ratio=.99),
                       dict(reversed_stamps=1),dict(source_age_ms=dict(p95=151,p99=190)),dict(max_gap_sec=.251),dict(negative_source_age=1)]:
            r=self.fixture();r.update(update);self.assertFalse(view_gate(r)['passed'],update)

    def test_missing_gpu_or_missing_view_is_not_pass(self):
        row=dict(measurements={'yolo_front':self.fixture()},checks={'gpu_sample_available':False},failures=[])
        r=first_report(row);self.assertFalse(r['passed']);self.assertFalse(r['vram_headroom_target_met'])


if __name__=='__main__':unittest.main()
