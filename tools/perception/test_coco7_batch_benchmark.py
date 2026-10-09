import unittest
from types import SimpleNamespace
from benchmark_coco7_batches import planned_pixels, stage_totals


class BatchEvidenceTests(unittest.TestCase):
    def test_padding_cost(self):
        self.assertEqual(planned_pixels(640,'2plus1'),860160)
        self.assertEqual(planned_pixels(640,'batch3'),921600)
        self.assertEqual(planned_pixels(1280,'batch3'),2826240)
        self.assertEqual(planned_pixels(960,'sequential'),planned_pixels(960,'2plus1'))

    def test_shared_timer_is_counted_once(self):
        result={v:SimpleNamespace(timings=dict(inference_ms=7,mask_transfer_ms=3,batch_size=3))
                for v in ['front','left','right']}
        self.assertEqual(stage_totals(result,[['front','left','right']]),dict(inference_ms=7,mask_transfer_ms=3))
        result['right'].timings['mask_transfer_ms']=3.001
        self.assertEqual(stage_totals(result,[['front','left','right']])['mask_transfer_ms'],3.001)
        result['right'].timings['inference_ms']=8
        with self.assertRaises(ValueError):stage_totals(result,[['front','left','right']])

    def test_wrong_batch_size_is_not_accepted(self):
        result={'front':SimpleNamespace(timings=dict(inference_ms=7,mask_transfer_ms=3,batch_size=3))}
        with self.assertRaises(ValueError):stage_totals(result,[['front']])


if __name__=='__main__':unittest.main()
