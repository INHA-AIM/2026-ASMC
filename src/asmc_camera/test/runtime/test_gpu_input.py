import unittest
import numpy as np
from asmc_camera.runtime.gpu_input import RawGpuCache, PreprocessSpec
from asmc_camera.runtime.shared_input import CacheBusy
from asmc_camera.camera.geometry import compute_letterbox_geometry


class FakeRaw:
    def __init__(self, image, device):
        self.available = True
        self.acquires = self.releases = 0
    def acquire(self):
        self.acquires += 1
    def release(self):
        self.releases += 1
    def idle(self):
        return self.available
    def close(self):
        pass


class GpuInputTest(unittest.TestCase):
    def test_two_consumers_upload_once_and_do_not_share_across_camera_epoch(self):
        cache = RawGpuCache(factory=FakeRaw)
        image = np.zeros((4, 4, 3), np.uint8)
        with cache.lease('front', (0, 123), image) as a:
            with cache.lease('front', (0, 123), image) as b:
                self.assertIs(a, b)
        self.assertEqual((cache.uploads, cache.hits, cache.bytes), (1, 1, 48))
        with cache.lease('left', (0, 123), image):
            pass
        with cache.lease('front', (1, 123), image):
            pass
        self.assertEqual(cache.uploads, 3)
        self.assertEqual((a.acquires, a.releases), (2, 2))

    def test_gpu_completion_and_python_refs_both_prevent_eviction(self):
        cache = RawGpuCache(factory=FakeRaw)
        image = np.zeros((4, 4, 3), np.uint8)
        with cache.lease('front', 1, image) as a:
            a.available = False
        with cache.lease('front', 2, image):
            with self.assertRaises(CacheBusy):
                with cache.lease('front', 3, image):
                    pass
        a.available = True
        with cache.lease('front', 3, image):
            pass
        self.assertEqual(len(cache.rows['front']), 2)

    def test_exception_returns_lease_and_close_rejects_new_input(self):
        cache = RawGpuCache(factory=FakeRaw)
        with self.assertRaises(ValueError):
            with cache.lease('front', 1, np.zeros((2, 2, 3), np.uint8)):
                raise ValueError('inference failed')
        self.assertEqual(cache.rows['front'][0]['refs'], 0)
        cache.close()
        with self.assertRaises(RuntimeError):
            with cache.lease('front', 2, np.zeros((2, 2, 3), np.uint8)):
                pass

    def test_cuda_release_error_still_returns_python_lease(self):
        class FailedRelease(FakeRaw):
            def release(self):
                raise RuntimeError('CUDA context failed')
        cache=RawGpuCache(factory=FailedRelease)
        with self.assertRaises(RuntimeError):
            with cache.lease('front',1,np.zeros((2,2,3),np.uint8)):
                pass
        self.assertEqual(cache.rows['front'][0]['refs'],0)

    def test_spec_separates_mask_precision_interpolation_and_geometry(self):
        baseline = PreprocessSpec((480, 640), 'area')
        self.assertNotEqual(baseline, PreprocessSpec((480, 640), 'area', 'hood-sha'))
        self.assertNotEqual(baseline, PreprocessSpec((480, 640), 'area', dtype='float16'))
        self.assertNotEqual(baseline, PreprocessSpec((480, 640), 'linear'))
        self.assertEqual(compute_letterbox_geometry((480, 640), baseline.target_hw).pad_ltrb, (0, 0, 0, 0))

    def test_linear_rounding_matches_opencv_on_uint8_and_color_channels(self):
        import torch
        import cv2
        from asmc_camera.runtime.gpu_input import _linear_uint8
        image = np.random.RandomState(20261011).randint(0,256,(72,128,3),dtype=np.uint8)
        actual = _linear_uint8(torch.from_numpy(image),(54,96)).numpy().astype(np.uint8)
        expected = cv2.resize(image,(96,54),interpolation=cv2.INTER_LINEAR)
        np.testing.assert_array_equal(actual,expected)


if __name__ == '__main__':
    unittest.main()
