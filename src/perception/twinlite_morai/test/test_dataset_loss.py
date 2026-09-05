import json
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np
import torch
from torch.utils.data import DataLoader

from twinlite_morai.dataset import MoraiTwinLiteDataset
from twinlite_morai.losses import MaskedTwinLiteLoss, adapt_twinlite_outputs
from twinlite_morai.training import twinlite_training_step


def write_png(path, image):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), image):
        raise RuntimeError("failed to write test PNG")


class TinyTwin(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.drivable = torch.nn.Conv2d(3, 2, 1)
        self.lane = torch.nn.Conv2d(3, 2, 1)

    def forward(self, image):
        return self.drivable(image), self.lane(image)


class DatasetAndLossTest(unittest.TestCase):
    def make_dataset(self, root):
        data_root = root / "data"
        version_root = data_root / "dataset_versions/twinlite_morai_v1"
        run_root = data_root / "datasets/run_001"
        image_rel = "datasets/run_001/frames/intensity/front/000000.png"
        image = np.zeros((6, 10, 4), dtype=np.uint8)
        image[:, :, :3] = (10, 20, 30)
        image[:, :, 3] = 255
        write_png(data_root / image_rel, image)

        target_hw = [8, 10]
        target_paths = {}
        for name in ("lane", "drivable"):
            relative = (
                f"datasets/run_001/derived/twinlite_384x640_v2/"
                f"{name}_masks/front/000000.png"
            )
            mask = np.zeros(target_hw, dtype=np.uint8)
            mask[3:, 4:6] = 1
            write_png(data_root / relative, mask)
            target_paths[name] = relative
        valid_rel = (
            "datasets/run_001/derived/twinlite_384x640_v2/"
            "valid_masks/front/000000.png"
        )
        valid = np.zeros(target_hw, dtype=np.uint8)
        valid[1:7] = 1
        write_png(data_root / valid_rel, valid)
        drivable_valid_rel = (
            "datasets/run_001/derived/twinlite_384x640_v2/"
            "valid_masks/drivable/front/000000.png"
        )
        drivable_valid = valid.copy()
        drivable_valid[3:5, 4:6] = 0
        write_png(data_root / drivable_valid_rel, drivable_valid)

        geometry = {
            "original_hw": [6, 10],
            "target_hw": target_hw,
            "resized_hw": [6, 10],
            "pad_ltrb": [0, 1, 0, 1],
            "scale": 1.0,
            "scale_xy": [1.0, 1.0],
            "valid_roi_xyxy": [0, 1, 10, 7],
        }
        row = {
            "schema_version": "morai-perception-sample-1.0.0",
            "dataset_version": "twinlite_morai_v1",
            "split": "train",
            "sample_id": "run_001/front/000000",
            "run_id": "run_001",
            "frame_id": 0,
            "view": "front",
            "image": image_rel,
            "targets": target_paths,
            "valid_mask": valid_rel,
            "valid_masks": {
                "lane": valid_rel,
                "drivable": drivable_valid_rel,
            },
            "geometry": geometry,
            "task_valid": {"lane": True, "drivable": True},
        }
        version_root.mkdir(parents=True)
        (version_root / "train.jsonl").write_text(
            json.dumps(row) + "\n", encoding="utf-8"
        )
        (version_root / "dataset.json").write_text(
            json.dumps(
                {
                    "schema_version": "morai-perception-dataset-1.0.0",
                    "target_hw": target_hw,
                    "training_heads": ["lane", "drivable"],
                }
            )
            + "\n",
            encoding="utf-8",
        )
        (version_root / "_SUCCESS").write_text("{}\n", encoding="utf-8")
        return data_root

    def test_dataset_applies_geometry_and_returns_named_targets(self):
        with tempfile.TemporaryDirectory() as tmp:
            dataset = MoraiTwinLiteDataset(self.make_dataset(Path(tmp)), split="train")
            sample = dataset[0]
            self.assertEqual(tuple(sample["image"].shape), (3, 8, 10))
            self.assertEqual(sample["image"].dtype, torch.float32)
            self.assertTrue(torch.allclose(sample["image"][:, 0], torch.full((3, 10), 114 / 255.0)))
            self.assertEqual(tuple(sample["targets"]["lane"].shape), (8, 10))
            self.assertEqual(sample["targets"]["lane"].dtype, torch.int64)
            self.assertEqual(sample["valid_mask"].dtype, torch.bool)
            self.assertFalse(bool(sample["valid_mask"][0, 0]))
            self.assertTrue(bool(sample["valid_masks"]["lane"][3, 4]))
            self.assertFalse(bool(sample["valid_masks"]["drivable"][3, 4]))
            self.assertEqual(sample["meta"]["view"], "front")

    def test_tuple_output_order_matches_official_twinlite(self):
        drivable = torch.randn(1, 2, 4, 6)
        lane = torch.randn(1, 2, 4, 6)
        outputs = adapt_twinlite_outputs((drivable, lane))
        self.assertIs(outputs["drivable"], drivable)
        self.assertIs(outputs["lane"], lane)

    def test_invalid_padding_has_zero_gradient(self):
        criterion = MaskedTwinLiteLoss()
        drivable = torch.randn(1, 2, 4, 6, requires_grad=True)
        lane = torch.randn(1, 2, 4, 6, requires_grad=True)
        targets = {
            "drivable": torch.zeros(1, 4, 6, dtype=torch.long),
            "lane": torch.zeros(1, 4, 6, dtype=torch.long),
        }
        valid = torch.ones(1, 4, 6, dtype=torch.bool)
        valid[:, 0] = False
        losses = criterion((drivable, lane), targets, valid)
        losses["loss"].backward()
        self.assertTrue(torch.isfinite(losses["loss"]))
        self.assertEqual(int(torch.count_nonzero(drivable.grad[:, :, 0])), 0)
        self.assertEqual(int(torch.count_nonzero(lane.grad[:, :, 0])), 0)
        self.assertGreater(int(torch.count_nonzero(drivable.grad[:, :, 1:])), 0)

    def test_real_batch_contract_reaches_forward_and_backward(self):
        with tempfile.TemporaryDirectory() as tmp:
            dataset = MoraiTwinLiteDataset(self.make_dataset(Path(tmp)), split="train")
            batch = next(iter(DataLoader(dataset, batch_size=1)))
            model = TinyTwin()
            _, losses = twinlite_training_step(model, batch, MaskedTwinLiteLoss())
            losses["loss"].backward()
            self.assertTrue(torch.isfinite(losses["loss"]))
            self.assertIsNotNone(model.drivable.weight.grad)
            self.assertIsNotNone(model.lane.weight.grad)

    def test_head_specific_valid_masks_gate_each_heads_gradient(self):
        criterion = MaskedTwinLiteLoss()
        drivable = torch.randn(1, 2, 4, 6, requires_grad=True)
        lane = torch.randn(1, 2, 4, 6, requires_grad=True)
        targets = {
            "drivable": torch.zeros(1, 4, 6, dtype=torch.long),
            "lane": torch.zeros(1, 4, 6, dtype=torch.long),
        }
        valid = {
            "drivable": torch.ones(1, 4, 6, dtype=torch.bool),
            "lane": torch.ones(1, 4, 6, dtype=torch.bool),
        }
        valid["drivable"][:, :, :2] = False
        valid["lane"][:, :, 4:] = False
        losses = criterion((drivable, lane), targets, valid)
        losses["loss"].backward()
        self.assertEqual(int(torch.count_nonzero(drivable.grad[:, :, :, :2])), 0)
        self.assertGreater(int(torch.count_nonzero(drivable.grad[:, :, :, 4:])), 0)
        self.assertEqual(int(torch.count_nonzero(lane.grad[:, :, :, 4:])), 0)
        self.assertGreater(int(torch.count_nonzero(lane.grad[:, :, :, :2])), 0)


if __name__ == "__main__":
    unittest.main()
