#!/usr/bin/env python3

import json
import struct
import tempfile
import unittest
from pathlib import Path

from data_collection.capture_sync import (
    CAMERAS,
    MODALITIES,
    STATE_SENSORS,
    CaptureSyncError,
    load_successful_records,
    preflight_run,
    select_records,
    sync_run,
)


def write_fake_png(path, height, width):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + b"\x00\x00\x00\x0dIHDR" + struct.pack(">II", width, height))


class CaptureSyncTest(unittest.TestCase):
    def test_partial_bbox_preserves_missing_views_without_empty_labels(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);sensor,data,capture=self.make_run(root)
            (sensor/'CAMERA_1/run_001_000000_BBox2D.txt').write_text('native GT\n')
            result=sync_run(sensor,data/'capture_runs',data/'datasets','run_001',profile='bbox3_partial',jobs=1)
            row=json.loads((data/'datasets/run_001/manifest.jsonl').read_text().splitlines()[0])
            self.assertEqual(row['missing_bbox_views'],['left','right'])
            self.assertNotIn('left',row['paths']['bbox'])
            self.assertEqual(result['bbox_missing_view_count'],2)

    def make_run(self, root, run_id="run_001"):
        sensor_root = root / "SensorData"
        data_root = root / "data"
        capture_dir = data_root / "capture_runs" / run_id
        capture_dir.mkdir(parents=True)
        record = {
            "sequence": 0,
            "custom_name": run_id + "_000000",
            "success": True,
            "morai_sim_time_ms": 123,
            "state": {"ego": {"heading_deg": 10.0}},
        }
        (capture_dir / "capture_manifest.jsonl").write_text(
            json.dumps(record) + "\n", encoding="utf-8"
        )
        (capture_dir / "meta.json").write_text("{}\n", encoding="utf-8")
        (capture_dir / "summary.json").write_text(
            json.dumps({"run_id": run_id, "succeeded": 1}) + "\n",
            encoding="utf-8",
        )
        for sensor_dir, _, hw in CAMERAS:
            for suffix, _ in MODALITIES:
                write_fake_png(
                    sensor_root / sensor_dir / (record["custom_name"] + "_" + suffix + ".png"),
                    hw[0],
                    hw[1],
                )
        for sensor_dir, _ in STATE_SENSORS:
            path = sensor_root / sensor_dir / (record["custom_name"] + ".txt")
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("state\n", encoding="utf-8")
        return sensor_root, data_root, capture_dir

    def test_preflight_and_sync_complete_frame(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            sensor_root, data_root, capture_dir = self.make_run(root)
            records = load_successful_records(capture_dir / "capture_manifest.jsonl")
            frames, _ = preflight_run(sensor_root, records)
            self.assertEqual(len(frames), 1)
            self.assertEqual(len(frames[0][1]), 18)

            result = sync_run(
                sensor_root,
                data_root / "capture_runs",
                data_root / "datasets",
                "run_001",
                jobs=1,
            )
            self.assertEqual(result["frame_count"], 1)
            self.assertEqual(result["file_count"], 18)
            run_root = data_root / "datasets" / "run_001"
            self.assertTrue((run_root / "_SUCCESS").is_file())
            self.assertTrue((run_root / "frames/intensity/front/000000.png").is_file())
            self.assertTrue((run_root / "state/gps/000000.txt").is_file())

    def test_preflight_rejects_missing_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            sensor_root, _, capture_dir = self.make_run(root)
            missing = sensor_root / "CAMERA_1/run_001_000000_Semantic.png"
            missing.unlink()
            records = load_successful_records(capture_dir / "capture_manifest.jsonl")
            with self.assertRaises(CaptureSyncError):
                preflight_run(sensor_root, records)

    def test_bbox_profile_requires_native_boxes_and_optional_extras(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            sensor_root,data_root,capture_dir=self.make_run(root)
            records=load_successful_records(capture_dir/'capture_manifest.jsonl')
            with self.assertRaises(CaptureSyncError):
                preflight_run(sensor_root,records,profile='bbox3')
            for sensor,view,hw in CAMERAS[:3]:
                (sensor_root/sensor/'run_001_000000_BBox2D.txt').write_text('')
                (sensor_root/sensor/'run_001_000000_Depth.png').unlink()
            for path in (sensor_root/'CAMERA_4').iterdir():path.unlink()
            for sensor,name in STATE_SENSORS:
                (sensor_root/sensor/'run_001_000000.txt').unlink()
            result=sync_run(sensor_root,data_root/'capture_runs',data_root/'datasets','run_001',profile='bbox3',jobs=1)
            self.assertEqual(result['file_count'],12)
            dest=data_root/'datasets/run_001'
            self.assertTrue((dest/'frames/bbox/front/000000.txt').is_file())
            # Same-size corruption must not survive a resumed copy.
            image=dest/'frames/intensity/front/000000.png'
            image.write_bytes(b'x'*image.stat().st_size)
            sync_run(sensor_root,data_root/'capture_runs',data_root/'datasets','run_001',profile='bbox3',jobs=1)
            self.assertEqual(image.read_bytes(),(sensor_root/'CAMERA_1/run_001_000000_Intensity.png').read_bytes())

    def test_explicit_exclusion_keeps_lineage_and_skips_incomplete_capture(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            sensor_root, data_root, capture_dir = self.make_run(root)
            records = load_successful_records(capture_dir / "capture_manifest.jsonl")
            kept, excluded = select_records(records, excluded_sequences=[])
            self.assertEqual(len(kept), 1)
            self.assertEqual(excluded, [])
            with self.assertRaises(CaptureSyncError):
                select_records(records, excluded_sequences=[999])

            # A second valid record remains after excluding the incomplete first one.
            second = dict(records[0])
            second["sequence"] = 1
            second["custom_name"] = "run_001_000001"
            with (capture_dir / "capture_manifest.jsonl").open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(second) + "\n")
            (capture_dir / "summary.json").write_text(
                json.dumps({"run_id": "run_001", "succeeded": 2}) + "\n",
                encoding="utf-8",
            )
            for sensor_dir, _, hw in CAMERAS:
                for suffix, _ in MODALITIES:
                    write_fake_png(
                        sensor_root / sensor_dir / (second["custom_name"] + "_" + suffix + ".png"),
                        hw[0],
                        hw[1],
                    )
            for sensor_dir, _ in STATE_SENSORS:
                path = sensor_root / sensor_dir / (second["custom_name"] + ".txt")
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("state\n", encoding="utf-8")
            (sensor_root / "CAMERA_1/run_001_000000_Semantic.png").unlink()

            result = sync_run(
                sensor_root,
                data_root / "capture_runs",
                data_root / "datasets",
                "run_001",
                jobs=1,
                excluded_sequences=[0],
            )
            self.assertEqual(result["frame_count"], 1)
            self.assertEqual(result["source_successful_frame_count"], 2)
            self.assertEqual(result["excluded_sequences"], [0])
            run_root = data_root / "datasets" / "run_001"
            dataset = json.loads((run_root / "dataset.json").read_text(encoding="utf-8"))
            self.assertEqual(dataset["excluded_frame_count"], 1)
            self.assertEqual(dataset["excluded_captures"][0]["sequence"], 0)
            self.assertFalse((run_root / "frames/intensity/front/000000.png").exists())
            self.assertTrue((run_root / "frames/intensity/front/000001.png").exists())


if __name__ == "__main__":
    unittest.main()
