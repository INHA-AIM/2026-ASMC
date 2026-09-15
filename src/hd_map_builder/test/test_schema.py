import unittest

from hd_map_builder.schema import (
    SCHEMA_VERSION,
    make_candidate_feature,
    validate_canonical_document,
)


def _document():
    feature = make_candidate_feature(
        feature_type="lane_boundary",
        canonical_id="kcity2025_hd_candidate_v1:lane_boundary:legacy:B1",
        points=[[0, 0, 0], [1, 0, 0]],
        attributes={"mgeo": {"lane_color": ["white"]}},
        source_id="B1",
        source_sha256="abc123",
        target_link_ids=["A2256"],
        validation={"tier": "A", "reviewed": False},
    )
    return {
        "schema_version": SCHEMA_VERSION,
        "map_id": "kcity2025_hd_candidate_v1",
        "frame": {
            "frame_id": "map",
            "crs": "+proj=utm +zone=52 +datum=WGS84 +units=m +no_defs",
            "local_origin_in_global": [302595.0, 4124145.0, 0.0],
        },
        "sources": {},
        "features": [feature],
    }


class SchemaTest(unittest.TestCase):
    def test_candidate_schema_keeps_legacy_id_only_in_provenance(self):
        document = _document()
        validate_canonical_document(document)
        feature = document["features"][0]
        self.assertTrue(feature["id"].startswith(document["map_id"] + ":"))
        self.assertEqual(feature["provenance"]["source_id"], "B1")
        self.assertEqual(feature["topology"]["current_link_ids"], ["A2256"])
        self.assertEqual(feature["status"], "candidate")

    def test_bare_legacy_id_cannot_be_canonical_id(self):
        document = _document()
        document["features"][0]["id"] = "B1"
        with self.assertRaisesRegex(ValueError, "canonical map namespace"):
            validate_canonical_document(document)


if __name__ == "__main__":
    unittest.main()
