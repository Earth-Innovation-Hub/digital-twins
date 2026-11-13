"""Unit tests for VLM JSON → SDF (no network)."""
import tempfile
import unittest
from pathlib import Path

from twin_builder import sdf_props
from twin_builder.catalog import MAX_PROPS


class SdfPropsTests(unittest.TestCase):
    def test_validate_caps_and_filters(self):
        raw = (
            [{"class": "building", "lat": 33.4, "lon": -111.9, "size_m": [10, 8, 6]}]
            + [{"class": "tree", "lat": 33.4, "lon": -111.9}] * (MAX_PROPS + 5)
            + [{"class": "spaceship", "lat": 0, "lon": 0}]
            + [{"class": "car", "lat": 200, "lon": 0}]
        )
        out = sdf_props.validate_props(raw)
        self.assertEqual(len(out), MAX_PROPS)
        self.assertEqual(out[0]["class"], "building")
        self.assertTrue(all(p["class"] in ("building", "tree") for p in out))

    def test_write_fragment(self):
        props = [
            {"class": "car", "lat": 33.4001, "lon": -111.9001, "yaw_deg": 90, "size_m": [4, 2, 1.5]},
            {"class": "water", "lat": 33.4, "lon": -111.9},
        ]
        xml, clean = sdf_props.props_xml_fragment(props, 33.4, -111.9)
        self.assertEqual(len(clean), 2)
        self.assertIn("prop_car_0", xml)
        self.assertIn("<box>", xml)
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "props.sdf"
            written = sdf_props.write_props_include(path, props, 33.4, -111.9)
            self.assertEqual(len(written), 2)
            self.assertTrue(path.is_file())


if __name__ == "__main__":
    unittest.main()
