"""Satellite ground image and the ground-plane splice. No network."""
import tempfile
import unittest
import urllib.error
from pathlib import Path
from unittest import mock

from twin_builder import satellite_ground as sat


GROUND = """<model name="ground_plane">
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>0.01 0.01</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient></material>
        </visual>
      </link>
      <pose>0 0 -2.0 0 0 0</pose>
    </model>"""


class SatelliteGroundTests(unittest.TestCase):
    def test_visual_is_the_patch_and_the_floor_moves_to_the_origin(self):
        out = sat.apply_ground(GROUND, 150.0, 120.0)
        self.assertIn("<size>150.00 120.00</size>", out)
        self.assertIn("<size>1 1</size>", out)
        self.assertIn("<albedo_map>/root/ground.jpg</albedo_map>", out)
        self.assertIn("<pose>0 0 0 0 0 0</pose>", out)
        self.assertNotIn("-2.0", out)

    def test_download_is_saved_when_the_response_is_a_jpeg(self):
        body = b"\xff\xd8" + b"\x00" * 2000
        resp = mock.Mock()
        resp.read.return_value = body
        resp.__enter__ = mock.Mock(return_value=resp)
        resp.__exit__ = mock.Mock(return_value=False)
        job = {
            "look_at": {"lat": 33.37, "lon": -111.94},
            "width_m": 150,
            "length_m": 150,
        }
        with tempfile.TemporaryDirectory() as td:
            dest = Path(td) / "ground.jpg"
            with mock.patch.object(sat.urllib.request, "urlopen", return_value=resp):
                got = sat.fetch_ground(job, dest)
            self.assertIsNotNone(got)
            self.assertTrue(dest.is_file())
            self.assertAlmostEqual(got["width_m"], 150.0, delta=5.0)
            self.assertAlmostEqual(got["length_m"], 150.0, delta=5.0)

    def test_a_small_patch_asks_for_a_wider_image_and_crops_it(self):
        box = {"west": -112.0, "south": 33.0, "east": -111.9998, "north": 33.0002}
        px, wider, frac = sat._request(box, 20.0, 20.0)
        self.assertGreaterEqual(px * sat.MIN_METRES_PER_PIXEL, 20.0)
        self.assertLess(frac, 1.0)
        self.assertLess(wider["west"], box["west"])
        self.assertGreater(wider["east"], box["east"])

    def test_a_network_failure_leaves_the_plane_alone(self):
        job = {"look_at": {"lat": 33.37, "lon": -111.94}, "width_m": 150, "length_m": 150}
        with mock.patch.object(sat.urllib.request, "urlopen", side_effect=urllib.error.URLError("down")):
            self.assertIsNone(sat.fetch_ground(job, "unused.jpg"))
        self.assertEqual(sat.apply_ground(GROUND, 0, 0), GROUND)


if __name__ == "__main__":
    unittest.main()
