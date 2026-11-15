"""Nadir bytes land in the capture directory. No network."""
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from twin_builder import capture


class CaptureNadirTests(unittest.TestCase):
    def test_viewport_jpeg_is_kept(self):
        jpeg = b"\xff\xd8\xff\xd9"
        look = {"lat": 33.4, "lon": -111.9, "elev_m": 350.0}
        bbox = {"west": -111.9001, "south": 33.3999, "east": -111.8999, "north": 33.4001}
        with tempfile.TemporaryDirectory() as td, \
                mock.patch.object(capture, "sample_dem", return_value=([350.0] * (capture.GRID ** 2), 350.0, 350.0)):
            meta = capture.capture_site(Path(td), look, bbox, nadir_jpeg=jpeg)
        nadir = Path(meta["nadir"])
        self.assertEqual(nadir.read_bytes(), jpeg)
        self.assertLess(meta["size_x_m"], 30)
        self.assertLess(meta["size_y_m"], 30)
