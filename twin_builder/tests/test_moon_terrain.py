"""Lunar heightmap bake and the world the session image actually loads."""
import tempfile
import unittest
from pathlib import Path

from twin_builder import bake, capture, spawn


def _dem(grid=17, relief=40.0):
    """Centre cell is the look-at, so it stays 0. The relief is the north-west corner."""
    heights = []
    for row in range(grid):
        for col in range(grid):
            heights.append(relief if row == 0 and col == 0 else 0.0)
    return {"grid": grid, "heights": heights}


class MoonTerrainTests(unittest.TestCase):
    def test_clean_dem_rejects_an_even_grid(self):
        with self.assertRaises(ValueError):
            capture.clean_dem({"grid": 18, "heights": [0.0] * (18 * 18)})

    def test_clean_dem_rejects_a_grid_above_the_look_at(self):
        grid = 17
        heights = [400.0] * (grid * grid)
        with self.assertRaises(ValueError):
            capture.clean_dem({"grid": grid, "heights": heights})

    def test_bake_writes_a_relative_heightmap(self):
        job = {
            "id": "moon1",
            "callsign": "ktwin-moon01",
            "look_at": {"lat": -89.77, "lon": -156.2, "elev_m": 1800},
            "bbox": {},
            "width_m": 150,
            "length_m": 150,
            "dem": _dem(),
        }
        with tempfile.TemporaryDirectory() as td:
            art = bake.bake_moon(job, job_root=td)
            self.assertEqual(art["body"], "moon")
            self.assertEqual(art["props"], [])
            self.assertAlmostEqual(art["heightmap_z_min"], 0.0)
            self.assertGreater(art["heightmap_span"], 30)
            self.assertTrue(Path(art["heightmap_image"]).is_file())
            png = Path(art["heightmap_image"]).read_bytes()
            self.assertTrue(png.startswith(b"\x89PNG"))
            self.assertNotIn("normal_image", art)

    def test_session_world_uses_the_moon_and_the_heightmap(self):
        world = """<sdf><world name="default">
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>37.0</latitude_deg>
    </spherical_coordinates>
    <model name="ground_plane"><link name="link"></link></model>
  </world></sdf>"""
        text = spawn.apply_site(world, {"lat": -89.77, "lon": -156.2, "elev_m": 1800}, "", "MOON_SCS")
        text = spawn.apply_moon_terrain(text, 150, 120, -20, 60)
        self.assertIn("<surface_model>MOON_SCS</surface_model>", text)
        self.assertNotIn("EARTH_WGS84", text)
        self.assertNotIn("37.0", text)
        self.assertIn("/root/moon_heightmap.png", text)
        self.assertIn("<size>150.00 120.00 60.000</size>", text)
        self.assertIn("<pos>0 0 -20.000</pos>", text)
        self.assertEqual(text.count('name="ground_plane"'), 1)

    def test_dry_run_names_the_vehicle_for_the_moon_page(self):
        root = tempfile.mkdtemp()
        previous = spawn.SESSIONS_ROOT
        spawn.SESSIONS_ROOT = Path(root)
        try:
            art = {
                "callsign": "ktwin-moon02",
                "look_at": {"lat": -89.77, "lon": -156.2, "elev_m": 1800},
                "body": "moon",
                "spawn_z": 15,
                "airframe": "x500_gimbal",
            }
            meta = spawn.spawn_session(art, dry_run=True)
            self.assertEqual(meta["body"], "moon")
            env = (Path(root) / "ktwin-moon02" / "session.env").read_text()
            self.assertIn("PX4_GZ_MODEL_POSE=0,0,15.00,0,0,0", env)
            self.assertEqual(spawn._session_vehicle_name("ktwin-moon02"), "Moon session kestrel")
        finally:
            spawn.SESSIONS_ROOT = previous

    def test_earth_heightmap_clears_the_ridge_and_keeps_the_photo(self):
        job = {
            "id": "earth1",
            "callsign": "ktwin-earth01",
            "look_at": {"lat": 33.4, "lon": -111.9, "elev_m": 400},
            "bbox": {},
            "width_m": 80,
            "length_m": 60,
            "dem": _dem(relief=12.0),
        }
        with tempfile.TemporaryDirectory() as td:
            art = bake.bake_earth(job, job_root=td)
            self.assertEqual(art["body"], "earth")
            self.assertEqual(art["heightmap_mount"], "/root/earth_heightmap.png")
            self.assertGreaterEqual(art["spawn_z"], art["capture"]["max_z"] + 2)
            self.assertGreaterEqual(art["spawn_z"], 3)
            self.assertTrue(Path(art["heightmap_image"]).is_file())
            self.assertTrue(Path(art["normal_image"]).is_file())
            from PIL import Image
            with Image.open(art["normal_image"]) as normal:
                self.assertEqual(normal.size, (17, 17))
                # The centre cell is flat, so its detail normal points up.
                self.assertEqual(normal.getpixel((8, 8)), (128, 128, 255))
            world = """<sdf><world name="default">
    <model name="ground_plane"><link name="link"></link></model>
  </world></sdf>"""
            bare = spawn.apply_moon_terrain(
                world, 80, 60, art["heightmap_z_min"], art["heightmap_span"],
                art["heightmap_mount"], "/root/earth_diffuse.png")
            self.assertNotIn("<texture>", bare)
            text = spawn.apply_moon_terrain(
                world, 80, 60, art["heightmap_z_min"], art["heightmap_span"],
                art["heightmap_mount"], "/root/earth_diffuse.png", art["normal_mount"])
            self.assertIn("/root/earth_heightmap.png", text)
            self.assertIn("<diffuse>/root/earth_diffuse.png</diffuse>", text)
            self.assertIn("<normal>/root/earth_normal.png</normal>", text)
            self.assertNotIn("__default__", text)
            self.assertNotIn("<plane>", text)
            self.assertEqual(text.count('name="ground_plane"'), 1)

    def test_earth_heightmap_session_raises_the_vehicle(self):
        root = tempfile.mkdtemp()
        previous = spawn.SESSIONS_ROOT
        spawn.SESSIONS_ROOT = Path(root)
        try:
            art = {
                "callsign": "ktwin-earth02",
                "look_at": {"lat": 33.4, "lon": -111.9, "elev_m": 400},
                "body": "earth",
                "spawn_z": 7.5,
                "heightmap_image": "/tmp/earth_heightmap.png",
                "airframe": "x500_gimbal",
            }
            meta = spawn.spawn_session(art, dry_run=True)
            self.assertEqual(meta["body"], "earth")
            env = (Path(root) / "ktwin-earth02" / "session.env").read_text()
            self.assertIn("PX4_GZ_MODEL_POSE=0,0,7.50,0,0,0", env)
            self.assertEqual(spawn._session_vehicle_name("ktwin-earth02"), "Session kestrel")
            flat = {
                "callsign": "ktwin-earth03",
                "look_at": {"lat": 33.4, "lon": -111.9, "elev_m": 400},
                "body": "earth",
                "airframe": "x500_gimbal",
            }
            spawn.spawn_session(flat, dry_run=True)
            flat_env = (Path(root) / "ktwin-earth03" / "session.env").read_text()
            self.assertNotIn("PX4_GZ_MODEL_POSE", flat_env)
        finally:
            spawn.SESSIONS_ROOT = previous

    def test_satellite_jpeg_becomes_a_png(self):
        from PIL import Image

        from twin_builder import rasters

        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            Image.new("RGB", (24, 18), (40, 90, 140)).save(root / "ground.jpg", "JPEG")
            out = rasters.satellite_png(root / "ground.jpg", root / "earth_diffuse.png")
            self.assertTrue(out.endswith("earth_diffuse.png"))
            with Image.open(out) as image:
                self.assertEqual(image.mode, "RGB")
                self.assertEqual(image.size, (24, 18))
