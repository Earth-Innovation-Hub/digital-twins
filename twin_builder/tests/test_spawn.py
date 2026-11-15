"""Protected names and dry-run spawn (no docker)."""
import os
import tempfile
import unittest
from pathlib import Path

from twin_builder import spawn


class SpawnSafetyTests(unittest.TestCase):
    def test_protected_names(self):
        for bad in ("locutus", "otter", "aquatic", "digital-twin-locutus", "kestrel"):
            with self.assertRaises(ValueError):
                spawn._assert_safe_name(bad)
        spawn._assert_safe_name("ktwin-abc12")

    def test_dry_run_session(self):
        root = tempfile.mkdtemp()
        os.environ["DEEPGIS_TWIN_SESSIONS_DIR"] = root
        spawn.SESSIONS_ROOT = Path(root)
        art = {
            "callsign": "ktwin-unit01",
            "look_at": {"lat": 33.4, "lon": -111.9, "elev_m": 350},
            "world_sdf": str(Path(root) / "site.sdf"),
            "airframe": "x500_gimbal",
        }
        meta = spawn.spawn_session(art, dry_run=True)
        self.assertEqual(meta["status"], "ready")
        self.assertTrue(meta.get("dry_run"))
        self.assertTrue((Path(root) / "ktwin-unit01" / "session.json").is_file())
        en = spawn.enroll_pose("ktwin-unit01", art["look_at"])
        self.assertEqual(en["vehicle"], "ktwin-unit01")

    def test_site_origin_replaces_the_image_home_and_props_are_inserted(self):
        text = """<sdf><world name="default">
    <spherical_coordinates>
      <latitude_deg>37.4593473</latitude_deg>
      <longitude_deg>-118.4444592</longitude_deg>
      <elevation>1401.47</elevation>
    </spherical_coordinates>
    <model name="ground_plane"></model>
  </world></sdf>"""
        out = spawn.apply_site(
            text,
            {"lat": 33.37344, "lon": -111.93548, "elev_m": 332.0},
            '    <model name="prop_building_0"></model>\n',
        )
        self.assertIn("<latitude_deg>33.37344000</latitude_deg>", out)
        self.assertIn("<longitude_deg>-111.93548000</longitude_deg>", out)
        self.assertIn("<elevation>332.000</elevation>", out)
        self.assertNotIn("37.4593473", out)
        self.assertIn('name="prop_building_0"', out)
        self.assertEqual(out.count("<spherical_coordinates>"), 1)

    def test_registry_port_is_published_off_the_lab_desktops(self):
        self.assertEqual(spawn.publish_args(40100), ["-p", "40100:40001"])
        for reserved in (40077, 40088, 40099, 80, 41000):
            with self.assertRaises(ValueError):
                spawn.publish_args(reserved)
        root = tempfile.mkdtemp()
        os.environ["DEEPGIS_TWIN_SESSIONS_DIR"] = root
        spawn.SESSIONS_ROOT = Path(root)
        art = {
            "callsign": "ktwin-unit02",
            "look_at": {"lat": 33.4, "lon": -111.9},
            "world_sdf": str(Path(root) / "site.sdf"),
            "host_port": 40100,
        }
        meta = spawn.spawn_session(art, dry_run=True)
        self.assertEqual(meta["host_port"], 40100)
        self.assertTrue(meta.get("dry_run"))


if __name__ == "__main__":
    unittest.main()
