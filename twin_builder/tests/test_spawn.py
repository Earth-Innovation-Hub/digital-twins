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


if __name__ == "__main__":
    unittest.main()
