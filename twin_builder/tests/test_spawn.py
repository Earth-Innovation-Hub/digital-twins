"""Protected names and dry-run spawn (no docker)."""
import hashlib
import hmac
import json
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
        script = (Path(root) / "ktwin-unit01" / "start_session.sh").read_text()
        bridge = (Path(root) / "ktwin-unit01" / "camera_bridge.sh").read_text()
        self.assertIn("camera_bridge.sh", script)
        self.assertIn("/drone_camera_info|/${PX4_UXRCE_DDS_NS}/camera/camera_info", script)
        self.assertIn("/drone_camera|/${PX4_UXRCE_DDS_NS}/camera/image", script)
        info_at = script.index("drone_camera_info")
        image_at = script.index("s|/drone_camera|")
        self.assertLess(info_at, image_at)
        self.assertIn("x500_gimbal_0/link/camera_link/sensor/camera", bridge)
        self.assertIn("node:=camera_bridge", bridge)
        self.assertIn("flock -n 9", bridge)
        self.assertIn("gz topic -l", bridge)
        self.assertIn(":=/$NS/camera/image", bridge)
        self.assertIn(":=/$NS/gimbal/pitch", bridge)
        self.assertIn(":=/$NS/odometry", bridge)
        self.assertNotIn("__ns", bridge)
        self.assertNotIn("ktwin-", bridge)
        self.assertEqual(meta["dds_ns"], "ktwin_unit01")
        env = (Path(root) / "ktwin-unit01" / "session.env").read_text()
        self.assertIn("PX4_UXRCE_DDS_NS=ktwin_unit01", env)

    def test_enroll_token_is_the_fleet_hmac(self):
        root = tempfile.mkdtemp()
        os.environ["DEEPGIS_TWIN_SESSIONS_DIR"] = root
        spawn.SESSIONS_ROOT = Path(root)
        os.environ["DEEPGIS_VEHICLE_FLEET_SECRET"] = "fleet-secret"
        try:
            spawn.enroll_pose("ktwin-unit01", {"lat": 33.4, "lon": -111.9})
            token = hmac.new(b"fleet-secret", b"ktwin-unit01", hashlib.sha256).hexdigest()
            text = (Path(root) / "ktwin-unit01" / "pose.env").read_text()
            self.assertIn(f"VEHICLE_POSE_TOKEN={token}", text)
            folded = hashlib.sha256(b"fleet-secret:ktwin-unit01").hexdigest()
            self.assertNotIn(folded, text)
        finally:
            os.environ.pop("DEEPGIS_VEHICLE_FLEET_SECRET", None)

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


    def test_session_controls_leave_lab_containers_alone(self):
        for name in ("locutus", "aquatic", "kestrel", "digital-twin-locutus"):
            for fn in (spawn.stop_session, spawn.start_session, spawn.purge_session):
                with self.assertRaises(ValueError):
                    fn(name)

    def test_purge_removes_only_that_session_dir(self):
        root = tempfile.mkdtemp()
        os.environ["DEEPGIS_TWIN_SESSIONS_DIR"] = root
        spawn.SESSIONS_ROOT = Path(root)
        folder = Path(root) / "ktwin-unit03"
        folder.mkdir()
        (folder / "pose.env").write_text("token\n", encoding="utf-8")
        kept = Path(root) / "ktwin-other"
        kept.mkdir()
        calls = []

        def fake_docker(*args, check=True):
            calls.append(args)

            class Result:
                returncode = 0
                stdout = ""
                stderr = ""

            return Result()

        original = spawn._docker
        spawn._docker = fake_docker
        try:
            out = spawn.purge_session("ktwin-unit03")
        finally:
            spawn._docker = original
        self.assertEqual(out["status"], "purged")
        self.assertFalse(folder.exists())
        self.assertTrue(kept.is_dir())
        self.assertIn(("rm", "-f", "deepgis-frames-ktwin-unit03"), calls)
        self.assertIn(("rm", "-f", "digital-twin-session-ktwin-unit03"), calls)

    def test_frames_sidecar_always_mounts_a_udp_profile(self):
        root = tempfile.mkdtemp()
        os.environ["DEEPGIS_TWIN_SESSIONS_DIR"] = root
        spawn.SESSIONS_ROOT = Path(root)
        agent = Path(root) / "agent"
        agent.mkdir()
        (agent / "deepgis_frame_agent.py").write_text("# agent\n", encoding="utf-8")
        os.environ["DEEPGIS_FRAME_AGENT_DIR"] = str(agent)
        os.environ["DEEPGIS_VEHICLE_FLEET_SECRET"] = "fleet-secret"
        os.environ.pop("DEEPGIS_FRAME_UDP_XML", None)
        calls = []

        def fake_docker(*args, check=True):
            calls.append(args)

            class Result:
                returncode = 0
                stdout = ""
                stderr = ""

            return Result()

        original = spawn._docker
        spawn._docker = fake_docker
        try:
            out = spawn.start_frames("ktwin-unit01")
        finally:
            spawn._docker = original
            os.environ.pop("DEEPGIS_FRAME_AGENT_DIR", None)
            os.environ.pop("DEEPGIS_VEHICLE_FLEET_SECRET", None)
        self.assertEqual(out["status"], "up")
        flat = [arg for call in calls for arg in call]
        self.assertIn("FASTRTPS_DEFAULT_PROFILES_FILE=/udp.xml", flat)
        xml = (Path(root) / "ktwin-unit01" / "dds_udp.xml").read_text()
        self.assertIn("<useBuiltinTransports>false</useBuiltinTransports>", xml)
        self.assertIn("<type>UDPv4</type>", xml)
        self.assertNotRegex(xml, r"\d+\.\d+\.\d+\.\d+")
        self.assertIn("DEEPGIS_FRAME_TOPIC=/ktwin_unit01/camera/image", flat)

    def test_quota_keeps_a_stopped_session_and_stops_its_sidecar(self):
        root = tempfile.mkdtemp()
        os.environ["DEEPGIS_TWIN_SESSIONS_DIR"] = root
        spawn.SESSIONS_ROOT = Path(root)
        for name in ("ktwin-live", "ktwin-idle", "ktwin-gone"):
            folder = Path(root) / name
            folder.mkdir()
            (folder / "session.json").write_text(
                json.dumps({"callsign": name, "status": "ready", "updated": 0, "created": 0}),
                encoding="utf-8",
            )
        states = {
            "digital-twin-session-ktwin-live": "running",
            "deepgis-frames-ktwin-live": "running",
            "digital-twin-session-ktwin-idle": "exited",
            "deepgis-frames-ktwin-idle": "running",
            "deepgis-frames-ktwin-gone": "running",
        }
        calls = []

        def fake_docker(*args, check=True):
            calls.append(args)

            class Result:
                returncode = 0
                stdout = ""
                stderr = ""

            if len(args) >= 4 and args[0] == "inspect" and args[2] == "{{.State.Status}}":
                if args[3] in states:
                    Result.stdout = states[args[3]] + "\n"
                else:
                    Result.returncode = 1
            return Result()

        original = spawn._docker
        spawn._docker = fake_docker
        try:
            spawn.enforce_quota("ktwin-new")
        finally:
            spawn._docker = original
        self.assertNotIn(("rm", "-f", "digital-twin-session-ktwin-live"), calls)
        self.assertNotIn(("rm", "-f", "digital-twin-session-ktwin-idle"), calls)
        self.assertNotIn(("rm", "-f", "deepgis-frames-ktwin-live"), calls)
        self.assertIn(("stop", "-t", "15", "deepgis-frames-ktwin-idle"), calls)
        self.assertIn(("rm", "-f", "deepgis-frames-ktwin-gone"), calls)
        idle = json.loads((Path(root) / "ktwin-idle" / "session.json").read_text())
        self.assertEqual(idle["status"], "ready")

    def test_reconcile_recreates_a_stale_sidecar_and_stops_an_orphaned_one(self):
        root = tempfile.mkdtemp()
        os.environ["DEEPGIS_TWIN_SESSIONS_DIR"] = root
        spawn.SESSIONS_ROOT = Path(root)
        agent = Path(root) / "agent"
        agent.mkdir()
        (agent / "deepgis_frame_agent.py").write_text("# agent\n", encoding="utf-8")
        os.environ["DEEPGIS_FRAME_AGENT_DIR"] = str(agent)
        os.environ["DEEPGIS_VEHICLE_FLEET_SECRET"] = "fleet-secret"
        os.environ.pop("DEEPGIS_FRAME_UDP_XML", None)
        states = {
            "digital-twin-session-ktwin-stale": "running",
            "deepgis-frames-ktwin-stale": "running",
            "digital-twin-session-ktwin-down": "exited",
            "deepgis-frames-ktwin-down": "running",
            "digital-twin-session-ktwin-fresh": "running",
            "deepgis-frames-ktwin-fresh": "running",
            "digital-twin-session-ktwin-same": "running",
            "deepgis-frames-ktwin-same": "running",
            "deepgis-frames-kestrel": "running",
        }
        started = {
            "digital-twin-session-ktwin-stale": "2026-11-15T20:00:00Z",
            "deepgis-frames-ktwin-stale": "2026-11-15T19:00:00Z",
            "digital-twin-session-ktwin-fresh": "2026-11-15T20:00:00Z",
            "deepgis-frames-ktwin-fresh": "2026-11-15T20:00:01Z",
            "digital-twin-session-ktwin-same": "2026-11-15T20:00:01Z",
            "deepgis-frames-ktwin-same": "2026-11-15T20:00:01.5Z",
        }
        calls = []

        def fake_docker(*args, check=True):
            calls.append(args)

            class Result:
                returncode = 0
                stdout = ""
                stderr = ""

            if args[:2] == ("ps", "-a"):
                Result.stdout = "\n".join(states) + "\n"
            elif len(args) >= 4 and args[0] == "inspect" and args[2] == "{{.State.Status}}":
                if args[3] in states:
                    Result.stdout = states[args[3]] + "\n"
                else:
                    Result.returncode = 1
            elif len(args) >= 4 and args[0] == "inspect" and args[2] == "{{.State.StartedAt}}":
                Result.stdout = started.get(args[3], "") + "\n"
            return Result()

        original = spawn._docker
        spawn._docker = fake_docker
        try:
            changed = spawn.reconcile_session_frames()
        finally:
            spawn._docker = original
            os.environ.pop("DEEPGIS_FRAME_AGENT_DIR", None)
            os.environ.pop("DEEPGIS_VEHICLE_FLEET_SECRET", None)
        self.assertEqual(changed, ["ktwin-down", "ktwin-stale"])
        self.assertIn(("stop", "-t", "15", "deepgis-frames-ktwin-down"), calls)
        self.assertIn(("rm", "-f", "deepgis-frames-ktwin-stale"), calls)
        self.assertNotIn(("rm", "-f", "deepgis-frames-ktwin-fresh"), calls)
        self.assertNotIn(("rm", "-f", "deepgis-frames-ktwin-same"), calls)
        self.assertNotIn(("stop", "-t", "15", "deepgis-frames-kestrel"), calls)
        self.assertTrue(any(
            "container:digital-twin-session-ktwin-stale" in arg
            for call in calls for arg in call
        ))

    def test_session_route_is_only_stop_start_purge(self):
        from twin_builder.worker import session_route
        self.assertEqual(session_route("/sessions/ktwin-abc/stop"), ("ktwin-abc", "stop"))
        self.assertEqual(session_route("/sessions/ktwin-abc/purge"), ("ktwin-abc", "purge"))
        self.assertIsNone(session_route("/sessions/ktwin-abc/fly"))
        self.assertIsNone(session_route("/sessions/locutus/stop/extra"))
        self.assertIsNone(session_route("/jobs"))


if __name__ == "__main__":
    unittest.main()
