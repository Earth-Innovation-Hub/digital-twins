"""One camera-layered HUD image for whatever xudrone behaviour is running.

Every runner built on UdroneHelixController draws into the same image on
``/xudrone/hud/image`` (UDRONE_HUD_TOPIC), and the latest frame is written
to ``/tmp/xudrone_hud.jpg`` once a second. UDRONE_HUD=0 turns it off.

Layers, back to front:
  camera     the task's camera (front, up, or down), switched per stage
  tags       AprilTag boxes that camera detected
  world      the reference path ahead, targets, the boat, and the task's own
             geometry, projected through that camera
  flight     horizon and the commanded hull direction (front camera)
  panel      task, stage, depth, speed, error, gate, pose source, latch
  insets     the other two cameras, and a north-up map around the hull

A runner adds to it with ``_hud_layers()``, which returns a dict with any
of: camera, stage, lines, markers, paths, cone, draw. The pure helpers here
(projection, map scale) are ROS-free and tested on their own.
"""
from __future__ import annotations

import math
import os
import time

# Mounts from model/xudrone/model.sdf. Image +x is hull -Y on all three. Image
# +y is hull -Z on the front camera, hull -X on the up camera, hull +X on the
# down camera.
CAMERAS = {
    "front": {
        "image": "/xudrone/camera_front/image",
        "info": "/xudrone/camera_front/camera_info",
        "det": "/xudrone/detections",
        "offset": (0.185, 0.0, 0.0),
        "hfov": 1.396,
    },
    "up": {
        "image": "/xudrone/camera_up/image",
        "info": "/xudrone/camera_up/camera_info",
        "det": "/xudrone/detections_up",
        "offset": (0.0, 0.0, 0.13),
        "hfov": 1.2,
    },
    "down": {
        "image": "/xudrone/camera_down/image",
        "info": "/xudrone/camera_down/camera_info",
        "det": "/xudrone/detections_down",
        "offset": (0.05, 0.0, -0.06),
        "hfov": 1.2,
    },
}
TASKS = {
    "UdroneHelixController": ("helix", "front"),
    "UdroneSwimController": ("swim", "front"),
    "ShapeController": ("shape", "front"),
    "UdroneCoverageController": ("coverage", "front"),
    "UdroneHoopController": ("hoop", "front"),
    "UdroneTrackOtter": ("track otter_k", "up"),
    "UdroneSearchOtter": ("search otter_k", "up"),
    "UdroneRendezvous": ("rendezvous + dock", "up"),
    "UdroneApproachController": ("approach tag", "front"),
    "UdroneHold": ("hold", "front"),
}
WIDTH, HEIGHT = 640, 480
MIN_DEPTH_M = 0.15
FRESH_S = 0.7
TAG_FRESH_S = 0.5
RENDER_S = 0.1
JPG_PATH = "/tmp/xudrone_hud.jpg"
MAP_PX = 170
# Global map from xudrone_mapper: OccupancyGrid obstacles and bed depth (decimetres).
MAP_NS = os.environ.get("UDRONE_HUD_MAP", "/xudrone/map")
MAP_DEPTH_DM = 100
MAP_VIEW_M = 20.0

CYAN = (255, 200, 0)
AMBER = (0, 200, 255)
MAGENTA = (255, 0, 255)
GREEN = (0, 220, 0)
WHITE = (255, 255, 255)
RED = (60, 60, 255)
YELLOW = (0, 255, 255)
GREY = (150, 150, 150)
# Aviation attitude ball (BGR): sky / ground.
ATT_SKY = (210, 150, 55)
ATT_GND = (45, 105, 155)
ATT_RADIUS = 62          # px; sits on the 170 px map
ATT_PITCH_FULL_DEG = 45.0  # ±this fills the ball


def body_axes(q):
    """World unit vectors of body x (forward), y (left), z (up) for q = (w, x, y, z)."""
    w, x, y, z = (float(v) for v in q[:4])
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)),
        (2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)),
        (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)),
    )


def rpy_from_q(q):
    """Roll, pitch, yaw (ZYX, radians) from quaternion (w, x, y, z)."""
    w, x, y, z = (float(v) for v in q[:4])
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def attitude_horizon_shift(pitch_rad, radius: int = ATT_RADIUS,
                           full_deg: float = ATT_PITCH_FULL_DEG) -> float:
    """Pixels to shift the horizon: nose-up moves the horizon down (aviation ADI)."""
    return math.degrees(float(pitch_rad)) * (float(radius) / max(1.0, float(full_deg)))


def default_intrinsics(camera: str, width: int = WIDTH, height: int = HEIGHT):
    f = (width / 2.0) / math.tan(CAMERAS[camera]["hfov"] / 2.0)
    return (f, f, width / 2.0, height / 2.0)


def camera_point(camera: str, pos, q):
    fwd, left, up = body_axes(q)
    off = CAMERAS[camera]["offset"]
    return tuple(float(pos[i]) + off[0] * fwd[i] + off[1] * left[i] + off[2] * up[i] for i in range(3))


def project(camera: str, pos, q, world, intr):
    """Pixel of a world point in `camera`, or None when it is behind the lens."""
    fwd, left, up = body_axes(q)
    cam = camera_point(camera, pos, q)
    d = tuple(float(world[i]) - cam[i] for i in range(3))
    f = sum(d[i] * fwd[i] for i in range(3))
    lft = sum(d[i] * left[i] for i in range(3))
    u = sum(d[i] * up[i] for i in range(3))
    if camera == "front":
        depth, x, y = f, -lft, -u
    elif camera == "up":
        depth, x, y = u, -lft, -f
    else:
        depth, x, y = -u, -lft, f
    if depth < MIN_DEPTH_M:
        return None
    fx, fy, cx, cy = intr
    return (cx + fx * x / depth, cy + fy * y / depth)


def map_scale(center, points, px: int = MAP_PX, r_min: float = 3.0, r_max: float = 40.0):
    """Pixels per metre so the farthest point (up to r_max) fits the map."""
    far = r_min
    for p in points:
        far = max(far, min(r_max, math.hypot(float(p[0]) - center[0], float(p[1]) - center[1])))
    return (px / 2.0 - 12.0) / far


def map_pixel(center, pt, scale, px: int = MAP_PX):
    """North-up map pixel of a world point; the hull is the centre."""
    return (
        px / 2.0 + (float(pt[0]) - center[0]) * scale,
        px / 2.0 - (float(pt[1]) - center[1]) * scale,
    )


def task_of(owner):
    name = type(owner).__name__
    label, camera = TASKS.get(name, (name, "front"))
    return getattr(owner, "hud_task", None) or label, camera


class TaskHud:
    """Subscribes to the three cameras and renders the HUD for its owner node."""

    def __init__(self, node, topic: str | None = None):
        import cv2
        import numpy as np
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image

        try:
            from apriltag_msgs.msg import AprilTagDetectionArray
        except ImportError:
            AprilTagDetectionArray = None
        self._cv2, self._np, self._Image = cv2, np, Image
        self.node = node
        self.task, self.camera = task_of(node)
        self.topic = topic or os.environ.get("UDRONE_HUD_TOPIC", "/xudrone/hud/image")
        self._frames = {}
        self._intr = {c: default_intrinsics(c) for c in CAMERAS}
        self._tags = {c: [] for c in CAMERAS}
        self._render_t = 0.0
        self._jpg_t = 0.0
        self._stamp = None
        self._err = False
        for cam, spec in CAMERAS.items():
            node.create_subscription(
                Image, spec["image"], lambda m, c=cam: self._on_image(c, m), qos_profile_sensor_data)
            node.create_subscription(
                CameraInfo, spec["info"], lambda m, c=cam: self._on_info(c, m), qos_profile_sensor_data)
            if AprilTagDetectionArray is not None:
                node.create_subscription(
                    AprilTagDetectionArray, spec["det"],
                    lambda m, c=cam: self._on_tags(c, m), qos_profile_sensor_data)
        self._maps = {}
        self._map_status = None
        self._plan_pts = []
        try:
            import json
            from nav_msgs.msg import OccupancyGrid, Path
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from std_msgs.msg import String

            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            for key in ("grid", "bed_depth"):
                node.create_subscription(OccupancyGrid, f"{MAP_NS}/{key}",
                                         lambda m, k=key: self._on_map(k, m), latched)
            node.create_subscription(String, f"{MAP_NS}/status", self._on_map_status, 2)
            node.create_subscription(Path, f"{MAP_NS}/coverage_plan", self._on_plan_path, latched)
            self._json = json
        except ImportError:
            self._json = None
        self.pub = node.create_publisher(Image, self.topic, 2)
        print(f"HUD {self.task} on {self.topic} (camera {self.camera})", flush=True)

    # ---- inputs ------------------------------------------------------------
    def _on_image(self, cam, m):
        self._frames[cam] = (m, time.time())

    def _on_info(self, cam, m):
        k = list(m.k)
        if len(k) >= 6 and k[0] > 1.0 and k[4] > 1.0:
            self._intr[cam] = (float(k[0]), float(k[4]), float(k[2]), float(k[5]))

    def _on_tags(self, cam, m):
        now = time.time()
        tags = []
        for d in m.detections:
            pts = [(float(c.x), float(c.y)) for c in d.corners]
            if len(pts) >= 4:
                tags.append((int(d.id), pts, (float(d.centre.x), float(d.centre.y)), now))
        self._tags[cam] = tags

    def _on_map(self, key, m):
        w, h = int(m.info.width), int(m.info.height)
        if w < 1 or h < 1 or len(m.data) < w * h:
            return
        arr = self._np.frombuffer(bytes(m.data), dtype=self._np.int8)[: w * h].reshape(h, w)
        self._maps[key] = (arr, float(m.info.resolution),
                           float(m.info.origin.position.x), float(m.info.origin.position.y))

    def _on_map_status(self, m):
        if self._json is None:
            return
        try:
            self._map_status = self._json.loads(m.data)
        except ValueError:
            pass

    def _on_plan_path(self, m):
        self._plan_pts = [(float(p.pose.position.x), float(p.pose.position.y)) for p in m.poses]

    def _map_raster(self, center, scale, px):
        """BGR raster of the global map around `center`, and the mask of known cells."""
        np = self._np
        got = self._maps.get("grid")
        if got is None:
            return None, None
        occ, res, ox, oy = got
        uv = (np.arange(px, dtype=np.float32) + 0.5 - px / 2.0) / scale
        xs = center[0] + uv[None, :]
        ys = center[1] - uv[:, None]
        cols = np.floor((xs - ox) / res).astype(np.int32)
        rows = np.floor((ys - oy) / res).astype(np.int32)
        inside = (cols >= 0) & (cols < occ.shape[1]) & (rows >= 0) & (rows < occ.shape[0])
        cols = np.where(inside, cols, 0)
        rows = np.where(inside, rows, 0)
        cols, rows = np.broadcast_to(cols, (px, px)), np.broadcast_to(rows, (px, px))
        o = np.where(inside, occ[rows, cols], -1)
        out = np.zeros((px, px, 3), np.uint8)
        out[o == 0] = (60, 60, 60)
        dep = self._maps.get("bed_depth")
        if dep is not None and dep[0].shape == occ.shape:
            d = np.where(inside, dep[0][rows, cols], -1).astype(np.int16)
            known = d >= 0
            if known.any():
                lut = self._cv2.applyColorMap(
                    np.clip(d * 255 // MAP_DEPTH_DM, 0, 255).astype(np.uint8), self._cv2.COLORMAP_VIRIDIS)
                out[known] = lut[known]
        out[o >= 50] = RED
        return out, o >= 0

    def _bgr(self, cam):
        got = self._frames.get(cam)
        if got is None or time.time() - got[1] > FRESH_S:
            return None
        m = got[0]
        cv2, np = self._cv2, self._np
        w, h = int(m.width), int(m.height)
        if w < 2 or h < 2:
            return None
        arr = np.frombuffer(m.data, dtype=np.uint8)
        key = (m.encoding or "").lower()
        if key in ("rgb8", "") and arr.size >= w * h * 3:
            return cv2.cvtColor(arr[: w * h * 3].reshape(h, w, 3), cv2.COLOR_RGB2BGR)
        if key == "bgr8" and arr.size >= w * h * 3:
            return arr[: w * h * 3].reshape(h, w, 3).copy()
        if key == "rgba8" and arr.size >= w * h * 4:
            return cv2.cvtColor(arr[: w * h * 4].reshape(h, w, 4), cv2.COLOR_RGBA2BGR)
        if key == "bgra8" and arr.size >= w * h * 4:
            return cv2.cvtColor(arr[: w * h * 4].reshape(h, w, 4), cv2.COLOR_BGRA2BGR)
        return None

    # ---- render ------------------------------------------------------------
    def render(self, pos, q, boat, cmd, tm):
        """Draw one frame. Never raises into the control loop."""
        now = time.time()
        if now - self._render_t < RENDER_S:
            return
        self._render_t = now
        try:
            self._render(pos, q, boat, cmd, tm)
        except Exception as exc:
            if not self._err:
                self._err = True
                print(f"HUD error {exc!r}", flush=True)

    def _render(self, pos, q, boat, cmd, tm):
        cv2, np = self._cv2, self._np
        owner = self.node
        layers = {}
        hook = getattr(owner, "_hud_layers", None)
        if hook is not None:
            layers = hook() or {}
        cam = layers.get("camera") or self.camera
        if cam not in CAMERAS:
            cam = "front"
        frame = self._bgr(cam)
        if frame is None:
            img = np.full((HEIGHT, WIDTH, 3), 24, np.uint8)
            cv2.putText(img, f"no {cam} image", (WIDTH // 2 - 70, HEIGHT // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, GREY, 1, cv2.LINE_AA)
        else:
            img = frame
        height, width = img.shape[:2]
        intr = self._intr[cam]

        def clip(pt):
            return (int(max(-2000, min(2000 + width, pt[0]))), int(max(-2000, min(2000 + height, pt[1]))))

        def proj(world):
            return project(cam, pos, q, world, intr)

        self._draw_tags(img, cam, clip)
        if cam == "front":
            self._draw_horizon(img, pos, q, proj, clip)
        for path in layers.get("paths", []):
            self._draw_path(img, path, proj, clip)
        markers = list(layers.get("markers", []))
        if boat is not None:
            markers.append({"pos": boat[0], "label": "otter_k", "color": WHITE, "shape": "square"})
        p_des = getattr(owner, "_p_des", None)
        if p_des is not None and getattr(owner, "hud_ref", True):
            markers.append({"pos": p_des, "label": "ref", "color": MAGENTA, "shape": "circle"})
        for mk in markers:
            self._draw_marker(img, mk, proj, clip)
        draw = layers.get("draw")
        if draw is not None:
            draw(img, proj, clip)
        if cam == "front":
            self._draw_flight_path(img, pos, q, proj, clip)
            self._draw_command(img, cmd, intr, clip)
        self._draw_panel(img, owner, pos, q, layers, cam, tm)
        self._draw_insets(img, cam)
        self._draw_map(img, owner, pos, q, boat, layers, markers)
        self._draw_footer(img, cmd, q)
        self._publish(img)

    def _draw_tags(self, img, cam, clip):
        cv2, np = self._cv2, self._np
        now = time.time()
        for tid, pts, centre, seen in self._tags.get(cam, []):
            if now - seen > TAG_FRESH_S:
                continue
            poly = np.array([clip(p) for p in pts], np.int32)
            cv2.polylines(img, [poly], True, CYAN, 2, cv2.LINE_AA)
            cv2.putText(img, f"tag {tid}", clip((centre[0] + 6, centre[1] - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, CYAN, 1, cv2.LINE_AA)

    def _draw_horizon(self, img, pos, q, proj, clip):
        cv2 = self._cv2
        fwd = body_axes(q)[0]
        h = math.hypot(fwd[0], fwd[1])
        if h < 0.2:
            return
        hx, hy = fwd[0] / h, fwd[1] / h
        z = float(pos[2]) + CAMERAS["front"]["offset"][0] * fwd[2]
        a = proj((pos[0] + 50 * hx - 30 * hy, pos[1] + 50 * hy + 30 * hx, z))
        b = proj((pos[0] + 50 * hx + 30 * hy, pos[1] + 50 * hy - 30 * hx, z))
        if a is not None and b is not None:
            cv2.line(img, clip(a), clip(b), (120, 200, 120), 1, cv2.LINE_AA)

    def _draw_path(self, img, path, proj, clip):
        cv2, np = self._cv2, self._np
        color = path.get("color", MAGENTA)
        width = int(path.get("width", 1))
        pixs = []
        for p in path.get("pts", []):
            pix = proj(p)
            pixs.append(clip(pix) if pix is not None else None)
        # Filled corridor / cone slabs (airport glide-path look).
        if path.get("fill") and path.get("closed"):
            poly = [p for p in pixs if p is not None]
            if len(poly) >= 3:
                overlay = img.copy()
                cv2.fillPoly(overlay, [np.array(poly, np.int32)], color, cv2.LINE_AA)
                alpha = float(path.get("fill_alpha", 0.25))
                cv2.addWeighted(overlay, alpha, img, 1.0 - alpha, 0, img)
                cv2.polylines(img, [np.array(poly, np.int32)], True, color, max(1, width), cv2.LINE_AA)
        else:
            prev = None
            for pix in pixs:
                if pix is not None and prev is not None:
                    cv2.line(img, prev, pix, color, width, cv2.LINE_AA)
                prev = pix if pix is not None else prev
            if path.get("closed"):
                first = next((p for p in pixs if p is not None), None)
                last = next((p for p in reversed(pixs) if p is not None), None)
                if first is not None and last is not None and first != last:
                    cv2.line(img, last, first, color, width, cv2.LINE_AA)
        last = next((p for p in reversed(pixs) if p is not None), None)
        if path.get("label") and last is not None:
            cv2.putText(img, path["label"], clip((last[0] + 6, last[1] - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)

    def _draw_marker(self, img, mk, proj, clip):
        cv2 = self._cv2
        pix = proj(mk["pos"])
        if pix is None:
            return
        color = mk.get("color", WHITE)
        shape = mk.get("shape", "cross")
        p = clip(pix)
        if shape == "circle":
            cv2.circle(img, p, 7, color, 2, cv2.LINE_AA)
        elif shape == "square":
            cv2.drawMarker(img, p, color, cv2.MARKER_SQUARE, 16, 1)
        elif shape == "diamond":
            cv2.drawMarker(img, p, color, cv2.MARKER_DIAMOND, 14, 1)
        else:
            cv2.drawMarker(img, p, color, cv2.MARKER_CROSS, 16, 2)
        if mk.get("label"):
            cv2.putText(img, mk["label"], clip((pix[0] + 9, pix[1] + 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)

    def _draw_flight_path(self, img, pos, q, proj, clip):
        cv2 = self._cv2
        d = getattr(self.node, "_dir", None)
        if d is None:
            return
        cam = camera_point("front", pos, q)
        pix = proj(tuple(cam[i] + 5.0 * d[i] for i in range(3)))
        if pix is None:
            return
        x, y = clip(pix)
        cv2.circle(img, (x, y), 6, GREEN, 1, cv2.LINE_AA)
        cv2.line(img, (x - 16, y), (x - 6, y), GREEN, 1, cv2.LINE_AA)
        cv2.line(img, (x + 6, y), (x + 16, y), GREEN, 1, cv2.LINE_AA)
        cv2.line(img, (x, y - 6), (x, y - 12), GREEN, 1, cv2.LINE_AA)

    def _draw_command(self, img, cmd, intr, clip):
        cv2 = self._cv2
        _thrust, _roll, pitch_u, yaw_u = cmd
        _fx, _fy, cx, cy = intr
        # +yaw command is nose left (image left). +pitch command is nose down.
        tip = clip((cx - float(yaw_u) * 140.0, cy + float(pitch_u) * 140.0))
        cv2.arrowedLine(img, clip((cx, cy)), tip, YELLOW, 2, tipLength=0.28)

    def _text(self, img, lines, x, y, color=AMBER, scale=0.46, step=17):
        cv2 = self._cv2
        if not lines:
            return
        w = max(cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0][0] for s in lines)
        y0 = y - 13
        roi = img[max(0, y0): min(img.shape[0], y0 + step * len(lines) + 6),
                  max(0, x - 4): min(img.shape[1], x + w + 6)]
        if roi.size:
            roi[:] = (roi * 0.45).astype(roi.dtype)
        for s in lines:
            cv2.putText(img, s, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)
            y += step

    def _draw_panel(self, img, owner, pos, q, layers, cam, tm):
        stage = layers.get("stage") or getattr(owner, "stage", None) or getattr(owner, "_stage", None) or ""
        if owner.phase == "handoff":
            stage = "handoff to hold"
        vel = getattr(owner, "vel", (0.0, 0.0, 0.0))
        speed = math.sqrt(sum(float(v) * float(v) for v in vel))
        err = getattr(owner, "_err_n", float("nan"))
        lines = [
            f"XUDRONE  {self.task.upper()}  {stage}",
            f"t {tm:5.1f}s  depth {-float(pos[2]):.2f} m  speed {speed:.2f} m/s",
            f"err {err:.2f} m  gate {getattr(owner, '_gate_rate', 1.0):.2f}  "
            f"pose {getattr(owner, 'pose_source', '?')}  latch {owner.latch.state}  cam {cam}",
        ]
        lines += [str(s) for s in layers.get("lines", [])]
        lines += self._map_status_lines()
        self._text(img, lines, 8, 20)

    def _map_status_lines(self):
        """Global map / plan / factor-graph summary for the telemetry panel."""
        st = self._map_status
        if not st:
            return []
        out = []
        m = st.get("map") or {}
        if m:
            out.append(
                f"map bed {m.get('bed_cells', 0)} ({m.get('bed_area_m2', 0)} m2)  "
                f"obst {m.get('obstacle_cells', 0)}  cover {m.get('cover_cells', 0)}"
            )
        p = st.get("plan")
        if p:
            pct = p.get("pct")
            pct_s = f"  {pct:g}%" if pct is not None else ""
            out.append(
                f"plan {p.get('target', '?')} {p.get('waypoints', 0)} wps "
                f"{p.get('length_m', 0)} m{pct_s}"
            )
        g = st.get("graph") or {}
        if g.get("enabled"):
            factors = g.get("factors") or {}
            n_f = sum(int(v) for v in factors.values()) if isinstance(factors, dict) else 0
            err = g.get("err_m")
            oerr = g.get("odom_err_m")
            sigma = g.get("sigma_m")
            err_s = f"  err {err:.2f} m" if err is not None else ""
            odom_s = f"  odom {oerr:.2f} m" if oerr is not None else ""
            sig_s = f"  σ {sigma:.2f} m" if sigma is not None else ""
            out.append(
                f"graph kf {g.get('keyframes', 0)}  lm {len(g.get('landmarks') or [])}  "
                f"factors {n_f}{sig_s}{err_s}{odom_s}"
            )
        elif "enabled" in g:
            out.append("graph off")
        return out

    def _draw_insets(self, img, main):
        cv2 = self._cv2
        h, w = img.shape[:2]
        tw, th = 152, 114
        y = 6
        for cam in CAMERAS:
            if cam == main:
                continue
            x = w - tw - 6
            frame = self._bgr(cam)
            if frame is not None:
                img[y:y + th, x:x + tw] = cv2.resize(frame, (tw, th), interpolation=cv2.INTER_AREA)
            else:
                img[y:y + th, x:x + tw] = 24
            cv2.rectangle(img, (x, y), (x + tw - 1, y + th - 1), GREY, 1)
            fresh = [t for t in self._tags.get(cam, []) if time.time() - t[3] <= TAG_FRESH_S]
            label = cam + (f"  tag {fresh[0][0]}" if fresh else "")
            cv2.putText(img, label, (x + 4, y + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                        CYAN if fresh else WHITE, 1, cv2.LINE_AA)
            y += th + 4

    def _draw_map(self, img, owner, pos, q, boat, layers, markers):
        cv2, np = self._cv2, self._np
        h, w = img.shape[:2]
        px = MAP_PX
        x0, y0 = w - px - 6, h - px - 26
        roi = img[y0:y0 + px, x0:x0 + px]
        roi[:] = (roi * 0.35).astype(roi.dtype)
        center = (float(pos[0]), float(pos[1]))
        pts = [mk["pos"] for mk in markers]
        for path in layers.get("paths", []):
            pts += list(path.get("pts", []))
        graph = (self._map_status or {}).get("graph") or {}
        for xy in graph.get("kf_xy") or []:
            pts.append((xy[0], xy[1], 0.0))
        for lm in graph.get("landmarks") or []:
            pts.append((lm["x"], lm["y"], lm.get("z", 0.0)))
        pts += [(p[0], p[1], 0.0) for p in self._plan_pts]
        scale = map_scale(center, pts, r_min=MAP_VIEW_M if "grid" in self._maps else 3.0)
        raster, seen = self._map_raster(center, scale, px)
        if raster is not None:
            roi[seen] = (0.45 * roi[seen] + 0.55 * raster[seen]).astype(roi.dtype)

        def mp(p):
            u, v = map_pixel(center, p, scale)
            return (int(x0 + max(0, min(px - 1, u))), int(y0 + max(0, min(px - 1, v))))

        cv2.rectangle(img, (x0, y0), (x0 + px - 1, y0 + px - 1), GREY, 1)
        for ring_m in (1.0, 5.0, 10.0, 20.0):
            r = int(ring_m * scale)
            if 6 < r < px // 2:
                cv2.circle(img, mp(center), r, (70, 70, 70), 1, cv2.LINE_AA)
                cv2.putText(img, f"{ring_m:g}m", (mp(center)[0] + r - 18, mp(center)[1] - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.3, GREY, 1, cv2.LINE_AA)
        kf = graph.get("kf_xy") or []
        if len(kf) >= 2:
            chain = [mp((p[0], p[1])) for p in kf]
            cv2.polylines(img, [np.array(chain, np.int32)], False, CYAN, 1, cv2.LINE_AA)
        for lm in graph.get("landmarks") or []:
            c = mp((lm["x"], lm["y"]))
            cv2.drawMarker(img, c, GREEN, cv2.MARKER_DIAMOND, 8, 1)
        if len(self._plan_pts) >= 2:
            chain = [mp(p) for p in self._plan_pts]
            cv2.polylines(img, [np.array(chain, np.int32)], False, MAGENTA, 1, cv2.LINE_AA)
        for path in layers.get("paths", []):
            chain = [mp(p) for p in path.get("pts", [])]
            if len(chain) < 2:
                continue
            closed = bool(path.get("closed"))
            if path.get("fill") and closed and len(chain) >= 3:
                overlay = img.copy()
                cv2.fillPoly(overlay, [np.array(chain, np.int32)], path.get("color", MAGENTA))
                alpha = float(path.get("fill_alpha", 0.25))
                cv2.addWeighted(overlay, alpha, img, 1.0 - alpha, 0, img)
            cv2.polylines(
                img, [np.array(chain, np.int32)], closed,
                path.get("color", MAGENTA), 1, cv2.LINE_AA,
            )
        cone = layers.get("cone")
        if cone is not None:
            r = int(cone["radius"] * scale)
            cv2.circle(img, mp(cone["center"]), max(2, r), GREEN if cone.get("inside") else AMBER, 1, cv2.LINE_AA)
        if boat is not None:
            bf = body_axes(boat[1])[0]
            b = mp(boat[0])
            tip = mp((boat[0][0] + 1.2 * bf[0], boat[0][1] + 1.2 * bf[1]))
            cv2.rectangle(img, (b[0] - 4, b[1] - 4), (b[0] + 4, b[1] + 4), WHITE, 1)
            cv2.line(img, b, tip, WHITE, 1, cv2.LINE_AA)
        for mk in markers:
            if mk.get("label") == "otter_k":
                continue
            cv2.circle(img, mp(mk["pos"]), 3, mk.get("color", WHITE), -1, cv2.LINE_AA)
        fwd = body_axes(q)[0]
        hn = math.hypot(fwd[0], fwd[1]) or 1.0
        fx, fy = fwd[0] / hn, fwd[1] / hn
        c = map_pixel(center, center, scale)
        tri = []
        for along, side in ((9.0, 0.0), (-6.0, 5.0), (-6.0, -5.0)):
            ex = along * fx - side * fy
            ny = along * fy + side * fx
            tri.append((int(x0 + c[0] + ex), int(y0 + c[1] - ny)))
        cv2.fillPoly(img, [np.array(tri, np.int32)], YELLOW)
        cv2.putText(img, "N", (x0 + px // 2 - 4, y0 + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, WHITE, 1, cv2.LINE_AA)
        # Aviation attitude ball centred on the map (roll / pitch ADI).
        self._draw_attitude(img, x0 + px // 2, y0 + px // 2, ATT_RADIUS, q)

    def _draw_attitude(self, img, cx, cy, radius, q):
        """Circular roll/pitch attitude indicator (aviation ADI) over the map."""
        cv2, np = self._cv2, self._np
        roll, pitch, _yaw = rpy_from_q(q)
        r = int(radius)
        d = 2 * r + 4
        mid = d // 2
        tile = np.zeros((d, d, 3), np.uint8)
        shift = attitude_horizon_shift(pitch, r)
        hy = int(round(mid + shift))
        tile[: max(0, min(d, hy)), :] = ATT_SKY
        tile[max(0, min(d, hy)) :, :] = ATT_GND
        # Pitch ladder (before bank rotate): ±10° / ±20° bars.
        px_per_deg = float(r) / ATT_PITCH_FULL_DEG
        for deg in (-20, -10, 10, 20):
            y = int(round(mid + shift - deg * px_per_deg))
            if 2 <= y < d - 2:
                half = 18 if abs(deg) == 10 else 12
                cv2.line(tile, (mid - half, y), (mid + half, y), WHITE, 1, cv2.LINE_AA)
                cv2.putText(tile, f"{abs(deg)}", (mid + half + 2, y + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.28, WHITE, 1, cv2.LINE_AA)
        # Bank: rotate sky/ground with roll (instrument view).
        M = cv2.getRotationMatrix2D((mid, mid), -math.degrees(roll), 1.0)
        tile = cv2.warpAffine(
            tile, M, (d, d), flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT, borderValue=(28, 28, 28),
        )
        mask = np.zeros((d, d), np.uint8)
        cv2.circle(mask, (mid, mid), r, 255, -1)
        x0, y0 = int(cx) - mid, int(cy) - mid
        h, w = img.shape[:2]
        x1, y1 = max(0, x0), max(0, y0)
        x2, y2 = min(w, x0 + d), min(h, y0 + d)
        if x2 <= x1 or y2 <= y1:
            return
        tx1, ty1 = x1 - x0, y1 - y0
        tx2, ty2 = tx1 + (x2 - x1), ty1 + (y2 - y1)
        roi = img[y1:y2, x1:x2]
        src = tile[ty1:ty2, tx1:tx2]
        m = mask[ty1:ty2, tx1:tx2] > 0
        roi[m] = src[m]
        # Bezel + roll ticks (fixed); sky pointer at top.
        cx_i, cy_i = int(cx), int(cy)
        cv2.circle(img, (cx_i, cy_i), r, WHITE, 1, cv2.LINE_AA)
        for deg in range(-60, 61, 30):
            a = math.radians(deg - 90.0)  # 0° bank at top
            c0, s0 = math.cos(a), math.sin(a)
            inner, outer = r - (8 if deg % 60 == 0 else 5), r
            cv2.line(
                img,
                (int(cx + inner * c0), int(cy + inner * s0)),
                (int(cx + outer * c0), int(cy + outer * s0)),
                WHITE, 1, cv2.LINE_AA,
            )
        top = (cx_i, cy_i - r + 1)
        cv2.fillPoly(
            img,
            [np.array([(top[0], top[1] + 7), (top[0] - 5, top[1] - 1), (top[0] + 5, top[1] - 1)], np.int32)],
            YELLOW,
        )
        # Fixed wing / waterline symbol.
        cv2.line(img, (cx_i - r // 2, cy_i), (cx_i - 7, cy_i), YELLOW, 2, cv2.LINE_AA)
        cv2.line(img, (cx_i + 7, cy_i), (cx_i + r // 2, cy_i), YELLOW, 2, cv2.LINE_AA)
        cv2.circle(img, (cx_i, cy_i), 3, YELLOW, 1, cv2.LINE_AA)
        cv2.putText(
            img,
            f"R{math.degrees(roll):+.0f} P{math.degrees(pitch):+.0f}",
            (cx_i - r + 4, cy_i + r - 6),
            cv2.FONT_HERSHEY_SIMPLEX, 0.32, WHITE, 1, cv2.LINE_AA,
        )

    def _draw_footer(self, img, cmd, q):
        thrust, _roll, pitch_u, yaw_u = cmd
        roll, pitch, yaw = rpy_from_q(q)
        heading = (90.0 - math.degrees(yaw)) % 360.0
        line = (f"hdg {heading:03.0f}  roll {math.degrees(roll):+.0f}  "
                f"pitch {math.degrees(pitch):+.0f}  "
                f"thrust {thrust:+.2f}  pitch_u {pitch_u:+.2f}  yaw_u {yaw_u:+.2f}")
        self._text(img, [line], 8, img.shape[0] - 10, YELLOW, 0.46)

    def _publish(self, img):
        import array

        height, width = img.shape[:2]
        msg = self._Image()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "xudrone/hud"
        msg.height = height
        msg.width = width
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = width * 3
        msg.data = array.array("B", img.tobytes())
        self.pub.publish(msg)
        if time.time() >= self._jpg_t:
            self._jpg_t = time.time() + 1.0
            try:
                self._cv2.imwrite(JPG_PATH, img)
            except Exception:
                pass
