#!/usr/bin/env python3
"""xudrone mapper: TF tree, the global 2D map, and the sonar feed for OctoMap.

in   /xudrone/fmu/out/vehicle_local_position, vehicle_odometry   PX4 estimate
     Gazebo dynamic poses                                          check, and otter_k
     /xudrone/range/bottom                                         altimeter (bed)
     /xudrone/sonar/points_raw                                     multibeam, sonar frame
     /xudrone/octomap/points                                       OctoMap occupied voxels
     /xudrone/tf_front, /xudrone/tf, /xudrone/tf_up                apriltag camera -> tag TFs
     /xudrone/map/plan_coverage                                    String JSON coverage request
     /xudrone/map/reset                                            std_msgs/Empty clears map and graph
out  /tf, /tf_static     map -> odom -> xudrone/base_link -> sensors; map -> otter_k/base_link
     /xudrone/odom                    nav_msgs/Odometry (the pose the behaviours fly)
     /xudrone/sonar/points            finite returns, stamped with the TF, for OctoMap
     /xudrone/map/grid_map            grid_map_msgs/GridMap: bed_z, obstacle, cover, visited, sonar
     /xudrone/map/grid                nav_msgs/OccupancyGrid: 100 obstacle, 0 seen, -1 unknown
     /xudrone/map/bed_depth           nav_msgs/OccupancyGrid: bed depth in decimetres
     /xudrone/map/local_cloud         OctoMap voxels and altimeter hits within LOCAL_R_M
     /xudrone/map/track               nav_msgs/Path
     /xudrone/map/image               north-up render, also /tmp/xudrone_map.png
     /xudrone/map/costmap             nav_msgs/OccupancyGrid, Nav2 costs at CRUISE_Z
     /xudrone/map/coverage_plan       nav_msgs/Path of the latest plan (latched)
     /xudrone/map/coverage_plan_info  String JSON: request id, waypoints, stats (latched)
     /xudrone/map/status              String JSON at 1 Hz: map, plan, and graph summary (the HUD reads it)
     /xudrone/graph/path              nav_msgs/Path of optimised keyframes
     /xudrone/graph/pose              PoseWithCovarianceStamped, graph pose now
     /xudrone/graph/markers           MarkerArray: keyframes and tag landmarks

`map` is Kiwanis ENU (the Gazebo world frame, origin at the lake centre) and
`odom` coincides with it. The 2D map is saved every minute and reloaded on
start (UDRONE_MAP_FRESH=1 starts empty). The pose graph (factor_graph.py)
runs beside the flown pose: PX4 odometry between keyframes, emulated pressure
depth and surface GNSS from Gazebo with noise, and AprilTag bearing-range
landmarks. Its error against Gazebo is reported, and it is dumped to
/root/udrone_boat/maps/xudrone_graph.json.
"""
from __future__ import annotations

import array
import collections
import json
import math
import os
import random
import sys
import time

sys.path.insert(0, os.environ.get("UDRONE_SRC", "/root/udrone_boat/src"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, LaserScan, PointCloud2, PointField
from std_msgs.msg import Empty, Float32MultiArray, MultiArrayDimension, String
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

try:
    from grid_map_msgs.msg import GridMap
except ImportError:
    GridMap = None

import coverage_plan
import factor_graph as fg
import map_geom
from pose_frame import PoseFuser, ned_to_enu, px4_q_to_enu
from udrone_sinusoidal_controller import GzPoses, detect_target

MAP_FRAME = "map"
ODOM_FRAME = "odom"
BASE = "xudrone/base_link"
SONAR_FRAME = "xudrone/sonar"
BOAT_FRAME = "otter_k/base_link"
LOCAL_R_M = 20.0
SAVE_PATH = "/root/udrone_boat/maps/xudrone_map.npz"
GRAPH_PATH = "/root/udrone_boat/maps/xudrone_graph.json"
PNG_PATH = "/tmp/xudrone_map.png"
IMG_PX = 640
POSE_STALE_S = 0.5
SONAR_MAX_M = 29.5
TAG_FRESH_S = 0.6            # keep sightings long enough for the next tag keyframe
TAG_PREFIX = "tag36h11:"
# Emulated sensors drawn from Gazebo truth (sigma, m).
DEPTH_NOISE_M = 0.03
GNSS_NOISE_M = 0.5
# apriltag TF topic -> camera mount (run_tags.sh isolates each camera's tree).
TAG_TOPICS = {
    "/xudrone/tf_front": "xudrone/camera_front",
    "/xudrone/tf": "xudrone/camera_down",
    "/xudrone/tf_up": "xudrone/camera_up",
}

_H = math.sqrt(0.5)
# base_link -> sensor (translation, quaternion w x y z), from model/xudrone/model.sdf.
MOUNTS = {
    SONAR_FRAME: (map_geom.SONAR_ORIGIN,
                  (math.cos(map_geom.SONAR_PITCH / 2), 0.0, math.sin(map_geom.SONAR_PITCH / 2), 0.0)),
    "xudrone/altimeter": ((0.0, 0.0, -0.12), (0.0, _H, 0.0, -_H)),
    "xudrone/surface_ranger": ((0.0, 0.0, 0.12), (0.0, -_H, 0.0, -_H)),
    "xudrone/camera_front": ((0.185, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
    "xudrone/camera_down": ((0.05, 0.0, -0.06), (_H, 0.0, _H, 0.0)),
    "xudrone/camera_up": ((0.0, 0.0, 0.13), (_H, 0.0, -_H, 0.0)),
}
# The bridges stamp Gazebo's scoped names; alias them onto the same mounts.
GZ_ALIASES = {
    "lidar_sensor_link/lidar": "xudrone/altimeter",
    "surface_range_link/surface_lidar": "xudrone/surface_ranger",
    "sonar_link/sonar": SONAR_FRAME,
    "front_camera_link/front_camera": "xudrone/camera_front",
    "down_camera_link/down_camera": "xudrone/camera_down",
    "up_camera_link/up_camera": "xudrone/camera_up",
}


def _tf(parent, child, stamp, t, q):
    m = TransformStamped()
    m.header.stamp = stamp
    m.header.frame_id = parent
    m.child_frame_id = child
    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = (float(v) for v in t)
    (m.transform.rotation.w, m.transform.rotation.x,
     m.transform.rotation.y, m.transform.rotation.z) = (float(v) for v in q)
    return m


def _xyz_cloud(points, frame, stamp):
    m = PointCloud2()
    m.header.frame_id = frame
    m.header.stamp = stamp
    m.height = 1
    m.width = int(len(points))
    m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                for i, n in enumerate("xyz")]
    m.is_bigendian = False
    m.point_step = 12
    m.row_step = 12 * m.width
    m.is_dense = True
    m.data = array.array("B", map_geom.pack_xyz(points))
    return m


class XudroneMapper(Node):
    def __init__(self):
        super().__init__("xudrone_mapper")
        self.model, _inst = detect_target()
        self.gz = GzPoses()
        self.fuser = PoseFuser()
        self.grid = self._load_grid()
        self._vpos = None
        self._odom_q = None
        self._pose = None            # (pos, q, wall time)
        self._boat = None
        self._altitude = None
        self._alt_hits = collections.deque(maxlen=400)
        self._track = collections.deque(maxlen=4000)
        self._local = np.zeros((0, 3), np.float32)
        self._local_t = 0.0
        self._n_sonar = 0
        self._n_obst = 0
        self._save_t = time.time() + 60.0
        self._png_t = 0.0
        self._gm_t = 0.0
        self._cost = None            # costmap at CRUISE_Z, refreshed at 1 Hz
        self._plan = None            # latest coverage plan info (dict) + "mask"
        self._gz_auv = None          # Gazebo truth (pos, q), for emulated sensors and errors
        self._odom_pose = None       # gtsam.Pose3 from PX4 odometry
        self._tags = {}              # id -> (body vector, wall time)
        self.graph = fg.PoseGraph() if fg.available() else None
        self._graph_err = (None, None)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        ns = "/xudrone/fmu/out"
        self.create_subscription(VehicleLocalPosition, f"{ns}/vehicle_local_position", self._on_local, px4_qos)
        self.create_subscription(VehicleOdometry, f"{ns}/vehicle_odometry", self._on_odom, px4_qos)
        self.create_subscription(LaserScan, "/xudrone/range/bottom", self._on_altimeter, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/xudrone/sonar/points_raw", self._on_sonar, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/xudrone/octomap/points", self._on_octomap, 1)
        self.create_subscription(Empty, "/xudrone/map/reset", self._on_reset, 1)
        self.create_subscription(String, "/xudrone/map/plan_coverage", self._on_plan_request, 5)
        for topic, mount in TAG_TOPICS.items():
            self.create_subscription(TFMessage, topic, lambda m, c=mount: self._on_tag_tf(c, m), 20)

        self.tf = TransformBroadcaster(self)
        self.tf_static = StaticTransformBroadcaster(self)
        self.pub_odom = self.create_publisher(Odometry, "/xudrone/odom", 10)
        self.pub_sonar = self.create_publisher(PointCloud2, "/xudrone/sonar/points", 5)
        self.pub_grid = self.create_publisher(OccupancyGrid, "/xudrone/map/grid", latched)
        self.pub_depth = self.create_publisher(OccupancyGrid, "/xudrone/map/bed_depth", latched)
        self.pub_gm = self.create_publisher(GridMap, "/xudrone/map/grid_map", latched) if GridMap else None
        self.pub_local = self.create_publisher(PointCloud2, "/xudrone/map/local_cloud", 2)
        self.pub_track = self.create_publisher(Path, "/xudrone/map/track", latched)
        self.pub_img = self.create_publisher(Image, "/xudrone/map/image", 2)
        self.pub_cost = self.create_publisher(OccupancyGrid, "/xudrone/map/costmap", latched)
        self.pub_plan = self.create_publisher(Path, "/xudrone/map/coverage_plan", latched)
        self.pub_plan_info = self.create_publisher(String, "/xudrone/map/coverage_plan_info", latched)
        self.pub_status = self.create_publisher(String, "/xudrone/map/status", 2)
        self.pub_gpath = self.create_publisher(Path, "/xudrone/graph/path", latched)
        self.pub_gpose = self.create_publisher(PoseWithCovarianceStamped, "/xudrone/graph/pose", 5)
        self.pub_gmark = self.create_publisher(MarkerArray, "/xudrone/graph/markers", latched)

        self._send_static()
        self.create_timer(0.05, self._tick)
        self.create_timer(1.0, self._publish_maps)
        self.get_logger().info(
            f"xudrone mapper: model={self.model} grid {self.grid.n}x{self.grid.n} "
            f"@ {self.grid.res:g} m, saved map {'loaded' if self.grid.bed_n.any() else 'empty'}, "
            f"pose graph {'on (gtsam iSAM2)' if self.graph else 'off (gtsam missing)'}"
        )

    # ---- setup ---------------------------------------------------------
    def _load_grid(self):
        if os.environ.get("UDRONE_MAP_FRESH", "") != "1" and os.path.isfile(SAVE_PATH):
            try:
                g = map_geom.Grid.load(SAVE_PATH)
                if g.res == map_geom.RES_M and g.half == map_geom.HALF_M:
                    return g
            except Exception as exc:
                print(f"MAP_LOAD failed {exc!r}; starting empty", flush=True)
        return map_geom.Grid()

    def _send_static(self):
        now = self.get_clock().now().to_msg()
        out = [_tf(MAP_FRAME, ODOM_FRAME, now, (0, 0, 0), (1, 0, 0, 0))]
        for child, (t, q) in MOUNTS.items():
            out.append(_tf(BASE, child, now, t, q))
        for scoped, mount in GZ_ALIASES.items():
            out.append(_tf(mount, f"{self.model}/{scoped}", now, (0, 0, 0), (1, 0, 0, 0)))
        self.tf_static.sendTransform(out)

    # ---- inputs --------------------------------------------------------
    def _on_local(self, m):
        if m.xy_valid and m.z_valid and m.v_xy_valid and m.v_z_valid:
            self._vpos = (float(m.x), float(m.y), float(m.z),
                          float(m.vx), float(m.vy), float(m.vz), time.time())

    def _on_odom(self, m):
        if len(m.q) >= 4:
            self._odom_q = px4_q_to_enu(m.q)

    def _on_reset(self, _m):
        self.grid = map_geom.Grid()
        self._track.clear()
        self._alt_hits.clear()
        self._plan = None
        self._tags.clear()
        if self.graph is not None:
            self.graph = fg.PoseGraph()
        print("MAP_RESET", flush=True)

    def _on_tag_tf(self, mount, m):
        t_m, q_m = MOUNTS[mount]
        now = time.time()
        for tr in m.transforms:
            if not tr.child_frame_id.startswith(TAG_PREFIX):
                continue
            try:
                tid = int(tr.child_frame_id[len(TAG_PREFIX):])
            except ValueError:
                continue
            t = tr.transform.translation
            if t.z <= 0.0:
                continue
            self._tags[tid] = (fg.optical_to_body((t.x, t.y, t.z), t_m, q_m), now)

    def _on_plan_request(self, m):
        try:
            req = json.loads(m.data or "{}")
        except ValueError:
            req = {}
        info = self._plan_coverage(req)
        print(f"COVERAGE_PLAN id={info.get('id')} ok={info.get('ok')} "
              f"wps={len(info.get('waypoints', []))} length={info.get('length_m')} "
              f"lanes={info.get('lanes')} skipped={info.get('skipped')} {info.get('reason', '')}",
              flush=True)

    def _plan_coverage(self, req):
        pose = self._fresh_pose()
        here = pose[0] if pose else (0.0, 0.0, 0.0)
        cid = req.get("id")
        try:
            center = (float(req.get("center_east_m", here[0])), float(req.get("center_north_m", here[1])))
            width = float(req.get("width_m", 20.0))
            length = float(req.get("length_m", width))
            heading = float(req.get("heading_deg", 0.0))
            swath = float(req.get("swath_m", 3.0))
            z = float(req.get("z", map_geom.CRUISE_Z))
            target = str(req.get("target", "bed"))
            start = np.asarray(req.get("start", here[:2]), float)[:2]
            if width <= 0 or length <= 0 or swath <= 0 or target not in coverage_plan.TARGETS:
                raise ValueError("needs positive width/length/swath and target bed|camera")
        except (TypeError, ValueError) as exc:
            info = {"id": cid, "ok": False, "reason": str(exc), "waypoints": []}
            self.pub_plan_info.publish(String(data=json.dumps(info)))
            return info
        boat_xy = self._boat[0][:2] if self._boat is not None else None
        cost = map_geom.costmap(self.grid, z=z, boat_xy=boat_xy)
        t0 = time.perf_counter()
        out = coverage_plan.plan(self.grid, cost, center, width, length, heading, swath, z,
                                 start=start, target=target)
        region = {"center": list(center), "width_m": width, "length_m": length,
                  "heading_deg": heading, "swath_m": swath, "z": z}
        info = {"id": cid, "ok": bool(out["waypoints"]), "region": region,
                "plan_ms": round(1000.0 * (time.perf_counter() - t0), 1), **out}
        if not out["waypoints"]:
            info["reason"] = "nothing left to cover in the region"
        mask = coverage_plan.region_mask(self.grid, center, width, length, heading)
        mask &= cost < coverage_plan.BLOCK_COST
        self._plan = dict(info, mask=mask)
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = MAP_FRAME
        for x, y, zz in out["waypoints"]:
            ps = PoseStamped()
            ps.header.frame_id = MAP_FRAME
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, zz
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.pub_plan.publish(path)
        self.pub_plan_info.publish(String(data=json.dumps(info)))
        return info

    def _fresh_pose(self):
        if self._pose is None or time.time() - self._pose[2] > POSE_STALE_S:
            return None
        return self._pose

    def _on_altimeter(self, m):
        pose = self._fresh_pose()
        if pose is None or not m.ranges:
            return
        r = float(m.ranges[0])
        if not math.isfinite(r) or r >= 0.99 * float(m.range_max) or r <= float(m.range_min):
            self._altitude = None
            return
        hit = map_geom.beam_hit(pose[0], pose[1], map_geom.ALTIMETER, r)
        self.grid.add_bed(hit)
        self._alt_hits.append(hit.astype(np.float32))
        self._altitude = float(pose[0][2] - hit[2])

    def _on_sonar(self, m):
        pose = self._fresh_pose()
        if pose is None:
            return
        offs = {f.name: f.offset for f in m.fields}
        if not all(k in offs for k in "xyz"):
            return
        pts = map_geom.cloud_xyz(bytes(m.data), m.point_step, m.width * m.height,
                                 (offs["x"], offs["y"], offs["z"]))
        rng = np.linalg.norm(pts, axis=1)
        pts = pts[(rng >= 0.3) & (rng < SONAR_MAX_M)]
        stamp = self.get_clock().now().to_msg()
        self._send_pose_tf(stamp)
        self.pub_sonar.publish(_xyz_cloud(pts, SONAR_FRAME, stamp))
        if len(pts):
            world = map_geom.sonar_to_world(pts.astype(np.float64), pose[0], pose[1])
            boat_xy = self._boat[0][:2] if self._boat is not None else None
            self._n_obst += self.grid.add_sonar(world, boat_xy)
            self._n_sonar += len(pts)

    def _on_octomap(self, m):
        now = time.time()
        if now - self._local_t < 1.0:
            return
        self._local_t = now
        offs = {f.name: f.offset for f in m.fields}
        if not all(k in offs for k in "xyz"):
            return
        self._local = map_geom.cloud_xyz(bytes(m.data), m.point_step, m.width * m.height,
                                         (offs["x"], offs["y"], offs["z"]))

    # ---- pose + TF -----------------------------------------------------
    def _tick(self):
        fresh, _sim_t, (auv, boat) = self.gz.read(self.model, "heron_0")
        gz_pos, gz_q = (auv if (fresh and auv is not None) else (None, None))
        pos, q = self.fuser.fuse(gz_pos, gz_q, self._vpos, self._odom_q)
        self._boat = boat if fresh else None
        self._gz_auv = (gz_pos, gz_q) if gz_pos is not None else None
        if pos is None or q is None:
            return
        self._step_graph(pos, q)
        self._pose = (tuple(pos), tuple(q), time.time())
        stamp = self.get_clock().now().to_msg()
        self._send_pose_tf(stamp)
        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = ODOM_FRAME
        od.child_frame_id = BASE
        p = od.pose.pose
        p.position.x, p.position.y, p.position.z = (float(v) for v in pos)
        p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z = (float(v) for v in q)
        if self.fuser.vel is not None:
            # Odometry twist is in the child (body) frame.
            vb = map_geom.rot(q).T @ np.asarray(self.fuser.vel, float)
            od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z = (float(v) for v in vb)
        self.pub_odom.publish(od)
        self.grid.add_visit(pos)
        self.grid.add_cover(pos, self._altitude)
        if not self._track or math.dist(self._track[-1], pos) >= 0.5:
            self._track.append(tuple(pos))

    def _step_graph(self, pos, q):
        """Cut keyframes and attach factors; iSAM2 updates only when something was added."""
        g = self.graph
        v = self._vpos
        if g is None or v is None or self._odom_q is None or time.time() - v[6] > POSE_STALE_S:
            self._odom_pose = None
            return
        now = time.time()
        odom = fg.pose3(ned_to_enu(v[0], v[1], v[2]), self._odom_q)
        self._odom_pose = odom
        # Only tags that still need a factor on a *new* keyframe. Attaching a
        # bearing to a stale keyframe after the hull has moved biases landmarks.
        tags = {tid: vec for tid, (vec, t) in self._tags.items()
                if now - t <= TAG_FRESH_S and g._tag_kf.get(tid) != g.k}
        want_tag_kf = bool(tags) and g.k >= 0
        added = False
        if g.need_keyframe(odom, now, tag=want_tag_kf):
            # Depth always comes from a vertical reference. Prefer Gazebo truth
            # (emulated pressure); if the gazebo reader blipped, use the fused pose.
            depth_src = self._gz_auv[0][2] if self._gz_auv is not None else pos[2]
            depth = float(depth_src) + random.gauss(0.0, DEPTH_NOISE_M)
            gnss = None
            if self._gz_auv is not None and self._gz_auv[0][2] > fg.SURFACE_Z:
                gz = self._gz_auv[0]
                gnss = [c + random.gauss(0.0, GNSS_NOISE_M) for c in gz[:2]] + [gz[2]]
            g.add_keyframe(odom, fg.pose3(pos, q), now, depth_z=depth, gnss=gnss)
            added = True
        # Tag factors only on the keyframe cut above (same tick), so the body
        # vector matches the pose that carries them.
        if added and tags:
            for tid, vec in tags.items():
                g.add_tag(tid, vec)
        if added:
            try:
                g.update()
            except RuntimeError as exc:
                # Drop the pending batch; keep the optimised graph so a bad
                # factor does not wipe the mission's map.
                print(f"GRAPH update failed {exc!r}; dropping pending factors", flush=True)
                if fg.gtsam is not None:
                    g._new = fg.gtsam.NonlinearFactorGraph()
                    g._init = fg.gtsam.Values()

    def _send_pose_tf(self, stamp):
        if self._pose is None:
            return
        pos, q, _t = self._pose
        out = [_tf(ODOM_FRAME, BASE, stamp, pos, q)]
        if self._boat is not None:
            out.append(_tf(MAP_FRAME, BOAT_FRAME, stamp, self._boat[0], self._boat[1]))
        self.tf.sendTransform(out)

    # ---- products ------------------------------------------------------
    def _grid_msg(self, values, stamp):
        g = self.grid
        m = OccupancyGrid()
        m.header.stamp = stamp
        m.header.frame_id = MAP_FRAME
        m.info.resolution = float(g.res)
        m.info.width = g.n
        m.info.height = g.n
        m.info.origin.position.x = float(g.x_min)
        m.info.origin.position.y = float(g.y_min)
        m.info.origin.orientation.w = 1.0
        m.data = array.array("b", map_geom.occupancy_data(values).tobytes())
        return m

    def _gridmap_msg(self, stamp):
        g = self.grid
        m = GridMap()
        m.header.stamp = stamp
        m.header.frame_id = MAP_FRAME
        m.info.resolution = float(g.res)
        m.info.length_x = float(g.n * g.res)
        m.info.length_y = float(g.n * g.res)
        m.info.pose.position.x = float(g.x_min + g.half)
        m.info.pose.position.y = float(g.y_min + g.half)
        m.info.pose.orientation.w = 1.0
        layers = {
            "bed_z": g.bed_z(),
            "obstacle": (g.obs_n >= map_geom.OBST_MIN_N).astype(np.float32),
            "cover": g.cover.astype(np.float32),
            "visited": g.visited.astype(np.float32),
            "sonar": g.son_n.astype(np.float32),
        }
        m.layers = list(layers)
        m.basic_layers = ["bed_z"]
        for arr in layers.values():
            d = Float32MultiArray()
            d.layout.dim = [
                MultiArrayDimension(label="column_index", size=g.n, stride=g.n * g.n),
                MultiArrayDimension(label="row_index", size=g.n, stride=g.n),
            ]
            d.data = array.array("f", map_geom.gridmap_data(arr).tobytes())
            m.data.append(d)
        m.outer_start_index = 0
        m.inner_start_index = 0
        return m

    def _publish_maps(self):
        stamp = self.get_clock().now().to_msg()
        self.pub_grid.publish(self._grid_msg(self.grid.occupancy(), stamp))
        self.pub_depth.publish(self._grid_msg(self.grid.depth_dm(), stamp))
        boat_xy = self._boat[0][:2] if self._boat is not None else None
        self._cost = map_geom.costmap(self.grid, boat_xy=boat_xy)
        self.pub_cost.publish(self._grid_msg(self._cost, stamp))
        self._publish_graph(stamp)
        self.pub_status.publish(String(data=json.dumps(self._status())))
        now = time.time()
        if self.pub_gm is not None and now - self._gm_t >= 2.0:
            self._gm_t = now
            self.pub_gm.publish(self._gridmap_msg(stamp))
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = MAP_FRAME
        for x, y, z in list(self._track)[-1500:]:
            ps = PoseStamped()
            ps.header.frame_id = MAP_FRAME
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.pub_track.publish(path)
        pose = self._fresh_pose()
        if pose is not None:
            near = self._local
            if len(near):
                near = near[np.linalg.norm(near - np.asarray(pose[0], np.float32), axis=1) <= LOCAL_R_M]
            hits = np.array(self._alt_hits, np.float32).reshape(-1, 3)
            self.pub_local.publish(_xyz_cloud(np.vstack([near, hits]), MAP_FRAME, stamp))
        self._publish_image(stamp)
        if now >= self._save_t:
            self._save_t = now + 60.0
            try:
                os.makedirs(os.path.dirname(SAVE_PATH), exist_ok=True)
                self.grid.save(SAVE_PATH)
            except OSError as exc:
                print(f"MAP_SAVE failed {exc!r}", flush=True)
            if self.graph is not None and self.graph.k >= 0:
                try:
                    self.graph.dump(GRAPH_PATH)
                except (OSError, RuntimeError) as exc:
                    print(f"GRAPH_SAVE failed {exc!r}", flush=True)
            gs = self._status()["graph"]
            bed_cells = int(np.isfinite(self.grid.bed_z()).sum())
            print(
                f"MAP bed_cells={bed_cells} "
                f"(alt={int((self.grid.bed_n > 0).sum())}) "
                f"sonar_cells={int((self.grid.son_n > 0).sum())} "
                f"obstacle_cells={int((self.grid.obs_n >= map_geom.OBST_MIN_N).sum())} "
                f"returns={self._n_sonar} voxels_near={len(self._local)} pose={self.fuser.source} "
                f"graph kf={gs.get('keyframes')} lm={len(gs.get('landmarks', []))} "
                f"err={gs.get('err_m')} odom_err={gs.get('odom_err_m')}",
                flush=True,
            )

    def _publish_graph(self, stamp):
        g = self.graph
        self._graph_err = (None, None)
        if g is None or g.est is None or g.k < 0:
            return
        g.refresh_sigma()
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = MAP_FRAME
        for p in g.kf_pos:
            ps = PoseStamped()
            ps.header.frame_id = MAP_FRAME
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(v) for v in p)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.pub_gpath.publish(path)
        if self._odom_pose is not None:
            now_p = g.now_pose(self._odom_pose)
            dr = g.odom_only(self._odom_pose)
            if self._gz_auv is not None:
                gz = np.asarray(self._gz_auv[0], float)
                self._graph_err = (
                    None if now_p is None else float(np.linalg.norm(now_p.translation() - gz)),
                    None if dr is None else float(np.linalg.norm(dr.translation() - gz)),
                )
            if now_p is not None:
                m = PoseWithCovarianceStamped()
                m.header.stamp = stamp
                m.header.frame_id = MAP_FRAME
                t = now_p.translation()
                qq = now_p.rotation().toQuaternion()
                m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = (float(v) for v in t)
                (m.pose.pose.orientation.w, m.pose.pose.orientation.x,
                 m.pose.pose.orientation.y, m.pose.pose.orientation.z) = qq.w(), qq.x(), qq.y(), qq.z()
                if g.cov is not None:
                    # gtsam (rot, trans) -> ROS (trans, rot)
                    order = [3, 4, 5, 0, 1, 2]
                    m.pose.covariance = [float(v) for v in g.cov[np.ix_(order, order)].reshape(-1)]
                self.pub_gpose.publish(m)
        ma = MarkerArray()
        line = Marker()
        line.header.stamp = stamp
        line.header.frame_id = MAP_FRAME
        line.ns, line.id, line.type = "graph", 0, Marker.LINE_STRIP
        line.scale.x = 0.05
        line.color.r, line.color.g, line.color.b, line.color.a = 0.2, 0.9, 1.0, 1.0
        line.pose.orientation.w = 1.0
        line.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in g.kf_pos]
        ma.markers.append(line)
        for lm in g.landmark_list():
            s = Marker()
            s.header = line.header
            s.ns, s.id, s.type = "landmarks", lm["id"], Marker.SPHERE
            s.pose.position.x, s.pose.position.y, s.pose.position.z = lm["x"], lm["y"], lm["z"]
            s.pose.orientation.w = 1.0
            s.scale.x = s.scale.y = s.scale.z = 0.4
            s.color.g, s.color.a = 1.0, 1.0
            ma.markers.append(s)
            txt = Marker()
            txt.header = line.header
            txt.ns, txt.id, txt.type = "landmark_ids", lm["id"], Marker.TEXT_VIEW_FACING
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = lm["x"], lm["y"], lm["z"] + 0.6
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.4
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text = f"tag {lm['id']} ({lm['obs']})"
            ma.markers.append(txt)
        self.pub_gmark.publish(ma)

    def _status(self):
        g = self.grid
        bed = np.isfinite(g.bed_z())
        status = {
            "t": round(time.time(), 2),
            "map": {
                "res_m": g.res,
                "bed_cells": int(bed.sum()),
                "bed_area_m2": round(float(bed.sum()) * g.res * g.res, 1),
                "obstacle_cells": int((g.obs_n >= map_geom.OBST_MIN_N).sum()),
                "cover_cells": int((g.cover > 0).sum()),
                "box": [round(float(v), 1) for v in g.known_box(margin_m=5.0, min_half_m=15.0,
                        around=self._pose[0] if self._pose else None)],
            },
            "plan": None,
            "graph": {"enabled": self.graph is not None},
        }
        if self._plan is not None:
            p = self._plan
            mask = p["mask"]
            need = coverage_plan.needed(g, p.get("target", "bed"))
            total = int(mask.sum())
            done = int((mask & ~need).sum())
            status["plan"] = {
                "id": p.get("id"), "ok": p.get("ok"), "target": p.get("target"),
                "waypoints": len(p.get("waypoints", [])), "length_m": p.get("length_m"),
                "lanes": p.get("lanes"), "pct": round(100.0 * done / total, 1) if total else None,
                "region": p.get("region"),
            }
        if self.graph is not None:
            gs = self.graph.summary()
            kf = self.graph.kf_pos
            step = max(1, len(kf) // 300)
            gs["kf_xy"] = [[round(float(p[0]), 2), round(float(p[1]), 2)] for p in kf[::step]]
            gs["err_m"] = None if self._graph_err[0] is None else round(self._graph_err[0], 3)
            gs["odom_err_m"] = None if self._graph_err[1] is None else round(self._graph_err[1], 3)
            gs["enabled"] = True
            status["graph"] = gs
        return status

    def _publish_image(self, stamp):
        import cv2

        g = self.grid
        pose = self._fresh_pose()
        x0, y0, x1, y1 = g.known_box(around=pose[0] if pose else None)
        i0, j0, _ = g.index(x0, y0)
        i1, j1, _ = g.index(x1, y1)
        i0, j0 = max(0, int(i0)), max(0, int(j0))
        i1, j1 = min(g.n - 1, int(i1)), min(g.n - 1, int(j1))
        if i1 <= i0 or j1 <= j0:
            return
        sl = (slice(i0, i1 + 1), slice(j0, j1 + 1))
        bed = g.bed_z()[sl]
        known = np.isfinite(bed)
        depth = np.clip(-np.nan_to_num(bed, nan=0.0) / 10.0, 0.0, 1.0)
        rgb = cv2.applyColorMap((255 * (1.0 - depth)).astype(np.uint8), cv2.COLORMAP_VIRIDIS)
        img = np.full(rgb.shape, 30, np.uint8)
        img[known] = rgb[known]
        sonar_only = (g.son_n[sl] > 0) & ~known
        img[sonar_only] = (70, 55, 45)
        cover = g.cover[sl] > 0
        img[cover] = (0.7 * img[cover] + 0.3 * 255).astype(np.uint8)
        img[g.visited[sl] > 0] = (230, 230, 230)
        if self._cost is not None:
            c = self._cost[sl]
            infl = (c > 0) & (c < map_geom.COST_LETHAL)
            img[infl] = (0.6 * img[infl] + 0.4 * np.array([60, 60, 200])).astype(np.uint8)
            img[c == map_geom.COST_LETHAL] = (40, 40, 160)
        img[g.obs_n[sl] >= map_geom.OBST_MIN_N] = (40, 40, 255)
        # [ix, iy] -> north-up rows.
        img = np.ascontiguousarray(img.transpose(1, 0, 2)[::-1])
        h, w = img.shape[:2]
        scale = IMG_PX / max(h, w)
        img = cv2.resize(img, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_NEAREST)
        px_per_m = scale / g.res
        top_y = g.y_min + (j1 + 1) * g.res
        left_x = g.x_min + i0 * g.res

        def pix(x, y):
            return (int((x - left_x) * px_per_m), int((top_y - y) * px_per_m))

        if self._plan is not None and self._plan.get("waypoints"):
            pts = np.array([pix(x, y) for x, y, _z in self._plan["waypoints"]], np.int32)
            cv2.polylines(img, [pts], False, (255, 0, 255), 1, cv2.LINE_AA)
        if self.graph is not None and self.graph.kf_pos:
            pts = np.array([pix(p[0], p[1]) for p in self.graph.kf_pos], np.int32)
            cv2.polylines(img, [pts], False, (255, 220, 60), 1, cv2.LINE_AA)
            for p in pts[:: max(1, len(pts) // 200)]:
                cv2.circle(img, tuple(int(v) for v in p), 2, (255, 220, 60), -1)
            for lm in self.graph.landmark_list():
                c = pix(lm["x"], lm["y"])
                cv2.drawMarker(img, c, (80, 255, 80), cv2.MARKER_DIAMOND, 12, 2)
                cv2.putText(img, f"L{lm['id']}", (c[0] + 7, c[1] - 5), cv2.FONT_HERSHEY_SIMPLEX,
                            0.4, (80, 255, 80), 1, cv2.LINE_AA)

        if pose is not None:
            pos, q = pose[0], pose[1]
            fwd = map_geom.rot(q)[:, 0]
            yaw = math.atan2(fwd[1], fwd[0])
            c = pix(pos[0], pos[1])
            tri = []
            for along, side in ((10, 0), (-6, 5), (-6, -5)):
                tri.append((int(c[0] + along * math.cos(yaw) - side * math.sin(yaw)),
                            int(c[1] - along * math.sin(yaw) - side * math.cos(yaw))))
            cv2.fillPoly(img, [np.array(tri, np.int32)], (0, 255, 255))
        if self._boat is not None:
            b = pix(self._boat[0][0], self._boat[0][1])
            cv2.rectangle(img, (b[0] - 5, b[1] - 5), (b[0] + 5, b[1] + 5), (255, 255, 255), 1)
        bar_m = 10.0 if (x1 - x0) > 30 else 5.0
        y = img.shape[0] - 12
        cv2.line(img, (10, y), (10 + int(bar_m * px_per_m), y), (255, 255, 255), 2)
        cv2.putText(img, f"{bar_m:g} m", (12, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.putText(img, "N", (img.shape[1] // 2 - 5, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(
            img,
            f"xudrone map  bed {int(known.sum())} cells  obstacles "
            f"{int((g.obs_n >= map_geom.OBST_MIN_N).sum())}  depth 0-10 m",
            (8, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA,
        )
        m = Image()
        m.header.stamp = stamp
        m.header.frame_id = MAP_FRAME
        m.height, m.width = img.shape[:2]
        m.encoding = "bgr8"
        m.step = m.width * 3
        m.data = array.array("B", img.tobytes())
        self.pub_img.publish(m)
        if time.time() >= self._png_t:
            self._png_t = time.time() + 5.0
            cv2.imwrite(PNG_PATH, img)

    def destroy_node(self):
        try:
            os.makedirs(os.path.dirname(SAVE_PATH), exist_ok=True)
            self.grid.save(SAVE_PATH)
            if self.graph is not None and self.graph.k >= 0:
                self.graph.dump(GRAPH_PATH)
        except (OSError, RuntimeError):
            pass
        self.gz.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = XudroneMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
