from __future__ import annotations

import math
import sys
import time
import xml.etree.ElementTree as ET
from functools import partial
from pathlib import Path

import cv2
import json
import numpy as np
import pymap3d
from PIL import Image
import socket

from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from shapely.geometry import Point, Polygon

# Locate protobuf definitions relative to this script
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "protobuf"))
import waypoints_pb2

_SCRIPT_DIR = Path(__file__).resolve().parent
_ENGINE_PATH = _SCRIPT_DIR.parent / "yolo_stuff" / "best-v2-shuffled.engine"


# ── HVRP configuration ────────────────────────────────────────────────────────

KML_PATH     = Path(__file__).parent / "Co-GLANCE-2.kml"
AERIAL_DEPOT = (30.3930908, -97.7285992)  # (lat, lon)
GROUND_DEPOT = (30.3930908, -97.7285992)  # (lat, lon)


# ── HVRP constants ────────────────────────────────────────────────────────────

AERIAL_VEHICLE_IDX  = 0
GROUND_VEHICLE_IDX  = 1
NUM_VEHICLES        = 2
AERIAL_DEPOT_IDX    = 0
GROUND_DEPOT_IDX    = 1

EARTH_RADIUS_METERS = 6_378_137.0  # WGS84 equatorial radius

VALID_NODE_TYPES = {"aerial", "ground", "both", "either"}

LARGE_PENALTY    = 1_000_000
N_CANDIDATES     = 8
INITIAL_RADIUS_M = 5.0
AIR_RADIUS_M     = 20.0
RADIUS_STEP_M    = 2.0

KML_NS = "http://www.opengis.net/kml/2.2"


# ── KML parsing ───────────────────────────────────────────────────────────────

def _parse_coord_string(coord_str: str) -> list[tuple[float, float]]:
    pts = []
    for token in coord_str.strip().split():
        parts = token.split(",")
        lon, lat = float(parts[0]), float(parts[1])
        pts.append((lon, lat))
    return pts


def parse_kml(kml_path: Path) -> tuple[Polygon, list[Polygon]]:
    tree = ET.parse(kml_path)
    root = tree.getroot()

    aoi = None
    obstacles: list[Polygon] = []

    for placemark in root.iter(f"{{{KML_NS}}}Placemark"):
        name_el = placemark.find(f"{{{KML_NS}}}name")
        name    = name_el.text.strip() if name_el is not None and name_el.text else ""

        poly_el = placemark.find(f".//{{{KML_NS}}}Polygon")
        if poly_el is None:
            continue

        outer = poly_el.find(
            f".//{{{KML_NS}}}outerBoundaryIs/{{{KML_NS}}}LinearRing/{{{KML_NS}}}coordinates"
        )
        if outer is None or not outer.text:
            continue

        pts = _parse_coord_string(outer.text)
        polygon = Polygon(pts)

        if name.lower() == "area of interest":
            aoi = polygon
        else:
            obstacles.append(polygon)

    if aoi is None:
        raise ValueError("KML file contains no placemark named 'Area of interest'")

    return aoi, obstacles


def _to_mercator_polygon(polygon: Polygon) -> Polygon:
    coords = [gps_to_web_mercator(lat, lon) for lon, lat in polygon.exterior.coords]
    return Polygon(coords)


# ── Node filtering ────────────────────────────────────────────────────────────

def filter_nodes_by_aoi(nodes: list[dict], aoi_mercator: Polygon) -> list[dict]:
    return [n for n in nodes if aoi_mercator.contains(Point(*gps_to_web_mercator(n["lat"], n["lon"])))]


# ── Coordinate projection ─────────────────────────────────────────────────────

def gps_to_web_mercator(lat: float, lon: float) -> tuple[float, float]:
    x = EARTH_RADIUS_METERS * math.radians(lon)
    y = EARTH_RADIUS_METERS * math.log(math.tan(math.pi / 4 + math.radians(lat) / 2))
    return x, y


def web_mercator_to_gps(x: float, y: float) -> tuple[float, float]:
    lon = math.degrees(x / EARTH_RADIUS_METERS)
    lat = math.degrees(2 * math.atan(math.exp(y / EARTH_RADIUS_METERS)) - math.pi / 2)
    return lat, lon


def project_node(node: dict) -> dict:
    x, y = gps_to_web_mercator(node["lat"], node["lon"])
    return {**node, "x": x, "y": y}


# ── Node expansion ────────────────────────────────────────────────────────────

def expand_task_nodes(task_nodes_projected: list[dict]) -> list[dict]:
    routing_locations = []
    for node in task_nodes_projected:
        if node["type"] == "aerial":
            routing_locations.append({**node, "allowed_vehicles": [AERIAL_VEHICLE_IDX]})
        elif node["type"] == "ground":
            routing_locations.append({**node, "allowed_vehicles": [GROUND_VEHICLE_IDX]})
        elif node["type"] == "either":
            routing_locations.append({**node, "allowed_vehicles": None})
        elif node["type"] == "both":
            routing_locations.append({**node, "name": node["name"] + "_aerial",
                                      "allowed_vehicles": [AERIAL_VEHICLE_IDX]})
            routing_locations.append({**node, "name": node["name"] + "_ground",
                                      "allowed_vehicles": [GROUND_VEHICLE_IDX]})
    return routing_locations


# ── Distance utilities ────────────────────────────────────────────────────────

def euclidean_distance_meters(a: dict, b: dict) -> float:
    return math.sqrt((a["x"] - b["x"]) ** 2 + (a["y"] - b["y"]) ** 2)


def build_distance_matrix(locations: list[dict]) -> list[list[int]]:
    n = len(locations)
    return [
        [round(euclidean_distance_meters(locations[i], locations[j])) for j in range(n)]
        for i in range(n)
    ]


# ── Distance callbacks ────────────────────────────────────────────────────────

def _aerial_distance_callback(from_routing_index, to_routing_index, manager, distance_matrix):
    from_node = manager.IndexToNode(from_routing_index)
    to_node   = manager.IndexToNode(to_routing_index)
    return distance_matrix[from_node][to_node]


def _ground_distance_callback(from_routing_index, to_routing_index, manager, distance_matrix):
    from_node = manager.IndexToNode(from_routing_index)
    to_node   = manager.IndexToNode(to_routing_index)
    return 10 * distance_matrix[from_node][to_node]


# ── HVRP routing model ────────────────────────────────────────────────────────

def _build_routing_model(distance_matrix, expanded_task_nodes, time_limit_seconds):
    manager = pywrapcp.RoutingIndexManager(
        len(distance_matrix),
        NUM_VEHICLES,
        [AERIAL_DEPOT_IDX, GROUND_DEPOT_IDX],
        [AERIAL_DEPOT_IDX, GROUND_DEPOT_IDX],
    )
    routing = pywrapcp.RoutingModel(manager)

    aerial_cb = partial(_aerial_distance_callback, manager=manager, distance_matrix=distance_matrix)
    ground_cb = partial(_ground_distance_callback, manager=manager, distance_matrix=distance_matrix)

    aerial_transit_idx = routing.RegisterTransitCallback(aerial_cb)
    ground_transit_idx = routing.RegisterTransitCallback(ground_cb)
    routing.SetArcCostEvaluatorOfVehicle(aerial_transit_idx, AERIAL_VEHICLE_IDX)
    routing.SetArcCostEvaluatorOfVehicle(ground_transit_idx, GROUND_VEHICLE_IDX)

    for task_node in expanded_task_nodes:
        routing_index = manager.NodeToIndex(task_node["location_index"])
        if task_node["allowed_vehicles"] is not None:
            routing.VehicleVar(routing_index).SetValues(task_node["allowed_vehicles"])

    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_params.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_params.time_limit.seconds = time_limit_seconds

    return manager, routing, search_params


def _extract_hvrp_route(vehicle_idx, solution, routing, manager, all_locations):
    stops, total_distance = [], 0
    routing_index = routing.Start(vehicle_idx)
    while not routing.IsEnd(routing_index):
        node_index = manager.IndexToNode(routing_index)
        stops.append(all_locations[node_index])
        next_routing_index = solution.Value(routing.NextVar(routing_index))
        total_distance += routing.GetArcCostForVehicle(
            routing_index, next_routing_index, vehicle_idx
        )
        routing_index = next_routing_index
    stops.append(all_locations[manager.IndexToNode(routing_index)])
    return stops, total_distance


# ── Candidate generation with obstacle avoidance ──────────────────────────────

def _generate_candidates(target: dict, target_idx: int, radius_m: float,
                         n: int = N_CANDIDATES) -> list[dict]:
    cx, cy = gps_to_web_mercator(target["lat"], target["lon"])
    candidates = []
    for i in range(n):
        angle = 2 * math.pi * i / n
        px = cx + radius_m * math.cos(angle)
        py = cy + radius_m * math.sin(angle)
        lat, lon = web_mercator_to_gps(px, py)
        candidates.append({
            "name":       f"{target['name']}_c{i}",
            "lat":        lat,
            "lon":        lon,
            "x":          px,
            "y":          py,
            "target_idx": target_idx,
        })
    return candidates


def _generate_valid_candidates(target: dict, target_idx: int,
                               obstacles: list[Polygon],
                               n_candidates: int) -> tuple[list[dict], float]:
    radius = INITIAL_RADIUS_M
    while True:
        all_cands = _generate_candidates(target, target_idx, radius, n_candidates)
        valid = [
            c for c in all_cands
            if not any(obs.contains(Point(c["x"], c["y"])) for obs in obstacles)
        ]
        if valid:
            return valid, radius
        radius += RADIUS_STEP_M


# ── Area routing model (single-vehicle TSP with disjunctions) ─────────────────

def _area_distance_callback(from_idx, to_idx, manager, distance_matrix):
    return distance_matrix[manager.IndexToNode(from_idx)][manager.IndexToNode(to_idx)]


def _build_area_routing_model(depot_projected: dict, all_candidates: list[dict],
                               candidate_groups: list[list[int]], time_limit_seconds: int):
    all_nodes       = [depot_projected] + all_candidates
    distance_matrix = build_distance_matrix(all_nodes)

    manager = pywrapcp.RoutingIndexManager(len(all_nodes), 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    cb_idx = routing.RegisterTransitCallback(
        partial(_area_distance_callback, manager=manager, distance_matrix=distance_matrix)
    )
    routing.SetArcCostEvaluatorOfAllVehicles(cb_idx)

    for group in candidate_groups:
        routing_indices = [manager.NodeToIndex(i) for i in group]
        routing.AddDisjunction(routing_indices, LARGE_PENALTY, 1)

    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_params.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_params.time_limit.seconds = time_limit_seconds

    return manager, routing, search_params, all_nodes


def _extract_area_route(solution, routing, manager, all_nodes: list[dict]):
    stops, total_distance = [], 0
    idx = routing.Start(0)
    while not routing.IsEnd(idx):
        stops.append(all_nodes[manager.IndexToNode(idx)])
        next_idx = solution.Value(routing.NextVar(idx))
        total_distance += routing.GetArcCostForVehicle(idx, next_idx, 0)
        idx = next_idx
    stops.append(all_nodes[manager.IndexToNode(idx)])
    return stops, total_distance


# ── Observation vectors ───────────────────────────────────────────────────────

def _compute_observation_vectors(stops: list[dict],
                                 targets_projected: list[dict],
                                 north_zero_cw: bool = False) -> list[dict]:
    """Yaw convention depends on north_zero_cw:
      False (default): 0 = south, increases CCW. Formula: atan2(dx, -dy).
      True:            0 = north, increases CW.  Formula: atan2(dx,  dy).
    dx/dy are Web Mercator offsets (x=East, y=North), candidate→target."""
    vectors = []
    for stop in stops:
        if "target_idx" not in stop:
            continue
        tgt = targets_projected[stop["target_idx"]]
        dx  = tgt["x"] - stop["x"]
        dy  = tgt["y"] - stop["y"]
        yaw = math.atan2(dx, dy) if north_zero_cw else math.atan2(dx, -dy)
        vectors.append({
            "candidate": stop["name"],
            "lat":       stop["lat"],
            "lon":       stop["lon"],
            "target":    tgt["name"],
            "cx": stop["x"], "cy": stop["y"],
            "tx": tgt["x"],  "ty": tgt["y"],
            "dx": dx, "dy": dy,
            "yaw_rad":   yaw,
        })
    return vectors


# ── Aerial fixed-sequence route ───────────────────────────────────────────────

def _build_aerial_stops(depot_projected: dict, aerial_targets_projected: list[dict],
                        radius_m: float, n_candidates: int) -> tuple[list[dict], int]:
    stops = [depot_projected]
    for t_idx, target in enumerate(aerial_targets_projected):
        stops.extend(_generate_candidates(target, t_idx, radius_m, n_candidates))
    stops.append(depot_projected)
    total = sum(
        round(euclidean_distance_meters(stops[i], stops[i + 1]))
        for i in range(len(stops) - 1)
    )
    return stops, total


# ── HVRP solver ───────────────────────────────────────────────────────────────

def solve_hvrp(
    waypoints: list[tuple[float, float, str]],
    aerial_depot: tuple[float, float] = AERIAL_DEPOT,
    ground_depot: tuple[float, float] = GROUND_DEPOT,
    time_limit: int = 10,
    n_candidates: int = N_CANDIDATES,
) -> tuple[list[tuple[float, float, float]], list[tuple[float, float, float]]]:
    """Solve the HVRP and return observation waypoints for each vehicle.

    Args:
        waypoints:     Target locations as (lat, lon, label) triples. The label
                       must be one of 'aerial', 'ground', 'both', or 'either';
                       unrecognised labels default to 'either'.
        aerial_depot:  (lat, lon) start/end for the aerial vehicle.
        ground_depot:  (lat, lon) start/end for the ground vehicle.
        time_limit:    OR-Tools solver time limit in seconds.
        n_candidates:  Observation candidate positions generated per target.

    Returns:
        (aerial_waypoints, ground_waypoints) — each a list of (lat, lon, yaw_rad).
        Yaw convention: 0 = south, increases CCW, pointing candidate → target.
        Aerial waypoints follow a fixed depot → ring-orbit sequence.
        Ground waypoints are TSP-optimised with obstacle avoidance.
    """
    aoi_gps, obstacles_gps = parse_kml(KML_PATH)
    aoi_mercator       = _to_mercator_polygon(aoi_gps)
    obstacles_mercator = [_to_mercator_polygon(o) for o in obstacles_gps]

    task_nodes = [
        {"name": f"wp_{i}", "lat": lat, "lon": lon,
         "type": label if label in VALID_NODE_TYPES else "either"}
        for i, (lat, lon, label) in enumerate(waypoints)
    ]
    task_nodes = filter_nodes_by_aoi(task_nodes, aoi_mercator)

    aerial_depot_dict = {"name": "Aerial Depot", "lat": aerial_depot[0], "lon": aerial_depot[1]}
    ground_depot_dict = {"name": "Ground Depot", "lat": ground_depot[0], "lon": ground_depot[1]}

    aerial_depot_proj = project_node(aerial_depot_dict)
    ground_depot_proj = project_node(ground_depot_dict)
    task_nodes_proj   = [project_node(n) for n in task_nodes]

    expanded_task_nodes = expand_task_nodes(task_nodes_proj)
    for pos, node in enumerate(expanded_task_nodes):
        node["location_index"] = GROUND_DEPOT_IDX + 1 + pos

    all_locations   = [aerial_depot_proj, ground_depot_proj] + expanded_task_nodes
    distance_matrix = build_distance_matrix(all_locations)

    manager, routing, search_params = _build_routing_model(
        distance_matrix, expanded_task_nodes, time_limit
    )
    solution = routing.SolveWithParameters(search_params)

    def _interior_targets(vehicle_idx: int) -> list[dict]:
        if solution is None:
            return []
        stops, _ = _extract_hvrp_route(vehicle_idx, solution, routing, manager, all_locations)
        return [{"name": s["name"], "lat": s["lat"], "lon": s["lon"]} for s in stops[1:-1]]

    ground_targets = _interior_targets(GROUND_VEHICLE_IDX)
    aerial_targets = _interior_targets(AERIAL_VEHICLE_IDX)

    # Ground: obstacle-aware candidates + TSP
    ground_targets_proj      = [project_node(t) for t in ground_targets]
    ground_all_candidates:   list[dict]       = []
    ground_candidate_groups: list[list[int]]  = []

    for t_idx, target in enumerate(ground_targets_proj):
        valid_cands, _ = _generate_valid_candidates(
            target, t_idx, obstacles_mercator, n_candidates
        )
        start = len(ground_all_candidates) + 1
        ground_candidate_groups.append(list(range(start, start + len(valid_cands))))
        ground_all_candidates.extend(valid_cands)

    ground_obs_vectors: list[dict] = []
    if ground_targets:
        g_manager, g_routing, g_params, g_all_nodes = _build_area_routing_model(
            ground_depot_proj, ground_all_candidates, ground_candidate_groups, time_limit
        )
        g_solution = g_routing.SolveWithParameters(g_params)
        if g_solution is not None:
            ground_stops, _ = _extract_area_route(g_solution, g_routing, g_manager, g_all_nodes)
            ground_obs_vectors = _compute_observation_vectors(ground_stops, ground_targets_proj, north_zero_cw=True)

    # Aerial: fixed-sequence ring orbit
    aerial_targets_proj = [project_node(t) for t in aerial_targets]
    aerial_stops, _     = _build_aerial_stops(
        aerial_depot_proj, aerial_targets_proj, AIR_RADIUS_M, n_candidates
    )
    aerial_obs_vectors = _compute_observation_vectors(aerial_stops, aerial_targets_proj)

    aerial_waypoints = [(v["lat"], v["lon"], AIR_RADIUS_M, v["yaw_rad"]) for v in aerial_obs_vectors]
    ground_waypoints = [(v["lat"], v["lon"], v["yaw_rad"]) for v in ground_obs_vectors]

    return aerial_waypoints, ground_waypoints


def masks_to_gps(
    masks,
    image,
    gps,
    attitude,
    labels=None,
    camera_angle: float = 45.0,
) -> list[tuple[float, float, str]]:
    """
    Project each mask's centroid pixel to a GPS coordinate on the ground plane.

    Parameters
    ----------
    masks : list of (H, W) array-like
        Binary (bool or uint8) masks, one per detected object.
    image : PIL.Image or (H, W, 3) ndarray
        Camera image — used only to determine pixel dimensions.
    gps : dict or sequence
        Drone GPS. Dict keys: 'latitude', 'longitude', 'altitude' (metres MSL).
        Sequence form: [lat, lon, alt].
    attitude : dict or sequence
        Drone attitude. Dict form: {'quaternion': {'w','x','y','z'}}.
        Sequence form: [w, x, y, z].
    labels : list of str, optional
        Class label for each mask. Defaults to None per mask if not provided.
    camera_angle : float
        Degrees the camera optical axis is tilted *below* horizontal toward
        the body +X (forward) axis. Default 45°.

    Returns
    -------
    list of (lat, lon, label) tuples, one per mask (in the same order as `masks`).
    Masks whose centroid ray points upward (no ground intersection) produce
    a (None, None, label) entry.
    """
    # ── Parse inputs ─────────────────────────────────────────────────────────
    if isinstance(gps, dict):
        lat = gps["latitude"]
        lon = gps["longitude"]
        alt = gps["altitude"]
    else:
        lat, lon, alt = float(gps[0]), float(gps[1]), float(gps[2])

    if isinstance(attitude, dict):
        q = attitude["quaternion"]
        q_wxyz = np.array([q["w"], q["x"], q["y"], q["z"]], dtype=float)
    else:
        q_wxyz = np.asarray(attitude, dtype=float)

    if isinstance(image, Image.Image):
        img_w, img_h = image.size
    else:
        img_h, img_w = np.asarray(image).shape[:2]

    drone_pos = [lat, lon, alt]

    # ── Camera intrinsics (csi_cam_0 calibration) ────────────────────────────
    fx, fy = 1217.642102546154, 1205.813741470882
    cx_px, cy_px = 539.4282560125398, 352.5539369151344

    # ── Camera-to-body rotation matrix ───────────────────────────────────────
    # Camera optical axis is tilted `camera_angle`° below body +X (forward).
    # Body frame: X=Forward, Y=Left, Z=Up  (FLU)
    # Camera frame: Z=optical axis, X=image right, Y=image down
    theta = math.radians(camera_angle)
    cam_x_in_body = np.array([0.0, -1.0, 0.0])                          # image right → body -Y
    cam_z_in_body = np.array([math.cos(theta), 0.0, -math.sin(theta)])  # optical axis
    cam_y_in_body = np.cross(cam_z_in_body, cam_x_in_body)              # image down
    R_cam_to_body = np.column_stack([cam_x_in_body, cam_y_in_body, cam_z_in_body])

    if labels is None:
        labels = [None] * len(masks)

    results = []
    for mask, label in zip(masks, labels):
        mask_arr = np.asarray(mask, dtype=bool)
        ys, xs = np.where(mask_arr)

        if len(xs) == 0:
            results.append((None, None, label))
            continue

        u = float(xs.mean())
        v = float(ys.mean())

        try:
            gps_lat, gps_lon = _pixel_to_gps(
                u, v, drone_pos, q_wxyz,
                fx, fy, cx_px, cy_px,
                R_cam_to_body,
            )
        except ValueError:
            gps_lat, gps_lon = None, None

        results.append((gps_lat, gps_lon, label))

    return results

def proto_sender(points, spot_addr=("192.168.2.26", 9000)):
    msg = waypoints_pb2.WaypointList()
    msg.timestamp_ns = time.time_ns()

    for lat, lon, yaw in points:
        wp = msg.waypoints.add()
        wp.lat, wp.lon, wp.yaw = lat, lon, yaw

    payload = msg.SerializeToString()

    # TCP — recommended for a waypoint list (delivery matters)
    with socket.create_connection(spot_addr) as s:
        # length-prefix so the dog knows where the message ends
        s.sendall(len(payload).to_bytes(4, "big") + payload)
# ── Helpers ──────────────────────────────────────────────────────────

def _quat_rotate(q_wxyz, v):
    w, x, y, z = q_wxyz
    qv = np.array([x, y, z])
    t = 2.0 * np.cross(qv, v)
    return v + w * t + np.cross(qv, t)


def _pixel_to_gps(u, v, drone_pos, q_wxyz, fx, fy, cx_px, cy_px, R_cam_to_body):
    ray_cam = np.array([(u - cx_px) / fx, (v - cy_px) / fy, 1.0])
    ray_body = R_cam_to_body @ ray_cam
    ray_enu = _quat_rotate(q_wxyz, ray_body)
    ray_enu /= np.linalg.norm(ray_enu)

    if ray_enu[2] >= 0.0:
        raise ValueError("Ray points upward — no ground intersection")

    t = -drone_pos[2] / ray_enu[2]
    hit_enu = t * ray_enu

    target_lat, target_lon, _ = pymap3d.enu2geodetic(
        hit_enu[0], hit_enu[1], hit_enu[2],
        drone_pos[0], drone_pos[1], drone_pos[2],
    )
    return target_lat, target_lon


# ── ROS pipeline ──────────────────────────────────────────────────────────────

def _patch_torchvision_nms_for_jetson():
    try:
        import torch
        from torchvision.ops import nms
        boxes = torch.rand(2, 4, device="cuda")
        scores = torch.rand(2, device="cuda")
        nms(boxes, scores, 0.5)
    except Exception:
        import types
        from ultralytics.utils.nms import TorchNMS
        ops = types.SimpleNamespace(
            nms=lambda boxes, scores, iou_threshold: TorchNMS.nms(boxes, scores, iou_threshold)
        )
        tv = types.ModuleType("torchvision")
        tv.ops = ops
        sys.modules["torchvision"] = tv
        sys.modules["torchvision.ops"] = ops


def _grab_frame(bridge, timeout=10.0):
    from sensor_msgs.msg import Image as RosImage
    import rospy
    msg = rospy.wait_for_message("/csi_cam_0/image_raw", RosImage, timeout=timeout)
    return bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


def _grab_gps(timeout=10.0):
    from sensor_msgs.msg import NavSatFix
    import rospy
    msg = rospy.wait_for_message("/dji_sdk/gps_position", NavSatFix, timeout=timeout)
    return [msg.latitude, msg.longitude, msg.altitude]


def _grab_attitude(timeout=10.0):
    from geometry_msgs.msg import QuaternionStamped
    import rospy
    msg = rospy.wait_for_message("/dji_sdk/attitude", QuaternionStamped, timeout=timeout)
    q = msg.quaternion
    return [q.w, q.x, q.y, q.z]


def _masks_from_result(result, frame_shape):
    """Return (masks, labels) from an Ultralytics result.

    masks  : list of (H, W) uint8 binary arrays
    labels : list of str class names, one per mask
    """
    masks, labels = [], []
    if result.masks is None:
        return masks, labels
    h, w = frame_shape[:2]
    cls_ids = result.boxes.cls.tolist() if result.boxes is not None else []
    names = result.names  # dict: int → str
    for i, poly in enumerate(result.masks.xy):
        binary = np.zeros((h, w), dtype=np.uint8)
        pts = np.asarray(poly, dtype=np.int32).reshape(-1, 1, 2)
        if len(pts) >= 3:
            cv2.fillPoly(binary, [pts], 1)
        masks.append(binary)
        label = names[int(cls_ids[i])] if i < len(cls_ids) else None
        labels.append(label)
    return masks, labels


def _run_yolo_subprocess(frame: np.ndarray) -> tuple[list, list, np.ndarray]:
    """Run YOLO in a child process so TensorRT fully releases GPU memory on exit."""
    import subprocess
    import tempfile
    import json

    with tempfile.TemporaryDirectory() as tmpdir:
        tmp = Path(tmpdir)
        np.savez_compressed(str(tmp / "frame.npz"), frame=frame)

        script = f"""
import sys, json
import numpy as np
import cv2

try:
    import torch
    from torchvision.ops import nms
    boxes = torch.rand(2, 4, device='cuda')
    scores = torch.rand(2, device='cuda')
    nms(boxes, scores, 0.5)
except Exception:
    import types
    try:
        from ultralytics.utils.nms import TorchNMS
        ops = types.SimpleNamespace(nms=lambda b, s, i: TorchNMS.nms(b, s, i))
    except Exception:
        ops = types.SimpleNamespace(nms=None)
    tv = types.ModuleType('torchvision')
    tv.ops = ops
    sys.modules['torchvision'] = tv
    sys.modules['torchvision.ops'] = ops

frame = np.load({repr(str(tmp / 'frame.npz'))})['frame']
from ultralytics import YOLO
model = YOLO({repr(str(_ENGINE_PATH))}, task='segment')
for _ in range(3):
    model.predict(source=frame, imgsz=640, device='0', half=True, verbose=False)
result = model.predict(source=frame, imgsz=640, device='0', half=True, verbose=False)[0]

masks, labels = [], []
if result.masks is not None:
    h, w = frame.shape[:2]
    cls_ids = result.boxes.cls.tolist() if result.boxes is not None else []
    names = result.names
    for i, poly in enumerate(result.masks.xy):
        binary = np.zeros((h, w), dtype=np.uint8)
        pts = np.asarray(poly, dtype=np.int32).reshape(-1, 1, 2)
        if len(pts) >= 3:
            cv2.fillPoly(binary, [pts], 1)
        masks.append(binary)
        label = names[int(cls_ids[i])] if i < len(cls_ids) else None
        labels.append(label)

yolo_img = result.plot()
cv2.imwrite({repr(str(tmp / 'yolo_output.jpg'))}, yolo_img)
np.save({repr(str(tmp / 'mask_count.npy'))}, np.array([len(masks)]))
if masks:
    np.savez_compressed({repr(str(tmp / 'masks.npz'))}, **{{f'm{{i}}': m for i, m in enumerate(masks)}})
with open({repr(str(tmp / 'labels.json'))}, 'w') as f:
    json.dump(labels, f)
print(f"YOLO: {{len(masks)}} detection(s)")
"""
        proc = subprocess.run(
            [sys.executable, "-c", script],
            capture_output=True, text=True, timeout=300,
        )
        if proc.stdout:
            print(proc.stdout.strip())
        if proc.returncode != 0:
            print(proc.stderr)
            raise RuntimeError(f"YOLO subprocess failed (exit {proc.returncode})")

        mask_count = int(np.load(str(tmp / "mask_count.npy"))[0])
        masks = []
        if mask_count > 0:
            data = np.load(str(tmp / "masks.npz"))
            masks = [data[f"m{i}"] for i in range(mask_count)]
        with open(str(tmp / "labels.json")) as f:
            labels = json.load(f)
        yolo_img = cv2.imread(str(tmp / "yolo_output.jpg"))

    return masks, labels, yolo_img


def _process_and_save(
    frame,
    gps: list[float],
    attitude: list[float],
) -> tuple[list, list]:
    """Run inference, save outputs to _DEBUG_DIR, solve HVRP, return (drone_wps, spot_wps)."""
    masks, labels, yolo_img = _run_yolo_subprocess(frame)
    print(f"Detected {len(masks)} object(s)")

    out = _DEBUG_DIR
    out.mkdir(parents=True, exist_ok=True)

    cv2.imwrite(str(out / "yolo_output.jpg"), yolo_img)
    print(f"  yolo_output.jpg: {len(masks)} detection(s)")

    gps_waypoints = masks_to_gps(masks, frame, gps, attitude, labels=labels)
    valid_wps = [(lat, lon, label) for lat, lon, label in gps_waypoints if lat is not None]
    print(f"Valid GPS waypoints ({len(valid_wps)}): {valid_wps}")

    proj_path = out / "projected_waypoints.txt"
    proj_path.write_text("\n".join(f"{lat},{lon},{label}" for lat, lon, label in valid_wps) + "\n")
    print(f"  projected_waypoints.txt: {len(valid_wps)} waypoint(s)")

    drone_wps, spot_wps = solve_hvrp(valid_wps)
    _save_waypoints(drone_wps, out / "drone_waypoints.txt")
    _save_waypoints(spot_wps,  out / "spot_waypoints.txt")

    w, x, y, z = attitude
    yaw_rad = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    uav_heading = math.pi / 2 - yaw_rad

    img_corners = _image_corners_to_gps(frame.shape, gps, attitude)
    print(f"  Image corners (TL TR BR BL): {img_corners}")

    save_satellite_waypoint_map(
        drone_wps, spot_wps, out / "satellite_map.png",
        projected_waypoints=[(la, lo, lbl) for la, lo, lbl in valid_wps],
        drone_pos=(gps[0], gps[1], uav_heading),
        image_corners=img_corners,
    )
    print(f"Saved to {out}/")
    return drone_wps, spot_wps


def run_pipeline():
    import rospy
    from cv_bridge import CvBridge

    rospy.init_node("drone_pipeline", anonymous=True)
    rospy.loginfo("Grabbing frame, GPS, and attitude from ROS topics...")
    bridge = CvBridge()
    frame = _grab_frame(bridge)
    gps   = _grab_gps()
    attitude = _grab_attitude()
    rospy.loginfo(f"Frame shape: {frame.shape}  GPS: {gps}")

    drone_wps, spot_wps = _process_and_save(frame, gps, attitude)

    proto_sender(spot_wps)
    rospy.loginfo("Spot waypoints sent.")


def _save_waypoints(waypoints: list[tuple], path: Path) -> None:
    path.write_text("\n".join(",".join(str(v) for v in wp) for wp in waypoints) + "\n")
    print(f"  {path.name}: {len(waypoints)} waypoint(s)")


def save_satellite_waypoint_map(
    aerial_waypoints: list[tuple[float, float, float]],
    ground_waypoints: list[tuple[float, float, float]],
    out_path: Path,
    projected_waypoints: list[tuple[float, float, str]] | None = None,
    drone_pos: tuple[float, float, float] | None = None,  # (lat, lon, heading_rad) 0=N CW
    image_corners: list[tuple[float, float] | None] | None = None,  # TL TR BR BL
    zoom: int = 18,
    pad_tiles: int = 1,
    arrow_m: float = 4.0,
) -> None:
    """Fetch a satellite basemap and plot aerial/ground waypoints onto it."""
    import io
    import requests
    import matplotlib.pyplot as plt
    from PIL import Image as _PILImage

    # Guard: if the live drone position is >100 m from the hardcoded depot the
    # depot constants are stale (e.g. testing in lab vs. field site) and the map
    # would span 8+ miles of useless tiles.
    if drone_pos is not None:
        def _haversine_m(lat1, lon1, lat2, lon2):
            dlat = math.radians(lat2 - lat1)
            dlon = math.radians(lon2 - lon1)
            a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
            return 2 * EARTH_RADIUS_METERS * math.asin(math.sqrt(a))

        dist_aerial = _haversine_m(drone_pos[0], drone_pos[1], AERIAL_DEPOT[0], AERIAL_DEPOT[1])
        dist_ground = _haversine_m(drone_pos[0], drone_pos[1], GROUND_DEPOT[0], GROUND_DEPOT[1])
        if dist_aerial > 100 or dist_ground > 100:
            print(
                f"  Skipping satellite map: live GPS is {dist_aerial:.0f} m from aerial depot "
                f"and {dist_ground:.0f} m from ground depot (hardcoded depots likely out of date)."
            )
            return

    TILE_SIZE  = 256
    DRONE_CLR  = "royalblue"
    SPOT_CLR   = "limegreen"
    PROJ_CLR   = "orange"
    UAV_CLR    = "magenta"

    def _ll_to_tile(lat, lon):
        n = 2 ** zoom
        tx = int((lon + 180) / 360 * n)
        lat_r = math.radians(lat)
        ty = int((1 - math.log(math.tan(lat_r) + 1 / math.cos(lat_r)) / math.pi) / 2 * n)
        return tx, ty

    def _ll_to_px(lat, lon, tx_min, ty_min):
        n = 2 ** zoom
        px = ((lon + 180) / 360 * n - tx_min) * TILE_SIZE
        lat_r = math.radians(lat)
        py = ((1 - math.log(math.tan(lat_r) + 1 / math.cos(lat_r)) / math.pi) / 2 * n - ty_min) * TILE_SIZE
        return px, py

    def _yaw_tip(lat, lon, yaw_rad, metres):
        d_n = metres * -math.cos(yaw_rad)
        d_e = metres *  math.sin(yaw_rad)
        return (lat  + math.degrees(d_n / EARTH_RADIUS_METERS),
                lon  + math.degrees(d_e / (EARTH_RADIUS_METERS * math.cos(math.radians(lat)))))

    all_pts = ([(wp[0], wp[1]) for wp in aerial_waypoints]
             + [(wp[0], wp[1]) for wp in ground_waypoints]
             + [(la, lo) for la, lo, _ in (projected_waypoints or [])])
    all_pts += [AERIAL_DEPOT, GROUND_DEPOT]
    if drone_pos is not None:
        all_pts.append((drone_pos[0], drone_pos[1]))
    if image_corners:
        all_pts += [pt for pt in image_corners if pt is not None]

    all_lats = [p[0] for p in all_pts]
    all_lons = [p[1] for p in all_pts]

    txs = [_ll_to_tile(la, lo)[0] for la, lo in zip(all_lats, all_lons)]
    tys = [_ll_to_tile(la, lo)[1] for la, lo in zip(all_lats, all_lons)]
    tx_min, tx_max = min(txs) - pad_tiles, max(txs) + pad_tiles
    ty_min, ty_max = min(tys) - pad_tiles, max(tys) + pad_tiles

    cols = tx_max - tx_min + 1
    rows = ty_max - ty_min + 1
    canvas = _PILImage.new("RGB", (cols * TILE_SIZE, rows * TILE_SIZE))
    print(f"  Fetching {cols * rows} satellite tile(s) at zoom {zoom}…")
    for ty in range(ty_min, ty_max + 1):
        for tx in range(tx_min, tx_max + 1):
            url = (f"https://server.arcgisonline.com/ArcGIS/rest/services"
                   f"/World_Imagery/MapServer/tile/{zoom}/{ty}/{tx}")
            try:
                resp = requests.get(url, timeout=10, headers={"User-Agent": "Mozilla/5.0"})
                resp.raise_for_status()
                tile = _PILImage.open(io.BytesIO(resp.content)).convert("RGB")
            except Exception as exc:
                print(f"    tile {tx},{ty} failed: {exc}")
                tile = _PILImage.new("RGB", (TILE_SIZE, TILE_SIZE), (60, 60, 60))
            canvas.paste(tile, ((tx - tx_min) * TILE_SIZE, (ty - ty_min) * TILE_SIZE))

    img = np.array(canvas)
    del canvas  # free PIL copy before matplotlib allocates
    fig, ax = plt.subplots(figsize=(8, 8), dpi=100)
    ax.imshow(img, origin="upper")
    del img  # matplotlib has its own copy now
    ax.set_axis_off()

    def _draw_vehicle(wps, color, label):
        if not wps:
            return
        pxs = [_ll_to_px(wp[0], wp[1], tx_min, ty_min)[0] for wp in wps]
        pys = [_ll_to_px(wp[0], wp[1], tx_min, ty_min)[1] for wp in wps]
        ax.plot(pxs, pys, color=color, linewidth=1.2, linestyle="--", alpha=0.6)
        for i, (wp, px, py) in enumerate(zip(wps, pxs, pys), start=1):
            la, lo, yaw = wp[0], wp[1], wp[-1]
            ax.scatter(px, py, color=color, s=20, zorder=4,
                       edgecolors="white", linewidths=0.5)
            ax.annotate(str(i), (px, py), textcoords="offset points", xytext=(3, 2),
                        fontsize=6, fontweight="bold", color="white",
                        bbox=dict(boxstyle="round,pad=0.12", facecolor=color,
                                  edgecolor="none", alpha=0.85))
            tip = _yaw_tip(la, lo, yaw, arrow_m)
            tip_px, tip_py = _ll_to_px(*tip, tx_min, ty_min)
            ax.annotate("", xy=(tip_px, tip_py), xytext=(px, py),
                        arrowprops=dict(arrowstyle="-|>", color=color,
                                        lw=1.2, mutation_scale=7))
        ax.scatter([], [], color=color, s=20, edgecolors="white",
                   linewidths=0.5, label=f"{label} ({len(wps)})")

    def _draw_projected(wps, color, label):
        if not wps:
            return
        for i, (la, lo, cls) in enumerate(wps, start=1):
            px, py = _ll_to_px(la, lo, tx_min, ty_min)
            ax.scatter(px, py, color=color, s=50, zorder=5, marker="*",
                       edgecolors="white", linewidths=0.5)
            ax.annotate(f"{i}" if not cls else f"{i}:{cls}", (px, py),
                        textcoords="offset points", xytext=(4, 3),
                        fontsize=6, fontweight="bold", color="white",
                        bbox=dict(boxstyle="round,pad=0.12", facecolor=color,
                                  edgecolor="none", alpha=0.85))
        ax.scatter([], [], color=color, s=50, marker="*", edgecolors="white",
                   linewidths=0.5, label=f"{label} ({len(wps)})")

    # Depot marker
    dep_px, dep_py = _ll_to_px(AERIAL_DEPOT[0], AERIAL_DEPOT[1], tx_min, ty_min)
    ax.scatter(dep_px, dep_py, color="white", s=80, zorder=6, marker="D",
               edgecolors="black", linewidths=0.8)
    ax.annotate("Depot", (dep_px, dep_py), textcoords="offset points", xytext=(5, 4),
                fontsize=7, fontweight="bold", color="white",
                bbox=dict(boxstyle="round,pad=0.15", facecolor="black",
                          edgecolor="none", alpha=0.7))
    ax.scatter([], [], color="white", s=80, marker="D", edgecolors="black",
               linewidths=0.8, label="Depot")

    _draw_vehicle(aerial_waypoints, DRONE_CLR, "Drone")
    _draw_vehicle(ground_waypoints, SPOT_CLR,  "Spot")
    _draw_projected(projected_waypoints or [], PROJ_CLR, "Projected targets")

    # UAV position + heading arrow
    if drone_pos is not None:
        dp_lat, dp_lon, dp_hdg = drone_pos
        dp_px, dp_py = _ll_to_px(dp_lat, dp_lon, tx_min, ty_min)
        # heading: 0=north CW → d_n = cos(hdg), d_e = sin(hdg)
        tip_lat = dp_lat + math.degrees(arrow_m * 15 * math.cos(dp_hdg) / EARTH_RADIUS_METERS)
        tip_lon = dp_lon + math.degrees(
            arrow_m * 15 * math.sin(dp_hdg)
            / (EARTH_RADIUS_METERS * math.cos(math.radians(dp_lat)))
        )
        tip_px, tip_py = _ll_to_px(tip_lat, tip_lon, tx_min, ty_min)
        ax.scatter(dp_px, dp_py, color=UAV_CLR, s=120, zorder=7, marker="^",
                   edgecolors="white", linewidths=0.8)
        ax.annotate("", xy=(tip_px, tip_py), xytext=(dp_px, dp_py),
                    arrowprops=dict(arrowstyle="-|>", color=UAV_CLR,
                                    lw=2.0, mutation_scale=12))
        ax.annotate("UAV", (dp_px, dp_py), textcoords="offset points", xytext=(5, 4),
                    fontsize=7, fontweight="bold", color="white",
                    bbox=dict(boxstyle="round,pad=0.15", facecolor=UAV_CLR,
                              edgecolor="none", alpha=0.85))
        ax.scatter([], [], color=UAV_CLR, s=120, marker="^", edgecolors="white",
                   linewidths=0.8, label="UAV position")

    # Image footprint corners: TL TR BR BL
    if image_corners:
        CORNER_CLR = "yellow"
        CORNER_LABELS = ["TL", "TR", "BR", "BL"]
        valid_corners = [(i, pt) for i, pt in enumerate(image_corners) if pt is not None]
        if valid_corners:
            # Close the polygon by repeating the first valid corner
            ring_idxs = [i for i, _ in valid_corners]
            ring_pts  = [pt for _, pt in valid_corners]
            # Draw edges in order, skipping None corners
            for k in range(len(ring_pts)):
                la0, lo0 = ring_pts[k]
                la1, lo1 = ring_pts[(k + 1) % len(ring_pts)]
                px0, py0 = _ll_to_px(la0, lo0, tx_min, ty_min)
                px1, py1 = _ll_to_px(la1, lo1, tx_min, ty_min)
                ax.plot([px0, px1], [py0, py1], color=CORNER_CLR,
                        linewidth=1.5, linestyle="-", alpha=0.85, zorder=5)
            for i, (la, lo) in zip(ring_idxs, ring_pts):
                px, py = _ll_to_px(la, lo, tx_min, ty_min)
                ax.scatter(px, py, color=CORNER_CLR, s=60, zorder=6,
                           marker="s", edgecolors="black", linewidths=0.6)
                ax.annotate(CORNER_LABELS[i], (px, py),
                            textcoords="offset points", xytext=(4, 3),
                            fontsize=7, fontweight="bold", color="black",
                            bbox=dict(boxstyle="round,pad=0.12", facecolor=CORNER_CLR,
                                      edgecolor="none", alpha=0.9))
            ax.scatter([], [], color=CORNER_CLR, s=60, marker="s",
                       edgecolors="black", linewidths=0.6, label="Image corners")

    # Zoom to fit all waypoints with 20 % margin
    all_pxs = [_ll_to_px(la, lo, tx_min, ty_min)[0] for la, lo in zip(all_lats, all_lons)]
    all_pys = [_ll_to_px(la, lo, tx_min, ty_min)[1] for la, lo in zip(all_lats, all_lons)]
    cx = (min(all_pxs) + max(all_pxs)) / 2
    cy = (min(all_pys) + max(all_pys)) / 2
    span = max(max(all_pxs) - min(all_pxs), max(all_pys) - min(all_pys))
    half = max(span / 2 * 1.25, TILE_SIZE * 0.75)
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy + half, cy - half)

    ax.legend(loc="upper right", fontsize=8,
              facecolor="#222", edgecolor="none", labelcolor="white")
    ax.set_title("Observation Waypoints — Drone & Spot", color="white",
                 fontsize=11, pad=6)
    fig.patch.set_facecolor("#111")
    plt.tight_layout(pad=0.4)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(out_path), dpi=100, bbox_inches="tight")
    plt.close(fig)
    print(f"  satellite_map.png: {out_path}")


def _image_corners_to_gps(
    frame_shape,
    gps_pos: list[float],
    q_wxyz,
    camera_angle: float = 45.0,
) -> list[tuple[float, float] | None]:
    """Return GPS coords for the 4 image corners: TL, TR, BR, BL."""
    fx, fy = 1217.642102546154, 1205.813741470882
    cx_px, cy_px = 539.4282560125398, 352.5539369151344

    theta = math.radians(camera_angle)
    cam_x_in_body = np.array([0.0, -1.0, 0.0])
    cam_z_in_body = np.array([math.cos(theta), 0.0, -math.sin(theta)])
    cam_y_in_body = np.cross(cam_z_in_body, cam_x_in_body)
    R_cam_to_body = np.column_stack([cam_x_in_body, cam_y_in_body, cam_z_in_body])

    h, w = frame_shape[:2]
    corners_uv = [(0, 0), (w - 1, 0), (w - 1, h - 1), (0, h - 1)]  # TL TR BR BL

    result = []
    for u, v in corners_uv:
        try:
            lat, lon = _pixel_to_gps(
                float(u), float(v), gps_pos, np.asarray(q_wxyz, dtype=float),
                fx, fy, cx_px, cy_px, R_cam_to_body,
            )
            result.append((lat, lon))
        except ValueError:
            result.append(None)
    return result


_DEBUG_DIR = _SCRIPT_DIR / "debug"


def run_debug(
    image_path: str,
    gps: list[float] = [30.3926, -97.7285, 50.0],   # lat, lon, alt_m
    attitude: list[float] = [1.0, 0.0, 0.0, 0.0],   # w, x, y, z (level flight)
):
    frame = cv2.imread(image_path)
    if frame is None:
        raise FileNotFoundError(f"Could not load image: {image_path}")
    print(f"Image: {image_path}  shape={frame.shape}  GPS={gps}  attitude={attitude}")
    _, spot_wps = _process_and_save(frame, gps, attitude)
    proto_sender(spot_wps)
    print("Spot waypoints sent.")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--debug", action="store_true", help="Run without ROS using a local image")
    parser.add_argument("--image", default=None, help="Path to image file (required with --debug)")
    parser.add_argument("--gps", nargs=3, type=float, metavar=("LAT", "LON", "ALT"),
                        default=[30.3926, -97.7285, 50.0])
    parser.add_argument("--attitude", nargs=4, type=float, metavar=("W", "X", "Y", "Z"),
                        default=[1.0, 0.0, 0.0, 0.0])
    args = parser.parse_args()

    if args.debug:
        if args.image is None:
            parser.error("--image is required with --debug")
        run_debug(args.image, args.gps, args.attitude)
    else:
        run_pipeline()
