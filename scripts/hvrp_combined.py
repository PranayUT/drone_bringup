#!/usr/bin/env python3
"""
Heterogeneous Vehicle Routing Problem (HVRP) — combined pipeline.

Solves a two-agent (aerial + ground) VRP, then refines the ground route by
generating candidate observation positions around each target and running a
TSP with disjunction constraints (≥1 candidate per target visited). The aerial
agent gets a fixed-sequence route visiting all candidates around each target.

Obstacle avoidance: candidate positions for the ground agent are checked against
obstacle polygons parsed from the KML file. The initial ring radius is 5 m; if
no candidate falls outside all obstacles the radius grows by 2 m per attempt.

Input waypoints are filtered to those inside the area-of-interest polygon from
the KML. Target waypoints themselves are not checked for obstacles.

Usage:
    python hvrp_combined.py \\
        --nodes nodes.json \\
        --kml Co-GLANCE-2.kml \\
        --aerial-depot 30.3926 -97.7285 \\
        --ground-depot 30.3926 -97.7285

nodes JSON schema (list of dicts):
    [{"name": str, "lat": float, "lon": float,
      "type": "aerial"|"ground"|"both"|"either"}, ...]
"""

import argparse
import inspect
import json
import math
import xml.etree.ElementTree as ET
from datetime import datetime
from functools import partial
from pathlib import Path

import contextily as ctx
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import FuncFormatter
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from shapely.geometry import Point, Polygon


# ── Constants ─────────────────────────────────────────────────────────────────

AERIAL_VEHICLE_IDX  = 0
GROUND_VEHICLE_IDX  = 1
NUM_VEHICLES        = 2
AERIAL_DEPOT_IDX    = 0
GROUND_DEPOT_IDX    = 1

EARTH_RADIUS_METERS = 6_378_137.0  # WGS84 equatorial radius

LARGE_PENALTY       = 1_000_000
N_CANDIDATES        = 8
INITIAL_RADIUS_M    = 5.0
RADIUS_STEP_M       = 2.0

NODE_TYPE_COLORS     = {"aerial": "royalblue", "ground": "forestgreen",
                        "both": "darkorchid",  "either": "darkorange"}
VEHICLE_ROUTE_COLORS = ["royalblue", "forestgreen"]
VEHICLE_LABELS       = ["Aerial route", "Ground route"]
VEHICLE_NAMES        = ["Aerial", "Ground"]

KML_NS = "http://www.opengis.net/kml/2.2"


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="Solve a Heterogeneous VRP with KML-based obstacle avoidance.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--nodes", required=True,
        help="Path to a JSON file OR an inline JSON string with task nodes",
    )
    parser.add_argument(
        "--kml", required=True, type=Path,
        help="KML file containing the area-of-interest and obstacle polygons",
    )
    parser.add_argument(
        "--aerial-depot", nargs=2, type=float, metavar=("LAT", "LON"), default=None,
        help="GPS coordinates of the aerial agent depot (overrides --start-index)",
    )
    parser.add_argument(
        "--ground-depot", nargs=2, type=float, metavar=("LAT", "LON"), default=None,
        help="GPS coordinates of the ground agent depot (overrides --start-index)",
    )
    parser.add_argument(
        "--start-index", type=int, default=None, metavar="N",
        help="Use the Nth start position from the JSON file as the agent depots",
    )
    parser.add_argument(
        "--n-candidates", type=int, default=N_CANDIDATES,
        help=f"Candidate positions per target (default: {N_CANDIDATES})",
    )
    parser.add_argument(
        "--output-dir", default="runs",
        help="Parent directory for timestamped run folders (default: ./runs)",
    )
    parser.add_argument(
        "--time-limit", type=int, default=10,
        help="Solver time limit in seconds (default: 10)",
    )
    return parser.parse_args()


# ── KML parsing ───────────────────────────────────────────────────────────────

def _parse_coord_string(coord_str: str) -> list[tuple[float, float]]:
    """Parse a KML coordinate string into (x, y) / (lon, lat) tuples."""
    pts = []
    for token in coord_str.strip().split():
        parts = token.split(",")
        lon, lat = float(parts[0]), float(parts[1])
        pts.append((lon, lat))
    return pts


def parse_kml(kml_path: Path) -> tuple[Polygon, list[Polygon]]:
    """Return (aoi_polygon, obstacle_polygons) as shapely Polygons in GPS (lon, lat)."""
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
        polygon = Polygon(pts)  # (lon, lat) pairs

        if name.lower() == "area of interest":
            aoi = polygon
        else:
            obstacles.append(polygon)

    if aoi is None:
        raise ValueError("KML file contains no placemark named 'Area of interest'")

    return aoi, obstacles


def _to_mercator_polygon(polygon: Polygon) -> Polygon:
    """Convert a shapely Polygon from GPS (lon, lat) to Web Mercator (x, y)."""
    coords = [(gps_to_web_mercator(lat, lon)) for lon, lat in polygon.exterior.coords]
    return Polygon(coords)


# ── Node loading & AOI filtering ──────────────────────────────────────────────

def load_task_nodes(nodes_arg: str):
    """Accept a path to a JSON file or an inline JSON string.

    Returns (nodes, starts_or_None).
    """
    candidate_path = Path(nodes_arg)
    data = json.loads(candidate_path.read_text()) if candidate_path.exists() else json.loads(nodes_arg)
    if isinstance(data, list):
        return data, None
    return data["nodes"], data.get("starts")


def filter_nodes_by_aoi(nodes: list[dict], aoi_mercator: Polygon) -> list[dict]:
    """Remove nodes whose projected position falls outside the AOI polygon."""
    kept = []
    for n in nodes:
        x, y = gps_to_web_mercator(n["lat"], n["lon"])
        if aoi_mercator.contains(Point(x, y)):
            kept.append(n)
    removed = len(nodes) - len(kept)
    if removed:
        print(f"Removed {removed} node(s) outside the area-of-interest polygon")
    return kept


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
    """Split 'both' nodes into two copies; attach allowed_vehicles constraints."""
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

def aerial_distance_callback(from_routing_index, to_routing_index, manager, distance_matrix):
    from_node = manager.IndexToNode(from_routing_index)
    to_node   = manager.IndexToNode(to_routing_index)
    return distance_matrix[from_node][to_node]


def ground_distance_callback(from_routing_index, to_routing_index, manager, distance_matrix):
    from_node = manager.IndexToNode(from_routing_index)
    to_node   = manager.IndexToNode(to_routing_index)
    return 10 * distance_matrix[from_node][to_node]


# ── HVRP routing model ────────────────────────────────────────────────────────

def build_routing_model(distance_matrix, expanded_task_nodes, time_limit_seconds):
    manager = pywrapcp.RoutingIndexManager(
        len(distance_matrix),
        NUM_VEHICLES,
        [AERIAL_DEPOT_IDX, GROUND_DEPOT_IDX],
        [AERIAL_DEPOT_IDX, GROUND_DEPOT_IDX],
    )
    routing = pywrapcp.RoutingModel(manager)

    aerial_cb = partial(aerial_distance_callback, manager=manager, distance_matrix=distance_matrix)
    ground_cb = partial(ground_distance_callback, manager=manager, distance_matrix=distance_matrix)

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


def extract_hvrp_route(vehicle_idx, solution, routing, manager, all_locations):
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

def generate_candidates(target: dict, target_idx: int, radius_m: float,
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


def generate_valid_candidates(target: dict, target_idx: int,
                              obstacles: list[Polygon],
                              n_candidates: int) -> tuple[list[dict], float]:
    """Generate candidates on a ring, filtering out obstacle-overlapping ones.

    Starts at INITIAL_RADIUS_M and increments by RADIUS_STEP_M until at
    least one candidate is outside all obstacles. Returns (valid_candidates, radius_used).
    """
    radius = INITIAL_RADIUS_M
    while True:
        all_cands = generate_candidates(target, target_idx, radius, n_candidates)
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


def build_area_routing_model(depot_projected: dict, all_candidates: list[dict],
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


def extract_area_route(solution, routing, manager, all_nodes: list[dict]):
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

def compute_observation_vectors(stops: list[dict],
                                targets_projected: list[dict]) -> list[dict]:
    """Yaw: 0 = south, increases CCW. Formula: atan2(dx, -dy), candidate→target."""
    vectors = []
    for stop in stops:
        if "target_idx" not in stop:
            continue
        tgt = targets_projected[stop["target_idx"]]
        dx  = tgt["x"] - stop["x"]
        dy  = tgt["y"] - stop["y"]
        yaw = math.atan2(dx, -dy)
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

def build_aerial_stops(depot_projected: dict, aerial_targets_projected: list[dict],
                       radius_m: float, n_candidates: int) -> tuple[list[dict], int]:
    """depot → target1_c0…cN → target2_c0…cN → … → depot (no optimisation)."""
    stops = [depot_projected]
    for t_idx, target in enumerate(aerial_targets_projected):
        stops.extend(generate_candidates(target, t_idx, radius_m, n_candidates))
    stops.append(depot_projected)
    total = sum(
        round(euclidean_distance_meters(stops[i], stops[i + 1]))
        for i in range(len(stops) - 1)
    )
    return stops, total


# ── Console output ────────────────────────────────────────────────────────────

def print_hvrp_plan(solution, routing, manager, all_locations):
    if solution is None:
        print("No solution found — check node constraints for feasibility.")
        return
    print(f"Solver status : {routing.status()}  (1 = feasible/optimal)")
    print(f"Total distance: {solution.ObjectiveValue()} m")
    for vehicle_idx in range(NUM_VEHICLES):
        stops, distance = extract_hvrp_route(vehicle_idx, solution, routing, manager, all_locations)
        route_str = " → ".join(s["name"] for s in stops)
        print(f"\n{VEHICLE_NAMES[vehicle_idx]} route ({distance} m):")
        print(f"  {route_str}")


# ── Visualisation ─────────────────────────────────────────────────────────────

def _draw_kml_polygons(ax, aoi_gps: Polygon, obstacles_gps: list[Polygon]):
    """Overlay AOI (green) and obstacle (red) polygon outlines."""
    def _poly_mercator_xy(poly_gps):
        xs, ys = [], []
        for lon, lat in poly_gps.exterior.coords:
            x, y = gps_to_web_mercator(lat, lon)
            xs.append(x)
            ys.append(y)
        return xs, ys

    xs, ys = _poly_mercator_xy(aoi_gps)
    ax.plot(xs, ys, color="limegreen", linewidth=1.5, linestyle="-", alpha=0.8,
            label="Area of interest")

    for i, obs in enumerate(obstacles_gps):
        xs, ys = _poly_mercator_xy(obs)
        ax.fill(xs, ys, color="red", alpha=0.25)
        ax.plot(xs, ys, color="red", linewidth=1.0, linestyle="-",
                label="Obstacle" if i == 0 else None)


def plot_combined(
    solution, routing, manager, all_locations,
    task_nodes_projected, aerial_depot_projected, ground_depot_projected,
    ground_stops, ground_obs_vectors, aerial_stops, aerial_obs_vectors,
    ground_all_candidates, ground_candidate_groups,
    aerial_candidates, aerial_groups,
    aoi_gps, obstacles_gps,
    origin_x, origin_y,
):
    fig, axes = plt.subplots(1, 2, figsize=(20, 10))

    # ── Left: HVRP routes ──────────────────────────────────────────────────────
    ax = axes[0]
    _draw_kml_polygons(ax, aoi_gps, obstacles_gps)

    if solution is not None:
        for vehicle_idx in range(NUM_VEHICLES):
            stops, _ = extract_hvrp_route(vehicle_idx, solution, routing, manager, all_locations)
            color = VEHICLE_ROUTE_COLORS[vehicle_idx]
            xs = [s["x"] for s in stops]
            ys = [s["y"] for s in stops]
            ax.plot(xs, ys, color=color, linestyle="--", linewidth=1.5, alpha=0.8,
                    label=VEHICLE_LABELS[vehicle_idx])
            for step, (start, end) in enumerate(zip(stops[:-1], stops[1:]), start=1):
                mid_x = (start["x"] + end["x"]) / 2
                mid_y = (start["y"] + end["y"]) / 2
                ax.annotate(str(step), (mid_x, mid_y), fontsize=7, fontweight="bold",
                            color="white", ha="center", va="center", zorder=6,
                            bbox=dict(boxstyle="round,pad=0.25", facecolor=color,
                                      edgecolor="none", alpha=0.9))

    label_style = dict(fontsize=8, color="white", fontweight="bold",
                       bbox=dict(boxstyle="round,pad=0.1", facecolor="black",
                                 alpha=0.4, edgecolor="none"))
    for node in task_nodes_projected:
        color = NODE_TYPE_COLORS[node["type"]]
        ax.scatter(node["x"], node["y"], color=color, s=80, zorder=4,
                   edgecolors="white", linewidths=0.8)
        ax.annotate(node["name"], (node["x"], node["y"]),
                    textcoords="offset points", xytext=(6, 4), **label_style)

    for depot, color, label in [
        (aerial_depot_projected, "royalblue",   "Aerial Depot"),
        (ground_depot_projected, "forestgreen", "Ground Depot"),
    ]:
        ax.scatter(depot["x"], depot["y"], color=color, marker="*", s=350, zorder=5,
                   edgecolors="white", linewidths=0.8, label=label)
        ax.annotate(depot["name"], (depot["x"], depot["y"]),
                    textcoords="offset points", xytext=(8, 4), **label_style)

    _finish_ax(ax, origin_x, origin_y, "Heterogeneous VRP — Aerial & Ground Agents",
               NODE_TYPE_COLORS)

    # ── Right: observation routes ──────────────────────────────────────────────
    ax = axes[1]
    _draw_kml_polygons(ax, aoi_gps, obstacles_gps)
    cmap = plt.colormaps["tab10"]

    # Ground candidates
    for t_idx, group in enumerate(ground_candidate_groups):
        colour = cmap(t_idx % 10)
        cands = [ground_all_candidates[i - 1] for i in group]
        xs = [c["x"] for c in cands] + [cands[0]["x"]]
        ys = [c["y"] for c in cands] + [cands[0]["y"]]
        ax.plot(xs, ys, color=colour, linestyle="--", linewidth=0.8, alpha=0.5)
        ax.scatter([c["x"] for c in cands], [c["y"] for c in cands],
                   color=colour, s=18, alpha=0.6, zorder=3)

    # Aerial candidates
    for t_idx, group in enumerate(aerial_groups):
        colour = cmap(t_idx % 10)
        cands = [aerial_candidates[i - 1] for i in group]
        xs = [c["x"] for c in cands] + [cands[0]["x"]]
        ys = [c["y"] for c in cands] + [cands[0]["y"]]
        ax.plot(xs, ys, color=colour, linestyle="-", linewidth=0.8, alpha=0.4)

    # Ground observation route
    if ground_stops:
        xs = [s["x"] for s in ground_stops]
        ys = [s["y"] for s in ground_stops]
        ax.plot(xs, ys, color="forestgreen", linewidth=2, alpha=0.9,
                label="Ground observation route")
        for step, (start, end) in enumerate(zip(ground_stops[:-1], ground_stops[1:]), start=1):
            ax.annotate(str(step),
                        ((start["x"] + end["x"]) / 2, (start["y"] + end["y"]) / 2),
                        fontsize=7, fontweight="bold", color="white",
                        ha="center", va="center", zorder=6,
                        bbox=dict(boxstyle="round,pad=0.25", facecolor="forestgreen",
                                  edgecolor="none", alpha=0.9))

    # Aerial observation route
    if aerial_stops:
        xs = [s["x"] for s in aerial_stops]
        ys = [s["y"] for s in aerial_stops]
        ax.plot(xs, ys, color="royalblue", linewidth=1.5, alpha=0.7, linestyle=":",
                label="Aerial observation route")

    # Target positions
    all_targets_proj = list({t["name"]: t for t in
                             task_nodes_projected}.values())
    for node in task_nodes_projected:
        ax.scatter(node["x"], node["y"], marker="*", s=180, color="gold",
                   edgecolors="black", linewidths=0.5, zorder=5)
        ax.annotate(node["name"], (node["x"], node["y"]),
                    textcoords="offset points", xytext=(6, 4), fontsize=7,
                    color="white", fontweight="bold",
                    bbox=dict(boxstyle="round,pad=0.1", facecolor="black",
                              alpha=0.4, edgecolor="none"))

    # Observation vectors — ground
    if ground_obs_vectors:
        ax.quiver(
            [v["cx"] for v in ground_obs_vectors],
            [v["cy"] for v in ground_obs_vectors],
            [v["dx"] for v in ground_obs_vectors],
            [v["dy"] for v in ground_obs_vectors],
            angles="xy", scale_units="xy", scale=3,
            color="white", width=0.004, headwidth=4, headlength=5,
            zorder=8, label="Ground observation direction",
        )

    _finish_ax(ax, origin_x, origin_y, "Observation Routes — Ground (optimised) & Aerial (fixed)",
               None)

    plt.tight_layout()
    return fig


def _finish_ax(ax, origin_x, origin_y, title, type_colors):
    pad = 10
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    ax.set_xlim(x0 - pad, x1 + pad)
    ax.set_ylim(y0 - pad, y1 + pad)

    ctx.add_basemap(ax, crs="EPSG:3857", source=ctx.providers.Esri.WorldImagery,
                    zoom="auto", attribution_size=6)

    ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v - origin_x:+.0f} m"))
    ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v - origin_y:+.0f} m"))

    handles, _ = ax.get_legend_handles_labels()
    if type_colors:
        type_patches = [mpatches.Patch(color=c, label=f"Node type: {t}")
                        for t, c in type_colors.items()]
        handles = handles + type_patches

    ax.legend(handles=handles, loc="upper right", fontsize=7,
              facecolor="#222222", edgecolor="none", labelcolor="white")
    ax.set_xlabel("East offset from aerial depot  (m)")
    ax.set_ylabel("North offset from aerial depot  (m)")
    ax.set_title(title)
    ax.grid(True, alpha=0.25, color="white")


# ── Save run ──────────────────────────────────────────────────────────────────

def save_run(
    run_dir: Path,
    run_timestamp: str,
    # HVRP inputs
    task_nodes, aerial_depot, ground_depot, kml_path,
    time_limit_seconds,
    all_locations,
    # HVRP outputs
    solution, routing, manager,
    # Ground area outputs
    ground_depot_dict, ground_targets,
    ground_all_candidates, ground_candidate_groups,
    ground_stops, ground_distance, ground_obs_vectors,
    ground_radius_info,  # list of {target, radius_m, n_filtered}
    # Aerial area outputs
    aerial_depot_dict, aerial_targets,
    aerial_candidates, aerial_groups,
    aerial_stops, aerial_distance, aerial_obs_vectors,
    # Figure
    fig,
):
    run_dir.mkdir(parents=True, exist_ok=True)

    def hvrp_route_log(vehicle_idx):
        if solution is None:
            return None
        stops, distance = extract_hvrp_route(vehicle_idx, solution, routing, manager, all_locations)
        return {
            "nodes_in_order":   [s["name"] for s in stops],
            "location_indices": [s.get("location_index", all_locations.index(s)) for s in stops],
            "distance_m":       distance,
        }

    def waypoints_from_vectors(obs_vectors):
        return [
            {"name": v["candidate"], "lat": v["lat"], "lon": v["lon"],
             "target": v["target"], "yaw_rad": v["yaw_rad"]}
            for v in obs_vectors
        ]

    log = {
        "run_timestamp": run_timestamp,
        "inputs": {
            "aerial_depot": aerial_depot,
            "ground_depot": ground_depot,
            "kml_path":     str(kml_path.resolve()),
            "task_nodes":   task_nodes,
            "all_locations_expanded": [
                {
                    "location_index": loc.get("location_index", idx),
                    "name":             loc["name"],
                    "lat":              loc["lat"],
                    "lon":              loc["lon"],
                    "type":             loc.get("type"),
                    "allowed_vehicles": loc.get("allowed_vehicles"),
                }
                for idx, loc in enumerate(all_locations)
            ],
            "search_params": {
                "first_solution_strategy":    "PATH_CHEAPEST_ARC",
                "local_search_metaheuristic": "GUIDED_LOCAL_SEARCH",
                "time_limit_seconds":         time_limit_seconds,
            },
            "distance_callbacks": {
                "aerial": inspect.getsource(aerial_distance_callback),
                "ground": inspect.getsource(ground_distance_callback),
            },
        },
        "hvrp_outputs": {
            "solver_status":    routing.status(),
            "total_distance_m": solution.ObjectiveValue() if solution else None,
            "routes": {
                "aerial": hvrp_route_log(AERIAL_VEHICLE_IDX),
                "ground": hvrp_route_log(GROUND_VEHICLE_IDX),
            },
        },
        "area_outputs": {
            "ground": {
                "depot":   ground_depot_dict,
                "targets": ground_targets,
                "radius_info": ground_radius_info,
                "solver_status": None,
                "total_distance_m": ground_distance,
                "route": {
                    "nodes_in_order": [
                        {"name": s["name"], "lat": s["lat"], "lon": s["lon"]}
                        for s in (ground_stops or [])
                    ],
                    "distance_m": ground_distance,
                },
                "observation_vectors": [
                    {"candidate": v["candidate"], "lat": v["lat"], "lon": v["lon"],
                     "target": v["target"], "yaw_rad": v["yaw_rad"]}
                    for v in ground_obs_vectors
                ],
            },
            "aerial": {
                "depot":   aerial_depot_dict,
                "targets": aerial_targets,
                "radius_m": INITIAL_RADIUS_M,
                "total_distance_m": aerial_distance,
                "route": {
                    "nodes_in_order": [
                        {"name": s["name"], "lat": s["lat"], "lon": s["lon"]}
                        for s in (aerial_stops or [])
                    ],
                    "distance_m": aerial_distance,
                },
                "observation_vectors": [
                    {"candidate": v["candidate"], "lat": v["lat"], "lon": v["lon"],
                     "target": v["target"], "yaw_rad": v["yaw_rad"]}
                    for v in aerial_obs_vectors
                ],
            },
        },
    }

    log_path          = run_dir / "log.json"
    image_path        = run_dir / "routes.png"
    ground_wp_path    = run_dir / "ground_waypoints.json"
    aerial_wp_path    = run_dir / "aerial_waypoints.json"

    log_path.write_text(json.dumps(log, indent=2))
    fig.savefig(image_path, dpi=150, bbox_inches="tight")
    plt.close(fig)

    ground_wp_path.write_text(json.dumps(waypoints_from_vectors(ground_obs_vectors), indent=2))
    aerial_wp_path.write_text(json.dumps(waypoints_from_vectors(aerial_obs_vectors), indent=2))

    for p in (log_path, image_path, ground_wp_path, aerial_wp_path):
        print(f"  {p.name:<28s}  {p.stat().st_size / 1024:.1f} KB")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    args = parse_args()

    # ── Parse KML ─────────────────────────────────────────────────────────────
    aoi_gps, obstacles_gps = parse_kml(args.kml)
    aoi_mercator           = _to_mercator_polygon(aoi_gps)
    obstacles_mercator     = [_to_mercator_polygon(o) for o in obstacles_gps]
    print(f"KML: AOI polygon loaded, {len(obstacles_gps)} obstacle polygon(s)")

    # ── Load & filter task nodes ───────────────────────────────────────────────
    task_nodes, starts = load_task_nodes(args.nodes)
    task_nodes = filter_nodes_by_aoi(task_nodes, aoi_mercator)

    # ── Resolve depots ─────────────────────────────────────────────────────────
    aerial_lat, aerial_lon = args.aerial_depot or (None, None)
    ground_lat, ground_lon = args.ground_depot or (None, None)
    if args.start_index is not None:
        if not starts:
            raise SystemExit("--start-index given but the JSON contains no 'starts' entries")
        by_index = {s["index"]: s for s in starts}
        if args.start_index not in by_index:
            raise SystemExit(f"Start index {args.start_index} not found; available: {sorted(by_index)}")
        chosen     = by_index[args.start_index]
        aerial_lat = aerial_lat or chosen["aerial"]["lat"]
        aerial_lon = aerial_lon or chosen["aerial"]["lon"]
        ground_lat = ground_lat or chosen["ground"]["lat"]
        ground_lon = ground_lon or chosen["ground"]["lon"]
    if aerial_lat is None or ground_lat is None:
        raise SystemExit(
            "Depot coordinates required: provide --start-index or both "
            "--aerial-depot and --ground-depot"
        )

    aerial_depot = {"name": "Aerial Depot", "lat": aerial_lat, "lon": aerial_lon}
    ground_depot = {"name": "Ground Depot", "lat": ground_lat, "lon": ground_lon}

    aerial_depot_proj = project_node(aerial_depot)
    ground_depot_proj = project_node(ground_depot)
    task_nodes_proj   = [project_node(n) for n in task_nodes]
    origin_x = aerial_depot_proj["x"]
    origin_y = aerial_depot_proj["y"]

    # ── HVRP solve ─────────────────────────────────────────────────────────────
    expanded_task_nodes = expand_task_nodes(task_nodes_proj)
    for pos, node in enumerate(expanded_task_nodes):
        node["location_index"] = GROUND_DEPOT_IDX + 1 + pos

    all_locations   = [aerial_depot_proj, ground_depot_proj] + expanded_task_nodes
    distance_matrix = build_distance_matrix(all_locations)

    manager, routing, search_params = build_routing_model(
        distance_matrix, expanded_task_nodes, args.time_limit
    )
    solution = routing.SolveWithParameters(search_params)
    print_hvrp_plan(solution, routing, manager, all_locations)

    # ── Extract targets from solved routes ─────────────────────────────────────
    loc_lookup = {
        loc["location_index"]: {"name": loc["name"], "lat": loc["lat"], "lon": loc["lon"]}
        for loc in all_locations
        if "location_index" in loc
    }
    # Also add depots by their fixed indices
    loc_lookup[AERIAL_DEPOT_IDX] = aerial_depot
    loc_lookup[GROUND_DEPOT_IDX] = ground_depot

    def interior_targets(vehicle_idx):
        if solution is None:
            return []
        stops, _ = extract_hvrp_route(vehicle_idx, solution, routing, manager, all_locations)
        return [{"name": s["name"], "lat": s["lat"], "lon": s["lon"]}
                for s in stops[1:-1]]  # strip depot at both ends

    ground_targets = interior_targets(GROUND_VEHICLE_IDX)
    aerial_targets = interior_targets(AERIAL_VEHICLE_IDX)

    # ── Ground: obstacle-aware candidates + TSP ────────────────────────────────
    ground_targets_proj = [project_node(t) for t in ground_targets]
    ground_all_candidates:   list[dict] = []
    ground_candidate_groups: list[list[int]] = []
    ground_radius_info: list[dict] = []

    for t_idx, target in enumerate(ground_targets_proj):
        valid_cands, radius_used = generate_valid_candidates(
            target, t_idx, obstacles_mercator, args.n_candidates
        )
        n_filtered = args.n_candidates - len(valid_cands)
        if n_filtered:
            print(f"  Target '{target['name']}': radius expanded to {radius_used} m "
                  f"({n_filtered} candidate(s) in obstacles)")
        ground_radius_info.append({
            "target":     target["name"],
            "radius_m":   radius_used,
            "n_filtered": n_filtered,
        })
        start = len(ground_all_candidates) + 1
        ground_candidate_groups.append(list(range(start, start + len(valid_cands))))
        ground_all_candidates.extend(valid_cands)

    print(f"\nGround: {len(ground_targets)} target(s), "
          f"{len(ground_all_candidates)} valid candidate(s)")

    ground_stops, ground_distance, ground_obs_vectors = [], 0, []
    if ground_targets:
        g_manager, g_routing, g_params, g_all_nodes = build_area_routing_model(
            ground_depot_proj, ground_all_candidates, ground_candidate_groups, args.time_limit
        )
        g_solution = g_routing.SolveWithParameters(g_params)
        if g_solution is None:
            print("Ground area routing: no solution found.")
        else:
            ground_stops, ground_distance = extract_area_route(
                g_solution, g_routing, g_manager, g_all_nodes
            )
            ground_obs_vectors = compute_observation_vectors(ground_stops, ground_targets_proj)
            route_str = " → ".join(s["name"] for s in ground_stops)
            print(f"Ground observation route ({ground_distance} m):\n  {route_str}")

    # ── Aerial: fixed-sequence route ───────────────────────────────────────────
    aerial_targets_proj = [project_node(t) for t in aerial_targets]
    aerial_candidates:   list[dict] = []
    aerial_groups:       list[list[int]] = []
    for t_idx, target in enumerate(aerial_targets_proj):
        cands = generate_candidates(target, t_idx, INITIAL_RADIUS_M, args.n_candidates)
        start = len(aerial_candidates) + 1
        aerial_groups.append(list(range(start, start + len(cands))))
        aerial_candidates.extend(cands)

    aerial_stops, aerial_distance = build_aerial_stops(
        aerial_depot_proj, aerial_targets_proj, INITIAL_RADIUS_M, args.n_candidates
    )
    aerial_obs_vectors = compute_observation_vectors(aerial_stops, aerial_targets_proj)
    print(f"Aerial: {len(aerial_targets)} target(s), "
          f"{len(aerial_candidates)} candidate(s), "
          f"route distance {aerial_distance} m")

    # ── Plot ───────────────────────────────────────────────────────────────────
    fig = plot_combined(
        solution, routing, manager, all_locations,
        task_nodes_proj, aerial_depot_proj, ground_depot_proj,
        ground_stops, ground_obs_vectors,
        aerial_stops, aerial_obs_vectors,
        ground_all_candidates, ground_candidate_groups,
        aerial_candidates, aerial_groups,
        aoi_gps, obstacles_gps,
        origin_x, origin_y,
    )

    # ── Save ───────────────────────────────────────────────────────────────────
    run_timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    run_dir       = Path(args.output_dir) / run_timestamp
    print(f"\nSaved to: {run_dir}/")
    save_run(
        run_dir, run_timestamp,
        task_nodes, aerial_depot, ground_depot, args.kml, args.time_limit,
        all_locations, solution, routing, manager,
        ground_depot, ground_targets,
        ground_all_candidates, ground_candidate_groups,
        ground_stops, ground_distance, ground_obs_vectors, ground_radius_info,
        aerial_depot, aerial_targets,
        aerial_candidates, aerial_groups,
        aerial_stops, aerial_distance, aerial_obs_vectors,
        fig,
    )


if __name__ == "__main__":
    main()
