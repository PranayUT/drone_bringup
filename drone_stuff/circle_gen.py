#!/usr/bin/env python3
"""
Read a waypoints file, transform each waypoint, and write results to a new file.

Input format:  lat, lon, alt   (one per line; # comments and blank lines ignored)
Output format: same
"""

import sys
import os
import webbrowser
import folium
from pyproj import Geod

def transform_waypoint(lat, lon, alt):
    """
    Given a single input waypoint, return a list of output waypoints.
    Each element should be a (lat, lon, alt) tuple.

    """
    CIRCLE_RES = 10
    geod = Geod(ellps="WGS84")
    output_waypoints = []
    for n in range(CIRCLE_RES):
        yaw = (360 / CIRCLE_RES) * n
        new_lon, new_lat, _ = geod.fwd(lon, lat, az=yaw, dist=alt)
        output_waypoints.append((new_lat, new_lon, alt))
    return output_waypoints


def parse_waypoints(filepath):
    waypoints = []
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            lat, lon, alt = float(parts[0]), float(parts[1]), float(parts[2])
            waypoints.append((lat, lon, alt))
    return waypoints


def write_waypoints(filepath, waypoints):
    with open(filepath, "w") as f:
        f.write("# Generated waypoints — lat, lon, alt_above_takeoff_metres\n")
        for lat, lon, alt in waypoints:
            f.write(f"{lat}, {lon}, {alt}\n")


def preview_map(input_waypoints, output_waypoints, map_path):
    center_lat = sum(p[0] for p in input_waypoints) / len(input_waypoints)
    center_lon = sum(p[1] for p in input_waypoints) / len(input_waypoints)

    m = folium.Map(
        location=[center_lat, center_lon],
        zoom_start=18,
        tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        attr="Esri World Imagery",
    )

    for lat, lon, alt in input_waypoints:
        folium.CircleMarker(
            location=[lat, lon],
            radius=8,
            color="red",
            fill=True,
            fill_color="red",
            fill_opacity=0.9,
            tooltip=f"INPUT  {lat:.7f}, {lon:.7f}, {alt}m",
        ).add_to(m)

    for lat, lon, alt in output_waypoints:
        folium.CircleMarker(
            location=[lat, lon],
            radius=5,
            color="cyan",
            fill=True,
            fill_color="cyan",
            fill_opacity=0.7,
            tooltip=f"GEN  {lat:.7f}, {lon:.7f}, {alt}m",
        ).add_to(m)

    m.save(map_path)
    webbrowser.open(f"file://{os.path.abspath(map_path)}")
    print(f"Map saved to {map_path}")


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <input_waypoints.txt> [output_waypoints.txt]")
        sys.exit(1)

    input_path = sys.argv[1]
    if len(sys.argv) >= 3:
        output_path = sys.argv[2]
    else:
        base, ext = os.path.splitext(input_path)
        output_path = f"{base}_transformed{ext}"

    input_waypoints = parse_waypoints(input_path)
    print(f"Read {len(input_waypoints)} waypoint(s) from {input_path}")

    output_waypoints = []
    for lat, lon, alt in input_waypoints:
        generated = transform_waypoint(lat, lon, alt)
        output_waypoints.extend(generated)

    write_waypoints(output_path, output_waypoints)
    print(f"Wrote {len(output_waypoints)} waypoint(s) to {output_path}")

    map_path = os.path.splitext(output_path)[0] + "_preview.html"
    preview_map(input_waypoints, output_waypoints, map_path)


if __name__ == "__main__":
    main()
