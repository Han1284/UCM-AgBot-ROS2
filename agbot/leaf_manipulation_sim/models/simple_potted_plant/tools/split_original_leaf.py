#!/usr/bin/env python3
"""Extract one textured leaf from FlowerPot.dae as five articulated OBJ parts.

The downloaded asset stores four leaves in a single Foliage geometry.  The
first leaf occupies vertex indices 0..81.  Faces are assigned to the nearest
section of a hand-checked centreline, while positions are translated into the
corresponding SDF link frame.  UVs and normals are copied without alteration.
"""

from pathlib import Path
import math
import xml.etree.ElementTree as ET


MODEL_DIR = Path(__file__).resolve().parents[1]
SOURCE = MODEL_DIR / "meshes" / "FlowerPot.dae"
OUTPUT_DIR = MODEL_DIR / "meshes" / "original_leaf_pilot"
NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
SCALE = 0.10

# Centreline in the source mesh coordinate system, from root to tip.
CENTRELINE = [
    (0.10, 0.05, 2.00),
    (-0.05, -0.10, 2.85),
    (-0.28, -0.50, 3.85),
    (-0.72, -1.18, 4.72),
    (-1.25, -2.08, 5.28),
    (-1.57, -2.74, 5.30),
]


def triples(values):
    return [tuple(values[i : i + 3]) for i in range(0, len(values), 3)]


def pairs(values):
    return [tuple(values[i : i + 2]) for i in range(0, len(values), 2)]


def nearest_section(point):
    best = (float("inf"), 0)
    for index, (start, end) in enumerate(zip(CENTRELINE, CENTRELINE[1:])):
        direction = tuple(end[k] - start[k] for k in range(3))
        length2 = sum(value * value for value in direction)
        offset = tuple(point[k] - start[k] for k in range(3))
        ratio = max(0.0, min(1.0, sum(offset[k] * direction[k] for k in range(3)) / length2))
        closest = tuple(start[k] + ratio * direction[k] for k in range(3))
        distance2 = sum((point[k] - closest[k]) ** 2 for k in range(3))
        if distance2 < best[0]:
            best = (distance2, index)
    return best[1]


def main():
    root = ET.parse(SOURCE).getroot()
    geometry = next(
        item for item in root.findall(".//c:geometry", NS) if item.get("name") == "Foliage"
    )

    def source_values(name):
        element = geometry.find(f'.//c:source[@name="{name}"]/c:float_array', NS)
        return list(map(float, element.text.split()))

    positions = triples(source_values("Plane.007-positions"))
    normals = triples(source_values("Plane.007-normals"))
    texcoords = pairs(source_values("Plane.007-tex0"))
    indices = list(map(int, geometry.find(".//c:polylist/c:p", NS).text.split()))
    triangles = list(zip(indices[::3], indices[1::3], indices[2::3]))
    leaf_triangles = [triangle for triangle in triangles if max(triangle) <= 81]

    sections = [[] for _ in range(5)]
    for triangle in leaf_triangles:
        centroid = tuple(sum(positions[i][axis] for i in triangle) / 3.0 for axis in range(3))
        sections[nearest_section(centroid)].append(triangle)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    material = OUTPUT_DIR / "original_leaf.mtl"
    material.write_text(
        "newmtl OriginalFoliage\n"
        "Ka 1.000 1.000 1.000\n"
        "Kd 1.000 1.000 1.000\n"
        "Ks 0.000 0.000 0.000\n"
        "d 1.0\n"
        "map_Kd ../../materials/textures/FlowerPot.png\n",
        encoding="utf-8",
    )

    for section_index, triangles_in_section in enumerate(sections):
        origin = tuple(value * SCALE for value in CENTRELINE[section_index])
        used = sorted({index for triangle in triangles_in_section for index in triangle})
        remap = {old: new for new, old in enumerate(used, start=1)}
        output = OUTPUT_DIR / f"original_leaf_segment_{section_index + 1}.obj"
        lines = ["mtllib original_leaf.mtl", "o OriginalLeafSegment", "usemtl OriginalFoliage"]
        for old in used:
            position = tuple(positions[old][axis] * SCALE - origin[axis] for axis in range(3))
            lines.append("v " + " ".join(f"{value:.9f}" for value in position))
        for old in used:
            uv = texcoords[old]
            lines.append(f"vt {uv[0]:.9f} {uv[1]:.9f}")
        for old in used:
            normal = normals[old]
            magnitude = math.sqrt(sum(value * value for value in normal)) or 1.0
            lines.append("vn " + " ".join(f"{value / magnitude:.9f}" for value in normal))
        for triangle in triangles_in_section:
            face = [remap[index] for index in triangle]
            lines.append("f " + " ".join(f"{index}/{index}/{index}" for index in face))
        output.write_text("\n".join(lines) + "\n", encoding="utf-8")
        print(f"{output.name}: {len(triangles_in_section)} triangles, {len(used)} vertices")


if __name__ == "__main__":
    main()
