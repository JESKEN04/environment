#!/usr/bin/env python3
"""
generate_world.py
=================
Generates the complete Gazebo simulation world for:
  "面向巡检任务的无人机编队策略研究"

Output files (written to ~/ros2_ws/src/environment-main/uav_inspection/worlds/):
  • mountain_mesh.obj    - Mountain terrain mesh
  • inspection_world.sdf - Complete Gazebo world SDF

Usage:
  python3 generate_world.py
"""

import numpy as np
import os, math

# ─────────────────────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────────────────────
WORLD_SIZE   = 400          # World XY footprint (m), square
WORLD_Z_MAX  = 60.0         # Heightmap elevation ceiling (m)
MESH_RES     = 65           # Mesh resolution (vertices per side)
TOWER_H      = 20.0         # Transmission tower height (m) - increased to avoid wire clipping
ARM_LEN      = 3.5          # Cross-arm half-length (m)
WIRE_RADIUS  = 0.04         # Power wire cylinder radius (m)
N_WIRE_SEGS  = 35           # Catenary segments per wire
N_TREES      = 20

# Candidate tower world positions (auto-adjusted to terrain after heightmap gen)
# Placed on opposite sides of the mountain to ensure wires don't clip through terrain
BASE_TOWER_CANDIDATE = (-130.0, -35.0)   # mountain south side (lower ground)
PEAK_TOWER_CANDIDATE = ( -30.0,  45.0)   # mountain north side (higher ground)

# Use absolute path based on current user's home directory
import getpass
USER_HOME = os.path.expanduser("~") if os.path.expanduser("~") != "~" else f"/home/{getpass.getuser()}"
OUT_DIR = os.path.join(USER_HOME, "ros2_ws/src/environment-main/uav_inspection/worlds")
os.makedirs(OUT_DIR, exist_ok=True)
MESH_PATH   = os.path.join(OUT_DIR, "mountain_mesh.obj")
WORLD_PATH  = os.path.join(OUT_DIR, "inspection_world.sdf")


# ─────────────────────────────────────────────────────────────────────────────
# Step 1 – Generate mountain heightmap
# ─────────────────────────────────────────────────────────────────────────────
def build_heightmap(resolution: int = MESH_RES) -> np.ndarray:
    """
    Generates a heightmap of an irregular mountain range.
    Returns raw height array in metres.
    """
    # World-coordinate grids
    lin   = np.linspace(-WORLD_SIZE / 2, WORLD_SIZE / 2, resolution)
    WX, WY = np.meshgrid(lin, lin)

    hm = np.zeros_like(WX, dtype=np.float32)

    # ── Primary mountain ridge ──────────────────────────────────────────────
    N_RIDGE = 120
    for t in np.linspace(0.0, 1.0, N_RIDGE):
        rx = -130.0 + 230.0 * t
        ry = (28.0 * np.sin(t * np.pi * 1.25) +
               9.0 * np.sin(t * np.pi * 3.8 + 0.7) +
               4.0 * np.cos(t * np.pi * 6.1 + 1.2))

        rh = 52.0 * np.sin(t * np.pi) * max(0.0,
             1.0 + 0.18 * np.sin(t * 7.3)
                 + 0.13 * np.cos(t * 11.1)
                 + 0.08 * np.sin(t * 15.7 + 0.5))

        sigma = 30.0 + 16.0 * np.sin(t * np.pi)

        dist2 = (WX - rx) ** 2 + (WY - ry) ** 2
        hm = np.maximum(hm, rh * np.exp(-dist2 / (2.0 * sigma ** 2)))

    # ── Secondary ridge ───────────────────────────────────────────────────
    for t in np.linspace(0.05, 0.95, 60):
        rx = -120.0 + 200.0 * t + 15.0 * np.sin(t * 4.0)
        ry = -30.0 + 20.0 * np.sin(t * np.pi * 2.1 + 1.0)
        rh = 28.0 * np.sin(t * np.pi) * max(0.0,
             1.0 + 0.2 * np.sin(t * 8.0))
        sigma = 22.0 + 8.0 * np.sin(t * np.pi)
        dist2 = (WX - rx) ** 2 + (WY - ry) ** 2
        hm = np.maximum(hm, rh * np.exp(-dist2 / (2.0 * sigma ** 2)))

    # ── Smooth terrain noise ────────────────────────────────────────────────
    rng   = np.random.RandomState(2024)
    noise = rng.randn(resolution, resolution).astype(np.float32)
    try:
        from scipy.ndimage import gaussian_filter
        noise = gaussian_filter(noise, sigma=2) * 1.5
    except ImportError:
        k = np.ones(5) / 5.0
        noise = np.apply_along_axis(lambda x: np.convolve(x, k, 'same'), 0, noise)
        noise = np.apply_along_axis(lambda x: np.convolve(x, k, 'same'), 1, noise)
        noise *= 1.2

    hm = np.clip(hm + noise, 0.0, WORLD_Z_MAX)

    return hm


def query_height(hm: np.ndarray, wx: float, wy: float) -> float:
    """Bilinear query of terrain height at world (wx, wy)."""
    res = hm.shape[0]
    fc = (wx + WORLD_SIZE / 2) / WORLD_SIZE * (res - 1)
    fr = (wy + WORLD_SIZE / 2) / WORLD_SIZE * (res - 1)
    c0, r0 = int(fc), int(fr)
    c1, r1 = c0 + 1, r0 + 1
    c0, c1 = np.clip([c0, c1], 0, res - 1)
    r0, r1 = np.clip([r0, r1], 0, res - 1)
    tx, ty = fc - int(fc), fr - int(fr)
    h = (hm[r0, c0] * (1 - tx) * (1 - ty) +
         hm[r0, c1] * tx * (1 - ty) +
         hm[r1, c0] * (1 - tx) * ty +
         hm[r1, c1] * tx * ty)
    return float(h)


def find_foot_position(hm: np.ndarray, cx: float, cy: float,
                       target_h: float = 4.0, radius: float = 20.0) -> tuple:
    """Search near (cx,cy) for a position whose terrain height is close to target_h."""
    best = (cx, cy)
    best_err = abs(query_height(hm, cx, cy) - target_h)
    for dx in np.linspace(-radius, radius, 20):
        for dy in np.linspace(-radius, radius, 20):
            wx, wy = cx + dx, cy + dy
            err = abs(query_height(hm, wx, wy) - target_h)
            if err < best_err:
                best_err = err
                best = (wx, wy)
    return best


def find_flat_spot(hm: np.ndarray, cx: float, cy: float, radius: float = 30.0, max_slope: float = 0.1) -> tuple:
    """Search near (cx,cy) for a position with low terrain slope."""
    best = (cx, cy)
    best_slope = 1e9
    eps = 1.0  # for gradient computation
    for dx in np.linspace(-radius, radius, 20):
        for dy in np.linspace(-radius, radius, 20):
            wx, wy = cx + dx, cy + dy
            # Compute gradient using central differences
            hx1 = query_height(hm, wx - eps, wy)
            hx2 = query_height(hm, wx + eps, wy)
            hy1 = query_height(hm, wx, wy - eps)
            hy2 = query_height(hm, wx, wy + eps)
            dzdx = (hx2 - hx1) / (2 * eps)
            dzdy = (hy2 - hy1) / (2 * eps)
            slope = math.sqrt(dzdx**2 + dzdy**2)
            if slope < best_slope:
                best_slope = slope
                best = (wx, wy)
                if slope < max_slope:
                    return best, slope
    return best, best_slope


# ─────────────────────────────────────────────────────────────────────────────
# Step 2 – Generate OBJ mesh file
# ─────────────────────────────────────────────────────────────────────────────
def write_obj_mesh(hm: np.ndarray, filepath: str) -> None:
    """Write heightmap as OBJ mesh file with normals."""
    res = hm.shape[0]
    half_size = WORLD_SIZE / 2

    # Compute vertices
    vertices = []
    for i in range(res):
        for j in range(res):
            x = -half_size + j * (WORLD_SIZE / (res - 1))
            y = -half_size + i * (WORLD_SIZE / (res - 1))
            z = hm[i, j]
            vertices.append((x, y, z))

    # Compute normals per vertex (average of surrounding face normals)
    normals = []
    for i in range(res):
        for j in range(res):
            # Get neighboring heights for normal computation
            z_c = hm[i, j]
            z_l = hm[i, max(0, j-1)]
            z_r = hm[i, min(res-1, j+1)]
            z_u = hm[max(0, i-1), j]
            z_d = hm[min(res-1, i+1), j]

            # Compute normal using central differences
            dx = (z_r - z_l) / 2.0 if j > 0 and j < res-1 else (z_r - z_l)
            dy = (z_d - z_u) / 2.0 if i > 0 and i < res-1 else (z_d - z_u)

            # Normal vector (pointing up and outward)
            nx = dx
            ny = dy
            nz = 1.0

            # Normalize
            length = math.sqrt(nx*nx + ny*ny + nz*nz)
            nx, ny, nz = nx/length, ny/length, nz/length
            normals.append((nx, ny, nz))

    with open(filepath, 'w') as f:
        f.write("# Mountain terrain mesh for UAV inspection simulation\n")
        f.write(f"# Generated by generate_world.py\n")
        f.write(f"# Vertices: {res * res}\n\n")

        f.write("mtllib mountain_mesh_material.mtl\n")
        f.write("usemtl terrain_material\n\n")

        # Write vertices
        for v in vertices:
            f.write(f"v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}\n")

        f.write("\n")

        # Write normals
        for n in normals:
            f.write(f"vn {n[0]:.4f} {n[1]:.4f} {n[2]:.4f}\n")

        f.write("\n")

        # Write faces (two triangles per grid cell) with normals
        for i in range(res - 1):
            for j in range(res - 1):
                # Vertex/normal indices (1-based in OBJ)
                v00 = i * res + j + 1
                v10 = i * res + (j + 1) + 1
                v01 = (i + 1) * res + j + 1
                v11 = (i + 1) * res + (j + 1) + 1

                # Two triangles per quad with normals
                f.write(f"f {v00}//{v00} {v10}//{v10} {v11}//{v11}\n")
                f.write(f"f {v00}//{v00} {v11}//{v11} {v01}//{v01}\n")

    print(f"  [✓] Mesh OBJ → {filepath}")

    # Write material file
    mtl_path = filepath.replace('.obj', '_material.mtl')
    with open(mtl_path, 'w') as f:
        f.write("# Terrain material\n")
        f.write("newmtl terrain_material\n")
        f.write("Ka 0.35 0.28 0.18\n")  # Ambient
        f.write("Kd 0.45 0.38 0.25\n")  # Diffuse
        f.write("Ks 0.05 0.05 0.05\n")  # Specular
        f.write("Ns 10.0\n")            # Shininess
        f.write("d 1.0\n")              # Opacity

    print(f"  [✓] Material MTL → {mtl_path}")


# ─────────────────────────────────────────────────────────────────────────────
# Step 3 – SDF geometry helpers
# ─────────────────────────────────────────────────────────────────────────────

def _cylinder_sdf(name: str, p1: tuple, p2: tuple,
                  radius: float, color: str) -> str:
    """Return SDF <model> string for a cylinder connecting two 3-D points."""
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    mx, my, mz = (x1+x2)/2, (y1+y2)/2, (z1+z2)/2
    L = math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)
    if L < 1e-6:
        return ""

    dx, dy, dz = (x2-x1)/L, (y2-y1)/L, (z2-z1)/L

    # Rotation: align default cylinder axis (Z) with segment direction
    cross = np.cross([0, 0, 1], [dx, dy, dz])
    cl    = float(np.linalg.norm(cross))
    dot   = min(1.0, max(-1.0, dz))
    angle = math.acos(dot)

    if cl < 1e-9:
        roll, pitch, yaw = (0.0, 0.0, 0.0) if dz > 0 else (math.pi, 0.0, 0.0)
    else:
        ax, ay, az = cross / cl
        c, s, tc   = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
        R = [[tc*ax*ax+c,     tc*ax*ay-az*s, tc*ax*az+ay*s],
             [tc*ay*ax+az*s,  tc*ay*ay+c,    tc*ay*az-ax*s],
             [tc*az*ax-ay*s,  tc*az*ay+ax*s, tc*az*az+c   ]]
        pitch = math.atan2(-R[2][0], math.sqrt(R[0][0]**2 + R[1][0]**2))
        cp    = math.cos(pitch)
        yaw   = math.atan2(R[1][0]/cp, R[0][0]/cp) if abs(cp) > 1e-9 else 0.0
        roll  = math.atan2(R[2][1]/cp, R[2][2]/cp) if abs(cp) > 1e-9 else 0.0

    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{mx:.3f} {my:.3f} {mz:.3f} {roll:.5f} {pitch:.5f} {yaw:.5f}</pose>
      <link name="link">
        <visual name="v">
          <geometry><cylinder>
            <radius>{radius}</radius><length>{L:.4f}</length>
          </cylinder></geometry>
          <material>
            <ambient>{color}</ambient><diffuse>{color}</diffuse>
          </material>
        </visual>
        <collision name="c">
          <geometry><cylinder>
            <radius>{radius}</radius><length>{L:.4f}</length>
          </cylinder></geometry>
        </collision>
      </link>
    </model>"""


def _catenary_points(p1: tuple, p2: tuple,
                     sag_factor: float = 0.05,  # Reduced sag to prevent clipping
                     n: int = N_WIRE_SEGS) -> list:
    """Parabolic catenary approximation between two 3-D endpoints."""
    pts = []
    span = math.sqrt(sum((b-a)**2 for a, b in zip(p1, p2)))
    sag  = sag_factor * span
    for i in range(n + 1):
        t = i / n
        x = p1[0] + (p2[0] - p1[0]) * t
        y = p1[1] + (p2[1] - p1[1]) * t
        z = p1[2] + (p2[2] - p1[2]) * t - sag * 4 * t * (1 - t)
        pts.append((x, y, z))
    return pts


def wire_sdf(prefix: str, p1: tuple, p2: tuple) -> str:
    """SDF for a segmented catenary power wire."""
    pts   = _catenary_points(p1, p2)
    parts = []
    for i in range(len(pts) - 1):
        parts.append(_cylinder_sdf(f"{prefix}_s{i:02d}",
                                   pts[i], pts[i+1],
                                   WIRE_RADIUS, "0.08 0.08 0.08 1"))
    return "\n".join(parts)


def tower_sdf(name: str, wx: float, wy: float, gz: float) -> str:
    """SDF for a lattice transmission tower."""
    h   = TOWER_H
    arm = ARM_LEN
    cr  = 0.22
    br  = 0.09
    cols = [(-0.9, -0.9), (-0.9,  0.9), (0.9, -0.9), (0.9,  0.9)]

    parts = []

    # 4 vertical columns
    for (cdx, cdy) in cols:
        p1 = (wx+cdx, wy+cdy, gz)
        p2 = (wx+cdx, wy+cdy, gz + h)
        parts.append(_cylinder_sdf(f"{name}_col{cdx:.0f}{cdy:.0f}",
                                   p1, p2, cr, "0.72 0.72 0.72 1"))

    # Horizontal ring braces at 3 heights
    for frac, tag in [(0.25,"b1"),(0.50,"b2"),(0.75,"b3")]:
        bz = gz + h * frac
        edges = [
            ((-0.9,-0.9),(-0.9, 0.9)),
            ((-0.9, 0.9),( 0.9, 0.9)),
            (( 0.9, 0.9),( 0.9,-0.9)),
            (( 0.9,-0.9),(-0.9,-0.9)),
        ]
        for j, ((ax1,ay1),(ax2,ay2)) in enumerate(edges):
            parts.append(_cylinder_sdf(f"{name}_{tag}_e{j}",
                                       (wx+ax1, wy+ay1, bz),
                                       (wx+ax2, wy+ay2, bz),
                                       br, "0.68 0.68 0.68 1"))

    # Diagonal X-braces on each face
    faces = [
        ((-0.9,-0.9),(-0.9, 0.9)),
        (( 0.9,-0.9),( 0.9, 0.9)),
        ((-0.9,-0.9),( 0.9,-0.9)),
        ((-0.9, 0.9),( 0.9, 0.9)),
    ]
    for fi, ((ax1,ay1),(ax2,ay2)) in enumerate(faces):
        for zi, (z_bot, z_top) in enumerate([(gz, gz+h*0.5),(gz+h*0.5, gz+h*0.9)]):
            parts.append(_cylinder_sdf(f"{name}_diag{fi}_{zi}a",
                                       (wx+ax1,wy+ay1,z_bot),
                                       (wx+ax2,wy+ay2,z_top),
                                       br*0.7, "0.65 0.65 0.65 1"))
            parts.append(_cylinder_sdf(f"{name}_diag{fi}_{zi}b",
                                       (wx+ax2,wy+ay2,z_bot),
                                       (wx+ax1,wy+ay1,z_top),
                                       br*0.7, "0.65 0.65 0.65 1"))

    # Cross-arm
    arm_z = gz + h - 0.8
    p_arm_l = (wx,   wy - arm, arm_z)
    p_arm_r = (wx,   wy + arm, arm_z)
    p_arm_c = (wx,   wy,       arm_z)
    parts.append(_cylinder_sdf(f"{name}_arm",
                               p_arm_l, p_arm_r, 0.14, "0.72 0.72 0.72 1"))

    # Vertical support
    parts.append(_cylinder_sdf(f"{name}_vsup",
                               (wx, wy, gz+h), p_arm_c, 0.12, "0.72 0.72 0.72 1"))

    # Insulators
    for side, sy in [("L", -arm), ("R", +arm)]:
        ins_top = (wx, wy + sy, arm_z)
        ins_bot = (wx, wy + sy, arm_z - 0.9)
        parts.append(_cylinder_sdf(f"{name}_ins{side}",
                                   ins_top, ins_bot, 0.06, "0.9 0.85 0.3 1"))

    return "\n".join(p for p in parts if p)


def tree_sdf(name: str, wx: float, wy: float, gz: float, h: float) -> str:
    """SDF for a tree: cylindrical trunk + spherical canopy."""
    trunk_h    = h * 0.55
    trunk_r    = 0.22 + 0.06 * (h - 5) / 5
    canopy_r   = h * 0.32
    trunk_lz   = trunk_h / 2
    canopy_lz  = trunk_h + canopy_r * 0.55

    rng = np.random.RandomState(abs(hash(name)) % 2**31)
    lean_r = rng.uniform(-0.04, 0.04)
    lean_p = rng.uniform(-0.04, 0.04)

    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{wx:.2f} {wy:.2f} {gz:.2f} 0 0 {rng.uniform(0, 6.28):.2f}</pose>
      <link name="link">
        <visual name="trunk">
          <pose>0 0 {trunk_lz:.3f} {lean_r:.3f} {lean_p:.3f} 0</pose>
          <geometry><cylinder>
            <radius>{trunk_r:.3f}</radius>
            <length>{trunk_h:.3f}</length>
          </cylinder></geometry>
          <material>
            <ambient>0.32 0.18 0.05 1</ambient>
            <diffuse>0.38 0.21 0.07 1</diffuse>
          </material>
        </visual>
        <collision name="trunk_c">
          <pose>0 0 {trunk_lz:.3f} 0 0 0</pose>
          <geometry><cylinder>
            <radius>{trunk_r:.3f}</radius>
            <length>{trunk_h:.3f}</length>
          </cylinder></geometry>
        </collision>
        <visual name="canopy">
          <pose>0 0 {canopy_lz:.3f} 0 0 0</pose>
          <geometry><sphere>
            <radius>{canopy_r:.3f}</radius>
          </sphere></geometry>
          <material>
            <ambient>0.07 0.38 0.07 1</ambient>
            <diffuse>0.10 0.48 0.10 1</diffuse>
          </material>
        </visual>
        <collision name="canopy_c">
          <pose>0 0 {canopy_lz:.3f} 0 0 0</pose>
          <geometry><sphere>
            <radius>{canopy_r:.3f}</radius>
          </sphere></geometry>
        </collision>
      </link>
    </model>"""


# ─────────────────────────────────────────────────────────────────────────────
# Step 4 – Tree placement
# ─────────────────────────────────────────────────────────────────────────────
def generate_trees(hm: np.ndarray,
                   base_xy: tuple, peak_xy: tuple) -> list:
    """Place 20 trees: majority near the power-line corridor."""
    rng  = np.random.RandomState(777)
    bx, by = base_xy
    px, py = peak_xy
    trees  = []

    def add_tree(wx, wy):
        wx = float(np.clip(wx, -WORLD_SIZE/2 + 8, WORLD_SIZE/2 - 8))
        wy = float(np.clip(wy, -WORLD_SIZE/2 + 8, WORLD_SIZE/2 - 8))
        gz = query_height(hm, wx, wy)
        h  = rng.uniform(5.0, 10.0)
        trees.append((wx, wy, gz, h))

    # 13 trees along corridor
    for _ in range(13):
        t  = rng.uniform(0.0, 1.0)
        wx = bx + (px - bx) * t + rng.uniform(-28, 28)
        wy = by + (py - by) * t + rng.uniform(-20, 20)
        add_tree(wx, wy)

    # 7 trees on slopes
    for _ in range(7):
        wx = rng.uniform(-110, 60)
        wy = rng.uniform(-60, 60)
        add_tree(wx, wy)

    return trees


# ─────────────────────────────────────────────────────────────────────────────
# Step 5 – Assemble SDF world
# ─────────────────────────────────────────────────────────────────────────────
def build_world(hm: np.ndarray) -> None:

    # Find precise tower positions
    bx, by = find_foot_position(hm, *BASE_TOWER_CANDIDATE, target_h=4.0)
    bgz    = query_height(hm, bx, by)

    best_h = -1.0
    px, py = PEAK_TOWER_CANDIDATE
    for dx in np.linspace(-25, 25, 15):
        for dy in np.linspace(-25, 25, 15):
            wx, wy = PEAK_TOWER_CANDIDATE[0] + dx, PEAK_TOWER_CANDIDATE[1] + dy
            h_test = query_height(hm, wx, wy)
            if h_test > best_h:
                best_h = h_test
                px, py = wx, wy
    pgz = query_height(hm, px, py)

    print(f"  Base tower: ({bx:.1f}, {by:.1f}), terrain z = {bgz:.2f} m")
    print(f"  Peak tower: ({px:.1f}, {py:.1f}), terrain z = {pgz:.2f} m")

    # Wire attachment points
    arm_z_local = TOWER_H - 0.8 - 0.9
    b_arm_z = bgz + arm_z_local
    p_arm_z = pgz + arm_z_local
    b_L = (bx, by - ARM_LEN, b_arm_z)
    b_R = (bx, by + ARM_LEN, b_arm_z)
    p_L = (px, py - ARM_LEN, p_arm_z)
    p_R = (px, py + ARM_LEN, p_arm_z)

    # Drone spawn positions
    # Find a flat spot near base tower for drone spawning
    flat_spot, flat_slope = find_flat_spot(hm, bx, by, radius=30.0, max_slope=0.05)
    fx, fy = flat_spot
    fz = query_height(hm, fx, fy)
    print(f"  Drone flat spot: ({fx:.1f}, {fy:.1f}), terrain z = {fz:.2f} m, slope = {flat_slope:.4f}")

    # Place three drones in a line (along world Y axis) with spacing 3 meters
    spacing = 3.0
    drones = []
    for i in range(3):
        offset = (i - 1) * spacing  # -3, 0, +3
        dx = fx
        dy = fy + offset
        dz = query_height(hm, dx, dy) + 12.0  # 12m above terrain
        drones.append((dx, dy, dz))

    # Generate SDF elements
    tower_part = (tower_sdf("base_tower", bx, by, bgz) +
                  "\n" +
                  tower_sdf("peak_tower", px, py, pgz))

    wire_part  = (wire_sdf("wire_L", b_L, p_L) +
                  "\n" +
                  wire_sdf("wire_R", b_R, p_R))

    tree_list  = generate_trees(hm, (bx, by), (px, py))
    tree_part  = "\n".join(
        tree_sdf(f"tree_{i:02d}", tx, ty, tz, th)
        for i, (tx, ty, tz, th) in enumerate(tree_list)
    )

    mesh_uri = f"file://{MESH_PATH}"

    # Full SDF with mesh terrain instead of heightmap
    sdf = f"""<?xml version="1.0" ?>
<!--
  Power-Line Inspection Simulation World
  Generated by generate_world.py
  山脉高度: ~50m  输电塔高度: {TOWER_H}m  树木数量: {N_TREES}
  底部塔: ({bx:.1f}, {by:.1f}, {bgz:.1f})  顶部塔: ({px:.1f}, {py:.1f}, {pgz:.1f})
-->
<sdf version="1.9">
  <world name="power_line_inspection">

    <!-- ═══════════════════════════════════════════════════════
         Core simulation plugins (Gz Sim Garden / Harmonic)
         ═══════════════════════════════════════════════════════ -->
    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
      <engine>gz-physics-dartsim-plugin</engine>
    </plugin>
    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands"/>
    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster"/>
    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin
      filename="gz-sim-imu-system"
      name="gz::sim::systems::Imu"/>
    <plugin
      filename="gz-sim-air-pressure-system"
      name="gz::sim::systems::AirPressure"/>
    <plugin
      filename="gz-sim-magnetometer-system"
      name="gz::sim::systems::Magnetometer"/>
    <plugin
      filename="gz-sim-navsat-system"
      name="gz::sim::systems::NavSat"/>
    <plugin
      filename="gz-sim-contact-system"
      name="gz::sim::systems::Contact"/>

    <!-- ═══════════════════════════════════════════════════════
         Physics
         ═══════════════════════════════════════════════════════ -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- ═══════════════════════════════════════════════════════
         Environment
         ═══════════════════════════════════════════════════════ -->
    <gravity>0 0 -9.8066</gravity>
    <magnetic_field>3.20e-05 -5.00e-07 3.50e-05</magnetic_field>
    <atmosphere type="adiabatic"/>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>30.5728</latitude_deg>
      <longitude_deg>104.0668</longitude_deg>
      <elevation>500.0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <!-- ═══════════════════════════════════════════════════════
         Scene & Lighting
         ═══════════════════════════════════════════════════════ -->
    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.53 0.81 0.98 1</background>
      <shadows>true</shadows>
      <sky></sky>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 500 0 0.4 0.6</pose>
      <diffuse>0.95 0.90 0.80 1</diffuse>
      <specular>0.30 0.30 0.30 1</specular>
      <direction>-0.5 0.3 -0.8</direction>
      <attenuation>
        <range>2000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.0005</quadratic>
      </attenuation>
    </light>

    <light type="directional" name="fill_light">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 100 0 -0.3 2.5</pose>
      <diffuse>0.4 0.45 0.55 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
      <direction>0.5 -0.3 -0.5</direction>
    </light>

    <!-- ═══════════════════════════════════════════════════════
         Flat ground plane
         ═══════════════════════════════════════════════════════ -->
    <model name="ground_plane">
      <static>true</static>
      <pose>0 0 -0.01 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>600 600</size>
            </plane>
          </geometry>
          <surface>
            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
            <contact><ode/></contact>
          </surface>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>600 600</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.28 0.35 0.18 1</ambient>
            <diffuse>0.32 0.40 0.22 1</diffuse>
            <specular>0.02 0.02 0.02 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- ═══════════════════════════════════════════════════════
         Mountain terrain (mesh)
         Size: {WORLD_SIZE}m × {WORLD_SIZE}m × {WORLD_Z_MAX}m
         ═══════════════════════════════════════════════════════ -->
    <model name="mountain_terrain">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <collision name="terrain_collision">
          <geometry>
            <mesh>
              <uri>{mesh_uri}</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <ode><mu>0.8</mu><mu2>0.8</mu2></ode>
            </friction>
          </surface>
        </collision>
        <visual name="terrain_visual">
          <geometry>
            <mesh>
              <uri>{mesh_uri}</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <material>
            <ambient>0.35 0.28 0.18 1</ambient>
            <diffuse>0.45 0.38 0.25 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- ═══════════════════════════════════════════════════════
         Transmission towers
         ═══════════════════════════════════════════════════════ -->
    {tower_part}

    <!-- ═══════════════════════════════════════════════════════
         Power wires (catenary)
         ═══════════════════════════════════════════════════════ -->
    {wire_part}

    <!-- ═══════════════════════════════════════════════════════
         Trees / obstacles  ({N_TREES} total)
         ═══════════════════════════════════════════════════════ -->
    {tree_part}

    <!-- ═══════════════════════════════════════════════════════
         GUI Plugins for camera control and interaction
         ═══════════════════════════════════════════════════════ -->
    <gui>
      <plugin filename="gz-sim-scene3d-plugin" name="Scene3D">
        <engine>ogre2</engine>
        <scene>scene</scene>
        <camera>camera</camera>
        <ambient_light>0.4 0.4 0.4</ambient_light>
        <background_color>0.53 0.81 0.98 1</background_color>
      </plugin>
      <plugin filename="gz-sim-camera-plugin" name="Camera"/>
      <plugin filename="gz-sim-transform-control-plugin" name="Transform Control"/>
      <plugin filename="gz-sim-resource-spawner-plugin" name="Resource Spawner"/>
      <plugin filename="gz-sim-entity-context-menu-plugin" name="Entity Context Menu"/>
      <plugin filename="gz-sim-spawner-plugin" name="Spawn"/>
    </gui>

  </world>
</sdf>
"""

    with open(WORLD_PATH, "w") as f:
        f.write(sdf)
    print(f"  [✓] World SDF → {WORLD_PATH}")

    # Print drone spawn info
    print("\n  ┌─ Drone spawn positions (use in start_px4.sh) ──────────────┐")
    for i, (dx, dy, dz) in enumerate(drones):
        print(f"  │  UAV{i+1}: PX4_GZ_MODEL_POSE=\"{dx:.2f},{dy:.2f},{dz:.2f},0,0,0\"")
    print(f"  │")
    print(f"  │  Base tower world pos : ({bx:.2f}, {by:.2f}, {bgz:.2f})")
    print(f"  │  Peak tower world pos : ({px:.2f}, {py:.2f}, {pgz:.2f})")
    print(f"  │  Wire top (base tower): z = {b_arm_z:.2f} m")
    print(f"  │  Wire top (peak tower): z = {p_arm_z:.2f} m")
    print(f"  └────────────────────────────────────────────────────────────┘")

    # Save spawn config
    import json
    config = {
        "base_tower": {"x": bx, "y": by, "z": bgz,
                       "top_z": bgz + TOWER_H,
                       "wire_z_L": b_arm_z, "arm_y_L": by - ARM_LEN,
                       "wire_z_R": b_arm_z, "arm_y_R": by + ARM_LEN},
        "peak_tower": {"x": px, "y": py, "z": pgz,
                       "top_z": pgz + TOWER_H,
                       "wire_z_L": p_arm_z, "arm_y_L": py - ARM_LEN,
                       "wire_z_R": p_arm_z, "arm_y_R": py + ARM_LEN},
        "drones": [{"id": i+1, "x": dx, "y": dy, "z": dz}
                   for i, (dx, dy, dz) in enumerate(drones)],
        "trees": [{"x": tx, "y": ty, "z": tz, "h": th}
                  for tx, ty, tz, th in tree_list],
    }
    cfg_path = os.path.join(OUT_DIR, "world_config.json")
    with open(cfg_path, "w") as f:
        json.dump(config, f, indent=2)
    print(f"\n  [✓] World config → {cfg_path}")
    return config


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import time
    t0 = time.time()
    print("═" * 60)
    print("  UAV Inspection World Generator")
    print("═" * 60)

    print("\n[1/3] Generating mountain heightmap …")
    hm = build_heightmap()

    print("\n[2/3] Writing terrain mesh file …")
    write_obj_mesh(hm, MESH_PATH)

    print("\n[3/3] Building Gazebo SDF world …")
    build_world(hm)

    print(f"\n  Done in {time.time()-t0:.1f}s")
    print("═" * 60)
