"""
Full runnable script: mat preview + fully enclosed low-poly robot (car) model and tight bounding hitbox.

Controls:
 - Left mouse drag: rotate camera (yaw and pitch)
 - No keyboard movement or mouse-wheel zoom (disabled)
 - Robot spawns centered between inner wall face and mat outer edge, aligned parallel to walls
 - Arrow above robot indicates forward direction (rotates with robot)
"""

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from math import sin, cos, radians, atan2, sqrt, tan, pi

# ---------- Config ----------
SHOW_MARKER_SQUARES = True
MARKER_SQUARE_COLOR = (0.8, 0.8, 0.8)  # light grey (marker outlines solid)
MARKER_SQUARE_SIZE_MM = 50.0
MARKER_SQUARE_SIZE_CM = MARKER_SQUARE_SIZE_MM / 10.0  # 5.0 cm
MARKER_STROKE_MM = 1.0
MARKER_STROKE_CM = MARKER_STROKE_MM / 10.0  # 0.1 cm

# Circle parameters (85 mm diameter)
CIRCLE_DIAMETER_MM = 85.0
CIRCLE_DIAMETER_CM = CIRCLE_DIAMETER_MM / 10.0  # 8.5 cm
CIRCLE_RADIUS_CM = CIRCLE_DIAMETER_CM / 2.0     # 4.25 cm
CIRCLE_SEGMENTS = 48                            # smoothness for outline

# CMYK 20,0,100,0 -> convert to RGB
C, M, Y, K = 0.20, 0.0, 1.00, 0.0
CIRCLE_RGB = ((1.0 - C) * (1.0 - K), (1.0 - M) * (1.0 - K), (1.0 - Y) * (1.0 - K))

# dash/gap lengths for stipple (in cm)
DASH_LEN_CM = 2.0   # 2 cm dash
GAP_LEN_CM = 1.0    # 1 cm gap

DEBUG_DRAW_MARKERS = False  # set True to draw small debug cubes at markpoints

# ---------- Scale and geometry (1 unit = 1 cm) ----------
MAT_HALF = 150.0
BORDER_WIDTH = 10.0
OUTER_HALF = MAT_HALF + BORDER_WIDTH  # 160

INNER_HALF = 40.0
INNER_WALL_HALF = 50.0

# Vertical positions
FLOOR_TOP_Y = -1.0
FLOOR_THICKNESS = 0.1
FLOOR_BOTTOM_Y = FLOOR_TOP_Y - FLOOR_THICKNESS

WALL_HEIGHT = 8.9
WALL_BOTTOM_Y = FLOOR_TOP_Y
WALL_THICKNESS = 1.0

EPS = 0.001

# Colors
COLOR_FLOOR = (1.0, 1.0, 1.0)
COLOR_BORDER = (0.0, 0.0, 0.5)
COLOR_INNER_SQUARE = (0.0, 0.0, 0.5)
COLOR_WALLS = (0.0, 0.0, 0.0)
COLOR_ORANGE = (1.0, 0.45, 0.0)
COLOR_BLUE = (0.0, 0.45, 1.0)
COLOR_GRID = (0.8, 0.8, 0.8)  # light grey stippled lines
COLOR_MARKER_DEBUG = (1.0, 0.0, 0.0)

# Markpoint parameters
ROW_A_CM = 40.0   # 400 mm -> 40 cm
ROW_B_CM = 60.0   # 600 mm -> 60 cm
ACROSS_POSITIONS = (0.0, 50.0, 100.0)  # cm across the zone

OUTER_INSET_CM = 42.0

# ---------- Basic box drawer ----------
def draw_box(x_min, y_min, z_min, x_max, y_max, z_max, color):
    glColor3f(*color)
    glBegin(GL_QUADS)
    # Top
    glVertex3f(x_min, y_max, z_min)
    glVertex3f(x_max, y_max, z_min)
    glVertex3f(x_max, y_max, z_max)
    glVertex3f(x_min, y_max, z_max)
    # Bottom
    glVertex3f(x_min, y_min, z_min)
    glVertex3f(x_min, y_min, z_max)
    glVertex3f(x_max, y_min, z_max)
    glVertex3f(x_max, y_min, z_min)
    # Front (+Z)
    glVertex3f(x_min, y_min, z_max)
    glVertex3f(x_max, y_min, z_max)
    glVertex3f(x_max, y_max, z_max)
    glVertex3f(x_min, y_max, z_max)
    # Back (-Z)
    glVertex3f(x_min, y_min, z_min)
    glVertex3f(x_min, y_max, z_min)
    glVertex3f(x_max, y_max, z_min)
    glVertex3f(x_max, y_min, z_min)
    # Left (-X)
    glVertex3f(x_min, y_min, z_min)
    glVertex3f(x_min, y_min, z_max)
    glVertex3f(x_min, y_max, z_max)
    glVertex3f(x_min, y_max, z_min)
    # Right (+X)
    glVertex3f(x_max, y_min, z_min)
    glVertex3f(x_max, y_max, z_min)
    glVertex3f(x_max, y_max, z_max)
    glVertex3f(x_max, y_min, z_max)
    glEnd()

# ---------- Scene pieces ----------
def ground_with_hole():
    draw_box(-MAT_HALF, FLOOR_BOTTOM_Y, INNER_HALF, MAT_HALF, FLOOR_TOP_Y, MAT_HALF, COLOR_FLOOR)
    draw_box(-MAT_HALF, FLOOR_BOTTOM_Y, -MAT_HALF, MAT_HALF, FLOOR_TOP_Y, -INNER_HALF, COLOR_FLOOR)
    draw_box(-MAT_HALF, FLOOR_BOTTOM_Y, -INNER_HALF, -INNER_HALF, FLOOR_TOP_Y, INNER_HALF, COLOR_FLOOR)
    draw_box(INNER_HALF, FLOOR_BOTTOM_Y, -INNER_HALF, MAT_HALF, FLOOR_TOP_Y, INNER_HALF, COLOR_FLOOR)

def border():
    draw_box(-OUTER_HALF, FLOOR_BOTTOM_Y, MAT_HALF, OUTER_HALF, FLOOR_TOP_Y, OUTER_HALF, COLOR_BORDER)
    draw_box(-OUTER_HALF, FLOOR_BOTTOM_Y, -OUTER_HALF, OUTER_HALF, FLOOR_TOP_Y, -MAT_HALF, COLOR_BORDER)
    draw_box(-OUTER_HALF, FLOOR_BOTTOM_Y, -OUTER_HALF, -MAT_HALF, FLOOR_TOP_Y, OUTER_HALF, COLOR_BORDER)
    draw_box(MAT_HALF, FLOOR_BOTTOM_Y, -OUTER_HALF, OUTER_HALF, FLOOR_TOP_Y, OUTER_HALF, COLOR_BORDER)

def inner_square_plate():
    half = INNER_HALF - EPS
    top_y = FLOOR_TOP_Y + EPS
    bottom_y = FLOOR_TOP_Y + EPS * 0.1
    draw_box(-half, bottom_y, -half, half, top_y, half, COLOR_INNER_SQUARE)

def inner_walls():
    outer = INNER_WALL_HALF
    inner = INNER_WALL_HALF - WALL_THICKNESS
    draw_box(-outer, WALL_BOTTOM_Y, inner, outer, WALL_HEIGHT, outer, COLOR_WALLS)
    draw_box(-outer, WALL_BOTTOM_Y, -outer, outer, WALL_HEIGHT, -inner, COLOR_WALLS)
    draw_box(-outer, WALL_BOTTOM_Y, -outer, -inner, WALL_HEIGHT, outer, COLOR_WALLS)
    draw_box(inner, WALL_BOTTOM_Y, -outer, outer, WALL_HEIGHT, outer, COLOR_WALLS)

def outer_walls_inside_border():
    inner = MAT_HALF - WALL_THICKNESS
    outer = MAT_HALF
    draw_box(-outer, WALL_BOTTOM_Y, inner, outer, WALL_HEIGHT, outer, COLOR_WALLS)
    draw_box(-outer, WALL_BOTTOM_Y, -outer, outer, WALL_HEIGHT, -inner, COLOR_WALLS)
    draw_box(-outer, WALL_BOTTOM_Y, -outer, -inner, WALL_HEIGHT, outer, COLOR_WALLS)
    draw_box(inner, WALL_BOTTOM_Y, -outer, outer, WALL_HEIGHT, outer, COLOR_WALLS)

# ---------- Corner stripe helpers ----------
def outer_edge_targets_for_corner(outer_corner_x, outer_corner_z, inset_cm=OUTER_INSET_CM):
    sx = -1.0 if outer_corner_x < 0 else 1.0
    sz = -1.0 if outer_corner_z < 0 else 1.0
    tx_x = outer_corner_x + (inset_cm if sx < 0 else -inset_cm)
    tx_z_x = outer_corner_z
    tz_z = outer_corner_z + (inset_cm if sz < 0 else -inset_cm)
    tz_x_z = outer_corner_x
    return (tx_x, tx_z_x), (tz_x_z, tz_z)

def build_clockwise_segments():
    segments = []
    inner_face = INNER_WALL_HALF - WALL_THICKNESS

    inner_corners = [
        (-inner_face,  inner_face),
        ( inner_face,  inner_face),
        ( inner_face, -inner_face),
        (-inner_face, -inner_face)
    ]

    outer_corners = [
        (-MAT_HALF,  MAT_HALF),
        ( MAT_HALF,  MAT_HALF),
        ( MAT_HALF, -MAT_HALF),
        (-MAT_HALF, -MAT_HALF)
    ]

    for (ix, iz), (ox, oz) in zip(inner_corners, outer_corners):
        target_x_alongX, target_z_alongZ = outer_edge_targets_for_corner(ox, oz, OUTER_INSET_CM)
        t1x, t1z = target_x_alongX
        t2x, t2z = target_z_alongZ

        out_x = ox
        out_z = oz
        mag = sqrt(out_x*out_x + out_z*out_z)
        if mag == 0:
            out_vx, out_vz = 0.0, 0.0
        else:
            out_vx, out_vz = out_x / mag, out_z / mag

        cand = [
            (ix, iz, t1x, t1z),
            (ix, iz, t2x, t2z)
        ]

        for seg in cand:
            sx, sz, ex, ez = seg
            dir_x = ex - sx
            dir_z = ez - sz
            dot = dir_x * out_vx + dir_z * out_vz
            if dot > 0.0:
                segments.append((sx, sz, ex, ez))

    def seg_mid_angle(seg):
        sx, sz, ex, ez = seg
        mx = (sx + ex) / 2.0
        mz = (sz + ez) / 2.0
        ang = atan2(mx, mz)
        return ang

    segments_sorted = sorted(segments, key=seg_mid_angle, reverse=False)
    return segments_sorted

def draw_stripe_between(start_x, start_z, end_x, end_z, width_cm, y_bottom, y_top, color):
    dx = end_x - start_x
    dz = end_z - start_z
    length = sqrt(dx * dx + dz * dz)
    if length < 0.5:
        return
    angle_rad = atan2(dx, dz)
    cx = (start_x + end_x) / 2.0
    cz = (start_z + end_z) / 2.0
    dir_x = sin(angle_rad)
    dir_z = cos(angle_rad)
    perp_x = -dir_z
    perp_z = dir_x
    half_w = width_cm / 2.0
    half_lx = (length / 2.0) * dir_x
    half_lz = (length / 2.0) * dir_z

    p1x = cx - half_w * perp_x - half_lx
    p1z = cz - half_w * perp_z - half_lz

    p2x = cx + half_w * perp_x - half_lx
    p2z = cz + half_w * perp_z - half_lz

    p3x = cx + half_w * perp_x + half_lx
    p3z = cz + half_w * perp_z + half_lz

    p4x = cx - half_w * perp_x + half_lx
    p4z = cz - half_w * perp_z + half_lz

    glColor3f(*color)
    glBegin(GL_QUADS)
    # top face
    glVertex3f(p1x, y_top, p1z)
    glVertex3f(p2x, y_top, p2z)
    glVertex3f(p3x, y_top, p3z)
    glVertex3f(p4x, y_top, p4z)
    # bottom face (thin)
    glVertex3f(p1x, y_bottom, p1z)
    glVertex3f(p4x, y_bottom, p4z)
    glVertex3f(p3x, y_bottom, p3z)
    glVertex3f(p2x, y_bottom, p2z)
    glEnd()

def draw_alternating_segments():
    stripe_width = 2.0
    stripe_y_top = FLOOR_TOP_Y + 0.6
    stripe_y_bottom = stripe_y_top - 0.5

    segments = build_clockwise_segments()
    for i, seg in enumerate(segments):
        col = COLOR_BLUE if (i % 2) == 0 else COLOR_ORANGE
        sx, sz, ex, ez = seg
        draw_stripe_between(sx, sz, ex, ez, stripe_width, stripe_y_bottom, stripe_y_top, col)

# ---------- Markpoint computation ----------
def compute_markpoints():
    inner_face = INNER_WALL_HALF - WALL_THICKNESS
    markpoints = {2: [], 4: [], 6: [], 8: []}

    # Zone 2 (top middle)
    z_base = inner_face
    for across in ACROSS_POSITIONS:
        x = -50.0 + across
        markpoints[2].append((x, z_base + ROW_A_CM, 'A'))
        markpoints[2].append((x, z_base + ROW_B_CM, 'B'))

    # Zone 8 (bottom middle)
    z_base = -inner_face
    for across in ACROSS_POSITIONS:
        x = -50.0 + across
        markpoints[8].append((x, z_base - ROW_A_CM, 'A'))
        markpoints[8].append((x, z_base - ROW_B_CM, 'B'))

    # Zone 4 (left middle)
    x_base = -inner_face
    for across in ACROSS_POSITIONS:
        z = 50.0 - across
        markpoints[4].append((x_base - ROW_A_CM, z, 'A'))
        markpoints[4].append((x_base - ROW_B_CM, z, 'B'))

    # Zone 6 (right middle)
    x_base = inner_face
    for across in ACROSS_POSITIONS:
        z = 50.0 - across
        markpoints[6].append((x_base + ROW_A_CM, z, 'A'))
        markpoints[6].append((x_base + ROW_B_CM, z, 'B'))

    return markpoints

# ---------- Marker square solid hollow outline (solid lines) ----------
def draw_marker_square_solid_outline(cx, cz, color=MARKER_SQUARE_COLOR):
    half = MARKER_SQUARE_SIZE_CM / 2.0  # 2.5 cm
    stroke = MARKER_STROKE_CM           # 0.1 cm

    y_top = FLOOR_TOP_Y + 0.02
    y_bottom = y_top - 0.2

    ox_min = cx - half
    ox_max = cx + half
    oz_min = cz - half
    oz_max = cz + half

    ix_min = ox_min + stroke
    ix_max = ox_max - stroke
    iz_min = oz_min + stroke
    iz_max = oz_max - stroke

    if ix_min >= ix_max or iz_min >= iz_max:
        draw_box(ox_min, y_bottom, oz_min, ox_max, y_top, oz_max, color)
        return

    # Top strip (solid)
    draw_box(ox_min, y_bottom, iz_max, ox_max, y_top, oz_max, color)
    # Bottom strip (solid)
    draw_box(ox_min, y_bottom, oz_min, ox_max, y_top, iz_min, color)
    # Left strip (solid)
    draw_box(ox_min, y_bottom, iz_min, ix_min, y_top, iz_max, color)
    # Right strip (solid)
    draw_box(ix_max, y_bottom, iz_min, ox_max, y_top, iz_max, color)

# ---------- Draw hollow circle outline at marker ----------
def draw_circle_outline(cx, cz, outer_radius_cm, stroke_cm, segments, color_rgb):
    inner_radius = max(0.0, outer_radius_cm - stroke_cm)
    y = FLOOR_TOP_Y + 0.03  # slightly above mat surface

    glColor3f(*color_rgb)
    glBegin(GL_TRIANGLE_STRIP)
    for i in range(segments + 1):
        theta = (2.0 * pi * i) / segments
        ox = cx + outer_radius_cm * cos(theta)
        oz = cz + outer_radius_cm * sin(theta)
        ix = cx + inner_radius * cos(theta)
        iz = cz + inner_radius * sin(theta)
        glVertex3f(ox, y, oz)
        glVertex3f(ix, y, iz)
    glEnd()

# ---------- Utility overlap checks ----------
def segment_overlaps_square(ax, az, bx, bz, square_cx, square_cz, half_size_cm):
    sx_min = square_cx - half_size_cm
    sx_max = square_cx + half_size_cm
    sz_min = square_cz - half_size_cm
    sz_max = square_cz + half_size_cm

    if abs(ax - bx) < 1e-6:
        x = ax
        if x < sx_min or x > sx_max:
            return False
        z0 = min(az, bz)
        z1 = max(az, bz)
        return not (z1 < sz_min or z0 > sz_max)
    elif abs(az - bz) < 1e-6:
        z = az
        if z < sz_min or z > sz_max:
            return False
        x0 = min(ax, bx)
        x1 = max(ax, bx)
        return not (x1 < sx_min or x0 > sx_max)
    else:
        return False

def segment_overlaps_circle(ax, az, bx, bz, circle_cx, circle_cz, radius_cm):
    if abs(ax - bx) < 1e-6:
        x = ax
        dx = abs(x - circle_cx)
        if dx > radius_cm:
            return False
        dz = sqrt(max(0.0, radius_cm * radius_cm - dx * dx))
        cz_min = circle_cz - dz
        cz_max = circle_cz + dz
        z0 = min(az, bz)
        z1 = max(az, bz)
        return not (z1 < cz_min or z0 > cz_max)
    elif abs(az - bz) < 1e-6:
        z = az
        dz = abs(z - circle_cz)
        if dz > radius_cm:
            return False
        dx = sqrt(max(0.0, radius_cm * radius_cm - dz * dz))
        cx_min = circle_cx - dx
        cx_max = circle_cx + dx
        x0 = min(ax, bx)
        x1 = max(ax, bx)
        return not (x1 < cx_min or x0 > cx_max)
    else:
        return False

# ---------- Draw stippled axis-aligned segments ----------
def draw_stippled_axis_aligned(ax, az, bx, bz, stroke_cm, dash_len_cm, gap_len_cm, color, marker_squares, circles, center_clip_half):
    combined_squares = list(marker_squares)
    combined_squares.append((0.0, 0.0, center_clip_half))

    if abs(ax - bx) < 1e-6:
        x = ax
        z0 = az
        z1 = bz
        total_len = abs(z1 - z0)
        dir_sign = 1.0 if z1 >= z0 else -1.0
        pos = 0.0
        while pos < total_len - 1e-6:
            seg_start = pos
            seg_end = min(pos + dash_len_cm, total_len)
            s_z = z0 + dir_sign * seg_start
            e_z = z0 + dir_sign * seg_end
            overlap = False
            for (mcx, mcz, half) in combined_squares:
                if segment_overlaps_square(x, s_z, x, e_z, mcx, mcz, half):
                    overlap = True
                    break
            if not overlap:
                for (ccx, ccz, cr) in circles:
                    if segment_overlaps_circle(x, s_z, x, e_z, ccx, ccz, cr):
                        overlap = True
                        break
            if not overlap:
                half_w = stroke_cm / 2.0
                y_top = FLOOR_TOP_Y + 0.02
                y_bottom = y_top - 0.2
                draw_box(x - half_w, y_bottom, min(s_z, e_z), x + half_w, y_top, max(s_z, e_z), color)
            pos += dash_len_cm + gap_len_cm
    elif abs(az - bz) < 1e-6:
        z = az
        x0 = ax
        x1 = bx
        total_len = abs(x1 - x0)
        dir_sign = 1.0 if x1 >= x0 else -1.0
        pos = 0.0
        while pos < total_len - 1e-6:
            seg_start = pos
            seg_end = min(pos + dash_len_cm, total_len)
            s_x = x0 + dir_sign * seg_start
            e_x = x0 + dir_sign * seg_end
            overlap = False
            for (mcx, mcz, half) in combined_squares:
                if segment_overlaps_square(s_x, z, e_x, z, mcx, mcz, half):
                    overlap = True
                    break
            if not overlap:
                for (ccx, ccz, cr) in circles:
                    if segment_overlaps_circle(s_x, z, e_x, z, ccx, ccz, cr):
                        overlap = True
                        break
            if not overlap:
                half_w = stroke_cm / 2.0
                y_top = FLOOR_TOP_Y + 0.02
                y_bottom = y_top - 0.2
                draw_box(min(s_x, e_x), y_bottom, z - half_w, max(s_x, e_x), y_top, z + half_w, color)
            pos += dash_len_cm + gap_len_cm
    else:
        return

# ---------- Build connections and extensions ----------
def build_marker_connections(markpoints):
    segments = []
    for zone in (2, 8):
        rows = {}
        for (x, z, row) in markpoints[zone]:
            rows.setdefault(row, []).append((x, z))
        for pts in rows.values():
            pts_sorted = sorted(pts, key=lambda p: p[0])
            for i in range(len(pts_sorted) - 1):
                ax, az = pts_sorted[i]
                bx, bz = pts_sorted[i+1]
                segments.append((ax, az, bx, bz))
    for zone in (4, 6):
        rows = {}
        for (x, z, row) in markpoints[zone]:
            rows.setdefault(row, []).append((x, z))
        for pts in rows.values():
            pts_sorted = sorted(pts, key=lambda p: -p[1])
            for i in range(len(pts_sorted) - 1):
                ax, az = pts_sorted[i]
                bx, bz = pts_sorted[i+1]
                segments.append((ax, az, bx, bz))
    for zone in (2, 4, 6, 8):
        rows = {}
        for (x, z, row) in markpoints[zone]:
            rows.setdefault(row, []).append((x, z))
        a_pts = rows.get('A', [])
        b_pts = rows.get('B', [])
        if zone in (2, 8):
            for ax, az in a_pts:
                for bx, bz in b_pts:
                    if abs(bx - ax) < 1e-3:
                        segments.append((ax, az, bx, bz))
                        break
        else:
            for ax, az in a_pts:
                for bx, bz in b_pts:
                    if abs(bz - az) < 1e-3:
                        segments.append((ax, az, bx, bz))
                        break
    return segments

def build_edge_extensions(markpoints, circle_radius_cm):
    segments = []
    eps = 1e-3
    for (x, z, row) in markpoints[2]:
        if abs(x - 0.0) < eps:
            start_z = z + circle_radius_cm
            segments.append((x, start_z, x, MAT_HALF))
    for (x, z, row) in markpoints[8]:
        if abs(x - 0.0) < eps:
            start_z = z - circle_radius_cm
            segments.append((x, start_z, x, -MAT_HALF))
    for (x, z, row) in markpoints[4]:
        if abs(z - 0.0) < eps:
            start_x = x - circle_radius_cm
            segments.append((start_x, z, -MAT_HALF, z))
    for (x, z, row) in markpoints[6]:
        if abs(z - 0.0) < eps:
            start_x = x + circle_radius_cm
            segments.append((start_x, z, MAT_HALF, z))
    return segments

def build_interior_stippled_extensions(markpoints, circle_radius_cm):
    segments = []
    eps = 1e-3
    inner_face = INNER_WALL_HALF - WALL_THICKNESS
    for (x, z, row) in markpoints[2]:
        if abs(x - 0.0) < eps:
            start_z = z - circle_radius_cm
            segments.append((x, start_z, x, inner_face))
    for (x, z, row) in markpoints[8]:
        if abs(x - 0.0) < eps:
            start_z = z + circle_radius_cm
            segments.append((x, start_z, x, -inner_face))
    for (x, z, row) in markpoints[4]:
        if abs(z - 0.0) < eps:
            start_x = x + circle_radius_cm
            segments.append((start_x, z, -inner_face, z))
    for (x, z, row) in markpoints[6]:
        if abs(z - 0.0) < eps:
            start_x = x - circle_radius_cm
            segments.append((start_x, z, inner_face, z))
    return segments

# ---------- Low-poly enclosed car (robot) and bounding hitbox ----------
def draw_cylinder_x(center_x, center_y, center_z, radius, half_width, segments, color):
    """Draw a short cylinder aligned along X with closed end caps for a solid wheel."""
    glColor3f(*color)
    x0 = center_x - half_width
    x1 = center_x + half_width

    # Rounded side surface
    glBegin(GL_QUADS)
    for i in range(segments):
        a0 = (2.0 * pi * i) / segments
        a1 = (2.0 * pi * (i + 1)) / segments
        z0 = center_z + radius * cos(a0)
        z1 = center_z + radius * cos(a1)
        y0 = center_y + radius * sin(a0)
        y1 = center_y + radius * sin(a1)
        glVertex3f(x0, y0, z0)
        glVertex3f(x1, y0, z0)
        glVertex3f(x1, y1, z1)
        glVertex3f(x0, y1, z1)
    glEnd()

    # End caps to close the wheel faces
    for cap_x in (x0, x1):
        glBegin(GL_TRIANGLE_FAN)
        glVertex3f(cap_x, center_y, center_z)
        for i in range(segments + 1):
            a = (2.0 * pi * i) / segments
            z = center_z + radius * cos(a)
            y = center_y + radius * sin(a)
            glVertex3f(cap_x, y, z)
        glEnd()

def draw_wheel(cx, cz, wheel_radius_cm, wheel_half_width_cm, y_center, color=(0.05,0.05,0.05)):
    draw_cylinder_x(cx, y_center, cz, wheel_radius_cm, wheel_half_width_cm, segments=20, color=color)

def draw_enclosed_lowpoly_car(x, z, yaw_deg):
    """
    Fully enclosed low-poly robot drawn as one rigid object.
    The entire body, wheels, wireframe box, and directional arrow rotate together.
    """
    glPushMatrix()
    glTranslatef(x, 0.0, z)
    glRotatef(yaw_deg, 0.0, 1.0, 0.0)

    # Dimensions (cm)
    wheel_radius = 3.25
    wheel_half_width = 2.5 / 2.0
    wheelspan = 11.0
    body_bottom = 1.2
    body_height = 7.5
    body_length = 21.5
    body_width = 8.0
    rear_wheel_center_from_rear = 4.5

    half_body_len = body_length / 2.0
    half_body_w = body_width / 2.0
    half_wheelspan = wheelspan / 2.0
    body_top = body_bottom + body_height

    # Axles
    rear_axle_local_z = -half_body_len + rear_wheel_center_from_rear
    front_axle_local_z = rear_axle_local_z + (body_length - 2 * rear_wheel_center_from_rear)

    left_x = -half_wheelspan
    right_x = half_wheelspan
    wheel_y_center = FLOOR_TOP_Y + wheel_radius

    # Draw wheels in local robot space
    draw_wheel(left_x, rear_axle_local_z, wheel_radius, wheel_half_width, wheel_y_center)
    draw_wheel(right_x, rear_axle_local_z, wheel_radius, wheel_half_width, wheel_y_center)
    draw_wheel(left_x, front_axle_local_z, wheel_radius, wheel_half_width, wheel_y_center)
    draw_wheel(right_x, front_axle_local_z, wheel_radius, wheel_half_width, wheel_y_center)

    # Body vertices in local robot space
    y0 = FLOOR_TOP_Y + body_bottom
    base_top = FLOOR_TOP_Y + (body_bottom + body_height * 0.35)
    roof_y = FLOOR_TOP_Y + body_bottom + (body_height * 0.6)
    roof_top = roof_y + 1.0

    bl = (-half_body_w, y0, -half_body_len)
    br = ( half_body_w, y0, -half_body_len)
    fr = ( half_body_w, y0,  half_body_len)
    fl = (-half_body_w, y0,  half_body_len)

    bl_t = (-half_body_w, base_top, -half_body_len)
    br_t = ( half_body_w, base_top, -half_body_len)
    fr_t = ( half_body_w, base_top,  half_body_len)
    fl_t = (-half_body_w, base_top,  half_body_len)

    roof_len = body_length * 0.5
    roof_w = body_width * 0.8
    hr_len = roof_len / 2.0
    hr_w = roof_w / 2.0
    r_bl = (-hr_w, roof_y, -hr_len)
    r_br = ( hr_w, roof_y, -hr_len)
    r_fr = ( hr_w, roof_y,  hr_len)
    r_fl = (-hr_w, roof_y,  hr_len)
    r_bl_t = (-hr_w, roof_top, -hr_len)
    r_br_t = ( hr_w, roof_top, -hr_len)
    r_fr_t = ( hr_w, roof_top,  hr_len)
    r_fl_t = (-hr_w, roof_top,  hr_len)

    # Body faces
    glColor3f(0.15, 0.35, 0.8)
    glBegin(GL_QUADS)
    for v in (bl, br, fr, fl):
        glVertex3f(*v)
    glEnd()

    glBegin(GL_QUADS)
    for v in (bl, fl, fl_t, bl_t):
        glVertex3f(*v)
    for v in (br, br_t, fr_t, fr):
        glVertex3f(*v)
    glEnd()

    glBegin(GL_QUADS)
    for v in (bl_t, br_t, br, bl):
        glVertex3f(*v)
    glEnd()

    glBegin(GL_QUADS)
    for v in (fl_t, fr_t, r_fr, r_fl):
        glVertex3f(*v)
    glEnd()

    glBegin(GL_QUADS)
    for v in (r_bl_t, r_br_t, r_fr_t, r_fl_t):
        glVertex3f(*v)
    for v in (r_bl, r_fl, r_fl_t, r_bl_t):
        glVertex3f(*v)
    for v in (r_br, r_br_t, r_fr_t, r_fr):
        glVertex3f(*v)
    for v in (r_bl, r_br, r_br_t, r_bl_t):
        glVertex3f(*v)
    for v in (r_fl, r_fr, r_fr_t, r_fl_t):
        glVertex3f(*v)
    glEnd()

    glBegin(GL_QUADS)
    for v in (bl_t, fl_t, r_fl, r_bl):
        glVertex3f(*v)
    for v in (br_t, r_br, r_fr, fr_t):
        glVertex3f(*v)
    glEnd()

    # Headlights
    glColor3f(1.0, 0.9, 0.6)
    hx_l = (-half_body_w * 0.6, base_top - 0.2, half_body_len + 0.01)
    hx_r = ( half_body_w * 0.6, base_top - 0.2, half_body_len + 0.01)
    for hx in (hx_l, hx_r):
        draw_box(hx[0] - 0.6, hx[1] - 0.2, hx[2] - 0.2, hx[0] + 0.6, hx[1] + 0.2, hx[2] + 0.2, (1.0, 0.9, 0.6))

    # Windshield
    glColor3f(0.05, 0.15, 0.25)
    glBegin(GL_QUADS)
    glVertex3f(-hr_w * 0.9, roof_y + 0.5, hr_len)
    glVertex3f(-half_body_w * 0.9, base_top - 0.1, half_body_len)
    glVertex3f( half_body_w * 0.9, base_top - 0.1, half_body_len)
    glVertex3f( hr_w * 0.9, roof_y + 0.5, hr_len)
    glEnd()

    # Directional arrow in local space
    arrow_height = roof_top + 6.0
    base_local_z = half_body_len - 2.0
    tip_local_z = half_body_len + 3.0
    glColor3f(1.0, 0.2, 0.2)
    glLineWidth(3.0)
    glBegin(GL_LINES)
    glVertex3f(0.0, arrow_height, base_local_z)
    glVertex3f(0.0, arrow_height, tip_local_z)
    glEnd()
    glBegin(GL_TRIANGLES)
    glVertex3f(0.0, arrow_height, tip_local_z)
    glVertex3f(-1.5, arrow_height, base_local_z)
    glVertex3f( 1.5, arrow_height, base_local_z)
    glEnd()

    # Tight wireframe box around the robot in local space
    body_length = 21.5
    body_width = 8.0
    wheel_radius = 3.25
    wheel_half_width = 2.5 / 2.0
    half_len = body_length / 2.0
    half_w = body_width / 2.0
    margin = max(wheel_radius, wheel_half_width) + 0.5
    lx = half_w + margin
    lz = half_len + margin
    y0 = FLOOR_TOP_Y + 1.2
    y1 = y0 + 7.5
    corners = [
        (-lx, y0, -lz),
        ( lx, y0, -lz),
        ( lx, y0,  lz),
        (-lx, y0,  lz),
        (-lx, y1, -lz),
        ( lx, y1, -lz),
        ( lx, y1,  lz),
        (-lx, y1,  lz),
    ]
    glColor3f(0.0, 0.0, 0.0)
    glLineWidth(2.0)
    glBegin(GL_LINES)
    for i in range(4):
        a = corners[i]; b = corners[(i + 1) % 4]
        glVertex3f(*a); glVertex3f(*b)
    for i in range(4, 8):
        a = corners[i]; b = corners[4 + ((i + 1) % 4)]
        glVertex3f(*a); glVertex3f(*b)
    for i in range(4):
        a = corners[i]; b = corners[i + 4]
        glVertex3f(*a); glVertex3f(*b)
    glEnd()

    glPopMatrix()

# ---------- Spawn logic: place robot centered between inner face and mat outer edge ----------
def compute_spawn_between_inner_and_outer():
    """
    Compute spawn position so the car is centered in the corridor between the inner wall face
    and the mat outer edge, with its facing direction parallel to the walls.
    """
    inner_face = INNER_WALL_HALF - WALL_THICKNESS
    outer_edge = MAT_HALF
    spawn_z = (inner_face + outer_edge) / 2.0
    spawn_x = 0.0
    spawn_yaw_deg = 90.0  # parallel to the inner/outer wall edges (faces along +X)
    return spawn_x, spawn_z, spawn_yaw_deg

# ---------- Main (third-person, mouse rotation only, start looking down) ----------
def main():
    pygame.init()
    display = (1280, 720)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)

    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glShadeModel(GL_SMOOTH)
    glClearColor(0.2, 0.2, 0.2, 1.0)

    max_distance = 2000
    fov = 45
    half_size = OUTER_HALF + 20

    # Camera: start looking down from above
    yaw_deg = 0.0
    pitch_deg = 80.0   # steep downward angle
    min_pitch = 5.0
    max_pitch = 89.0

    target_x = 0.0
    target_z = 0.0
    target_y = FLOOR_TOP_Y + 0.0
    cam_distance = 220.0  # cm

    mouse_down = False
    last_mouse_pos = (0, 0)
    mouse_sensitivity = 0.25  # degrees per pixel

    clock = pygame.time.Clock()

    markpoints = compute_markpoints()
    marker_half = MARKER_SQUARE_SIZE_CM / 2.0  # 2.5 cm

    marker_squares = []
    circles = []
    for zone in (2, 4, 6, 8):
        for (mx, mz, row) in markpoints[zone]:
            marker_squares.append((mx, mz, marker_half))
            circles.append((mx, mz, CIRCLE_RADIUS_CM))

    connections = build_marker_connections(markpoints)
    edge_extensions = build_edge_extensions(markpoints, CIRCLE_RADIUS_CM)
    interior_stippled_extensions = build_interior_stippled_extensions(markpoints, CIRCLE_RADIUS_CM)

    pygame.mouse.set_visible(True)

    # Compute spawn position between inner and outer wall
    spawn_x, spawn_z, spawn_yaw_deg = compute_spawn_between_inner_and_outer()

    while True:
        dt_ms = clock.tick(60)
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                return
            if event.type == MOUSEBUTTONDOWN:
                if event.button == 1:
                    mouse_down = True
                    last_mouse_pos = event.pos
            if event.type == MOUSEBUTTONUP:
                if event.button == 1:
                    mouse_down = False
            if event.type == MOUSEMOTION and mouse_down:
                mx, my = event.pos
                lx, ly = last_mouse_pos
                dx = mx - lx
                dy = my - ly
                last_mouse_pos = (mx, my)

                yaw_deg += dx * mouse_sensitivity
                # invert vertical drag so dragging up looks down
                pitch_deg -= dy * mouse_sensitivity

                if pitch_deg < min_pitch:
                    pitch_deg = min_pitch
                if pitch_deg > max_pitch:
                    pitch_deg = max_pitch

        # compute camera position from spherical coordinates around target
        yaw_rad = radians(yaw_deg)
        pitch_rad = radians(pitch_deg)
        cam_x = target_x + cam_distance * cos(pitch_rad) * sin(yaw_rad)
        cam_y = target_y + cam_distance * sin(pitch_rad)
        cam_z = target_z + cam_distance * cos(pitch_rad) * cos(yaw_rad)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        gluPerspective(fov, (display[0] / display[1]), 0.1, max_distance)
        gluLookAt(cam_x, cam_y, cam_z, target_x, target_y, target_z, 0.0, 1.0, 0.0)

        # Draw scene
        ground_with_hole()
        border()
        inner_square_plate()
        draw_alternating_segments()
        inner_walls()
        outer_walls_inside_border()

        # marker square outlines (solid)
        if SHOW_MARKER_SQUARES:
            for zone in (2, 4, 6, 8):
                for (mx, mz, row) in markpoints[zone]:
                    draw_marker_square_solid_outline(mx, mz, MARKER_SQUARE_COLOR)

        # circles (hollow outlines)
        for zone in (2, 4, 6, 8):
            for (mx, mz, row) in markpoints[zone]:
                draw_circle_outline(mx, mz, CIRCLE_RADIUS_CM, MARKER_STROKE_CM, CIRCLE_SEGMENTS, CIRCLE_RGB)

        if DEBUG_DRAW_MARKERS:
            # small debug cubes at markpoints
            size = 0.5
            y_bottom = FLOOR_TOP_Y + 0.01
            y_top = y_bottom + 1.0
            for zone, pts in markpoints.items():
                for (x, z, row) in pts:
                    draw_box(x - size, y_bottom, z - size, x + size, y_top, z + size, COLOR_MARKER_DEBUG)

        # zone grid lines (stippled)
        for x in (-50.0, 50.0):
            draw_stippled_axis_aligned(x, -MAT_HALF, x, -INNER_HALF, MARKER_STROKE_CM, DASH_LEN_CM, GAP_LEN_CM, COLOR_GRID, marker_squares, circles, INNER_HALF - EPS)
            draw_stippled_axis_aligned(x, INNER_HALF, x, MAT_HALF, MARKER_STROKE_CM, DASH_LEN_CM, GAP_LEN_CM, COLOR_GRID, marker_squares, circles, INNER_HALF - EPS)

        for z in (-50.0, 50.0):
            draw_stippled_axis_aligned(-MAT_HALF, z, -INNER_HALF, z, MARKER_STROKE_CM, DASH_LEN_CM, GAP_LEN_CM, COLOR_GRID, marker_squares, circles, INNER_HALF - EPS)
            draw_stippled_axis_aligned(INNER_HALF, z, MAT_HALF, z, MARKER_STROKE_CM, DASH_LEN_CM, GAP_LEN_CM, COLOR_GRID, marker_squares, circles, INNER_HALF - EPS)

        # orthogonal connections between marker squares (stippled, includes A-B spans)
        for (ax, az, bx, bz) in connections:
            draw_stippled_axis_aligned(ax, az, bx, bz, MARKER_STROKE_CM, DASH_LEN_CM, GAP_LEN_CM, COLOR_GRID, marker_squares, circles, INNER_HALF - EPS)

        # edge extensions to exterior (stippled)
        for (ax, az, bx, bz) in edge_extensions:
            draw_stippled_axis_aligned(ax, az, bx, bz, MARKER_STROKE_CM, DASH_LEN_CM, GAP_LEN_CM, COLOR_GRID, marker_squares, circles, INNER_HALF - EPS)

        # interior stippled extensions toward inner wall faces (stippled)
        for (ax, az, bx, bz) in interior_stippled_extensions:
            draw_stippled_axis_aligned(ax, az, bx, bz, MARKER_STROKE_CM, DASH_LEN_CM, GAP_LEN_CM, COLOR_GRID, marker_squares, circles, INNER_HALF - EPS)

        # explicit middle column and middle row lines (stippled across full mat, clipped)
        draw_stippled_axis_aligned(0.0, -MAT_HALF, 0.0, MAT_HALF, MARKER_STROKE_CM, DASH_LEN_CM, GAP_LEN_CM, COLOR_GRID, marker_squares, circles, INNER_HALF - EPS)
        draw_stippled_axis_aligned(-MAT_HALF, 0.0, MAT_HALF, 0.0, MARKER_STROKE_CM, DASH_LEN_CM, GAP_LEN_CM, COLOR_GRID, marker_squares, circles, INNER_HALF - EPS)

        # Draw the robot as a single rigid object, including wheels, wireframe box, and arrow
        draw_enclosed_lowpoly_car(spawn_x, spawn_z, spawn_yaw_deg)

        pygame.display.flip()

if __name__ == "__main__":
    main()
