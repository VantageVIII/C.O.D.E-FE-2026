# wro_fe_mat.py
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from math import sin, cos, radians, tan

# ---------- Scale and geometry (1 unit = 1 cm) ----------
# Mat: 300 cm x 300 cm -> half-size 150
MAT_HALF = 150.0
BORDER_WIDTH = 10.0            # 10 cm border
OUTER_HALF = MAT_HALF + BORDER_WIDTH  # 160

INNER_HALF = 40.0              # inner square half-size (80 cm square)

# Vertical positions (you can shift these if you prefer different absolute Y)
FLOOR_TOP_Y = -1.0             # top surface of floor (arbitrary baseline)
FLOOR_THICKNESS = 0.1          # 0.1 cm thick downward
FLOOR_BOTTOM_Y = FLOOR_TOP_Y - FLOOR_THICKNESS

WALL_HEIGHT = 8.9              # wall top Y (as in your earlier code)
WALL_BOTTOM_Y = FLOOR_TOP_Y

WALL_THICKNESS = 1.0           # 1 cm thick

# Small epsilon to avoid z-fighting
EPS = 0.001

# Colors (RGB)
COLOR_FLOOR = (1.0, 1.0, 1.0)          # white mat
COLOR_BORDER = (0.0, 0.0, 0.5)         # dark blue border
COLOR_INNER_SQUARE = (0.0, 0.0, 0.5)   # dark blue inner square
COLOR_WALLS = (0.0, 0.0, 0.0)          # black walls

# ---------- Utility: draw axis-aligned box ----------
def draw_box(x_min, y_min, z_min, x_max, y_max, z_max, color):
    glColor3f(*color)
    glBegin(GL_QUADS)
    # Top face (y_max)
    glVertex3f(x_min, y_max, z_min)
    glVertex3f(x_max, y_max, z_min)
    glVertex3f(x_max, y_max, z_max)
    glVertex3f(x_min, y_max, z_max)
    # Bottom face (y_min)
    glVertex3f(x_min, y_min, z_min)
    glVertex3f(x_min, y_min, z_max)
    glVertex3f(x_max, y_min, z_max)
    glVertex3f(x_max, y_min, z_min)
    # Front face (+Z)
    glVertex3f(x_min, y_min, z_max)
    glVertex3f(x_max, y_min, z_max)
    glVertex3f(x_max, y_max, z_max)
    glVertex3f(x_min, y_max, z_max)
    # Back face (-Z)
    glVertex3f(x_min, y_min, z_min)
    glVertex3f(x_min, y_max, z_min)
    glVertex3f(x_max, y_max, z_min)
    glVertex3f(x_max, y_min, z_min)
    # Left face (-X)
    glVertex3f(x_min, y_min, z_min)
    glVertex3f(x_min, y_min, z_max)
    glVertex3f(x_min, y_max, z_max)
    glVertex3f(x_min, y_max, z_min)
    # Right face (+X)
    glVertex3f(x_max, y_min, z_min)
    glVertex3f(x_max, y_max, z_min)
    glVertex3f(x_max, y_max, z_max)
    glVertex3f(x_max, y_min, z_max)
    glEnd()

# ---------- Scene pieces ----------
def ground_with_hole():
    """
    Draw the mat as four boxes around the inner square hole to avoid overlap.
    Top strip (+Z), bottom strip (-Z), left strip (-X), right strip (+X).
    """
    # Top strip: z from INNER_HALF to MAT_HALF
    draw_box(-MAT_HALF, FLOOR_BOTTOM_Y, INNER_HALF,
             MAT_HALF, FLOOR_TOP_Y, MAT_HALF,
             COLOR_FLOOR)

    # Bottom strip: z from -MAT_HALF to -INNER_HALF
    draw_box(-MAT_HALF, FLOOR_BOTTOM_Y, -MAT_HALF,
             MAT_HALF, FLOOR_TOP_Y, -INNER_HALF,
             COLOR_FLOOR)

    # Left strip: x from -MAT_HALF to -INNER_HALF, z between -INNER_HALF and INNER_HALF
    draw_box(-MAT_HALF, FLOOR_BOTTOM_Y, -INNER_HALF,
             -INNER_HALF, FLOOR_TOP_Y, INNER_HALF,
             COLOR_FLOOR)

    # Right strip: x from INNER_HALF to MAT_HALF, z between -INNER_HALF and INNER_HALF
    draw_box(INNER_HALF, FLOOR_BOTTOM_Y, -INNER_HALF,
             MAT_HALF, FLOOR_TOP_Y, INNER_HALF,
             COLOR_FLOOR)

def border():
    # Border strips around the mat (same thickness as floor)
    # Top border (+Z)
    draw_box(-OUTER_HALF, FLOOR_BOTTOM_Y, MAT_HALF,
             OUTER_HALF, FLOOR_TOP_Y, OUTER_HALF,
             COLOR_BORDER)
    # Bottom border (-Z)
    draw_box(-OUTER_HALF, FLOOR_BOTTOM_Y, -OUTER_HALF,
             OUTER_HALF, FLOOR_TOP_Y, -MAT_HALF,
             COLOR_BORDER)
    # Left border (-X)
    draw_box(-OUTER_HALF, FLOOR_BOTTOM_Y, -OUTER_HALF,
             -MAT_HALF, FLOOR_TOP_Y, OUTER_HALF,
             COLOR_BORDER)
    # Right border (+X)
    draw_box(MAT_HALF, FLOOR_BOTTOM_Y, -OUTER_HALF,
             OUTER_HALF, FLOOR_TOP_Y, OUTER_HALF,
             COLOR_BORDER)

def inner_square_plate():
    # Slightly raised thin plate to avoid z-fighting
    half = INNER_HALF - EPS
    top_y = FLOOR_TOP_Y + EPS
    bottom_y = FLOOR_TOP_Y + EPS * 0.1
    draw_box(-half, bottom_y, -half,
             half, top_y, half,
             COLOR_INNER_SQUARE)

def inner_walls():
    # Inner walls: outer face at +/-INNER_HALF; inner face at +/- (INNER_HALF - WALL_THICKNESS)
    outer = INNER_HALF
    inner = INNER_HALF - WALL_THICKNESS

    # Front wall (+Z)
    draw_box(-outer, WALL_BOTTOM_Y, inner,
             outer, WALL_HEIGHT, outer,
             COLOR_WALLS)
    # Back wall (-Z)
    draw_box(-outer, WALL_BOTTOM_Y, -outer,
             outer, WALL_HEIGHT, -inner,
             COLOR_WALLS)
    # Left wall (-X)
    draw_box(-outer, WALL_BOTTOM_Y, -outer,
             -inner, WALL_HEIGHT, outer,
             COLOR_WALLS)
    # Right wall (+X)
    draw_box(inner, WALL_BOTTOM_Y, -outer,
             outer, WALL_HEIGHT, outer,
             COLOR_WALLS)

def outer_walls():
    # Outer walls: inner face at +/-OUTER_HALF; outer face at +/- (OUTER_HALF + WALL_THICKNESS)
    inner = OUTER_HALF
    outer = OUTER_HALF + WALL_THICKNESS

    # Front (+Z)
    draw_box(-outer, WALL_BOTTOM_Y, inner,
             outer, WALL_HEIGHT, outer,
             COLOR_WALLS)
    # Back (-Z)
    draw_box(-outer, WALL_BOTTOM_Y, -outer,
             outer, WALL_HEIGHT, -inner,
             COLOR_WALLS)
    # Left (-X)
    draw_box(-outer, WALL_BOTTOM_Y, -outer,
             -inner, WALL_HEIGHT, outer,
             COLOR_WALLS)
    # Right (+X)
    draw_box(inner, WALL_BOTTOM_Y, -outer,
             outer, WALL_HEIGHT, outer,
             COLOR_WALLS)

# ---------- Camera helper ----------
def calc_camera_distance(fov_deg, half_size):
    return half_size / tan(radians(fov_deg / 2))

# ---------- Main ----------
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

    rot_x, rot_y = 0.0, 0.0
    pos_x = 0.0
    pos_z = -calc_camera_distance(fov, half_size)
    mouse_down = False

    clock = pygame.time.Clock()

    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                return
            if event.type == MOUSEBUTTONDOWN:
                if event.button == 1:
                    mouse_down = True
                if event.button == 4:
                    fov = max(10, fov - 2)
                    pos_z = -calc_camera_distance(fov, half_size)
                if event.button == 5:
                    fov = min(90, fov + 2)
                    pos_z = -calc_camera_distance(fov, half_size)
            if event.type == MOUSEBUTTONUP:
                if event.button == 1:
                    mouse_down = False
            if event.type == MOUSEMOTION and mouse_down:
                dx, dy = event.rel
                rot_y += dx * 0.2
                rot_x += dy * 0.2

        keys = pygame.key.get_pressed()
        speed = 0.5
        angle_rad = radians(rot_y)
        if keys[K_w]:
            pos_z += speed * -cos(angle_rad)
            pos_x += speed * sin(angle_rad)
        if keys[K_s]:
            pos_z -= speed * -cos(angle_rad)
            pos_x -= speed * sin(angle_rad)
        if keys[K_a]:
            pos_z += speed * sin(angle_rad)
            pos_x += speed * cos(angle_rad)
        if keys[K_d]:
            pos_z -= speed * sin(angle_rad)
            pos_x -= speed * cos(angle_rad)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        gluPerspective(fov, (display[0] / display[1]), 0.1, max_distance)

        glTranslatef(pos_x, 0.0, pos_z)
        glRotatef(rot_x, 1.0, 0.0, 0.0)
        glRotatef(rot_y, 0.0, 1.0, 0.0)

        # Draw scene
        ground_with_hole()
        border()
        inner_square_plate()
        inner_walls()
        outer_walls()

        pygame.display.flip()
        clock.tick(60)

if __name__ == "__main__":
    main()
