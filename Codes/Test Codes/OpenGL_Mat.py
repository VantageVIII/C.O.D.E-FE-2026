import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from math import sin, cos, radians, tan

# White mat (300x300, centered)
mat_vertices = (
    (-150, -1.1, 150),
    (150, -1.1, 150),
    (150, -1.1, -150),
    (-150, -1.1, -150)
)

def ground():
    # Draw white mat
    glColor3f(1, 1, 1)
    glBegin(GL_QUADS)
    for v in mat_vertices:
        glVertex3fv(v)
    glEnd()

def border():
    glColor3f(0, 0, 0.5)  # dark blue

    glBegin(GL_QUADS)
    # Top strip (extended to walls)
    glVertex3f(-160, -1.1, 150)
    glVertex3f(160, -1.1, 150)
    glVertex3f(160, -1.1, 160)
    glVertex3f(-160, -1.1, 160)

    # Bottom strip (extended to walls)
    glVertex3f(-160, -1.1, -150)
    glVertex3f(160, -1.1, -150)
    glVertex3f(160, -1.1, -160)
    glVertex3f(-160, -1.1, -160)

    # Left strip (extended to walls)
    glVertex3f(-150, -1.1, -160)
    glVertex3f(-150, -1.1, 160)
    glVertex3f(-160, -1.1, 160)
    glVertex3f(-160, -1.1, -160)

    # Right strip (extended to walls)
    glVertex3f(150, -1.1, -160)
    glVertex3f(150, -1.1, 160)
    glVertex3f(160, -1.1, 160)
    glVertex3f(160, -1.1, -160)
    glEnd()

def inner_square():
    glColor3f(0, 0, 0.5)  # dark blue
    glBegin(GL_QUADS)
    glVertex3f(-40, -1.1, 40)
    glVertex3f(40, -1.1, 40)
    glVertex3f(40, -1.1, -40)
    glVertex3f(-40, -1.1, -40)
    glEnd()

def walls():
    glColor3f(0, 0, 0)  # black
    glBegin(GL_QUADS)
    # Front wall
    glVertex3f(-160, -1.1, 160)
    glVertex3f(160, -1.1, 160)
    glVertex3f(160, 8.9, 160)
    glVertex3f(-160, 8.9, 160)

    # Back wall
    glVertex3f(-160, -1.1, -160)
    glVertex3f(160, -1.1, -160)
    glVertex3f(160, 8.9, -160)
    glVertex3f(-160, 8.9, -160)

    # Left wall
    glVertex3f(-160, -1.1, -160)
    glVertex3f(-160, -1.1, 160)
    glVertex3f(-160, 8.9, 160)
    glVertex3f(-160, 8.9, -160)

    # Right wall
    glVertex3f(160, -1.1, -160)
    glVertex3f(160, -1.1, 160)
    glVertex3f(160, 8.9, 160)
    glVertex3f(160, 8.9, -160)
    glEnd()

def calc_camera_distance(fov_deg, half_size):
    return half_size / tan(radians(fov_deg / 2))

def main():
    pygame.init()
    display = (1280, 720)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)

    max_distance = 1000
    fov = 45
    half_size = 160  # mat + border

    # Background color
    glClearColor(0.2, 0.2, 0.2, 1)  # dark grey

    # Camera state
    rot_x, rot_y = 0, 0
    pos_x = 0
    pos_z = -calc_camera_distance(fov, half_size)
    mouse_down = False

    clock = pygame.time.Clock()

    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                quit()

            if event.type == MOUSEBUTTONDOWN:
                if event.button == 1:
                    mouse_down = True
                if event.button == 4:  # scroll up
                    fov = max(10, fov - 2)
                    pos_z = -calc_camera_distance(fov, half_size)
                if event.button == 5:  # scroll down
                    fov = min(90, fov + 2)
                    pos_z = -calc_camera_distance(fov, half_size)
            if event.type == MOUSEBUTTONUP:
                if event.button == 1:
                    mouse_down = False
            if event.type == MOUSEMOTION and mouse_down:
                dx, dy = event.rel
                rot_y += dx * 0.2
                rot_x += dy * 0.2

        # Keyboard movement
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

        glTranslatef(pos_x, 0, pos_z)
        glRotatef(rot_x, 1, 0, 0)
        glRotatef(rot_y, 0, 1, 0)

        ground()
        border()
        inner_square()
        walls()

        pygame.display.flip()
        clock.tick(60)

main()
