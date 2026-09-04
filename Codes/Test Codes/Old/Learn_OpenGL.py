# pygame imports
import pygame
from pygame.locals import *

#OpenGl imports
from OpenGL.GL import *
from OpenGL.GLU import *

import random

vertices = (
#   (x, y, z),
    (1, -1, -1), # node 1
    (1, 1, -1), # node 2
    (-1, 1, -1), # node 3
    (-1, -1, -1), # node 4
    (1, -1, 1), # node 5
    (1, 1, 1), # node 6
    (-1, -1, 1), # node 7
    (-1, 1, 1) # node 8
)

edges = (
# 3 connections per node
    (0, 1), # edge 1
    (0, 3), # edge 2
    (0, 4), # edge 3
    (2, 1), # edge 4
    (2, 3), # edge 5
    (2, 7), # edge 6
    (6, 3), # edge 7
    (6, 4), # edge 8
    (6, 7), # edge 9
    (5, 1), # edge 10
    (5, 4), # edge 11
    (5, 7) # edge 12
)

surfaces = (
    (0, 1, 2, 3), # surface 1
    (3, 2, 7, 6), # surface 2
    (6, 7, 5, 4), # surface 3
    (4, 5, 1, 0), # surface 4
    (1, 5, 7, 2), # surface 5
    (4, 0, 3, 6) # surface 6
)

colors = (
#   (R, G, B) color
    (1, 0, 0),
    (0, 1, 0),
    (0, 0, 1),
    (0, 1, 0),
    (1, 1, 1),
    (0, 1, 1),
    (1, 0, 0),
    (0, 1, 0),
    (0, 0, 1),
    (0, 0, 0),
    (1, 1, 1),
    (0, 1, 1),
)

#ground_vertices = (
#    (-10, -1.1, 20),
#    (10, -1.1, 20),
#    (-10, -1.1, -300),
#    (10, -1.1, -300)
#)

#def ground():
#    glBegin(GL_QUADS)
#    for vertex in ground_vertices:
#        glColor3fv((0, 0.5, 0.5))
#        glVertex3fv(vertex)
#         
#    glEnd()

def set_vertices(max_distance, min_distance = -20, camera_x = 0, camera_y = 0):

    camera_x = -1*int(camera_x)
    camera_y = -1*int(camera_y)
    
    x_value_change = random.randrange(camera_x - 75, camera_x + 75)
    y_value_change = random.randrange(camera_y - 75, camera_y + 75)
    z_value_change = random.randrange(-1*max_distance, min_distance)
    
    new_vertices = []
    
    for vert in vertices:
        new_vert = []
        
        new_x = vert[0] + x_value_change
        new_y = vert[1] + y_value_change
        new_z = vert[2] + z_value_change
        
        new_vert.append(new_x)
        new_vert.append(new_y)
        new_vert.append(new_z)
        
        new_vertices.append(new_vert)
    
    return new_vertices       
    

def Cube(vertices):
    glBegin (GL_QUADS)
    for surface in surfaces:
        x = 0
        
        for vertex in surface:
            x += 1
            glColor3fv(colors[x]) 
            glVertex3fv(vertices[vertex])
        
    glEnd()
    
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex]) # specifies a vertex
    glEnd()
    
def main():
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)

    max_distance = 100    
    gluPerspective(45, (display[0]/display[1]), 0.1, max_distance)
    glTranslatef(0, 0, -40)
    
    # object_passed = False
    
    x_move = 0
    y_move = 0
    
    cur_x = 0
    cur_y = 0
    
    game_speed = 2
    direction_speed = 2
    
    cube_dict = {}
    
    for x in range(75):
        cube_dict[x] = set_vertices(max_distance)
        
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
                
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    x_move = direction_speed
                if event.key == pygame.K_RIGHT:
                    x_move = -direction_speed
                    
                if event.key == pygame.K_UP:
                    y_move = -direction_speed  
                if event.key == pygame.K_DOWN:
                    y_move = direction_speed

            if event.type == pygame.KEYUP:
                if event.key == pygame.K_LEFT or event.key == pygame.K_RIGHT:
                    x_move = 0
                if event.key == pygame.K_UP or event.key == pygame.K_DOWN:
                    y_move = 0

                    
                                    
#           if event.type == pygame.MOUSEBUTTONDOWN:
#               if event.button == 4:
#                    glTranslatef(0.0, 0.0, 0.5)
#               if event.button == 5:
#                   glTranslatef(0.0, 0.0, -0.5)
                
        # glRotatef(1, 3, 1, 1)
        
        x = glGetDoublev(GL_MODELVIEW_MATRIX)
        
        camera_x = x[3][0]
        camera_y = x[3][1]
        camera_z = x[3][2]
        
        cur_x += x_move
        cur_y += y_move
            
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        glTranslatef(x_move, y_move, game_speed)
        
        
#        ground()
        
        for each_cube in cube_dict:
            Cube(cube_dict[each_cube])

        for each_cube in cube_dict:
            if camera_z <= cube_dict[each_cube][0][2]:
                new_max = int(-1*(camera_z - (max_distance * 2)))
                cube_dict[each_cube] = set_vertices(new_max, int(camera_z - max_distance), cur_x, cur_y)
                    
                    
        pygame.display.flip()
        # pygame.time.wait(10)
main()  