import numpy as np
import math
import pygameinvkine as pik 
import pygame

# XY CROSS-SECTION



# XZ CROSS-SECTION 



# YZ CROSS-SECTION


### X, Y, Z (mm)

### Consider J1 to be origin, XY PLANE ROTATION

### J1 ANGLES: -170 - 170 deg


j1_plane = "xy"

j1_degrees = [-170, 170]
j1_rads    = [math.radians(-170), math.radians(170)]

### J2 ANGLES: 120 - -60 deg XZ PLANE ROTATION

j2_plane = "xz"

j2_degrees = [-60, 120]
j2_rads    = [math.radians(-60), math.radians(120)]

### J3 ANGLES XZ PLANE ROTATION: 

j3_plane = "xz"

j3_degrees = [-132.5, 180]
j3_rads    = [math.radians(-132.5), math.radians(180)]

# DISREGARD J4-J6 FOR NOW



# Simple Game of joints following a ball.
# Extensions: Wherever the mouse is, 3D 
# pygame.init()

# size = width, height = 1280, 960
# screen = pygame.display.set_mode(size)

# # points = list(map(Vector2, [(100, 100), (200, 100), (300, 100), (400, 100), (500, 100)]))
# # target = Vector2(450, 300)
# # target_speed = Vector2(3, 3)

# rel_points = []
# angles = []

# # max_angle = 360 # Adjust for limited angles

# for i in range(1, len(points)):
#     rel_points.append(points[i] - points[i-1])
#     angles.append(0)

# def solve_ik(i, endpoint, target):
#     if i < len(points) - 2:
#         endpoint = solve_ik(i+1, endpoint, target)
#     current_point = points[i]

#     angle = (endpoint-current_point).angle_to(target-current_point)
#     angles[i] += min(max(-3, angle), 3)
#     angles[i] = min(max(180-max_angle, (angles[i]+180)%360), 180+max_angle)-180

#     return current_point + (endpoint-current_point).rotate(angle)

# def render():
#     black = 0, 0, 0
#     white = 255, 255, 255

#     screen.fill(white)
#     for i in range(1, len(points)):
#         prev = points[i-1]
#         cur = points[i]
#         pygame.draw.aaline(screen, black, prev, cur)
#     for point in points:
#         pygame.draw.circle(screen, black, (int(point[0]), int(point[1])), 5)
#     pygame.draw.circle(screen, black, (int(target[0]), int(target[1])), 10)
#     pygame.display.flip()

# while 1:
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT: sys.exit()

#     solve_ik(0, points[-1], target)
#     angle = 0
#     for i in range(1, len(points)):
#         angle += angles[i-1]
#         points[i] = points[i-1] + rel_points[i-1].rotate(angle)

#     target += target_speed
#     if target.x <= 0 or target.x >= width:
#         target_speed.x = -target_speed.x
#     if target.y <= 0 or target.y >= height:
#         target_speed.y = -target_speed.y

#     render()

#     pygame.time.wait(int(1000/60))

## MAYBE MAKE AN IMMOVABLE JOINT CLASS? 
## WRAP IT AS AN EDGE CLASS ? 

simple_fanuc = pik.Wireframe()
simple_ur3e  = pik.Wireframe()

### STARTUP POSITIONS

### INVARIANT --> J1 to J2 connection must stay rigid
### 			  J2 CAN ONLY TRAVEL IN A CIRCLE AROUND J1
j1 = (0,0,0)
j2 = (0, -116, 137.484)
j3 = (0, -116 ,777.484)
j4 = (256, 0, 969.484)

### UR3 positions, each motor can move freely 360 degrees,
### but obviously can still make impossible positions by colliding
### 
u1 = (0,0,94.95)
u2 = (-62.43, 0, 151.25)
u3 = (-73.03, 0, 394.8)
u4 = (-70.38, 0, 608)
u5 = (-131.33, 0, 651.11)
u6 = (-131.33, -43.1, 693.35)

### FROM THESE STARTING POSITIONS, WE CAN TRACK THE CURRENT
### ANGLE OF THE MOTOR
### LOCKOUTS AND PLANES OF MOTION

# (xy, xz, yz)

j1_motion = (j1_rads, 0, 0)
j2_motion = (0, j2_rads, 0)
j3_motion = (0, j3_rads, 0)

fanuc_angles = [0, 0, 0] # rads

fanuc_nodes = [j1, j2, j3, j4]
ur3e_nodes  = [u1,u2,u3,u4,u5,u6]

dis_param = 500 # add this value to shift position of wireframe
fanuc_nodes = [(x+dis_param, y+dis_param, z+dis_param) for x,y,z in fanuc_nodes]
ur3e_nodes = [(x+dis_param,y+dis_param,z+dis_param) for x,y,z in ur3e_nodes]
# print(fanuc_nodes)
simple_fanuc.addNodes(np.array(fanuc_nodes))
simple_ur3e.addNodes(np.array(ur3e_nodes))
    
simple_fanuc.addEdges([(0,1), (1,2), (2,3)])
simple_ur3e.addEdges([(0,1), (1,2), (2,3), (3,4), (4,5)])
    
# simple_fanuc.outputNodes()
# simple_fanuc.outputEdges()

pv = pik.ProjectionViewer(1200, 800)
# pv.addWireframe('fanuc', simple_fanuc)
pv.addWireframe('ur3e', simple_ur3e)
pv.run()
