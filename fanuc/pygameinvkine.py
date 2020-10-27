import sys, pygame
import time
from pygame.math import Vector2
import math
import numpy as np

# Resources: http://archive.petercollingridge.co.uk/book/export/html/460
# http://www.stat.cmu.edu/~ryantibs/convexopt-F18/lectures/coord-desc.pdf
# https://link.springer.com/article/10.1186/1471-2105-6-159
# https://www.youtube.com/watch?v=MvuO9ZHGr6k
# http://www.virtualpuppetry.com/inverse_kinematics_ccd/paper.pdf
# https://www.sciencedirect.com/topics/engineering/inverse-kinematics-problem


# Simple Game of joints following a ball.
# Extensions: Wherever the mouse is, 3D 
# pygame.init()

# size = width, height = 640, 480
# screen = pygame.display.set_mode(size)

# points = list(map(Vector2, [(100, 100), (200, 100), (300, 100), (400, 100), (500, 100)]))
# target = Vector2(450, 300)
# target_speed = Vector2(3, 3)

# rel_points = []
# angles = []

# max_angle = 90 # Adjust for limited angles

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


class Node:
    def __init__(self, coordinates):
        self.x = coordinates[0]
        self.y = coordinates[1]
        self.z = coordinates[2]
        
class Edge:
    def __init__(self, start, stop):
        self.start = start
        self.stop  = stop

class Wireframe():
    def __init__(self, *args):
        if len(args) > 0:
            self.nodes = args[0]
            self.edges = args[1]
        else: 
            self.nodes = np.zeros((0, 4))
            self.edges = []   

    def addNodes(self, node_array):
        ones_column = np.ones((len(node_array), 1))
        ones_added = np.hstack((node_array, ones_column))
        self.nodes = np.vstack((self.nodes, ones_added))

    def addEdges(self, edgeList):
        self.edges += edgeList

    def outputNodes(self):
        print("\n --- Nodes --- ")
        for i, (x, y, z, _) in enumerate(self.nodes):
            print("   %d: (%d, %d, %d)" % (i, x, y, z))
            
    def outputEdges(self):
        print("\n --- Edges --- ")
        for i, (node1, node2) in enumerate(self.edges):
            print("   %d: %d -> %d" % (i, node1, node2))

    def translate(self, axis, d):
        """ Add constant 'd' to the coordinate 'axis' of each node of a wireframe """
            
        if axis in ['x', 'y', 'z']:
            for node in self.nodes:
                setattr(node, axis, getattr(node, axis) + d)

    def scale(self, centre_x, centre_y, scale):
        """ Scale the wireframe from the centre of the screen """

        for node in self.nodes:
            node.x = centre_x + scale * (node.x - centre_x)
            node.y = centre_y + scale * (node.y - centre_y)
            node.z *= scale

    def findCentre(self):
        """ Find the centre of the wireframe. """

        num_nodes = len(self.nodes)
        meanX = np.sum(self.nodes[:,0]) / num_nodes
        meanY = np.sum(self.nodes[:,1]) / num_nodes
        meanZ = np.sum(self.nodes[:,2]) / num_nodes

        return (meanX, meanY, meanZ)

    # def rotateByPoint(self, x,y,z, radians):


    # def rotatePointToAngle(self, x,y,z, radians):


    def rotateZ(self, cx,cy,cz, radians):        
        # for node in self.nodes:
        #     x      = node.x - cx
        #     y      = node.y - cy
        #     d      = math.hypot(y, x)
        #     theta  = math.atan2(y, x) + radians
        #     node.x = cx + d * math.cos(theta)
        #     node.y = cy + d * math.sin(theta)

        x = self.nodes[:,0] - cx
        y = self.nodes[:,1] - cy
        d = np.hypot(y,x)
        theta = np.arctan2(y,x) + radians
        self.nodes[:,0] = cx + d * np.cos(theta)
        self.nodes[:,1] = cy + d * np.sin(theta)

    def rotateX(self, cx,cy,cz, radians):
        # for node in self.nodes:
        #     y      = node.y - cy
        #     z      = node.z - cz
        #     d      = math.hypot(y, z)
        #     theta  = math.atan2(y, z) + radians
        #     node.z = cz + d * math.cos(theta)
        #     node.y = cy + d * math.sin(theta)

        y = self.nodes[:,1] - cy
        z = self.nodes[:,2] - cz
        d = np.hypot(y,z)
        theta = np.arctan2(y, z) + radians
        self.nodes[:,2] = cz + d * np.cos(theta)
        self.nodes[:,1] = cy + d * np.sin(theta)

    def rotateY(self, cx,cy,cz, radians):
        # for node in self.nodes:
        #     x      = node.x - cx
        #     z      = node.z - cz
        #     d      = math.hypot(x, z)
        #     theta  = math.atan2(x, z) + radians
        #     node.z = cz + d * math.cos(theta)
        #     node.x = cx + d * math.sin(theta)

        x = self.nodes[:,0] - cx
        z = self.nodes[:,2] - cz
        d = np.hypot(x,z)
        theta = np.arctan2(x,z) + radians
        self.nodes[:,2] = cz + d * np.cos(theta)
        self.nodes[:,0] = cx + d * np.sin(theta)

    def transform(self, matrix):
        """ Apply a transformation defined by a given matrix. """

        self.nodes = np.dot(self.nodes, matrix)

    def translationMatrix(self, dx=0, dy=0, dz=0):
        """ Return matrix for translation along vector (dx, dy, dz). """
        
        return np.array([[1,0,0,0],
                         [0,1,0,0],
                         [0,0,1,0],
                         [dx,dy,dz,1]])


class ProjectionViewer:
    """ Displays 3D objects on a Pygame screen """

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption('Wireframe Display')
        self.background = (10,10,50)

        self.wireframes = {}
        self.displayNodes = True
        self.displayEdges = True
        self.nodeColour = (255,255,255)
        self.edgeColour = (200,200,200)
        self.nodeRadius = 4

        self.key_to_function = {
            pygame.K_LEFT: (lambda x: x.translateAll([-10, 0, 0])),
            pygame.K_RIGHT:(lambda x: x.translateAll([ 10, 0, 0])),
            pygame.K_DOWN: (lambda x: x.translateAll([0,  10, 0])),
            pygame.K_UP:   (lambda x: x.translateAll([0, -10, 0])),
            pygame.K_EQUALS: (lambda x: x.scaleAll(1.25)),
            pygame.K_MINUS:  (lambda x: x.scaleAll( 0.8)),
            pygame.K_q: (lambda x: x.rotateAll('X',  0.3925)),
            pygame.K_w: (lambda x: x.rotateAll('X', -0.3925)),
            pygame.K_a: (lambda x: x.rotateAll('Y',  0.3925)),
            pygame.K_s: (lambda x: x.rotateAll('Y', -0.3925)),
            pygame.K_z: (lambda x: x.rotateAll('Z',  0.3925)),
            pygame.K_x: (lambda x: x.rotateAll('Z',  -0.3925))}

    def run(self):
        running = True;
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key in self.key_to_function:
                        self.key_to_function[event.key](self)
                    
            self.display()  
            pygame.display.flip()

    def addWireframe(self, name, wireframe):
        """ Add a named wireframe object. """
        self.wireframes[name] = wireframe

    def display(self):
        """ Draw the wireframes on the screen. """

        self.screen.fill(self.background)

        for wireframe in self.wireframes.values():
            if self.displayEdges:
                for n1, n2 in wireframe.edges:
                    pygame.draw.aaline(self.screen, self.edgeColour, wireframe.nodes[n1][:2], wireframe.nodes[n2][:2], 1)

            if self.displayNodes:
                for node in wireframe.nodes:
                    pygame.draw.circle(self.screen, self.nodeColour, (int(node[0]), int(node[1])), self.nodeRadius, 0)


    def translateAll(self, axis, d):
        """ Translate all wireframes along a given axis by d units. """

        for wireframe in self.wireframes.values():
            wireframe.translate(axis, d)

    def translateAll(self, vector):
        """ Translate all wireframes along a given axis by d units. """

        matrix = self.translationMatrix(*vector)
        for wireframe in self.wireframes.values():
            wireframe.transform(matrix)

    def scaleAll(self, scale):
        """ Scale all wireframes by a given scale, centred on the centre of the screen. """

        centre_x = self.width/2
        centre_y = self.height/2

        matrix = self.scaleMatrix(sx=scale, sy=scale, sz=scale)

        for wireframe in self.wireframes.values():
            wireframe.transform(matrix)

    def rotateAll(self, axis, theta):
        """ Rotate all wireframe about their centre, along a given axis by a given angle. """

        rotateFunction = 'rotate' + axis

        def maptofunc(func, axis, theta):
            func(*axis,theta)

        for wireframe in self.wireframes.values():

            mapping = {'rotateX': wireframe.rotateX, 'rotateY': wireframe.rotateY, 'rotateZ': wireframe.rotateZ }
            centre = wireframe.findCentre()
            maptofunc(mapping[rotateFunction], centre, theta)

    # def rotateAll(self, axis, theta): 
    #     if axis == 'X':
    #         matrix = rotateXMatrix(theta)
    #     elif axis == "Y":
    #         matrix = rotateYMatrix(theta)
    #     else: 
    #         matrix = rotateZMatrix(theta)

    #     for wireframe in self.wireframes.values():
    #         centre = wireframe.findCentre()


    def scaleMatrix(self, sx=0, sy=0, sz=0):
        """ Return matrix for scaling equally along all axes centred on the point (cx,cy,cz). """
        
        return np.array([[sx, 0,  0,  0],
                         [0,  sy, 0,  0],
                         [0,  0,  sz, 0],
                         [0,  0,  0,  1]])

    def rotateXMatrix(radians):
        """ Return matrix for rotating about the x-axis by 'radians' radians """
        
        c = np.cos(radians)
        s = np.sin(radians)
        return np.array([[1, 0, 0, 0],
                         [0, c,-s, 0],
                         [0, s, c, 0],
                         [0, 0, 0, 1]])

    def rotateYMatrix(radians):
        """ Return matrix for rotating about the y-axis by 'radians' radians """
        
        c = np.cos(radians)
        s = np.sin(radians)
        return np.array([[ c, 0, s, 0],
                         [ 0, 1, 0, 0],
                         [-s, 0, c, 0],
                         [ 0, 0, 0, 1]])

    def rotateZMatrix(radians):
        """ Return matrix for rotating about the z-axis by 'radians' radians """
        
        c = np.cos(radians)
        s = np.sin(radians)
        return np.array([[c,-s, 0, 0],
                         [s, c, 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])

    def translationMatrix(self, dx=0, dy=0, dz=0):
        """ Return matrix for translation along vector (dx, dy, dz). """
        
        return np.array([[1,0,0,0],
                         [0,1,0,0],
                         [0,0,1,0],
                         [dx,dy,dz,1]])


### TODO: ADD A GRID AND AXES ? 



# cube = Wireframe()
# cube_nodes = [(x,y,z) for x in (20,200) for y in (20,200) for z in (20,200)]
# cube.addNodes(np.array(cube_nodes))
    
# cube.addEdges([(n,n+4) for n in range(0,4)])
# cube.addEdges([(n,n+1) for n in range(0,8,2)])
# cube.addEdges([(n,n+2) for n in (0,1,4,5)])
    
# cube.outputNodes()
# cube.outputEdges()

# pv = ProjectionViewer(400, 300)
# pv.addWireframe('cube', cube)
# pv.run()


