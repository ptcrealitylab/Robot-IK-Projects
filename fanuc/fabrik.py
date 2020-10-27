# https://www.sciencedirect.com/science/article/pii/S1524070311000178?via%3Dihub

# Input: The joint positions pi for i = 1, … , n, the target
#  position t and the distances between each joint
#  di = ∣pi+1 − pi∣ for i = 1,…,n − 1
# Output: The new joint positions pi for i = 1, … , n.
# 	% The distance between root and target
# 	dist = ∣p1 − t∣
# 	% Check whether the target is within reach
# 	if dist > d1 + d2 +…+ dn−1 then
# 	 % The target is unreachable
# 	 for i = 1, … , n − 1 do
# 	 % Find the distance ri between the target t and the joint
#  position pi
# 	 ri = ∣t − pi∣
# 	 λi = di/ri
#	 % Find the new joint positions pi.
#	 pi+1 = (1 − λi) pi + λit
#	 end
#	else
#	 % The target is reachable; thus, set as b the initial position of the
#  joint p1
#	 b = p1
#	 % Check whether the distance between the end effector pn
#  and the target t is greater than a tolerance.
#	 difA = ∣pn − t∣
#	 while difA > tol do
#	 % STAGE 1: FORWARD REACHING
#	 % Set the end effector pn as target t
#	 pn = t
#	 for i = n − 1, … , 1 do
#	  % Find the distance ri between the new joint position
#   pi+1 and the joint pi
#	  ri = ∣pi+1 − pi∣
#	  λi = di/ri
#	  % Find the new joint positions pi.
#	  pi = (1 − λi) pi+1 + λipi
#	 end
#	 % STAGE 2: BACKWARD REACHING
#	 % Set the root p1 its initial position.
#	 p1 = b
#	 for i = 1,…,n − 1 do
#	  % Find the distance ri between the new joint position pi
#   and the joint pi+1
#	  ri = ∣pi+1 − pi∣
#	  λi = di/ri
#	  % Find the new joint positions pi.
#	  pi+1 = (1 − λi)pi + λipi+1
#	 end
#	 difA = ∣pn − t∣
#	 end
#	end

import numpy as np

def distanceBetween(j1, j2): 
	return abs(np.linalg.norm(j1-j2))

# class MaxDistanceException(Exception):
# 	pass

class SimpleArm:
	def __init__(self):
		self.nodes = np.zeros((0, 4))
		self.edges = []

	def __init__(self, nodes, edges):
		self.edges = []
		if (nodes.shape[1] == 3):
			self.nodes = np.zeros((0,4))
			self.addNodes(nodes)
		else:
			self.nodes = nodes
		self.addEdges(edges)
		self.distances = [distanceBetween(self.nodes[i,:], self.nodes[i+1,:]) 
							for i in range(self.nodes.shape[0] - 1)]

	def checkDistances(self, new_dists):
		for a, b in zip(self.distances, new_dists):
			rounded_a = float("{:.7f}".format(a))
			rounded_b = float("{:.7f}".format(b))
			if rounded_a != rounded_b:
				return False
		return True

	def updateDistances(self):
		temp = [distanceBetween(self.nodes[i,:], self.nodes[i+1,:]) 
							for i in range(self.nodes.shape[0] - 1)]
		if (not self.checkDistances(temp)):
			print("WARNING: DISTANCES CHANGED")

		self.distances = temp

	def addNodes(self, node_array):
		ones_column = np.ones((len(node_array), 1))
		ones_added = np.hstack((node_array, ones_column))
		self.nodes = np.vstack((self.nodes, ones_added))

	def addEdges(self, edgeList):
		self.edges += edgeList

	def outputNodes(self):
		print("\n --- Nodes --- ")
		for i, (x, y, z, _) in enumerate(self.nodes):
			print("   %d: (%s, %s, %s)" % (i, x, y, z))
			
	def outputEdges(self):
		print("\n --- Edges --- ")
		for i, (node1, node2) in enumerate(self.edges):
			print("   %d: %d -> %d" % (i, node1, node2))

	def outputDistances(self):
		self.updateDistances()
		print("\n --- Distances --- ")
		for i, dist in enumerate(self.distances):
			print("   %d: %s" % (i, dist))

	def max_distance(self):
		# print([self.nodes[i,:][:-1] for i in range(self.nodes.shape[0])])
		return np.sum(self.distances)

	def num_nodes(self):
		return self.nodes.shape[0]

	def num_links(self):
		return len(self.edges)

	def getNode(self, pos):
		return self.nodes[pos,:][:-1]

	def getEndEffector(self):
		return self.nodes[self.num_links(), :][:-1]

	def getDistanceOfJointLink(self, pos):
		return self.distances[pos]

	def setNode(self, pos, arr):
		arr = np.append(arr, [1])
		# print(arr)
		self.nodes[pos,:] = arr

	def createWireframe(self):
		return pik.Wireframe(self.nodes, self.edges)

	# def angleBetweenJoints(self, j1, j2):

# [1 0 0]
# [0 1 0]
# [0 0 1]

j1 = (0,0,0) 
j2 = (0, -116, 137.484)
j3 = (0, -116 ,777.484)
j4 = (256, 0, 969.484)
fanuc_nodes = [j1, j2, j3, j4]
fanuc_edges = [(0,1), (1,2), (2,3)]
s = SimpleArm(np.array(fanuc_nodes), fanuc_edges)
copy = SimpleArm(np.array(fanuc_nodes), fanuc_edges)
test = SimpleArm(np.array([(0,0,0),(1,1,1),(2,2,2)]), [(0,1), (1,2)])
# test.outputDistances()
s.outputDistances()
# print(s.num_links())

target = np.array((100,-200, 455))
# test.outputNodes()
# test.outputEdges()
# print(test.max_distance())
# s.outputNodes()
# s.outputEdges()
# print(s.max_distance())

# IMPLEMENTING https://www.sciencedirect.com/science/article/pii/S1524070311000178?via%3Dihub
# FABRIK ALGORITHM 1
# s  -> SimpleArm instance
# t  -> target position
# di -> joint distances
# tolerance -> error for joint reaching position
# count      -> failsafe to turn off algorithm after count iterations
# TODO: AFTER FABRIK, DO WE NEED MOTOR VELOCITIES FUNCTION? 
def FABRIK(s, t, tolerance, count): 
	# base node
	root = s.getNode(0)
	# distance between root and target
	dist = distanceBetween(root, t)
	# check if target is within reach
	iterations = []
	if (dist >= s.max_distance()):
		# print("CAN'T MOVE TO THAT POSITION")
		for i in range(s.num_links()):
			# find the distance ri between target and pi 
			curr_node = s.getNode(i)
			next_node = s.getNode(i+1)
			ri = distanceBetween(curr_node, target) 
			# λi = di/ri
			lambda_i = s.getDistanceOfJointLink(i)/ri
			# find new joint positions: pi+1 = (1 − λi) pi + λit
			new_node = ((1 - lambda_i) * curr_node) + (lambda_i * t)
			s.setNode(i+1, new_node)
		# raise MaxDistanceException
		return [s.nodes]
	else: 
		# print("CAN MOVE HERE")
		# set as b the initial position of the joint p1
		b = root
		ee = s.getEndEffector()
		# Check whether the distance between the end effector pn and the target t is greater than a tolerance.
		dif_a = distanceBetween(ee, t)
		ctr = 0
		while dif_a > tolerance and ctr < count:
			# FORWARD REACHING 
			# Set the end effector pn as target t
			s.setNode(s.num_links(), t)
			for i in range(s.num_links() - 1, 0, -1):
				# Find the distance ri between the new joint position pi+1 and the joint pi
				curr_node = s.getNode(i)
				next_node = s.getNode(i+1)
				ri = distanceBetween(curr_node, next_node)
				lambda_i = s.getDistanceOfJointLink(i)/ri
				# pi = (1 − λi) pi+1 + λipi
				new_node = ((1 - lambda_i) * next_node) + (lambda_i * curr_node)
				s.setNode(i, new_node)
				# iterations.append(s.nodes)
			# BACKWARD REACHING 
			s.setNode(0, b)
			for i in range(s.num_links()):
				#  Find the distance ri between the new joint position pi and the joint pi+1
				curr_node = s.getNode(i)
				next_node = s.getNode(i+1)
				ri = distanceBetween(curr_node, next_node)
				lambda_i = s.getDistanceOfJointLink(i)/ri
				# pi+1 = (1 − λi)pi + λipi+1
				new_node = ((1 - lambda_i) * curr_node) + (lambda_i * next_node)
				s.setNode(i + 1, new_node)
				# iterations.append(s.nodes)
			dif_a = distanceBetween(s.getEndEffector(), t)
			iterations.append(s.nodes)
			ctr += 1
	return iterations

iters = FABRIK(s, target, 1, 1000)
print(len(iters))
s.outputDistances()

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax  = fig.add_subplot(121, projection='3d')
ax2 = fig.add_subplot(122, projection='3d')

x1 = copy.nodes[:,0]
y1 = copy.nodes[:,1]
z1 = copy.nodes[:,2]
ax.scatter3D(x1, y1, z1 )
ax.scatter3D(target[0], target[1], target[2], 'red')
ax.plot3D(x1, y1, z1, 'gray')

x2 = s.nodes[:,0]
y2 = s.nodes[:,1]
z2 = s.nodes[:,2]
ax2.scatter3D(x2, y2, z2)
ax2.scatter3D(target[0], target[1], target[2], 'red')
ax2.plot3D(x2, y2, z2, 'gray')

plt.show()
# print(iters)
# s.outputNodes()
# s.outputDistances()
# FABRIK(test, target, 1, 1000)
# test.outputNodes()
# test.outputDistances()

# Assume we have a ball-and-socket joint with orientational limits described 
# by the rotor R and rotational limits described by the angles θ1,…,θ4.

# https://www.mecademic.com/resources/Euler-angles/Euler-angles
# In robotics, FANUC and KUKA use the fixed XYZ Euler angle convention,
# meanwhile mecademic uses mobile XYZ Euler angle 

