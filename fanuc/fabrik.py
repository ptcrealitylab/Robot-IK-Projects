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

# WE WILL USE EULER ANGLES AS LIMITATIONS
# THEN CONVERT TO QUATERNION TO PREVENT 
# GIMBAL LOCK. 

# THIS MEANS one or two of the i,j,k axes 
# will be 

import numpy as np
from rotations import *

def quatsFromJointLimits(jlims):
	quats = []
	for min_max, axis in jlims:
		lower = min_max[0]
		upper = min_max[1]
		temp = []
		temp.append(axisAngleToQuat(axis, lower))
		temp.append(axisAngleToQuat(axis, upper))
		quats.append(temp)

	return quats

def distanceBetween(j1, j2): 
	return abs(np.linalg.norm(j1-j2))

# class MaxDistanceException(Exception):
# 	pass

class SimpleArm:
	# def __init__(self):
	# 	self.nodes = np.zeros((0, 4))
	# 	self.edges = []
	# 	self.j_const = []

	def __init__(self, *args):
		if (len(args) >= 2):
			nodes = args[0]
			edges = args[1]
			self.edges = []
			if (nodes.shape[1] == 3):
				self.nodes = np.zeros((0,4))
				self.addNodes(nodes)
			else:
				self.nodes = nodes
			self.addEdges(edges)
			self.distances = [distanceBetween(self.nodes[i,:], self.nodes[i+1,:]) 
								for i in range(self.nodes.shape[0] - 1)]
			self.jlims = []
		if (len(args) == 3):
			self.jlims = quatsFromJointLimits(args[2])
			self.axes  = [s[1] for s in args[2]]
			self.angle_lims = [s[0] for s in args[2]]
		self.rotations = [0 for i in range(len(edges))]
		self.eulerangles = [EulerAngles(0,0,0) for i in range(len(edges))]

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

	def copy(self):
		return SimpleArm(self.nodes, self.edges, self.jlims)

	def addNodes(self, node_array):
		ones_column = np.ones((len(node_array), 1))
		ones_added = np.hstack((node_array, ones_column))
		self.nodes = np.vstack((self.nodes, ones_added))

	def addEdges(self, edgeList):
		self.edges += edgeList

	def currRot(self):
		print("CURRENT ROTATIONS:")
		for i, rot in enumerate(self.rotations):
			print("\tJ%s: %s" % (i + 1, rot))
			# print("\tJ%s: %s" % (i + 1, self.eulerangles[i]))

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

	def setAngles(self, i, theta):
		self.rotations[i] = theta
		curr_axis = self.axes[i]
		index = curr_axis.index(1)

		self.eulerangles[i].setByIndex(index, radians(theta))

	def getAngle(self, i):
		return self.rotations[i]

	def getEulerAngles(self, i):
		return self.eulerangles[i]

	def getNode(self, pos):
		return self.nodes[pos,:][:-1]

	def getJLims(self, pos):
		return self.jlims[pos]

	def getAngleLims(self, pos):
		return self.angle_lims[pos]

	def getAxis(self, pos):
		return self.axes[pos]

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

	def constrainQuat(self, q, motor_index):
		euler = quatToEuler(q).toList()
		# if 180, set to 0 (due to floating point errors)
		for i, ang in enumerate(euler):
			if ang > 179 and ang < 181:
				euler[i] = 0

		# print("CURRENT INDEX: %s" % (motor_index))
		# print("EULER ANGLES: %s" % euler)
		axis = self.getAxis(motor_index)
		j_min, j_max = self.getAngleLims(motor_index)
		angle = self.getAngle(motor_index)
		focus_index = axis.index(1)
		focus_angle = euler[focus_index]
		# print("FOCUS ANGLE: %s" %(focus_angle))
		# print("AXIS:", axis)
		# print("JOINT LIMITATIONS: %s" % ([j_min, j_max]))
		# print("CURRENT ANGLE: %s" % (angle))

		if angle == j_min or angle == j_max:
			return Quaternion(0,0,0,0)
		elif focus_angle < angle + j_min:
			euler[focus_index] = j_min
			self.setAngles(motor_index, j_min)
			return eulerToQuat(EulerAngles(euler[2], euler[1], euler[0]))
		elif focus_angle > angle + j_max:
			euler[focus_index] = j_max
			self.setAngles(motor_index, j_max)
			return eulerToQuat(EulerAngles(euler[2], euler[1], euler[0]))
		else: 
			self.setAngles(motor_index, angle + focus_angle)
			return q

	def rotateJByQuat(self, index, q):
		last_index = self.num_links()
		if (index == last_index):
			# can't rotate last joint --> TCP 
			return 

		joint      = self.getNode(index)
		next_joint = self.getNode(index + 1)

		child_joints = [self.getNode(i) for i in range(index + 2, last_index + 1)]

		vec = next_joint - joint

		new_joint = rotatePByQuat(vec, q) + joint
		self.setNode(index + 1, new_joint)
		ctr = index + 2
		for extra in child_joints: 
			vec = extra - joint
			self.setNode(ctr, rotatePByQuat(vec, q) + joint)
			ctr += 1

	def isQuatWithinBounds(self, q_check, q_range):
		q_min = q_range[0]
		q_max = q_range[1]

		# Criteria for True
		# If 0's line up 
		# If each component stays within max and min bounds
		q_check_list = q_check.toList()
		q_min_list   = q_min.toList()
		q_max_list   = q_max.toList()

		for i, q_c in enumerate(q_check_list):
			q_min_val = q_min_list[i]
			q_max_val = q_max_list[i]
			if q_min_val != q_c and q_min_val == 0:
				return False
			if q_c <= q_min_val or q_c >= q_max_val:
				return False

		return True

	def checkBounds(self, j_to_rotate, motor_index, target):
		# collect motor coords and its motor range
		motor       = self.getNode(motor_index)
		motor_range = self.getJLims(motor_index)

		# change axis of rotation to motor's position
		curr_vec   = j_to_rotate - motor 
		target_vec = target      - motor

		quat = findQuatforVecs(curr_vec, target_vec)
		return self.isQuatWithinBounds(quat, motor_range)
		# if not self.isQuatWithinBounds(quat, motor_range):
			# place previous point in a position such that 
			# the target is within motor range

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

# ASSUME CREATION OF SIMPLEARM IS ROBOT'S ZERO POSITION
# CALCULATE MIN AND MAX QUATERNION ROTATIONS FOR EACH MOTOR
fanuc_motors = [[(-180, 180), (0,0,1)], [(-60, 120), (0,1,0)], [(-45, 45), (0,1,0)]]

s = SimpleArm(np.array(fanuc_nodes), fanuc_edges, fanuc_motors)
copy = SimpleArm(np.array(fanuc_nodes), fanuc_edges, fanuc_motors)
copy2 = SimpleArm(np.array(fanuc_nodes), fanuc_edges, fanuc_motors)
test = SimpleArm(np.array([(0,0,0),(1,1,1),(2,2,2)]), [(0,1), (1,2)])

target = np.array((100,-200, 455))
target2 = np.array((200, -400, 340))
target3 = np.array((-150, 200, 200))

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

	# s = s_a.copy()
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
			# NEW: GET ROTATION FROM PREVIOUS JOINT TO END-EFFECTOR 
			# AND SEE IF IT IS WITHIN BOUNDS OF PREVIOUS JOINT
			# s.checkBounds(ee, s.num_links() - 1, t)
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

# TRANSLATION OF: 
# https://github.com/zalo/zalo.github.io/blob/master/assets/js/IK/IKExample.js
def CCDIK(s, t, tolerance, steps):
	ctr = 0
	
	dist = distanceBetween(s.getEndEffector(), t)
	while (dist > tolerance and ctr < steps):
		for i in range(s.num_links() - 1, -1, -1):
			curr = s.getNode(i)
			ee   = s.getEndEffector()

			vec_to_ee = ee - curr
			vec_to_target = t - curr

			quat_rot = findQuatforVecs(vec_to_ee, vec_to_target)
			quat_inv = quat_rot.inverse()
			curr_axis = s.getAxis(i)

			# FIND THE ROTATION FROM THIS JOINT TO PARENT JOINT
			# AND CONSTRAIN BY JOINT AXIS TO EMULATE HINGE ROTATION
			parent_axis = rotatePByQuat(curr_axis, quat_inv)
			quat_axis = findQuatforVecs(curr_axis, parent_axis)
			new_quat = quat_rot.multiply(quat_axis)

			## CONSTRAIN THE ROTATION IF THE RESULTING ROTATION
			## IS GREATER THAN THE CURRENT EULER ANGLE POSITION
			## OF THE JOINT 
			## IF JOINT IS ALREADY AT MAX ROTATION, NO ROTATION
			## CAN BE MADE 
			new_q = s.constrainQuat(new_quat, i)
			# s.currRot()
			if not new_q.equals(Quaternion(0,0,0,0)):
				s.rotateJByQuat(i, new_quat)

		dist = distanceBetween(s.getEndEffector(), t)
		# print(dist)
		ctr += 1

	return ctr



# foreach joint in jointsTipToBase {
#   # Point the effector towards the goal
#   directionToEffector = effector.position - joint.position;
#   directionToGoal = goal.position - joint.position;
#   joint.rotateFromTo(directionToEffector, directionToGoal);
#
#   # Constrain to rotate about the axis
#   curHingeAxis = joint.rotation * joint.axis;
#   hingeAxis = joint.parent.rotation * joint.axis;
#   joint.rotateFromTo(curHingeAxis, hingeAxis);
#   joint.localRotation.clampEuler(joint.minLimit, joint.maxLimit);
# }

# iters = FABRIK(s, target, 1, 1000)
# print(len(iters))
s.outputDistances()
s1 = SimpleArm(np.array(fanuc_nodes), fanuc_edges, fanuc_motors)
steps  = CCDIK(s, target, 2, 1000)
steps2 = CCDIK(copy, target2, 2, 1000)
steps3 = CCDIK(copy2, target3, 2, 1000)
print("CCDIK TO TARGET POINT %s TOOK %s STEPS" % (target, steps))
# copy2.rotateJByQuat(1, Quaternion(6, 0, 1, 0))
s.outputDistances()
copy2.outputDistances()
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax4 = fig.add_subplot(141, projection='3d')
ax  = fig.add_subplot(142, projection='3d')
ax2 = fig.add_subplot(143, projection='3d')
ax3 = fig.add_subplot(144, projection='3d')

x_lims = [-200,200]
y_lims = [-200,200]
z_lims = [0, 1000]

ax.set_xlim(x_lims)
ax.set_ylim(y_lims)
ax.set_zlim(z_lims)

ax2.set_xlim(x_lims)
ax2.set_ylim(y_lims)
ax2.set_zlim(z_lims)

ax3.set_xlim(x_lims)
ax3.set_ylim(y_lims)
ax3.set_zlim(z_lims)

ax4.set_xlim(x_lims)
ax4.set_ylim(y_lims)
ax4.set_zlim(z_lims)

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
ax2.scatter3D(target2[0], target2[1], target2[2], 'red')
ax2.plot3D(x2, y2, z2, 'gray')

x3 = copy2.nodes[:,0]
y3 = copy2.nodes[:,1]
z3 = copy2.nodes[:,2]
ax3.scatter3D(x3, y3, z3)
ax3.scatter3D(target3[0], target3[1], target3[2], 'red')
ax3.plot3D(x3, y3, z3, 'gray')

x4 = s1.nodes[:,0]
y4 = s1.nodes[:,1]
z4 = s1.nodes[:,2]
ax4.scatter3D(x4, y4, z4)
ax4.plot3D(x4, y4, z4, 'gray')

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

