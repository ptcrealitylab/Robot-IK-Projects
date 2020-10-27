import numpy as np

# EVERYTHING IN MM
# https://www.sciencedirect.com/science/article/pii/S1524070311000178?via%3Dihub

# SPECIFY AN ORIENTATION AND POSITION 
# FABRIK

# A manipulator such as a robot arm or an animated graphics character 
# is modelled as a chain composed of rigid links connected at their ends
#  by rotating joints. Any translation and/or rotation of the i-th joint 
#  affects the translation and rotation of any joint placed later in the chain.

# FOR A 6 DOF ROBOT
# 6 motors
# 5 links 
# 6 motor restrictions [-360, 360] and normal vectors
# starting configuration and base plane

# Motor coordinates: 
# [x,y,z] = j
# [j1, j2, j3, ...] = jointpos
# [[0, 1], [1, 2], [2, 3], [3, 4]] = jointlinks

# NORMAL VECTOR OF ROBOT BASE (normal vector of initial motor range)
# [1,0,0] OR [0,1,0] OR [0,0,1] OR [-1,0,0] OR [0,-1,0] OR [0,0,-1] = ground_plane

class RobotArm:
	# TODO: Add volume estimate of each joint link (for collision tracking)
	def __init__(self, pos=None, links=None, start=None, restrictions=None, plane=None):
		self.jointpos   		= pos
		self.jointlinks 		= links
		self.starting_config    = start
		self.joint_restrictions = restrictions
		self.ground_plane       = plane

	# Rotate specified joint to a desired angle
	def RotateJointToAngle(self):
		return None

	# return current joint position coordinates
	def ReturnPosition(self):
		return None 

	# Attempt to move the robot to a specified coordinate position
	def GoToPosition(self):
		return None 

	# Determine the angle between two joint links
	def AngleBetweenJoints(self):
		return None 

	# output necessary data to create a Wireframe object
	def outputAsWireframe(self):
		return None 

	# FOR DEBUGGING: print current positions
	def PrintPositions(self):
		return None 

	# FOR DEBUGGING: print joint links
	def PrintLinks(self):
		return None 
