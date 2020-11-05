from rotations import * 

# rotations.py TESTS 


## 
q = Quaternion(0,1,2,3)
# SHOULD KEEP EVERYTHING WITHIN -pi/2 <= deg <= pi/2 
# OR ELSE THE ANGLES WILL BE FLIPPED SINCE THERE ARE NOW 2 POSSIBLE SOLUTIONS

# TESTING CONVERSIONS 
e = EulerAngles(0, radians(60), 0)
e_to_q = eulerToQuat(e)
print(e_to_q)
# exit()
q_to_e = quatToEuler(e_to_q)
e_back_to_q = eulerToQuat(q_to_e)
q_back_to_e = quatToEuler(e_back_to_q)
# print(e_to_q)
# print(q_to_e)
# print(e_back_to_q)
# print(q_back_to_e)

# TESTING QUATERNION FOR VECTORS AND RECOVERING ANGLES
q_fromvecs = findQuatforVecs([.5,.5,0], [1,0,0])
q_fromvec3 = findQuatforVecs([1, 0,0], [0,1,0])
q_fromvec2 = findQuatforVecs([1,2,3], [5,2,6])
print(q_fromvec3)
print(q_fromvecs)
print(q_fromvec2)
print(quatToEuler(q_fromvec2))
print(quatToEuler(q_fromvecs))
print(quatToEuler(q_fromvec3))
rotate_1 = rotatePByQuat([0,1,0], q_fromvecs)
rotate_2 = rotatePByQuat([0,1,0], q_fromvec2)
rotate_3 = rotatePByQuat([10,10,10], q_fromvec2)
print(rotate_1)
print(rotate_2)
print(rotate_3)
print(findQuatforVecs([10,10,10], rotate_3))
print(findQuatforVecs([0,1,0], rotate_2))
print(findQuatforVecs([0,1,0], rotate_1))
[axis, angle] = findAxisAngleforVecs([.5,.5,0], [1,0,0])
print(axis, angle)
[axis, angle] = quatToAxisAngle(q_fromvecs)
print(axisAngleToQuat(axis, angle))