# 

from numpy import sin, cos, arctan2, arcsin, cross, array, linalg, dot, sqrt, arccos, ndarray
from math import radians, degrees

def magnitude(p):
    return linalg.norm(p)

def normalize(p):
    if magnitude(p) == 0:
        return [0, 0, 0]
    return p / magnitude(p)

## CHECK TO SEE IF ABSOLUTE WORKS
def orthogonal(p):
    x = abs(p[0])
    y = abs(p[1])
    z = abs(p[2])

    if (x < y): 
        if (x < z):
            temp = [1, 0, 0]
        else: 
            temp = [0, 0, 1]
    else: 
        if (y < z): 
            temp = [0, 1, 0]
        else: 
            temp = [0, 0, 1]

    return cross(p, temp)

class Quaternion: 
    def __init__(self, w, x, y, z): 
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def normalize(self): 
        nom = normalize([self.w, self.x, self.y, self.z])
        return Quaternion(nom[0], nom[1], nom[2], nom[3])

    def toList(self):
        return [self.w, self.x, self.y, self.z]

    def inverse(self):
        return Quaternion(self.w, self.x * -1, self.y * -1, self.z * -1)

    def multiply(self, q):
        xyz   = array([self.x, self.y, self.z])
        q_xyz = array([q.x, q.y, q.z])

        new_xyz = cross(xyz, q_xyz) + (self.w * q_xyz) + (q.w * xyz)
        new_w   = self.w * q.w - dot(xyz, q_xyz)

        # new_xyz = [float('{:.4f}'.format(float('{:.4g}'.format(temp)))) for temp in new_xyz]
        # new_w   = float('{:.4f}'.format(float('{:.4g}'.format(new_w))))
        return Quaternion(new_w, new_xyz[0], new_xyz[1], new_xyz[2])

    def equals(self, q):
        w_check = self.w == q.w
        x_check = self.x == q.x
        y_check = self.y == q.y
        z_check = self.z == q.z

        return w_check and x_check and y_check and z_check

    def __str__(self):
        return "%.5f %.5fi %.5fj %.5fk" % (self.w, self.x, self.y, self.z)

class EulerAngles:
    def __init__(self, psi, theta, phi):
        self.psi = psi
        self.theta = theta
        self.phi = phi

    def toList(self):
        return [degrees(self.phi), degrees(self.theta), degrees(self.psi)]

    def setByIndex(self, index, angle):
        if index == 0:
            self.phi = angle
        elif index == 1:
            self.theta = angle
        else:
            self.psi = angle

    def __str__(self):
        return ("Degrees: psi: %.5f, theta: %.5f, phi: %.5f\nRadians: psi: %.5f, theta: %.5f, phi: %.5f" % 
                    (degrees(self.psi), degrees(self.theta), degrees(self.phi), self.psi, self.theta, self.phi))

#Q=[s1.*c2.*c3+c1.*s2.*s3,   c1.*s2.*c3-s1.*c2.*s3,   c1.*c2.*s3+s1.*s2.*c3,   c1.*c2.*c3-s1.*s2.*s3]
def radsToQuaternion(psi, theta, phi):
    c3 = cos(psi / 2)
    s3 = sin(psi / 2)
    c2 = cos(theta / 2)
    s2 = sin(theta / 2)
    c1 = cos(phi / 2)
    s1 = sin(phi / 2)

    w = (c1 * c2 * c3) + (s1 * s2 * s3)
    x = (s1 * c2 * c3) - (c1 * s2 * s3)
    y = (c1 * s2 * c3) + (s1 * c2 * s3)
    z = (c1 * c2 * s3) - (s1 * s2 * c3)

    return Quaternion(w, x, y, z).normalize()

def degreesToQuaternion(psi, theta, phi): 
    return radsToQuaternion(radians(psi), radians(theta), radians(phi))

def eulerToQuat(e):
    return radsToQuaternion(e.psi, e.theta, e.phi)

def quatToEuler(q):
    phi1  = 2 * ((q.w * q.x) + (q.y * q.z))
    phi2  = 1 - (2 * (q.x * q.x + q.y * q.y))
    phi   = arctan2(phi1, phi2)

    theta = 2 * ((q.w * q.y) - (q.z * q.x))
    theta = 1 if theta > +1.0 else theta
    theta = -1 if theta < -1.0 else theta
    theta = arcsin(theta)

    psi1  = 2 * ((q.w * q.z) + (q.x * q.y))
    psi2  = 1 - (2 * (q.y * q.y + q.z * q.z))
    psi   = arctan2(psi1, psi2)

    temp_list = [psi, theta, phi]
    psi, theta, phi = [float('{:.8f}'.format(float('{:.8f}'.format(temp)))) for temp in temp_list]
    return EulerAngles(psi, theta, phi)

# p' = qpq*
# p  = (0,  v) = 0  + iv1 + jv2 + kv3
# q  = (q0, q) = q0 + iq1 + jq2 + kq3 
# q* = (q0,-q) = q0 - iq1 - jq2 - kq3 = conjugate quaternion 

# t = 2q x v 
# v' = v + q0t + q x t

# p is a 3D vector (i, j, k)
def rotatePByQuat(p, q):
    if (type(p) != ndarray):
        p  = array(p)

    q = q.normalize()
    q_vec  = array([q.x, q.y, q.z])
    t      = cross(2 * q_vec, p)
    result = p + (q.w * t) + cross(q_vec, t)
    return [float('{:.10f}'.format(float('{:.10g}'.format(temp)))) for temp in result]

def rotatePByEuler(p, e):
    return rotatePByQuat(p, eulerToQuat(e))

def quatToAxisAngle(q):
    theta = 2 * arccos(q.w)
    if theta != 0: 
        x = q.x / sin(theta/2)
        y = q.y / sin(theta/2)
        z = q.z / sin(theta/2)
        return ([x,y,z], theta)
    else: 
        return ([0,0,0], 0)

def axisAngleToQuat(axis, angle):
    w = cos(angle/2)
    x = sin(angle/2) * axis[0]
    y = sin(angle/2) * axis[1]
    z = sin(angle/2) * axis[2]
    return Quaternion(w,x,y,z)


# https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
# https://github.com/toji/gl-matrix/blob/f0583ef53e94bc7e78b78c8a24f09ed5e2f7a20c/src/gl-matrix/quat.js#L54
def findQuatforVecs(v1, v2):
    v1 = normalize(v1)
    v2 = normalize(v2)
    _dot = dot(v1, v2)
    k    = sqrt(magnitude(v1) * magnitude(v2))
    if (_dot / k == -1):
        ortho = normalize(orthogonal(v1))
        return Quaternion(0, ortho[0], ortho[1], ortho[2])

    quat_vec = cross(v1, v2)
    quat_w   = _dot + k
    return Quaternion(quat_w, quat_vec[0], quat_vec[1], quat_vec[2]).normalize()

def findAxisAngleforVecs(v1, v2): 
    axis = normalize(cross(v1, v2))
    if all(a == 0 for a in axis): 
        return [axis, 0]
    angle = arccos(dot(v1, v2) / (magnitude(v1) * magnitude(v2)))
    return [axis, angle]

## ORIENTATION FRAMES --> MOVE THE ORIGIN TO A SPECIFIC JOINT AND 
## THE VECTOR TO THE NEXT JOINT REPRESENTS MOTION 

## CREATE A HIERARCHY OF ROTATIONS --> 3 QUATERNIONS OR EULER ANGLES FOR
## DIFFERENT AXIS MOVEMENT


