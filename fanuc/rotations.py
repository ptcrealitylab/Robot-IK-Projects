# 

from numpy import sin, cos, arctan2, arcsin, cross, array, linalg, dot, sqrt, arccos
from math import radians, degrees

def magnitude(p):
    return linalg.norm(p)

def normalize(p):
    if magnitude(p) == 0:
        return [0, 0, 0]
    return p / magnitude(p)

def orthogonal(p):
    x = p[0]
    y = p[1]
    z = p[2]

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

    def __str__(self):
        return "%.5f %.5fi %.5fj %.5fk" % (self.w, self.x, self.y, self.z)

class EulerAngles:
    def __init__(self, psi, theta, phi):
        self.psi = psi
        self.theta = theta
        self.phi = phi

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

    return EulerAngles(psi, theta, phi)

# p' = qpq*
# p  = (0,  v) = 0  + iv1 + jv2 + kv3
# q  = (q0, q) = q0 + iq1 + jq2 + kq3 
# q* = (q0,-q) = q0 - iq1 - jq2 - kq3 = conjugate quaternion 

# t = 2q x v 
# v' = v + q0t + q x t

# p is a 3D vector (i, j, k)
def rotatePByQuat(p, q):
    if (type(p) != np.ndarray):
        p  = array(p)

    q_vec  = array([q.x, q.y, q.z])
    t      = cross(2 * q_vec, p)
    return p + (q.w * t) + cross(q_vec, t)

def rotatePByEuler(p, e):
    return rotatePByQuat(p, eulerToQuat(e))


# https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
# https://github.com/toji/gl-matrix/blob/f0583ef53e94bc7e78b78c8a24f09ed5e2f7a20c/src/gl-matrix/quat.js#L54
def findQuatforVecs(v1, v2):
    _dot = dot(v1, v2)
    k    = sqrt(magnitude(v1) * magnitude(v2))
    if (_dot / k == -1):
        ortho = orthogonal(v1)
        return Quaternion(0, ortho[0], ortho[1], ortho[2])

    quat_vec = normalize(cross(v1, v2))
    quat_w   = _dot + k
    return Quaternion(quat_w, quat_vec[0], quat_vec[1], quat_vec[2]).normalize()

def findAxisAngleforVecs(v1, v2): 
    axis = normalize(cross(v1, v2))
    if all(a == 0 for a in axis): 
        return [axis, 0]
    angle = arccos(dot(v1, v2) / (magnitude(v1) * magnitude(v2)))
    return [axis, angle]

# TESTS 


## 
q = Quaternion(0,1,2,3)
# SHOULD KEEP EVERYTHING WITHIN -pi/2 <= deg <= pi/2 
# OR ELSE THE ANGLES WILL BE FLIPPED SINCE THERE ARE NOW 2 POSSIBLE SOLUTIONS

# TESTING CONVERSIONS 
e = EulerAngles(radians(60), radians(60), radians(45))
e_to_q = eulerToQuat(e)
q_to_e = quatToEuler(e_to_q)
e_back_to_q = eulerToQuat(q_to_e)
q_back_to_e = quatToEuler(e_back_to_q)
# print(e_to_q)
# print(q_to_e)
# print(e_back_to_q)
# print(q_back_to_e)

# TESTING QUATERNION FOR VECTORS AND RECOVERING ANGLES
q_fromvecs = findQuatforVecs([0,1,0], [1,0,0])
print(q_fromvecs)
print(quatToEuler(q_fromvecs))
[axis, angle] = findAxisAngleforVecs([0,1,0], [1,0,0])
print(axis, angle)


