"""\
Geometry module. Contains point (vector) and pose transformation classes, and
geometric descriptor functions for features.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from math import pi, sqrt, sin, cos, asin, acos, atan2, copysign
from functools import reduce


class Angle(float):
    """\
    Angle class. All operations are modulo 2S{pi}.
    """
    def __new__(cls, arg=0.0):
        return float.__new__(cls, float(arg) % (2 * pi))

    def __add__(self, other):
        return Angle(float(self) + float(other))

    def __sub__(self, other):
        return Angle(float(self) - float(other))

    def __neg__(self):
        return Angle(-float(self))


class Point(tuple):
    """\
    3D point (vector) class.
    """
    def __new__(cls, iterable=(0.0, 0.0, 0.0)):
        """\
        Constructor.
        """
        return tuple.__new__(cls, iterable)

    def __hash__(self):
        """\
        Hash function.
        """
        return hash(repr(self))

    def __eq__(self, p):
        """\
        Equality function.

        @param p: The other point.
        @type p: L{Point}
        """
        try:
            return all([abs(self[i] - p[i]) < 1e-4 for i in range(len(self))])
        except AttributeError:
            return False

    def __neq__(self, p):
        """\
        Inequality function.

        @param p: The other point.
        @type p: L{Point}
        """
        return not self.__eq__(p)

    def __add__(self, p):
        """\
        Vector addition.

        @param p: The operand vector.
        @type p: L{Point}
        @return: Result vector.
        @rtype: L{Point}
        """
        return self.__class__(tuple([self[i] + p[i] for i in range(3)]) + \
            self[3:])

    def __sub__(self, p):
        """\
        Vector subtraction.

        @param p: The operand vector.
        @type p: L{Point}
        @return: Result vector.
        @rtype: L{Point}
        """
        return self.__class__(tuple([self[i] - p[i] for i in range(3)]) + \
            self[3:])

    def __mul__(self, p):
        """\
        Scalar multiplication (if C{p} is a scalar) or dot product (if C{p} is a
        vector).

        @param p: The operand scalar or vector.
        @type p: C{float} or L{Point}
        @return: Result vector.
        @rtype: L{Point}
        """
        try:
            return self[0] * p[0] + self[1] * p[1] + self[2] * p[2]
        except TypeError:
            return self.__class__(tuple([self[i] * p for i in range(3)]) + \
                self[3:])

    def __rmul__(self, p):
        """\
        Scalar multiplication (if C{p} is a scalar) or dot product (if C{p} is a
        vector).

        @param p: The operand scalar or vector.
        @type p: C{float} or L{Point}
        @return: Result vector.
        @rtype: L{Point}
        """
        return self.__mul__(p)

    def __div__(self, p):
        """\
        Scalar division.

        @param p: The scalar divisor.
        @type p: C{float}
        @return: Result vector.
        @rtype: L{Point}
        """
        return self.__class__(tuple([self[i] / p for i in range(3)]) + self[3:])

    def __pow__(self, p):
        """\
        Cross product.

        @param p: The operand vector.
        @type p: L{Point}
        @return: Result vector.
        @rtype: L{Point}
        """
        return self.__class__(tuple([self[(i + 1) % 3] * p[(i + 2) % 3] - \
            self[(i + 2) % 3] * p[(i + 1) % 3] for i in range(3)]) + self[3:])

    def __neg__(self):
        """\
        Negation.

        @return: Result vector.
        @rtype: L{Point}
        """
        return self.__class__([-self[i] for i in range(3)])

    def __repr__(self):
        """\
        Canonical string representation.

        @return: Canonical string representation.
        @rtype: C{str}
        """
        return '%s((%.4f, %.4f, %.4f))' % (self.__class__.__name__,
            self[0], self[1], self[2])

    def __str__(self):
        """\
        String representation, displays in a tuple format.

        @return: Vector string.
        @rtype: C{str}
        """
        return '(%.4f, %.4f, %.4f)' % self

    @property
    def x(self):
        return self[0]

    @property
    def y(self):
        return self[1]

    @property
    def z(self):
        return self[2]

    @property
    def magnitude(self):
        """\
        Return the magnitude of this vector.

        @rtype: C{float}
        """
        try:
            return self._magnitude
        except AttributeError:
            self._magnitude = sqrt(sum([self[i] ** 2 for i in range(3)]))
            return self._magnitude

    @property
    def normal(self):
        """\
        Return a normalized (unit) vector in the direction of this vector.

        @rtype: L{Point}
        """
        try:
            return self._normal
        except AttributeError:
            m = self.magnitude
            try:
                self._normal = Point([self[i] / m for i in range(3)])
                return self._normal
            except ZeroDivisionError:
                raise ValueError('cannot normalize a zero vector')

    def euclidean(self, p):
        """\
        Return the Euclidean distance from this point to another.

        @param p: The other point.
        @type p: L{Point}
        @return: Euclidean distance.
        @rtype: C{float}
        """
        return sqrt(sum([(self[i] - p[i]) ** 2 for i in range(3)]))

    def angle(self, p):
        """\
        Return the angle between this vector and another.
    
        @param p: The other vector.
        @type p: L{Point}
        @return: Angle in radians.
        @rtype: L{Angle}
        """
        return Angle(acos(p.normal * self.normal))


class DirectionalPoint(Point):
    """\
    3D directional point (spatial-directional vector) class.
    """
    def __new__(cls, iterable=(0.0, 0.0, 0.0, 0.0, 0.0)):
        """\
        Constructor.
        """
        iterable = list(iterable[:3]) + [Angle(iterable[3]), Angle(iterable[4])]
        if iterable[3] > pi:
            iterable[3] -= 2. * (iterable[3] - pi)
            iterable[4] += pi
        return Point.__new__(cls, iterable)

    def __neg__(self):
        """\
        Negation.

        @return: Result vector.
        @rtype: L{DirectionalPoint}
        """
        return DirectionalPoint(tuple([-self[i] for i in range(3)]) + \
                                (self[3] + pi, self[4]))

    def __repr__(self):
        """\
        Canonical string representation.

        @return: Spatial-directional vector string.
        @rtype: C{str}
        """
        return '%s((%.4f, %.4f, %.4f, %.4f, %.4f))' % (self.__class__.__name__,
            self[0], self[1], self[2], self[3], self[4])

    def __str__(self):
        """\
        String representation, displays in a tuple format.

        @return: Spatial-directional vector string.
        @rtype: C{str}
        """
        return '(%.4f, %.4f, %.4f, %.4f, %.4f)' % self

    @property
    def rho(self):
        return self[3]

    @property
    def eta(self):
        return self[4]

    @property
    def direction_unit(self):
        """\
        Return a unit vector representation of the direction of this
        directional point.

        @rtype: L{Point}
        """
        return Point((sin(self[3]) * cos(self[4]),
                      sin(self[3]) * sin(self[4]), cos(self[3])))


class Quaternion(tuple):
    """\
    Quaternion class.
    """
    def __new__(cls, iterable=(1.0, Point())):
        """\
        Constructor.
        """
        return tuple.__new__(cls, (iterable[0], Point(iterable[1][:3])))

    @property
    def a(self):
        return self[0]

    @property
    def v(self):
        return self[1]

    @property
    def b(self):
        return self[1][0]

    @property
    def c(self):
        return self[1][1]

    @property
    def d(self):
        return self[1][2]

    def __add__(self, q):
        """\
        Quaternion addition.

        @param q: The operand quaternion.
        @type q: L{Quaternion}
        @return: Result quaternion.
        @rtype: L{Quaternion}
        """
        return Quaternion([self[i] + q[i] for i in range(2)])

    def __sub__(self, q):
        """\
        Quaternion subtraction.

        @param q: The operand quaternion.
        @type q: L{Quaternion}
        @return: Result quaternion.
        @rtype: L{Quaternion}
        """
        return Quaternion([self[i] - q[i] for i in range(2)])

    def __mul__(self, q):
        """\
        Quaternion multiplication.

        @param q: The operand scalar or quaternion.
        @type q: L{Quaternion}
        @return: Result quaternion.
        @rtype: L{Quaternion}
        """
        return Quaternion(((self[0] * q[0] - self[1] * q[1]),
            (self[0] * q[1] + q[0] * self[1] + self[1] ** q[1])))

    def __div__(self, q):
        """\
        Scalar division.

        @param q: The scalar divisor.
        @type q: C{float}
        @return: Result quaternion.
        @rtype: L{Quaternion}
        """
        return Quaternion((self[0] / q, self[1] / q))

    def __neg__(self):
        """\
        Negation.

        @return: Result quaternion.
        @rtype: L{Quaternion}
        """
        return Quaternion((-self[0], -self[1]))

    def __repr__(self):
        """\
        Canonical string representation.

        @return: Canonical string representation.
        @rtype: C{str}
        """
        return '%s(%f, %s)' % (self.__class__.__name__, self[0], self[1])

    def __str__(self):
        """\
        String representation, displays in a tuple format.

        @return: Vector string.
        @rtype: C{str}
        """
        return '(%.2f, %s)' % self

    @property
    def magnitude(self):
        """\
        Return the magnitude (norm) of this quaternion.

        @rtype: C{float}
        """
        try:
            return self._magnitude
        except AttributeError:
            self._magnitude = sqrt(self[0] ** 2 + \
                sum([self[1][i] ** 2 for i in range(3)]))
            return self._magnitude

    @property
    def conjugate(self):
        """\
        Return the conjugate of this quaternion.

        @rtype: L{Quaternion}
        """
        try:
            return self._conjugate
        except AttributeError:
            self._conjugate = Quaternion((self[0], -self[1]))
            return self._conjugate

    @property
    def inverse(self):
        """\
        Return the multiplicative inverse of this quaternion.

        @rtype: L{Quaternion}
        """
        try:
            return self._inverse
        except AttributeError:
            self._inverse = self.conjugate / (self.magnitude ** 2)
            return self._inverse

    def rotate(self, p):
        """\
        Rotate (conjugate) the operand point by this quaternion.

        @param p: The operand point.
        @type p: L{Point}
        @return: The rotated (conjugated) point.
        @rtype: L{Point}
        """
        return (self * Quaternion((0.0, p)) * self.inverse)[1]


class Rotation(object):
    """\
    3D Euclidean rotation class. Handles multiple representations of SO(3).
    """
    __slots__ = ['Q']

    def __init__(self, Q=Quaternion()):
        """\
        Constructor.
        """
        self.Q = Q

    def __getstate__(self):
        return self.Q

    def __setstate__(self, state):
        self.Q = state

    def __repr__(self):
        """\
        Canonical string representation.

        @return: Canonical string representation.
        @rtype: C{str}
        """
        return '%s(%s)' % (self.__class__.__name__, str(self.Q))

    def __add__(self, other):
        """\
        Rotation composition.

        @param other: The other rotation.
        @type other: L{Rotation}
        @return: The composed rotation.
        @rtype: L{Rotation}
        """
        return Rotation(self.Q * other.Q)

    def __sub__(self, other):
        """\
        Rotation composition with the inverse.

        @param other: The other rotation.
        @type other: L{Rotation}
        @return: The composed rotation.
        @rtype: L{Rotation}
        """
        return self.__add__(other.inverse)

    def __neg__(self):
        """\
        Negation.

        @return: The inverse rotation.
        @rtype: L{Rotation}
        """
        return Rotation(self.Q.inverse)

    def __hash__(self):
        """\
        Hash.

        @return: Hash of this rotation.
        @rtype: C{int}
        """
        return hash(self.Q)

    def rotate(self, p):
        """\
        Rotate a vector.

        @param p: The vector to rotate.
        @type p: L{Point}
        @return: The rotated vector.
        @rtype: L{Point}
        """
        return self.Q.rotate(p)

    @staticmethod
    def from_rotation_matrix(R):
        """\
        Generate the internal quaternion representation from a rotation matrix.

        @param R: The rotation matrix.
        @type R: C{list} of C{list}
        @return: Quaternion representation of the rotation.
        @rtype: L{Rotation}
        """
        u = max([(abs(R[i][i]), i) for i in range(3)])[1]
        v, w = (u + 1) % 3, (u + 2) % 3
        r = sqrt(1.0 + R[u][u] - R[v][v] - R[w][w])
        if r == 0:
            return Quaternion()
        Qa = (R[w][v] - R[v][w]) / (2.0 * r)
        Qv = [0.0] * 3
        Qv[u] = r / 2.0
        Qv[v] = (R[u][v] + R[v][u]) / (2.0 * r)
        Qv[w] = (R[w][u] + R[u][w]) / (2.0 * r)
        return Rotation(Quaternion((Qa, Point(Qv))))
    
    @staticmethod
    def from_axis_angle(theta, axis):
        """\
        Generate the internal quaternion representation from an axis and
        angle representation.

        @param theta: The angle of rotation.
        @type theta: L{Angle}
        @param axis: The axis of rotation.
        @type axis: L{Point}
        @return: Quaternion representation of the rotation.
        @rtype: L{Rotation}
        """
        return Rotation(Quaternion((cos(theta / 2.0), sin(theta / 2.0) \
            * axis.normal)))

    @staticmethod
    def from_euler(convention, angles):
        """\
        Generate the internal quaternion representation from Euler angles.

        @param convention: The convention to use (e.g. C{zyx}).
        @type convention: C{str}
        @param angles: The angles of rotation.
        @type angles: C{tuple} of L{Angle}
        @return Quaternion representation of the rotation.
        @rtype: L{Rotation}
        """
        def qterm(index):
            bin = lambda x: [(x >> 2) % 2, (x >> 1) % 2, x % 2]
            return copysign(1, index) * reduce(lambda a, b: a * b,
                [(bit and sin or cos)(angles[i] / 2.0) \
                for i, bit in enumerate(bin(abs(index)))])
        eulerquat = {'xyx': [0, -5, 1, 4, 2, 7, 3, -6],
                     'xyz': [0, -7, 1, 6, 2, -5, 3, 4],
                     'xzx': [0, -5, 1, 4, -3, 6, 2, 7],
                     'xzy': [0, 7, 1, -6, -3, 4, 2, 5],
                     'yxy': [0, -5, 2, 7, 1, 4, -3, 6],
                     'yxz': [0, 7, 2, 5, 1, -6, -3, 4],
                     'yzx': [0, -7, 3, 4, 1, 6, 2, -5],
                     'yzy': [0, -5, 3, -6, 1, 4, 2, 7],
                     'zxy': [0, -7, 2, -5, 3, 4, 1, 6],
                     'zxz': [0, -5, 2, 7, 3, -6, 1, 4],
                     'zyx': [0, 7, -3, 4, 2, 5, 1, -6],
                     'zyz': [0, -5, -3, 6, 2, 7, 1, 4]}
        a, b, c, d = [sum([qterm(eulerquat[convention][i + j * 2]) \
                      for i in range(2)]) for j in range(4)]
        if a > 0:
            return Rotation(Quaternion((a, Point((-b, -c, -d)))))
        else:
            return Rotation(Quaternion((-a, Point((b, c, d)))))

    def to_rotation_matrix(self):
        """\
        Return the rotation matrix representation from the internal quaternion
        representation.

        @return: Rotation matrix.
        @rtype: C{list} of C{list}
        """
        a = self.Q[0]
        b, c, d = self.Q[1]
        R = [[a ** 2 + b ** 2 - c ** 2 - d ** 2,
              2.0 * b * c - 2.0 * a * d,
              2.0 * b * d + 2.0 * a * c],
             [2.0 * b * c + 2.0 * a * d,
              a ** 2 - b ** 2 + c ** 2 - d ** 2,
              2.0 * c * d - 2.0 * a * b],
             [2.0 * b * d - 2.0 * a * c,
              2.0 * c * d + 2.0 * a * b,
              a ** 2 - b ** 2 - c ** 2 + d ** 2]]
        return R

    def to_axis_angle(self):
        """\
        Return the axis and angle representation from the internal quaternion
        representation.

        @return: Axis and angle rotation form.
        @rtype: C{tuple} of L{Point} and L{Angle}
        """
        theta = Angle(copysign(2.0 * acos(self.Q.a), self.Q.v.magnitude))
        try:
            return (self.Q.v.normal, theta)
        except ValueError:
            return (Point((1.0, 0.0, 0.0)), theta)

    def to_euler_zyx(self):
        """\
        Return three fixed-axis (Euler zyx) rotation angles from the internal
        rotation matrix representation.

        @return: Three fixed-axis (Euler zyx) angle rotation form.
        @rtype: C{tuple} of L{Angle}
        """
        a = self.Q[0]
        b, c, d = self.Q[1]
        theta = atan2(2.0 * (c * d - a * b), 1.0 - 2.0 * (b ** 2 + c ** 2))
        phi = -asin(2.0 * (a * c + d * b))
        psi = atan2(2.0 * (b * c - a * d), 1.0 - 2.0 * (c ** 2 + d ** 2))
        return (theta, phi, psi)


class Pose(object):
    """\
    Pose (rigid 3D Euclidean transformation) class.
    """
    __slots__ = ['_T', '_R', '_inverse']

    def __init__(self, T=Point(), R=Rotation()):
        """\
        Constructor.

        @param T: The 3-element translation vector.
        @type T: L{Point}
        @param R: The 3x3 rotation matrix.
        @type R: L{Rotation}
        """
        self._T = T
        self._R = R
    
    @property
    def T(self):
        return self._T

    @property
    def R(self):
        return self._R

    def __getstate__(self):
        return self._T, self._R

    def __setstate__(self, state):
        self._T, self._R = state

    def __add__(self, other):
        """\
        Pose composition: M{PB(PA(x)) = (PA + PB)(x)}.

        @param other: The other pose transformation.
        @type other: L{Pose}
        @return: Composed transformation.
        @rtype: L{Pose}
        """
        Tnew = other.R.rotate(self.T) + other.T
        Rnew = other.R + self.R
        return Pose(Tnew, Rnew)

    def __sub__(self, other):
        """\
        Pose composition with the inverse.
    
        @param other: The other pose transformation.
        @type other: L{Pose}
        @return: Composed transformation.
        @rtype: L{Pose}
        """
        return self.__add__(-other)

    def __neg__(self):
        """\
        Pose inversion (with caching).

        @return: Inverted pose.
        @rtype: L{Pose}
        """
        try:
            return self._inverse
        except AttributeError:
            self._inverse = Pose(-(-self.R).rotate(self.T), -self.R)
            return self._inverse

    def __hash__(self):
        """\
        Hash.

        @return: Hash of this pose.
        @rtype: C{int}
        """
        return hash(self.T) + hash(self.R)

    def __repr__(self):
        """\
        String representation, display T and R.

        @return: String representations of T and R.
        @rtype: C{str}
        """
        return '%s(%s, %s)' % (self.__class__.__name__, self.T, self.R)

    @property
    def nonzero(self):
        """\
        Check if this pose transformation has any effect.

        @rtype: C{bool}
        """
        if sum(self.T.tuple) == 0 and sum(self.R.Q.v.tuple) == 0:
            return False
        else:
            return True

    def map(self, p):
        """\
        Map a point/vector through this pose.

        @param p: The point/vector to transform.
        @type p: L{Point}
        @return: The mapped point/vector.
        @rtype: L{Point}
        """
        q = self.R.rotate(p)
        try:
            unit = self.R.rotate(p.direction_unit)
            try:
                rho = acos(unit.z)
            except ValueError:
                if unit.z > 0:
                    rho = 0.0
                else:
                    rho = pi
            eta = atan2(unit.y, unit.x)
            return DirectionalPoint(tuple(q) + (rho, eta)) + self.T
        except AttributeError:
            return q + self.T


class Face(object):
    """\
    Face class.
    """
    __slots__ = ['vertices', 'edges', '_normal', '_planing_pose']

    def __init__(self, vertices):
        """\
        Constructor.

        @param v: Vertices (in counterclockwise order looking at the face).
        @type v: C{tuple} of L{Point}
        """
        self.vertices = vertices
        self.edges = [vertices[(i + 1) % len(vertices)] - vertices[i] \
            for i in range(len(vertices))]

    @property
    def normal(self):
        """\
        Normal vector to this triangle.
        """
        try:
            return self._normal
        except AttributeError:
            self._normal = (self.edges[0] ** self.edges[1]).normal
            return self._normal

    @property
    def planing_pose(self):
        """\
        Pose which transforms this face into the x-y plane, with the first
        vertex at the origin.

        @rtype: L{Pose}
        """
        try:
            return self._planing_pose
        except AttributeError:
            angle = self.normal.angle(Point((0, 0, 1)))
            axis = self.normal ** Point((0, 0, 1))
            try:
                R = Rotation.from_axis_angle(angle, axis)
            except ValueError:
                R = Rotation()
            self._planing_pose = Pose(T=-R.rotate(self.vertices[0]), R=R)
            return self._planing_pose


class Triangle(Face):
    """\
    Triangle class.
    """
    __slots__ = ['vertices', 'edges', '_normal', '_planing_pose']

    def __init__(self, vertices):
        """\
        Constructor.

        @param v: Vertices.
        @type v: C{tuple} of L{Point}
        """
        super(Triangle, self).__init__(vertices)

    def intersection(self, origin, end):
        """\
        Return whether the line segment between the two given points intersects
        with this triangle.

            - T. Möller and B. Trumbore, "Fast, Minimum Storage Ray/Triangle
              Intersection," J. Graphics Tools, vol. 2, no. 1, pp. 21-28, 1997.
        
        @param origin: The origin of the segment.
        @type origin: L{Point}
        @param end: The end of the segment.
        @type end: L{Point}
        @return: True if intersection exists.
        @rtype: C{bool}
        """
        direction = (end - origin).normal
        P = direction ** -self.edges[2]
        det = self.edges[0] * P
        if det > -1e-4 and det < 1e-4:
            return False
        inv_det = 1.0 / det
        T = origin - self.vertices[0]
        u = (T * P) * inv_det
        if u < 0 or u > 1.0:
            return False
        Q = T ** self.edges[0]
        v = (direction * Q) * inv_det
        if v < 0 or u + v > 1.0:
            return False
        t = (-self.edges[2] * Q) * inv_det
        if t < 1e-04 or t > origin.euclidean(end) - 1e-04:
            return False
        return True


def which_side(points, direction, vertex):
    """\
    Check which side of the projection of the given vertex onto the given
    direction the projections of the given points lie upon.

        - D. Eberly, "Intersection of Convex Objects: The Method of Separating
          Axes," 2008.

    @param points: The points to check.
    @type points: C{list} of L{Point}
    @param direction: The direction upon which to project.
    @type direction: L{Point}
    @param vertex: The criterion vertex.
    @type vertex: L{Point}
    @return: 0 if positive side, -1 if negative side, 0 if both.
    @rtype: C{int}
    """
    positive, negative = 0, 0
    for point in points:
        t = direction * (point - vertex)
        if t > 0:
            positive += 1
        elif t < 0:
            negative += 1
        if positive and negative:
            return 0
    if positive:
        return 1
    else:
        return -1
