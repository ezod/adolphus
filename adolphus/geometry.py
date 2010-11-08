"""\
Geometry module. Contains point (vector) and pose transformation classes, and
geometric descriptor functions for features.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from math import pi, sqrt, sin, cos, asin, acos, atan, atan2, copysign
from numbers import Number
import numpy

from visualization import visual, VisualizationError, VisualizationObject

class Angle(float):
    """\
    Angle class. All operations are modulo 2 * pi.
    """
    def __new__(cls, arg=0.0):
        return float.__new__(cls, float(arg) % (2 * pi))

    def __eq__(self, other):
        return float(self) == float(other) % (2 * pi)

    def __ne__(self, other):
        return not self == other

    def __gt__(self, other):
        return float(self) > float(other) % (2 * pi)

    def __lt__(self, other):
        return float(self) < float(other) % (2 * pi)

    def __ge__(self, other):
        return float( self) >= float(other) % (2 * pi)

    def __le__(self, other):
        return float(self) <= float(other) % (2 * pi)

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
        assert len(iterable) == 3 or len(iterable) == 5
        return tuple.__new__(cls, iterable)

    def __eq__(self, p):
        """\
        Equality function.

        @param p: The other point.
        @type p: L{Point}
        """
        epsilon = 1e-4
        try:
            return all([abs(self[i] - p[i]) < epsilon \
                for i in range(len(self))])
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
        return Point(tuple([self[i] + p[i] for i in range(3)]) + self[3:])

    def __sub__(self, p):
        """\
        Vector subtraction.

        @param p: The operand vector.
        @type p: L{Point}
        @return: Result vector.
        @rtype: L{Point}
        """
        return Point(tuple([self[i] - p[i] for i in range(3)]) + self[3:])

    def __mul__(self, p):
        """\
        Scalar multiplication (if p is a scalar) or dot product (if p is a
        vector).

        @param p: The operand scalar or vector.
        @type p: C{float} or L{Point}
        @return: Result vector.
        @rtype: L{Point}
        """
        try:
            return sum([self[i] * p[i] for i in range(3)])
        except TypeError:
            return Point(tuple([self[i] * p for i in range(3)]) + self[3:])

    def __rmul__(self, p):
        """\
        Scalar multiplication (if p is a scalar) or dot product (if p is a
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
        return Point(tuple([self[i] / p for i in range(3)]) + self[3:])

    def __pow__(self, p):
        """\
        Cross product.

        @param p: The operand vector.
        @type p: L{Point}
        @return: Result vector.
        @rtype: L{Point}
        """
        if not isinstance(p, Point):
            raise TypeError('cross product operand must be a vector')
        return Point([self[(i + 1) % 3] * p[(i + 2) % 3] - self[(i + 2) % 3] \
            * p[(i + 1) % 3] for i in range(3)])

    def __neg__(self):
        """\
        Negation.

        @return: Result vector.
        @rtype: L{Point}
        """
        return Point([-self[i] for i in range(3)])

    def __repr__(self):
        """\
        Canonical string representation.

        @return: Canonical string representation.
        @rtype: C{str}
        """
        return '%s(%f, %f, %f)' % (self.__class__.__name__,
            self[0], self[1], self[2])

    def __str__(self):
        """\
        String representation, displays in a tuple format.

        @return: Vector string.
        @rtype: C{str}
        """
        return '(%.2f, %.2f, %.2f)' % self

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
    def array(self):
        """\
        Return a NumPy array of this vector.

        @rtype: C{numpy.ndarray}
        """
        return numpy.array([[self[0]], [self[1]], [self[2]]])

    @property
    def magnitude(self):
        """\
        Return the magnitude of this vector.

        @rtype: C{float}
        """
        return sqrt(sum([self[i] ** 2 for i in range(3)]))

    @property
    def normal(self):
        """\
        Return a normalized (unit) vector in the direction of this vector.

        @rtype: L{Point}
        """
        m = self.magnitude
        try:
            return Point([self[i] / m for i in range(3)])
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

    distance = euclidean

    def angle(self, p):
        """\
        Return the angle between this vector and another.
    
        @param p: The other vector.
        @type p: L{Point}
        @return: Angle in radians.
        @rtype: L{Angle}
        """
        return Angle(acos(p.normal * self.normal))

    def visualize(self, scale=1.0, color=(1, 1, 1), opacity=1.0):
        """\
        Plot the point in a 3D visual model.

        @param scale: The scale of the sphere.
        @type scale: C{float}
        @param color: The color in which to plot the point.
        @type color: C{tuple}
        @param opacity: The opacity with which to plot the point.
        @type opacity: C{float}
        """
        if not visual:
            raise VisualizationError("visual module not loaded")
        try:
            self.vis.members['point'].radius = 0.1 * scale
            self.vis.members['point'].color = color
            self.vis.members['point'].opacity = opacity
        except AttributeError:
            self.vis = VisualizationObject(self)
            self.vis.add('point', visual.sphere(frame=self.vis,
                radius=(0.1 * scale), color=color, opacity=opacity))
        self.vis.pos = self[:3]


class DirectionalPoint(Point):
    """\
    3D directional point (spatial-directional vector) class.
    """
    def __new__(cls, iterable=(0.0, 0.0, 0.0, 0.0, 0.0)):
        """\
        Constructor.
        """
        assert len(iterable) == 5
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
        return '%s(%f, %f, %f, %f, %f)' % (self.__class__.__name__,
            self[0], self[1], self[2], self[3], self[4])

    def __str__(self):
        """\
        String representation, displays in a tuple format.

        @return: Spatial-directional vector string.
        @rtype: C{str}
        """
        return '(%.2f, %.2f, %.2f, %.2f, %.2f)' % self

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

    def visualize(self, scale=1.0, color=(1, 1, 1), opacity=1.0):
        """\
        Plot the directional point in a 3D visual model.

        @param scale: The scale of the sphere.
        @type scale: C{float}
        @param color: The color in which to plot the point.
        @type color: C{tuple}
        @param opacity: The opacity with which to plot the point.
        @type opacity: C{float}
        """
        Point.visualize(self, scale=scale, color=color, opacity=opacity)
        unit = scale * self.direction_unit
        try:
            self.vis.members['dir'].axis = unit
            self.vis.members['dir'].color = color
            self.vis.members['dir'].opacity = opacity
        except KeyError:
            self.vis.add('dir', visual.arrow(frame=self.vis, axis=unit,
                color=color, opacity=opacity))


class Quaternion(tuple):
    """\
    Quaternion class.
    """
    def __new__(cls, iterable=(1.0, Point())):
        """\
        Constructor.
        """
        if len(iterable) == 4:
            return tuple.__new__(cls, (float(iterable[0]),
                                       Point(iterable[1:4])))
        else:
            assert len(iterable) == 2 and isinstance(iterable[1], Point)
            return tuple.__new__(cls, iterable)

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
        Scalar multiplication (if q is a scalar) or quaternion multiplication
        (if q is a quaternion).

        @param q: The operand scalar or quaternion.
        @type q: C{float} or L{Quaternion}
        @return: Result quaternion.
        @rtype: L{Quaternion}
        """
        try:
            return Quaternion(((self[0] * q[0] - self[1] * q[1]),
                Point(self[0] * q[1] + q[0] * self[1] + self[1] ** q[1])))
        except TypeError:
            return Quaternion([self[i] * q for i in range(2)])

    def __div__(self, q):
        """\
        Scalar division.

        @param q: The scalar divisor.
        @type q: C{float}
        @return: Result quaternion.
        @rtype: L{Quaternion}
        """
        return Quaternion([self[i] / q for i in range(2)])

    def __neg__(self):
        """\
        Negation.

        @return: Result quaternion.
        @rtype: L{Quaternion}
        """
        return Quaternion([-self[i] for i in range(2)])

    def __repr__(self):
        """\
        Canonical string representation.

        @return: Canonical string representation.
        @rtype: C{str}
        """
        return "%s(%f, %s)" % (self.__class__.__name__, self[0], self[1])

    def __str__(self):
        """\
        String representation, displays in a tuple format.

        @return: Vector string.
        @rtype: C{str}
        """
        return "(%.2f, %s)" % self

    @property
    def magnitude(self):
        """\
        Return the magnitude (norm) of this quaternion.

        @rtype: C{float}
        """
        return sqrt(self[0] ** 2 + sum([self[1][i] ** 2 for i in range(3)]))

    @property
    def conjugate(self):
        """\
        Return the conjugate of this quaternion.

        @rtype: L{Quaternion}
        """
        return Quaternion((self[0], -self[1]))

    @property
    def inverse(self):
        """\
        Return the multiplicative inverse of this quaternion.

        @rtype: L{Quaternion}
        """
        return self.conjugate / (self.magnitude ** 2)

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
    def __init__(self, Q=None):
        """\
        Constructor.
        """
        if Q is None:
            self.Q = Quaternion()
        elif isinstance(Q, Rotation):
            self.Q = Q.Q
        elif isinstance(Q, Quaternion):
            self.Q = Q
        else:
            self.Q = Quaternion(Q)

    def __repr__(self):
        """\
        Canonical string representation.

        @return: Canonical string representation.
        @rtype: C{str}
        """
        return 'Rotation(%s)' % str(self.Q)

    __str__ = __repr__

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
        @type R: C{numpy.ndarray}
        @return: Quaternion representation of the rotation.
        @rtype: L{Quaternion}
        """
        u = max([(abs(R[i][i]), i) for i in range(3)])[1]
        v, w = (u + 1) % 3, (u + 2) % 3
        r = sqrt(1.0 + R[u][u] - R[v][v] - R[w][w])
        if r == 0:
            return Quaternion(1, (0, 0, 0))
        Qa = (R[w][v] - R[v][w]) / (2.0 * r)
        Qv = [0.0] * 3
        Qv[u] = r / 2.0
        Qv[v] = (R[u][v] + R[v][u]) / (2.0 * r)
        Qv[w] = (R[w][u] + R[u][w]) / (2.0 * r)
        return Quaternion((Qa, Point(Qv)))
    
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
        @rtype: L{Quaternion}
        """
        if not isinstance(axis, Point):
            axis = Point(axis)
        return Quaternion((cos(theta / 2.0), sin(theta / 2.0) * axis.normal))

    @staticmethod
    def from_euler(convention, angles):
        """\
        TODO
        """
        def qterm(index):
            bin = lambda x: [(x >> 2) % 2, (x >> 1) % 2, x % 2]
            return copysign(1, index) * reduce(lambda a, b: a * b,
                [(bit and sin or cos)(angles[i] / 2.0) \
                for i, bit in enumerate(bin(abs(index)))])
        # TODO: can these be generated from the convention?
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
            return Quaternion((a, -b, -c, -d))
        else:
            return Quaternion((-a, b, c, d))

    def to_rotation_matrix(self):
        """\
        Return the rotation matrix representation from the internal quaternion
        representation.

        @return: Rotation matrix.
        @rtype: C{numpy.ndarray}
        """
        a, b, c, d = self.Q.a, self.Q.b, self.Q.c, self.Q.d
        R = numpy.ndarray((3, 3))
        R[0][0] = a ** 2 + b ** 2 - c ** 2 - d ** 2
        R[0][1] = 2.0 * b * c - 2.0 * a * d
        R[0][2] = 2.0 * b * d + 2.0 * a * c
        R[1][0] = 2.0 * b * c + 2.0 * a * d
        R[1][1] = a ** 2 - b ** 2 + c ** 2 - d ** 2
        R[1][2] = 2.0 * c * d - 2.0 * a * b
        R[2][0] = 2.0 * b * d - 2.0 * a * c
        R[2][1] = 2.0 * c * d + 2.0 * a * b
        R[2][2] = a ** 2 - b ** 2 - c ** 2 + d ** 2
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
        a, b, c, d = self.Q.a, self.Q.b, self.Q.c, self.Q.d
        theta = atan2(2.0 * (c * d - a * b), 1.0 - 2.0 * (b ** 2 + c ** 2))
        phi = -asin(2.0 * (a * c + d * b))
        psi = atan2(2.0 * (b * c - a * d), 1.0 - 2.0 * (c ** 2 + d ** 2))
        return (theta, phi, psi)


class Pose(object):
    """\
    Pose (rigid 3D Euclidean transformation) class.
    """
    def __init__(self, T=None, R=None):
        """\
        Constructor.

        @param T: The 3-element translation vector.
        @type T: L{Point}
        @param R: The 3x3 rotation matrix.
        @type R: L{Rotation}
        """
        if isinstance(T, Point):
            self.T = T
        elif T is None:
            self.T = Point()
        else:
            raise TypeError("translation vector must be a Point or None")
        if isinstance(R, Rotation):
            self.R = R
        else:
            self.R = Rotation(R)

    def __add__(self, other):
        """\
        Pose composition: PB(PA(x)) = (PA + PB)(x).

        @param other: The other pose transformation.
        @type other: L{Pose}
        @return: Composed transformation.
        @rtype: L{Pose}
        """
        if not isinstance(other, Pose):
            raise TypeError('argument must be a Pose')
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
        Pose inversion.

        @return: Inverted pose.
        @rtype: L{Pose}
        """
        return Pose(-(-self.R).rotate(self.T), -self.R)

    def __str__(self):
        """\
        String representation, display T and R.

        @return: String representations of T and R.
        @rtype: C{str}
        """
        return 'Pose(%s, %s)' % (self.T, self.R)

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
        return self.map_translate(self.map_rotate(p))

    def map_rotate(self, p):
        """\
        Rotation component of point/vector mapping.

        @param p: The point/vector to rotate.
        @type p: L{Point}
        @return: The rotated point/vector.
        @rtype: L{Point}
        """
        if not isinstance(p, Point):
            p = Point(p)
        q = self.R.rotate(p)
        if isinstance(p, DirectionalPoint):
            unit = Pose(None, self.R).map(p.direction_unit)
            try:
                rho = acos(unit.z)
            except ValueError:
                if unit.z > 0:
                    rho = 0.0
                else:
                    rho = pi
            eta = atan2(unit.y, unit.x)
            return DirectionalPoint(tuple(q) + (rho, eta))
        else:
            return Point(q)

    def map_translate(self, p):
        """\
        Translation component of point/vector mapping.

        @param p: The point/vector to translate.
        @type p: L{Point}
        @return: The translated point/vector.
        @rtype: L{Point}
        """
        if not isinstance(p, Point):
            p = Point(p)
        q = p + self.T
        return q


class Posable(object):
    """\
    Base class for posable objects.
    """
    def __init__(self, pose=Pose(), mount=None, config=None):
        """\
        Constructor.

        @param pose: The pose of the object (optional).
        @type pose: L{Pose}
        @param mount: The mount of the object (optional).
        @type mount: L{Posable}
        @param config: The configuration of the object (optional).
        @type config: C{object}
        """
        if self.__class__ is Posable:
            raise NotImplementedError("cannot directly instantiate Posable")
        self._pose = pose
        self.mount = mount
        self.config = config
        self.vis = None

    @property
    def pose(self):
        """\
        The pose of the object.
        """
        if self.mount:
            return self._pose + self.mount.mount_pose()
        else:
            return self._pose

    @pose.setter
    def pose(self, value):
        """\
        Set the pose of the object.
        """
        # TODO: handle += and the like?
        self._pose = value

    def mount_pose(self):
        """\
        Return the overall pose transformation to the mount end (unless
        otherwise specified, this will simply place the mounted object into
        this object's coordinate system).

        @return: The overall pose.
        @rtype: L{Pose}
        """
        return self.pose

    def intersection(self, pa, pb):
        """\
        Return the 3D point of intersection (if any) of the line segment
        between the two specified points and this object.

        @param pa: The first vertex of the line segment.
        @type pa: L{Point}
        @param pb: The second vertex of the line segment.
        @type pb: L{Point}
        @return: The point of intersection with the object.
        @rtype: L{Point}
        """
        return None

    def visualize(self, scale=1.0, color=(1, 1, 1), opacity=1.0):
        """\
        Visualize the object.

        @param scale: The scale of the visualization (optional).
        @type scale: C{float}
        @param color: The color of the visualization (optional).
        @type color: C{tuple}
        @param opacity: The opacity of the visualization (optional).
        @type opacity: C{float}
        @return: True if visualization was initialized for the first time.
        @rtype: C{bool}
        """
        if not visual:
            raise VisualizationError("visual module not loaded")
        try:
            self.update_visualization()
        except VisualizationError:
            self.vis = VisualizationObject(self, properties={'scale': scale,
                'color': color, 'opacity': opacity})
            if self.__class__.visualize == Posable.visualize:
                self.update_visualization()
            return True
        return False

    def update_visualization(self):
        """\
        Update the visualization.
        """
        if not self.vis:
            raise VisualizationError("object has not yet been visualized")
        self.vis.transform(self.pose)


class Plane(Posable):
    """\
    Plane segment (2D subspace of 3D space) class.
    """
    def __init__(self, pose=None, mount=None, **kwargs):
        """\
        Constructor.

        @param pose: The pose of the plane normal (from z-hat).
        @type pose: L{Pose}
        """
        Posable.__init__(self, pose, mount)
        if pose is None:
            dims = []
            try:
                for key in ['x', 'y', 'z']:
                    try:
                        if len(kwargs[key]) == 2:
                            dims.append(key)
                    except TypeError:
                        pass
            except KeyError:
                raise ValueError("pose or full dimensions must be supplied")
            if dims == ['x', 'y']:
                self.pose = Pose(T=Point((0.0, 0.0, kwargs['z'])))
            elif dims == ['x', 'z']:
                self.pose = Pose(T=Point((0.0, kwargs['y'], 0.0)),
                                 R=Rotation(Rotation.from_euler('zyx',
                                 (-pi / 2.0, 0, 0))))
            elif dims == ['y', 'z']:
                self.pose = Pose(T=Point((kwargs['x'], 0.0, 0.0)),
                                 R=Rotation(Rotation.from_euler('zyx',
                                 (0, -pi / 2.0, -pi / 2.0))))
            self.x = (float(min(kwargs[dims[0]])), float(max(kwargs[dims[0]])))
            self.y = (float(min(kwargs[dims[1]])), float(max(kwargs[dims[1]])))
        else:
            try:
                self.x = (float(min(kwargs['x'])), float(max(kwargs['x'])))
                self.y = (float(min(kwargs['y'])), float(max(kwargs['y'])))
            except KeyError:
                self.x, self.y = None, None

    @property
    def center(self):
        """\
        Return the absolute 3D point at the center of this plane segment.

        @rtype: L{Point}
        """
        if self.x is None or self.y is None:
            return None
        return self.pose.map(Point(((self.x[1] - self.x[0]) / 2.0 + self.x[0],
            (self.y[1] - self.y[0]) / 2.0 + self.y[0], 0)))

    @property
    def corners(self):
        """\
        Return the corners of the plane.

        @return: The corners of the plane.
        @rtype: C{list} of L{Point}
        """
        if self.x is None or self.y is None:
            return None
        return [self.pose.map(Point((x, y, 0.0))) \
            for x in self.x for y in self.y]

    def intersection(self, pa, pb):
        """\
        Return the 3D point of intersection (if any) of the line segment
        between the two specified points and this plane.

        @param pa: The first vertex of the line segment.
        @type pa: L{Point}
        @param pb: The second vertex of the line segment.
        @type pb: L{Point}
        @return: The point of intersection with the plane.
        @rtype: L{Point}
        """
        pa = (-self.pose).map(pa)
        pb = (-self.pose).map(pb)
        M = numpy.array([[pa.x - pb.x, 1.0, 0.0],
                         [pa.y - pb.y, 0.0, 1.0],
                         [pa.z - pb.z, 0.0, 0.0]])
        t = numpy.dot(numpy.linalg.inv(M), pa.array)[0][0]
        if t < 0 or t > 1:
            return None
        pr = pa + t * (pb - pa)
        if pr.x < self.x[0] or pr.x > self.x[1] \
        or pr.y < self.y[0] or pr.y > self.y[1]:
            return None
        return self.pose.map(pr)

    def visualize(self, scale=1.0, color=(1, 1, 1), opacity=1.0):
        """\
        Visualize the plane.

        @param scale: The scale of the visualization (optional).
        @type scale: C{float}
        @param color: The color of the visualization (optional).
        @type color: C{tuple}
        @param opacity: The opacity of the visualization (optional).
        @type opacity: C{float}
        @return: True if visualization was initialized for the first time.
        @rtype: C{bool}
        """
        if self.x is None or self.y is None:
            raise ValueError('cannot plot an infinite plane')
        if Posable.visualize(self, scale=scale, color=color, opacity=opacity): 
            self.vis.add('plane', visual.box(frame=self.vis,
                pos=(0, 0, 0), width=(scale / 30.0),
                material=visual.materials.wood))
            self.update_visualization()
            return True
        else:
            return False
        
    def update_visualization(self):
        """\
        Update the visualization.
        """
        if not self.vis:
            raise VisualizationError('object has not yet been visualized')
        self.vis.transform(Pose(self.center, self.pose.R))
        self.vis.members['plane'].length = self.x[1] - self.x[0]
        self.vis.members['plane'].height = self.y[1] - self.y[0]
        self.vis.members['plane'].color = self.vis.properties['color']
        self.vis.members['plane'].opacity = self.vis.properties['opacity']


def pointrange(xrange, yrange, zrange, step, ddiv=None):
    """\
    Generate discrete (directional) points in a range.

    @param xrange: The range in the x direction.
    @type xrange: C{tuple} of C{float}
    @param yrange: The range in the y direction.
    @type yrange: C{tuple} of C{float}
    @param zrange: The range in the z direction.
    @type zrange: C{tuple} of C{float}
    @param step: The resolution of the discrete point set.
    @type step: C{float}
    @param ddiv: The fraction of pi for discrete direction angles.
    @type ddiv: C{int}
    """
    x, y, z = xrange[0], yrange[0], zrange[0]
    while x <= xrange[1]:
        while y <= yrange[1]:
            while z <= zrange[1]:
                if ddiv:
                    rho, eta = 0.0, 0.0
                    while rho <= pi:
                        if rho == 0.0 or rho == pi:
                            yield DirectionalPoint((x, y, z, rho, 0.0))
                        else:
                            while eta < 2 * pi:
                                yield DirectionalPoint((x, y, z, rho, eta))
                                eta += pi / ddiv
                        rho += pi / ddiv
                else:
                    yield Point((x, y, z))
                z += step
            z = zrange[0]
            y += step
        y = yrange[0]
        x += step
