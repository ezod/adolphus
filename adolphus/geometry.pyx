# cython: profile=True

"""\
Geometry module. Contains point (vector) and pose transformation classes, and
geometric descriptor functions for features.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from math import pi, sqrt, sin, cos, asin, acos, atan2, copysign
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
        return tuple.__new__(cls, iterable)

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
        Scalar multiplication (if p is a scalar) or dot product (if p is a
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
            raise VisualizationError('visual module not loaded')
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
            Point(self[0] * q[1] + q[0] * self[1] + self[1] ** q[1])))

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
    def __init__(self, Q=Quaternion()):
        """\
        Constructor.
        """
        self.Q = Q

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
        TODO

        @param convention: The convention to use (e.g. 'zyx').
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
            return Rotation(Quaternion((a, Point((-b, -c, -d)))))
        else:
            return Rotation(Quaternion((-a, Point((b, c, d)))))

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
    def __init__(self, T=Point(), R=Rotation()):
        """\
        Constructor.

        @param T: The 3-element translation vector.
        @type T: L{Point}
        @param R: The 3x3 rotation matrix.
        @type R: L{Rotation}
        """
        self.T = T
        self.R = R

    def __add__(self, other):
        """\
        Pose composition: PB(PA(x)) = (PA + PB)(x).

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
            raise NotImplementedError('cannot directly instantiate Posable')
        self._pose = pose
        self.mount = mount
        self.config = config
        self.vis = None

    @property
    def pose_(self):
        """\
        The pose of the object.
        """
        if self.mount:
            return self._pose + self.mount.mount_pose()
        else:
            return self._pose

    @pose_.setter
    def pose(self, value):
        """\
        Set the pose of the object.
        """
        # TODO: handle += and the like?
        self._pose = value

    del pose_

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
            raise VisualizationError('visual module not loaded')
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
            raise VisualizationError('object has not yet been visualized')
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
                raise ValueError('pose or full dimensions must be supplied')
            if dims == ['x', 'y']:
                self.pose = Pose(T=Point((0.0, 0.0, kwargs['z'])))
            elif dims == ['x', 'z']:
                self.pose = Pose(T=Point((0.0, kwargs['y'], 0.0)),
                                 R=Rotation.from_euler('zyx',
                                    (-pi / 2.0, 0, 0)))
            elif dims == ['y', 'z']:
                self.pose = Pose(T=Point((kwargs['x'], 0.0, 0.0)),
                                 R=Rotation.from_euler('zyx',
                                    (0, -pi / 2.0, -pi / 2.0)))
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
        try:
            return self._corners
        except AttributeError:
            if self.x is None or self.y is None:
                return None
            self._corners = [self.pose.map(Point((x, y, 0.0))) \
                for x in self.x for y in self.y]
            return self._corners

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
        c = self.corners
        M = numpy.array([[pa.x - pb.x, c[1].x - c[0].x, c[2].x - c[0].x],
                         [pa.y - pb.y, c[1].y - c[0].y, c[2].y - c[0].y],
                         [pa.z - pb.z, c[1].z - c[0].z, c[2].z - c[0].z]])
        t = numpy.dot(numpy.linalg.inv(M), pa.array)[0][0]
        if t < 0 or t > 1:
            return None
        pr = pa + t * (pb - pa)
        spr = (-self.pose).map(pr)
        if spr.x < self.x[0] or spr.x > self.x[1] \
        or spr.y < self.y[0] or spr.y > self.y[1]:
            return None
        return pr

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
