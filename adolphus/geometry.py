"""\
Geometry module. Contains point (vector) and pose transformation classes, and
geometric descriptor functions for features.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from math import pi, sqrt, sin, cos, asin, acos, atan, atan2
from numbers import Number
import numpy

try:
    import visual
    VIS = True
except ImportError:
    VIS = False


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


class Point(object):
    """\
    3D point (vector) class.
    """
    def __init__(self, *args):
        """\
        Constructor.
        """
        if not args or len(args) == 1 and args[0] is None:
            self.x, self.y, self.z = [0.0] * 3
        elif len(args) == 1 and isinstance(args[0], Point):
            self.x, self.y, self.z = args[0].tuple[:3]
        elif len(args) == 1 and isinstance(args[0], tuple):
            self.x, self.y, self.z = args[0][:3]
        elif len(args) == 1 and isinstance(args[0], numpy.ndarray):
            self.x, self.y, self.z = [args[0][i][0] for i in range(3)]
        elif len(args) >= 3 and isinstance(args[0], Number) \
        and isinstance(args[1], Number) and isinstance(args[2], Number):
            self.x, self.y, self.z = [float(args[i]) for i in range(3)]
        else:
            raise TypeError("incompatible type in initializer")
    
    def __hash__(self):
        """\
        Hash function.
        """
        return self.x + self.y + self.z

    def __eq__(self, p):
        """\
        Equality function.

        @param p: The other point.
        @type p: L{Point}
        """
        try:
            return self.x == p.x and self.y == p.y and self.z == p.z
        except AttributeError:
            return False

    def __neq__(self, p):
        """\
        Inequality function.

        @param p: The other point.
        @type p: L{Point}
        """
        return not self.__eq__(p)

    def __getitem__(self, i):
        """\
        Return x, y, and z via numerical indexing.
    
        @param i: The numerical index.
        @type i: C{int}
        @return: The indexed coordinate.
        @rtype: C{float}
        """
        if i == 0:
            return self.x
        elif i == 1:
            return self.y
        elif i == 2:
            return self.z

    def __setitem__(self, i, value):
        """\
        Set x, y, and z via numerical indexing.
    
        @param i: The numerical index.
        @type i: C{int}
        @param value: The new coordinate value.
        @type value: C{float}
        """
        if i == 0:
            self.x = value
        elif i == 1:
            self.y = value
        elif i == 2:
            self.z = value

    def __add__(self, p):
        """\
        Vector addition.

        @param p: The operand vector.
        @type p: L{Point}
        @return: Result vector.
        @rtype: L{Point}
        """
        return Point(self.x + p.x, self.y + p.y, self.z + p.z)

    def __sub__(self, p):
        """\
        Vector subtraction.

        @param p: The operand vector.
        @type p: L{Point}
        @return: Result vector.
        @rtype: L{Point}
        """
        return Point(self.x - p.x, self.y - p.y, self.z - p.z)

    def __mul__(self, p):
        """\
        Scalar multiplication (if p is a scalar) or dot product (if p is a
        vector).

        @param p: The operand scalar or vector.
        @type p: C{float} or L{Point}
        @return: Result vector.
        @rtype: L{Point}
        """
        if isinstance(p, Point):
            return (self.x * p.x + self.y * p.y + self.z * p.z)
        else:
            return Point(self.x * p, self.y * p, self.z * p)

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
        return Point(self.x / p, self.y / p, self.z / p)

    def __pow__(self, p):
        """\
        Cross product.

        @param p: The operand vector.
        @type p: L{Point}
        @return: Result vector.
        @rtype: L{Point}
        """
        if not isinstance(p, Point):
            raise TypeError("cross product operand must be a vector")
        return Point(self.y * p.z - self.z * p.y,
                     self.z * p.x - self.x * p.z,
                     self.x * p.y - self.y * p.x)

    def __neg__(self):
        """\
        Negation.

        @return: Result vector.
        @rtype: L{Point}
        """
        return Point(-self.x, -self.y, -self.z)

    def __repr__(self):
        """\
        Canonical string representation.

        @return: Canonical string representation.
        @rtype: C{str}
        """
        return "%s(%f, %f, %f)" % (self.__class__.__name__,
            self.x, self.y, self.z)

    def __str__(self):
        """\
        String representation, displays in a tuple format.

        @return: Vector string.
        @rtype: C{str}
        """
        return "(%f, %f, %f)" % (self.x, self.y, self.z)

    @property
    def tuple(self):
        """
        Return a tuple of this vector.

        @rtype: C{tuple}
        """
        return (self.x, self.y, self.z)

    @property
    def array(self):
        """\
        Return a NumPy array of this vector.

        @rtype: C{numpy.ndarray}
        """
        return numpy.array([[self.x], [self.y], [self.z]])

    @property
    def magnitude(self):
        """\
        Return the magnitude of this vector.

        @rtype: C{float}
        """
        return sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    @property
    def normal(self):
        """\
        Return a normalized (unit) vector in the direction of this vector.

        @rtype: L{Point}
        """
        m = self.magnitude
        if m == 0:
            raise ValueError("cannot normalize a zero vector")
        return Point(self.x / m, self.y / m, self.z / m)

    def euclidean(self, p):
        """\
        Return the Euclidean distance from this point to another.

        @param p: The other point.
        @type p: L{Point}
        @return: Euclidean distance.
        @rtype: C{float}
        """
        return sqrt((self.x - p.x) ** 2 + (self.y - p.y) ** 2 + \
                    (self.z - p.z) ** 2)

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
        if not VIS:
            raise ImportError("visual module not loaded")
        try:
            self.vis.members['point'].radius = 0.1 * scale
            self.vis.members['point'].color = color
            self.vis.members['point'].opacity = opacity
        except AttributeError:
            self.vis = visual.frame()
            self.vis.point = self
            self.vis.members = {}
            self.vis.members['point'] = visual.sphere(frame=self.vis, 
                radius=(0.1 * scale), color=color, opacity=opacity)
        self.vis.pos = self.tuple


class DirectionalPoint(Point):
    """\
    3D directional point (spatial-directional vector) class.
    """
    def __init__(self, *args):
        """\
        Constructor.
        """
        Point.__init__(self, *args)
        if not args or len(args) == 1 and args[0] is None:
            self.rho, self.eta = [0.0] * 2
        elif len(args) == 1 and isinstance(args[0], DirectionalPoint):
            self.rho, self.eta = args[0].tuple[-2:]
        elif len(args) == 1 and isinstance(args[0], tuple):
            self.rho, self.eta = args[0][-2:]
        elif len(args) == 1 and isinstance(args[0], numpy.ndarray):
            self.rho, self.eta = [args[0][i][0] for i in [3, 4]]
        elif len(args) >= 5 and isinstance(args[3], Number) \
        and isinstance(args[4], Number):
            self.rho, self.eta = [args[i] for i in [3, 4]]
        else:
            raise TypeError("incompatible type in initializer")
        self.rho = Angle(self.rho)
        self.eta = Angle(self.eta)
        self._normalize_direction()

    def __hash__(self):
        """\
        Hash function.
        """
        return self.x + self.y + self.z + self.rho + self.eta

    def __eq__(self, p):
        """\
        Equality function.

        @param p: The other point.
        @type p: L{DirectionalPoint}
        """
        try:
            return self.x == p.x and self.y == p.y and self.z == p.z \
                   and self.rho == p.rho and self.eta == p.eta
        except AttributeError:
            return False

    def _normalize_direction(self):
        """\
        Normalize the inclination angle to within [0,pi), reversing the azimuth
        angle as necesary.
        """
        if self.rho > pi:
            self.rho -= 2. * (self.rho - pi)
            self.eta += pi

    def __getitem__(self, i):
        """\
        Return x, y, z, rho, and eta via numerical indexing.
    
        @param i: The numerical index.
        @type i: C{int}
        @return: The indexed coordinate.
        @rtype: C{float}
        """
        if i < 3:
            return Point.__getitem__(self, i)
        elif i == 3:
            return self.rho
        elif i == 4:
            return self.eta

    def __setitem__(self, i, value):
        """\
        Set x, y, z, rho, and eta via numerical indexing.
    
        @param i: The numerical index.
        @type i: C{int}
        @param value: The new coordinate value.
        @type value: C{float}
        """
        if i < 3:
            Point.__setitem__(self, i, value)
        elif i == 3:
            self.rho = value
        elif i == 4:
            self.eta = value

    def __add__(self, p):
        """\
        Vector addition.

        @param p: The operand vector.
        @type p: L{Point}
        @return: Result vector.
        @rtype: L{DirectionalPoint}
        """
        return DirectionalPoint(self.x + p.x, self.y + p.y, self.z + p.z,
                                self.rho, self.eta)

    def __sub__(self, p):
        """\
        Vector subtraction.

        @param p: The operand vector.
        @type p: L{Point}
        @return: Result vector.
        @rtype: L{DirectionalPoint}
        """
        return DirectionalPoint(self.x - p.x, self.y - p.y, self.z - p.z,
                                self.rho, self.eta)

    def __mul__(self, p):
        """\
        Scalar multiplication (if p is a scalar) or dot product (if p is a
        vector).

        @param p: The operand scalar or vector.
        @type p: C{float} or L{Point}
        @return: Result vector.
        @rtype: L{DirectionalPoint}
        """
        if isinstance(p, Point):
            return (self.x * p.x + self.y * p.y + self.z * p.z)
        else:
            return DirectionalPoint(self.x * p, self.y * p, self.z * p,
                                    self.rho, self.eta)

    def __rmul__(self, p):
        """\
        Scalar multiplication (if p is a scalar) or dot product (if p is a
        vector).

        @param p: The operand scalar or vector.
        @type p: C{float} or L{Point}
        @return: Result vector.
        @rtype: L{DirectionalPoint}
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
        return DirectionalPoint(self.x / p, self.y / p, self.z / p,
                                self.rho, self.eta)

    def __neg__(self):
        """\
        Negation.

        @return: Result vector.
        @rtype: L{DirectionalPoint}
        """
        return DirectionalPoint(-self.x, -self.y, -self.z,
                                self.rho + pi, self.eta)

    def __repr__(self):
        """\
        Canonical string representation.

        @return: Spatial-directional vector string.
        @rtype: C{str}
        """
        return "%s(%f, %f, %f, %f, %f)" % (self.__class__.__name__,
            self.x, self.y, self.z, self.rho, self.eta)

    def __str__(self):
        """\
        String representation, displays in a tuple format.

        @return: Spatial-directional vector string.
        @rtype: C{str}
        """
        return "(%f, %f, %f, %f, %f)" \
            % (self.x, self.y, self.z, self.rho, self.eta)

    @property
    def direction_unit(self):
        """\
        Return a unit vector representation of the direction of this
        directional point.

        @rtype: L{Point}
        """
        return Point(sin(self.rho) * cos(self.eta),
                     sin(self.rho) * sin(self.eta), cos(self.rho))

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
        Point.visualize(self, scale = scale, color = color, opacity = opacity)
        unit = scale * self.direction_unit
        try:
            self.vis.members['dir'].axis = unit.tuple
            self.vis.members['dir'].color = color
            self.vis.members['dir'].opacity = opacity
        except KeyError:
            self.vis.members['dir'] = visual.arrow(frame=self.vis, 
                axis=unit.tuple, color=color, opacity=opacity)


class Plane(object):
    """\
    Plane segment (2D subspace of 3D space) class.
    """
    def __init__(self, pose, x=None, y=None):
        """\
        Constructor.

        @param pose: The pose of the plane normal (from z-hat).
        @type pose: L{Pose}
        @param x: The x-range before transformation (optional).
        @type x: C{tuple} of C{float}
        @param y: The y-range before transformation (optional).
        @type y: C{tuple} of C{float}
        """
        if x is not None or y is not None:
            if not len(x) == 2 or not len(y) == 2:
                raise ValueError("boundaries must consist of two values")
            self.x = (float(min(x)), float(max(x)))
            self.y = (float(min(y)), float(max(y)))
        else:
            self.x, self.y = None, None
        self.pose = pose
        if VIS:
            self.vis_plane = None

    @property
    def center(self):
        """\
        Return the 3D point at the center of this plane segment.

        @rtype: L{Point}
        """
        if self.x is None or self.y is None:
            return None
        return self.pose.map(Point((self.x[1] - self.x[0]) / 2.0 + self.x[0],
                (self.y[1] - self.y[0]) / 2.0 + self.y[0], 0))

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

    def visualize(self, color=(1, 1, 1), opacity=1.0):
        """\
        Plot the directional point in a 3D visual model.

        @param color: The color in which to plot the plane segment.
        @type color: C{tuple}
        @param opacity: The opacity with which to plot the plane segment.
        @type opacity: C{float}
        """
        if not VIS:
            raise ImportError("visual module not loaded")
        if self.x is None or self.y is None:
            raise ValueError("cannot plot an infinite plane")
        try:
            self.vis_plane.pos = self.center.tuple
            self.vis_plane.axis = self.pose.map_rotate(Point(0, 0, 1)).tuple
            self.vis_plane.width = self.x[1] - self.x[0]
            self.vis_plane.height = self.y[1] - self.y[0]
            self.vis_plane.color = color
            self.vis_plane.opacity = opacity
        except AttributeError:
            self.vis_plane = visual.box(pos=self.center.tuple,
                axis=self.pose.map_rotate(Point(0, 0, 1)).tuple,
                width=(self.x[1] - self.x[0]), height=(self.y[1] - self.y[0]),
                length=1, color=color, opacity=opacity)


class Rotation(object):
    """\
    3D Euclidean rotation class. Handles multiple representations of SO(3).
    """
    def __init__(self, *args):
        """\
        Constructor.
        """
        if not args or len(args) == 1 and args[0] is None:
            self.R = numpy.diag([1, 1, 1])
        elif len(args) == 1 and isinstance(args[0], numpy.ndarray):
            self.R = args[0]
        elif len(args) == 2:
            self.R = self.from_axis_angle(Point(args[0]), Angle(args[1]))
        elif len(args) == 3:
            self.R = self.from_euler_xyz(Angle(args[0]), Angle(args[1]),
                Angle(args[2]))
        else:
            raise TypeError("unrecognized initial value format")

    def __str__(self):
        """\
        String representation.

        @return: String representation.
        @rtype: C{str}
        """
        return str(self.R)

    @staticmethod
    def from_axis_angle(axis, theta):
        """\
        Generate the internal rotation matrix representation from an axis and
        angle representation (Rodrigues' formula).

        @param axis: The axis of rotation.
        @type axis: L{Point}
        @param theta: The angle of rotation.
        @type theta: L{Angle}
        """
        axis = axis.normal
        Ax = numpy.array([[0.0, -(axis.z), axis.y],
                          [axis.z, 0.0, -(axis.x)],
                          [-(axis.y), axis.x, 0.0]])
        R = numpy.diag([1.0, 1.0, 1.0]) + sin(theta) * Ax + (1 - cos(theta)) * \
            (axis.array * axis.array.transpose() - numpy.diag([1.0, 1.0, 1.0]))
        return R.transpose()

    @staticmethod
    def from_euler_xyz(theta, phi, psi):
        """\
        Generate the internal rotation matrix representation from three fixed-
        axis (Euler xyz) rotation angles.

        @param vector: The rotation angle vector.
        @type vector: C{tuple} of L{Angle}
        @return: Rotation matrix.
        @rtype: L{numpy.ndarray}
        """
        R = numpy.ndarray((3, 3))
        R[0][0] = cos(phi) * cos(psi)
        R[1][0] = sin(theta) * sin(phi) * cos(psi) - cos(theta) * sin(psi)
        R[2][0] = cos(theta) * sin(phi) * cos(psi) + sin(theta) * sin(psi)
        R[0][1] = cos(phi) * sin(psi)
        R[1][1] = sin(theta) * sin(phi) * sin(psi) + cos(theta) * cos(psi)
        R[2][1] = cos(theta) * sin(phi) * sin(psi) - sin(theta) * cos(psi)
        R[0][2] = -sin(phi)
        R[1][2] = sin(theta) * cos(phi)
        R[2][2] = cos(theta) * cos(phi)
        return R

    def to_axis_angle(self):
        """\
        Return the axis and angle representation from the internal rotation
        matrix representation.

        @return: Axis and angle rotation form.
        @rtype: C{tuple} of L{Point} and L{Angle}
        """
        theta = acos((self.R.trace() - 1.0) / 2.0)
        if theta == 0.0:
            return (Point(1, 0, 0), theta)
        axis = -Point(tuple([(1.0 / 2.0 * sin(theta)) * Ri for Ri in \
            [self.R[pair[0]][pair[1]] - self.R[pair[1]][pair[0]] for pair in \
            [((i + 2) % 3, (i + 1) % 3) for i in range(3)]]]))
        return (axis, theta)

    def to_euler_xyz(self):
        """\
        Return three fixed-axis (Euler xyz) rotation angles from the internal
        rotation matrix representation.

        @return Three fixed-axis (Euler xyz) angle rotation form.
        @rtype: C{tuple} of L{Angle}
        """
        phi = Angle(asin(-1.0 * self.R[0][2]))
        sign = cos(phi) / abs(cos(phi))
        theta = Angle(atan(self.R[1][2] / self.R[2][2]))
        if sign * self.R[2][2] < 0:
            theta += pi
        psi = Angle(atan(self.R[0][1] / self.R[0][0]))
        if sign * self.R[0][0] < 0:
            psi += pi
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
        @type R: C{numpy.ndarray}
        """
        if isinstance(T, Point):
            self.T = T
        elif T is None:
            self.T = Point(0, 0, 0)
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
            raise TypeError("argument must be a Pose")
        Tnew = Point(numpy.dot(other.R.R, self.T.array)) + other.T
        Rnew = numpy.dot(other.R.R, self.R.R)
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
        Rinv = self.R.R.transpose()
        Tinv = -Point(numpy.dot(Rinv, self.T.array))
        return Pose(Tinv, Rinv)

    def __str__(self):
        """\
        String representation, display T and R.

        @return: String representations of T and R.
        @rtype: C{str}
        """
        return str(self.T) + "\n" + str(self.R)

    @property
    def nonzero(self):
        """\
        Check if this pose transformation has any effect.

        @rtype: C{bool}
        """
        if sum(self.T.tuple) == 0 \
        and (self.R.R - numpy.diag((1.0, 1.0, 1.0))).any() == 0:
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
        q = numpy.dot(self.R.R, p.array)
        if isinstance(p, DirectionalPoint):
            unit = Pose(None, self.R.R).map(p.direction_unit)
            rho = acos(unit.z)
            eta = atan2(unit.y, unit.x)
            return DirectionalPoint(tuple([q[i][0] for i in range(3)]) \
                + (rho, eta))
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
        q = p + self.T
        return q
