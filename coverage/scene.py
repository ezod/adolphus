"""\
Scene model module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from numpy import arange
from math import pi
from fuzz import RealRange, FuzzySet, FuzzyElement

from coverage.geometry import Point, DirectionalPoint, Pose

try:
    import visual
    VIS = True
except ImportError:
    VIS = False

class Scene(object):
    """\
    Discrete spatial-directional range with occlusion class.
    """
    def __init__(self, x, y, z, pstep, dstep):
        """\
        Constructor.

        @param x: Range in the x direction.
        @type x: L{fuzz.RealRange}
        @param y: Range in the y direction.
        @type y: L{fuzz.RealRange}
        @param z: Range in the z direction.
        @type z: L{fuzz.RealRange}
        @param pstep: Spatial resolution.
        @type pstep: C{float}
        @param dstep: Angular resolution.
        @type dstep: C{float}
        """
        self.x = RealRange(x)
        self.y = RealRange(y)
        self.z = RealRange(z)
        self.pstep = pstep
        self.dstep = dstep
        self.opaque = set()
        self.D = FuzzySet()

    def make_opaque(self, x, y, z):
        """\
        Add a range to the set of opaque voxels.

        @param x: Range in the x direction.
        @type x: L{fuzz.RealRange}
        @param y: Range in the y direction.
        @type y: L{fuzz.RealRange}
        @param z: Range in the z direction.
        @type z: L{fuzz.RealRange}
        """
        for xi in arange(x[0], x[1], self.pstep):
            for yi in arange(y[0], y[1], self.pstep):
                for zi in arange(z[0], z[1], self.pstep):
                    self.opaque.add(Point(xi, yi, zi))
                    try:
                        self.D.remove(Point(xi, yi, zi))
                    except KeyError:
                        pass
                    for rho in arange(0., pi + self.dstep, self.dstep):
                        for eta in arange(0., 2 * pi, self.dstep):
                            try:
                                self.D.remove(DirectionalPoint( \
                                              xi, yi, zi, rho, eta))
                            except KeyError:
                                pass

    def make_desired(self, x, y, z, d = 'all', mu = 1.0):
        """\
        Add a range to the desired coverage set.

        @param x: Range in the x direction.
        @type x: L{fuzz.RealRange}
        @param y: Range in the y direction.
        @type y: L{fuzz.RealRange}
        @param z: Range in the z direction.
        @type z: L{fuzz.RealRange}
        @param d: Optional direction (will use all dstep otherwise).
        @type d: C{tuple} of L{geometry.Angle}
        @param mu: Membership value to assign to this range.
        @type mu: C{float}
        """
        for dpoint in self._generate_points_specified(x, y, z, d):
            if not Point(dpoint.x, dpoint.y, dpoint.z) in self.opaque:
                self.D.add(FuzzyElement(dpoint, mu))

    def generate_points(self, d = 'all'):
        """\
        Discrete directional point generator for the entire scene.

        @return: The next directional point in the range.
        @rtype: L{geometry.DirectionalPoint}
        @param d: Optional direction (will use all dstep otherwise).
        @type d: C{tuple} of L{geometry.Angle}
        """
        return self._generate_points_specified(self.x, self.y, self.z, d = d)

    def _generate_points_specified(self, x, y, z, d = 'all'):
        """\
        General discrete directional point generator.

        @param x: Range in the x direction.
        @type x: L{fuzz.RealRange}
        @param y: Range in the y direction.
        @type y: L{fuzz.RealRange}
        @param z: Range in the z direction.
        @type z: L{fuzz.RealRange}
        @param d: Optional direction (will use all dstep otherwise).
        @type d: C{tuple} of L{geometry.Angle}
        @return: The next directional point in the range.
        @rtype: L{geometry.DirectionalPoint}
        """
        for xi in arange(x[0], x[1], self.pstep):
            for yi in arange(y[0], y[1], self.pstep):
                for zi in arange(z[0], z[1], self.pstep):
                    if Point(xi, yi, zi) in self.opaque:
                        continue
                    if not d:
                        yield Point(xi, yi, zi)
                        continue
                    if d != 'all':
                        yield DirectionalPoint(xi, yi, zi, d[0], d[1])
                        continue
                    for rho in arange(0., pi + self.dstep, self.dstep):
                        if rho in [0, pi]:
                            yield DirectionalPoint(xi, yi, zi, rho, 0)
                            continue
                        for eta in arange(0., 2 * pi, self.dstep):
                            yield DirectionalPoint(xi, yi, zi, rho, eta)

    def occluded(self, p, cam = Point()):
        """\
        Find the 3D Bresenham voxel traversal of the line segment between the
        specified point and the principal point of a camera, and return whether
        the point is occluded from the camera viewpoint by opaque voxels.

        @param p: The point to test.
        @type p: L{geometry.Point}
        @param cam: The camera principal point (translation component of pose). 
        @type cam: L{geometry.Point}
        @return: True if occluded, false otherwise.
        @rtype: C{bool}
        """
        P1 = Point(round(p.x / self.pstep), \
                   round(p.y / self.pstep), \
                   round(p.z / self.pstep))
        P2 = Point(round(cam.x / self.pstep), \
                   round(cam.y / self.pstep), \
                   round(cam.z / self.pstep))
        d = [P2[i] - P1[i] for i in range(3)]
        a = [abs(d[i] ) * 2 for i in range(3)]
        s = [d[i] >= 0 and 1 or -1 for i in range(3)]
        P = Point(P1.x, P1.y, P1.z)
        if(a[0] >= max(a[1], a[2])):
            x, y, z = 0, 1, 2
        elif(a[1] >= max(a[0], a[2])):
            x, y, z = 1, 2, 0
        else:
            x, y, z = 2, 0, 1
        yd = a[y] - a[x] / 2
        zd = a[z] - a[x] / 2
        while True:
            if(P[x] == P2[x]):
                break
            if(yd >= 0):
                P[y] += s[y]
                yd -= a[x]
            if(zd >= 0):
                P[z] += s[z]
                zd -= a[x]
            P[x] += s[x]
            yd += a[y]
            zd += a[z]
            if self.voxel(P.x, P.y, P.z) in self.opaque:
                return True
        return False
 
    def voxel(self, x, y, z):
        """\
        TODO
        """
        return Point(x * self.pstep, y * self.pstep, z * self.pstep)

    def visualize(self, color = (1, 1, 1)):
        """\
        Visualize the opaque scene objects.

        @param color: The color of opaque scene objects.
        @type color: C{tuple}
        """
        if not VIS:
            raise NotImplementedError( "visual module not loaded" )
        for point in self.opaque:
            visual.box(pos = point.tuple, color = color, length = self.pstep,
                       height = self.pstep, width = self.pstep)
