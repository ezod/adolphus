"""\
Posable TODO

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import pyximport; pyximport.install()

from math import pi
from number import Number
import numpy
import yaml

from geometry import Point, Pose, Rotation
from visualization import Visualizable


class Posable(object):
    """\
    Posable abstract base class.
    """
    def __init__(self, pose=Pose(), mount=None, config=None, filename=None):
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
        self.planes = set()
        if filename:
            definition = yaml.load(open(filename))
            for plane in definition['planes']:
                self.planes.add(Plane(**plane))

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
        intersections = [plane.intersection(pa, pb) for plane in self.planes]
        for intersection in intersections:
            if intersection:
                return intersection
        return None
        

class Plane(Posable, Visualizable):
    """\
    Plane segment (2D subspace of 3D space) class.
    """
    def __init__(self, pose=None, mount=None, x=None, y=None, z=None):
        """\
        Constructor.

        @param pose: The pose of the plane normal (from z-hat).
        @type pose: L{Pose}
        """
        Posable.__init__(self, pose, mount)
        if pose is None:
            if isinstance(z, Number):
                self.pose = Pose(T=Point((0.0, 0.0, z)))
                self.x, self.y = [float(n) for n in x], [float(n) for n in y]
            elif isinstance(y, Number):
                self.pose = Pose(T=Point((0.0, y, 0.0)), R=Rotation.from_euler(\
                    'zyx', (-pi / 2.0, 0, 0)))
                self.x, self.y = [float(n) for n in x], [float(n) for n in z]
            else:
                self.pose = Pose(T=Point((x, 0.0, 0.0)), R=Rotation.from_euler(\
                    'zyx', (0, -pi / 2.0, -pi / 2.0)))
                self.x, self.y = [float(n) for n in y], [float(n) for n in z]
        else:
            self.x, self.y = [float(n) for n in x], [float(n) for n in y]
        definition = {'primitives': [{'type':       'box',
                                      'length':     self.x[1] - self.x[0],
                                      'height':     self.y[1] - self.y[0],
                                      'width':      0.03,
                                      'material':   'wood'}]}
        Visualizable.__init__(self, [definition])

    @property
    def center(self):
        """\
        Return the absolute 3D point at the center of this plane segment.

        @rtype: L{Point}
        """
        try:
            return self._center
        except AttributeError:
            self._center = self.pose.map(Point(((self.x[1] - self.x[0]) / 2.0 \
                + self.x[0], (self.y[1] - self.y[0]) / 2.0 + self.y[0], 0)))
            return self._center

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
        M = numpy.array([[pa[0] - pb[0], c[1][0] - c[0][0], c[2][0] - c[0][0]],
                         [pa[1] - pb[1], c[1][1] - c[0][1], c[2][1] - c[0][1]],
                         [pa[2] - pb[2], c[1][2] - c[0][2], c[2][2] - c[0][2]]])
        t = numpy.dot(numpy.linalg.inv(M), pa.array)[0][0]
        if t < 0 or t > 1:
            return None
        pr = pa + t * (pb - pa)
        spr = (-self.pose).map(pr)
        if spr[0] < self.x[0] or spr[0] > self.x[1] \
        or spr[1] < self.y[0] or spr[1] > self.y[1]:
            return None
        return pr

    def update_visualization(self):
        """\
        Update the visualization. Use a slightly different pose definition to
        account for possible off-center true center.
        """
        for display in self.actuals.keys():
            for sprite in self.actuals[display]:
                sprite.transform(Pose(self.center, self.pose.R))
                sprite.opacity = self.opacity
