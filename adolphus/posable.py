"""\
Posable objects module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from math import pi
from numbers import Number
import numpy

import cython
from .geometry import Point, Pose, Rotation
from .visualization import Visualizable


class Posable(object):
    """\
    Posable base class.
    """
    def __init__(self, pose=Pose(), mount_pose=Pose(), mount=None):
        """\
        Constructor.

        @param pose: The pose of the object (optional).
        @type pose: L{Pose}
        @param mount_pose: The transformation to the mounting end (optional).
        @type mount_pose: L{Pose}
        @param mount: The mount of the object (optional).
        @type mount: L{Posable}
        """
        self._pose = pose
        self._mount_pose = mount_pose
        self.mount = mount

    @property
    def pose(self):
        """\
        The pose of the object.
        """
        if self.mount:
            return self._pose + self.mount.mount_pose()
        else:
            return self._pose

    def set_absolute_pose(self, pose):
        """\
        Set the absolute (world frame) pose of the object.

        @param pose: The absolute pose to set.
        @type pose: L{Pose}
        """
        self._pose = pose - self.mount.mount_pose()

    def set_relative_pose(self, pose):
        """\
        Set the relative (mounted) pose of the object.

        @param pose: The relative pose to set.
        @type pose: L{Pose}
        """
        self._pose = pose

    def mount_pose(self):
        """\
        Return the overall pose transformation to the mount end (unless
        otherwise specified, this will simply place the mounted object into
        this object's coordinate system).

        @return: The overall pose.
        @rtype: L{Pose}
        """
        return self._mount_pose + self.pose


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
        Posable.__init__(self, pose=pose, mount=mount)
        if pose is None:
            if isinstance(z, Number):
                self.set_relative_pose(Pose(T=Point((0.0, 0.0, z))))
                self.x, self.y = [float(n) for n in x], [float(n) for n in y]
            elif isinstance(y, Number):
                self.set_relative_pose(Pose(T=Point((0.0, y, 0.0)),
                    R=Rotation.from_euler('zyx', (-pi / 2.0, 0, 0))))
                self.x, self.y = [float(n) for n in x], [float(n) for n in z]
            else:
                self.set_relative_pose(Pose(T=Point((x, 0.0, 0.0)),
                    R=Rotation.from_euler('zyx', (0, -pi / 2.0, -pi / 2.0))))
                self.x, self.y = [float(n) for n in y], [float(n) for n in z]
        else:
            self.x, self.y = [float(n) for n in x], [float(n) for n in y]
        primitives = [{'type':       'box',
                       'length':     self.x[1] - self.x[0],
                       'height':     self.y[1] - self.y[0],
                       'width':      1,
                       'material':   'wood'}]
        Visualizable.__init__(self, primitives)

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
        try:
            t = numpy.dot(numpy.linalg.inv(M), (pa - c[0]).array)[0][0]
        except numpy.linalg.linalg.LinAlgError:
            return None
        if t <= 0 or t >= 1:
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
        for display in self.actuals:
            self.actuals[display].transform(Pose(self.center, self.pose.R))
            self.actuals[display].opacity = self.opacity


class SceneObject(Posable, Visualizable):
    """\
    Sprite-based scene object.
    """
    def __init__(self, pose=Pose(), mount_pose=Pose(), mount=None,
                 primitives=[], planes=[]):
        """\
        Constructor.

        @param pose: The pose of the object (optional).
        @type pose: L{Pose}
        @param mount_pose: The transformation to the mounting end (optional).
        @type mount_pose: L{Pose}
        @param mount: The mount of the object (optional).
        @type mount: L{Posable}
        @param primitives: A list of sprite primitive sets (optional).
        @type primitives: C{list} of C{dict}
        @param planes: The opaque planes associated with this object (optional).
        @type planes: C{list} of L{Plane}
        """
        Posable.__init__(self, pose, mount_pose, mount)
        Visualizable.__init__(self, primitives)
        self.planes = set()
        for plane in planes:
            plane['mount'] = self
            self.planes.add(Plane(**plane))
        self._planes_view = False

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
        
    def toggle_planes(self):
        """\
        Toggle display of occluding planes in the visualization.
        """
        if self._planes_view:
            self.opacity = 1.0
            self.update_visualization()
            for plane in self.planes:
                plane.visible = False
        else:
            self.opacity = 0.1
            self.update_visualization()
            for plane in self.planes:
                plane.visualize()
                plane.visible = True
        self._planes_view = not self._planes_view


class Robot(Posable):
    """\
    Sprite-based robot.
    """
    def __init__(self, pose=Pose(), mount=None, pieces=[], config=None):
        super(Robot, self).__init__(pose, mount)
        self.pieces = []
        nextpose = Pose()
        for i, piece in enumerate(pieces):
            offset = piece['offset']
            try:
                mount = self.pieces[i - 1]
            except IndexError:
                mount = self.mount
            try:
                planes = piece['planes']
            except KeyError:
                planes = []
            try:
                primitives = piece['primitives']
            except KeyError:
                primitives = []
            self.pieces.append(SceneObject(nextpose, offset, mount, primitives,
                planes))
            nextpose = self.generate_joint_pose(piece['joint'])
        self._mount_pose = nextpose
        self.joints = [piece['joint'] for piece in pieces]
        self._config = [joint['home'] for joint in self.joints]
        if config:
            self.config = config
        self._visible = False

    @property
    def config(self):
        """\
        The configuration of the robot.

        @rtype: C{list} of C{float}
        """
        return self._config

    @config.setter
    def config(self, value):
        """\
        Set the configuration of the robot.

        @param value: The configuration of the robot.
        @type value: C{list} of C{float}
        """
        if not len(value) == len(self.pieces):
            raise ValueError('incorrect configuration length')
        for i, position in enumerate(value):
            try:
                self.pieces[i + 1].set_relative_pose(self.generate_joint_pose(\
                    self.joints[i], position))
            except IndexError:
                self._mount_pose = \
                    self.generate_joint_pose(self.joints[i], position)
            self._config[i] = position

    @property
    def visible(self):
        return self._visible

    @visible.setter
    def visible(self, value):
        self._visible = value
        for piece in self.pieces:
            piece.visible = value

    def mount_pose(self):
        """\
        Return the overall pose transformation to the tool end.

        @return: The overall tool pose.
        @rtype: L{Pose}
        """
        return self._mount_pose + self.pieces[-1].mount_pose()

    @staticmethod
    def generate_joint_pose(joint, position=None):
        if position is None:
            position = joint['home']
        else:
            if position < joint['limits'][0] or position > joint['limits'][1]:
                raise ValueError('position out of joint range')
        if joint['type'] == 'revolute':
            position *= pi / 180.0
            return Pose(R=Rotation.from_axis_angle(position, Point(joint['axis'])))
        elif joint['type'] == 'prismatic':
            return Pose(T=(position * Point(joint['axis'])))
        else:
            raise ValueError('invalid joint type')

    def visualize(self):
        """\
        Visualize this robot.
        """
        for piece in self.pieces:
            piece.visualize()
        self._visible = True

    def update_visualization(self):
        """\
        Update this robot's visualization.
        """
        for piece in self.pieces:
            piece.update_visualization()
