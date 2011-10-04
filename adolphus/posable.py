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
from .geometry import Point, Pose, Rotation, Triangle
from .visualization import visual, Visualizable


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
        self.children = set()
        self.posecallbacks = set()
        self._mount = None
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

    def _pose_changed_hook(self):
        """\
        Hook called on pose change.
        """
        for child in self.children:
            child._pose_changed_hook()
        for callback in self.posecallbacks:
            callback()

    @property
    def mount(self):
        """\
        The mount of this posable.
        """
        return self._mount

    @mount.setter
    def mount(self, value):
        """\
        Set the mount of this posable.
        """
        if self._mount:
            self._mount.children.discard(self)
        self._mount = value
        if value:
            value.children.add(self)
        for child in self.children:
            child._pose_changed_hook()

    def set_absolute_pose(self, pose):
        """\
        Set the absolute (world frame) pose of the object.

        @param pose: The absolute pose to set.
        @type pose: L{Pose}
        """
        try:
            self._pose = pose - self.mount.mount_pose()
        except AttributeError:
            self._pose = pose
        self._pose_changed_hook()

    def set_relative_pose(self, pose):
        """\
        Set the relative (mounted) pose of the object.

        @param pose: The relative pose to set.
        @type pose: L{Pose}
        """
        self._pose = pose
        self._pose_changed_hook()

    def mount_pose(self):
        """\
        Return the overall pose transformation to the mount end (unless
        otherwise specified, this will simply place the mounted object into
        this object's coordinate system).

        @return: The overall pose.
        @rtype: L{Pose}
        """
        return self._mount_pose + self.pose


class OcclusionTriangle(Posable, Visualizable):
    """\
    Triangle in 3D space used for polyhedral occlusion.
    """
    def __init__(self, vertices, pose=Pose(), mount=None):
        """\
        Constructor.

        @param vertices: The vertices of the triangle.
        @type vertices: C{tuple} of L{Point}
        @param pose: The pose of the triangle normal (from z-hat).
        @type pose: L{Pose}
        @param mount: The mount of the triangle (optional).
        @type mount: L{Posable}
        """
        Posable.__init__(self, pose=pose, mount=mount)
        vertices = [Point(vertex) for vertex in vertices]
        self._triangle = Triangle(vertices)
        polygon = visual.Polygon([(self._triangle.planing_pose.map(p).x,
                                   self._triangle.planing_pose.map(p).y) \
                                  for p in self._triangle.vertices])
        primitives = [{'type':      'extrusion',
                       'pos':       [(0, 0, 0.5), (0, 0, -0.5)],
                       'shape':     polygon}]
        Visualizable.__init__(self, primitives)

    def intersection(self, pa, pb):
        """\
        Return the intersection of a line segment with this triangle.

        @param pa: The first point.
        @type pa: L{Point}
        @param pb: The second point.
        @type pb: L{Point}
        @return: The point of intersection.
        @rtype: L{Point}
        """
        # TODO: get rid of self.pose.map and return a bool for speed?
        intersection = self._triangle.intersection((-self.pose).map(pa),
            (-self.pose).map(pb))
        if intersection:
            return self.pose.map(intersection)
        else:
            return None

    def update_visualization(self):
        """\
        Update the visualization of this triangle.
        """
        for display in self.actuals:
            print self._triangle.planing_pose
            self.actuals[display].transform(-self._triangle.planing_pose \
                + self.pose)
            self.actuals[display].opacity = self.opacity


class SceneObject(Posable, Visualizable):
    """\
    Sprite-based scene object.
    """
    def __init__(self, name, pose=Pose(), mount_pose=Pose(), mount=None,
                 primitives=[], triangles=[]):
        """\
        Constructor.

        @param name: The name of the object.
        @type name: C{str}
        @param pose: The pose of the object (optional).
        @type pose: L{Pose}
        @param mount_pose: The transformation to the mounting end (optional).
        @type mount_pose: L{Pose}
        @param mount: The mount of the object (optional).
        @type mount: L{Posable}
        @param primitives: A list of sprite primitive sets (optional).
        @type primitives: C{list} of C{dict}
        @param triangles: The opaque triangles of this object (optional).
        @type triangles: C{list} of L{OcclusionTriangle}
        """
        self.name = name
        Posable.__init__(self, pose=pose, mount_pose=mount_pose, mount=mount)
        Visualizable.__init__(self, primitives=primitives)
        try:
            self.triangles = set()
        except AttributeError:
            # child class has defined a custom triangles property
            pass
        else:
            for triangle in triangles:
                if isinstance(triangle, OcclusionTriangle):
                    self.triangles.add(triangle)
                else:
                    # FIXME: this case seems "unnatural" - move to yamlparser?
                    triangle['mount'] = self
                    try:
                        triangle['pose'] = triangle['pose'] - self._mount_pose
                    except KeyError:
                        triangle['pose'] = -self._mount_pose
                    self.triangles.add(OcclusionTriangle(**triangle))
            self._triangles_view = False

    def toggle_triangles(self):
        """\
        Toggle display of occluding triangles in the visualization.
        """
        if self._triangles_view:
            self.opacity = 1.0
            self.update_visualization()
            for triangle in self.triangles:
                triangle.visible = False
        else:
            self.opacity = 0.1
            self.update_visualization()
            for triangle in self.triangles:
                triangle.visualize()
                triangle.visible = True
        self._triangles_view = not self._triangles_view

    def update_visualization(self):
        """\
        Update this object's visualization.
        """
        Visualizable.update_visualization(self)
        for child in self.children:
            try:
                child.update_visualization()
            except AttributeError:
                pass


class SceneTriangle(SceneObject):
    def __init__(self, name, vertices, pose=Pose(), mount_pose=Pose(),
                 mount=None):
        """\
        Plain triangle directly in the scene.
        """
        self.triangle = OcclusionTriangle(vertices, pose=pose, mount=mount)
        super(SceneTriangle, self).__init__(name, pose=self.triangle.pose,
            mount_pose=mount_pose, mount=mount, primitives=[],
            triangles=[self.triangle])

    @property
    def mount(self):
        """\
        The mount of this scene triangle.
        """
        return self._mount

    @mount.setter
    def mount(self, value):
        """\
        Set the mount of this scene triangle.
        """
        if self._mount:
            self._mount.children.discard(self)
        self._mount = value
        if value:
            value.children.add(self)
        for child in self.children:
            child._pose_changed_hook()
        self.triangle.mount = value

    def toggle_triangles(self): pass

    def highlight(self, color=(0, 1, 0)): self.triangle.highlight(color)

    def unhighlight(self): self.triangle.unhighlight()

    def visualize(self): self.triangle.visualize()

    def update_visualization(self): self.triangle.update_visualization()


class Robot(SceneObject):
    """\
    Sprite-based robot.
    """
    def __init__(self, name, pose=Pose(), mount=None, pieces=[], config=None,
                 occlusion=True):
        super(Robot, self).__init__(name, pose=pose, mount=mount)
        self.pieces = []
        nextpose = pose
        for i, piece in enumerate(pieces):
            offset = piece['offset']
            try:
                mount = self.pieces[i - 1]
            except IndexError:
                mount = self.mount
            try:
                assert occlusion
                triangles = piece['triangles']
            except (AssertionError, KeyError):
                triangles = []
            try:
                primitives = piece['primitives']
            except KeyError:
                primitives = []
            self.pieces.append(SceneObject('%s-%i' % (name, i), pose=nextpose,
                mount_pose=offset, mount=mount, primitives=primitives,
                triangles=triangles))
            nextpose = self.generate_joint_pose(piece['joint'])
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
        self._pose_changed_hook()

    @property
    def visible(self):
        return self._visible

    @visible.setter
    def visible(self, value):
        self._visible = value
        for piece in self.pieces:
            piece.visible = value

    def highlight(self, color=(0, 1, 0)):
        """\
        Highlight this robot with a bright uniform color.

        @param color: The color of the highlight.
        @type color: C{tuple}
        """
        for piece in self.pieces:
            piece.highlight(color)

    def unhighlight(self):
        """\
        Unhighlight this robot (if highlighted).
        """
        for piece in self.pieces:
            piece.unhighlight()

    def mount_pose(self):
        """\
        Return the overall pose transformation to the tool end.

        @return: The overall tool pose.
        @rtype: L{Pose}
        """
        return self.pieces[-1].mount_pose()

    @property
    def pose(self):
        """\
        The pose of the robot.
        """
        return self.pieces[0].pose

    def set_absolute_pose(self, pose):
        """\
        Set the absolute (world frame) pose of the robot.

        @param pose: The absolute pose to set.
        @type pose: L{Pose}
        """
        self.pieces[0].set_absolute_pose(pose)
        super(Robot, self).set_absolute_pose(pose)

    def set_relative_pose(self, pose):
        """\
        Set the relative (mounted) pose of the robot.

        @param pose: The relative pose to set.
        @type pose: L{Pose}
        """
        self.pieces[0].set_relative_pose(pose)
        super(Robot, self).set_relative_pose(pose)

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

    @property
    def triangles(self):
        """\
        Occluding triangles.
        """
        return reduce(lambda a, b: a | b,
            [piece.triangles for piece in self.pieces])

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
            piece.opacity = self.opacity
            piece.update_visualization()
        for child in self.children:
            try:
                child.update_visualization()
            except AttributeError:
                pass
        
    def toggle_triangles(self):
        """\
        Toggle display of occluding triangles in the visualization.
        """
        for piece in self.pieces:
            piece.toggle_triangles()
        self.opacity = self.pieces[0].opacity
