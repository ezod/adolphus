"""\
Posable objects module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from .geometry import Point, Pose, Triangle
from .visualization import visual, Visualizable


class Posable(object):
    """\
    Posable base class.

    A L{Posable} is a geometric object which may be queried for its 3D L{Pose}.
    In the simplest case, the pose is absolute with respect to the world
    coordinate system. Alternatively, the object may be mounted on another
    L{Posable}; in this case, the stored pose is relative, and the absolute
    pose is computed by composition with the pose (and mount pose) of the object
    on which it is mounted. The mount pose is a pose relative to the object
    describing the "starting point" of the pose of a mounted object.
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
        The (absolute) pose of the object.
        """
        if self.mount:
            return self._pose + self.mount.mount_pose()
        else:
            return self._pose

    @property
    def relative_pose(self):
        """\
        The relative pose of the object.
        """
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
        if self._mount:
            self._mount.children.discard(self)
        self._mount = value
        if value:
            value.children.add(self)
        for child in self.children:
            child._pose_changed_hook()

    def set_absolute_pose(self, pose):
        """\
        Set the absolute (world frame) pose of the object. If the object is
        mounted, this computes the necessary relative pose to achieve the
        specified absolute pose.

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

    This is essentially a wrapper around the basic L{Triangle} object allowing
    it to be posed and visualized. Usually, this class is instantiated based on
    occluding triangle definitions in L{SceneObject} sprites.
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
        vertices = [Point(vertex) for vertex in vertices]
        planing_pose = Triangle(vertices).planing_pose
        self.triangle = Triangle([planing_pose.map(v) for v in vertices])
        Posable.__init__(self, pose=(-planing_pose + pose), mount=mount)
        polygon = visual.Polygon([v[0:2] for v in self.triangle.vertices])
        primitives = [{'type':      'extrusion',
                       'pos':       [(0, 0, 0.5), (0, 0, -0.5)],
                       'shape':     polygon}]
        Visualizable.__init__(self, primitives)

    @property
    def pose(self):
        """\
        The pose of the triangle. Triangles ignore the mount pose of the parent
        object when mounted, to simplify manual definition.
        """
        if self.mount:
            return self._pose + self.mount.pose
        else:
            return self._pose

    def _pose_changed_hook(self):
        """\
        Hook called on pose change.
        """
        try:
            del self._mapped_triangle
        except AttributeError:
            pass
        super(OcclusionTriangle, self)._pose_changed_hook()

    @property
    def mapped_triangle(self):
        """\
        Pose-mapped triangle.
        """
        try:
            return self._mapped_triangle
        except AttributeError:
            self._mapped_triangle = Triangle([self.pose.map(v) \
                for v in self.triangle.vertices])
            return self._mapped_triangle

    def intersection(self, pa, pb, limit=True):
        """\
        Return the intersection of a line or line segment with this triangle.

        @param pa: The first point.
        @type pa: L{Point}
        @param pb: The second point.
        @type pb: L{Point}
        @param limit: If true, limit intersection to the line segment.
        @type limit: C{bool}
        @return: The point of intersection.
        @rtype: L{Point}
        """
        return self.mapped_triangle.intersection(pa, pb, limit=limit)


class SceneObject(Posable, Visualizable):
    """\
    Sprite-based scene object class.

    A L{SceneObject} is a L{Posable}-L{Visualizable} base class for regular
    objects in the scene. Beyond the basic functionality of the parent classes,
    it allows a set of occluding triangles to be defined, which in general
    defines an occluding polyhedral solid, and maintains a public set of
    L{OcclusionTriangle} objects.
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
            # child class handles triangles separately
            pass
        else:
            for triangle in triangles:
                triangle.mount = self
                self.triangles.add(triangle)
            self._triangles_view = False
        self.click_actions = {'shift':  'modify %s' % name}

    def _pose_changed_hook(self):
        """\
        Hook called on pose change.
        """
        try:
            del self._bounding_box
        except AttributeError:
            pass
        super(SceneObject, self)._pose_changed_hook()

    @property
    def visible(self):
        """\
        Visibility of this object.
        """
        return self._visible

    @visible.setter
    def visible(self, value):
        if not self.primitives:
            for triangle in self.triangles:
                triangle.visible = value
            self._triangles_view = value
        for display in self.actuals:
            self.actuals[display].visible = value
        self._visible = value

    @property
    def bounding_box(self):
        """\
        Axis-aligned bounding box of occluding triangles.
        """
        try:
            return self._bounding_box
        except AttributeError:
            if not self.triangles:
                self._bounding_box = None
            else:
                vertices = set([vertex for triangles in [[v for v in \
                    t.mapped_triangle.vertices] for t in self.triangles] \
                    for vertex in triangles])
                p = vertices.pop()
                self._bounding_box = [[p.x, p.x], [p.y, p.y], [p.z, p.z]]
                for p in vertices:
                    for i in range(3):
                        if p[i] < self._bounding_box[i][0]:
                            self._bounding_box[i][0] = p[i]
                        elif p[i] > self._bounding_box[i][1]:
                            self._bounding_box[i][1] = p[i]
            return self._bounding_box

    def toggle_triangles(self):
        """\
        Toggle display of occluding triangles in the visualization. This fades
        the object sprites so that the opaque triangles can be seen clearly.
        """
        if not self.primitives:
            return
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

    def visualize(self):
        """\
        Visualize this object. If no sprites are defined, the attached
        occluding triangles are visualized instead.
        """
        Visualizable.visualize(self)
        if not self.primitives:
            for triangle in self.triangles:
                triangle.visualize()
            self._triangles_view = True

    def update_visualization(self):
        """\
        Update this object's visualization.
        """
        Visualizable.update_visualization(self)
        if not self.primitives:
            for triangle in self.triangles:
                triangle.update_visualization()
        for child in self.children:
            try:
                child.update_visualization()
            except AttributeError:
                pass
