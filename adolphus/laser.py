"""\
Laser range imaging module. Contains extensions and objects necessary for
modeling laser line based range imaging cameras.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from math import pi, sin, tan

from .geometry import Angle, Pose, Point, DirectionalPoint, Triangle
from .coverage import PointCache, Camera, Model
from .posable import SceneObject


class LineLaser(SceneObject):
    """\
    Line laser class.
    """
    def __init__(self, name, fan, depth, pose=Pose(), mount_pose=Pose(),
                 mount=None, primitives=[], triangles=[]):
        """\
        Constructor.

        @param name: The name of the laser.
        @type name: C{str}
        @param fan: Fan angle of the laser line.
        @type fan: L{Angle}
        @param depth: Projection depth of the laser line.
        @type depth: C{float}
        @param pose: Pose of the laser in space (optional).
        @type pose: L{Pose}
        @param mount_pose: The transformation to the mounting end (optional).
        @type mount_pose: L{Pose}
        @param mount: Mount object for the laser (optional).
        @type mount: C{object}
        @param primitives: Sprite primitives for the laser.
        @type primitives: C{dict}
        @param triangles: The opaque triangles of this laser (optional).
        @type triangles: C{list} of L{OcclusionTriangle}
        """
        super(LineLaser, self).__init__(name, pose=pose, mount_pose=mount_pose,
            mount=mount, primitives=primitives, triangles=triangles)
        self._fan = Angle(fan)
        self._depth = depth
        self.click_actions = {'shift':  'modify %s' % name}

    def _pose_changed_hook(self):
        """\
        Hook called on pose change.
        """
        try:
            del self._triangle
        except AttributeError:
            pass
        super(LineLaser, self)._pose_changed_hook()

    @property
    def fan(self):
        """\
        Fan angle.
        """
        return self._fan

    @property
    def depth(self):
        """\
        Projection depth.
        """
        return self._depth

    @property
    def triangle(self):
        """\
        Triangle formed by the laser plane.
        """
        try:
            return self._triangle
        except AttributeError:
            width = self._depth * tan(self._fan / 2.0)
            self._triangle = Triangle((self.pose.T,
                self.pose.map(Point((-width, 0, self._depth))),
                self.pose.map(Point((width, 0, self._depth)))))
            return self._triangle

    def occluded_by(self, triangle, task_params):
        """\
        Return whether this laser's projection plane is occluded (in part) by
        the specified triangle.

        @param triangle: The triangle to check.
        @type triangle: L{OcclusionTriangle}
        @param task_params: Task parameters (not used).
        @type task_params: C{dict}
        @return: True if occluded.
        @rtype: C{bool}
        """
        return self.triangle.overlap(triangle.mapped_triangle)


class RangeCamera(Camera):
    """\
    Single-camera coverage strength model for laser line based range camera.
    """
    def zres(self, resolution):
        """\
        Return the depth at which the specified resolution occurs. In a range
        camera, only horizontal resolution is considered (and the resolution
        coverage component behaves accordingly).

        @param resolution: Horizontal resolution in millimeters per pixel.
        @type resolution: C{float}
        @return: Depth (distance along camera M{z}-axis).
        @rtype: C{float}
        """
        try:
            return self._mr / resolution
        except ZeroDivisionError:
            return float('inf')
        except AttributeError:
            self._mr = float(self._params['dim'][0]) / self.fov['2sah2']
            return self.zres(resolution)

    def ch(self, p, tp):
        """\
        Height resolution component of the coverage function. Returns zero
        coverage for non-directional points as it is intended for use in range
        coverage only.

        @param p: The point to test.
        @type p: L{Point}
        @param tp: Task parameters.
        @type tp: C{dict}
        @return: The height resolution coverage component value in M{[0, 1]}.
        @rtype: C{float}
        """
        try:
            ascale = sin(p.direction_unit.angle(-p))
        except AttributeError:
            return 0.0
        mr = ascale * float(self._params['dim'][1]) / self.fov['2sav2']
        zhres = lambda resolution: mr / resolution
        try:
            zhmini = zhres(tp['hres_min_ideal'])
        except ZeroDivisionError:
            zhmini = float('inf')
        try:
            zhmina = zhres(tp['hres_min_acceptable'])
        except ZeroDivisionError:
            zhmina = float('inf')
        if zhmina == zhmini:
            return float(p.z < zhmina)
        else:
            return min(max((zhmina - p.z) / (zhmina - zhmini), 0.0), 1.0)

    def strength(self, point, task_params):
        """\
        Return the coverage strength for a directional point. Includes the
        height resolution component.

        @param point: The (directional) point to test.
        @type point: L{Point}
        @param task_params: Task parameters.
        @type task_params: C{dict}
        @return: The coverage strength of the point.
        @rtype: C{float}
        """
        cp = (-self.pose).map(point)
        return self.cv(cp, task_params) * self.cr(cp, task_params) \
             * self.cf(cp, task_params) * self.cd(cp, task_params) \
             * self.ch(cp, task_params)


class RangeModel(Model):
    """\
    Multi-camera coverage strength model for laser line based range imaging.

    In addition to the base L{Model} functionality, a L{RangeModel} manages
    line lasers and laser occlusion, and provides range imaging coverage
    methods.
    """
    # class of camera handled by this model class
    camera_class = RangeCamera
    # object types for which occlusion caching is handled by this class
    oc_sets = ['cameras', 'lasers']

    def __init__(self):
        self.lasers = set()
        super(RangeModel, self).__init__()

    def __setitem__(self, key, value):
        if isinstance(value, LineLaser):
            self.lasers.add(key)
        super(RangeModel, self).__setitem__(key, value)

    def __delitem__(self, key):
        self.lasers.discard(key)
        super(RangeModel, self).__delitem__(key)

    def range_coverage_linear(self, task, laser, taxis=None, subset=None):
        """\
        Return the coverage model using a linear range coverage scheme. It is
        assumed that the specified task is mounted on the target object, which
        will be moved linearly TODO

        @param task: The range coverage task.
        @type task: L{Task}
        @param laser: The ID of the laser line generator to use.
        @type laser: C{str}
        @param taxis: The transport axis (defaults to laser plane normal).
        @type taxis: L{Point}
        @param subset: Subset of cameras (defaults to all active cameras).
        @type subset: C{set}
        @return: The coverage model.
        @rtype: L{PointCache}
        """
        if not taxis:
            taxis = self[laser].triangle.normal
        rho, eta = self[laser].pose.map(DirectionalPoint((0, 0, 0, pi, 0)))[3:5]
        original_pose = task.mount.pose
        task_original = PointCache(task.mapped)
        coverage = PointCache()
        for point in task_original:
            # TODO: several opportunities for optimization in this block
            lp = self[laser].triangle.intersection(point, point + taxis,
                limit=False)
            if not lp:
                coverage[point] = 0.0
                continue
            pose = Pose(T=(lp - point))
            task.mount.set_absolute_pose(original_pose + pose)
            mp = pose.map(point)
            if self.occluded(mp, laser):
                coverage[point] = 0.0
            else:
                mdp = DirectionalPoint(tuple(mp) + (rho, eta))
                coverage[point] = self.strength(mdp, task.params, subset=subset)
        task.mount.set_absolute_pose(original_pose)
        return coverage
