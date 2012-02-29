"""\
Laser range imaging module. Contains extensions and objects necessary for
modeling laser line based range imaging cameras.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from math import pi, sin, tan, atan

from .geometry import Angle, Pose, Point, DirectionalPoint, Triangle
from .coverage import PointCache, Task, Camera, Model
from .posable import SceneObject


class RangeTask(Task):
    """\
    Range imaging task model class.

    The L{RangeTask} adds height resolution to the set of task parameters:

        - C{hres_min_ideal}: ideal min. height resolution (mm/pixel).
        - C{hres_min_acceptable}: min. acceptable height resolution (mm/pixel).
        - C{inc_angle_max}: max. laser incidence angle (radians).
    """
    defaults = Task.defaults
    defaults['hres_min_ideal'] = float('inf')
    defaults['hres_min_acceptable'] = float('inf')
    defaults['inc_angle_max'] = pi / 2.0

    def setparam(self, param, value):
        """\
        Set a camera parameter.

        @param param: The name of the paramater to set.
        @type param: C{str}
        @param value: The value to which to set the parameter.
        @type value: C{object}
        """
        super(RangeTask, self).setparam(param, value)
        if param == 'hres_min_ideal':
            self._params['hres_min_acceptable'] = \
                max(value, self.getparam('hres_min_acceptable'))
        elif param == 'hres_min_acceptable':
            self._params['hres_min_ideal'] = \
                min(value, self.getparam('hres_min_ideal'))


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
        self.click_actions = {'shift':  'modify %s' % name,
                              'ctrl':   'guide %s' % name}

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

    def triangle_primitives(self):
        """\
        Generate the curve primitives for this laser's triangle.

        @return: Triangle primitives.
        @rtype: C{list} of C{dict}
        """
        width = self._depth * tan(self._fan / 2.0)
        return [{'type': 'curve', 'color': (1, 0, 0), 'pos': [(0, 0, 0),
            (-width, 0, self._depth), (width, 0, self._depth), (0, 0, 0)]}]


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
            return self._mr * resolution
        except AttributeError:
            self._mr = float(self._params['dim'][0]) / self.fov['tah']
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
        mr = ascale * float(self._params['dim'][1]) / self.fov['tav']
        zhres = lambda resolution: mr * resolution
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
        try:
            if abs(cp.direction_unit.x) > 1e-4:
                raise ValueError('point is not aligned for range coverage')
        except AttributeError:
            raise TypeError('point must be directional for range coverage')
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
        self._active_laser = None
        self._rcl_cache = {}
        super(RangeModel, self).__init__()

    def __setitem__(self, key, value):
        if isinstance(value, LineLaser):
            self.lasers.add(key)
        super(RangeModel, self).__setitem__(key, value)

    def __delitem__(self, key):
        self.lasers.discard(key)
        super(RangeModel, self).__delitem__(key)

    @property
    def active_laser(self):
        """\
        The active line laser.
        """
        if self._active_laser:
            return self._active_laser
        elif len(self.lasers) == 1:
            self.active_laser = iter(self.lasers).next()
            return self._active_laser
        else:
            raise RuntimeError('no active laser specified from multiple lasers')

    @active_laser.setter
    def active_laser(self, value):
        if value in self.lasers:
            if value != self._active_laser:
                self._clear_caches()
                def callback():
                    self._clear_caches()
                try:
                    del self[self._active_laser].posecallbacks['rc_cache']
                except KeyError:
                    pass
                self[value].posecallbacks['rc_cache'] = callback
            self._active_laser = value
        else:
            raise KeyError('invalid laser %s' % value)

    def _clear_caches(self):
        self._rcl_cache = {}
        # TODO: delete rotary range coverage cache

    def occluded(self, point, obj, task_params=None):
        """\
        Return whether the specified point is occluded with respect to the
        specified object. If task parameters are specified, an occlusion cache
        is used.

        For efficiency in range coverage, this also returns the incidence angle
        to a nearby surface normal in the laser plane if the object is a laser.

        @param point: The point to check.
        @type point: L{Point}
        @param obj: The object ID to check.
        @type obj: C{str}
        @param task_params: Task parameters (optional).
        @type task_params: C{dict}
        @return: True if occluded, plus incidence angle.
        @rtype: C{bool}, C{float}
        """
        if not isinstance(self[obj], LineLaser):
            return super(RangeModel, self).occluded(point, obj,
                task_params=task_params)
        if task_params:
            key = self._update_occlusion_cache(task_params)
            tset = set([t for ts in \
                [self._occlusion_cache[key][obj][sceneobject] \
                for sceneobject in self] for t in ts])
        else:
            tset = set([t for ts in [self[sceneobject].triangles \
                for sceneobject in self] for t in ts])
        d = self[obj].pose.T.euclidean(point)
        ds = float('inf')
        surface = None
        for triangle in tset:
            ip = triangle.intersection(self[obj].pose.T, point, limit=False)
            if not ip:
                continue
            di = self[obj].pose.T.euclidean(ip)
            if di < d:
                return True, None
            elif di < ds:
                ds = di
                surface = triangle
        if surface and ds - d < 1e-04:
            ln = (-self[obj].pose).map(surface.mapped_triangle.normal)
            angle = atan(ln.x / ln.z)
        else:
            angle = None
        return False, angle

    def range_coverage_linear(self, task, taxis=None, subset=None):
        """\
        Return the coverage model using a linear range coverage scheme. It is
        assumed that the specified task is mounted on the target object, which
        will be moved linearly along the transport axis through the laser
        plane. Binary laser coverage is computed, and covered points are
        converted to directional points in the laser plane for camera coverage
        per L{RangeCamera}.

        This function caches the laser coverage calculations, which might
        otherwise be repeated many times during tests or optimization. A cache
        entry is specific to the task, transport axis, and initial target pose,
        and is cleared if the active laser is changed or has its pose modified.

        @param task: The range coverage task.
        @type task: L{Task}
        @param taxis: The transport axis (defaults to laser plane normal).
        @type taxis: L{Point}
        @param subset: Subset of cameras (defaults to all active cameras).
        @type subset: C{set}
        @return: The coverage model.
        @rtype: L{PointCache}
        """
        if not isinstance(task, RangeTask):
            raise TypeError('task is not a range coverage task')
        if not taxis:
            taxis = self[self.active_laser].triangle.normal
        elif not taxis * self[self.active_laser].triangle.normal:
            raise ValueError('transport axis parallel to laser plane')
        rho, eta = self[self.active_laser].pose.map(\
            DirectionalPoint((0, 0, 0, pi, 0)))[3:5]
        original_pose = task.mount.pose
        task_original = PointCache(task.mapped)

        oc_mask_original = self._oc_mask
        def oc_mask_target(obj):
            self._oc_mask.add(obj.name)
            for child in obj.children:
                if isinstance(child, SceneObject):
                    oc_mask_target(child)
        oc_mask_target(task.mount)

        coverage = PointCache()

        rcl_cache = {}
        cache_key = hash(tuple(task.original.keys())) + \
                    hash(tuple(task.params.values())) + \
                    hash(taxis) + hash(original_pose)

        try:
            for point in task_original:
                if cache_key in self._rcl_cache:
                    try:
                        mdp = self._rcl_cache[cache_key][point][0]
                    except KeyError:
                        coverage[point] = 0.0
                        continue
                    else:
                        pose = self._rcl_cache[cache_key][point][1]
                        task.mount.set_absolute_pose(original_pose + pose)
                else:
                    lp = self[self.active_laser].triangle.intersection(point,
                        point + taxis, limit=False)
                    if not lp:
                        coverage[point] = 0.0
                        continue
                    pose = Pose(T=(lp - point))
                    task.mount.set_absolute_pose(original_pose + pose)
                    mdp = DirectionalPoint(tuple(pose.map(point)) + (rho, eta))
                    occluded, inc_angle = self.occluded(mdp, self.active_laser)
                    if occluded or inc_angle > task.params['inc_angle_max']:
                        coverage[point] = 0.0
                        continue
                    rcl_cache[point] = (mdp, pose)
                coverage[point] = self.strength(mdp, task.params, subset=subset)
            if not cache_key in self._rcl_cache:
                self._rcl_cache[cache_key] = rcl_cache
        finally:
            task.mount.set_absolute_pose(original_pose)
            self._oc_mask = oc_mask_original

        return coverage
