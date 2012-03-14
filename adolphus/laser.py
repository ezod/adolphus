"""\
Laser range imaging module. Contains extensions and objects necessary for
modeling laser line based range imaging cameras.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from math import pi, sin, tan, atan
from copy import copy

from geometry import Angle, Pose, Point, DirectionalPoint, Triangle
from coverage import PointCache, Task, Camera, Model
from posable import SceneObject


class RangeTask(Task):
    """\
    Range imaging task model class.

    The L{RangeTask} adds to the set of task parameters:

        - C{hres_min}: minimum height resolution (mm/pixel), ideal/acceptable.
        - C{inc_angle_max}: max. laser incidence angle (radians).
    """
    defaults = copy(Task.defaults)
    defaults['hres_min'] = [float('inf')] * 2
    defaults['inc_angle_max'] = pi / 2.0
    _lt_params = Task._lt_params + ['hres_min']


class LineLaser(SceneObject):
    """\
    Line laser class.
    """
    param_keys = ['fan', 'depth']

    def __init__(self, name, params, pose=Pose(), mount_pose=Pose(), mount=None,
                 primitives=[], triangles=[]):
        """\
        Constructor.

        @param name: The name of the laser.
        @type name: C{str}
        @param params: The laser paramters.
        @type params: C{dict}
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
        self.paramcallbacks = {}
        self._params = {}
        for param in params:
            try:
                self.setparam(param, params[param])
            except KeyError:
                pass

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
    def params(self):
        """\
        Laser parameters.
        """
        return self._params

    def getparam(self, param):
        """\
        Retrieve a laser parameter.

        @param param: The name of the parameter to retrieve.
        @type param: C{str}
        @return: The value of the parameter.
        @rtype: C{object}
        """
        return self._params[param]

    def setparam(self, param, value):
        """\
        Set a laser parameter.

        @param param: The name of the paramater to set.
        @type param: C{str}
        @param value: The value to which to set the parameter.
        @type value: C{float} or C{list} of C{float}
        """
        if not param in self.param_keys:
            raise KeyError(param)
        if param == 'fan':
            value = Angle(value)
        self._params[param] = value
        try:
            del self._triangle
        except AttributeError:
            pass
        for callback in self.paramcallbacks.values():
            callback()

    @property
    def triangle(self):
        """\
        Triangle formed by the laser plane.
        """
        try:
            return self._triangle
        except AttributeError:
            width = self._params['depth'] * tan(self._params['fan'] / 2.0)
            self._triangle = Triangle(self.pose.T,
                self.pose._map(Point(-width, 0, self._params['depth'])),
                self.pose._map(Point(width, 0, self._params['depth'])))
            return self._triangle

    def occluded_by(self, triangle, *args):
        """\
        Return whether this laser's projection plane is occluded (in part) by
        the specified triangle.

        @param triangle: The triangle to check.
        @type triangle: L{OcclusionTriangle}
        @return: True if occluded.
        @rtype: C{bool}
        """
        return self.triangle.overlap(triangle)

    def triangle_primitives(self):
        """\
        Generate the curve primitives for this laser's triangle.

        @return: Triangle primitives.
        @rtype: C{list} of C{dict}
        """
        width = self._params['depth'] * tan(self._params['fan'] / 2.0)
        return [{'type': 'curve', 'color': (1, 0, 0),
            'pos': [(0, 0, 0), (-width, 0, self._params['depth']),
            (width, 0, self._params['depth']), (0, 0, 0)]}]


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

    def zhres(self, resolution, angle):
        """\
        Return the depth at which the specified height resolution occurs, for
        the given angle (with respect to the projection axis).

        @param resolution: Height resolution in millimeters per pixel.
        @type resolution: C{float}
        @param angle: The angle to the point.
        @type angle: L{Angle}
        @return: Depth (distance along camera M{z}-axis).
        @rtype: C{float}
        """
        mr = sin(angle) * float(self._params['dim'][1]) / self.fov['tav']
        return mr * resolution

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
            angle = p.direction_unit().angle(-p)
        except AttributeError:
            return 0.0
        try:
            zhmini = self.zhres(tp['hres_min'][0], angle)
        except ZeroDivisionError:
            zhmini = float('inf')
        try:
            zhmina = self.zhres(tp['hres_min'][1], angle)
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
        cp = self.pose.inverse().map(point)
        try:
            if abs(cp.direction_unit().x) > 1e-4:
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
        super(RangeModel, self).__init__()

    def __setitem__(self, key, value):
        if isinstance(value, LineLaser):
            self.lasers.add(key)
        super(RangeModel, self).__setitem__(key, value)

    def __delitem__(self, key):
        self.lasers.discard(key)
        super(RangeModel, self).__delitem__(key)

    def get_active_laser(self):
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

    def set_active_laser(self, value):
        if value in self.lasers:
            self._active_laser = value
        else:
            raise KeyError('invalid laser %s' % value)

    active_laser = property(get_active_laser, set_active_laser)

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
        key = self._update_occlusion_cache(task_params)
        d = self[obj].pose.T.euclidean(point)
        ds = float('inf')
        surface = None
        for triangle in self._occlusion_cache[key][obj]:
            ip = triangle.intersection(self[obj].pose.T, point, False)
            if not ip:
                continue
            di = self[obj].pose.T.euclidean(ip)
            if di < d:
                return True, None
            elif di < ds:
                ds = di
                surface = triangle
        if surface and ds - d < 1e-04:
            ln = self[obj].pose.inverse()._map(surface.normal())
            angle = atan(ln.x / ln.z)
        else:
            angle = None
        return False, angle

    class Transport(object):
        def __init__(self, model):
            self.model = model
            self.laser = self.model[self.model.active_laser]

        def __enter__(self):
            self.original_pose = self.tobject.pose
            self.original_oc_mask = copy(self.model._oc_mask)
            self.oc_mask(self.tobject)
            return self

        def __exit__(self, exc_type, exc_value, exc_traceback):
            self.model._oc_mask = self.original_oc_mask
            self.tobject.set_absolute_pose(self.original_pose)

        def oc_mask(self, sceneobject):
            self.model.occlusion_cache_mask(sceneobject.name)
            for child in sceneobject.children:
                if isinstance(child, SceneObject):
                    self.oc_mask(child)

        @property
        def tobject(self):
            raise NotImplementedError

        def transport(self):
            raise NotImplementedError

    class LinearTargetTransport(Transport):
        def __init__(self, model, taxis=None):
            super(RangeModel.LinearTargetTransport, self).__init__(model)
            if not taxis:
                self.taxis = self.laser.triangle.normal()
            elif not taxis.dot(self.laser.triangle.normal()):
                raise ValueError('transport axis parallel to laser plane')
            else:
                self.taxis = taxis
        
        @property
        def tobject(self):
            return self.task.mount

        def transport(self):
            rho, eta = self.laser.pose._dmap(DirectionalPoint(0, 0, 0, pi, 0))[3:5]
            task_original = PointCache(self.task.mapped)
            for point in task_original:
                lp = self.laser.triangle.intersection(point, point + self.taxis, False)
                if not lp:
                    yield point, None
                pose = Pose(T=(lp - point))
                self.tobject.absolute_pose = self.original_pose + pose
                mp = pose._map(point)
                yield point, DirectionalPoint(mp.x, mp.y, mp.z, rho, eta)

    def range_coverage(self, task, transport, subset=None, **kwargs):
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
        @param transport:
        @type transport: C{str}
        @param subset: Subset of cameras (defaults to all active cameras).
        @type subset: C{set}
        @return: The coverage model.
        @rtype: L{PointCache}
        """
        if not isinstance(task, RangeTask):
            raise TypeError('task is not a range coverage task')
        coverage = PointCache()
        transport = RangeModel.LinearTargetTransport(self)
        transport.task = task
        with transport:
            for point, mdp in transport.transport():
                if not mdp:
                    coverage[point] = 0.0
                    continue
                occluded, inc_angle = self.occluded(mdp, self.active_laser)
                if occluded or inc_angle > task.params['inc_angle_max']:
                    coverage[point] = 0.0
                    continue
                coverage[point] = self.strength(mdp, task.params, subset=subset)
        return coverage
