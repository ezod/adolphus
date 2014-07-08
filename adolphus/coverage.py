"""\
Coverage model module. Contains the coverage strength model classes for single
cameras and multi-camera networks, as well as point cache and task model
classes.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from math import cos, tan, atan, pi
from numbers import Number
from itertools import combinations
from copy import deepcopy

HYPERGRAPH_ENABLED = True
try:
    import hypergraph
except ImportError:
    HYPGERGRAPH_ENABLED = False

from .geometry import Point, Pose, triangle_frustum_intersection
from .posable import Posable, SceneObject
from .visualization import Visualizable, VISUAL_SETTINGS


class PointCache(dict):
    """\
    Point cache class.

    The L{PointCache} object attempts to improve the efficiency of operations
    involving task models and discrete coverage functions. It provides the
    equivalent of standard fuzzy intersection and union via the C{&} and C{|}
    operators, respectively. It also provides a visualization method.
    """
    def __or__(self, other):
        if len(self) < len(other):
            ds = self
            dl = other
        else:
            ds = other
            dl = self
        result = type(self)(dl)
        for point, value in ds.iteritems():
            if not point in result or result[point] < value:
                result[point] = value
        return result

    def __ior__(self, other):
        self = self | other
        return self

    def __and__(self, other):
        if len(self) < len(other):
            ds = self
            dl = other
        else:
            ds = other
            dl = self
        result = type(self)(dl)
        for point, value in ds.iteritems():
            if not point in result:
                result[point] = value
            else:
                result[point] = min(result[point], value)
        return result

    def __iand__(self, other):
        self = self & other
        return self

    def __del__(self):
        try:
            self.visual.visible = False
        except AttributeError:
            pass

    def visualize(self, color=(1, 0, 0)):
        """\
        Visualize the point cache, with opacity representing coverage strength.

        @param color: The color of the points.
        @type color: C{tuple} of C{float}
        """
        try:
            self.visual.visible = False
            del self.visual
        except AttributeError:
            pass
        primitives = []
        for point in set([point for point in self if self[point]]):
            primitives.append({'type': 'sphere', 'pos': (point.x, point.y,
                point.z), 'radius': 3 * VISUAL_SETTINGS['scale'],
                'color': color, 'opacity': self[point]})
            try:
                primitives.append({'type': 'arrow', 'pos': (point.x, point.y,
                    point.z), 'axis': tuple(point.direction_unit() * 30 * \
                    VISUAL_SETTINGS['scale']), 'color': color,
                    'opacity': self[point]})
            except AttributeError:
                pass
        self.visual = Visualizable(primitives=primitives)
        self.visual.visualize()


class Task(Posable):
    """\
    Task model class.

    A L{Task} object is functionally a posable discrete set of relevant points
    to be covered according to requirements defined by its task parameters. It
    manages two L{PointCache} objects: a static original and a volatile version
    mapped through its pose.

    The available task parameters are:

        - C{ocular}: mutual camera coverage degree (number of cameras).
        - C{boundary_padding}: degradation boundary around image (pixels).
        - C{res_max}: maximum resolution (mm/pixel), ideal/acceptable.
        - C{res_min}: minimum resolution (mm/pixel), ideal/acceptable.
        - C{blur_max}: maximum blur circle diameter (pixels), ideal/acceptable.
        - C{angle_max}: maximum view angle (radians), ideal/acceptable.

    See L{Task.defaults} for default (permissive) values. Note that subclasses
    may define additional task parameters.
    """
    defaults = {'ocular': 1,
                'boundary_padding': 0.0,
                'res_max': [0.0] * 2,
                'res_min': [float('inf')] * 2,
                'blur_max': [1.0, float('inf')],
                'angle_max': [pi / 2.0] * 2}

    _gt_params = ['res_max']
    _lt_params = ['res_min', 'blur_max', 'angle_max']

    def __init__(self, params, original, pose=Pose(), mount=None):
        """\
        Constructor.

        @param params: The task parameters for this task.
        @type params: C{dict}
        @param original: The original set of task model points.
        @type original: L{PointCache}
        @param pose: The pose of this task model.
        @type pose: L{Pose}
        @param mount: The mount of this task model.
        @type mount: L{SceneObject}
        """
        super(Task, self).__init__(pose=pose, mount=mount)
        self._params = {}
        for param in params:
            self.setparam(param, params[param])
        self._original = original

    def _pose_changed_hook(self):
        """\
        Hook called on pose change.
        """
        try:
            del self._mapped
        except AttributeError:
            pass
        super(Task, self)._pose_changed_hook()

    @property
    def original(self):
        """\
        Original (unmapped) task model points.
        """
        return self._original

    @property
    def mapped(self):
        """\
        Actual (mapped) task model points.
        """
        try:
            return self._mapped
        except AttributeError:
            self._mapped = PointCache()
            for point in self.original:
                self._mapped[self.pose.map(point)] = self.original[point]
            return self._mapped

    @property
    def params(self):
        """\
        Task parameters, with defaults filled in for missing values.
        """
        params = self.defaults
        for param in self._params:
            params[param] = self._params[param]
        return params

    def getparam(self, param):
        """\
        Retrieve a task parameter.

        @param param: The name of the parameter to retrieve.
        @type param: C{str}
        @return: The value of the parameter.
        @rtype: C{object}
        """
        try:
            return self._params[param]
        except KeyError:
            return self.defaults[param]

    def setparam(self, param, value):
        """\
        Set a camera parameter.

        @param param: The name of the paramater to set.
        @type param: C{str}
        @param value: The value to which to set the parameter.
        @type value: C{float}
        """
        if not param in self.defaults:
            raise KeyError(param)
        if hasattr(self.defaults[param], '__iter__'):
            if isinstance(value, Number):
                # Split single-valued parameters into pairs, if appropriate.
                value = [value] * 2
            else:
                # Replace 'default' with the numerical default value.
                value = [self.defaults[param][i] if value[i] == 'default' \
                    else value[i] for i in range(2)]
                # Ensure that pairs of values are valid.
                try:
                    if param in self._gt_params:
                        assert value[0] >= value[1]
                    elif param in self._lt_params:
                        assert value[0] <= value[1]
                except AssertionError:
                    raise ValueError('invalid value pair for %s' % param)
        # The 'ocular' parameter is integer-valued.
        if param == 'ocular':
            value = int(value)
        self._params[param] = value

    def visualize(self):
        """\
        Visualize the task model.
        """
        # Ducktype visualizable by visualizing a deep copy of the mapped point
        # cache object.
        try:
            del self.vis
        except AttributeError:
            pass
        self.vis = deepcopy(self.mapped)
        self.vis.visualize()

    def update_visualization(self):
        """\
        Update the task model visualization, if it exists.
        """
        if self.vis:
            del self.vis
            self.visualize()


class Camera(SceneObject):
    """\
    Single-camera coverage strength model.

    A L{Camera} object implements the coverage function for a single camera
    based on camera and task parameters. Being a L{SceneObject} itself, it also
    provides the usual functionality of pose, visualization, and occlusion.
    """
    param_keys = ['A', 'dim', 'f', 'o', 's', 'zS']

    def __init__(self, name, params, pose=Pose(), mount_pose=Pose(), mount=None,
                 primitives=list(), triangles=list()):
        """\
        Constructor.

        @param name: The name of the camera.
        @type name: C{str}
        @param params: The intrinsic camera parameters.
        @type params: C{dict}
        @param pose: Pose of the camera in space (optional).
        @type pose: L{Pose}
        @param mount_pose: The transformation to the mounting end (optional).
        @type mount_pose: L{Pose}
        @param mount: Mount object for the camera (optional).
        @type mount: C{object}
        @param primitives: Sprite primitives for the camera.
        @type primitives: C{dict}
        @param triangles: The opaque triangles of this camera (optional).
        @type triangles: C{list} of L{OcclusionTriangle}
        """
        super(Camera, self).__init__(name, pose=pose, mount_pose=mount_pose,
            mount=mount, primitives=primitives, triangles=triangles)
        self.paramcallbacks = {}
        self._params = {}
        for param in params:
            try:
                self.setparam(param, params[param])
            except KeyError:
                pass
        try:
            self.active = params['active']
        except KeyError:
            self.active = True

    @property
    def params(self):
        """\
        Camera parameters.
        """
        return self._params

    def getparam(self, param):
        """\
        Retrieve a camera parameter.

        @param param: The name of the parameter to retrieve.
        @type param: C{str}
        @return: The value of the parameter.
        @rtype: C{object}
        """
        return self._params[param]

    def setparam(self, param, value):
        """\
        Set a camera parameter.

        @param param: The name of the paramater to set.
        @type param: C{str}
        @param value: The value to which to set the parameter.
        @type value: C{float} or C{list} of C{float}
        """
        if not param in self.param_keys:
            raise KeyError(param)
        # Split the 's' parameter into a pair if specified as a single value.
        if param == 's' and isinstance(value, Number):
            value = [value, value]
        self._params[param] = value
        # Clear cached values if they depend on the parameter.
        if param in ['f', 's', 'o', 'dim']:
            try:
                del self._fov
            except AttributeError:
                pass
            try:
                del self._mr
            except AttributeError:
                pass
        # Fire parameter change callbacks.
        for callback in self.paramcallbacks.values():
            callback()

    @property
    def fov(self):
        """\
        Pre-computed angles of view and the size of the field of view at
        M{z = 1}. These are cached for efficiency.

        @rtype: C{dict}
        """
        try:
            return self._fov
        except AttributeError:
            self._fov = {}
            # Horizontal field of view values.
            self._fov['ahl'] = atan((self._params['o'][0] * \
                self._params['s'][0]) / self._params['f'])
            self._fov['ahr'] = atan(((self._params['dim'][0] - \
                self._params['o'][0]) * self._params['s'][0]) / \
                self._params['f'])
            self._fov['ah'] = self._fov['ahl'] + self._fov['ahr']
            self._fov['tahl'] = -tan(self._fov['ahl'])
            self._fov['tahr'] = tan(self._fov['ahr'])
            self._fov['tah'] = self._fov['tahr'] - self._fov['tahl']
            # Vertical field of view values.
            self._fov['avt'] = atan((self._params['o'][1] * \
                self._params['s'][1]) / self._params['f'])
            self._fov['avb'] = atan(((self._params['dim'][1] - \
                self._params['o'][1]) * self._params['s'][1]) / \
                self._params['f'])
            self._fov['av'] = self._fov['avt'] + self._fov['avb']
            self._fov['tavt'] = -tan(self._fov['avt'])
            self._fov['tavb'] = tan(self._fov['avb'])
            self._fov['tav'] = self._fov['tavb'] - self._fov['tavt']
            return self._fov

    def image(self, point):
        """\
        Return the projected subpixel coordinates of the specified 3D point.

        @param point: The point to project.
        @type point: L{Point}
        @return: The subpixel coordinates of the point (if any).
        @rtype: C{tuple} of C{float}
        """
        cp = self.pose.inverse().map(point)
        if self.cv(cp, {'boundary_padding': 0.0}):
            return tuple([(self._params['f'] / self._params['s'][i]) * \
                (cp[i] / cp.z) + self._params['o'][i] for i in range(2)])
        else:
            return None

    def zres(self, resolution):
        """\
        Return the depth at which the specified resolution occurs.

        @param resolution: Resolution in millimeters per pixel.
        @type resolution: C{float}
        @return: Depth (distance along camera M{z}-axis).
        @rtype: C{float}
        """
        try:
            return self._mr * resolution
        except AttributeError:
            self._mr = min([float(self._params['dim'][0]) / self.fov['tah'],
                            float(self._params['dim'][1]) / self.fov['tav']])
            return self.zres(resolution)

    def zc(self, c):
        """\
        Return the depth values at which a circle of confusion of a given size
        occurs.

        @param c: The diameter of the circle of confusion in millimeters.
        @type c: C{float}
        @return: Two depth values.
        @rtype: C{tuple} of C{float}
        """
        r = []
        for s in [1, -1]:
            r.append((self._params['A'] * self._params['f'] \
                * self._params['zS']) / (self._params['A'] \
                * self._params['f'] + s * c * (self._params['zS'] \
                - self._params['f'])))
        # If the second result is negative, it is (effectively) infinite depth.
        if r[1] <= 0:
            r[1] = float('inf')
        return tuple(r)

    def cv(self, p, tp):
        """\
        Visibility component of the coverage function.

        @param p: The point to test.
        @type p: L{Point}
        @param tp: Task parameters.
        @type tp: C{dict}
        @return: The visibility coverage component value in M{[0, 1]}.
        @rtype: C{float}
        """
        if not tp['boundary_padding']:
            return float(p.x / p.z > self.fov['tahl'] \
                     and p.x / p.z < self.fov['tahr'] \
                     and p.y / p.z > self.fov['tavt'] \
                     and p.y / p.z < self.fov['tavb']) if p.z > 0 else 0.0
        else:
            gh = tp['boundary_padding'] / \
                float(self._params['dim'][0]) * self.fov['tah']
            gv = tp['boundary_padding'] / \
                float(self._params['dim'][1]) * self.fov['tav']
            return min(min(max((min(p.x / p.z - self.fov['tahl'],
                self.fov['tahr'] - p.x / p.z) / gh), 0.0), 1.0),
                min(max((min(p.y / p.z - self.fov['tavt'], self.fov['tavb'] - \
                p.y / p.z) / gv), 0.0), 1.0)) if p.z > 0 else 0.0

    def cr(self, p, tp):
        """\
        Resolution component of the coverage function.

        @param p: The point to test.
        @type p: L{Point}
        @param tp: Task parameters.
        @type tp: C{dict}
        @return: The resolution coverage component value in M{[0, 1]}.
        @rtype: C{float}
        """
        zrmaxi = self.zres(tp['res_max'][0])
        zrmaxa = self.zres(tp['res_max'][1])
        zrmini = self.zres(tp['res_min'][0])
        zrmina = self.zres(tp['res_min'][1])
        if zrmaxa == zrmaxi and zrmina == zrmini:
            return float(p.z > zrmaxa and p.z < zrmina)
        elif zrmaxa == zrmaxi:
            return min(max((zrmina - p.z) / (zrmina - zrmini), 0.0), 1.0) \
                if p.z > zrmaxa else 0.0
        elif zrmina == zrmini:
            return min(max((p.z - zrmaxa) / (zrmaxi - zrmaxa), 0.0), 1.0) \
                if p.z < zrmina else 0.0
        else:
            return min(max(min((p.z - zrmaxa) / (zrmaxi - zrmaxa),
                (zrmina - p.z) / (zrmina - zrmini)), 0.0), 1.0)

    def cf(self, p, tp):
        """\
        Focus component of the coverage function.

        @param p: The point to test.
        @type p: L{Point}
        @param tp: Task parameters.
        @type tp: C{dict}
        @return: The focus coverage component value in M{[0, 1]}.
        @rtype: C{float}
        """
        zn, zf = self.zc(tp['blur_max'][1] * min(self._params['s']))
        if tp['blur_max'][0] == tp['blur_max'][1]:
            return float(p.z > zn and p.z < zf)
        else:
            zl, zr = self.zc(tp['blur_max'][0] * min(self._params['s']))
            return min(max(min((p.z - zn) / (zl - zn),
                               (zf - p.z) / (zf - zr)), 0.0), 1.0)

    def cd(self, p, tp):
        """\
        View angle component of the coverage function.

        @param p: The point to test.
        @type p: L{Point}
        @param tp: Task parameters.
        @type tp: C{dict}
        @return: The view angle coverage component value in M{[0, 1]}.
        @rtype: C{float}
        """
        try:
            # Find the cosine of the angle between the ray from the camera to
            # the point and the direction of the directional point.
            sigma = -(p.unit()).dot(p.direction_unit())
        except (ValueError, AttributeError):
            # Point is at the origin or is non-directional.
            return 1.0
        aa = cos(tp['angle_max'][1])
        if tp['angle_max'][0] == tp['angle_max'][1]:
            return float(sigma > aa)
        else:
            ai = cos(tp['angle_max'][0])
            return min(max((sigma - aa) / (ai - aa), 0.0), 1.0)

    def occluded_by(self, triangle, task_params):
        """\
        Return whether this camera's field of view is occluded (in part) by the
        specified triangle.

            - D. Eberly, "Intersection of Convex Objects: The Method of
              Separating Axes," 2008.

        @param triangle: The triangle to check.
        @type triangle: L{OcclusionTriangle}
        @param task_params: Task parameters.
        @type task_params: C{dict}
        @return: True if occluded.
        @rtype: C{bool}
        """
        # Build a pyramid containing the frustum.
        hull = gen_frustum_hull(task_params)
        # Map the triangle to camera coordinates.
        ctriangle = triangle.pose_map(self.pose.inverse())
        # Return whether an intersection exists.
        return triangle_frustum_intersection(ctriangle, hull)

    def strength(self, point, task_params):
        """\
        Return the coverage strength for a directional point. Note that since
        the L{Camera} object is not internally aware of the scene it inhabits,
        occlusion is computed in the L{Model} object instead.

        @param point: The (directional) point to test.
        @type point: L{Point}
        @param task_params: Task parameters.
        @type task_params: C{dict}
        @return: The coverage strength of the point.
        @rtype: C{float}
        """
        # Map the point to camera coordinates.
        cp = self.pose.inverse().map(point)
        # Return the coverage value.
        return self.cv(cp, task_params) * self.cr(cp, task_params) \
             * self.cf(cp, task_params) * self.cd(cp, task_params)

    def update_visualization(self):
        """\
        Update the visualization for camera active state and pose.
        """
        # TODO: use something else for active, so camera triangles can be seen
        self.opacity = 1.0 if self.active else 0.2
        super(Camera, self).update_visualization()

    def gen_frustum_hull(self, task_params):
        """\
        Generate the points for this camera's frustum's hull for a given task.

        @param task_params: Task parameters.
        @type task_params: C{dict}
        @return: Frustum hull.
        @rtype: C{list} of C{tuple}
        """
        # Find the minimum and maximum depths of coverage in the frustum.
        z_lim = [max(self.zres(task_params['res_max'][1]),
                     self.zc(task_params['blur_max'][1] * \
                     min(self._params['s']))[0]),
                 min(self.zres(task_params['res_min'][1]),
                     self.zc(task_params['blur_max'][1] * \
                     min(self._params['s']))[1])]
        # No primitives if no coverage.
        if not z_lim[0] < z_lim[1]:
            return []
        # Otherwise, generate the hull.
        hull = []
        for z in z_lim:
            hull += [(self.fov['tahl'] * z, self.fov['tavt'] * z, z),
                     (self.fov['tahl'] * z, self.fov['tavb'] * z, z),
                     (self.fov['tahr'] * z, self.fov['tavb'] * z, z),
                     (self.fov['tahr'] * z, self.fov['tavt'] * z, z)]
        return hull

    def frustum_primitives(self, task_params):
        """\
        Generate the curve primitives for this camera's frustum for a given
        task.

        @param task_params: Task parameters.
        @type task_params: C{dict}
        @return: Frustum primitives.
        @rtype: C{list} of C{dict}
        """
        hull = self.gen_frustum_hull(task_params)
        if not hull:
            return []
        # Return the primitives based on the hull.
        return [{'type': 'curve', 'color': (1, 0, 0), 'pos': hull[i:i + 4] + \
            hull[i:i + 1]} for i in range(0, 16, 4)] + \
            [{'type': 'curve', 'color': (1, 0, 0),
            'pos': [hull[i], hull[i + 4]]} for i in range(4)]


class Model(dict):
    """\
    Multi-camera I{k}-ocular coverage strength model.

    A L{Model} object, at its core, is a dictionary of the scene, including the
    cameras. It maintains a set of keys which identify cameras, and provides a
    variety of coverage-related functions.
    """
    yaml = {'cameras': Camera, 'tasks': Task}

    # object types for which occlusion caching is handled by this class
    oc_sets = ['cameras']

    def __init__(self):
        """\
        Constructor.
        """
        dict.__init__(self)
        self.cameras = set()
        self._occlusion_cache = {}
        self._oc_updated = {}
        self._oc_needs_update = {}
        self._oc_mask = set()

    def __setitem__(self, key, value):
        # Mark occlusion cache for update.
        for ckey in self._occlusion_cache:
            self._oc_updated[ckey][key] = False
            self._oc_needs_update[ckey] = True
        # Register pose/parameter change callbacks.
        def callback():
            for ckey in self._occlusion_cache:
                self._oc_updated[ckey][key] = False
                self._oc_needs_update[ckey] = True
        value.posecallbacks['occlusion_cache'] = callback
        if hasattr(value, 'paramcallbacks'):
            value.paramcallbacks['occlusion_cache'] = callback
        # If this is a camera, add it to the camera set.
        if isinstance(value, Camera):
            self.cameras.add(key)
        super(Model, self).__setitem__(key, value)

    def __delitem__(self, key):
        self[key].visible = False
        self[key].__del__()
        self.cameras.discard(key)
        self._oc_mask.discard(key)
        super(Model, self).__delitem__(key)

    def __del__(self):
        for sceneobject in self:
            self[sceneobject].visible = False

    @property
    def active_cameras(self):
        """\
        The set of active cameras.

        @rtype: C{set}
        """
        return set([key for key in self.cameras if self[key].active])

    def views(self, ocular=1, subset=None):
        """\
        Return the set of I{k}-ocular views from the active cameras.

        @param ocular: The value of I{k}.
        @type ocular: C{int}
        @param subset: Subset of cameras (defaults to all active cameras).
        @type subset: C{set}
        @return: All I{k}-ocular views.
        @rtype: C{set} of C{frozenset} of C{str}
        """
        cameras = subset or self.active_cameras
        return set([frozenset(view) for view in combinations(cameras, ocular)])

    @property
    def oc_mask(self):
        """\
        The set of objects masked from the occlusion cache. All triangles for
        such objects will be checked for occlusion, regardless of whether they
        are candidates; however, they need not be checked for candidacy. In some
        cases, masking objects from the cache is a worthwhile tradeoff.

        @rtype: C{set}
        """
        return self._oc_mask

    def occlusion_cache_mask(self, sceneobject):
        """\
        Mask an object from inclusion in the occlusion cache.

        @param sceneobject: The object to mask.
        @type sceneobject: C{str}
        """
        if not sceneobject in self:
            raise KeyError(sceneobject)
        # Add the object to the mask.
        self._oc_mask.add(sceneobject)
        # Remove the object from existing cache.
        obj_set = set(reduce(lambda a, b: a | b, [getattr(self, oc_set) \
            for oc_set in self.oc_sets]))
        for ckey in self._occlusion_cache:
            for obj in obj_set:
                for triangle in self[sceneobject].triangles:
                    try:
                        del self._occlusion_cache[ckey][obj]\
                            [triangle.triangle]
                    except KeyError:
                        pass
            self._oc_updated[ckey][sceneobject] = True
        # Remove occlusion cache callbacks from object.
        try:
            del self[sceneobject].posecallbacks['occlusion_cache']
        except (AttributeError, KeyError):
            pass
        try:
            del self[sceneobject].paramcallbacks['occlusion_cache']
        except (AttributeError, KeyError):
            pass

    def occlusion_cache_unmask(self, sceneobject):
        """\
        Unmask an object for inclusion in the occlusion cache.

        @param sceneobject: The object to unmask.
        @type sceneobject: C{str}
        """
        # Remove the object from the mask.
        self._oc_mask.remove(sceneobject)
        # Mark the occlusion cache for update.
        for ckey in self._occlusion_cache:
            self._oc_updated[ckey][sceneobject] = False
            self._oc_needs_update[ckey] = True
        # Reinstate occlusion cache callbacks.
        def callback():
            for ckey in self._occlusion_cache:
                self._oc_updated[ckey][sceneobject] = False
                self._oc_needs_update[ckey] = True
        self[sceneobject].posecallbacks['occlusion_cache'] = callback
        if hasattr(self[sceneobject], 'paramcallbacks'):
            self[sceneobject].paramcallbacks['occlusion_cache'] = callback

    def _update_occlusion_cache(self, task_params=None):
        if task_params:
            # The key is a hash on the values defining the frustum depth.
            key = (task_params['res_min'][1], task_params['blur_max'][1])
        else:
            key = None
        if not key in self._occlusion_cache:
            # Build a new occlusion cache entry if necessary.
            self._occlusion_cache[key] = {}
            self._oc_updated[key] = {}
            for sceneobject in self:
                self._oc_updated[key][sceneobject] = False
            self._oc_needs_update[key] = True
        elif not self._oc_needs_update[key]:
            return key
        # Build a set of objects which can be occluded (e.g. cameras).
        obj_set = set(reduce(lambda a, b: a | b, [getattr(self, oc_set) \
            for oc_set in self.oc_sets]))
        # Update cache for all objects for any occludables needing update.
        for obj in set(obj_set):
            if not self._oc_updated[key][obj]:
                self._occlusion_cache[key][obj] = {}
                for sceneobject in self:
                    if sceneobject in self._oc_mask:
                        continue
                    if key is None:
                        for triangle in self[sceneobject].triangles:
                            self._occlusion_cache[key][obj]\
                                [triangle.triangle] = triangle.mapped_triangle()
                    else:
                        for triangle in self[sceneobject].triangles:
                            mt = triangle.mapped_triangle()
                            if self[obj].occluded_by(mt, task_params):
                                self._occlusion_cache[key][obj]\
                                    [triangle.triangle] = mt
                obj_set.remove(obj)
        # Update cache for all occludables for any objects needing update.
        for sceneobject in self:
            if sceneobject in self._oc_mask:
                continue
            if not self._oc_updated[key][sceneobject]:
                for obj in obj_set:
                    if key is None:
                        for triangle in self[sceneobject].triangles:
                            self._occlusion_cache[key][obj]\
                                [triangle.triangle] = triangle.mapped_triangle()
                    else:
                        for triangle in self[sceneobject].triangles:
                            mt = triangle.mapped_triangle()
                            if self[obj].occluded_by(mt, task_params):
                                self._occlusion_cache[key][obj]\
                                    [triangle.triangle] = mt
                            else:
                                try:
                                    del self._occlusion_cache[key][obj]\
                                        [triangle.triangle]
                                except KeyError:
                                    pass
                self._oc_updated[key][sceneobject] = True
        self._oc_needs_update[key] = False
        return key

    def occluded(self, point, obj, task_params=None, triangle_set=None):
        """\
        Return whether the specified point is occluded with respect to the
        specified object. If task parameters are specified, an occlusion cache
        is used.

        @param point: The point to check.
        @type point: L{Point}
        @param obj: The object ID to check.
        @type obj: C{str}
        @param task_params: Task parameters (optional).
        @type task_params: C{dict}
        @param triangle_set: Alternative triangle set to use.
        @type triangle_set: C{list} of L{Triangle}
        @return: True if occluded.
        @rtype: C{bool}
        """
        if triangle_set is None:
            key = self._update_occlusion_cache(task_params)
            triangle_set = self._occlusion_cache[key][obj].values()
        for triangle in triangle_set:
            if triangle.intersection(self[obj].pose.T, point, True):
                return True
        return False

    def strength(self, point, task_params, subset=None, triangle_set=None):
        """\
        Return the individual coverage strength of a point in the coverage
        strength model.

        @param point: The (directional) point to test.
        @type point: L{Point}
        @param task_params: Task parameters.
        @type task_params: C{dict}
        @param subset: Subset of cameras (defaults to all active cameras).
        @type subset: C{set}
        @param triangle_set: Set of additional triangles for occlusion.
        @type triangle_set: C{list} of L{Triangle}
        @return: The coverage strength of the point.
        @rtype: C{float}
        """
        maxstrength = 0.0
        # If there are no views, this loop does nothing and 0.0 is returned.
        for view in self.views(ocular=task_params['ocular'], subset=subset):
            # The view has at least one camera, so minstrength will be
            # overwritten with the minimum strength within the view.
            minstrength = float('inf')
            for camera in view:
                # Check the coverage strength for this camera.
                strength = self[camera].strength(point, task_params)
                # The following should short-circuit if strength = 0, and thus
                # not incur a performance hit for the occlusion check(s).
                if not strength or self.occluded(point, camera,
                task_params=task_params) or (triangle_set and
                self.occluded(point, camera, triangle_set=triangle_set)):
                    minstrength = 0.0
                    break
                elif strength < minstrength:
                    minstrength = strength
            # Update the maximum strength for any view.
            maxstrength = max(maxstrength, minstrength)
            # If the strength is 1.0, return it immediately.
            if maxstrength == 1.0:
                break
        return maxstrength

    def coverage(self, task, subset=None):
        """\
        Return the coverage model of this multi-camera network with respect to
        the points in a given task model.

        @param task: The task model.
        @type task: L{Task}
        @param subset: Subset of cameras (defaults to all active cameras).
        @type subset: C{set}
        @return: The coverage model.
        @rtype: L{PointCache}
        """
        coverage = PointCache()
        for point in task.mapped:
            # Calculate coverage strength for each mapped task point.
            coverage[point] = self.strength(point, task.params, subset)
        return coverage

    def performance(self, task, subset=None, coverage=None):
        """\
        Return the coverage performance of this multi-camera network with
        respect to a given task model. If a previously computed coverage cache
        is provided, it is assumed to contain the same points as the mapped
        task model.

        @param task: The task model.
        @type task: L{Task}
        @param subset: Subset of cameras (defaults to all active cameras).
        @type subset: C{set}
        @param coverage: Previously computed coverage cache for these points.
        @type coverage: L{PointCache}
        @return: Performance metric in [0, 1].
        @rtype: C{float}
        """
        coverage = coverage or self.coverage(task, subset)
        Fn, Fd = 0.0, 0.0
        for point in coverage.keys():
            Fn += coverage[point] * task.mapped[point]
            Fd += task.mapped[point]
        return Fn / Fd

    def performance_complex(self, task, subset=None, coverage=None):
        """\
        Return the coverage performance of this multi-camera network with
        respect to a given task model, with the relevance function interpreted
        as a complex feature.

        @param task: The task model.
        @type task: L{Task}
        @param subset: Subset of cameras (defaults to all active cameras).
        @type subset: C{set}
        @param coverage: Previously computed coverage cache for these points.
        @type coverage: L{PointCache}
        @return: Performance metric in [0, 1].
        @rtype: C{float}
        """
        coverage = coverage or self.coverage(task, subset)
        return min(coverage.values())

    def best_view(self, task, current=None, threshold=0, candidates=None):
        """\
        Return the best I{k}-view of the given task model. If the current (or
        previous) best view is specified, the hysteresis threshold adds some
        bias to that view to smooth the transition sequence. A restricted set of
        candidate views may be specified.

        @param task: The task model.
        @type task: L{Task}
        @param current: The current (previous) best view (optional).
        @type current: C{str}
        @param threshold: The hysteresis threshold in [0, 1] for transition.
        @type threshold: C{float}
        @param candidates: The set of candidate views (optional).
        @type candidates: C{set} of C{frozenset} of C{str}
        @return: The best view and its score.
        @rtype: C{frozenset} of C{str}, C{float}
        """
        if candidates:
            scores = dict.fromkeys(candidates)
        else:
            scores = dict.fromkeys(self.views(ocular=task.params['ocular']))
        for view in scores:
            scores[view] = self.performance(task, subset=view)
        if current and scores[current]:
            scores[current] += threshold
        best = max(scores.keys(), key=scores.__getitem__)
        if current and not scores[best]:
            return current, 0.0
        return best, scores[best] - (threshold if best == current else 0)

    def coverage_hypergraph(self, task, K=None):
        """\
        Return the coverage hypergraph of this multi-camera network. If C{K} is
        specified, return the I{K}-coverage hypergraph.

        @param task: The task model.
        @type task: L{Task}
        @param K: A set of possible hyperedge sizes.
        @type K: C{list} of C{int}
        @return: The coverage hypergraph.
        @rtype: C{Hypergraph}
        """
        if not HYPERGRAPH_ENABLED:
            raise ImportError('hypergraph module not loaded')
        active_cameras = self.active_cameras
        H = hypergraph.core.Hypergraph(vertices=active_cameras)
        if K is None:
            K = range(2, len(self) + 1)
        elif isinstance(K, int):
            K = [K]
        else:
            K.sort()
        cache = {}
        for camera in active_cameras:
            subset = frozenset([camera])
            cache[subset] = self.coverage(task, subset=subset)
            weight = sum(cache[subset].values())
            if weight:
                H.add_edge(hypergraph.core.Edge(subset), weight=weight)
        pk = 1
        for k in K:
            if k < 2:
                continue
            for subset in combinations(active_cameras, k):
                if not all([frozenset(sc) in cache \
                for sc in combinations(subset, pk)]):
                    continue
                subset = frozenset(subset)
                sc = set(subset)
                ec = set()
                for i in range(k - pk):
                    ec.add(sc.pop())
                cache[subset] = cache[frozenset(sc)]
                while ec:
                    cache[subset] &= cache[frozenset([ec.pop()])]
                weight = sum(cache[subset].values())
                if weight > 1e-4:
                    H.add_edge(hypergraph.core.Edge(subset), weight=weight)
                else:
                    del cache[subset]
            pk = k
        return H

    def visualize(self):
        """\
        Visualize all cameras and the directional points of the coverage model
        (with opacity reflecting degree of coverage).

        @return: True if the visualization was initialized for the first time.
        @rtype: C{bool}
        """
        for sceneobject in self:
            self[sceneobject].visualize()

    def update_visualization(self):
        """\
        Update the visualization.
        """
        for sceneobject in self:
            self[sceneobject].update_visualization()
