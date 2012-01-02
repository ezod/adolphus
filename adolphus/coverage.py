"""\
Coverage model module. Contains the coverage strength model classes for single
cameras and multi-camera networks, as well as point cache and relevance model
classes.

The available task parameters are:

    - C{boundary_padding}: width of degradation boundary around image (pixels).
    - C{res_max_ideal}: upper limit of ideal resolution range (mm/pixel).
    - C{res_max_acceptable}: maximum acceptable resolution (mm/pixel).
    - C{res_min_ideal}: lower limit of ideal resolution range (mm/pixel).
    - C{res_min_acceptable}: minimum acceptable resolution (mm/pixel).
    - C{blur_max_ideal}: ideal maximum blur circle diameter (pixels).
    - C{blur_max_acceptable}: maximum acceptable blur circle diameter (pixels).
    - C{angle_max_ideal}: ideal maximum view angle (radians).
    - C{angle_max_acceptable}: maximum acceptable view angle (radians).

See L{TP_DEFAULTS} for default (permissive) task parameter values.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from math import sin, cos, atan, pi
from numbers import Number
from itertools import combinations

try:
    import hypergraph
except ImportError:
    hypergraph = None

from .geometry import Point, Pose, Face, which_side
from .posable import Posable, SceneObject
from .visualization import Visualizable


# TODO: make this specific to the model class?
TP_DEFAULTS = {'boundary_padding': 0.0,
               'res_max_ideal': float('inf'),
               'res_max_acceptable': float('inf'),
               'res_min_ideal': 0.0,
               'res_min_acceptable': 0.0,
               'blur_max_ideal': 1.0,
               'blur_max_acceptable': 1.0,
               'angle_max_ideal': pi / 2.0,
               'angle_max_acceptable': pi / 2.0}


class PointCache(dict):
    """\
    Point cache class.

    The L{PointCache} object attempts to improve the efficiency of operations
    involving relevance models and discrete coverage functions. It provides the
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
        result = self.__class__(dl)
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
        result = self.__class__(dl)
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

    def visualize(self, color=(1, 0, 0), scale=1.0):
        """\
        Visualize the point cache, with opacity representing coverage strength.

        @param color: The color of the points.
        @type color: C{tuple} of C{float}
        @param scale: The relative scale of the points.
        @type scale: C{float}
        """
        try:
            self.visual.visible = False
            del self.visual
        except AttributeError:
            pass
        primitives = []
        for point in set([point for point in self if self[point]]):
            primitives.append({'type': 'sphere', 'pos': point[:3],
                'radius': 3 * scale, 'color': color, 'opacity': self[point]})
            try:
                primitives.append({'type': 'arrow', 'pos': point[:3],
                    'axis': point.direction_unit * 30 * scale, 'color': color,
                    'opacity': self[point]})
            except AttributeError:
                pass
        self.visual = Visualizable(primitives=primitives)
        self.visual.visualize()


class RelevanceModel(Posable):
    """\
    Relevance model class.

    A L{RelevanceModel} object is functionally a posable set of discrete
    relevance function points. In practice it manages two L{PointCache}
    objects: a static original and a volatile version mapped through the pose.
    """
    def __init__(self, original, pose=Pose(), mount=None):
        """\
        Constructor.

        @param original: The original set of relevance model points.
        @type original: L{PointCache}
        @param pose: The pose of this relevance model.
        @type pose: L{Pose}
        @param mount: The mount of this relevance model.
        @type mount: L{SceneObject}
        """
        super(RelevanceModel, self).__init__(pose=pose, mount=mount)
        self._original = original

    def _pose_changed_hook(self):
        """\
        Hook called on pose change.
        """
        try:
            del self._mapped
        except AttributeError:
            pass
        super(RelevanceModel, self)._pose_changed_hook()

    @property
    def original(self):
        """\
        Original (unmapped) relevance model points.
        """
        return self._original

    @property
    def mapped(self):
        """\
        Actual (mapped) relevance model points.
        """
        try:
            return self._mapped
        except AttributeError:
            self._mapped = PointCache()
            for point in self.original:
                self._mapped[self.pose.map(point)] = self.original[point]
            return self._mapped

    @property
    def pose(self):
        """\
        The pose of the relevance model.
        """
        if self.mount:
            return self._pose + self.mount.mount_pose()
        else:
            return self._pose

    def visualize(self):
        """\
        Visualize the relevance model.
        """
        self.mapped.visualize()


class Camera(SceneObject):
    """\
    Single-camera coverage strength model.

    A L{Camera} object implements the coverage function for a single camera
    based on camera and task parameters. Being a L{SceneObject} itself, it also
    provides the usual functionality of pose, visualization, and occlusion.
    """
    def __init__(self, name, params, pose=Pose(), mount=None, primitives=[],
                 active=True):
        """\
        Constructor.

        @param name: The name of the camera.
        @type name: C{str}
        @param params: Dictionary of application parameters.
        @type params: C{dict}
        @param pose: Pose of the camera in space (optional).
        @type pose: L{Pose}
        @param mount: Mount object for the camera (optional).
        @type mount: C{object}
        @param primitives: Sprite primitives for the camera.
        @type primitives: C{dict}
        @param active: Initial active state of camera (optional).
        @type active: C{bool}
        """
        super(Camera, self).__init__(name, pose=pose, mount=mount,
            primitives=primitives)
        if isinstance(params['s'], Number):
            params['s'] = (params['s'], params['s'])
        self._params = params
        self._generate_cv()
        self._generate_cr()
        self._generate_cf()
        self._generate_cd()
        self._generate_fovvis()
        self.active = active
        self.click_actions = {'none':   'active %s' % name,
                              'ctrl':   'fov %s' % name,
                              'alt':    'cameraview %s' % name,
                              'shift':  'modify %s' % name}

    def _delete_fov_data(self):
        try:
            del self._fov_hull_vertices
        except AttributeError:
            pass
        try:
            del self._fov_hull_edges
        except AttributeError:
            pass
        try:
            del self._fov_hull_faces
        except AttributeError:
            pass

    def _pose_changed_hook(self):
        """\
        Hook called on pose change.
        """
        self._delete_fov_data()
        super(Camera, self)._pose_changed_hook()

    def getparam(self, param):
        """\
        Retrieve a camera parameter from this camera.

        @param param: The name of the parameter to retrieve.
        @type param: C{str}
        @return: The value of the parameter.
        @rtype: C{object}
        """
        try:
            return self._params[param]
        except KeyError:
            return TP_DEFAULTS[param]

    def setparam(self, param, value):
        """\
        Set a camera parameter on this camera.

        @param param: The name of the paramater to set.
        @type param: C{str}
        @param value: The value to which to set the parameter.
        @type value: C{object}
        """
        if not param in self._params and not param in TP_DEFAULTS:
            raise KeyError('invalid parameter %s' % param)
        self._params[param] = value
        fov = False
        fovvis = False
        if param in ['f', 's', 'o', 'dim']:
            try:
                del self._fov
            except AttributeError:
                pass
            fov = True
            fovvis = True
        if fov or param == 'boundary_padding':
            self._generate_cv()
        if fov or param.startswith('res'):
            self._generate_cr()
            fovvis = True
        if param in ['A', 'zS', 'f', 's'] or param.startswith('blur'):
            self._generate_cf()
            fovvis = True
        if param.startswith('angle'):
            self._generate_cd()
        if fovvis:
            try:
                del self._fov_hull
            except AttributeError:
                pass
            self._delete_fov_data()
            self._generate_fovvis()

    def _generate_cv(self):
        sahl = self.fov['sahl']
        sahr = self.fov['sahr']
        savt = self.fov['savt']
        savb = self.fov['savb']
        if not self.getparam('boundary_padding'):
            self.Cv = lambda p: p.z > 0 and \
                float(p.x / p.z > sahl and p.x / p.z < sahr and \
                      p.y / p.z > savt and p.y / p.z < savb) or 0.0
        else:
            gh = (self.getparam('boundary_padding') / float(\
                self.getparam('dim')[0])) * 2.0 * sin(self.fov['ah'] / 2.0)
            gv = (self.getparam('boundary_padding') / float(\
                self.getparam('dim')[1])) * 2.0 * sin(self.fov['av'] / 2.0)
            self.Cv = lambda p: p.z > 0 and min(min(max((min(p.x / p.z - sahl,
                sahr - p.x / p.z) / gh), 0.0), 1.0), min(max((min(p.y / p.z \
                - savt, savb - p.y / p.z) / gv), 0.0), 1.0)) or 0.0

    def _generate_cr(self):
        mr = min([float(self.getparam('dim')[0]) \
            / (2 * sin(self.fov['ah'] / 2.0)), float(self.getparam('dim')[1]) \
            / (2 * sin(self.fov['av'] / 2.0))])
        zrmaxi = (1.0 / self.getparam('res_max_ideal')) * mr
        zrmaxa = (1.0 / self.getparam('res_max_acceptable')) * mr
        try:
            zrmini = (1.0 / self.getparam('res_min_ideal')) * mr
        except ZeroDivisionError:
            zrmini = float('inf')
        try:
            zrmina = (1.0 / self.getparam('res_min_acceptable')) * mr
        except ZeroDivisionError:
            zrmina = float('inf')
        if zrmaxa == zrmaxi and zrmina == zrmini:
            self.Cr = lambda p: float(p.z > zrmaxa and p.z < zrmina)
        elif zrmaxa == zrmaxi:
            self.Cr = lambda p: p.z > zrmaxa and min(max((zrmina - p.z) / \
                (zrmina - zrmini), 0.0), 1.0) or 0.0
        elif zrmina == zrmini:
            self.Cr = lambda p: p.z < zrmina and min(max((p.z - zrmaxa) / \
                (zrmaxi - zrmaxa), 0.0), 1.0) or 0.0
        else:
            self.Cr = lambda p: min(max(min((p.z - zrmaxa) / (zrmaxi - zrmaxa),
                (zrmina - p.z) / (zrmina - zrmini)), 0.0), 1.0)
        self._zrmaxa, self._zrmina = zrmaxa, zrmina

    def _generate_cf(self):
        zn, zf = self.zc(self.getparam('blur_max_acceptable') \
            * min(self.getparam('s')))
        if self.getparam('blur_max_ideal') == \
           self.getparam('blur_max_acceptable'):
            self.Cf = lambda p: float(p.z > zn and p.z < zf)
        else:
            zl, zr = self.zc(self.getparam('blur_max_ideal') \
                * min(self.getparam('s')))
            self.Cf = lambda p: min(max(min((p.z - zn) / (zl - zn),
                (zf - p.z) / (zf - zr)), 0.0), 1.0)
        self._zn, self._zf = zn, zf

    def _generate_cd(self):
        aa = cos(self.getparam('angle_max_acceptable'))
        if self.getparam('angle_max_ideal') == \
           self.getparam('angle_max_acceptable'):
            cdval = lambda sigma: float(sigma > aa)
        else:
            ai = cos(self.getparam('angle_max_ideal'))
            cdval = lambda sigma: min(max((sigma - aa) / (ai - aa), 0.0), 1.0)
        def Cd(p):
            try:
                sigma = cos(p.direction_unit.angle(-p))
            except (ValueError, AttributeError):
                # p is at origin or not a directional point
                return 1.0
            else:
                return cdval(sigma)
        self.Cd = Cd

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
            # horizontal
            self._fov['ahl'] = 2.0 * atan((self.getparam('o')[0] * \
                self.getparam('s')[0]) / (2.0 * self.getparam('f')))
            self._fov['ahr'] = 2.0 * atan(((self.getparam('dim')[0] - \
                self.getparam('o')[0]) * self.getparam('s')[0]) / \
                (2.0 * self.getparam('f')))
            self._fov['ah'] = self._fov['ahl'] + self._fov['ahr']
            self._fov['sahl'] = -sin(self._fov['ahl'])
            self._fov['sahr'] = sin(self._fov['ahr'])
            self._fov['sah'] = self._fov['sahr'] - self._fov['sahl']
            # vertical
            self._fov['avt'] = 2.0 * atan((self.getparam('o')[1] * \
                self.getparam('s')[1]) / (2.0 * self.getparam('f')))
            self._fov['avb'] = 2.0 * atan(((self.getparam('dim')[1] - \
                self.getparam('o')[1]) * self.getparam('s')[1]) / \
                (2.0 * self.getparam('f')))
            self._fov['av'] = self._fov['avt'] + self._fov['avb']
            self._fov['savt'] = -sin(self._fov['avt'])
            self._fov['savb'] = sin(self._fov['avb'])
            self._fov['sav'] = self._fov['savb'] - self._fov['savt']
            return self._fov

    @property
    def fov_hull(self):
        """\
        Vertices of the viewing frustum (region of 3-space in which coverage is
        nonzero before occlusion).
        """
        try:
            return self._fov_hull
        except AttributeError:
            self._fov_hull = []
            for z in [max(self._zn, self._zrmaxa), min(self._zf, self._zrmina)]:
                self._fov_hull += \
                    [Point((self.fov['sahl'] * z, self.fov['savt'] * z, z)),
                     Point((self.fov['sahl'] * z, self.fov['savb'] * z, z)),
                     Point((self.fov['sahr'] * z, self.fov['savb'] * z, z)),
                     Point((self.fov['sahr'] * z, self.fov['savt'] * z, z))]
            return self._fov_hull

    @property
    def fov_hull_vertices(self):
        try:
            return self._fov_hull_vertices
        except AttributeError:
            self._fov_hull_vertices = [self.pose.T] + \
                [self.pose.map(v) for v in self.fov_hull[4:]]
            return self._fov_hull_vertices

    @property
    def fov_hull_edges(self):
        try:
            return self._fov_hull_edges
        except AttributeError:
            H = self.fov_hull_vertices
            self._fov_hull_edges = [(H[i] - H[0], H[0]) for i in range(1, 5)] \
                + [(H[(i + 1) % 4 + 1] - H[i + 1], H[i + 1]) for i in range(4)]
            return self._fov_hull_edges

    @property
    def fov_hull_faces(self):
        try:
            return self._fov_hull_faces
        except AttributeError:
            H = self.fov_hull_vertices
            self._fov_hull_faces = [Face([H[4], H[3], H[2], H[1]])] \
                + [Face([H[0], H[i + 1], H[(i + 1) % 4 + 1]]) for i in range(4)]
            return self._fov_hull_faces

    def _generate_fovvis(self):
        self.fovvis = [{'type': 'curve', 'color': (1, 0, 0), 'pos': \
            self.fov_hull[i:i + 4] + self.fov_hull[i:i + 1]} for i in \
            range(0, 16, 4)] + [{'type': 'curve', 'color': (1, 0, 0),
            'pos': [self.fov_hull[i], self.fov_hull[i + 4]]} for i in range(4)]

    def zc(self, c):
        """\
        Return the depth values at which a circle of confusion of a given size
        occurs.

        @param c: The diameter of the circle of confusion.
        @type c: C{float}
        @return: Two depth values.
        @rtype: C{tuple} of C{float}
        """
        r = []
        for s in [1, -1]:
            r.append((self.getparam('A') * self.getparam('f') \
                * self.getparam('zS')) / (self.getparam('A') \
                * self.getparam('f') + s * c * (self.getparam('zS') \
                - self.getparam('f'))))
        if r[1] < 0:
            r[1] = float('inf')
        return tuple(r)

    def occluded_by(self, triangle):
        """\
        Return whether this camera's field of view is occluded (in part) by the
        specified triangle.

            - D. Eberly, "Intersection of Convex Objects: The Method of
              Separating Axes," 2008.

        @param triangle: The triangle to check.
        @type triangle: L{OcclusionTriangle}
        @return: True if occluded.
        @rtype: C{bool}
        """
        for face in self.fov_hull_faces:
            if which_side(triangle.mapped_triangle.vertices, face.normal,
                face.vertices[0]) > 0:
                return False
        if which_side(self.fov_hull_vertices, triangle.mapped_triangle.normal,
            triangle.mapped_triangle.vertices[0]) > 0:
            return False
        for fedge, fvertex in self.fov_hull_edges:
            for i in range(3):
                direction = fedge ** triangle.mapped_triangle.edges[i]
                side0 = which_side(self.fov_hull_vertices, direction, fvertex)
                if side0 == 0:
                    continue
                side1 = which_side(triangle.mapped_triangle.vertices, direction,
                    fvertex)
                if side1 == 0:
                    continue
                if side0 * side1 < 0:
                    return False
        return True

    def strength(self, point):
        """\
        Return the coverage strength for a directional point. Note that since
        the L{Camera} object is not internally aware of the scene it inhabits,
        occlusion is computed in the L{Model} object instead.

        @param point: The (directional) point to test.
        @type point: L{Point}
        @return: The coverage strength of the point.
        @rtype: C{float}
        """
        cp = (-self.pose).map(point)
        return self.Cv(cp) * self.Cr(cp) * self.Cf(cp) * self.Cd(cp)

    def image(self, point):
        """\
        Return the projected subpixel coordinates of the specified 3D point.

        @param point: The point to project.
        @type point: L{Point}
        @return: The subpixel coordinates of the point (if any).
        @rtype: C{tuple} of C{float}
        """
        cp = (-self.pose).map(point)
        if self.Cv(cp):
            return tuple([(self.getparam('f') / self.getparam('s')[i]) * \
                (cp[i] / cp.z) + self.getparam('o')[i] for i in range(2)])
        else:
            return None

    def update_visualization(self):
        """\
        Update the visualization for camera active state and pose.
        """
        self.opacity = self.active and 1.0 or 0.2
        super(Camera, self).update_visualization()


class Model(dict):
    """\
    Multi-camera I{k}-ocular coverage strength model.

    A L{Model} object, at its core, is a dictionary of the scene, including the
    cameras. It maintains a set of keys which identify cameras, and provides a
    variety of coverage-related functions.
    """
    camera_class = Camera

    def __init__(self, task_params=dict()):
        """\
        Constructor.

        @param task_params: Task parameters.
        @type task_params: C{dict}
        """
        dict.__init__(self)
        self.cameras = set()
        self._occlusion_cache = {}
        self._oc_updated = {}
        self._oc_needs_update = True
        self._task_params = task_params

    def __setitem__(self, key, value):
        self._oc_updated[key] = False
        def callback():
            self._oc_updated[key] = False
            self._oc_needs_update = True
        value.posecallbacks.add(callback)
        if isinstance(value, Camera):
            self.cameras.add(key)
            for param in self._task_params:
                value.setparam(param, self._task_params[param])
        super(Model, self).__setitem__(key, value)

    def __delitem__(self, key):
        self[key].visible = False
        self.cameras.discard(key)
        super(Model, self).__delitem__(key)

    def __del__(self):
        for sceneobject in self:
            self[sceneobject].visible = False

    def getparam(self, param):
        """\
        Retrieve a task parameter from this model.

        @param param: The name of the task parameter to retrieve.
        @type param: C{str}
        @return: The value of the parameter.
        @rtype: C{object}
        """
        try:
            return self._task_params[param]
        except KeyError:
            return TP_DEFAULTS[param]

    def setparam(self, param, value):
        """\
        Set a task parameter on this model.

        @param param: The name of the task paramater to set.
        @type param: C{str}
        @param value: The value to which to set the parameter.
        @type value: C{object}
        """
        if not param in TP_DEFAULTS:
            raise KeyError('invalid parameter')
        self._task_params[param] = value
        for camera in self.cameras:
            self[camera].setparam(param, value)

    @property
    def active_cameras(self):
        """\
        Return the set of active cameras.

        @rtype: C{set}
        """
        return set([key for key in self.cameras if self[key].active])

    def views(self, ocular=1):
        """\
        Return the set of I{k}-ocular views from the active cameras.

        @param ocular: The value of I{k}.
        @type ocular: C{int}
        @return: All I{k}-ocular views.
        @rtype: C{set} of C{frozenset} of C{str}
        """
        return set([frozenset(view) \
            for view in combinations(self.active_cameras, ocular)])

    def _update_occlusion_cache(self):
        if not self._oc_needs_update:
            return
        remainder = set(self.cameras)
        for camera in self.cameras:
            if not self._oc_updated[camera]:
                remainder.remove(camera)
                self._occlusion_cache[camera] = {}
                for sceneobject in self:
                    self._occlusion_cache[camera][sceneobject] = \
                        set([triangle for triangle \
                            in self[sceneobject].triangles \
                            if self[camera].occluded_by(triangle)])
        for sceneobject in self:
            if not self._oc_updated[sceneobject]:
                for camera in remainder:
                    self._occlusion_cache[camera][sceneobject] = \
                        set([triangle for triangle \
                            in self[sceneobject].triangles \
                            if self[camera].occluded_by(triangle)])
                self._oc_updated[sceneobject] = True
        self._oc_needs_update = False

    def occluded(self, point, camera):
        """\
        Return whether the specified point is occluded with respect to the
        specified camera, using the occlusion cache triangles.

        @param point: The point to check.
        @type point: L{Point}
        @param camera: The camera ID to check.
        @type camera: C{str}
        @return: True if occluded.
        @rtype: C{bool}
        """
        for triangle in set([t for ts in [self._occlusion_cache[camera]\
            [sceneobject] for sceneobject in self] for t in ts]):
            if triangle.intersection(self[camera].pose.T, point):
                return True
        return False

    def strength(self, point, ocular=1, subset=None):
        """\
        Return the individual coverage strength of a point in the coverage
        strength model.

        @param point: The (directional) point to test.
        @type point: L{Point}
        @param ocular: Mutual camera coverage degree.
        @type ocular: C{int}
        @param subset: Subset of cameras (defaults to all active cameras).
        @type subset: C{set}
        @return: The coverage strength of the point.
        @rtype: C{float}
        """
        active_cameras = subset or self.active_cameras
        self._update_occlusion_cache()
        if len(active_cameras) < ocular:
            raise ValueError('too few active cameras')
        maxstrength = 0.0
        for combination in combinations(active_cameras, ocular):
            minstrength = float('inf')
            for camera in combination:
                strength = self[camera].strength(point)
                if not strength:
                    minstrength = 0.0
                    break
                elif strength < minstrength:
                    minstrength = strength * (not self.occluded(point, camera))
            if minstrength > 1.0:
                minstrength = 0.0
            maxstrength = max(maxstrength, minstrength)
        return maxstrength

    def coverage(self, relevance, ocular=1, subset=None):
        """\
        Return the coverage model of this multi-camera network with respect to
        the points in a given relevance model.

        @param relevance: The relevance model.
        @type relevance: L{RelevanceModel}
        @param ocular: Mutual camera coverage degree.
        @type ocular: C{int}
        @param subset: Subset of cameras (defaults to all active cameras).
        @type subset: C{set}
        @return: The coverage model.
        @rtype: L{PointCache}
        """
        coverage = PointCache()
        for point in relevance.mapped:
            coverage[point] = self.strength(point, ocular, subset)
        return coverage

    def performance(self, relevance, ocular=1, subset=None, coverage=None):
        """\
        Return the coverage performance of this multi-camera network with
        respect to a given relevance model. If a previously computed coverage
        cache is provided, it should be for the same point set as used by the
        relevance model.

        @param relevance: The relevance model.
        @type relevance: L{RelevanceModel}
        @param ocular: Mutual camera coverage degree.
        @type ocular: C{int}
        @param subset: Subset of cameras (defaults to all active cameras).
        @type subset: C{set}
        @param coverage: Previously computed coverage cache for these points.
        @type coverage: L{PointCache}
        @return: Performance metric in [0, 1].
        @rtype: C{float}
        """
        coverage = coverage or self.coverage(relevance, ocular, subset)
        return sum((coverage & relevance.mapped).values()) \
            / sum(relevance.mapped.values())

    def best_view(self, relevance, ocular=1, current=None, threshold=0,
                  candidates=None):
        """\
        Return the best I{k}-view of the given relevance model. If the current
        (or previous) best view is specified, the hysteresis threshold adds some
        bias to that view to smooth the transition sequence. A restricted set of
        candidate views may be specified.

        @param relevance: The relevance model.
        @type relevance: L{RelevanceModel}
        @param ocular: Mutual camera coverage degree.
        @type ocular: C{int}
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
            scores = dict.fromkeys(self.views(ocular=ocular))
        for view in scores:
            scores[view] = self.performance(relevance, subset=view)
        if current and scores[current]:
            scores[current] += threshold
        best = max(scores.keys(), key=scores.__getitem__)
        if current and not scores[best]:
            return current, 0.0
        return best, scores[best] - (best == current and threshold or 0)

    def coverage_hypergraph(self, relevance, K=None):
        """\
        Return the coverage hypergraph of this multi-camera network. If C{K} is
        specified, return the I{K}-coverage hypergraph.

        @param relevance: The relevance model.
        @type relevance: L{RelevanceModel}
        @param K: A set of possible hyperedge sizes.
        @type K: C{list} of C{int}
        @return: The coverage hypergraph.
        @rtype: C{Hypergraph}
        """
        if not hypergraph:
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
            cache[subset] = self.coverage(relevance, subset=subset)
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
