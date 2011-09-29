"""\
Coverage model module. Contains the coverage strength model classes for single
cameras and multi-camera networks, as well as the scene and relevance map
classes.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from math import sqrt, sin, cos, atan, pi
from numbers import Number
from itertools import combinations

try:
    import hypergraph
except ImportError:
    hypergraph = None

import cython
from .geometry import Point, DirectionalPoint, Pose
from .posable import Posable, SceneObject
from .visualization import Visualizable


class PointCache(dict):
    """\
    Point cache class.
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

    def visualize(self):
        """\
        Visualize the point cache, with opacity representing coverage strength.
        """
        try:
            self.visual.visible = False
            del self.visual
        except AttributeError:
            pass
        primitives = []
        for point in set([point for point in self if self[point]]):
            primitives.append({'type': 'sphere', 'pos': point[:3], 'radius': 3,
                'color': [1, 0, 0], 'opacity': self[point]})
            try:
                primitives.append({'type': 'arrow', 'pos': point[:3],
                    'axis': point.direction_unit * 30, 'color': [1, 0, 0],
                    'opacity': self[point]})
            except AttributeError:
                pass
        self.visual = Visualizable(primitives=primitives)
        self.visual.visualize()


class RelevanceModel(Posable):
    """\
    Relevance model class.
    """
    def __init__(self, original, pose=Pose(), mount=None):
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
        The pose of the object.
        """
        if self.mount:
            return self._pose + self.mount.mount_pose()
        else:
            return self._pose

    def visualize(self):
        self.mapped.visualize()


class Camera(SceneObject):
    """\
    Single-camera coverage strength model.
    """
    def __init__(self, params, pose=Pose(), mount=None, primitives=[],
                 active=True):
        """\
        Constructor.

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
        super(Camera, self).__init__(pose=pose, mount=mount, primitives=primitives)
        if isinstance(params['s'], Number):
            params['s'] = (params['s'], params['s'])
        self._params = params
        self._generate_cv()
        self._generate_cr()
        self._generate_cf()
        self._generate_cd()
        self._generate_fovvis()
        self.active = active

    def _pose_changed_hook(self):
        try:
            del self._fov_hull
        except AttributeError:
            pass
        super(Camera, self)._pose_changed_hook()

    def getparam(self, param):
        """\
        Retrieve a camera parameter from this camera.

        @param param: The name of the parameter to retrieve.
        @type param: C{str}
        @return: The value of the parameter.
        @rtype: C{object}
        """
        return self._params[param]

    def setparam(self, param, value):
        """\
        Set a camera parameter on this camera.

        @param param: The name of the paramater to set.
        @type param: C{str}
        @param value: The value to which to set the parameter.
        @type value: C{object}
        """
        self._params[param] = value
        fov = False
        fovvis = False
        if param in ['f', 's', 'dim']:
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
        ama = self.getparam('angle_max_acceptable')
        if self.getparam('angle_max_ideal') == \
           self.getparam('angle_max_acceptable'):
            cdval = lambda rho, a, b: float(float(rho) - (a * b) - pi + ama > 0)
        else:
            ami = self.getparam('angle_max_ideal')
            cdval = lambda rho, a, b: min(max((float(rho) - (a * b) - pi + ama) / (ama - ami), 0.0), 1.0)
        def Cd(p):
            if not isinstance(p, DirectionalPoint):
                return 1.0
            r = sqrt(p.x ** 2 + p.y ** 2)
            try:
                terma = (p.y / r) * sin(p.eta) + (p.x / r) * cos(p.eta)
            except ZeroDivisionError:
                terma = 1.0
            try:
                termb = atan(r / p.z)
            except ZeroDivisionError:
                termb = pi / 2.0
            return cdval(p.rho, terma, termb)
        self.Cd = Cd

    @property
    def fov(self):
        """\
        Pre-computed angles of view and the size of the field of view at z = 1.

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

    def occluded_by(self, plane):
        """\
        Return whether this camera's field of view is occluded (in part) by the
        specified plane.

        @param plane: The plane to check.
        @type plane: L{Plane}
        @return: True if occluded.
        @rtype: C{bool}
        """
        # FIXME: check for bounded plane intersection with viewing frustum
        hull = set([self.pose.map(point) for point in self.fov_hull])
        sign = cmp((-plane.pose).map(hull.pop()).z, 0)
        for point in hull:
            if cmp((-plane.pose).map(point).z, 0) != sign:
                print('true')
                return True
        print('false')
        return False

    def strength(self, point):
        """\
        Return the coverage strength for a directional point.
    
        @param point: The (directional) point to test.
        @type point: L{Point}
        @return: The coverage strength of the point.
        @rtype: C{float}
        """
        cp = (-self.pose).map(point)
        return self.Cv(cp) * self.Cr(cp) * self.Cf(cp) * self.Cd(cp)

    def image(self, point):
        """\
        Return the projected pixel coordinates of the specified 3D point.

        @param point: The point to project.
        @type point: L{Point}
        @return: The pixel coordinates of the point (if any).
        @rtype: C{tuple} of C{int}
        """
        cp = (-self.pose).map(point)
        if self.Cv(cp):
            return tuple([int((self.getparam('f') / self.getparam('s')[i]) * \
                (cp[i] / cp.z) + self.getparam('o')[i]) for i in range(2)])
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
    Multi-camera n-ocular coverage strength model.
    """
    def __init__(self):
        """\
        Constructor.
        """
        dict.__init__(self)
        self.cameras = set()
        self._occlusion_cache = {}
        self._oc_updated = {}
        self._oc_needs_update = True

    def __setitem__(self, key, value):
        self._oc_updated[key] = False
        def callback():
            self._oc_updated[key] = False
            self._oc_needs_update = True
        value.posecallbacks.add(callback)
        if isinstance(value, Camera):
            self.cameras.add(key)
        super(Model, self).__setitem__(key, value)

    def __delitem__(self, key):
        self[key].visible = False
        self.cameras.discard(key)
        super(Model, self).__delitem__(key)

    def __del__(self):
        for sceneobject in self:
            self[sceneobject].visible = False

    @property
    def active_cameras(self):
        """\
        Return a list of active cameras.

        @rtype: C{set}
        """
        return set([key for key in self.cameras if self[key].active])

    def _update_occlusion_cache(self):
        """\
        Update the occlusion cache.
        """
        if not self._oc_needs_update:
            return
        remainder = set(self.cameras)
        for camera in self.cameras:
            if not self._oc_updated[camera]:
                remainder.remove(camera)
                self._occlusion_cache[camera] = {}
                for sceneobject in self:
                    self._occlusion_cache[camera][sceneobject] = \
                        set([plane for plane in self[sceneobject].planes \
                             if self[camera].occluded_by(plane)])
        for sceneobject in self:
            if not self._oc_updated[sceneobject]:
                for camera in remainder:
                    self._occlusion_cache[camera][sceneobject] = \
                        set([plane for plane in self[sceneobject].planes \
                             if self[camera].occluded_by(plane)])
                self._oc_updated[sceneobject] = True
        self._oc_needs_update = False

    def occluded(self, point, camera):
        """\
        Return whether the specified point is occluded with respect to the
        specified camera, using the occlusion cache planes.

        @param point: The point to check.
        @type point: L{Point}
        @param camera: The camera ID to check.
        @type camera: C{str}
        @return: True if occluded.
        @rtype: C{bool}
        """
        for plane in set([plane for plane in \
            [self._occlusion_cache[camera][sceneobject] \
            for sceneobject in self] for plane in plane]):
            intersection = plane.intersection(point, self[camera].pose.T)
            if intersection is not None and intersection.euclidean(point) > 1e-4:
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
            raise ValueError('network has too few active cameras')
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

    def coverage_hypergraph(self, relevance, K=None):
        """\
        Return the coverage hypergraph of this multi-camera network. If K is
        specified, return the K-coverage hypergraph.

        @param relevance: The relevance model.
        @type relevance: L{RelevanceModel}
        @param K: A set of possible hyperedge sizes.
        @type K: C{list} of C{int}
        @return: The coverage hypergraph.
        @rtype: L{hypergraph.Hypergraph}
        """
        if not hypergraph:
            raise ImportError('hypergraph module not loaded')
        H = hypergraph.Hypergraph(vertices=self.keys())
        if K is None:
            K = range(2, len(self) + 1)
        else:
            K.sort()
        active_cameras = self.active_cameras
        cache = {}
        for camera in active_cameras:
            subset = frozenset([camera])
            cache[subset] = self.coverage(relevance, subset=subset)
            weight = sum(cache[subset].values())
            if weight:
                H.add_edge(hypergraph.Edge(subset), weight=weight)
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
                    H.add_edge(hypergraph.Edge(subset), weight=weight)
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
