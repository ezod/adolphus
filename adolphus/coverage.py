"""\
Coverage model module. Contains the coverage strength model classes for single
cameras and multi-camera networks, as well as the scene and relevance map
classes.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import pyximport; pyximport.install()

from math import sqrt, sin, cos, atan, pi
from numbers import Number
from itertools import combinations

from hypergraph import Hypergraph, Edge

from .geometry import Point, DirectionalPoint, Pose
from .posable import Posable
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

    @property
    def original(self):
        return self._original

    @property
    def mapped(self):
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

    @pose.setter
    def pose(self, value):
        """\
        Set the pose of the object.
        """
        self._pose = value
        del self._mapped

    def visualize(self):
        self.mapped.visualize()


class Camera(Posable, Visualizable):
    """\
    Single-camera coverage strength model.
    """
    def __init__(self, params, pose=Pose(), mount=None, sprites=[],
                 active=True):
        """\
        Constructor.

        @param params: Dictionary of application parameters.
        @type params: C{dict}
        @param pose: Pose of the camera in space (optional).
        @type pose: L{Pose}
        @param mount: Mount object for the camera (optional).
        @type mount: C{object}
        @param sprites: Sprite primitives for the object.
        @type sprites: C{dict}
        @param active: Initial active state of camera (optional).
        @type active: C{bool}
        """
        Posable.__init__(self, pose=pose, mount=mount)
        Visualizable.__init__(self, sprites)
        if isinstance(params['s'], Number):
            params['s'] = (params['s'], params['s'])
        self.params = params
        # visibility
        gh = (params['gamma'] / float(params['dim'][0])) * 2.0 \
            * sin(self.fov['ah'] / 2.0)
        gv = (params['gamma'] / float(params['dim'][1])) * 2.0 \
            * sin(self.fov['av'] / 2.0)
        # TODO: handle gh = 0, gv = 0
        self.Cv = lambda p: p.z > 0 and min(min(max((min(p.x / p.z + \
            sin(self.fov['ahl']), sin(self.fov['ahr']) - p.x / p.z) / gh), 0.0),
            1.0), min(max((min(p.y / p.z + sin(self.fov['avt']),
            sin(self.fov['avb']) - p.y / p.z) / gv), 0.0), 1.0)) or 0.0
        # resolution
        mr = min([float(params['dim'][0]) / (2 * sin(self.fov['ah'] / 2.0)),
                  float(params['dim'][1]) / (2 * sin(self.fov['av'] / 2.0))])
        zr1 = (1.0 / params['r1']) * mr
        zr2 = (1.0 / params['r2']) * mr
        # TODO: handle zr2 = zr1
        self.Cr = lambda p: min(max((zr2 - p.z) / (zr2 - zr1), 0.0), 1.0)
        # focus
        zl, zr = self.zc(min(params['s']))
        zn, zf = self.zc(params['cmax'])
        # TODO: handle cmax = cmin
        self.Cf = lambda p: min(max(min((p.z - zn) / (zl - zn),
            (zf - p.z) / (zf - zr)), 0.0), 1.0)
        # direction
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
            return min(max((float(p.rho) - (terma * termb) - pi + \
                self.params['zeta2']) / (self.params['zeta2'] - \
                self.params['zeta1']), 0.0), 1.0)
        self.Cd = Cd
        # active
        self.active = active
        # fov sprite
        zm = min(zf, zr2)
        hull = []
        for z in [zn, zm]:
            hull += [(self.fov['sahl'] * z, self.fov['savt'] * z, z),
                     (self.fov['sahl'] * z, self.fov['savb'] * z, z),
                     (self.fov['sahr'] * z, self.fov['savb'] * z, z),
                     (self.fov['sahr'] * z, self.fov['savt'] * z, z)]
        self.fovvis = \
            [{'type': 'curve', 'color': (1, 0, 0),
              'pos': hull[i:i + 4] + hull[i:i + 1]} for i in range(0, 16, 4)] +\
            [{'type': 'curve', 'color': (1, 0, 0),
              'pos': [hull[i], hull[i + 4]]} for i in range(4)]

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
            self._fov['ahl'] = 2.0 * atan((self.params['o'][0] * \
                self.params['s'][0]) / (2.0 * self.params['f']))
            self._fov['ahr'] = 2.0 * atan(((self.params['dim'][0] - \
                self.params['o'][0]) * self.params['s'][0]) / \
                (2.0 * self.params['f']))
            self._fov['ah'] = self._fov['ahl'] + self._fov['ahr']
            self._fov['sahl'] = -sin(self._fov['ahl'])
            self._fov['sahr'] = sin(self._fov['ahr'])
            self._fov['sah'] = self._fov['sahr'] - self._fov['sahl']
            # vertical
            self._fov['avt'] = 2.0 * atan((self.params['o'][1] * \
                self.params['s'][1]) / (2.0 * self.params['f']))
            self._fov['avb'] = 2.0 * atan(((self.params['dim'][1] - \
                self.params['o'][1]) * self.params['s'][1]) / \
                (2.0 * self.params['f']))
            self._fov['av'] = self._fov['avt'] + self._fov['avb']
            self._fov['savt'] = -sin(self._fov['avt'])
            self._fov['savb'] = sin(self._fov['avb'])
            self._fov['sav'] = self._fov['savb'] - self._fov['savt']
            return self._fov

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
            r.append((self.params['A'] * self.params['f'] * self.params['zS']) \
                / (self.params['A'] * self.params['f'] + s * c \
                * (self.params['zS'] - self.params['f'])))
        if r[1] < 0:
            r[1] = float('inf')
        return tuple(r)

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

    def coverage(self, relevance):
        """\
        Return the coverage model of this camera with respect to the points in a
        given relevance model.

        @param relevance: The relevance model.
        @type relevance: L{RelevanceModel}
        @return: The coverage model.
        @rtype: L{PointCache}
        """
        coverage = PointCache()
        for point in relevance.mapped:
            coverage[point] = self.strength(point)
        return coverage

    def performance(self, relevance, coverage=None):
        """\
        Return the coverage performance of this camera with respect to a given
        relevance model. If a previously computed coverage cache is provided, it
        should be for the same point set as used by the relevance model.

        @param relevance: The relevance model.
        @type relevance: L{RelevanceModel}
        @param coverage: Previously computed coverage cache for these points.
        @type coverage: L{PointCache}
        @return: Performance metric in [0, 1].
        @rtype: C{float}
        """
        coverage = coverage or self.coverage(relevance)
        return sum((coverage & relevance.mapped).values()) \
            / sum(relevance.mapped.values())

    def update_visualization(self):
        """\
        Update the visualization for camera active state and pose.
        """
        self.opacity = self.active and 1.0 or 0.2
        Visualizable.update_visualization(self)
        

class Scene(dict):
    """\
    Discrete spatial-directional range with occlusion class.
    """
    def __init__(self):
        """\
        Constructor.
        """
        dict.__init__(self)
        self._visible = False

    @property
    def visible(self):
        return self._visible

    @visible.setter
    def visible(self, value):
        self._visible = value
        for posable in self:
            self[posable].visible = value

    def occluded(self, p, cam=Point()):
        """\
        Return whether the specified point is occluded from the camera
        viewpoint (by opaque scene planes).
        """
        for posable in self:
            pr = self[posable].intersection(p, cam)
            if pr is not None and pr.euclidean(p) > 1e-4:
                return True
        return False

    def visualize(self):
        """\
        Visualize the scene objects.
        """
        for posable in self:
            try:
                self[posable].visualize()
            except AttributeError:
                pass
        self._visible = True

    def update_visualization(self):
        """\
        Update the visualizations of the scene objects.
        """
        for posable in self:
            try:
                self[posable].update_visualization()
            except AttributeError:
                pass


class MultiCamera(dict):
    """\
    Multi-camera n-ocular coverage strength model.
    """
    def __init__(self, name='Untitled', scene=Scene()):
        """\
        Constructor.
   
        @param name: The name of this model.
        @type name: C{str}
        @param scene: The discrete scene model.
        @type scene: L{Scene}
        """
        dict.__init__(self)
        self.name = name
        self.scene = scene

    def __del__(self):
        self.scene.visible = False
        for camera in self:
            self[camera].visible = False

    @property
    def active_cameras(self):
        """\
        Return a list of active cameras.

        @rtype: C{list}
        """
        return [key for key in self if self[key].active]
    
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
                    minstrength = strength * (not self.scene.occluded(point,
                        self[camera].pose.T))
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

    def performance(self, relevance, ocular=1, coverage=None):
        """\
        Return the coverage performance of this multi-camera network with
        respect to a given relevance model. If a previously computed coverage
        cache is provided, it should be for the same point set as used by the
        relevance model.

        @param relevance: The relevance model.
        @type relevance: L{RelevanceModel}
        @param ocular: Mutual camera coverage degree.
        @type ocular: C{int}
        @param coverage: Previously computed coverage cache for these points.
        @type coverage: L{PointCache}
        @return: Performance metric in [0, 1].
        @rtype: C{float}
        """
        coverage = coverage or self.coverage(relevance, ocular)
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
        @rtype: L{Hypergraph}
        """
        H = Hypergraph(vertices=self.keys())
        if K is None:
            K = range(2, len(self) + 1)
        else:
            K.sort()
        active_cameras = self.active_cameras
        cache = {}
        for camera in active_cameras:
            print('Computing single-camera coverage for %s...' % camera)
            subset = frozenset([camera])
            cache[subset] = self.coverage(relevance, subset=subset)
            weight = sum(cache[subset].values())
            if weight:
                H.add_edge(Edge(subset), weight=weight)
                print('    Edge added with weight %f.' % weight)
        pk = 1
        for k in K:
            if k < 2:
                continue
            for subset in combinations(active_cameras, k):
                print('Computing multi-camera coverage for %s...' % (subset,))
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
                    H.add_edge(Edge(subset), weight=weight)
                    print('    Edge added with weight %f.' % weight)
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
        self.scene.visualize()
        for camera in self:
            self[camera].visualize()

    def update_visualization(self):
        """\
        Update the visualization.
        """
        self.scene.update_visualization()
        for camera in self:
            self[camera].update_visualization()
