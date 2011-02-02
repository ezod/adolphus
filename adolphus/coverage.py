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

from geometry import Point, DirectionalPoint, Pose
from posable import Posable
from visualization import Visualizable


class PointCache(dict):
    """\
    Point cache class.
    """
    def __setitem__(self, key, item):
        super(PointCache, self).__setitem__(repr(key), item)

    def __getitem__(self, key):
        return super(PointCache, self).__getitem__(repr(key))

    def __delitem__(self, key):
        super(PointCache, self).__delitem__(repr(key))

    def keys(self):
        return [eval(key) for key in super(PointCache, self).keys()]

    def __or__(self, other):
        result = self.__class__()
        for point in self.keys():
            result[point] = self[point]
        for point in other.keys():
            if not point in result.keys() or result[point] < other[point]:
                result[point] = other[point]
        return result

    def __ior__(self, other):
        self = self | other
        return self

    def __and__(self, other):
        result = self.__class__()
        for point in self.keys():
            if point in other.keys():
                result[point] = min(self[point], other[point])
        return result

    def __iand__(self, other):
        self = self & other
        return self

    def __del__(self):
        try:
            for point in self.visual:
                point.visible = False
        except AttributeError:
            pass

    def visualize(self):
        """\
        Visualize the point cache, with opacity representing coverage strength.
        """
        try:
            self.update_visualization()
        except AttributeError:
            self.visual = set([point for point in self.keys() if self[point]])
            for point in self.visual:
                point.opacity = self[point]
                point.visualize()

    def update_visualization(self):
        """\
        Update the point cache visualization.
        """
        newvisual = set([point for point in self.keys() if self[point]])
        # remove old points
        for point in self.visual.difference(newvisual):
            point.visible = False
            self.visual.remove(point)
        # add new points
        for point in newvisual.difference(self.visual):
            self.visual.add(point)
            point.visualize()
        # update opacities
        for point in self.visual:
            point.opacity = self[point]
            point.update_visualization()


class RelevanceModel(PointCache, Posable):
    """\
    Relevance model class.
    """
    def __init__(self, pose=Pose(), mount=None):
        Posable.__init__(self, pose, mount)

    def __setitem__(self, key, item):
        PointCache.__setitem__(self, (-self.pose).map(key), item)

    def __getitem__(self, key):
        PointCache.__getitem__(self, (-self.pose).map(key))

    def __delitem__(self, key):
        PointCache.__delitem__(self, (-self.pose).map(key))

    def keys(self):
        return [self.pose.map(point) for point in PointCache.keys(self)]


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
        Posable.__init__(self, pose, mount)
        Visualizable.__init__(self, sprites)
        if isinstance(params['s'], Number):
            params['s'] = (params['s'], params['s'])
        self.params = params
        # visibility
        gh = (params['gamma'] / float(params['dim'][0])) * 2.0 \
            * sin(self.fov['ah'] / 2.0)
        gv = (params['gamma'] / float(params['dim'][1])) * 2.0 \
            * sin(self.fov['av'] / 2.0)
        self.Cv = lambda p: p.z > 0 and min(min(max((min(p.x / p.z + \
            sin(self.fov['ahl']), sin(self.fov['ahr']) - p.x / p.z) / gh), 0.0),
            1.0), min(max((min(p.y / p.z + sin(self.fov['avt']),
            sin(self.fov['avb']) - p.y / p.z) / gv), 0.0), 1.0)) or 0.0
        # resolution
        mr = min([float(params['dim'][0]) / (2 * sin(self.fov['ah'] / 2.0)),
                  float(params['dim'][1]) / (2 * sin(self.fov['av'] / 2.0))])
        zr1 = (1.0 / params['r1']) * mr
        zr2 = (1.0 / params['r2']) * mr
        self.Cr = lambda p: min(max((zr2 - p.z) / (zr2 - zr1), 0.0), 1.0)
        # focus
        zl, zr = self.zc(min(params['s']))
        zn, zf = self.zc(params['cmax'])
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
            return min(max((float(p.rho) - (terma * termb) - pi + self.params['zeta2']) / (self.params['zeta2'] - self.params['zeta1']), 0.0), 1.0)
        self.Cd = Cd
        # active
        self.active = active

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

        @param iterable: The initial set of opaque scene planes.
        @type iterable: C{iterable}
        """
        dict.__init__(self)

    def occluded(self, p, cam=Point()):
        """\
        Return whether the specified point is occluded from the camera
        viewpoint (by opaque scene planes).
        """
        for posable in self.keys():
            pr = self[posable].intersection(p, cam)
            if pr is not None and pr.euclidean(p) > 1e-4:
                return True
        return False

    def visualize(self):
        """\
        Visualize the scene objects.
        """
        for posable in self.keys():
            try:
                self[posable].visualize()
            except AttributeError:
                pass

    def update_visualization(self):
        """\
        Update the visualizations of the scene objects.
        """
        for posable in self.keys():
            try:
                self[posable].update_visualization()
            except AttributeError:
                pass


class MultiCamera(dict):
    """\
    Multi-camera n-ocular coverage strength model.
    """
    def __init__(self, name='Untitled', ocular=1, scene=Scene()):
        """\
        Constructor.
   
        @param name: The name of this model.
        @type name: C{str}
        @param ocular: Mutual camera coverage degree.
        @type ocular: C{int}
        @param scene: The discrete scene model.
        @type scene: L{Scene}
        """
        dict.__init__(self)
        self.name = name
        self.ocular = ocular
        self.scene = scene

    @property
    def active_cameras(self):
        """\
        Return a list of active cameras.

        @rtype: C{list}
        """
        return [key for key in self.keys() if self[key].active]
    
    def strength(self, point):
        """\
        Return the individual coverage strength of a point in the coverage
        strength model.

        @param point: The (directional) point to test.
        @type point: L{Point}
        @return: The coverage strength of the point.
        @rtype: C{float}
        """
        active_cameras = self.active_cameras
        if len(active_cameras) < self.ocular:
            raise ValueError('network has too few active cameras')
        maxstrength = 0.0
        for combination in combinations(active_cameras, self.ocular):
            minstrength = float('inf')
            for camera in combination:
                strength = self[camera].strength(point)
                if strength and strength < minstrength:
                    minstrength = strength * (not self.scene.occluded(point, self[camera].pose.T))
            if minstrength > 1.0:
                minstrength = 0.0
            maxstrength = max(maxstrength, minstrength)
        return maxstrength

    def coverage(self, relevance):
        """\
        Return the coverage model of this multi-camera network with respect to
        the points in a given relevance model.

        @param relevance: The relevance model.
        @type relevance: L{RelevanceModel}
        @return: The coverage model.
        @rtype: L{PointCache}
        """
        coverage = PointCache()
        for point in relevance.keys():
            coverage[point] = self.strength(point)
        return coverage

    def performance(self, relevance):
        """\
        Return the coverage performance of this multi-camera network with
        respect to a given relevance model.

        @param relevance: The relevance model.
        @type relevance: L{RelevanceModel}
        @return: Performance metric in [0, 1].
        @rtype: C{float}
        """
        coverage = self.coverage(relevance)
        return sum((coverage & relevance).values()) / sum(relevance.values())

    def visualize(self):
        """\
        Visualize all cameras and the directional points of the coverage model
        (with opacity reflecting degree of coverage).

        @return: True if the visualization was initialized for the first time.
        @rtype: C{bool}
        """
        self.scene.visualize()
        for camera in self.keys():
            self[camera].visualize()

    def update_visualization(self):
        """\
        Update the visualization.
        """
        self.scene.update_visualization()
        for camera in self.keys():
            self[camera].update_visualization()
