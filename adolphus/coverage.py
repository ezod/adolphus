"""\
Coverage model module. Contains the fuzzy coverage model classes for single
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
from fuzz import IndexedSet, TrapezoidalFuzzyNumber, FuzzySet, FuzzyElement, \
                 FuzzyGraph

from geometry import Point, DirectionalPoint, Pose, Rotation, Posable, Plane
from visualization import visual, VisualizationError, VisualizationObject


class PointFuzzySet(FuzzySet, Posable):
    """\
    Fuzzy set of points.
    """
    def __init__(self, iterable=set(), pose=Pose(), mount=None, config=None):
        """\
        Constructor.

        @param iterable: The iterable to construct from (optional).
        @type iterable: C{object}
        """
        FuzzySet.__init__(self, iterable)
        Posable.__init__(self, pose, mount)

    def __del__(self):
        """\
        Deleter.
        """
        for point in self.keys():
            try:
                for member in point.vis.members.keys():
                    point.vis.members[member].visible = False
                    del point.vis.members[member]
                point.vis.visible = False
                del point.vis
            except AttributeError:
                pass

    def update_visualization(self):
        """\
        Update the visualization.
        """
        Posable.update_visualization(self)
        for point in self.keys():
            if self.mu(point):
                try:
                    for member in point.vis.keys():
                        point.vis.members[member].opacity = self.mu(point)
                except AttributeError:
                    point.visualize(scale=self.vis.properties['scale'], color=\
                        self.vis.properties['color'], opacity=self.mu(point))
                    self.vis.add('%s' % point, point.vis)
            else:
                try:
                    for member in point.vis.members.keys():
                        point.vis.members[member].visible = False
                        del point.vis.members[member]
                    point.vis.visible = False
                    del point.vis
                except AttributeError:
                    pass
                continue


class Camera(Posable):
    """\
    Single-camera model, using continous fuzzy sets.
    """
    def __init__(self, params, pose=Pose(), mount=None, config=None,
                 active=True, models=None):
        """\
        Constructor.

        @param params: Dictionary of application parameters.
        @type params: C{dict}
        @param pose: Pose of the camera in space (optional).
        @type pose: L{Pose}
        @param mount: Mount object for the camera (optional).
        @type mount: C{object}
        @param config: Configuration of the camera (unused).
        @type config: C{object}
        @param active: Initial active state of camera (optional).
        @type active: C{bool}
        @param models: Visualization models to use (optional).
        @type models: C{list} of L{VisualizationObject}
        """
        Posable.__init__(self, pose, mount)
        if isinstance(params['s'], Number):
            params['s'] = (params['s'], params['s'])
        self.models = models
        self.vis = None
        self.params = params
        # fuzzy sets for visibility
        self.Cv = []
        for i in range(2):
            g = (params['gamma'] / float(params['dim'][i])) * 2.0 \
                * sin(self.fov['a'][i] / 2.0)
            self.Cv.append(TrapezoidalFuzzyNumber((self.fov['sl'][i] + g,
                self.fov['sr'][i] - g), (self.fov['sl'][i], self.fov['sr'][i])))
        # fuzzy set for resolution
        mr = min([float(params['dim'][i]) / (2 * sin(self.fov['a'][i] / 2.0)) \
                  for i in range(2)])
        zr1 = (1.0 / params['r1']) * mr
        zr2 = (1.0 / params['r2']) * mr
        self.Cr = TrapezoidalFuzzyNumber((0, zr1), (0, zr2))
        # fuzzy set for focus
        zl, zr = self.zc(min(params['s']))
        zn, zf = self.zc(params['cmax'])
        self.Cf = TrapezoidalFuzzyNumber((zl, zr), (zn, zf))
        # fuzzy set for direction
        self.Cd = TrapezoidalFuzzyNumber(((pi / 2.0) - params['zeta1'],
            pi / 2.0), ((pi / 2.0) - params['zeta2'], pi / 2.0))
        # pose
        self.pose = pose
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
            self._fov = {'a': [], 'al': [], 'ar': [],
                         's': [], 'sl': [], 'sr': []}
            for i in range(2):
                self._fov['al'].append(2.0 * atan((self.params['o'][i] \
                    * self.params['s'][i]) / (2.0 * self.params['f'])))
                self._fov['ar'].append(2.0 * atan(((self.params['dim'][i] \
                    - self.params['o'][i]) * self.params['s'][i]) \
                    / (2.0 * self.params['f'])))
                self._fov['a'].append(self._fov['al'][i] + self._fov['ar'][i])
                self._fov['sl'].append(-sin(self._fov['al'][i]))
                self._fov['sr'].append(sin(self._fov['ar'][i]))
                self._fov['s'].append(self._fov['sr'][i] - self._fov['sl'][i])
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
                * self.params['zS'] - self.params['f']))
        if r[1] < 0:
            r[1] = float('inf')
        return tuple(r)

    def mu(self, point):
        """\
        Return the membership degree (coverage) for a directional point.
    
        @param point: The (directional) point to test.
        @type point: L{Point}
        @return: The membership degree (coverage) of the point.
        @rtype: C{float}
        """
        if not isinstance(point, Point):
            raise TypeError("invalid point")

        campoint = (-self.pose).map(point)

        # visibility
        try:
            mu_v = min([self.Cv[i].mu(campoint[i] / campoint.z) for i in range(2)])
        except ZeroDivisionError:
            mu_v = 0.0

        # resolution
        mu_r = self.Cr.mu(campoint.z)

        # focus
        mu_f = self.Cf.mu(campoint.z)

        # direction
        if isinstance(campoint, DirectionalPoint):
            r = sqrt(campoint.x ** 2 + campoint.y ** 2)
            try:
                terma = (campoint.y / r) * sin(campoint.eta) + \
                        (campoint.x / r) * cos(campoint.eta)
            except ZeroDivisionError:
                terma = 1.0
            try:
                termb = atan(r / campoint.z)
            except ZeroDivisionError:
                termb = pi / 2.0
            mu_d = self.Cd.mu(float(campoint.rho) \
                - ((pi / 2.0) + terma * termb))
        else:
            mu_d = 1.0

        # algebraic product intersection
        return mu_v * mu_r * mu_f * mu_d

    def visualize(self, scale=1.0, color=(1, 1, 1), opacity=1.0):
        """\
        Visualize the camera.

        @param scale: The scale of the visualization (optional).
        @type scale: C{float}
        @param color: The color of the visualization (optional).
        @type color: C{tuple}
        @param opacity: The opacity of the visualization (optional).
        @type opacity: C{float}
        @return: True if visualization was initialized for the first time.
        @rtype: C{bool}
        """
        if not self.models:
            raise VisualizationError("no models to visualize")
        if Posable.visualize(self, scale=scale, color=color, opacity=opacity):
            for model in self.models:
                self.vis.add(model.__name__, model(self))
            self.update_visualization()
            return True
        else:
            return False

    def update_visualization(self):
        """\
        Update the visualization for camera active state and pose.
        """
        Posable.update_visualization(self)
        self.vis.fade(not self.active)
        try:
            self.vis.members['fov'].opacity = 0.1
        except KeyError:
            pass

    def visualize_fov_toggle(self):
        """\
        Visualize the field of view of the camera.
        """
        if self.vis:
            if self.vis.members.has_key('fov'):
                self.vis.members['fov'].visible = \
                    not self.vis.members['fov'].visible
            else:
                # TODO: verify if this is a good value for the depth
                scale = self.Cf.kernel[1]
                a = [self.fov['sr'][i] - (self.fov['s'][i] / 2.0) \
                     for i in range(2)]
                self.vis.add('fov', visual.pyramid(axis=(-a[0], a[1], -1),
                    pos=(scale * a[0], -scale * a[1], scale),
                    size=(scale, self.Cv[1].support.size * scale,
                    self.Cv[0].support.size * scale), opacity=0.1,
                    color=(0.2, 0.5, 0.6)))


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
        for widget in self.keys():
            pr = self[widget].intersection(p, cam)
            if pr is not None and pr.euclidean(p) > 0.0001:
                return True
        return False

    def visualize(self, scale=1.0, color=(1, 1, 1), opacity=1.0):
        """\
        Visualize the opaque scene objects.

        @param color: The color of opaque scene objects.
        @type color: C{tuple}
        """
        if not visual:
            raise VisualizationError("visual module not loaded")
        for widget in self.keys():
            self[widget].visualize(scale=scale, color=color, opacity=opacity)


class MultiCamera(dict):
    """\
    Multi-camera n-ocular fuzzy coverage model.
    """
    def __init__(self, name="Untitled", ocular=1, scene=Scene(), scale=1.0):
        """\
        Constructor.
   
        @param name: The name of this model.
        @type name: C{str}
        @param ocular: Mutual camera coverage degree.
        @type ocular: C{int}
        @param scene: The discrete scene model.
        @type scene: L{Scene}
        @param scale: The scale of the model (for visualization).
        @type scale: C{float}
        """
        dict.__init__(self)
        self.name = name
        if ocular < 1:
            raise ValueError("network must be at least 1-ocular")
        self.ocular = ocular
        self.scene = scene
        self.fvg = FuzzyGraph(directed=False)
        self.vis = False
        self.scale = scale

    def __setitem__(self, key, value):
        """\
        Assign a camera to a key, and update its in-scene model.

        @param key: The key to assign.
        @type key: C{object}
        @param value: The Camera object to assign to the key.
        @type value: L{Camera}
        """
        if not isinstance(value, Camera):
            raise ValueError("assigned value must be a Camera object")
        dict.__setitem__(self, key, value)

    @property
    def active_cameras(self):
        """\
        Return a list of active cameras.

        @rtype: C{list}
        """
        return [key for key in self.keys() if self[key].active]
    
    def update_fvg(self):
        """\
        Update the fuzzy vision graph.
        """
        self.fvg = FuzzyGraph(viter=self.keys(), directed=False)
        # TODO: update edges

    def mu(self, point):
        """\
        Return the individual membership degree of a point in the fuzzy
        coverage model.

        @param point: The (directional) point to test.
        @type point: L{Point}
        @return: The membership degree (coverage) of the point.
        @rtype: C{float}
        """
        active_cameras = self.active_cameras
        if len(active_cameras) < self.ocular:
            raise ValueError("network has too few cameras")
        return max([min([not self.scene.occluded(point,
            self[camera].pose.T) and self[camera].mu(point) or 0.0 \
            for camera in combination]) for combination \
            in combinations(active_cameras, self.ocular)])

    def coverage(self, relevance):
        """\
        Return the coverage model of this multi-camera network with respect to
        the points in a given relevance model.

        @param relevance: The relevance model.
        @type relevance: L{PointFuzzySet}
        @return: The coverage model.
        @rtype: L{PointFuzzySet}
        """
        coverage = PointFuzzySet(pose=relevance._pose, mount=relevance.mount)
        for point in relevance.keys():
            coverage.add(point, mu=self.mu(relevance.pose.map(point)))
        return coverage

    def performance(self, relevance):
        """\
        Return the coverage performance of this multi-camera network with
        respect to a given relevance model.

        @param relevance: The relevance model.
        @type relevance: L{PointFuzzySet}
        @return: Performance metric in [0, 1].
        @rtype: C{float}
        """
        return self.coverage(relevance).overlap(relevance)

    def visualize(self):
        """\
        Visualize all cameras and the directional points of the coverage model
        (with opacity reflecting degree of coverage).

        @return: True if the visualization was initialized for the first time.
        @rtype: C{bool}
        """
        if not visual:
            raise VisualizationError("visual module not loaded")
        try:
            self.update_visualization()
            return False
        except VisualizationError:
            # scene
            self.scene.visualize(scale=self.scale, color=(0.3, 0.3, 0.3))
            # cameras
            for camera in self.keys():
                self[camera].visualize()
                self[camera].vis.add('name', visual.label(color=(1, 1, 1),
                    height=6, text=camera))
                self[camera].vis.members['name'].visible = False
            self.vis = True
            return True

    def update_visualization(self):
        """\
        Update the visualization.
        """
        if not self.vis:
            raise VisualizationError("visualization not yet initialized")
        # cameras
        for camera in self:
            self[camera].update_visualization()

    def visualize_name_toggle(self):
        """\
        Toggle visibility of the camera name tags.
        """
        if not self.vis:
            raise VisualizationError("visualization not yet initialized")
        for camera in self:
            self[camera].vis.members['name'].visible = \
                not self[camera].vis.members['name'].visible

    def visualize_coverage(self, relevance):
        """\
        Visualize the discrete coverage model with respect to the points in a
        given relevance model, and return the coverage performance.

        @param relevance: The relevance model.
        @type relevance: L{PointFuzzySet}
        @return: Performance metric in [0, 1].
        @rtype: C{float}
        """
        try:
            del self._coverage_vis
        except AttributeError:
            pass
        self._coverage_vis = self.coverage(relevance)
        performance = self._coverage_vis.overlap(relevance)
        self._coverage_vis.visualize(scale=self.scale, color=(1, 0, 0))
        return performance
