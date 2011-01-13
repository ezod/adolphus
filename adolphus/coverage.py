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
from fuzz import FuzzySet, FuzzyGraph

from geometry import Point, DirectionalPoint, Pose, Posable
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
                    self.vis.add(str(point), point.vis)
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

    def mu(self, point):
        """\
        Return the membership degree (coverage) for a directional point.
    
        @param point: The (directional) point to test.
        @type point: L{Point}
        @return: The membership degree (coverage) of the point.
        @rtype: C{float}
        """
        cp = (-self.pose).map(point)
        return self.Cv(cp) * self.Cr(cp) * self.Cf(cp) * self.Cd(cp)

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
            raise VisualizationError('no models to visualize')
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
            try:
                self.vis_fov.visible = not self.vis_fov.visible
            except AttributeError:
                self.vis_fov = VisualizationObject(self, frame=self.vis)
                # FIXME: replace 1000 with something more dynamic?
                self.vis_fov.add('lb', visual.cylinder(pos=(0, 0, 0), radius=1,
                    axis=(1000 * self.fov['sahl'], 1000 * self.fov['savb'],
                    1000), color=(0, 1, 0), material=visual.materials.emissive))
                self.vis_fov.add('lt', visual.cylinder(pos=(0, 0, 0), radius=1,
                    axis=(1000 * self.fov['sahl'], 1000 * self.fov['savt'],
                    1000), color=(0, 1, 0), material=visual.materials.emissive))
                self.vis_fov.add('rb', visual.cylinder(pos=(0, 0, 0), radius=1,
                    axis=(1000 * self.fov['sahr'], 1000 * self.fov['savb'],
                    1000), color=(0, 1, 0), material=visual.materials.emissive))
                self.vis_fov.add('rt', visual.cylinder(pos=(0, 0, 0), radius=1,
                    axis=(1000 * self.fov['sahr'], 1000 * self.fov['savt'],
                    1000), color=(0, 1, 0), material=visual.materials.emissive))


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
            if pr is not None and pr.euclidean(p) > 1e-4:
                return True
        return False

    def visualize(self, scale=1.0, color=(1, 1, 1), opacity=1.0):
        """\
        Visualize the opaque scene objects.

        @param color: The color of opaque scene objects.
        @type color: C{tuple}
        """
        if not visual:
            raise VisualizationError('visual module not loaded')
        for widget in self.keys():
            self[widget].visualize(scale=scale, color=color, opacity=opacity)


class MultiCamera(dict):
    """\
    Multi-camera n-ocular fuzzy coverage model.
    """
    def __init__(self, name='Untitled', ocular=1, scene=Scene(), scale=1.0):
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
        #assert ocular > 0
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
        #assert isinstance(value, Camera)
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
            raise ValueError('network has too few active cameras')
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
            raise VisualizationError('visual module not loaded')
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
            raise VisualizationError('visualization not yet initialized')
        # cameras
        for camera in self:
            self[camera].update_visualization()

    def visualize_name_toggle(self):
        """\
        Toggle visibility of the camera name tags.
        """
        if not self.vis:
            raise VisualizationError('visualization not yet initialized')
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
