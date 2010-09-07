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
from fuzz import IndexedSet, TrapezoidalFuzzyNumber, PolygonalFuzzyNumber, \
                 FuzzySet, FuzzyElement

try:
    import yaml
except ImportError:
    yaml = None

from geometry import Point, DirectionalPoint, Pose, Rotation, Plane, pointrange
from visualization import VisualizationError, visual, transform


class Scene(set):
    """\
    Discrete spatial-directional range with occlusion class.
    """
    def __init__(self, iterable=set()):
        """\
        Constructor.

        @param iterable: The initial set of opaque scene planes.
        @type iterable: C{iterable}
        """
        for plane in iterable:
            if not isinstance(plane, Plane):
                raise TypeError("only planes can be added")
        set.__init__(self, iterable)

    def add(self, plane):
        """\
        Add an opaque plane to the scene.

        @param plane: The item to add.
        @type plane: C{object}
        """
        if not isinstance(plane, Plane):
            raise TypeError("only planes can be added")
        set.add(self, plane)

    def occluded(self, p, cam=Point()):
        """\
        Return whether the specified point is occluded from the camera
        viewpoint (by opaque scene planes).
        """
        for plane in self:
            pr = plane.intersection(p, cam)
            if pr is not None and pr.euclidean(p) > 0.0001:
                return True
        return False

    def visualize(self, color=(1, 1, 1)):
        """\
        Visualize the opaque scene objects.

        @param color: The color of opaque scene objects.
        @type color: C{tuple}
        """
        if not visual:
            raise VisualizationError("visual module not loaded")
        for plane in self:
            plane.visualize(color=color)


class Camera(object):
    """\
    Single-camera model, using continous fuzzy sets.
    """
    def __init__(self, params, pose=Pose(), active=True):
        """\
        Constructor.

        @param params: Dictionary of application parameters.
        @type params: C{dict}
        @param pose: Pose of the camera in space (optional).
        @type pose: L{Pose}
        @param active: Initial active state of camera.
        @type active: C{bool}
        """
        if isinstance(params['s'], Number):
            params['s'] = (params['s'], params['s'])
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
            r.append((self.params['A'] * self.params['f'] * self.params['zS']) / (self.params['A'] * self.params['f'] + s * c * self.params['zS'] - self.params['f']))
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
            mu_d = min(max((float(campoint.rho) - ((pi / 2.0) + terma \
                * termb)) / self.params['zeta'], 0.0), 1.0)
        else:
            mu_d = 1.0

        # algebraic product intersection
        return mu_v * mu_r * mu_f * mu_d

    def visualize(self, scale=1.0, fov=False):
        """\
        Plot the camera in a 3D visual model.

        @param scale: The scale of the camera.
        @type scale: C{float}
        @param fov: Toggle visualization of the field ov view.
        @type fov: C{bool}
        """
        if not visual:
            raise VisualizationError("visual module not loaded")
        if self.vis:
            self.update_visualization()
            return
        self.vis = visual.frame()
        self.vis.camera = self
        self.vis.fov = None
        self.vis.members = {}
        # camera body
        self.vis.members['body'] = visual.box(frame=self.vis,
            size=(scale, scale, scale), color=(0.3, 0.3, 0.3),
            material=visual.materials.rough)
        # lens body
        self.vis.members['lens'] = visual.cylinder(frame=self.vis,
            pos=(0.8 * scale, 0, 0), axis=(-0.3 * scale, 0, 0),
            radius=(0.4 * scale), color=(0.3, 0.3, 0.3),
            material=visual.materials.rough)
        # lens glass
        self.vis.members['glass'] = visual.cylinder(frame=self.vis,
            pos=(0.82 * scale, 0, 0), axis=(-0.02 * scale, 0, 0),
            radius=(0.36 * scale), color=(0.3, 0.7, 0.8), opacity=0.5,
            material=visual.materials.plastic)
        # lens ring
        self.vis.members['ring'] = visual.ring(frame=self.vis,
            pos=(0.82 * scale, 0, 0), axis=(-0.02 * scale, 0, 0),
            radius=(0.38 * scale), color=(0.3, 0.3, 0.3),
            material=visual.materials.rough)
        # indicator light
        self.vis.members['light'] = visual.sphere(frame=self.vis,
            pos=(0.5 * scale, 0, 0.45 * scale), radius=(0.05 * scale),
            material=visual.materials.emissive)
        # field of view
        if fov:
            self.visualize_fov_toggle(scale=scale)
        self.update_visualization()

    def update_visualization(self):
        """\
        Update the visualization for camera active state and pose.
        """
        if not self.vis:
            raise VisualizationError("visualization not yet initialized")
        for member in ['body', 'lens', 'ring', 'light']:
            self.vis.members[member].opacity = self.active and 1.0 or 0.2
        self.vis.members['glass'].opacity = self.active and 0.5 or 0.1
        self.vis.members['light'].color = (not self.active, self.active, 0)
        axis, angle = self.pose.R.to_axis_angle()
        transform(self.vis, self.pose.T.tuple, axis, angle)

    def visualize_fov_toggle(self, scale=1.0):
        """\
        Visualize the field of view of the camera.

        @param scale: The scale of the field of view.
        @type scale: C{float}
        """
        if self.vis:
            if self.vis.fov:
                self.vis.fov.visible = not self.vis.fov.visible
            else:
                a = [self.fov['sr'][i] - (self.fov['s'][i] / 2.0) \
                     for i in range(2)]
                self.vis.fov = visual.pyramid(frame=self.vis, pos=(scale,
                    scale * a[1], -scale * a[0]), axis=(-1, -a[1], a[0]),
                    size=(scale, self.Cv[0].support.size * scale,
                    self.Cv[1].support.size * scale), opacity=0.1,
                    color=(0.2, 0.5, 0.6))


class MultiCamera(dict):
    """\
    Multi-camera n-ocular fuzzy coverage model.
    """
    def __init__(self, name="Untitled", ocular=1, scene=Scene(), points=set(),
                 scale=1.0):
        """\
        Constructor.
   
        @param name: The name of this model.
        @type name: C{str}
        @param ocular: Mutual camera coverage degree.
        @type ocular: C{int}
        @param scene: The discrete scene model.
        @type scene: L{Scene}
        @param points: The initial set of points.
        @type points: C{set} of L{Point}
        @param scale: The scale of the model (for visualization).
        @type scale: C{float}
        """
        self.name = name
        if ocular < 1:
            raise ValueError("network must be at least 1-ocular")
        self.ocular = ocular
        self.scene = scene
        self.points = points
        self.model = FuzzySet()
        if visual:
            self._vis = False
            self._vis_ptmu = False
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
        return [key for key in self if self[key].active]
    
    def update_model(self):
        """\
        Update the n-ocular multi-camera network discrete coverage model.
        """
        if len(self.active_cameras) < self.ocular:
            raise ValueError("network has too few cameras")
        self.model = FuzzySet([FuzzyElement(point, self.mu(point)) \
                               for point in self.points])

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

    def performance(self, relevance):
        """\
        Return the coverage performance of this multi-camera network with
        respect to a given relevance model.

        @param desired: The relevance model.
        @type desired: L{FuzzySet}
        @return: Performance metric in [0, 1].
        @rtype: C{float}
        """
        actual = FuzzySet()
        for point in relevance.keys():
            actual.add(point, mu=self.mu(point))
        return actual.overlap(relevance)

    def visualize(self):
        """\
        Visualize all cameras and the directional points of the coverage model
        (with opacity reflecting degree of coverage).
        """
        if not visual:
            raise VisualizationError("visual module not loaded")
        for camera in self:
            self[camera].visualize(scale=self.scale)
            self[camera].vis.members['name'] = visual.label(frame=\
                self[camera].vis, pos=(0, self.scale, 0), height=6,
                color=(1, 1, 1), text=camera, visible=False)
        self.scene.visualize(color=(0.3, 0.3, 0.3))
        for point in self.model.keys():
            point.visualize(scale=self.scale, color=(1, 0, 0),
                            opacity=self.model.mu(point))
            point.vis.members['mu'] = visual.label(frame=point.vis,
                pos=(0, 0, 0), height=6, color=(1, 1, 1), visible=False,
                text=("%1.4f" % self.model.mu(point)))
        self._vis = True

    def visualize_ptmu_toggle(self):
        """\
        Toggle visibility of mu values over points.
        """
        self._vis_ptmu = not self._vis_ptmu
        for point in self.model.keys():
            point.vis.members['mu'].visible = self.model.mu(point) \
                and self._vis_ptmu or False

    def visualize_name_toggle(self):
        """\
        Toggle visibility of the camera name tags.
        """
        for camera in self:
            self[camera].vis.members['name'].visible = \
                not self[camera].vis.members['name'].visible

    def update_visualization(self):
        """\
        Update the visualization.
        """
        if not self._vis:
            raise VisualizationError("visualization not yet initialized")
        for camera in self:
            self[camera].update_visualization()
        for point in self.model.keys():
            try:
                point.vis.members['point'].opacity = self.model[point].mu
                try:
                    point.vis.members['dir'].opacity = self.model[point].mu
                except KeyError:
                    pass
                point.vis.members['mu'].text = "%1.4f" % self.model.mu(point)
            except AttributeError:
                point.visualize(scale=self.scale, color=(1, 0, 0),
                                opacity=self.model.mu(point))
                point.vis.members['mu'] = visual.label(frame=point.vis,
                    pos=(0, 0, 0), height=6, color=(1, 1, 1), visible=False,
                    text=("%1.4f" % self.model.mu(point)))


def load_model_from_yaml(filename, active=True):
    """\
    Load parameters for a multi-camera fuzzy coverage model from a YAML file.

    @param filename: The YAML file to load from.
    @type filename: C{str}
    @param active: The default active state of cameras (default True).
    @type active: C{bool}
    @return: The multi-camera fuzzy coverage model.
    @rtype: L{MultiCamera}
    """
    if not yaml:
        raise ImportError("YAML module could not be loaded")
    params = yaml.load(open(filename))

    # scene
    scene = Scene()
    try:
        for plane in params['scene']:
            if plane.has_key('z'):
                scene.add(Plane(x=plane['x'], y=plane['y'], z=plane['z']))
            else:
                scene.add(Plane(x=plane['x'], y=plane['y'],
                    pose=Pose(T=Point(tuple(plane['T'])),
                              R=Rotation(plane['R']))))
    except KeyError:
        pass

    # points
    points = set()
    try:
        for point in params['points']:
            if point.has_key('step'):
                if not point.has_key('ddiv'):
                    point['ddiv'] = None
                for p in pointrange(point['x'], point['y'], point['z'],
                                    point['step'], ddiv=point['ddiv']):
                       points.add(p)
            else:
                if len(point['point']) == 3:
                    points.add(Point(tuple(point['point'])))
                elif len(point['point']) == 5:
                    points.add(DirectionalPoint(tuple(point['point'])))
    except KeyError:
        pass

    model = MultiCamera(name=params['name'], ocular=params['ocular'],
                        scene=scene, points=points, scale=params['scale'])

    # cameras
    for camera in params['cameras']:
        for ap in ['gamma', 'r1', 'r2', 'cmax', 'zeta']:
            camera[ap] = params[ap]
        model[camera['name']] = Camera(camera, active=active,
            pose=Pose(T=Point(tuple(camera['T'])), R=Rotation(camera['R'])))

    # relevance models
    try:
        relevance_models = [generate_relevance_model(params['relevance'][i]) \
                            for i in range(len(params['relevance']))]
    except KeyError:
        relevance_models = []

    return model, relevance_models


def generate_relevance_model(params):
    """\
    Generate a relevance model from a set of x-y-z polygonal fuzzy sets.

    @param params: The parameters (from YAML) for the polygonal fuzzy sets.
    @type params: C{dict}
    @return: The relevance model.
    @rtype: L{FuzzySet}
    """
    whole_model = FuzzySet()
    ranges, extent = {}, {}
    for range in params['ranges']:
        for axis in ['x', 'y', 'z']:
            ranges[axis] = PolygonalFuzzyNumber(range[axis])
            extent[axis] = (ranges[axis].support[0][0],
                            ranges[axis].support[-1][1])
        part_model = FuzzySet()
        for point in pointrange(extent['x'], extent['y'], extent['z'],
                                params['step']):
            part_model.add(point, mu=min([ranges[axis].mu(getattr(point,
                axis)) for axis in ['x', 'y', 'z']]))
        whole_model |= part_model
    return whole_model
