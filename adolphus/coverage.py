"""\
Coverage model module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from math import sqrt, sin, cos, atan, pi
from numbers import Number
from itertools import combinations
from fuzz import IndexedSet, TrapezoidalFuzzyNumber, FuzzySet, FuzzyElement

from geometry import Point, DirectionalPoint, Pose
from scene import Scene

try:
    import visual
    VIS = True
except ImportError:
    VIS = False


class Camera(object):
    """\
    Single-camera model, using continous fuzzy sets.
    """
    def __init__(self, A, f, s, o, dim, zS, gamma, r1, r2, cmax, zeta,
                 pose = Pose(), active = True):
        """\
        Constructor.

        @param A: Aperture size (intrinsic).
        @type A: C{float}
        @param f: Focal length (intrinsic).
        @type f: C{float}
        @param s: Effective pixel size (intrinsic).
        @type s: C{tuple} of C{float}
        @param o: Pixel coordinates of principal point (intrinsic).
        @type o: C{tuple} of C{float}
        @param dim: Sensor pixel dimensions (intrinsic).
        @type dim: C{int}
        @param zS: Subject distance (intrinsic).
        @type zS: C{float}
        @param gamma: Fuzzification value for visibility (application).
        @type gamma: C{float}
        @param r1: Fully acceptable resolution (application).
        @type r1: C{float}
        @param r2: Acceptable resolution (application).
        @type r2: C{float}
        @param cmax: Maximum acceptable circle of confusion (application).
        @type cmax: C{float}
        @param zeta: Fuzzification value for direction (application).
        @type zeta: C{float}
        @param pose: Pose of the camera in space (optional).
        @type pose: L{geometry.Pose}
        @param active: Initial active state of camera.
        @type pose: C{bool}
        """
        if isinstance(s, Number):
            s = (s, s)
        self.vis = None
        # fuzzy sets for visibility
        self.Cv, al, ar = [], [], []
        for i in range(2):
            al.append(2.0 * atan((o[i] * s[i]) / (2.0 * f)))
            ar.append(2.0 * atan(((dim[i] - o[i]) * s[i]) / (2.0 * f)))
            g = (gamma / float(dim[i])) * 2.0 * sin((al[i] + ar[i]) / 2.0)
            self.Cv.append(TrapezoidalFuzzyNumber((-sin(al[i]) + g,
                sin(ar[i]) - g), (-sin(al[i]), sin(ar[i]))))
        # fuzzy set for resolution
        mr = min([float(dim[i]) / (2.0 * sin((al[i] + ar[i]) / 2.0)) \
                  for i in range(2)])
        zr1 = (1.0 / r1) * mr
        zr2 = (1.0 / r2) * mr
        self.Cr = TrapezoidalFuzzyNumber((0, zr1), (0, zr2))
        # fuzzy set for focus
        zl = (A * f * zS) / (A * f + min(s) * (zS - f))
        zr = (A * f * zS) / (A * f - min(s) * (zS - f))
        zn = (A * f * zS) / (A * f + cmax * (zS - f))
        zf = (A * f * zS) / (A * f - cmax * (zS - f))
        self.Cf = TrapezoidalFuzzyNumber((zl, zr), (zn, zf))
        # fuzzifier for direction
        self.zeta = zeta
        # pose
        self.pose = pose
        # active
        self.active = active

    def mu(self, point):
        """\
        Return the membership degree (coverage) for a directional point.
    
        @param point: The (directional) point to test.
        @type point: L{geometry.Point}
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
                * termb)) / self.zeta, 0.0), 1.0)
        else:
            mu_d = 1.0

        # algebraic product intersection
        return mu_v * mu_r * mu_f * mu_d

    def visualize(self, scale = 1.0, fov = False):
        """\
        Plot the camera in a 3D visual model.

        @param scale: The scale of the camera.
        @type scale: C{float}
        @param fov: Toggle visualization of the field ov view.
        @type fov: C{bool}
        """
        if not VIS:
            raise ImportError("visual module not loaded")
        if self.vis:
            self.update_visualization()
            return
        self.vis = visual.frame()
        self.vis.camera = self
        self.vis.fov = None
        self.vis.members = {}
        # camera body
        self.vis.members['body'] = visual.box(frame = self.vis,
            size = (scale, scale, scale), color = (0.3, 0.3, 0.3),
            material = visual.materials.rough)
        # lens body
        self.vis.members['lens'] = visual.cylinder(frame = self.vis,
            pos = (0.8 * scale, 0, 0), axis = (-0.3 * scale, 0, 0),
            radius = 0.4 * scale, color = (0.3, 0.3, 0.3),
            material = visual.materials.rough)
        # lens glass
        self.vis.members['glass'] = visual.cylinder(frame = self.vis,
            pos = (0.82 * scale, 0, 0), axis = (-0.02 * scale, 0, 0),
            radius = 0.36 * scale, color = (0.3, 0.7, 0.8),
            material = visual.materials.plastic, opacity = 0.5)
        # lens ring
        self.vis.members['ring'] = visual.ring(frame = self.vis,
            pos = (0.82 * scale, 0, 0), axis = (-0.02 * scale, 0, 0),
            radius = 0.38 * scale, color = (0.3, 0.3, 0.3),
            material = visual.materials.rough)
        # indicator light
        self.vis.members['light'] = visual.sphere(frame = self.vis,
            pos = (0.5 * scale, 0, 0.45 * scale),
            radius = 0.05 * scale, material = visual.materials.emissive)
        # field of view
        if fov:
            self.visualize_fov_toggle(scale = scale)
        self.update_visualization()

    def update_visualization(self):
        """\
        Update the visualization for camera active state and pose.
        """
        if not self.vis:
            raise ValueError("camera not yet visualized")
        for member in ['body', 'lens', 'ring', 'light']:
            self.vis.members[member].opacity = self.active and 1.0 or 0.2
        self.vis.members['glass'].opacity = self.active and 0.5 or 0.1
        self.vis.members['light'].color = (not self.active, self.active, 0)
        self.vis.pos = self.pose.T.tuple
        # FIXME: no idea why up = (-1, 0, 0) or why (-axis) is necessary
        self.vis.axis = (0, 0, 1)
        self.vis.up = (-1, 0, 0)
        axis, angle = self.pose.R.to_axis_angle()
        self.vis.rotate(axis = (-axis).tuple, angle = angle)

    def visualize_fov_toggle(self, scale = 1.0):
        """\
        Visualize the field of view of the camera.

        @param scale: The scale of the field of view.
        @type scale: C{float}
        """
        if self.vis:
            if self.vis.fov:
                self.vis.fov.visible = not self.vis.fov.visible
            else:
                ax = self.Cv[0].support[1] - (self.Cv[0].support.size / 2.0)
                ay = self.Cv[1].support[1] - (self.Cv[1].support.size / 2.0)
                self.vis.fov = visual.pyramid(frame = self.vis, pos = (scale,
                    scale * ay, -scale * ax), axis = (-1, -ay, ax),
                    size = (scale, self.Cv[0].support.size * scale,
                    self.Cv[1].support.size * scale), opacity = 0.1,
                    color = (0.2, 0.5, 0.6))


class MultiCamera(dict):
    """\
    Multi-camera n-ocular fuzzy coverage model.
    """
    def __init__(self, ocular = 1, scene = Scene(), points = set()):
        """\
        Constructor.
    
        @param ocular: Mutual camera coverage degree.
        @type ocular: C{int}
        @param scene: The discrete scene model.
        @type scene: L{Scene}
        @param points: The initial set of points.
        @type points: C{set} of L{geometry.Point}
        """
        if ocular < 1:
            raise ValueError("network must be at least 1-ocular")
        self.ocular = ocular
        self.scene = scene
        self.points = points
        self.model = FuzzySet()
        self._updated_state = None

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
        # TODO: something about self._updated_state

    @property
    def model_updated(self):
        """\
        Return whether the fuzzy coverage model is up to date.

        @rtype: C{bool}
        """
        if not self._updated_state:
            return False
        return True

    def mu(self, point):
        """\
        Return the individual membership degree of a point in the fuzzy
        coverage model.

        @param point: The (directional) point to test.
        @type point: L{geometry.Point}
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

    def performance(self, desired):
        """\
        Return the coverage performance of this multi-camera network.

        @param desired: The D model of desired coverage.
        @type desired: L{fuzz.FuzzySet}
        @return: Performance metric in [0, 1].
        @rtype: C{float}
        """
        return self.model.overlap(desired)

    def visualize(self, scale = 1.0):
        """\
        Visualize all cameras and the directional points of the coverage model
        (with opacity reflecting degree of coverage).

        @param scale: The scale of the individual elements.
        @type scale: C{float}
        """
        if not VIS:
            raise ImportError("visual module not loaded")
        for camera in self:
            self[camera].visualize(scale = scale)
            self[camera].vis.members['name'] = visual.label(frame = \
                self[camera].vis, pos = (0, scale, 0), height = 6,
                color = (1, 1, 1), text = camera, visible = False)
        self.scene.visualize(color = (0.3, 0.3, 0.3))
        for point in self.model.keys():
            point.visualize(scale = scale, color = (1, 0, 0),
                            opacity = self.model[point].mu)

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
        for camera in self:
            self[camera].update_visualization()
        for point in self.model.keys():
            if point.vis:
                point.vis.members['point'].opacity = self.model[point].mu
                try:
                    point.vis.members['dir'].opacity = self.model[point].mu
                except KeyError:
                    pass
            else:
                point.visualize(scale = scale, color = (1, 0, 0),
                                opacity = self.model[point].mu)
