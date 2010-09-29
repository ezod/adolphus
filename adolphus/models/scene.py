from ..geometry import Point, Pose, Plane
from ..visualization import  visual, VisualizationObject, VisualizationError


class VisionPlatform(object):
    """\
    Scene object for the vision platform.
    """
    def __init__(self, pose=None):
        """\
        Constructor.

        @param pose: The pose of the plane normal (from z-hat).
        @type pose: L{Pose}
        """
        self.pose = pose
        self.center = pose.T

    def intersection(self, pa, pb):
        """\
        Return the 3D point of intersection (if any) of the line segment
        between the two specified points and this object.

        @param pa: The first vertex of the line segment.
        @type pa: L{Point}
        @param pb: The second vertex of the line segment.
        @type pb: L{Point}
        @return: The point of intersection with the object.
        @rtype: L{Point}
        """
        return None
        # TODO: intersection

    def visualize(self, scale=None, color=None, opacity=1.0):
        """\
        Create a 3D visual model of the object.

        @param scale: Not used.
        @type scale: C{object}
        @param color: Not used.
        @type color: C{object}
        @param opacity: The opacity with which to plot the object.
        @type opacity: C{float}
        """
        if not visual:
            raise VisualizationError("visual module not loaded")
        self.vis = VisualizationObject(self)
        for x in [20, 900]:
            for y in [20, 1280, 2550]:
                self.vis.add('bar%d%d' % (x, y), visual.box(frame=self.vis,
                    pos=(x, y, 885), length=40, height=40, width=1770,
                    color=(1, 0.5, 0), opacity=opacity,
                    material=visual.materials.plastic))
                if y == 1280:
                    continue
                self.vis.add('wheel%d%d' % (x, y), visual.cylinder(frame=\
                    self.vis, pos=(x - 12, y, -60), axis=(24, 0, 0), radius=50,
                    color=(0.1, 0.1, 0.1), opacity=opacity,
                    material=visual.materials.rough))
        for x in [20, 900]:
            for z in [20, 510, 1750]:
                self.vis.add('bar%d%d' % (x, z), visual.box(frame=self.vis,
                    pos=(x, 1285, z), length=40, width=40, height=2570,
                    color=(1, 0.5, 0), opacity=opacity,
                    material=visual.materials.plastic))
        for y in [20, 1280, 2550]:
            for z in [20, 510, 1750]:
                self.vis.add('bar%d%d' % (y, z), visual.box(frame=self.vis,
                    pos=(460, y, z), height=40, width=40, length=920,
                    color=(1, 0.5, 0), opacity=opacity,
                    material=visual.materials.plastic))
        self.vis.add('top', visual.box(frame=self.vis, pos=(460, 1285, 1775),
            width=10, height=2530, length=860, color=(0.1, 0.1, 0.1),
            opacity=opacity, material=visual.materials.plastic))
        self.vis.add('steel', visual.box(frame=self.vis, pos=(460, 1590, 535),
            width=10, height=1900, length=860, color=(0.6, 0.6, 0.6),
            opacity=opacity, material=visual.materials.rough))
        self.vis.add('wood', visual.box(frame=self.vis, pos=(460, 650, 49.5),
            width=19, height=1240, length=860, color=(0.91, 0.85, 0.64),
            opacity=opacity, material=visual.materials.wood))
        axis, angle = self.pose.R.to_axis_angle()
        self.vis.transform(self.pose.T.tuple, axis, angle)


class CheckerCalibrationBoard(object):
    """\
    Scene object for the checkerboard pattern calibration target.
    """
    def __init__(self, pose=None):
        """\
        Constructor.

        @param pose: The pose of the plane normal (from z-hat).
        @type pose: L{Pose}
        """
        self.pose = pose
        self.plane = Plane(pose=pose, x=(-138, 174), y=(-111, 115))
        self.center = pose.T
    
    def intersection(self, pa, pb):
        """\
        Return the 3D point of intersection (if any) of the line segment
        between the two specified points and this object.

        @param pa: The first vertex of the line segment.
        @type pa: L{Point}
        @param pb: The second vertex of the line segment.
        @type pb: L{Point}
        @return: The point of intersection with the object.
        @rtype: L{Point}
        """
        pr = self.plane.intersection(pa, pb)
        prlocal = (-self.pose).map(pr)
        if prlocal.x > 276 and prlocal.y > -55 and prlocal.y < 57:
            return None
        return pr

    def visualize(self, scale=None, color=None, opacity=1.0):
        """\
        Create a 3D visual model of the object.

        @param scale: Not used.
        @type scale: C{object}
        @param color: Not used.
        @type color: C{object}
        @param opacity: The opacity with which to plot the object.
        @type opacity: C{float}
        """
        if not visual:
            raise VisualizationError("visual module not loaded")
        self.vis = VisualizationObject(self)
        self.vis.add('bm', visual.box(frame=self.vis, pos=(0, 0, 2),
            length=2, width=226, height=276, color=(0.74, 0.67, 0.53),
            opacity=opacity, material=visual.materials.wood))
        self.vis.add('bl', visual.box(frame=self.vis, pos=(0, 156, -83),
            length=2, width=56, height=36, color=(0.74, 0.67, 0.53),
            opacity=opacity, material=visual.materials.wood))
        self.vis.add('br', visual.box(frame=self.vis, pos=(0, 156, 86),
            length=2, width=58, height=36, color=(0.74, 0.67, 0.53),
            opacity=opacity, material=visual.materials.wood))
        for x in range(-7, 7):
            for y in range(-5, 5):
                self.vis.add('c%d%d' % (x, y), visual.box(frame=self.vis,
                    pos=(1, 9.5 + 19 * x, 9.5 + 19 * y), length=1.2, width=19,
                    height=19, color=((x + y) % 2, (x + y) % 2, (x + y) % 2),
                    opacity=opacity))
        axis, angle = self.pose.R.to_axis_angle()
        self.vis.transform(self.pose.T.tuple, axis, angle)
