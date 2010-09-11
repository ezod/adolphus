from adolphus import Point, Pose, Plane, \
                     visual, VisualizationObject, VisualizationError


class CheckerCalibrationBoard(Plane):
    """\
    Scene object for the checkerboard pattern calibration target.
    """
    center = None

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
        Plot the calibration target in a 3D visual model.

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
