from adolphus import Point, Pose, Plane, VisualizationError, visual, transform


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
        self.vis = visual.frame()
        self.vis.members = {}
        self.vis.members['bm'] = visual.box(pos=(0, 0, 0), width=2,
            height=226, length=276, color=(0.74, 0.67, 0.53), opacity=opacity,
            material=visual.materials.wood)
        self.vis.members['bl'] = visual.box(pos=(156, -85, 0), width=2,
            height=56, length=36, color=(0.74, 0.67, 0.53), opacity=opacity,
            material=visual.materials.wood)
        self.vis.members['br'] = visual.box(pos=(156, 84, 0), width=2,
            height=58, length=36, color=(0.74, 0.67, 0.53), opacity=opacity,
            material=visual.materials.wood)
        for x in range(-7, 7):
            for y in range(-5, 5):
                self.vis.members['c%d%d' % (x, y)] = visual.box(pos=(9.5 + 19 \
                    * x, 9.5 + 19 * y, 1), width=1.2, height=19, length=19,
                    color=((x + y) % 2, (x + y) % 2, (x + y) % 2),
                    opacity=opacity)
        axis, angle = self.pose.R.to_axis_angle()
        transform(self.vis, (0, 0, 0), axis, angle)
