"""\
Visual interface module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import visual
import sys


class Display(visual.display):
    """\
    Visual display class.
    """
    def __init__(self, title = 'Adolphus', center = (0, 0, 0)):
        """\
        Contstructor.

        @param title: Title of the window.
        @type title: C{str}
        @param center: Location of the center point.
        @type center: C{tuple} of C{float}
        """
        visual.display.__init__(self, title = title, center = center, up = (0,
            0, 1), background = (1, 1, 1), foreground = (0.3, 0.3, 0.3))


class Experiment(object):
    """\
    Experiment class.
    """
    def __init__(self, model, display = Display()):
        """\
        Constructor.

        @param model: The multi-camera model to use.
        @type model: L{coverage.MultiCamera}
        @param display: The 3D display to use.
        @type display: L{Display}
        """
        self.model = model
        self.display = display
        self.display.select()

    def run(self):
        """\
        Run this experiment.
        """
        self.model.update_model()
        self.model.visualize(scale = 30)
        self.axes = visual_axes(scale = 30, color = (0, 0, 1))
        self.axes.visible = False
        self.cdot = visual.sphere(pos = self.display.center, radius = 5,
            color = (0, 0, 1), material = visual.materials.emissive,
            visible = False)
        cam_vis = [primitive for objects in [vis.objects for vis in \
            [self.model[camera].vis for camera in self.model]] \
            for primitive in objects]
        point_vis = [primitive for objects in [vis.objects for vis in \
            [point.vis for point in self.model.model.keys()]] \
            for primitive in objects]
        while True:
            visual.rate(50)
            if self.display.mouse.events:
                m = self.display.mouse.getevent()
                if m.click == "left" and m.pick in cam_vis:
                    if m.ctrl:
                        m.pick.frame.camera.visualize_fov_toggle(scale = 1500)
                    else:
                        m.pick.frame.camera.active = not m.pick.frame.camera.active
                        m.pick.frame.camera.update_visualization()
                if m.click == "left" and m.pick in point_vis:
                    print "mu%s = %f" % (m.pick.frame.point,
                        self.model.model[m.pick.frame.point].mu)
            if self.display.kb.keys:
                k = self.display.kb.getkey()
                if k == 'f2':
                    print "Updating discrete fuzzy coverage model... ",
                    sys.stdout.flush()
                    self.model.update_model()
                    self.model.update_visualization()
                    print "done."
                elif k == 'f6':
                    self.model.visualize_name_toggle()
                elif k == 'f7':
                    self.axes.visible = not self.axes.visible
                elif k == 'f8':
                    self.cdot.visible = not self.cdot.visible
                elif k == 'left':
                    self.display.center = (self.display.center[0] - 30,
                        self.display.center[1], self.display.center[2])
                elif k == 'right':
                    self.display.center = (self.display.center[0] + 30,
                        self.display.center[1], self.display.center[2])
                elif k == 'down':
                    self.display.center = (self.display.center[0],
                        self.display.center[1] - 30, self.display.center[2])
                elif k == 'up':
                    self.display.center = (self.display.center[0],
                        self.display.center[1] + 30, self.display.center[2])
                elif k == 'page down':
                    self.display.center = (self.display.center[0],
                        self.display.center[1], self.display.center[2] - 30)
                elif k == 'page up':
                    self.display.center = (self.display.center[0],
                        self.display.center[1], self.display.center[2] + 30)
                self.cdot.pos = self.display.center


def visual_axes(scale = 1.0, color = (1, 1, 1)):
    """\
    Display a set of 3D axes.

    @param scale: The scale of the axis set.
    @type scale: C{float}
    @param color: The color of the axes.
    @type color: C{tuple}
    """
    frame = visual.frame()
    # axes
    for a in [tuple([i == j and scale * 5 or 0 for i in range(3)]) \
              for j in range(3)]:
        visual.arrow(pos = (0, 0, 0), axis = a, shaftwidth = scale / 10.0,
                     color = color, frame = frame)
    # X
    visual.cylinder(pos = ((scale * 6.0), -(scale / 4.0), 0),
        axis = (-(scale / 2.0), (scale / 2.0), 0), radius = scale / 20.0,
        color = color, frame = frame)
    visual.cylinder(pos = (scale * 5.5, -(scale / 4.0), 0),
        axis = ((scale / 2.0), (scale / 2.0), 0), radius = scale / 20.0,
        color = color, frame = frame)
    # Y
    visual.cylinder(pos = (0, (scale * 5.5), 0), axis = (0, (scale / 4.0), 0),
        radius = scale / 20.0, color = color, frame = frame)
    visual.cylinder(pos = (0, (scale * 5.75), 0), axis = (-(scale * 0.17),
        (scale / 4.0), (scale * 0.17)), radius = scale / 20.0, color = color,
        frame = frame)
    visual.cylinder(pos = (0, (scale * 5.75), 0), axis = ((scale * 0.17),
        (scale / 4.0), -(scale * 0.17)), radius = scale / 20.0, color = color,
        frame = frame)
    # Z
    visual.cylinder(pos = (0, -(scale / 4.0), (scale * 6.0)), axis = (0.0,
        (scale / 2.0), -(scale / 2.0)), radius = scale / 20.0, color = color,
        frame = frame)
    visual.cylinder(pos = (0, -(scale / 4.0), (scale * 6.0)), axis = (0.0, 0.0,
        -(scale / 2.0)), radius = scale / 20.0, color = color, frame = frame)
    visual.cylinder(pos = (0, (scale / 4.0), (scale * 6.0)), axis = (0.0, 0.0,
        -(scale / 2.0)), radius = scale / 20.0, color = color, frame = frame)
    return frame
