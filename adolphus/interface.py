"""\
Visual interface module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import sys

from visualization import VisualizationError, visual


class Display(visual.display):
    """\
    Visual display class.
    """
    def __init__(self, title='Adolphus', center=(0, 0, 0)):
        """\
        Contstructor.

        @param title: Title of the window.
        @type title: C{str}
        @param center: Location of the center point.
        @type center: C{tuple} of C{float}
        """
        visual.display.__init__(self, title=title, center=center,
            background=(1, 1, 1), foreground=(0.3, 0.3, 0.3))
        self.forward = (-1, -1, -1)
        self.up = (0, 0, 1)
        self._stored_view = None

    def camera_view(self, camera=None):
        """\
        Toggle camera view, displaying the scene from the point of view of a
        specific camera.

        @param camera: The camera object.
        @type: L{coverage.Camera}
        """
        if camera:
            if self._stored_view:
                raise VisualizationError("already in a camera view")
            #self.autoscale, self.userzoom, self.userspin = [False] * 3
            self.autoscale, self.userspin = [False] * 2
            self._stored_view = {'forward': tuple(self.forward),
                                 'center': tuple(self.center),
                                 'fov': self.fov}
            a = [camera.fov['sr'][i] - (camera.fov['s'][i] / 2.0) \
                 for i in range(2)]
            self.forward = camera.pose.map_rotate((0, 0, 1)).tuple
            self.up = camera.pose.map_rotate((0, -1, 0)).tuple
            self.center = camera.pose.map((0, 0, camera.params['zS'])).tuple
            self.fov = max(camera.fov['a'])
            # FIXME: manual zoom for now - this range stuff isn't working
            #extent = [camera.params['zS'] * max(-camera.fov['sl'][i], 
            #          camera.fov['sr'][i]) for i in range(2)]
            #self.range = camera.pose.map((extent[0], extent[1], 0)).tuple
        else:
            if not self._stored_view:
                return
            for key in self._stored_view:
                self.__setattr__(key, self._stored_view[key])
            self.up = (0, 0, 1)
            self._stored_view = None
            #self.autoscale, self.userzoom, self.userspin = [True] * 3
            self.autoscale, self.userspin = [True] * 2


class Experiment(object):
    """\
    Experiment class.
    """
    def __init__(self, model, display=Display()):
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

        Clicking a camera toggles its active state. CTRL + clicking a camera
        shows/hides its (approximate) field of view. Clicking a point prints
        its membership degree in the fuzzy coverage model. Other keybindings
        are F2 - update discrete fuzzy coverage model, F6 - show/hide camera
        names, F7 - show/hide axes, F8 - show/hide display center.
        """
        self.model.update_model()
        self.model.visualize(scale=30)
        self.axes = visual_axes(scale=30, color=(0, 0, 1))
        self.axes.visible = False
        self.cdot = visual.sphere(pos=self.display.center, radius=5,
            color=(0, 0, 1), material=visual.materials.emissive, visible=False)
        cam_vis = [primitive for objects in [vis.objects for vis in \
            [self.model[camera].vis for camera in self.model]] \
            for primitive in objects]
        point_vis = [primitive for objects in [vis.objects for vis in \
            [point.vis for point in self.model.model.keys()]] \
            for primitive in objects]
        # event loop
        while True:
            visual.rate(50)
            if self.display.mouse.events:
                m = self.display.mouse.getevent()
                if m.click == "left" and m.pick in cam_vis:
                    if m.ctrl:
                        m.pick.frame.camera.visualize_fov_toggle(scale=1500)
                    elif m.alt:
                        try:
                            self.display.camera_view(m.pick.frame.camera)
                            print "In camera view, F11 to exit."
                        except VisualizationError:
                            pass
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
                elif k == 'f11':
                    self.display.camera_view()
                    print "Exited camera view."
                elif k == 'f12':
                    print "Closing interactive event loop."
                    break
                # TODO: don't allow the rest in camera view
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


def visual_axes(scale=1.0, color=(1, 1, 1)):
    """\
    Display a set of 3D axes.

    @param scale: The scale of the axis set.
    @type scale: C{float}
    @param color: The color of the axes.
    @type color: C{tuple}
    """
    frame = visual.frame()
    axes = ['X', 'Y', 'Z']
    for axis in range(3):
        visual.arrow(frame=frame, shaftwidth=(scale / 10.0), color=color,
            axis=tuple([i == axis and scale * 5 or 0 for i in range(3)]))
        visual.label(frame=frame, height=6, color=(1, 1, 1), text=axes[axis],
            pos=tuple([i == axis and scale * 5.5 or 0 for i in range(3)]))
    return frame
