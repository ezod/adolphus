"""\
Visual interface module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import sys
from math import tan
from time import sleep

from geometry import Point
from visualization import VisualizationError, visual


class Display(visual.display):
    """\
    Visual display class.
    """
    def __init__(self, name="Untitled Model", center=(0, 0, 0), mscale=1.0,
                 background=(1, 1, 1), foreground=(0.3, 0.3, 0.3)):
        """\
        Contstructor.

        @param title: Title of the window.
        @type title: C{str}
        @param center: Location of the center point.
        @type center: C{tuple} of C{float}
        @param mscale: The scale of the model.
        @type mscale: C{float}
        @param background: The background color of the scene.
        @type background: C{tuple}
        @param foreground: The default foreground color of the scene.
        @type foreground: C{tuple}
        """
        if not visual:
            raise VisualizationError("visual module not loaded")
        visual.display.__init__(self, title="Adolphus - %s" % name,
            center=center, background=background, foreground=foreground)
        self.forward = (-1, -1, -1)
        self.up = (0, 0, 1)
        self.userzoom = False
        self.range = mscale * 50
        self.rmin = mscale
        self.rmax = mscale * 300
        self._stored_view = None

    def camera_view(self, camera=None):
        """\
        Toggle camera view, displaying the scene from the point of view of a
        specific camera.

        @param camera: The camera object.
        @type camera: L{coverage.Camera}
        """
        if camera:
            if self._stored_view:
                raise VisualizationError("already in a camera view")
            self.userspin = False
            camera.vis.visible = False
            self._stored_view = {'camera': camera,
                                 'forward': tuple(self.forward),
                                 'center': tuple(self.center),
                                 'fov': self.fov,
                                 'range': self.range}
            self.center = camera.pose.map((0, 0, camera.params['zS'])).tuple
            self.forward = camera.pose.map_rotate((0, 0, 1)).tuple
            self.up = camera.pose.map_rotate((0, -1, 0)).tuple
            self.fov = max(camera.fov['a'])
            # FIXME: zoom still isn't exactly right
            self.range = max(max(camera.fov['sl']), max(camera.fov['sr'])) \
                * camera.params['zS'] * 1.5
        else:
            if not self._stored_view:
                return
            for key in self._stored_view:
                self.__setattr__(key, self._stored_view[key])
            self.up = (0, 0, 1)
            self._stored_view['camera'].vis.visible = True
            self.userspin = True
            self._stored_view = None

    @property
    def in_camera_view(self):
        if self._stored_view:
            return True
        return False


class Experiment(object):
    """\
    Experiment class.
    """
    def __init__(self, model, relevance_models=[]):
        """\
        Constructor.

        @param model: The multi-camera model to use.
        @type model: L{coverage.MultiCamera}
        @param relevance_models: The list of relevance models for performance.
        @type relevance_models: C{list} of L{fuzz.FuzzySet}
        """
        if not visual:
            raise VisualizationError("visual module not loaded")
        self.model = model
        self.relevance_models = relevance_models
        self.display = Display(name=model.name, mscale=model.scale)
        self.display.select()
        self.rate = 50

    def run(self):
        """\
        Run this experiment.

        Clicking a camera toggles its active state. CTRL + clicking a camera
        shows/hides its (approximate) field of view. Clicking a point prints
        its membership degree in the fuzzy coverage model. Other keybindings
        are F2 - update discrete fuzzy coverage model, F6 - show/hide camera
        names, F7 - show/hide axes, F8 - show/hide display center.
        """
        self.model.visualize()
        self.axes = visual_axes(scale=self.model.scale, color=(0, 0, 1))
        self.axes.visible = False
        self.cdot = visual.sphere(pos=self.display.center, radius=5,
            color=(0, 0, 1), material=visual.materials.emissive, visible=False)
        cam_vis = [primitive for objects in [vis.objects for vis in \
            [self.model[camera].vis for camera in self.model]] \
            for primitive in objects]
        point_vis = [primitive for objects in [vis.objects for vis in \
            [point.vis for point in self.model.model.keys()]] \
            for primitive in objects]
        zoom = False
        spin = False
        # event loop
        while True:
            visual.rate(self.rate)
            if self.display.mouse.events:
                m = self.display.mouse.getevent()
                if m.drag == "middle" and not self.display.in_camera_view:
                    zoom = True
                    lastpos = m.pos
                elif m.drop == "middle":
                    zoom = False
                elif m.click == "left" and m.pick in cam_vis:
                    if m.ctrl:
                        m.pick.frame.parent.visualize_fov_toggle(scale=\
                            (self.model.scale * 50))
                    elif m.alt:
                        try:
                            self.display.camera_view(m.pick.frame.parent)
                            print "In camera view, F11 to exit."
                        except VisualizationError:
                            pass
                    else:
                        m.pick.frame.parent.active = \
                            not m.pick.frame.parent.active
                        m.pick.frame.parent.update_visualization()
                elif m.click == "left" and m.pick in point_vis:
                    print "mu%s = %f" % (m.pick.frame.parent,
                        self.model.model[m.pick.frame.parent].mu)
            elif zoom:
                newpos = self.display.mouse.pos
                if newpos != lastpos:
                    distance = (self.display.center \
                        - self.display.mouse.camera).mag
                    planex = -self.display.up.cross(self.display.forward)
                    planey = planex.cross(self.display.forward)
                    zdiff = (lastpos - newpos).proj(planey)
                    if zdiff.mag < self.display.rmin / 10.0:
                        continue
                    scaling = 10 ** ((zdiff.z / abs(zdiff.z)) * zdiff.mag / distance)
                    newrange = scaling * self.display.range.y
                    if self.display.rmin < newrange < self.display.rmax:
                        self.display.range = newrange
                        lastpos = scaling * newpos
            if self.display.kb.keys:
                k = self.display.kb.getkey()
                if k == 'f2':
                    print "Updating discrete fuzzy coverage model...",
                    sys.stdout.flush()
                    self.model.update_model()
                    self.model.update_visualization()
                    print "done."
                elif k == 'f3':
                    if len(self.relevance_models):
                        print "Computing coverage performance (%d)..." \
                            % len(self.relevance_models)
                        for map in self.relevance_models:
                            print "Performance:", self.model.performance(map)
                    else:
                        print "No relevance models."
                elif k == 'f5':
                    self.model.visualize_ptmu_toggle()
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
                elif k == 'left' and not self.display.in_camera_view:
                    self.display.center = (self.display.center[0] \
                        - self.model.scale, self.display.center[1],
                        self.display.center[2])
                elif k == 'right' and not self.display.in_camera_view:
                    self.display.center = (self.display.center[0] \
                        + self.model.scale, self.display.center[1],
                        self.display.center[2])
                elif k == 'down' and not self.display.in_camera_view:
                    self.display.center = (self.display.center[0],
                        self.display.center[1] - self.model.scale,
                        self.display.center[2])
                elif k == 'up' and not self.display.in_camera_view:
                    self.display.center = (self.display.center[0],
                        self.display.center[1] + self.model.scale,
                        self.display.center[2])
                elif k == 'page down' and not self.display.in_camera_view:
                    self.display.center = (self.display.center[0],
                        self.display.center[1], self.display.center[2] \
                        - self.model.scale)
                elif k == 'page up' and not self.display.in_camera_view:
                    self.display.center = (self.display.center[0],
                        self.display.center[1], self.display.center[2] \
                        + self.model.scale)
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
