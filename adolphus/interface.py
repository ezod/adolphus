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

from geometry import Point, DirectionalPoint
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

        # center marker
        self.cdot = visual.sphere(pos=self.center, radius=5, color=(0, 0, 1),
            material=visual.materials.emissive, visible=False)

        # axes
        self.axes = visual.frame(visible=False)
        axes = ['X', 'Y', 'Z']
        for axis in range(3):
            visual.arrow(frame=self.axes, shaftwidth=(mscale / 10.0),
                color=(0, 0, 1), axis=tuple([i == axis and mscale * 5 or 0 \
                for i in range(3)]))
            visual.label(frame=self.axes, height=6, color=(1, 1, 1),
                text=axes[axis], pos=tuple([i == axis and mscale * 5.5 or 0 \
                for i in range(3)]))

        # command/message box
        self._messagebox = visual.label(pos=center, height=12,
            color=(1, 1, 1), visible=False)

    def shift_center(self, direction=None):
        """\
        """
        if direction is None:
            direction = -self.center
        self.center += visual.vector(direction)
        self.cdot.pos = self.center
        self._messagebox.pos = self.center

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
        """\
        Return whether the display is in camera view.

        rtype: C{bool}
        """
        if self._stored_view:
            return True
        return False

    def message(self, text=None):
        """\
        Display a message on the screen.

        @param text: The message to display.
        @type text: C{str}
        """
        if not text:
            self._messagebox.visible = False
        else:
            self._messagebox.text = text
            self._messagebox.visible = True

    def prompt(self):
        """\
        Display a prompt and process the command entered by the user.

        @return: The command string to be processed by the parent object.
        @rtype: C{str}
        """
        self.userspin = False
        self.message(" ")
        cmd = ""
        process_cmd = False
        while True:
            visual.rate(100)
            if self.kb.keys:
                k = self.kb.getkey()
                if k == '\n':
                    process_cmd = True
                    self.message()
                    break
                elif k == 'backspace':
                    if len(cmd) > 0:
                        cmd = cmd[:-1]
                    else:
                        self.message()
                        break
                elif k.isalnum() or k in ' -_(),.':
                    cmd += k
            self.message(cmd + " ")
        self.userspin = True
        return process_cmd and cmd or None


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

        # interface commands
        # these should not raise KeyErrors
        def cmd_sc(args):
            """x y z"""
            self.display.shift_center((float(args[0]), float(args[1]),
                float(args[2])))

        def cmd_mu(args):
            """x y z [rho eta]"""
            if len(args) == 3:
                p = Point(tuple([float(args[i]) for i in range(3)]))
            elif len(args) == 5:
                p = DirectionalPoint(tuple([float(args[i]) for i in range(5)]))
            else:
                raise ValueError
            self.display.message(u'\u03bc%s = %.4f' % (p, self.model.mu(p)))

        def cmd_axes(args):
            self.display.axes.visible = not self.display.axes.visible

        def cmd_cdot(args):
            self.display.cdot.visible = not self.display.cdot.visible

        def cmd_name(args):
            self.model.visualize_name_toggle()

        def cmd_ptmu(args):
            self.model.visualize_ptmu_toggle()

        def cmd_camview(args):
            """name"""
            if len(args):
                try:
                    self.display.camera_view(self.model[args[0]])
                except KeyError:
                    self.display.message("Invalid camera name.")
            else:
                self.display.camera_view()

        def cmd_update(args):
            self.display.message("Updating discrete coverage model...")
            self.model.update_model()
            self.model.update_visualization()
            self.display.message()

        self.commands = {}
        for function in dir():
            if function.startswith('cmd_'):
                self.commands[function[4:]] = locals()[function]

    def execute(self, cmd):
        """\
        Execute an interface command.

        @param cmd: The command string to execute.
        @type cmd: C{str}
        """
        cmd, args = cmd.split()[0], cmd.split()[1:]
        try:
            self.commands[cmd](args)
        except KeyError:
            self.display.message("Invalid command.")
        except:
            self.display.message("Usage: %s %s" \
                % (cmd, self.commands[cmd].__doc__))

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
        cam_vis = [primitive for objects in [vis.objects for vis in \
            [self.model[camera].vis for camera in self.model]] \
            for primitive in objects]
        point_vis = [primitive for objects in [vis.objects for vis in \
            [point.vis for point in self.model.model.keys()]] \
            for primitive in objects]
        zoom = False
        spin = False
        msgctr = 0
        # event loop
        while True:
            visual.rate(self.rate)
            # clear mesages after a while
            if self.display._messagebox.visible:
                msgctr += 1
            if msgctr > 100:
                self.display.message()
                msgctr = 0
            # process mouse events
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
                if k == '\n':
                    cmd = self.display.prompt()
                    if cmd:
                        self.execute(cmd)
                elif k == 'f2':
                    self.execute("update")
                elif k == 'f3':
                    if len(self.relevance_models):
                        print "Computing coverage performance (%d)..." \
                            % len(self.relevance_models)
                        for map in self.relevance_models:
                            print "Performance:", self.model.performance(map)
                    else:
                        print "No relevance models."
                elif k == 'f5':
                    self.execute("ptmu")
                elif k == 'f6':
                    self.execute("name")
                elif k == 'f7':
                    self.execute("axes")
                elif k == 'f8':
                    self.execute("cdot")
                elif k == 'f11':
                    self.display.camera_view()
                    print "Exited camera view."
                elif k == 'f12':
                    print "Closing interactive event loop."
                    break
                elif k == 'left' and not self.display.in_camera_view:
                    self.execute("sc %f 0 0" % -self.model.scale)
                elif k == 'right' and not self.display.in_camera_view:
                    self.execute("sc %f 0 0" % self.model.scale)
                elif k == 'down' and not self.display.in_camera_view:
                    self.execute("sc 0 %f 0" % -self.model.scale)
                elif k == 'up' and not self.display.in_camera_view:
                    self.execute("sc 0 %f 0" % self.model.scale)
                elif k == 'page down' and not self.display.in_camera_view:
                    self.execute("sc 0 0 %f" % -self.model.scale)
                elif k == 'page up' and not self.display.in_camera_view:
                    self.execute("sc 0 0 %f" % self.model.scale)
