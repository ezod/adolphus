"""\
Visual interface module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import sys
import yaml
from math import tan, copysign
from time import sleep

from geometry import Point, DirectionalPoint, Rotation, Pose
from visualization import visual, VisualizationObject, VisualizationError


class Display(visual.display):
    """\
    Visual display class.
    """
    def __init__(self, name='Untitled Model', center=(0, 0, 0), mscale=1.0,
                 background=(1, 1, 1), foreground=(0.3, 0.3, 0.3)):
        """\
        Contstructor.

        @param name: Title of the window.
        @type name: C{str}
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
            raise VisualizationError('visual module not loaded')
        visual.display.__init__(self, title='Adolphus - %s' % name,
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
                raise VisualizationError('already in a camera view')
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
                * camera.params['zS'] * 1.2
            self.cdot.pos = self.center
            self._messagebox.pos = self.center
        else:
            if not self._stored_view:
                return
            for key in self._stored_view:
                self.__setattr__(key, self._stored_view[key])
            self.up = (0, 0, 1)
            self._stored_view['camera'].vis.visible = True
            self.userspin = True
            self.cdot.pos = self.center
            self._messagebox.pos = self.center
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
        self.message(' ')
        cmd = ''
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
                elif k.isalnum() or k in " -_(),.[]+*%=|&:<>'":
                    cmd += k
            self.message(cmd + ' ')
        self.userspin = True
        return process_cmd and cmd or None


class Experiment(object):
    """\
    Experiment class.
    """
    def __init__(self, model, relevance_models={}, config_file=None):
        """\
        Constructor.

        @param model: The multi-camera model to use.
        @type model: L{coverage.MultiCamera}
        @param relevance_models: The relevance models for performance.
        @type relevance_models: C{dict} of L{FuzzySet}
        """
        if not visual:
            raise VisualizationError('visual module not loaded')
        self.model = model
        self.relevance_models = relevance_models
        self.display = Display(name=model.name, mscale=model.scale)
        self.display.select()
        self.rate = 50

        # camera modifier
        self.modifier = VisualizationObject(None)
        for arrow in [['xp', (model.scale * 3, 0, 0)],
                      ['xn', (-model.scale * 3, 0, 0)],
                      ['yp', (0, model.scale * 3, 0)],
                      ['yn', (0, -model.scale * 3, 0)],
                      ['zp', (0, 0, model.scale * 3)],
                      ['zn', (0, 0, -model.scale * 3)]]:
            self.modifier.add(arrow[0], visual.arrow(frame=self.modifier,
                pos=arrow[1], axis=arrow[1], shaftwidth=(model.scale / 3.0),
                color=(0, 0.25, 0.75), material=visual.materials.emissive))
        for ring in [['rotx', (1, 0, 0)],
                     ['roty', (0, 1, 0)],
                     ['rotz', (0, 0, 1)]]:
            self.modifier.add(ring[0], visual.ring(frame=self.modifier,
                radius=(model.scale * 3), thickness=(model.scale / 6.0),
                axis=ring[1], color=(0, 0.25, 0.75),
                material=visual.materials.emissive))
        self.modifier.visible = False

        # load and parse config file
        if config_file:
            config = yaml.load(open(config_file))
        else:
            import pkg_resources
            config = yaml.load(pkg_resources.resource_string(__name__,
                'resources/config.yaml'))
        try:
            self.keybindings = config['keybindings']
        except KeyError:
            self.keybindings = {}

        # interface commands
        # these should not raise KeyErrors
        def cmd_sc(args):
            """x y z"""
            if self.display.in_camera_view:
                self.display.message('Cannot shift center in camera view.')
                return
            self.display.shift_center((float(args[0]), float(args[1]),
                float(args[2])))

        def cmd_mu(args):
            """x y z [rho eta]"""
            if len(args) == 3:
                p = Point([float(args[i]) for i in range(3)])
            elif len(args) == 5:
                p = DirectionalPoint([float(args[i]) for i in range(5)])
            else:
                raise ValueError
            self.display.message(u'\u03bc%s = %.4f' % (p, self.model.mu(p)))

        def cmd_axes(args):
            self.display.axes.visible = not self.display.axes.visible

        def cmd_cdot(args):
            self.display.cdot.visible = not self.display.cdot.visible

        def cmd_name(args):
            self.model.visualize_name_toggle()

        def cmd_clearpoints(args):
            self.model.visualize_clear_points()

        def cmd_pose(args):
            """name"""
            try:
                self.display.message('T: %s\n' % self.model[args[0]].pose.T + \
                    u'R: \u03d1 = %0.2f, \u03d5 = %0.2f, \u0471 = %0.2f' \
                    % self.model[args[0]].pose.R.to_euler_zyx())
            except KeyError:
                self.display.message('Invalid camera name.')

        def cmd_camview(args):
            """name"""
            if len(args):
                try:
                    self.display.camera_view(self.model[args[0]])
                    self.modifier.visible = False
                    self.modifier.parent = None
                    self.display.message('Camera view %s.' % args[0])
                except KeyError:
                    self.display.message('Invalid camera name.')
            else:
                self.display.camera_view()
                self.display.message()

        def cmd_modify(args):
            """name"""
            if len(args):
                try:
                    self.modifier.pos = self.model[args[0]].pose.T.tuple
                    self.modifier.visible = True
                    self.modifier.parent = self.model[args[0]]
                except KeyError:
                    self.display.message('Invalid camera name.')
            else:
                self.modifier.visible = False
                self.modifier.parent = None

        def cmd_fov(args):
            """name"""
            try:
                self.model[args[0]].visualize_fov_toggle()
            except KeyError:
                self.display.message('Invalid camera name.')

        def cmd_active(args):
            """name"""
            try:
                self.model[args[0]].active = not self.model[args[0]].active
                self.model[args[0]].update_visualization()
            except KeyError:
                self.display.message('Invalid camera name.')

        def cmd_coverage(args):
            """name"""
            try:
                self.display.message('Calculating coverage...')
                self.display.userspin = False
                perf = self.model.visualize_coverage(\
                    self.relevance_models[args[0]])
                self.display.message('Performance (%s): %.4f' % (args[0], perf))
            except KeyError:
                self.display.message('Invalid relevance model name.')
            except ValueError:
                self.display.message('Too few active cameras.')
            finally:
                self.display.userspin = True

        def cmd_eval(args):
            """code"""
            self.display.message(str(eval(' '.join(args))))

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
            self.display.message('Invalid command.')
        except Exception as e:
            if self.commands[cmd].__doc__:
                es = str(e)
                es += '\nUsage: %s %s' % (cmd, self.commands[cmd].__doc__)
                self.display.message(es)


    def run(self):
        """\
        Run this experiment.

        Clicking a camera toggles its active state. CTRL + clicking a camera
        shows/hides its (approximate) field of view. Clicking a point prints
        its membership degree in the fuzzy coverage model.
        """
        self.model.visualize()
        cam_vis = [primitive for objects in [self.model[camera].vis.primitives \
            for camera in self.model] for primitive in objects]
        zoom = False
        spin = False
        moving = None
        rotating = None
        msgctr = 0
        # event loop
        while True:
            visual.rate(self.rate)
            # clear mesages after a while
            if self.display._messagebox.visible:
                msgctr += 1
            if msgctr > 10 * len(self.display._messagebox.text):
                self.display.message()
                msgctr = 0
            # process mouse events
            if self.display.mouse.events:
                m = self.display.mouse.getevent()
                if m.drag == 'middle' and not self.display.in_camera_view:
                    zoom = True
                    lastpos = m.pos
                elif m.drop == 'middle':
                    zoom = False
                elif m.click == 'left' and m.pick in cam_vis:
                    if m.ctrl:
                        m.pick.frame.parent.visualize_fov_toggle()
                    elif m.alt:
                        try:
                            for camera in self.model.keys():
                                if m.pick.frame.parent == self.model[camera]:
                                    self.execute('camview %s' % camera)
                        except VisualizationError:
                            pass
                    elif m.shift:
                        if self.modifier.parent == m.pick.frame.parent:
                            self.execute('modify')
                        else:
                            for camera in self.model.keys():
                                if m.pick.frame.parent == self.model[camera]:
                                    self.execute('modify %s' % camera)
                    else:
                        m.pick.frame.parent.active = \
                            not m.pick.frame.parent.active
                        m.pick.frame.parent.update_visualization()
                elif m.drag == 'left' and m.pick in self.modifier.primitives:
                    for name in self.modifier.members.keys():
                        if m.pick == self.modifier.members[name]:
                            if name.startswith('rot'):
                                rotating = name[3]
                                axes = {'x': (1, 0, 0),
                                        'y': (0, 1, 0),
                                        'z': (0, 0, 1)}
                                lastpos = self.display.mouse.project(normal=\
                                    axes[rotating], point=self.modifier.pos)
                                m.pick.color = (0, 1, 0)
                            else:
                                moving = name[0]
                                lastpos = self.display.mouse.project(normal=\
                                    self.display.forward,
                                    point=self.modifier.pos)
                                for name in self.modifier.members.keys():
                                    if moving and name.startswith(moving):
                                        self.modifier.members[name].color = (0, 1, 0)
                elif m.drop == 'left' and (moving or rotating):
                    for name in self.modifier.members.keys():
                        self.modifier.members[name].color = (0, 0.25, 0.75)
                    moving = None
                    rotating = None
            elif moving:
                newpos = self.display.mouse.project(normal=self.display.forward,
                    point=self.modifier.pos)
                if newpos != lastpos:
                    setattr(self.modifier.parent.pose.T, moving,
                        getattr(self.modifier.parent.pose.T, moving) + getattr(\
                        newpos, moving) - getattr(lastpos, moving))
                    self.modifier.pos = self.modifier.parent.pose.T.tuple
                    self.modifier.parent.update_visualization()
                    lastpos = newpos
            elif rotating:
                axes = {'x': (1, 0, 0), 'y': (0, 1, 0), 'z': (0, 0, 1)}
                newpos = self.display.mouse.project(normal=axes[rotating],
                    point=self.modifier.pos)
                if newpos != lastpos:
                    planex = -self.display.up.cross(self.display.forward)
                    planey = planex.cross(self.display.forward)
                    zdiff = (lastpos - newpos).proj(planey)
                    if zdiff.mag < self.display.rmin / 10.0:
                        continue
                    self.modifier.parent.pose.R += \
                        Rotation.from_axis_angle(copysign(zdiff.mag,
                        zdiff.z) * 0.01, (-self.modifier.parent.pose\
                        ).map_rotate(Point(axes[rotating])))
                    self.modifier.parent.update_visualization()
                    lastpos = newpos
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
                    scaling = 10 ** (copysign(zdiff.mag, zdiff.z) / distance)
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
                elif k == 'left':
                    self.execute('sc %f 0 0' % -self.model.scale)
                elif k == 'right':
                    self.execute('sc %f 0 0' % self.model.scale)
                elif k == 'down':
                    self.execute('sc 0 %f 0' % -self.model.scale)
                elif k == 'up':
                    self.execute('sc 0 %f 0' % self.model.scale)
                elif k == 'page down':
                    self.execute('sc 0 0 %f' % -self.model.scale)
                elif k == 'page up':
                    self.execute('sc 0 0 %f' % self.model.scale)
                elif self.keybindings.has_key(k):
                    self.execute(self.keybindings[k])
