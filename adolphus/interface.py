"""\
Visual interface module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import pyximport; pyximport.install()

import yaml
from math import copysign
from hypergraph.orientation import minimum_maximum_weighted_indegree_orientation

from .geometry import Point, DirectionalPoint, Rotation
from .coverage import MultiCamera
from .visualization import visual, VisualizationError, Sprite, Visualizable
from .yamlparser import parse_experiment

if visual:
    from visual.filedialog import get_file


class Display(visual.display):
    """\
    Visual display class.
    """
    def __init__(self, zoom=False, center=(0, 0, 0), background=(1, 1, 1),
                 foreground=(0.3, 0.3, 0.3)):
        """\
        Contstructor.

        @param center: Location of the center point.
        @type center: C{tuple} of C{float}
        @param background: The background color of the scene.
        @type background: C{tuple}
        @param foreground: The default foreground color of the scene.
        @type foreground: C{tuple}
        """
        visual.display.__init__(self, title='Adolphus', center=center,
            background=background, foreground=foreground)
        self.forward = (-1, -1, -1)
        self.up = (0, 0, 1)
        self.userzoom = zoom
        self.range = 1500
        self.rmin = 30
        self.rmax = 18000
        self._stored_view = None

        # center marker
        self.cdot = visual.sphere(pos=self.center, radius=5, color=(0, 0, 1),
            material=visual.materials.emissive, visible=False)

        # axes
        self.axes = visual.frame(visible=False)
        axes = ['X', 'Y', 'Z']
        for axis in range(3):
            visual.arrow(frame=self.axes, shaftwidth=3,
                color=(0, 0, 1), axis=tuple([i == axis and 150 or 0 \
                for i in range(3)]))
            visual.label(frame=self.axes, height=6, color=(1, 1, 1),
                text=axes[axis], pos=tuple([i == axis and 165 or 0 \
                for i in range(3)]))

        # command/message box
        self._messagebox = visual.label(pos=center, height=12,
            color=(1, 1, 1), visible=False)

    def shift_center(self, direction=None):
        """\
        Shift the center of the view.

        @param direction: The vector by which to shift the center.
        @type direction: L{visual.vector}
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
            camera.visible = False
            self._stored_view = {'camera': camera,
                                 'forward': tuple(self.forward),
                                 'center': tuple(self.center),
                                 'fov': self.fov,
                                 'range': self.range}
            self.center = camera.pose.map((0, 0, camera.params['zS']))
            self.forward = camera.pose.R.rotate((0, 0, 1))
            self.up = camera.pose.R.rotate((0, -1, 0))
            self.fov = max(camera.fov['ah'], camera.fov['av'])
            # FIXME: zoom still isn't exactly right
            self.range = max(camera.fov['sahl'], camera.fov['sahr'],
                             camera.fov['savb'], camera.fov['savt']) \
                         * camera.params['zS'] * 1.2
            self.cdot.pos = self.center
            self._messagebox.pos = self.center
        else:
            if not self._stored_view:
                return
            for key in self._stored_view:
                self.__setattr__(key, self._stored_view[key])
            self.up = (0, 0, 1)
            self._stored_view['camera'].visible = True
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
                elif k.isalnum() or k in " -_(),.[]+*%=|&:<>'~/\\":
                    cmd += k
            self.message(cmd + ' ')
        self.userspin = True
        return process_cmd and cmd or None


class Experiment(object):
    """\
    Experiment class.
    """
    def __init__(self, model_file=None, config_file=None, zoom=False):
        """\
        Constructor.

        @param model_file: The YAML file for the model.
        @type model_file: C{str}
        @param config_file: The YAML configuration file.
        @type config_file: C{str}
        @param zoom: Use Visual's userzoom (disables camera view).
        @type zoom: C{bool}
        """
        if not visual:
            raise VisualizationError('visual module not loaded')

        # main display
        self.zoom = zoom
        self.display = Display(zoom=zoom)
        Visualizable.displays['main'] = self.display
        self.display.select()
        self.rate = 50

        # model and config functions - can also be used during runtime
        # these should not raise KeyErrors
        def cmd_open(args):
            """filename"""
            del self.model
            try:
                self.model, self.relevance_models = parse_experiment(args[0])
            except IndexError:
                self.model, self.relevance_models = parse_experiment(get_file().name)
            self.model.visualize()

        def cmd_config(args):
            """filename"""
            try:
                config = yaml.load(open(args[0]))
            except IndexError:
                config = yaml.load(get_file())
            except TypeError:
                import pkg_resources
                config = yaml.load(pkg_resources.resource_string(__name__,
                    'resources/config.yaml'))
            try:
                self.keybindings = config['keybindings']
            except KeyError:
                self.keybindings = {}

        if model_file:
            cmd_open([model_file])
        else:
            self.model = MultiCamera()
        cmd_config([config_file])

        self.coverage = {}
        self.fovvis = {}

        # camera modifier
        primitives = []
        for arrow in [(90, 0, 0), (-90, 0, 0), (0, 90, 0),
                      (0, -90, 0), (0, 0, 90), (0, 0, -90)]:
            primitives.append({'type':          'arrow',
                               'pos':           arrow,
                               'axis':          arrow,
                               'shaftwidth':    10,
                               'color':         [0, 0.25, 0.75],
                               'material':      visual.materials.emissive})
        for ring in [(30, 0, 0), (0, 30, 0), (0, 0, 30)]:
            primitives.append({'type':          'ring',
                               'radius':        90,
                               'thickness':     6,
                               'axis':          ring,
                               'color':         [0, 0.25, 0.75],
                               'material':      visual.materials.emissive})
        self.modifier = Sprite(primitives)
        self.modifier.visible = False

        # interface commands
        # these should not raise KeyErrors
        def cmd_sc(args):
            """x y z"""
            if self.display.in_camera_view:
                self.display.message('Cannot shift center in camera view.')
                return
            self.display.shift_center((float(args[0]), float(args[1]),
                float(args[2])))

        def cmd_strength(args):
            """ocular x y z [rho eta]"""
            ocular = int(args.pop(0))
            if len(args) == 3:
                p = Point([float(args[i]) for i in range(3)])
            elif len(args) == 5:
                p = DirectionalPoint([float(args[i]) for i in range(5)])
            else:
                raise ValueError
            self.display.message(u'\u03bc%s = %.4f' \
                % (p, self.model.strength(p, ocular=ocular)))

        def cmd_axes(args):
            self.display.axes.visible = not self.display.axes.visible

        def cmd_cdot(args):
            self.display.cdot.visible = not self.display.cdot.visible

        def cmd_name(args):
            for camera in self.model.keys():
                for display in self.model[camera].actuals.keys():
                    for member in self.model[camera].actuals[display].members:
                        if isinstance(member, visual.label):
                            member.visible = not member.visible

        def cmd_pose(args):
            """name"""
            try:
                self.display.message('T: (%.2f, %.2f, %.2f)\n' \
                    % self.model[args[0]].pose.T + \
                    u'R: \u03d1 = %.2f, \u03d5 = %.2f, \u0471 = %.2f' \
                    % self.model[args[0]].pose.R.to_euler_zyx())
            except KeyError:
                self.display.message('Invalid camera name.')

        def cmd_camview(args):
            """name"""
            if self.zoom:
                return
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
                    self.modifier.pos = self.model[args[0]].pose.T
                    self.modifier.visible = True
                    self.modifier.parent = self.model[args[0]]
                except KeyError:
                    self.display.message('Invalid camera name.')
            else:
                self.modifier.visible = False
                self.modifier.parent = None

        def cmd_fov(args):
            """name"""
            if args[0] in self.fovvis.keys():
                self.fovvis[args[0]].visible = not self.fovvis[args[0]].visible
            else:
                try:
                    self.fovvis[args[0]] = Sprite(self.model[args[0]].fovvis)
                    self.fovvis[args[0]].frame = self.model[args[0]].actuals['main']
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
            """ocular name*"""
            try:
                self.display.message('Calculating coverage...')
                self.display.userspin = False
                performance = {}
                ocular = int(args.pop(0))
                if not args:
                    args = self.relevance_models.keys()
                for arg in args:
                    self.coverage[arg] = \
                        self.model.coverage(self.relevance_models[arg],
                        ocular=ocular)
                    self.coverage[arg].visualize()
                    performance[arg] = self.model.performance(\
                        self.relevance_models[arg], ocular=ocular,
                        coverage=self.coverage[arg])
                self.display.message('\n'.join(['%s: %.4f' % (key,
                    performance[key]) for key in performance.keys()]))
            except KeyError:
                self.display.message('Invalid relevance model name.')
            except ValueError:
                self.display.message('Too few active cameras.')
            finally:
                self.display.userspin = True

        def cmd_clear(args):
            for key in self.coverage.keys():
                del self.coverage[key]

        def cmd_distribute(args):
            """name"""
            try:
                self.display.message('Calculating coverage hypergraph...')
                self.display.userspin = False
                L = minimum_maximum_weighted_indegree_orientation(self.model.\
                    coverage_hypergraph(self.relevance_models[args[0]]))
                D = {}
                for edge in L.edges:
                    try:
                        D[edge.head].append(frozenset(edge))
                    except KeyError:
                        D[edge.head] = [frozenset(edge)]
                print(D)
                self.display.message()
            except KeyError:
                self.display.message('Invalid relevance model name.')
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
        """
        cam_vis = [primitive for objects in \
            [self.model[cam].actuals['main'].objects for cam in self.model] \
            for primitive in objects]
        zoom = False
        moving = None
        rotating = None
        msgctr = 0
        self.execute('name')
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
                if m.drag == 'middle' and not self.display.in_camera_view \
                and not self.zoom:
                    zoom = True
                    lastpos = m.pos
                elif m.drop == 'middle':
                    zoom = False
                elif m.click == 'left' and m.pick in cam_vis:
                    if m.ctrl:
                        for camera in self.model.keys():
                            if m.pick.frame.parent == self.model[camera]:
                                self.execute('fov %s' % camera)
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
                elif m.drag == 'left' and m.pick in self.modifier.objects:
                    m.pick.color = (0, 1, 0)
                    if isinstance(m.pick, visual.arrow):
                        moving = Point(m.pick.axis).normal
                        lastpos = self.display.mouse.project(normal=\
                            self.display.forward, point=self.modifier.pos)
                    else:
                        rotating = Point(m.pick.axis).normal
                        lastpos = self.display.mouse.project(normal=rotating,
                            point=self.modifier.pos)
                elif m.drop == 'left' and (moving or rotating):
                    for member in self.modifier.objects:
                        member.color = (0, 0.25, 0.75)
                    moving = None
                    rotating = None
            elif moving:
                newpos = self.display.mouse.project(normal=self.display.forward,
                    point=self.modifier.pos)
                if newpos != lastpos:
                    if self.modifier.parent.mount:
                        self.modifier.parent._pose.T += \
                            (-self.modifier.parent.mount.mount_pose()).R.rotate\
                            (moving * (newpos * moving - lastpos * moving))
                    else:
                        self.modifier.parent._pose.T += \
                            moving * (newpos * moving - lastpos * moving)
                    self.modifier.pos = self.modifier.parent.pose.T
                    self.modifier.parent.update_visualization()
                    lastpos = newpos
            elif rotating:
                newpos = self.display.mouse.project(normal=rotating,
                    point=self.modifier.pos)
                if newpos != lastpos:
                    planex = -self.display.up.cross(self.display.forward)
                    planey = planex.cross(self.display.forward)
                    zdiff = (lastpos - newpos).proj(planey)
                    if zdiff.mag < self.display.rmin / 10.0:
                        continue
                    self.modifier.parent._pose.R += Rotation.from_axis_angle(\
                        copysign(zdiff.mag, zdiff.z) * 0.01,
                        (-self.modifier.parent.pose).R.rotate(rotating))
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
                elif self.keybindings.has_key(k):
                    self.execute(self.keybindings[k])
