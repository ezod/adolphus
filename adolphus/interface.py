"""\
Visual interface module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from threading import Thread, Event
from math import copysign

import commands
from .geometry import Point, Rotation, Pose
from .coverage import Model
from .posable import SceneObject
from .visualization import visual, VisualizationError, Sprite, Visualizable, \
                           vsettings


class Display(visual.display):
    """\
    Visual display class.

    The L{Display} class is a Visual display with some default properties, a few
    useful methods for controlling the view, and some messaging and prompting
    functionality.
    """
    def __init__(self, zoom=False, center=(0, 0, 0), title='Adolphus Viewer'):
        """\
        Constructor.

        @param zoom: Toggle user zoom enable.
        @type zoom: C{bool}
        @param center: Location of the center point.
        @type center: C{tuple} of C{float}
        """
        super(Display, self).__init__(title=title, center=center,
            background=(1, 1, 1), foreground=(0.3, 0.3, 0.3), visible=False)
        self.forward = (-1, -1, -1)
        self.up = (0, 0, 1)
        self.userzoom = zoom
        self.range = 1500
        self.rmin = 30
        self.rmax = 18000
        self.message_time = 0
        self._stored_view = None

        # command/message box
        self._messagebox = visual.label(pos=center, background=(0, 0, 0),
            height=vsettings['textsize'] * 2, color=(1, 1, 1), visible=False)

    def set_center(self, pos=(0, 0, 0)):
        """\
        Set the center of the view.

        @param pos: The position of the center.
        @type pos: C{tuple} of C{float}
        """
        self.center = visual.vector(pos)
        self._messagebox.pos = self.center

    def camera_view(self, camera=None):
        """\
        Toggle camera view, displaying the scene from the point of view of a
        specific camera.

        @param camera: The camera object.
        @type camera: L{coverage.Camera}
        """
        if self.userzoom:
            raise RuntimeError('zoom must be disabled for camera view')
        if camera:
            if self._stored_view:
                self.camera_view()
            self.userspin = False
            for display in camera.actuals:
                if Visualizable.displays[display] == self:
                    camera.actuals[display].visible = False
            self._stored_view = {'camera': camera,
                                 'forward': tuple(self.forward),
                                 'center': tuple(self.center),
                                 'fov': self.fov,
                                 'range': self.range}
            self.center = camera.pose.map((0, 0, camera.getparam('zS')))
            self.forward = camera.pose.R.rotate((0, 0, 1))
            self.up = camera.pose.R.rotate((0, -1, 0))
            self.fov = max(camera.fov['ah'], camera.fov['av'])
            # FIXME: zoom still isn't exactly right
            self.range = max(camera.fov['tahl'], camera.fov['tahr'],
                             camera.fov['tavb'], camera.fov['tavt']) \
                         * camera.getparam('zS') * 1.2
            self._messagebox.pos = self.center
        else:
            if not self._stored_view:
                return
            for display in self._stored_view['camera'].actuals:
                if Visualizable.displays[display] == self:
                    self._stored_view['camera'].actuals[display].visible = True
            del self._stored_view['camera']
            for key in self._stored_view:
                self.__setattr__(key, self._stored_view[key])
            self.up = (0, 0, 1)
            self.userspin = True
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
        self.message_time = 0
        if not text:
            self._messagebox.visible = False
        else:
            self._messagebox.text = text
            self._messagebox.visible = True

    def prompt(self, initial=''):
        """\
        Display a prompt and process the command entered by the user.

        @return: The command string to be processed by the parent object.
        @rtype: C{str}
        """
        self.userspin = False
        self.message(initial + ' ')
        cmd = ''
        process_cmd = False
        while True:
            visual.rate(vsettings['rate'] * 2)
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
            self.message(initial + ' ' + cmd)
        self.userspin = True
        return cmd if process_cmd else None


class Experiment(Thread):
    """\
    Experiment class.

    An L{Experiment} object is the main interface component of Adolphus. It
    manages a L{Model} along with any task models, and provides one or more
    Visual displays and command-based interaction.
    """
    def __init__(self, zoom=False):
        """\
        Constructor.

        @param zoom: Use Visual's userzoom (disables camera view).
        @type zoom: C{bool}
        """
        if not visual:
            raise VisualizationError('visual module not loaded')

        # displays
        self.display = Display(zoom=zoom)
        Visualizable.displays['main'] = self.display
        self.display.select()
        self.altdisplays = []

        # generic event flag
        self.event = Event()

        # state variables
        self.selected = None
        self._camera_names = True
        self.coverage = {}
        self.guides = {}
        self.exit = False
        self.select_time = 0

        # model and configuration data
        self.model = Model()
        self.tasks = {}
        self.prompt_enabled = False
        self.keybindings = {}
        self.mousebindings = {}

        # center marker
        self.centerdot = Sprite([{'type':       'sphere',
                                  'radius':     5,
                                  'color':      [0, 0, 1],
                                  'material':   visual.materials.emissive}])
        self.centerdot.visible = False
        
        # axes
        primitives = []
        for i, axname in enumerate(['X', 'Y', 'Z']):
            axis = [0, 0, 0]
            axis[i] += 150
            primitives.append({'type':          'arrow',
                               'axis':          axis,
                               'shaftwidth':    3,
                               'color':         [0, 0, 1],
                               'material':      visual.materials.emissive})
            axis[i] += 15
            primitives.append({'type':          'label',
                               'pos':           axis,
                               'text':          axname,
                               'height':        vsettings['textsize'],
                               'color':         [1, 1, 1],
                               'background':    [0, 0, 0]})
        self.axes = Sprite(primitives)
        self.axes.visible = False

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

        super(Experiment, self).__init__()

    def add_display(self, zoom=False):
        """\
        Add an alternate display to this experiment.

        @param zoom: Use Visual's userzoom (disables camera view).
        @type zoom: C{bool}
        """
        self.altdisplays.append(Display(zoom=zoom))
        Visualizable.displays['alt%d' % (len(self.altdisplays) - 1)] = \
            self.altdisplays[-1]
        if self.is_alive():
            self.altdisplays[-1].visible = True
            for sceneobject in self.model:
                self.model[sceneobject].visualize()
        self.display.select()

    def select(self, selection=None):
        """\
        Select an object in the scene.
        
        @param selection: The object to select.
        @type selection: L{SceneObject}
        """
        if self.selected:
            self.selected.unhighlight()
        if selection:
            selection.highlight()
        self.selected = selection
        self.select_time = 0

    def camera_names(self):
        """\
        Toggle display of camera names.
        """
        if not self.is_alive():
            return
        self._camera_names = not self._camera_names
        for camera in self.model.cameras:
            try:
                self.model[camera].nametag.visible = self._camera_names
            except AttributeError:
                self.model[camera].nametag = Sprite([{'type': 'label',
                    'color': [1, 1, 1], 'height': 6, 'background': [0, 0, 0],
                    'text': camera}])
                self.model[camera].nametag.frame = \
                    self.model[camera].actuals['main']

    def execute(self, cmd, response='pickle'):
        """\
        Execute a command.

        @param cmd: The command string to execute.
        @type cmd: C{str}
        @param response: Response format for client (see commands.py).
        @type response: C{str}
        @return: The return string of the command.
        @rtype: C{str}
        """
        cmd, args = cmd.split()[0], cmd.split()[1:]
        if cmd not in commands.commands:
            raise commands.CommandError('invalid command')
        try:
            if commands.commands[cmd].response:
                return commands.commands[cmd](self, args, response=response)
            else:
                return commands.commands[cmd](self, args)
        except commands.CommandError as e:
            es = str(e)
            if commands.commands[cmd].__doc__:
                for line in commands.commands[cmd].__doc__.split('\n'):
                    line = line.strip(' ')
                    if line.startswith('usage'):
                        es += '\n' + line % cmd
                        break
            raise commands.CommandError(es)

    def run(self):
        """\
        Run this experiment.
        """
        self.display.visible = True
        for display in self.altdisplays:
            display.visible = True
        zoom = False
        moving = None
        rotating = None
        # event loop
        while not self.exit:
            visual.rate(vsettings['rate'])
            # clear mesages after a while
            if self.display._messagebox.visible:
                self.display.message_time += 1
            if self.display.message_time > (vsettings['rate'] / 5) * \
                max(len(self.display._messagebox.text), 10):
                self.display.message()
            # clear selection highlight after a while
            if self.selected:
                self.select_time += 1
            else:
                self.select_time = 0
            if self.select_time > vsettings['rate'] * 2:
                self.selected.unhighlight()
                self.select_time = 0
            # process mouse events
            if self.display.mouse.events:
                m = self.display.mouse.getevent()
                if m.drag == 'middle' and not self.display.in_camera_view \
                and not self.display.userzoom:
                    zoom = True
                    lastpos = m.pos
                elif m.drop == 'middle':
                    zoom = False
                elif m.click == 'left':
                    try:
                        obj = m.pick.frame.parent
                        assert isinstance(obj, SceneObject)
                    except (AttributeError, AssertionError):
                        continue
                    try:
                        if m.ctrl:
                            self.execute(self.mousebindings[type(obj).__name__.\
                                lower()]['ctrl'] + ' ' + obj.name)
                        elif m.alt:
                            self.execute(self.mousebindings[type(obj).__name__.\
                                lower()]['alt'] + ' ' + obj.name)
                        elif m.shift:
                            self.execute(self.mousebindings[type(obj).__name__.\
                                lower()]['shift'] + ' ' + obj.name)
                        else:
                            self.execute(self.mousebindings[type(obj).__name__.\
                                lower()]['none'] + ' ' + obj.name)
                    except KeyError:
                        pass
                    except commands.CommandError as e:
                        self.display.message(str(e))
                elif m.drag == 'left' and m.pick in self.modifier.objects:
                    m.pick.color = (0, 1, 0)
                    if isinstance(m.pick, visual.arrow):
                        moving = Point(m.pick.axis).unit
                        lastpos = self.display.mouse.project(normal=\
                            self.display.forward, point=self.modifier.pos)
                    else:
                        rotating = Point(m.pick.axis).unit
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
                    newpose = Pose(self.modifier.parent.pose.T + moving * \
                        (newpos * moving - lastpos * moving),
                        self.modifier.parent.pose.R)
                    self.modifier.parent.set_absolute_pose(newpose)
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
                    newpose = Pose(self.modifier.parent.pose.T, self.modifier.\
                        parent.pose.R + Rotation.from_axis_angle(\
                        copysign(zdiff.mag, zdiff.z) * 0.01,
                        (-self.modifier.parent.pose).R.rotate(rotating)))
                    self.modifier.parent.set_absolute_pose(newpose)
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
            # process keyboard events
            if self.display.kb.keys:
                k = self.display.kb.getkey()
                if self.prompt_enabled and k == '\n':
                    cmd = self.display.prompt('Command:')
                    if cmd:
                        try:
                            self.display.message(self.execute(cmd,
                                response='text'))
                        except commands.CommandError as e:
                            self.display.message(str(e))
                elif k in self.keybindings:
                    try:
                        self.display.message(self.execute(self.keybindings[k],
                            response='text'))
                    except commands.CommandError as e:
                        self.display.message(str(e))
