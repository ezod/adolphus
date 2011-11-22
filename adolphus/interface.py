"""\
Visual interface module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

try:
    import cPickle as pickle
except ImportError:
    import pickle

import threading
import socket
from math import copysign
from inspect import getargspec

import cython
import commands
from .commands import CommandError
from .geometry import Point, Rotation, Pose
from .coverage import Model
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
            self.range = max(camera.fov['sahl'], camera.fov['sahr'],
                             camera.fov['savb'], camera.fov['savt']) \
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


class Experiment(threading.Thread):
    """\
    Experiment class.

    An L{Experiment} object is the main interface component of Adolphus. It
    manages a L{Model} along with any relevance models, and provides one or
    more Visual displays and command-based interaction.
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
        self.event = threading.Event()

        # state variables
        self.selected = None
        self._camera_names = False
        self.coverage = {}
        self.fovvis = {}
        self.laservis = {}
        self.valvis = {}
        self.exit = False

        # model and configuration data
        self.model = Model()
        self._sceneobject_vis = []
        self.relevance_models = {}
        self.keybindings = {}

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

        # interface commands
        self.commands = {}
        for function in dir(commands):
            if function.startswith('cmd_'):
                self.commands[function[4:]] = getattr(commands, function)

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
        if cmd not in self.commands:
            raise CommandError('invalid command')
        try:
            if 'response' in getargspec(self.commands[cmd]).args:
                return self.commands[cmd](self, args, response=response)
            else:
                return self.commands[cmd](self, args)
        except CommandError as e:
            es = str(e)
            if self.commands[cmd].__doc__:
                for line in self.commands[cmd].__doc__.split('\n'):
                    line = line.strip(' ')
                    if line.startswith('usage'):
                        es += '\n' + line % cmd
                        break
            raise CommandError(es)

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
        msgctr = 0
        # event loop
        while not self.exit:
            visual.rate(vsettings['rate'])
            # clear mesages after a while
            if self.display._messagebox.visible:
                msgctr += 1
            if msgctr > 10 * max(len(self.display._messagebox.text), 10):
                self.display.message()
                msgctr = 0
            # process mouse events
            if self.display.mouse.events:
                m = self.display.mouse.getevent()
                if m.drag == 'middle' and not self.display.in_camera_view \
                and not self.display.userzoom:
                    zoom = True
                    lastpos = m.pos
                elif m.drop == 'middle':
                    zoom = False
                elif m.click == 'left' and m.pick in self._sceneobject_vis:
                    try:
                        if m.ctrl:
                            self.execute(m.pick.frame.parent.click_actions['ctrl'])
                        elif m.alt:
                            self.execute(m.pick.frame.parent.click_actions['alt'])
                        elif m.shift:
                            self.execute(m.pick.frame.parent.click_actions['shift'])
                        else:
                            self.execute(m.pick.frame.parent.click_actions['none'])
                    except KeyError:
                        pass
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
                if k == '\n':
                    cmd = self.display.prompt()
                    if cmd:
                        try:
                            self.display.message(self.execute(cmd,
                                response='text'))
                        except CommandError as e:
                            self.display.message(str(e))
                elif k in self.keybindings:
                    try:
                        self.display.message(self.execute(self.keybindings[k],
                            response='text'))
                    except CommandError as e:
                        self.display.message(str(e))


class Controller(object):
    """\
    Socket-based IPC experiment controller class.
    """
    class ExperimentDeath(Exception):
        pass

    def __init__(self, experiment, port=0):
        """\
        Constructor.

        @param experiment: The experiment to control.
        @type experiment: L{Experiment}
        @param port: The port to which to bind the socket (optional).
        @type port: C{int}
        """
        self.experiment = experiment
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind(('localhost', port))
        self.sock.listen(0)

    @property
    def port(self):
        """\
        The port on which the controller is listening.
        """
        return self.sock.getsockname()[1]

    def main(self):
        """\
        Main loop.
        """
        self.experiment.start()
        # TODO: time out and check for experiment death here?
        channel, details = self.sock.accept()
        # get client response format
        client = ''
        while not client.endswith('#'):
            client += channel.recv(256)
        response = client.strip('#')
        try:
            channel.settimeout(0.1)
            while True:
                cmd = ''
                while not cmd.endswith('#'):
                    if not self.experiment.is_alive():
                        raise Controller.ExperimentDeath
                    try:
                        cmd += channel.recv(256)
                    except socket.error:
                        continue
                try:
                    rstring = self.experiment.execute(cmd.strip('#'),
                        response=response)
                except CommandError as e:
                    channel.sendall(pickle.dumps(e))
                if rstring:
                    channel.sendall(rstring)
                else:
                    channel.sendall(pickle.dumps(None))
        except Controller.ExperimentDeath:
            pass
        finally:
            channel.close()
            self.sock.close()
            if self.experiment.is_alive():
                self.experiment.execute('exit')
            self.experiment.join()
