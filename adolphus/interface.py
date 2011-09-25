"""\
Visual interface module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import threading
import socket
from math import copysign

import cython
import commands
from .commands import CommandError
from .geometry import Point, Rotation
from .coverage import MultiCamera
from .visualization import visual, VisualizationError, Sprite, Visualizable, \
                           vsettings


class Display(visual.display):
    """\
    Visual display class.
    """
    def __init__(self, zoom=False, center=(0, 0, 0)):
        """\
        Contstructor.

        @param zoom: Toggle user zoom enable.
        @type zoom: C{bool}
        @param center: Location of the center point.
        @type center: C{tuple} of C{float}
        """
        super(Display, self).__init__(title='Adolphus Viewer', center=center,
            background=(1, 1, 1), foreground=(0.3, 0.3, 0.3))
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
            visual.label(frame=self.axes, height=vsettings['textsize'], 
            color=(1, 1, 1), background=(0, 0, 0), text=axes[axis], 
            pos=tuple([i == axis and 165 or 0 for i in range(3)]))

        # command/message box
        self._messagebox = visual.label(pos=center, background=(0, 0, 0),
            height=vsettings['textsize'] * 2, color=(1, 1, 1), visible=False)

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
            self.center = camera.pose.map((0, 0, camera.getparam('zS')))
            self.forward = camera.pose.R.rotate((0, 0, 1))
            self.up = camera.pose.R.rotate((0, -1, 0))
            self.fov = max(camera.fov['ah'], camera.fov['av'])
            # FIXME: zoom still isn't exactly right
            self.range = max(camera.fov['sahl'], camera.fov['sahr'],
                             camera.fov['savb'], camera.fov['savt']) \
                         * camera.getparam('zS') * 1.2
            self.cdot.pos = self.center
            self._messagebox.pos = self.center
            self.message('Camera view.')
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
            self.message()

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

        self.coverage = {}
        self.fovvis = {}
        self.valvis = {}

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

        self.exit = False
        super(Experiment, self).__init__()

        # interface commands
        self.commands = {}
        for function in dir(commands):
            if function.startswith('cmd_'):
                self.commands[function[4:]] = getattr(commands, function)

        if model_file:
            self.commands['loadmodel'](self, [model_file], False)
        else:
            self.model = MultiCamera()
        self.commands['loadconfig'](self, [config_file], False)

    def execute(self, cmd, pickled=False):
        """\
        Execute a command.

        @param cmd: The command string to execute.
        @type cmd: C{str}
        @param pickled: Toggle client accepts pickled output.
        @type pickled: C{bool}
        @return: The return string of the command.
        @rtype: C{str}
        """
        cmd, args = cmd.split()[0], cmd.split()[1:]
        if cmd not in self.commands:
            raise CommandError('invalid command')
        try:
            return self.commands[cmd](self, args, pickled)
        except CommandError as e:
            es = str(e)
            if self.commands[cmd].__doc__:
                es += '\nusage: %s %s' % (cmd, self.commands[cmd].__doc__)
            return es

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
        while not self.exit:
            visual.rate(vsettings['rate'])
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
                        for camera in self.model:
                            if m.pick.frame.parent == self.model[camera]:
                                self.execute('fov %s' % camera)
                    elif m.alt:
                        try:
                            for camera in self.model:
                                if m.pick.frame.parent == self.model[camera]:
                                    self.execute('camview %s' % camera)
                        except VisualizationError:
                            pass
                    elif m.shift:
                        if self.modifier.parent == m.pick.frame.parent:
                            self.execute('modify')
                        else:
                            for camera in self.model:
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
                        self.display.message(self.execute(cmd))
                elif k in self.keybindings:
                    self.display.message(self.execute(self.keybindings[k]))


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

        Client settings are as follows:
            [0] - Accepts Python pickled responses.
        """
        self.experiment.start()
        # TODO: time out and check for experiment death here?
        channel, details = self.sock.accept()
        # get client settings
        client = ''
        while not client.endswith('#'):
            client += channel.recv(256)
        settings = [bool(int(v)) for v in client.strip('#')]
        # pad settings vector for backwards compatibility
        settings += [False] * (1 - len(settings))
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
                        pickled=settings[0])
                except CommandError:
                    pass
                if rstring:
                    channel.sendall(rstring)
        except Controller.ExperimentDeath:
            pass
        finally:
            channel.close()
            self.sock.close()
            if self.experiment.is_alive():
                self.experiment.execute('exit')
            self.experiment.join()
