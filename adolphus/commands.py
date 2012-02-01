"""\
Standard library of interface commands.

Commands are functions decorated with C{@command} and taking two positional
arguments: a reference to the experiment object, and a list of strings
comprising the command arguments.

A command may return data in the form of a string. If so, its base function
must accept a third (keyword) argument, C{response}, which defines the format
accepted by the interface and may take one of the following string values
(though it is acceptable to define base functions which reject some formats):

  - C{pickle} - pickled Python object generated with C{pickle.dumps()}
  - C{csv} - comma-delimited values terminated with hash (C{#}) character
  - C{text} - human-readable text format (can include newlines)

Commands may raise any type of exception, but these will be re-raised as
L{CommandError} so that they may be appropriately handled by the interface.

Custom commands may be added simply by importing the C{@command} decorator from
this module and wrapping an appropriately-formed base function.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

try:
    import cPickle as pickle
except ImportError:
    import pickle

import yaml
from copy import deepcopy
from inspect import getargspec

from .geometry import Point, DirectionalPoint
from .visualization import Sprite
from .robot import Robot
from .yamlparser import YAMLParser


commands = {}

class CommandError(Exception):
    pass

def command(f):
    if 'response' in getargspec(f).args:
        def wrapped(ex, args, response='pickle'):
            assert response in ['pickle', 'csv', 'text']
            try:
                return f(ex, args, response)
            except CommandError as e:
                raise e
            except Exception, e:
                raise CommandError('%s: %s' % (e.__class__.__name__, e))
        wrapped.response = True
    else:
        def wrapped(ex, args):
            try:
                return f(ex, args)
            except CommandError as e:
                raise e
            except Exception, e:
                raise CommandError('%s: %s' % (e.__class__.__name__, e))
        wrapped.response = False
    wrapped.__doc__ = f.__doc__
    commands[f.__name__] = wrapped
    return wrapped


@command
def loadmodel(ex, args):
    """\
    Load a model from a YAML file.

    usage: %s filename
    """
    clear(ex, [])
    modify(ex, [])
    cameraview(ex, [])
    cameranames(ex, [])
    for sceneobject in ex.model:
        ex.model[sceneobject].visible = False
    ex.model, ex.tasks = YAMLParser(args[0]).experiment
    ex.model.visualize()
    ex.display.select()

@command
def loadconfig(ex, args):
    """\
    Load viewer configuration from a YAML file.

    usage: %s filename
    """
    try:
        config = yaml.load(open(args[0]))
    except IndexError:
        import pkg_resources
        config = yaml.load(pkg_resources.resource_string(__name__,
            'resources/config.yaml'))
    try:
        ex.keybindings = config['keybindings']
    except KeyError:
        ex.keybindings = {}

@command
def event(ex, args):
    """\
    Set the generic experiment event.
    """
    ex.event.set()

@command
def exit(ex, args):
    """\
    Exit the viewer.
    """
    for display in ex.altdisplays:
        display.visible = False
    ex.display.visible = False
    ex.exit = True

@command
def clear(ex, args):
    """\
    Clear all point coverage visualizations.
    """
    for key in ex.coverage.keys():
        del ex.coverage[key]

@command
def axes(ex, args):
    """\
    Toggle display of 3D axes.
    """
    ex.axes.visible = not ex.axes.visible

@command
def centerdot(ex, args):
    """\
    Toggle display of a center indicator dot.
    """
    ex.centerdot.visible = not ex.centerdot.visible

@command
def setcenter(ex, args):
    """\
    Set the position of the display center.

    usage: %s x y z
    """
    if ex.display.in_camera_view:
        raise CommandError('cannot set center in camera view')
    pos = (float(args[0]), float(args[1]), float(args[2]))
    ex.display.set_center(pos)
    ex.centerdot.pos = pos

@command
def shiftcenter(ex, args):
    """\
    Shift the display center from its current position by the amount specified.

    usage: %s x y z
    """
    if ex.display.in_camera_view:
        raise CommandError('cannot shift center in camera view')
    pos = list(ex.display.center)
    for i in range(3):
        pos[i] += float(args[i])
    ex.display.set_center(tuple(pos))
    ex.centerdot.pos = tuple(pos)

@command
def getcenter(ex, args, response='pickle'):
    """\
    Get the position of the display center.

    usage: %s
    """
    if response == 'pickle':
        return pickle.dumps(Point(ex.centerdot.pos))
    elif response == 'csv':
        return '%s,%s,%s#' % tuple(ex.centerdot.pos)
    elif response == 'text':
        return '(%s, %s, %s)' % tuple(ex.centerdot.pos)

@command
def triangles(ex, args):
    """\
    Toggle display of occluding triangles.
    """
    for sceneobject in ex.model:
        ex.model[sceneobject].toggle_triangles()

@command
def cameranames(ex, args):
    """\
    Toggle display of camera identifiers.
    """
    ex.camera_names()

@command
def cameraview(ex, args):
    """\
    Switch to camera view for the specified camera.
    
    usage: %s name
    """
    if len(args):
        if ex.display.userzoom:
            raise CommandError('no camera view with external zoom enabled')
        ex.display.camera_view(ex.model[args[0]])
        ex.modifier.visible = False
        ex.modifier.parent = None
    else:
        try:
            ex.display.camera_view()
        except RuntimeError:
            pass

@command
def pose(ex, args, response='pickle'):
    """\
    Return the (absolute) pose of an object. If using CSV or text response,
    a rotation format may be specified (one of 'quaternion', 'matrix',
    'axis-angle', or 'euler-zyx').
    
    usage: %s name [rformat]
    """
    pose = ex.model[args[0]].pose
    if response == 'pickle':
        return pickle.dumps(pose)
    elif response == 'csv':
        flatpose = tuple(pose.T)
        try:
            if args[1] == 'quaternion':
                raise IndexError
            elif args[1] == 'matrix':
                flatpose += tuple(pose.R.to_rotation_matrix().flatten())
            elif args[1] == 'axis-angle':
                angle, axis = pose.R.to_axis_angle()
                flatpose += (angle,) + tuple(axis)
            elif args[1] == 'euler-zyx':
                flatpose += tuple(pose.R.to_euler_zyx())
            else:
                raise CommandError('invalid rotation format')
        except IndexError:
            flatpose += (pose.R.Q.a,) + tuple(pose.R.Q.v)
        return ','.join([str(float(e)) for e in flatpose]) + '#'
    elif response == 'text':
        tstr = 'T: (%.2f, %.2f, %.2f)\n' % ex.model[args[0]].pose.T
        try:
            if args[1] == 'quaternion':
                raise IndexError
            elif args[1] == 'matrix':
                return tstr + \
                    ('R:\t%.4f\t%.4f\t%.4f\n' \
                    + '\t%.4f\t%.4f\t%.4f\n' \
                    + '\t%.4f\t%.4f\t%.4f') \
                    % tuple(pose.R.to_rotation_matrix().flatten())
            elif args[1] == 'axis-angle':
                angle, axis = pose.R.to_axis_angle()
                return tstr + \
                    u'R: \u03d1 = %.2f about (%.2f, %.2f, %.2f)' \
                    % ((angle,) + axis)
            elif args[1] == 'euler-zyx':
                return tstr + \
                    u'R: \u03d1 = %.2f, \u03d5 = %.2f, \u0471 = %.2f' \
                    % pose.R.to_euler_zyx()
            else:
                raise CommandError('invalid rotation format')
        except IndexError:
            return tstr + 'R: %s' % (pose.R.Q,)

@command
def modify(ex, args):
    """\
    Enable interactive pose modification for the specified object, or disable
    modification if no object is specified.
    
    usage: %s [name]
    """
    if len(args) and not ex.modifier.parent == ex.model[args[0]]:
            ex.modifier.pos = ex.model[args[0]].pose.T
            ex.modifier.visible = True
            ex.modifier.parent = ex.model[args[0]]
    else:
        ex.modifier.visible = False
        ex.modifier.parent = None

@command
def position(ex, args, response='pickle'):
    """\
    Set the position of the specified robot, or return its position if no new
    position is specified.

    usage: %s robot [position]
    """
    assert isinstance(ex.model[args[0]], Robot)
    if len(args) > 1:
        ex.model[args[0]].config = [float(arg) for arg in args[1:]]
        ex.model[args[0]].update_visualization()
    else:
        if response == 'pickle':
            return pickle.dumps(ex.model[args[0]].config)
        elif response == 'csv':
            return ','.join([str(e) for e in \
                ex.model[args[0]].config]) + '#'
        elif response == 'text':
            return str(ex.model[args[0]].config)

@command
def active(ex, args):
    """\
    Toggle the active state of the specified camera (or, if unspecified, that
    of all cameras).
    
    usage: %s [camera]
    """
    try:
        ex.model[args[0]].active = not ex.model[args[0]].active
        ex.model[args[0]].update_visualization()
    except IndexError:
        for camera in ex.model.cameras:
            active(ex, [camera])

@command
def strength(ex, args, response='pickle'):
    """\
    Return the coverage strength of the specified point with respect to the
    task parameters of the specified task.
    
    usage: %s task x y z [rho eta]
    """
    task = ex.tasks[args.pop(0)]
    if len(args) == 3:
        p = Point([float(args[i]) for i in range(3)])
    elif len(args) == 5:
        p = DirectionalPoint([float(args[i]) for i in range(5)])
    else:
        raise CommandError('invalid point')
    strength = ex.model.strength(p, task.params)
    if response == 'pickle':
        return pickle.dumps(strength)
    elif response == 'csv':
        return '%f#' % strength
    elif response == 'text':
        return '%f' % strength

@command
def showtask(ex, args):
    """\
    Show the points of the specified task.
    
    usage: %s task
    """
    clear(ex, [])
    for arg in args:
        ex.coverage[arg] = deepcopy(ex.tasks[args[0]].mapped)
        ex.coverage[arg].visualize()

@command
def coverage(ex, args, response='pickle'):
    """\
    Return the coverage performance for the specified task(s).

    usage: %s task
    """
    clear(ex, [])
    ex.display.message('Calculating coverage...')
    try:
        ex.display.userspin = False
        performance = {}
        if not args:
            args = ex.tasks.keys()
        for arg in args:
            ex.coverage[arg] = ex.model.coverage(ex.tasks[arg])
            ex.coverage[arg].visualize()
            performance[arg] = ex.model.performance(ex.tasks[arg],
                coverage=ex.coverage[arg])
        if response == 'pickle':
            return pickle.dumps(performance)
        elif response == 'csv':
            return ','.join(['%s:%f' % (key, performance[key]) \
                for key in performance]) + '#'
        elif response == 'text':
            return '\n'.join(['%s: %.4f' % (key, performance[key]) \
                for key in performance])
    finally:
        ex.display.userspin = True

@command
def lrcoverage(ex, args, response='pickle'):
    """\
    Return the linear range imaging coverage performance based on laser and
    transport parameters.

    usage: %s task [laser] [tx ty tz]
    """
    clear(ex, [])
    ex.display.message('Calculating range imaging coverage...')
    try:
        ex.display.userspin = False
        d = 0
        laser = None
        try:
            if args[1] in ex.model.lasers:
                laser = args[1]
                d = 1
        except IndexError:
            pass
        try:
            taxis = Point([float(t) for t in args[1 + d:4 + d]])
        except IndexError:
            taxis = None
        ex.coverage['range'] = ex.model.range_coverage_linear(\
            ex.tasks[args[0]], laser=laser, taxis=taxis)
        ex.coverage['range'].visualize()
        performance = ex.model.performance(ex.tasks[args[0]],
            coverage=ex.coverage['range'])
        if response == 'pickle':
            return pickle.dumps(performance)
        elif response == 'csv':
            return 'range:%f#' % performance
        elif response == 'text':
            return 'range: %.4f' % performance
    finally:
        ex.display.userspin = True

@command
def showval(ex, args):
    """\
    Display a value in [0, 1] in 'bar graph' style next to the specified camera,
    or remove the display if no value is specified.
    
    usage: %s name [value]
    """
    if args[0] in ex.valvis:
        ex.valvis[args[0]].visible = False
        del ex.valvis[args[0]]
    try:
        value = float(args[1])
    except IndexError:
        pass
    else:
        ex.valvis[args[0]] = Sprite([{'type': 'cylinder',
            'color': (0.8, 0.8, 0.8), 'pos': (80, -30, 0), 'axis': (0,
            (1.0 - value) * 60, 0), 'radius': 6}, {'type': 'cylinder',
            'color': (1, 0, 0), 'pos': (80, -30 + (1.0 - value) * 60.0,
            0), 'axis': (0, value * 60.0, 0), 'radius': 6}])
        ex.valvis[args[0]].frame = ex.model[args[0]].actuals['main']

@command
def objecthierarchy(ex, args, response='pickle'):
    """\
    Return a scene object hierarchy in the form C{{'object': (parent, type)}}.

    usage: %s
    """
    if response != 'pickle':
        raise CommandError('command cannot return %s response' % response)
    hierarchy = {}
    for so in ex.model:
        try:
            hierarchy[so] = (ex.model[so].mount.name,
                ex.model[so].__class__)
        except AttributeError:
            hierarchy[so] = (None, ex.model[so].__class__)
    for task in ex.tasks:
        try:
            hierarchy[task] = (ex.tasks[task].mount.name,
                ex.tasks[task].__class__)
        except AttributeError:
            hierarchy[task] = (None, ex.tasks[task].__class__)
    return pickle.dumps(hierarchy)

@command
def select(ex, args):
    """\
    Select a scene object.

    usage: %s [object]
    """
    if not args:
        ex.select()
    else:
        ex.select(ex.model[args[0]])

@command
def eval(ex, args):
    """\
    Execute arbitrary code.
    
    usage: %s code
    """
    return str(eval(' '.join(args)))

@command
def help(ex, args, response='pickle'):
    """\
    Print the documentation for a command.

    usage: %s command
    """
    if response != 'text':
        raise CommandError('command cannot return %s response' % response)
    return commands[args[0]].__doc__.rstrip()
