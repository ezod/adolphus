"""\
Standard library of interface commands.

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

import cython
from .geometry import Point, DirectionalPoint
from .visualization import visual, Sprite
from .posable import Robot
from .yamlparser import YAMLParser

if visual:
    from visual.filedialog import get_file


class CommandError(Exception):
    pass


def cmd_open(ex, args, pickled):
    """filename"""
    cmd_clear(ex, [], False)
    cmd_modify(ex, [], False)
    cmd_indicate(ex, [], False)
    cmd_camview(ex, [], False)
    # TODO: clean up other visual stuff
    try:
        del ex.model
    except AttributeError:
        pass
    try:
        ex.model, ex.relevance_models = YAMLParser(args[0]).experiment
    except IndexError:
        ex.model, ex.relevance_models = YAMLParser(get_file().name).experiment
    ex.model.visualize()

def cmd_config(ex, args, pickled):
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
        ex.keybindings = config['keybindings']
    except KeyError:
        ex.keybindings = {}

def cmd_sc(ex, args, pickled):
    """x y z"""
    if ex.display.in_camera_view:
        raise CommandError('cannot shift center in camera view')
    ex.display.shift_center((float(args[0]), float(args[1]),
        float(args[2])))

def cmd_strength(ex, args, pickled):
    """ocular x y z [rho eta]"""
    ocular = int(args.pop(0))
    if len(args) == 3:
        p = Point([float(args[i]) for i in range(3)])
    elif len(args) == 5:
        p = DirectionalPoint([float(args[i]) for i in range(5)])
    else:
        raise CommandError('invalid point')
    if pickled:
        return pickle.dumps(ex.model.strength(p, ocular=ocular))
    else:
        return '%f#' % ex.model.strength(p, ocular=ocular)

def cmd_axes(ex, args, pickled):
    ex.display.axes.visible = not ex.display.axes.visible

def cmd_cdot(ex, args, pickled):
    ex.display.cdot.visible = not ex.display.cdot.visible

def cmd_planes(ex, args, pickled):
    for posable in ex.model.scene:
        ex.model.scene[posable].toggle_planes()

def cmd_name(ex, args, pickled):
    for camera in ex.model:
        for display in ex.model[camera].actuals:
            for member in ex.model[camera].actuals[display].members:
                if isinstance(member, visual.label):
                    member.visible = not member.visible

def cmd_pose(ex, args, pickled):
    """name [rformat]"""
    try:
        # TODO: should work for any object - need to flatten namespace
        pose = ex.model[args[0]].pose
        if pickled:
            return pickle.dumps(pose)
        else:
            flatpose = tuple(pose.T)
            try:
                if args[1] == 'quaternion':
                    raise IndexError
                elif args[1] == 'matrix':
                    flatpose += tuple(pose.R.to_rotation_matrix().flatten())
                elif args[1] == 'axis-angle':
                    axis, angle = pose.R.to_axis_angle()
                    flatpose += tuple(axis) + (angle,)
                elif args[1] == 'euler-zyx':
                    flatpose += tuple(pose.R.to_euler_zyx())
                else:
                    raise CommandError('invalid rotation format')
            except IndexError:
                flatpose += (pose.R.Q.a,) + tuple(pose.R.Q.v)
            return ','.join([str(float(e)) for e in flatpose]) + '#'
    except KeyError:
        raise CommandError('invalid camera name')

def cmd_camview(ex, args, pickled):
    """name"""
    if ex.zoom:
        raise CommandError('cannot do camera view with external zoom enabled')
    if len(args):
        try:
            ex.display.camera_view(ex.model[args[0]])
            ex.modifier.visible = False
            ex.modifier.parent = None
        except KeyError:
            raise CommandError('invalid camera name')
    else:
        ex.display.camera_view()

def cmd_indicate(ex, args, pickled):
    """name"""
    if len(args):
        try:
            ex.indicator.pos = ex.model[args[0]].pose.T
            ex.indicator.visible = True
            ex.modifier.parent = ex.model[args[0]]
        except KeyError:
            raise CommandError('invalid camera name')
    else:
        ex.indicator.visible = False
        ex.modifier.parent = None

def cmd_modify(ex, args, pickled):
    """name"""
    if len(args):
        try:
            ex.modifier.pos = ex.model[args[0]].pose.T
            ex.modifier.visible = True
            ex.modifier.parent = ex.model[args[0]]
        except KeyError:
            raise CommandError('invalid camera name')
    else:
        ex.modifier.visible = False
        ex.modifier.parent = None

def cmd_fov(ex, args, pickled):
    """name"""
    if args[0] in ex.fovvis:
        ex.fovvis[args[0]].visible = not ex.fovvis[args[0]].visible
    else:
        try:
            ex.fovvis[args[0]] = Sprite(ex.model[args[0]].fovvis)
            ex.fovvis[args[0]].frame = ex.model[args[0]].actuals['main']
        except KeyError:
            raise CommandError('invalid camera name')

def cmd_showval(ex, args, pickled):
    """name [value]"""
    if args[0] in ex.valvis:
        ex.valvis[args[0]].visible = False
        del ex.valvis[args[0]]
    try:
        value = float(args[1])
        ex.valvis[args[0]] = Sprite([{'type': 'cylinder',
            'color': (0.8, 0.8, 0.8), 'pos': (80, -30, 0), 'axis': (0,
            (1.0 - value) * 60, 0), 'radius': 6}, {'type': 'cylinder',
            'color': (1, 0, 0), 'pos': (80, -30 + (1.0 - value) * 60.0,
            0), 'axis': (0, value * 60.0, 0), 'radius': 6}])
        try:
            ex.valvis[args[0]].frame = ex.model[args[0]].actuals['main']
        except KeyError:
            raise CommandError('invalid camera name')
    except IndexError:
        pass

def cmd_active(ex, args, pickled):
    """name"""
    try:
        ex.model[args[0]].active = not ex.model[args[0]].active
        ex.model[args[0]].update_visualization()
    except IndexError:
        for camera in ex.model:
            cmd_active(ex, [camera], False)
    except KeyError:
        raise CommandError('invalid camera name')

def cmd_position(ex, args, pickled):
    """robot [position]"""
    try:
        assert isinstance(ex.model.scene[args[0]], Robot)
        if len(args) > 1:
            ex.model.scene[args[0]].config = [float(arg) for arg in args[1:]]
            ex.model.scene[args[0]].update_visualization()
        else:
            if pickled:
                return pickle.dumps(ex.model.scene[args[0]].config)
            else:
                return ','.join([str(e) for e in \
                    ex.model.scene[args[0]].config]) + '#'
    except AssertionError:
        raise CommandError('not a robot')
    except KeyError:
        raise CommandError('invalid robot name')

def cmd_coverage(ex, args, pickled):
    """ocular name*"""
    try:
        ex.display.message('Calculating coverage...')
        ex.display.userspin = False
        performance = {}
        ocular = int(args.pop(0))
        if not args:
            args = ex.relevance_models.keys()
        for arg in args:
            ex.coverage[arg] = \
                ex.model.coverage(ex.relevance_models[arg],
                ocular=ocular)
            ex.coverage[arg].visualize()
            performance[arg] = ex.model.performance(\
                ex.relevance_models[arg], ocular=ocular,
                coverage=ex.coverage[arg])
        if pickled:
            return pickle.dumps(performance)
        else:
            return ','.join(['%s:%f' % (key, performance[key]) \
                for key in performance]) + '#'
        
    except KeyError:
        raise CommandError('invalid relevance model name')
    except ValueError:
        raise CommandError('too few active cameras')
    finally:
        ex.display.userspin = True

def cmd_clear(ex, args, pickled):
    for key in ex.coverage.keys():
        del ex.coverage[key]

def cmd_eval(ex, args, pickled):
    """code"""
    return str(eval(' '.join(args)))

def cmd_exit(ex, args, pickled):
    ex.display.visible = False
    ex.exit = True
