"""\
Standard library of interface commands.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import yaml
from hypergraph.orientation import minimum_maximum_weighted_indegree_orientation

import cython
from .geometry import Point, DirectionalPoint
from .visualization import visual, Sprite
from .posable import Robot
from .robotpanel import RobotPanel
from .yamlparser import YAMLParser

if visual:
    from visual.filedialog import get_file


def cmd_open(ex, args):
    """filename"""
    cmd_clear(ex, [])
    cmd_modify(ex, [])
    cmd_indicate(ex, [])
    cmd_camview(ex, [])
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

def cmd_config(ex, args):
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

def cmd_sc(ex, args):
    """x y z"""
    if ex.display.in_camera_view:
        ex.display.message('Cannot shift center in camera view.')
        return
    ex.display.shift_center((float(args[0]), float(args[1]),
        float(args[2])))

def cmd_strength(ex, args):
    """ocular x y z [rho eta]"""
    ocular = int(args.pop(0))
    if len(args) == 3:
        p = Point([float(args[i]) for i in range(3)])
    elif len(args) == 5:
        p = DirectionalPoint([float(args[i]) for i in range(5)])
    else:
        raise ValueError
    ex.display.message(u'\u03bc%s = %.4f' \
        % (p, ex.model.strength(p, ocular=ocular)))

def cmd_axes(ex, args):
    ex.display.axes.visible = not ex.display.axes.visible

def cmd_cdot(ex, args):
    ex.display.cdot.visible = not ex.display.cdot.visible

def cmd_planes(ex, args):
    for posable in ex.model.scene:
        ex.model.scene[posable].toggle_planes()

def cmd_name(ex, args):
    for camera in ex.model:
        for display in ex.model[camera].actuals:
            for member in ex.model[camera].actuals[display].members:
                if isinstance(member, visual.label):
                    member.visible = not member.visible

def cmd_pose(ex, args):
    """name"""
    try:
        ex.display.message('T: (%.2f, %.2f, %.2f)\n' \
            % ex.model[args[0]].pose.T + \
            u'R: \u03d1 = %.2f, \u03d5 = %.2f, \u0471 = %.2f' \
            % ex.model[args[0]].pose.R.to_euler_zyx())
    except KeyError:
        ex.display.message('Invalid camera name.')

def cmd_camview(ex, args):
    """name"""
    if ex.zoom:
        return
    if len(args):
        try:
            ex.display.camera_view(ex.model[args[0]])
            ex.modifier.visible = False
            ex.modifier.parent = None
            ex.display.message('Camera view %s.' % args[0])
        except KeyError:
            ex.display.message('Invalid camera name.')
    else:
        ex.display.camera_view()
        ex.display.message()

def cmd_indicate(ex, args):
    """name"""
    if len(args):
        try:
            ex.indicator.pos = ex.model[args[0]].pose.T
            ex.indicator.visible = True
            ex.modifier.parent = ex.model[args[0]]
        except KeyError:
            ex.display.message('Invalid camera name.')
    else:
        ex.indicator.visible = False
        ex.modifier.parent = None

def cmd_modify(ex, args):
    """name"""
    if len(args):
        try:
            ex.modifier.pos = ex.model[args[0]].pose.T
            ex.modifier.visible = True
            ex.modifier.parent = ex.model[args[0]]
        except KeyError:
            ex.display.message('Invalid camera name.')
    else:
        ex.modifier.visible = False
        ex.modifier.parent = None

def cmd_fov(ex, args):
    """name"""
    if args[0] in ex.fovvis:
        ex.fovvis[args[0]].visible = not ex.fovvis[args[0]].visible
    else:
        try:
            ex.fovvis[args[0]] = Sprite(ex.model[args[0]].fovvis)
            ex.fovvis[args[0]].frame = ex.model[args[0]].actuals['main']
        except KeyError:
            ex.display.message('Invalid camera name.')

def cmd_showval(ex, args):
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
            ex.display.message('Invalid camera name.')
    except IndexError:
        pass

def cmd_active(ex, args):
    """name"""
    try:
        ex.model[args[0]].active = not ex.model[args[0]].active
        ex.model[args[0]].update_visualization()
    except IndexError:
        for camera in ex.model:
            cmd_active(ex, [camera])
    except KeyError:
        ex.display.message('Invalid camera name.')

def cmd_position(ex, args):
    """robot [position]"""
    try:
        assert isinstance(ex.model.scene[args[0]], Robot)
        if len(args) > 1:
            ex.model.scene[args[0]].config = [float(arg) for arg in args[1:]]
            ex.model.scene[args[0]].update_visualization()
        else:
            ex.display.message(str(ex.model.scene[args[0]].config))
    except AssertionError:
        ex.display.message('Not a robot.')
    except KeyError:
        ex.display.message('Invalid robot name.')

def cmd_panel(ex, args):
    """[robot]"""
    try:
        assert isinstance(ex.model.scene[args[0]], Robot)
        ex._panel = RobotPanel(ex.model.scene[args[0]])
        ex._panel.start()
    except IndexError:
        ex._panel.quit = True
        ex._panel.join()
    except AssertionError:
        ex.display.message('Not a robot.')

def cmd_coverage(ex, args):
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
        ex.display.message('\n'.join(['%s: %.4f' % (key,
            performance[key]) for key in performance]))
    except KeyError:
        ex.display.message('Invalid relevance model name.')
    except ValueError:
        ex.display.message('Too few active cameras.')
    finally:
        ex.display.userspin = True

def cmd_clear(ex, args):
    for key in ex.coverage.keys():
        del ex.coverage[key]

def cmd_distribute(ex, args):
    """name"""
    try:
        ex.display.message('Calculating coverage hypergraph...')
        ex.display.userspin = False
        L = minimum_maximum_weighted_indegree_orientation(ex.model.\
            coverage_hypergraph(ex.relevance_models[args[0]]))
        D = {}
        for edge in L.edges:
            try:
                D[edge.head].append(frozenset(edge))
            except KeyError:
                D[edge.head] = [frozenset(edge)]
        ex.display.message()
    except KeyError:
        ex.display.message('Invalid relevance model name.')
    finally:
        ex.display.userspin = True

def cmd_eval(ex, args):
    """code"""
    ex.display.message(str(eval(' '.join(args))))

def cmd_exit(ex, args):
    ex.display.visible = False
    ex.exit = True
