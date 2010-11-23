"""\
YAML parser module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import os
import imp
import yaml
from math import pi
from fuzz import PolygonalFuzzyNumber

from coverage import PointFuzzySet, Scene, Camera, MultiCamera
from geometry import Point, DirectionalPoint, Pose, Rotation, Plane, pointrange


def load_model_from_yaml(filename, active=True, apoverride=None):
    """\
    Load parameters for a multi-camera fuzzy coverage model from a YAML file.

    @param filename: The YAML file to load from.
    @type filename: C{str}
    @param active: The default active state of cameras (default True).
    @type active: C{bool}
    @param apoverride: An override set of application parameters.
    @type apoverride: C{dict}
    @return: The multi-camera fuzzy coverage model.
    @rtype: L{MultiCamera}
    """
    params = yaml.load(open(filename))

    # custom import
    try:
        external = imp.load_source(params['import'],
            os.path.join(os.path.split(filename)[0],
            params['import'] + '.py'))
    except ImportError:
        raise ImportError('could not load custom module')
    except KeyError:
        pass

    # scene
    scene = Scene()
    try:
        for widget in params['scene']:
            pose, mount, config = parse_widget(widget, scene)
            if widget.has_key('model'):
                wobject = getattr(external, widget['model'])(pose=pose,
                    mount=mount, config=config)
            elif widget.has_key('z'):
                wobject = Plane(x=widget['x'], y=widget['y'], z=widget['z'],
                    mount=mount)
            else:
                wobject = Plane(x=widget['x'], y=widget['y'], pose=pose,
                    mount=mount)
            scene[widget['name']] = wobject
    except KeyError:
        pass

    # cameras
    for i in [1, 2]:
        if params['zeta%d' % i] == 'max':
            params['zeta%d' % i] = pi / 2.0
    model = MultiCamera(name=params['name'], ocular=params['ocular'],
                        scene=scene, scale=params['scale'])
    for camera in params['cameras']:
        if apoverride:
            for ap in ['gamma', 'r1', 'r2', 'cmax', 'zeta1', 'zeta2']:
                camera[ap] = apoverride[ap]
        else:
            for ap in ['gamma', 'r1', 'r2', 'cmax', 'zeta1', 'zeta2']:
                camera[ap] = params[ap]
        pose, mount, config = parse_widget(camera, scene)
        model[camera['name']] = Camera(camera, active=active,
            pose=pose, mount=mount, config=config,
            models=[getattr(external, mdl) for mdl in camera['models']])

    # relevance models
    relevance_models = {}
    try:
        for rmodel in params['relevance']:
            relevance_models[rmodel['name']] = \
                generate_relevance_model(rmodel, scene)
    except KeyError:
        pass

    return model, relevance_models


def parse_rotation(R, format):
    """\
    Parse a rotation into a proper rotation object.

    @param R: The rotation to parse.
    @type R: C{list}
    @param format: The format of the rotation.
    @type format: C{str}
    @return: The rotation object.
    @rtype: L{Rotation}
    """
    if format == 'quaternion':
        return Rotation(R)
    elif format == 'matrix':
        return Rotation.from_rotation_matrix(R)
    elif format == 'axis-angle':
        return Rotation.from_axis_angle(R[0], R[1])
    elif format.startswith('euler'):
        convention, unit = format.split('-')[1:]
        if unit == 'deg':
            R = [r * pi / 180.0 for r in R]
        return Rotation.from_euler(convention, (R[0], R[1], R[2]))
    else:
        raise ValueError('unrecognized rotation format')


def parse_widget(widget, mounts):
    """\
    Aggregate the full pose of an object based on pose and mount.

    @param widget: The object with pose/mount.
    @type widget: C{dict}
    @param mounts: A dict populated with mounts.
    @type mounts: C{dict}
    @return: The full pose.
    @rtype: L{Pose}
    """
    if widget.has_key('pose'):
        pose = Pose(T=Point(widget['pose']['T']),
        R=parse_rotation(widget['pose']['R'], widget['pose']['Rformat']))
    else:
        pose = Pose()
    if mounts and widget.has_key('mount'):
        mount = mounts[widget['mount']]
    else:
        mount = None
    if widget.has_key('config'):
        config = widget['config']
    else:
        config = None
    return pose, mount, config


def generate_relevance_model(params, mounts=None):
    """\
    Generate a relevance model from a set of x-y-z polygonal fuzzy sets.

    @param params: The parameters (from YAML) for the polygonal fuzzy sets.
    @type params: C{dict}
    @return: The relevance model.
    @rtype: L{PointFuzzySet}
    """
    whole_model = PointFuzzySet()
    # ranges
    try:
        ranges, extent = {}, {}
        if params.has_key('ddiv'):
            ddiv = params['ddiv']
        else:
            ddiv = None
        for range in params['ranges']:
            if range.has_key('mu'):
                mu = range['mu']
            else:
                mu = 1.0
            if range.has_key('rho'):
                rhorange = range['rho']
            else:
                rhorange = (0.0, pi)
            if range.has_key('eta'):
                etarange = range['eta']
            else:
                etarange = (0.0, 2 * pi)
            part_model = PointFuzzySet()
            for point in pointrange(range['x'], range['y'], range['z'],
                params['step'], rhorange, etarange, ddiv=ddiv):
                part_model.add(point, mu=mu)
            whole_model |= part_model
    except KeyError:
        pass
    # points
    try:
        part_model = PointFuzzySet()
        for point in params['points']:
            if len(point['point']) == 3:
                pointobject = Point(point['point'])
            elif len(point['point']) == 5:
                pointobject = DirectionalPoint(point['point'])
            mu = 1.0
            if point.has_key('mu'):
                mu = point['mu']
            part_model.add(pointobject, mu=mu)
        whole_model |= part_model
    except KeyError:
        pass
    whole_model.pose, whole_model.mount = parse_widget(params, mounts)[:2]
    return whole_model
