"""\
YAML parser module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import pyximport; pyximport.install()

import os
import yaml
from math import pi

from coverage import PointCache, RelevanceModel, Scene, Camera, MultiCamera
from geometry import Point, DirectionalPoint, Pose, Rotation
from posable import Plane, SceneObject

PATH = '.'
MOUNTS = {}

def parse_pose(pose):
    """\
    Parse a pose from YAML.

    @param pose: The YAML dict of the pose.
    @type pose: C{dict}
    @return: The parsed pose.
    @rtype: L{Pose}
    """
    T = Point(pose['T'])
    if pose['Rformat'] == 'quaternion':
        R = Rotation(pose['R'])
    elif pose['Rformat'] == 'matrix':
        R = Rotation.from_rotation_matrix(pose['R'])
    elif pose['Rformat'].startswith('axis-angle'):
        unit = pose['Rformat'].split('-')[2]
        if unit == 'deg':
            angle = pose['R'][0] * pi / 180.0
        else:
            angle = pose['R'][0]
        R = Rotation.from_axis_angle(angle, pose['R'][1])
    elif pose['Rformat'].startswith('euler'):
        convention, unit = pose['Rformat'].split('-')[1:]
        if unit == 'deg':
            R = [r * pi / 180.0 for r in pose['R']]
        else:
            R = pose['R']
        R = Rotation.from_euler(convention, (R[0], R[1], R[2]))
    else:
        raise ValueError('unrecognized rotation format')
    return Pose(T, R)


def parse_primitives(sprite):
    """\
    Parse the primitives of a sprite.

    @param sprite: The YAML dict or filename of the sprite/posable.
    @type sprite: C{dict} or C{str}
    @return: The primitives list.
    @rtype: C{list} of C{dict}
    """
    if isinstance(sprite, str):
        sprite = yaml.load(open(os.path.join(PATH, sprite)))
    try:
        return sprite['primitives']
    except KeyError:
        return []


def parse_planes(sprite):
    """\
    Parse the opaque plane segments of a posable.

    @param sprite: The YAML dict or filename of the sprite/posable.
    @type sprite: C{dict} or C{str}
    @return: The planes list with parsed pose.
    @rtype: C{list} of C{dict}
    """
    if isinstance(sprite, str):
        sprite = yaml.load(open(os.path.join(PATH, sprite)))
    try:
        planes = sprite['planes']
        for plane in planes:
            try:
                plane['pose'] = parse_pose(plane['pose'])
            except KeyError:
                pass
        return planes
    except KeyError:
        return []


def parse_scene(scene):
    """\
    Parse a scene model from YAML.

    @param scene: The YAML dict of the scene model.
    @type scene: C{dict}
    @return: The parsed scene model.
    @rtype: L{Scene}
    """
    global MOUNTS
    rscene = Scene()
    for item in scene:
        # parse pose
        try:
            pose = parse_pose(item['pose'])
        except KeyError:
            pose = None
        try:
            mount = MOUNTS[item['mount']]
        except KeyError:
            mount = None
        if item.has_key('sprites'):
            # TODO: parse mount pose
            mount_pose = Pose()
            # parse sprites and planes
            sprites = reduce(lambda a, b: a + b, [parse_primitives(sprite) for sprite in item['sprites']])
            planes = reduce(lambda a, b: a + b, [parse_planes(sprite) for sprite in item['sprites']])
            # create object
            rscene[item['name']] = SceneObject(pose or Pose(), mount_pose, mount, planes, sprites)
        elif item.has_key('z'):
            rscene[item['name']] = Plane(pose, mount, item['x'], item['y'], item['z'])
        else:
            rscene[item['name']] = Plane(pose, mount, item['x'], item['y'])
        MOUNTS[item['name']] = rscene[item['name']]
    return rscene


def parse_model(model, active=True):
    """\
    Parse a multi-camera model from YAML.

    @param model: The YAML dict of the multi-camera model.
    @type model: C{dict}
    @param active: Default active state of cameras (optional).
    @type active: C{bool}
    @return: The parsed multi-camera model.
    @rtype: L{MultiCamera}
    """
    global MOUNTS
    if 'scene' in model.keys():
        scene = parse_scene(model['scene'])
    else:
        scene = Scene()
    rmodel = MultiCamera(model['name'], model['ocular'], scene)
    for i in [1, 2]:
        if model['zeta%d' % i] == 'max':
            model['zeta%d' % i] = pi / 2.0
    for camera in model['cameras']:
        for ap in ['gamma', 'r1', 'r2', 'cmax', 'zeta1', 'zeta2']:
            camera[ap] = model[ap]
        # parse pose
        try:
            pose = parse_pose(camera['pose'])
        except KeyError:
            pose = Pose()
        try:
            mount = MOUNTS[camera['mount']]
        except KeyError:
            mount = None
        mount = None
        # parse sprites
        sprites = reduce(lambda a, b: a + b, [parse_primitives(sprite) for sprite in camera['sprites']])
        sprites.append({'type': 'label', 'color': [1, 1, 1], 'height': 6, 'text': camera['name']})
        # create camera
        rmodel[camera['name']] = Camera(camera, pose, mount, sprites, active=active)
        MOUNTS[camera['name']] = rmodel[camera['name']]
    return rmodel


def pointrange(xr, yr, zr, step, rhor=(0.0, pi), etar=(0.0, 2 * pi), ddiv=None):
    """\
    Generate discrete (directional) points in a range.

    @param xrange: The range in the x direction.
    @type xrange: C{tuple} of C{float}
    @param yrange: The range in the y direction.
    @type yrange: C{tuple} of C{float}
    @param zrange: The range in the z direction.
    @type zrange: C{tuple} of C{float}
    @param step: The resolution of the discrete point set.
    @type step: C{float}
    @param ddiv: The fraction of pi for discrete direction angles.
    @type ddiv: C{int}
    """
    def rangify(r):
        try:
            return (r[0], r[1])
        except TypeError:
            return (r, r)
    xr = rangify(xr)    
    yr = rangify(yr)    
    zr = rangify(zr)    
    rhor = rangify(rhor)    
    etar = rangify(etar)
    x, y, z = xr[0], yr[0], zr[0]
    while x <= xr[1]:
        while y <= yr[1]:
            while z <= zr[1]:
                if ddiv:
                    rho, eta = rhor[0], etar[0]
                    while rho <= rhor[1] and rho <= pi:
                        if rho == 0.0 or rho == pi:
                            yield DirectionalPoint((x, y, z, rho, 0.0))
                        else:
                            while eta <= etar[1] and eta < 2 * pi:
                                yield DirectionalPoint((x, y, z, rho, eta))
                                eta += pi / ddiv
                        rho += pi / ddiv
                else:
                    yield Point((x, y, z))
                z += step
            z = zr[0]
            y += step
        y = yr[0]
        x += step


def parse_relevance(relevance):
    """\
    Parse a relevance model from YAML.
    
    @param relevance: The YAML dict of the relevance model.
    @type relevance: C{dict}
    @return: The parsed relevance model.
    @rtype: L{RelevanceModel}
    """
    whole_model = PointCache()
    try:
        try:
            ddiv = relevance['ddiv']
        except KeyError:
            ddiv = None
        for prange in relevance['ranges']:
            try:
                strength = prange['strength']
            except KeyError:
                strength = 1.0
            try:
                rhorange = prange['rho']
            except KeyError:
                rhorange = (0.0, pi)
            try:
                etarange = prange['eta']
            except KeyError:
                etarange = (0.0, 2 * pi)
            part_model = PointCache()
            for point in pointrange(prange['x'], prange['y'], prange['z'],
                relevance['step'], rhorange, etarange, ddiv=ddiv):
                part_model[point] = strength
            whole_model |= part_model
    except KeyError:
        pass
    try:
        part_model = PointCache()
        for point in relevance['points']:
            if len(point['point']) == 3:
                pointobject = Point(point['point'])
            elif len(point['point']) == 5:
                pointobject = DirectionalPoint(point['point'])
            try:
                strength = point['strength']
            except KeyError:
                strength = 1.0
            part_model[pointobject] = strength
        whole_model |= part_model            
    except KeyError:
        pass
    try:
        pose = parse_pose(relevance['pose'])
    except KeyError:
        pose = Pose()
    try:
        mount = MOUNTS[relevance['mount']]
    except KeyError:
        mount = None
    return RelevanceModel(whole_model, pose=pose, mount=mount)


def parse_experiment(filename, active=True):
    """\
    Parse an experiment from YAML.

    @param filename: The YAML file to load from.
    @type filename: C{str}
    @param active: Default active state of cameras (optional).
    @type active: C{bool}
    """
    global PATH
    PATH = os.path.split(filename)[0]
    experiment = yaml.load(open(filename))
    model = parse_model(experiment['model'], active=active)
    relevances = {}
    for relevance in experiment['relevance']:
        relevances[relevance['name']] = parse_relevance(relevance)
    return model, relevances
