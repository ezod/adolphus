"""\
YAML parser module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import os
import yaml
import pkg_resources
from math import pi
from functools import reduce
from copy import copy

from .coverage import PointCache, RelevanceModel, Camera, Model, TP_DEFAULTS
from .laser import LineLaser
from .geometry import Point, DirectionalPoint, Pose, Rotation, Quaternion
from .posable import SceneObject, SceneTriangle, Robot


class YAMLParser(object):
    """\
    YAML experiment parser class.
    """
    def __init__(self, filename):
        """\
        Constructor. Parses an experiment from YAML.

        @param filename: The YAML file to load from.
        @type filename: C{str}
        """
        self._path = os.path.split(filename)[0]
        self._mounts = {}
        experiment = yaml.load(open(filename))
        self.model = self._parse_model(experiment['model'])
        self.relevances = {}
        if not 'relevance' in experiment:
            experiment['relevance'] = []
        for relevance in experiment['relevance']:
            self.relevances[relevance['name']] = \
                self._parse_relevance(relevance)

    @property
    def experiment(self):
        """\
        Tuple containing the coverage model and relevance models for this
        experiment.
        """
        return self.model, self.relevances

    def _load_external(self, filename):
        """\
        Load an external YAML file specified inside a YAML model.

        @param filename: The external YAML file to load.
        @type filename: C{str}
        @return: The loaded YAML dict.
        @rtype: C{dict}
        """
        try:
            return yaml.load(open(os.path.join(self._path, filename)))
        except IOError:
            return yaml.load(pkg_resources.resource_string(__name__,
                'resources/' + filename))
    
    @staticmethod
    def _parse_pose(pose):
        """\
        Parse a pose from YAML.

        @param pose: The YAML dict of the pose.
        @type pose: C{dict}
        @return: The parsed pose.
        @rtype: L{Pose}
        """
        if 'T' in pose:
            T = Point(pose['T'])
        else:
            T = Point()
        if 'R' in pose:
            if pose['Rformat'] == 'quaternion':
                R = Rotation(Quaternion(pose['R']))
            elif pose['Rformat'] == 'matrix':
                R = Rotation.from_rotation_matrix(pose['R'])
            elif pose['Rformat'].startswith('axis-angle'):
                unit = pose['Rformat'].split('-')[2]
                if unit == 'deg':
                    angle = pose['R'][0] * pi / 180.0
                else:
                    angle = pose['R'][0]
                R = Rotation.from_axis_angle(angle, Point(pose['R'][1]))
            elif pose['Rformat'].startswith('euler'):
                convention, unit = pose['Rformat'].split('-')[1:]
                if unit == 'deg':
                    R = [r * pi / 180.0 for r in pose['R']]
                else:
                    R = pose['R']
                R = Rotation.from_euler(convention, (R[0], R[1], R[2]))
            else:
                raise ValueError('unrecognized rotation format')
        else:
            R = Rotation()
        return Pose(T, R)

    def _parse_primitives(self, sprite):
        """\
        Parse the primitives of a sprite.

        @param sprite: The YAML dict or filename of the sprite/posable.
        @type sprite: C{dict} or C{str}
        @return: The primitives list.
        @rtype: C{list} of C{dict}
        """
        tpath = ''
        if isinstance(sprite, str):
            tpath = os.path.split(sprite)[0]
            sprite = self._load_external(sprite)
        try:
            for primitive in sprite['primitives']:
                if 'texture' in primitive:
                    if os.path.exists(os.path.join(self._path, tpath,
                        primitive['texture'] + '.tga')):
                        primitive['texture'] = os.path.join(self._path, tpath,
                            primitive['texture'])
                    else:
                        primitive['texture'] = pkg_resources.resource_filename(\
                            __name__, os.path.join('resources/', tpath,
                            primitive['texture']))
            return sprite['primitives']
        except KeyError:
            return []

    def _parse_triangles(self, sprite):
        """\
        Parse the opaque triangles of a posable.

        @param sprite: The YAML dict or filename of the sprite/posable.
        @type sprite: C{dict} or C{str}
        @return: The triangle list with parsed pose.
        @rtype: C{list} of C{dict}
        """
        if isinstance(sprite, str):
            sprite = self._load_external(sprite)
        try:
            triangles = sprite['triangles']
            for triangle in triangles:
                try:
                    triangle['pose'] = self._parse_pose(triangle['pose'])
                except KeyError:
                    pass
            return triangles
        except KeyError:
            return []

    def _parse_robot(self, robot):
        """\
        Parse a robot from YAML.

        @param robot: The YAML list of the robot pieces.
        @type robot: C{list}
        @return: The parsed robot pieces.
        @rtype: C{list}
        """
        if isinstance(robot, str):
            robot = self._load_external(robot)
        pieces = robot['pieces']
        for piece in pieces:
            piece['offset'] = self._parse_pose(piece['offset'])
        return pieces

    def _parse_model(self, model):
        """\
        Parse a multi-camera model from YAML.

        @param model: The YAML dict of the multi-camera model.
        @type model: C{dict}
        @return: The parsed multi-camera model.
        @rtype: L{Model}
        """
        # complete and sanitize task paramters
        for tp in TP_DEFAULTS:
            if not tp in model:
                model[tp] = TP_DEFAULTS[tp]
        model['res_max_ideal'] = \
            min(model['res_max_ideal'], model['res_max_acceptable'])
        model['res_min_ideal'] = \
            max(model['res_min_ideal'], model['res_min_acceptable'])
        model['blur_max_ideal'] = \
            min(model['blur_max_ideal'], model['blur_max_acceptable'])
        model['angle_max_ideal'] = \
            min(model['angle_max_ideal'], model['angle_max_acceptable'])
        # create model
        task_params = {}
        for param in TP_DEFAULTS:
            try:
                task_params[param] = model[param]
            except KeyError:
                pass
        rmodel = Model(task_params=task_params)
        for objecttype in ['cameras', 'lasers', 'scene']:
            if not objecttype in model:
                model[objecttype] = []
        # parse cameras
        for camera in model['cameras']:
            if not 'features' in camera:
                camera['features'] = []
            # parse pose
            try:
                pose = self._parse_pose(camera['pose'])
            except KeyError:
                pose = Pose()
            # parse sprites
            sprites = reduce(lambda a, b: a + b, [self.\
                _parse_primitives(sprite) for sprite in camera['sprites']])
            # parse active state
            try:
                active = camera['active']
            except KeyError:
                active = True
            # create camera
            rmodel[camera['name']] = Camera(camera['name'], camera,
                camera['features'], pose=pose, mount=None, primitives=sprites,
                active=active)
            self._mounts[camera['name']] = rmodel[camera['name']]
        # parse lasers
        for laser in model['lasers']:
            # parse pose
            try:
                pose = self._parse_pose(laser['pose'])
            except KeyError:
                pose = Pose()
            # parse sprites
            sprites = reduce(lambda a, b: a + b, [self.\
                _parse_primitives(sprite) for sprite in laser['sprites']])
            # create laser
            rmodel[laser['name']] = LineLaser(laser['name'], laser['fan'],
                laser['depth'], pose=pose, mount=None, primitives=sprites)
            self._mounts[laser['name']] = rmodel[laser['name']]
        # parse scene
        for item in model['scene']:
            # parse pose
            try:
                pose = self._parse_pose(item['pose'])
            except KeyError:
                pose = Pose()
            try:
                occlusion = item['occlusion']
            except KeyError:
                occlusion = True
            try:
                mount_pose = self._parse_pose(item['mount_pose'])
            except KeyError:
                mount_pose = Pose()
            if 'sprites' in item:
                # parse sprites and triangles
                sprites = reduce(lambda a, b: a + b,
                    [self._parse_primitives(sprite) \
                    for sprite in item['sprites']])
                if occlusion:
                    triangles = reduce(lambda a, b: a + b,
                        [self._parse_triangles(sprite) \
                        for sprite in item['sprites']])
                else:
                    triangles = []
                # create object
                rmodel[item['name']] = SceneObject(item['name'],
                    pose=(pose or Pose()), mount_pose=mount_pose, mount=None,
                    primitives=sprites, triangles=triangles)
            elif 'robot' in item:
                pieces = self._parse_robot(item['robot'])
                try:
                    config = item['config']
                except KeyError:
                    config = None
                rmodel[item['name']] = Robot(item['name'],
                    pose=(pose or Pose()), mount=None, pieces=pieces,
                    config=config, occlusion=occlusion)
            elif 'vertices' in item:
                rmodel[item['name']] = SceneTriangle(item['name'],
                    item['vertices'], pose=pose, mount=None)
            else:
                rmodel[item['name']] = SceneObject(item['name'],
                    pose=(pose or Pose()), mount_pose=mount_pose)
            self._mounts[item['name']] = rmodel[item['name']]
        # parse mounts after to avoid ordering issues
        # FIXME: does mounting need to be this complicated anymore?
        for item in model['scene']:
            if 'mount' in item:
                rmodel[item['name']].mount = self._mounts[item['mount']]
        for camera in model['cameras']:
            if 'mount' in camera:
                rmodel[camera['name']].mount = self._mounts[camera['mount']]
        return rmodel

    @staticmethod
    def _pointrange(xr, yr, zr, step, rhor=(0.0, pi), etar=(0.0, 2 * pi),
                    ddiv=None):
        """\
        Generate discrete (directional) points in a range.

        @param xr: The range in the x direction.
        @type xr: C{tuple} of C{float}
        @param yr: The range in the y direction.
        @type yr: C{tuple} of C{float}
        @param zr: The range in the z direction.
        @type zr: C{tuple} of C{float}
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
                        while rho - rhor[1] < 1e-4 and rho - pi < 1e-4:
                            if abs(rho) < 1e-4 or abs(rho - pi) < 1e-4:
                                yield DirectionalPoint((x, y, z, rho, 0.0))
                            else:
                                while eta - etar[1] < 1e-4 and eta < 2 * pi:
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

    def _parse_relevance(self, relevance):
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
                for point in self._pointrange(prange['x'], prange['y'],
                    prange['z'], relevance['step'], rhorange, etarange,
                    ddiv=ddiv):
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
            pose = self._parse_pose(relevance['pose'])
        except KeyError:
            pose = Pose()
        if 'mount' in relevance:
            mount = self._mounts[relevance['mount']]
        else:
            mount = None
        return RelevanceModel(whole_model, pose=pose, mount=mount)
