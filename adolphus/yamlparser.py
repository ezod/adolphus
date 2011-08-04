"""\
YAML parser module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import os
import yaml
from math import pi
from functools import reduce

import cython
from .coverage import PointCache, RelevanceModel, Scene, Camera, MultiCamera
from .geometry import Point, DirectionalPoint, Pose, Rotation, Quaternion
from .posable import Plane, SceneObject, Robot


class YAMLParser(object):
    """\
    YAML experiment parser class.
    """
    def __init__(self, filename, active=True):
        """\
        Constructor. Parses an experiment from YAML.

        @param filename: The YAML file to load from.
        @type filename: C{str}
        @param active: Default active state of cameras (optional).
        @type active: C{bool}
        """
        self._path = os.path.split(filename)[0]
        self._mounts = {}
        experiment = yaml.load(open(filename))
        self.model = self._parse_model(experiment['model'], active=active)
        self.relevances = {}
        try:
            for relevance in experiment['relevance']:
                self.relevances[relevance['name']] = \
                    self._parse_relevance(relevance)
        except KeyError:
            pass

    @property
    def experiment(self):
        """\
        Tuple containing the coverage model and relevance models for this
        experiment.
        """
        return self.model, self.relevances
    
    @staticmethod
    def _parse_pose(pose):
        """\
        Parse a pose from YAML.

        @param pose: The YAML dict of the pose.
        @type pose: C{dict}
        @return: The parsed pose.
        @rtype: L{Pose}
        """
        T = Point(pose['T'])
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
        return Pose(T, R)

    def _parse_primitives(self, sprite):
        """\
        Parse the primitives of a sprite.

        @param sprite: The YAML dict or filename of the sprite/posable.
        @type sprite: C{dict} or C{str}
        @return: The primitives list.
        @rtype: C{list} of C{dict}
        """
        tpath = self._path
        if isinstance(sprite, str):
            tpath = os.path.split(os.path.join(self._path, sprite))[0]
            sprite = yaml.load(open(os.path.join(self._path, sprite)))
        try:
            for primitive in sprite['primitives']:
                if 'texture' in primitive:
                    primitive['texture'] = os.path.join(tpath,
                        primitive['texture'])
            return sprite['primitives']
        except KeyError:
            return []

    def _parse_planes(self, sprite):
        """\
        Parse the opaque plane segments of a posable.

        @param sprite: The YAML dict or filename of the sprite/posable.
        @type sprite: C{dict} or C{str}
        @return: The planes list with parsed pose.
        @rtype: C{list} of C{dict}
        """
        if isinstance(sprite, str):
            sprite = yaml.load(open(os.path.join(self._path, sprite)))
        try:
            planes = sprite['planes']
            for plane in planes:
                try:
                    plane['pose'] = self._parse_pose(plane['pose'])
                except KeyError:
                    pass
            return planes
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
            robot = yaml.load(open(os.path.join(self._path, robot)))
        pieces = robot['pieces']
        for piece in pieces:
            piece['offset'] = self._parse_pose(piece['offset'])
        return pieces

    def _parse_scene(self, scene):
        """\
        Parse a scene model from YAML.

        @param scene: The YAML dict of the scene model.
        @type scene: C{dict}
        @return: The parsed scene model.
        @rtype: L{Scene}
        """
        rscene = Scene()
        for item in scene:
            # parse pose
            try:
                pose = self._parse_pose(item['pose'])
            except KeyError:
                pose = Pose()
            if 'sprites' in item:
                try:
                    mount_pose = self._parse_pose(item['mount_pose'])
                except KeyError:
                    mount_pose = Pose()
                # parse sprites and planes
                sprites = reduce(lambda a, b: a + b, [self._parse_primitives(sprite) \
                    for sprite in item['sprites']])
                planes = reduce(lambda a, b: a + b, [self._parse_planes(sprite) \
                    for sprite in item['sprites']])
                # create object
                rscene[item['name']] = SceneObject(pose or Pose(), mount_pose,
                    None, sprites, planes)
            elif 'robot' in item:
                pieces = self._parse_robot(item['robot'])
                try:
                    config = item['config']
                except KeyError:
                    config = None
                rscene[item['name']] = \
                    Robot(pose or Pose(), None, pieces, config)
            elif 'z' in item:
                rscene[item['name']] = \
                    Plane(pose, None, item['x'], item['y'], item['z'])
            elif 'x' in item:
                rscene[item['name']] = Plane(pose, None, item['x'], item['y'])
            else:
                rscene[item['name']] = \
                    SceneObject(pose or Pose(), mount_pose, None, [])
            self._mounts[item['name']] = rscene[item['name']]
        return rscene

    def _parse_model(self, model, active=True):
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
        if 'scene' in model:
            scene = self._parse_scene(model['scene'])
        else:
            scene = Scene()
        rmodel = MultiCamera(model['name'], scene)
        tp_defaults = {'boundary_padding': 0.0,
                       'res_max_ideal': float('inf'),
                       'res_max_acceptable': float('inf'),
                       'res_min_ideal': 0.0,
                       'res_min_acceptable': 0.0,
                       'blur_max_ideal': 1.0,
                       'blur_max_acceptable': 1.0,
                       'angle_max_ideal': pi / 2.0,
                       'angle_max_acceptable': pi / 2.0}
        for tp in tp_defaults:
            if not tp in model:
                model[tp] = tp_defaults[tp]
        model['res_max_ideal'] = \
            min(model['res_max_ideal'], model['res_max_acceptable'])
        model['res_min_ideal'] = \
            max(model['res_min_ideal'], model['res_min_acceptable'])
        model['blur_max_ideal'] = \
            min(model['blur_max_ideal'], model['blur_max_acceptable'])
        model['angle_max_ideal'] = \
            min(model['angle_max_ideal'], model['angle_max_acceptable'])
        for camera in model['cameras']:
            for tp in tp_defaults:
                camera[tp] = model[tp]
            # parse pose
            try:
                pose = self._parse_pose(camera['pose'])
            except KeyError:
                pose = Pose()
            # parse sprites
            sprites = reduce(lambda a, b: a + b, [self.\
                _parse_primitives(sprite) for sprite in camera['sprites']])
            sprites.append({'type': 'label', 'color': [1, 1, 1], 'height': 6,
                'background': [0, 0, 0], 'text': camera['name']})
            # create camera
            rmodel[camera['name']] = Camera(camera, pose, None, sprites,
                active=active)
            self._mounts[camera['name']] = rmodel[camera['name']]
        # parse mounts after to avoid ordering issues
        try:
            for item in model['scene']:
                if 'mount' in item:
                    rmodel.scene[item['name']].mount = self._mounts[item['mount']]
        except KeyError:
            pass
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
