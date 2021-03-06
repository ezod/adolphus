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
import pkg_resources
from itertools import chain


from .robot import Robot
from .solid import Solid
from .laser import RangeModel
from .coverage import PointCache, Model
from .tensor import CameraTensor, TensorModel
from .posable import OcclusionTriangle, SceneObject
from .geometry import Angle, Point, DirectionalPoint, Pose, Rotation, Quaternion


modeltypes = {'standard': Model, 'range': RangeModel, 'tensor': TensorModel}


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
        try:
            modeltype = modeltypes[experiment['type']]
        except KeyError:
            modeltype = modeltypes['standard']
        self.model = self._parse_model(experiment['model'], modeltype)
        self.tasks = {}
        if 'tasks' in experiment:
            for task in experiment['tasks']:
                self.tasks[task['name']] = self._parse_task(task, modeltype)

    @property
    def experiment(self):
        """\
        Tuple containing the coverage model and task models for this experiment.
        """
        return self.model, self.tasks

    @staticmethod
    def _external_path(basepath, filename):
        """\
        Return the path to an external file specified inside another file.
        Defaults to searching the package resources if the file is not found.

        @param basepath: The current base path context.
        @type basepath: C{str}
        @param filename: The filename of the external file.
        @type filename: C{str}
        @return: The path to the external file.
        @rtype: C{str}
        """
        for path in [os.path.join(basepath, filename),
            pkg_resources.resource_filename(__name__, 'resources/' + filename)]:
            if os.path.exists(path):
                return path
        raise IOError('external file %s not found' % filename)

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
            T = Point(*pose['T'])
        else:
            T = Point(0, 0, 0)
        if 'R' in pose:
            if pose['Rformat'] == 'quaternion':
                R = Rotation(Quaternion(pose['R'][0], Point(*pose['R'][1])))
            elif pose['Rformat'] == 'matrix':
                R = Rotation.from_rotation_matrix(pose['R'])
            elif pose['Rformat'].startswith('axis-angle'):
                unit = pose['Rformat'].split('-')[2]
                angle = Angle(pose['R'][0], unit)
                R = Rotation.from_axis_angle(angle, Point(*pose['R'][1]))
            elif pose['Rformat'].startswith('euler'):
                convention, unit = pose['Rformat'].split('-')[1:]
                R = [Angle(r, unit) for r in pose['R']]
                R = Rotation.from_euler(convention, Point(R[0], R[1], R[2]))
            else:
                raise ValueError('unrecognized rotation format')
        else:
            R = Rotation()
        return Pose(T, R)

    def _parse_primitives(self, sprite):
        """\
        Parse the primitives of a sprite from YAML.

        @param sprite: The YAML dict or filename of the sprite.
        @type sprite: C{dict} or C{str}
        @return: The primitives list.
        @rtype: C{list} of C{dict}
        """
        path = self._path
        if isinstance(sprite, str):
            sprite_file = self._external_path(path, sprite)
            sprite = yaml.load(open(sprite_file, 'r'))
            path = os.path.split(sprite_file)[0]
        try:
            for primitive in sprite['primitives']:
                if 'texture' in primitive:
                    primitive['texture'] = \
                        self._external_path(path, primitive['texture'] + '.tga')
            return sprite['primitives']
        except KeyError:
            return []

    def _parse_triangles(self, sprite, path):
        """\
        Parse the triangles of a sprite from YAML.

        @param sprite: The YAML dict or filename of the sprite.
        @type sprite: C{dict} or C{str}
        @param path: The current path context.
        @type path: C{str}
        @return: The triangles list.
        @rtype: C{list} of L{OcclusionTriangle}
        """
        if isinstance(sprite, str):
            sprite_file = self._external_path(path, sprite)
            sprite = yaml.load(open(sprite_file, 'r'))
            path = os.path.split(sprite_file)[0]
        try:
            triangles = sprite['triangles']
            if isinstance(triangles, str):
                # load from raw ASCII triangle mesh format
                filename = triangles
                triangles = []
                with open(self._external_path(path, filename), 'r') as f:
                    for line in f.readlines():
                        triangles.append({'vertices': [[float(c) for c in \
                        line.rstrip().split(' ')][i * 3:i * 3 + 3] \
                        for i in range(3)]})
            parsed_triangles = []
            for triangle in triangles:
                try:
                    triangle['pose'] = self._parse_pose(triangle['pose'])
                except KeyError:
                    pass
                try:
                    parsed_triangles.append(OcclusionTriangle(**triangle))
                except ValueError:
                    pass
            return parsed_triangles
        except KeyError:
            return []

    def _parse_links(self, robot):
        """\
        Parse the links of a robot from YAML.

        @param robot: The YAML dict or filename of the robot.
        @type robot: C{dict} or C{str}
        @return: The parsed robot links.
        @rtype: C{list}
        """
        path = self._path
        if isinstance(robot, str):
            robot_file = self._external_path(path, robot)
            robot = yaml.load(open(robot_file, 'r'))
            path = os.path.split(robot_file)[0]
        links = robot['links']
        for link in links:
            link['offset'] = self._parse_pose(link['offset'])
            link['triangles'] = self._parse_triangles(link, path)
        return links

    def _parse_model(self, model, modeltype):
        """\
        Parse a multi-camera model from YAML.

        @param model: The YAML dict of the multi-camera model.
        @type model: C{dict}
        @param modeltype: The model class.
        @type modeltype: C{type}
        @return: The parsed multi-camera model.
        @rtype: L{Model}
        """
        # create model object
        rmodel = modeltype()
        mounts = {}
        for objecttype in ['cameras', 'lasers', 'robots', 'scene']:
            if not objecttype in model:
                continue
            for obj in model[objecttype]:
                pose = self._parse_pose(obj['pose']) \
                    if 'pose' in obj else Pose()
                mount_pose = self._parse_pose(obj['mount_pose']) \
                    if 'mount_pose' in obj else Pose()
                if 'sprites' in obj:
                    primitives = list(chain.from_iterable(\
                        [self._parse_primitives(sprite) \
                        for sprite in obj['sprites']]))
                else:
                    primitives = []
                occlusion = bool(obj['occlusion']) \
                    if 'occlusion' in obj else True
                _solid = False
                if occlusion and 'sprites' in obj:
                    try:
                        file = obj['sprites'][0]['triangles']
                        if (file[-4:] == '.raw' or file[-4:] == '.dae') and \
                            objecttype == 'scene':
                            file = self._external_path(self._path, file)
                            _solid = True
                    except:
                        pass
                    if not _solid:
                        triangles = list(chain.from_iterable(\
                            [self._parse_triangles(sprite, self._path) \
                            for sprite in obj['sprites']]))
                else:
                    triangles = []
                _tensor = True if modeltype == TensorModel else False
                if (objecttype in rmodel.yaml) and (not _tensor):
                    rmodel[obj['name']] = rmodel.yaml[objecttype](obj['name'],
                        obj, pose=pose, mount_pose=mount_pose,
                        primitives=primitives, triangles=triangles)
                elif (objecttype in rmodel.yaml) and _tensor:
                    tp = obj['tp'] if 'tp' in obj else {}
                    rmodel[obj['name']] = CameraTensor(tp, obj['name'], obj, \
                        pose=pose, mount_pose=mount_pose, primitives=primitives, \
                        triangles=triangles)
                elif objecttype == 'robots':
                    links = self._parse_links(obj['robot'])
                    config = obj['config'] if 'config' in obj else None
                    rmodel[obj['name']] = Robot(obj['name'], pose=pose,
                        links=links, config=config, occlusion=occlusion)
                elif _solid:
                    rmodel[obj['name']] = Solid(file, obj['name'], pose, mount_pose)
                else:
                    rmodel[obj['name']] = SceneObject(obj['name'], pose=pose,
                        mount_pose=mount_pose, primitives=primitives,
                        triangles=triangles)
                if 'mount' in obj:
                    mounts[obj['name']] = obj['mount']
        for name in mounts:
            rmodel[name].mount = rmodel[mounts[name]]
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
                                yield DirectionalPoint(x, y, z, rho, 0.0)
                            else:
                                while eta - etar[1] < 1e-4 and eta < 2 * pi:
                                    yield DirectionalPoint(x, y, z, rho, eta)
                                    eta += pi / ddiv
                            rho += pi / ddiv
                    else:
                        yield Point(x, y, z)
                    z += step
                z = zr[0]
                y += step
            y = yr[0]
            x += step

    def _parse_task(self, task, modeltype):
        """\
        Parse a task model from YAML.

        @param task: The YAML dict of the task model.
        @type task: C{dict}
        @param modeltype: The model class.
        @type modeltype: C{type}
        @return: The parsed task model.
        @rtype: L{Task}
        """
        whole_model = PointCache()
        if 'ranges' in task:
            ddiv = task['ddiv'] if 'ddiv' in task else None
            for prange in task['ranges']:
                try:
                    rhor = prange['rho']
                except KeyError:
                    rhor = (0.0, pi)
                try:
                    etar = prange['eta']
                except KeyError:
                    etar = (0.0, 2 * pi)
                part_model = PointCache()
                for point in self._pointrange(prange['x'], prange['y'],
                    prange['z'], task['step'], rhor=rhor, etar=etar, ddiv=ddiv):
                    part_model[point] = 1.0
                whole_model |= part_model
        if 'points' in task:
            part_model = PointCache()
            for point in task['points']:
                if len(point) == 3:
                    point = Point(*point)
                elif len(point) == 5:
                    point = DirectionalPoint(*point)
                part_model[point] = 1.0
            whole_model |= part_model
        params = task['parameters'] if 'parameters' in task else {}
        pose = self._parse_pose(task['pose']) if 'pose' in task  else Pose()
        mount = self.model[task['mount']] if 'mount' in task else None
        return modeltype.yaml['tasks'](params, whole_model, pose=pose,
                                       mount=mount)
