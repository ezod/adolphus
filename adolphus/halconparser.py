"""\
HALCON parameter parser module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import pyximport; pyximport.install()

from math import pi

from .geometry import Pose, Point, Rotation


def parse_internal(filename):
    """\
    Parse internal calibration parameters from a HALCON internal calibration
    output file.

    @param filename: The calibration output file to load from.
    @type filename: C{str}
    @return: Tuple containing the internal parameters f, s, o, and dim.
    @rtype: C{tuple}
    """
    f, s, o, dim = None, [None, None], [None, None], [None, None]
    with open(filename, 'r') as icfile:
        for line in icfile:
            line = line.rstrip()
            if not line or line.startswith('#') or line.startswith('\t') \
            or line.startswith('ParGroup'):
                continue
            value = float(line.split(':')[2][1:-1])
            if line.startswith('Focus'):
                f = 1e3 * value
            elif line.startswith('Sx'):
                s[0] = 1e3 * value
            elif line.startswith('Sy'):
                s[1] = 1e3 * value
            elif line.startswith('Cx'):
                o[0] = value
            elif line.startswith('Cy'):
                o[1] = value
            elif line.startswith('ImageWidth'):
                dim[0] = int(value)
            elif line.startswith('ImageHeight'):
                dim[1] = int(value)
    return f, s, o, dim


def parse_external(filename):
    """\
    Parse external calibration parameters from a HALCON external calibration
    output file.
    
    @param filename: The calibration output file to load from.
    @type filename: C{str}
    """
    with open(filename, 'r') as ecfile:
        for line in ecfile:
            line = line.rstrip()
            if line.startswith('t'):
                T = Point([1e3 * float(value) for value in line.split(' ')[1:]])
            elif line.startswith('r'):
                R = Rotation.from_euler('zyx', [(360. - float(value)) * pi \
                    / 180.0 for value in line.split(' ')[1:]])
    return Pose(T, R)
