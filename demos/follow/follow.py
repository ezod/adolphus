"""\
Camera selection simulation with a static camera network and a single dynamic
target.

Requires NumPy and SciPy.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import argparse
import os.path
import numpy
from scipy.interpolate import interp1d

try:
    import cPickle as pickle
except ImportError:
    import pickle

from adolphus.geometry import Point, Rotation, Pose
from adolphus.interface import Experiment


def interpolate_points(points):
    """\
    Interpolate the path function (mapping time to L{Point}s) from a set of
    waypoints or knot points of the path at even time intervals.

    @param points: List of path waypoints (one per unit time).
    @type points: C{list} of L{Point}
    @return: Smooth interpolated path function over time.
    @rtype: C{function}
    """
    split = [numpy.array([p[i] for p in points]) for i in range(3)]
    tm = [float(i) for i in range(len(points))]
    f = [interp1d(tm, p, kind='cubic') for p in split]
    return lambda t: Point(*[f[i](t) for i in range(3)])


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--interpolate', dest='interpolate',
        type=int, default=100, help='interpolation pitch')
    parser.add_argument('-t', '--threshold', dest='threshold', type=float,
        default=0.0, help='hysteresis threshold')
    parser.add_argument('pathfile', help='path waypoint file')
    args = parser.parse_args()
    # Load path waypoints.
    print('Loading path waypoints...')
    points = []
    with open(args.pathfile, 'r') as pf:
        for line in pf.readlines():
            points.append(Point(*[float(s) for s in line.rstrip().split(',')]))
    path = interpolate_points(points)
    # Set up displays and load the model.
    print('Loading model...')
    ex = Experiment(zoom=True)
    ex.add_display()
    ex.display.autoscale = True
    ex.display.userspin = False
    ex.display.forward = (0, 0, -1)
    ex.display.up = (0, 1, 0)
    ex.execute('loadmodel follow.yaml')
    ex.execute('loadconfig')
    # Compute the vision graph.
    if os.path.exists('vgraph.pickle'):
        print('Loading vision graph...')
        vision_graph = pickle.load(open('vgraph.pickle', 'r'))
    else:
        try:
            print('Computing vision graph...')
            vision_graph = ex.model.coverage_hypergraph(ex.tasks['scene'], K=2)
            with open('vgraph.pickle', 'w') as vgf:
                pickle.dump(vision_graph, vgf)
        except ImportError:
            vision_graph = None
    # Run demo.
    best = None
    ex.start()
    for t in range(1, args.interpolate * (len(points) - 1)):
        if ex.exit:
            break
        # Set target pose.
        normal = (path(t / float(args.interpolate)) - path((t - 1) \
                 / float(args.interpolate))).unit()
        angle = Point(0, -1, 0).angle(normal)
        axis = Point(0, -1, 0).cross(normal)
        R = Rotation.from_axis_angle(angle, axis)
        ex.model['Person'].set_absolute_pose(\
            Pose(T=path(t / float(args.interpolate)), R=R))
        ex.model['Person'].update_visualization()
        # Compute best view.
        current = best
        best, score = ex.model.best_view(ex.tasks['target'],
            current=(current and frozenset([current]) or None),
            threshold=args.threshold)
        best = set(best).pop()
        if current != best:
            ex.execute('select %s' % best)
            ex.altdisplays[0].camera_view(ex.model[best])
            try:
                ex.execute('guide %s target' % current)
            except:
                pass
            ex.execute('guide %s target' % best)
