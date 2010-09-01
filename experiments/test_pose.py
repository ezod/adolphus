from math import pi

import common
from adolphus import Scene, Camera, MultiCamera, Pose, Point, DirectionalPoint, Plane, Rotation, Experiment

print "Creating camera model..."
# cameras
C = [ \
    #('T1', Pose(Point(083.3794, -332.8404, 465.8211), Rotation(-3.7239, -6.0920, -6.2350))),
	('T2', Pose(Point(026.8646, -368.3093, 603.1179), Rotation(-3.6844, -0.0454, -0.1161))),
    ]

print "Creating scene..."
S = Scene()
# Floor
S.add(Plane(Pose(), x=(-107.9, 107.9), y=(-139.7, 139.7)))

print "Creating points..."
P = []

print "Creating discrete multi-camera model..."
M = MultiCamera(ocular = 1, scene = S, points = P)
for name, pose in C:
    M[name] = Camera(4.4765, 12.4922, 0.00465, (691.1516, 500.7902), (1360, 1024), 1216.1, 20, 3.0, 0.5, 0.036, 0.3, pose = pose)

print "Running experiment..."
E = Experiment(M)
E.run()