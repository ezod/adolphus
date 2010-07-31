from math import pi
from fuzz import RealRange
from numpy import array, arange
from visual import scene

import common
from adolphus import Scene, Camera, MultiCamera, Pose, Point, DirectionalPoint, Angle, rotation_matrix, visual_axes, Display

display = Display()
display.up = (0, 0, 1)

print "Creating camera model..."
C = [ \
    ( 'C1', -Pose(Point(-29.108133, 11.620695, 1201.241337), array([[-0.008148, -0.999858, 0.014732], [-0.830673, -0.001434, -0.556758], [0.556701, -0.016774, -0.830544]]))),
    ( 'C2', -Pose(Point(-3.954594, -40.551369, 1123.177412), array([[-0.151428, 0.988033, -0.029328], [0.873322, 0.119831, -0.472176], [-0.463011, -0.097113, -0.881016]]))),
    ( 'C3', -Pose(Point(-75.396607, -35.781230, 1035.206374), array([[0.999188, -0.029862, 0.027037], [-0.018387, -0.935259, -0.353485], [0.035842, 0.352701, -0.935049]]))),
    ( 'C4', -Pose(Point(48.442925, -50.477018, 1070.055651), array([[-0.913580, -0.406285, -0.017426], [-0.367485, 0.843168, -0.392458], [0.174143, -0.352138, -0.919605]]))),
    ( 'C5', -Pose(Point(113.817453, -23.763434, 1196.803207), array([[0.558915, -0.827985, -0.045333], [-0.681610, -0.427598, -0.593774], [0.472252, 0.362769, -0.803354]]))),
    ( 'C6', -Pose(Point(-15.476532, -62.178746, 1177.303550), array([[-0.601332, 0.797152, 0.054306], [0.674750, 0.543048, -0.499811], [-0.427916, -0.263909, -0.864430]]))),
    ]

print "Creating scene..."
S = Scene(RealRange((-400.0, 400.0)), RealRange((-400.0, 400.0)), RealRange((-100.0, 1000.0)), 50.0, pi / 2.0)
S.make_opaque(RealRange((-150.0, 150.0)), RealRange((-100.0, 100.0)), RealRange((-100.0, 0.0)))
S.make_desired(RealRange((-350.0, 350.0)), RealRange((-300.0, 300.0)), RealRange((-100.0, 0.0)), d = None)

print "Creating discrete multi-camera model..."
M = MultiCamera(ocular = 1, scene = S)
for name, pose in C:
    M[name] = Camera(4.4765, 12.5341, 0.00465, (760.1805, 495.1859), (1360, 1024), 1216.1, 20, 3.0, 0.5, 0.036, 1.3, pose = pose)

print "Updating in-scene model..."
M.update_model()
#print "Visualizing..."
M.visualize(scale = 30.0, color = (0.3, 0.3, 0.3))
visual_axes(scale = 30.0, color = (0.3, 0.3, 0.3))
#M.visualize(scale = 30.0)
#visual_axes(scale = 30.0)

print "Coverage Performance: %f" % M.performance()

#points of interest
P = [Point(-153, -58, -116),
     Point(-183, 72, -116),
     Point(-60, -140, -116),
     Point(117, -228, -116),
     Point(190, -25, -116),
     Point(154, 17, -116),
     Point(110, 146, -116),
     Point(-110, 255, -116)]

for p in P:
    p.visualize(scale = 40.0, color = (0, 0, 1), opacity = M.mu(p))
    print "Coverage of %s: %f" % (str(p), M.mu(p))
