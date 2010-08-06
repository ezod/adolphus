from math import pi

import common
from adolphus import Scene, Camera, MultiCamera, Pose, Point, Rotation, Experiment

print "Creating discrete multi-camera model..."
P = [Point(x, y, z) for x in [i * 200 for i in range(-6, 7)] for y in [i * 200 for i in range(-6, 7)] for z in [i * 100 for i in range(16)]]

M = MultiCamera(ocular = 1, points = P)
M['A'] = Camera(4.4765, 12.5341, 0.00465, (760.1805, 495.1859), (1360, 1024), 1216.1, 20, 3.0, 0.5, 0.036, 1.3, pose = Pose(Point(100, 0, 0), Rotation(0, pi / 4.0, 0)))

print "Running experiment..."
E = Experiment(M)
E.run()

#S.make_desired(RealRange((-350.0, 350.0)), RealRange((-300.0, 300.0)), RealRange((-100.0, 0.0)), d = None)
#print "Coverage Performance: %f" % M.performance()

#points of interest
#P = [Point(-153, -58, -116),
#     Point(-183, 72, -116),
#     Point(-60, -140, -116),
#     Point(117, -228, -116),
#     Point(190, -25, -116),
#     Point(154, 17, -116),
#     Point(110, 146, -116),
#     Point(-110, 255, -116)]

#for p in P:
#    p.visualize(scale = 40.0, color = (0, 0, 1), opacity = M.mu(p))
#    print "Coverage of %s: %f" % (str(p), M.mu(p))
