from math import pi
from fuzz import RealRange
from numpy import array, arange
from visual import scene

from coverage import Scene, Camera, MultiCameraSimple, MultiCamera3D, Pose, Point, DirectionalPoint, Angle, rotation_matrix, visual_axes

#from test_voxel3 import *

scene.background = (1, 1, 1)
scene.up = (0, 0, 1)

print "Creating camera model..."
P = [ \
    ('C1', Pose(Point(272.0795, -512.6482, 1111.6311), rotation_matrix((3.7496, 6.2382, 6.2694)))),
    ]
C = [Camera(p[0], 4.4765, 12.4922, 0.00465, 0.00465, 691.1516, 500.7902, 1360, 1024, 1216.1, 20, 3.0, 0.5, 0.036, 1.3, pose = p[1]) for p in P]
print "Creating scene..."
S = Scene(RealRange((-10.0, 420.0)), RealRange((-10.0, 380.0)), RealRange((-10.0, 310.0)), 10.0, pi / 2.0)
# Floor
S.make_opaque(RealRange((-10.0, 420.0)), RealRange((-10.0, 380.0)), RealRange((-10.0, 0.0)))
# A
S.make_opaque(RealRange((60.0, 160.0)), RealRange((160.0, 260.0)), RealRange((0.0, 200.0)))
# B
S.make_opaque(RealRange((0.0, 100.0)), RealRange((0.0, 90.0)), RealRange((0.0, 300.0)))
# C
S.make_opaque(RealRange((210.0, 410.0)), RealRange((210.0, 360.0)), RealRange((0.0, 200.0)))
# D
S.make_opaque(RealRange((140.0, 310.0)), RealRange((30.0, 120.0)), RealRange((0.0, 140.0)))
# E
S.make_opaque(RealRange((60.0, 180.0)), RealRange((290.0, 370.0)), RealRange((0.0, 100.0)))
print "Creating discrete multi-camera model..."
M = MultiCameraSimple( S, C )
print "Updating in-scene model..."
M.update_model()
#print "Visualizing..."
M.visualize( scale = 30.0, color = ( 0.3, 0.3, 0.3 ) )
visual_axes( scale = 30.0, color = ( 0.3, 0.3, 0.3 ) )
#M.visualize( scale = 30.0 )
#visual_axes( scale = 30.0 )

#print "Coverage Performance: %f" % M.performance()

#points of interest
#P = [ Point( -153, -58, -116 ),
#      Point( -183, 72, -116 ),
#      Point( -60, -140, -116 ),
#      Point( 117, -228, -116 ),
#      Point( 190, -25, -116 ),
#      Point( 154, 17, -116 ),
#      Point( 110, 146, -116 ),
#      Point( -110, 255, -116 ) ]
#
#for p in P:
#    p.visualize( scale = 40.0, color = ( 0, 0, 1 ), opacity = M.mu( p ) )
#    print "Coverage of %s: %f" % ( str( p ), M.mu( p ) )

#traverse( Point( -60, -140, -116 ), C[ 0 ].pose.T )
