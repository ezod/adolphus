from math import pi
from fuzz import RealRange
from numpy import array, arange
from visual import scene

from coverage import Scene, Camera, MultiCameraSimple, Pose, Point, DirectionalPoint, Angle, rotation_matrix, visual_axes

#scene.background = ( 1, 1, 1 )
scene.up = ( 0, 0, 1 )

print "Creating camera model..."
P = [ ( 'C1', -Pose( Point( -29.108133, 11.620695, 1201.241337 ), array( [ [ -0.008148, -0.999858, 0.014732 ], [ -0.830673, -0.001434, -0.556758 ], [ 0.556701, -0.016774, -0.830544 ] ] ) ) ),
      ( 'C2', -Pose( Point( -3.954594, -40.551369, 1123.177412 ), array( [ [ -0.151428, 0.988033, -0.029328 ], [ 0.873322, 0.119831, -0.472176 ], [ -0.463011, -0.097113, -0.881016 ] ] ) ) ),
      ( 'C3', -Pose( Point( -75.396607, -35.781230, 1035.206374 ), array( [ [ 0.999188, -0.029862, 0.027037 ], [ -0.018387, -0.935259, -0.353485 ], [ 0.035842, 0.352701, -0.935049 ] ] ) ) ),
      ( 'C4', -Pose( Point( 48.442925, -50.477018, 1070.055651 ), array( [ [ -0.913580, -0.406285, -0.017426 ], [ -0.367485, 0.843168, -0.392458 ], [ 0.174143, -0.352138, -0.919605 ] ] ) ) ),
      ( 'C5', -Pose( Point( 113.817453, -23.763434, 1196.803207 ), array( [ [ 0.558915, -0.827985, -0.045333 ], [ -0.681610, -0.427598, -0.593774 ], [ 0.472252, 0.362769, -0.803354 ] ] ) ) ),
      ( 'C6', -Pose( Point( -15.476532, -62.178746, 1177.303550 ), array( [ [ -0.601332, 0.797152, 0.054306 ], [ 0.674750, 0.543048, -0.499811 ], [ -0.427916, -0.263909, -0.864430 ] ] ) ) ) ]
C = [ Camera( p[ 0 ], 4.4765, 12.5341, 0.00465, 0.00465, 760.1805, 495.1859, 1360, 1024, 1216.1, 20, 3.0, 0.5, 0.036, 1.3, pose = p[ 1 ] ) for p in P ]
print "Creating scene..."
S = Scene( RealRange( ( -400.0, 400.0 ) ), RealRange( ( -400.0, 400.0 ) ), RealRange( ( -200.0, 1000.0 ) ), 50.0, pi / 2.0 )
S.make_opaque( RealRange( ( -137.0, 131.0 ) ), RealRange( ( -103.0, 100.0 ) ), RealRange( ( -116.0, 0.0 ) ) )
print "Creating discrete multi-camera model..."
M = MultiCameraSimple( S, C )
print "    %d cameras in model." % len( M )
print "Updating in-scene model..."
M.update()
print "Visualizing..."
#M.visualize( scale = 30.0, color = ( 0.3, 0.3, 0.3 ) )
#visual_axes( scale = 30.0, color = ( 0.3, 0.3, 0.3 ) )
M.visualize( scale = 30.0 )
visual_axes( scale = 30.0 )

#points of interest
P = [ DirectionalPoint( -153, -58, -90, 0.0, 0.0 ),
      DirectionalPoint( -183, 72, -90, 0.0, 0.0 ),
      DirectionalPoint( -60, -140, -90, 0.0, 0.0 ),
      DirectionalPoint( 117, -228, -90, 0.0, 0.0 ),
      DirectionalPoint( 190, -25, -90, 0.0, 0.0 ),
      DirectionalPoint( 154, 17, -90, 0.0, 0.0 ),
      DirectionalPoint( 110, 146, -90, 0.0, 0.0 ),
      DirectionalPoint( -110, 255, -90, 0.0, 0.0 ) ]

for p in P:
    p.visualize( scale = 40.0, color = ( 0, 0, 1 ), opacity = M.mu( p ) )
    print M.mu( p )
