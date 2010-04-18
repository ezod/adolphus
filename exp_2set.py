from math import pi
from fuzz import RealRange
from numpy import array, arange
from visual import scene

from coverage import Scene, Camera, MultiCameraSimple, Pose, Point, DirectionalPoint, Angle, rotation_matrix, visual_axes

scene.background = ( 1, 1, 1 )
scene.up = ( 0, -1, 0 )

print "Creating camera model..."
T = Point( -38.956913, 60.127483, 947.731327 )
R = array( [ [ -0.999847, -0.000850, -0.017475 ], [ 0.000802, 0.995543, -0.094301 ], [ 0.017477, -0.094300, -0.995390 ] ] )
C = Camera( 'C', 4.3393, 12.15, 0.00465, 0.00465, 718.82325, 422.9936, 1360, 1024, 784.2, 20, 3.0, 0.5, 0.048, 1.3, pose = -Pose( T, R ) )
print "Creating scene..."
S = Scene( RealRange( ( -400.0, 400.0 ) ), RealRange( ( -400.0, 400.0 ) ), RealRange( ( 0.0, 1000.0 ) ), 50.0, pi / 2.0 )
#S.make_opaque( RealRange( ( -400.0, 400.0 ) ), RealRange( ( -400.0, 400.0 ) ), RealRange( ( -100.0, 0.0 ) ) )
S.make_opaque( RealRange( ( 0.0, 100.0 ) ), RealRange( ( 0.0, 100.0 ) ), RealRange( ( 200.0, 300.0 ) ) )
print "Creating discrete multi-camera model..."
M = MultiCameraSimple( S, set( [ C ] ), directional = False )
print "Updating in-scene model..."
M.update()
print "Visualizing..."
M.visualize( scale = 30.0, color = ( 0.3, 0.3, 0.3 ) )
visual_axes( scale = 30.0, color = ( 0.3, 0.3, 0.3 ) )
#M.visualize( scale = 30.0 )
#visual_axes( scale = 30.0 )

#checkerboard
Point( 7 * 19, 5 * 19, 0 ).visualize( scale = 60.0, color = ( 0, 0, 1 ) )
Point( 7 * 19, -5 * 19, 0 ).visualize( scale = 60.0, color = ( 0, 0, 1 ) )
Point( -7 * 19, 5 * 19, 0 ).visualize( scale = 60.0, color = ( 0, 0, 1 ) )
Point( -7 * 19, -5 * 19, 0 ).visualize( scale = 60.0, color = ( 0, 0, 1 ) )

#fov
Point( 214.6, -206.9, 0 ).visualize( scale = 60.0, color = ( 0, 0, 1 ) )
Point( 214.6, 153.4, 0 ).visualize( scale = 60.0, color = ( 0, 0, 1 ) )
Point( -263.9, -206.9, 0 ).visualize( scale = 60.0, color = ( 0, 0, 1 ) )
Point( -263.9, 153.4, 0 ).visualize( scale = 60.0, color = ( 0, 0, 1 ) )

#points of interest
P = [ DirectionalPoint( 12.5, -8.5, 265.0, 0.0, 0.0 ),
      DirectionalPoint( 12.5, -8.5, 265.0, pi / 4.0, pi ),
      DirectionalPoint( 12.5, -8.5, 265.0, pi / 4.0, 0.0 ),
      DirectionalPoint( 67.0, -8.5, 398.0, 0.0, 0.0 ),
      DirectionalPoint( 67.0, -8.5, 398.0, pi / 4.0, pi ),
      DirectionalPoint( 67.0, -8.5, 398.0, pi / 2.0, pi ),
      DirectionalPoint( -114.0, -8.5, 80.0, 0.0, 0.0 ),
      DirectionalPoint( -114.0, -8.5, 80.0, pi / 4.0, pi ),
      DirectionalPoint( -114.0, -8.5, 80.0, pi / 4.0, 0.0 ) ]

for p in P:
    #p.visualize( scale = 40.0, color = ( 0, 0, 1 ), opacity = C.mu( p ) )
    print C.mu( p )
