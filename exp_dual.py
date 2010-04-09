from math import pi
from fuzz import RealRange

from coverage import Scene, Camera, MultiCameraSimple, Angle, Point, Pose, rotation_matrix

print "Creating camera model..."
C = Camera( 'C', 4.3214, 12.1, 0.00465, 0.00465, 684.7, 535.24, 1024, 1360, 784.2, 0, 3.0, 0.5, 0.036, 1.5, Pose( Point( 0, 300, -1000 ), rotation_matrix( ( Angle( 0.3 ), 0, 0 ) ) ) )
D = Camera( 'D', 4.3214, 12.1, 0.00465, 0.00465, 684.7, 535.24, 1024, 1360, 784.2, 0, 3.0, 0.5, 0.036, 1.5, Pose( Point( 0, 300, 1000 ), rotation_matrix( ( Angle( pi + 0.3 ), 0, Angle( pi ) ) ) ) )
print "Creating scene..."
S = Scene( RealRange( ( -1000.0, 1000.0 ) ), RealRange( ( -1000.0, 1000.0 ) ), RealRange( ( -1000.0, 1000.0 ) ), 100.0, pi / 2.0 )
print "Creating discrete multi-camera model..."
M = MultiCameraSimple( S, set( [ C, D ] ) )
print "Updating in-scene model..."
M.update()
print "Visualizing..."
M.visualize( scale = 40.0 )

Point( 0, 0, 0 ).visualize( scale = 120.0 )
