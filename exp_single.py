from math import pi
from fuzz import RealRange
from numpy import arange

from coverage import Scene, Camera, MultiCameraSimple, Point, visual_axes

print "Creating camera model..."
C = Camera( 'C', 4.3214, 12.1, 0.00465, 0.00465, 680, 512, 1360, 1024, 784.2, 0, 3.0, 0.5, 0.036, 1.5 )
print "Creating scene..."
S = Scene( RealRange( ( -1000.0, 1000.0 ) ), RealRange( ( -1000.0, 1000.0 ) ), RealRange( ( 100.0, 2000.0 ) ), 100.0, pi / 2.0 )
#for x in arange( -400, 500, 100 ):
#    for y in arange( -400, 500, 100 ):
#        for z in arange( 1100, 1300, 100 ):
#            S.make_opaque( Point( x, y, z ) )
#for p in S.opaque:
#    p.visualize( scale = 50.0 )
print "Creating discrete multi-camera model..."
M = MultiCameraSimple( S, set( [ C ] ) )
print "Updating in-scene model..."
M.update()
print "Visualizing..."
M.visualize( scale = 40.0 )
visual_axes( scale = 40.0 )
