from math import pi

from coverage import Point, DirectionalPoint, Pose, Angle, rotation_matrix, visual_axes

visual_axes()

P = Pose( Point( 1, 0, 0 ), rotation_matrix( ( Angle( pi / 4.0 ), 0, 0 ) ) )

L = [ DirectionalPoint( 0, 0, 0, 0, 0 ) ]
for i in range( 7 ):
    L.append( P.map( L[ -1 ] ) )
for Lp in L:
    Lp.visualize( color = ( 1, 0, 0 ) )
