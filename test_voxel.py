import visual
from numpy import arange

from coverage import Point, visual_axes

pstep = 50.0

def voxel( x, y, z ):
    #print x, y, z
    visual.box( pos = ( x * pstep, y * pstep, z * pstep ), length = pstep, height = pstep, width = pstep, color = ( 0, 0, 1 ), opacity = 0.2 )
    #raw_input()

def traverse( p, cam = Point( 0, 0, 0 ) ):
    p.visualize( color = ( 1, 0, 0 ), scale = 5.0 )
    cam.visualize( color = ( 1, 0, 0 ), scale = 5.0 )
    visual.cylinder( axis = ( p - cam ).tuple, pos = cam.tuple, radius = 0.1 )
    # setup
    P1 = Point( round( p.x / pstep ), round( p.y / pstep ), round( p.z / pstep ) )
    P2 = Point( round( cam.x / pstep ), round( cam.y / pstep ), round( cam.z / pstep ) )
    d = [ P2[ i ] - P1[ i ] for i in range( 3 ) ]
    a = [ abs( d[ i ] ) * 2 for i in range( 3 ) ]
    s = [ d[ i ] >= 0 and 1 or -1 for i in range( 3 ) ]
    P = Point( P1.x, P1.y, P1.z )
    if( a[ 0 ] >= max( a[ 1 ], a[ 2 ] ) ):
        print "x driving"
        x, y, z = 0, 1, 2
    elif( a[ 1 ] >= max( a[ 0 ], a[ 2 ] ) ):
        print "y driving"
        x, y, z = 1, 2, 0
    else:
        print "z driving"
        x, y, z = 2, 0, 1
    yd = a[ y ] - a[ x ] / 2
    zd = a[ z ] - a[ x ] / 2
    while True:
        voxel( P.x, P.y, P.z )
        if( P[ x ] == P2[ x ] ):
            break
        if( yd >= 0 ):
            P[ y ] += s[ y ]
            yd -= a[ x ]
        if( zd >= 0 ):
            P[ z ] += s[ z ]
            zd -= a[ x ]
        P[ x ] += s[ x ]
        yd += a[ y ]
        zd += a[ z ]


if __name__ == '__main__':
    visual_axes( 5.0 )
    traverse( Point( 2, -8, -8 ), Point( 10, -4, -4 ) )
    raw_input()
    traverse( Point( 5, 6, 7 ), Point( 9, 0, 9 ) )
    raw_input()
    traverse( Point( 15, -16, 17 ), Point( -9, 10, -9 ) )
    raw_input()
    traverse( Point( 10, 10, -10 ), Point ( 11, 11, 10 ) )
    raw_input()
    traverse( Point( 18, 19, 24 ), Point( -12, -13, -8 ) )
