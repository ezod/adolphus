from coverage import Point, visual_axes

def traverse( pstep, p, cam = Point( 0, 0, 0 ) ):
    # setup
    d = [ abs( p[ i ] - cam[ i ] ) for i in range( 3 ) ]
    u = [ cam[ i ] - int( cam[ i ] / pstep ) * pstep \
          for i in range( 3 ) ]
    s = [ 1.0, 1.0, 1.0 ]
    for i in range( 3 ):
        try:
            s[ i ] = ( p[ i ] - cam[ i ] ) / ( d[ i ] )
        except ZeroDivisionError:
            s = 1.0

    # driving axis
    if d[ 0 ] > d[ 1 ] and d[ 0 ] > d[ 2 ]:
        x, y, z = 0, 1, 2
    elif d[ 1 ] > d[ 0 ] and d[ 1 ] > d[ 2 ]:
        x, y, z = 1, 2, 0
    else:
        x, y, z = 2, 0, 1

    # voxel traversal
    P = [ int( cam[ i ] / pstep ) for i in range( 3 ) ]
    Point( P[ 0 ], P[ 1 ], P[ 2 ] ).visualize()
    P2 = int( p[ x ] / pstep )
    exy = ( u[ y ] - pstep ) * d[ x ] + \
          ( pstep - u[ x ] ) * d[ y ]
    exz = ( u[ z ] - pstep ) * d[ x ] + \
          ( pstep - u[ x ] ) * d[ z ]
    dxy = pstep * d[ y ]
    d1xy = dxy - pstep * d[ x ]
    dxz = pstep * d[ z ]
    d1xz = dxz - pstep * d[ x ]
    while P[ x ] < P2:
        if exy > 0:
            if exz > 0:
                if exy * d[ z ] > exz * d[ y ]:
                    Point( P[ 0 ] + ( y == 0 and s[ 0 ] or 0 ),
                           P[ 1 ] + ( y == 1 and s[ 1 ] or 0 ),
                           P[ 2 ] + ( y == 2 and s[ 2 ] or 0 ) ).visualize()
                else:
                    Point( P[ 0 ] + ( z == 0 and s[ 0 ] or 0 ),
                           P[ 1 ] + ( z == 1 and s[ 1 ] or 0 ),
                           P[ 2 ] + ( z == 2 and s[ 2 ] or 0 ) ).visualize()
                Point( P[ 0 ] + ( y == 0 and s[ 0 ] or 0 ) \
                              + ( z == 0 and s[ 0 ] or 0 ),
                       P[ 1 ] + ( y == 1 and s[ 0 ] or 0 ) \
                              + ( z == 1 and s[ 0 ] or 0 ),
                       P[ 2 ] + ( y == 2 and s[ 0 ] or 0 ) \
                              + ( z == 2 and s[ 0 ] or 0 ) ).visualize()
                P = [ P[ i ] + s[ i ] for i in range( 3 ) ]
                Point( P[ 0 ], P[ 1 ], P[ 2 ] ).visualize()
                exy += d1xy
                exz += d1xz
            else:
                Point( P[ 0 ] + ( y == 0 and s[ 0 ] or 0 ),
                       P[ 1 ] + ( y == 1 and s[ 1 ] or 0 ),
                       P[ 2 ] + ( y == 2 and s[ 2 ] or 0 ) ).visualize()
                Point( P[ 0 ] + ( x == 0 and s[ 0 ] or 0 ) \
                              + ( y == 0 and s[ 0 ] or 0 ),
                       P[ 1 ] + ( x == 1 and s[ 0 ] or 0 ) \
                              + ( y == 1 and s[ 0 ] or 0 ),
                       P[ 2 ] + ( x == 2 and s[ 0 ] or 0 ) \
                              + ( y == 2 and s[ 0 ] or 0 ) ).visualize()
                P[ x ] += s[ x ]
                P[ y ] += s[ y ]
                exy = exy + d1xy
                exz = exz + dxz
        elif exz > 0:
            Point( P[ 0 ] + ( z == 0 and s[ 0 ] or 0 ),
                   P[ 1 ] + ( z == 1 and s[ 1 ] or 0 ),
                   P[ 2 ] + ( z == 2 and s[ 2 ] or 0 ) ).visualize()
            Point( P[ 0 ] + ( x == 0 and s[ 0 ] or 0 ) \
                          + ( z == 0 and s[ 0 ] or 0 ),
                   P[ 1 ] + ( x == 1 and s[ 0 ] or 0 ) \
                          + ( z == 1 and s[ 0 ] or 0 ),
                   P[ 2 ] + ( x == 2 and s[ 0 ] or 0 ) \
                          + ( z == 2 and s[ 0 ] or 0 ) ).visualize()
            P[ x ] += s[ x ]
            P[ z ] += s[ z ]
            exy += dxy
            exz += d1xz
        else:
            Point( P[ 0 ] + ( x == 0 and s[ 0 ] or 0 ),
                   P[ 1 ] + ( x == 1 and s[ 1 ] or 0 ),
                   P[ 2 ] + ( x == 2 and s[ 2 ] or 0 ) ).visualize()
            P[ x ] += s[ x ]
            exy = exy + dxy
            exz = exz + dxz


if __name__ == '__main__':
    visual_axes( 5.0 )
    for p in [ ( 5, 6, 7 ), ( -5, -6, 7 ), ( 6, -1, 3 ), ( 2, 7, 1 ) ]:
        Point( p[ 0 ], p[ 1 ], p[ 2 ] ).visualize( color = ( 1, 0, 0 ), scale = 5.0 )
        traverse( 1.0, p )
