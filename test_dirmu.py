from coverage import Point, DirectionalPoint, Angle, Pose, rotation_matrix, visual_axes
from visual import color, arrow, cylinder
from math import pi, sqrt, sin, cos, atan
from numpy import arange


def generate_points( p, dstep ):
    for rho in arange( 0., pi + dstep, dstep ):
        if rho in [ 0, pi ]:
            yield DirectionalPoint( p.x, p.y, p.z, rho, 0 )
            continue
        for eta in arange( 0., 2 * pi, dstep ):
            yield DirectionalPoint( p.x, p.y, p.z, rho, eta )


def dir_mu( p, zeta ):
    r = sqrt( p.x ** 2 + p.y ** 2 )
    try:
        terma = ( p.y / r ) * sin( p.eta ) + ( p.x / r ) * cos( p.eta )
    except ZeroDivisionError:
        terma = 1.0
    try:
        termb = atan( r / p.z )
    except ZeroDivisionError:
        termb = pi / 2.0
    return min( max( ( float( p.rho ) - ( ( pi / 2.0 ) + terma * termb ) ) / zeta, 0.0 ), 1.0 )


if __name__ == "__main__":
    visual_axes( 1 )
    for x in arange( -5, 10, 5 ):
        for y in arange( -5, 10, 5 ):
            for z in arange( 5, 15, 5 ):
                for p in generate_points( Point( x, y, z ), pi / 2.0 ):
                    p.visualize( color = color.red, opacity = dir_mu( p, 1.5 ) )
