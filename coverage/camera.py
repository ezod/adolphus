"""\
Single-camera coverage model module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from math import sin, atan, pi
from fuzz import TrapezoidalFuzzyNumber

from geometry import Angle, Point, Pose, rotation_matrix


class Camera( object ):
    """\
    Single-camera model.
    """
    def __init__( self, A, f, su, sv, ou, ov, h, w, zS, gamma,
                  r1, r2, cmax, delta, pose = Pose( None, None ) ):
        """\
        Constructor.

        @param A: Aperture size (intrinsic).
        @type A: C{float}
        @param f: Focal length (intrinsic).
        @type f: C{float}
        @param su: Effective pixel width (intrinsic).
        @type su: C{float}
        @param sv: Effective pixel height (intrinsic).
        @type sv: C{float}
        @param ou: Pixel u-coordinate of principal point (intrinsic).
        @type ou: C{int}
        @param ov: Pixel v-coordinate of principal point (intrinsic).
        @type ov: C{int}
        @param w: Sensor pixel width (intrinsic).
        @type w: C{int}
        @param h: Sensor pixel height (intrinsic).
        @type h: C{int}
        @param zS: Subject distance (intrinsic).
        @type zS: C{float}
        @param gamma: Fuzzification value for visibility (application).
        @type gamma: C{float}
        @param r1: Fully acceptable resolution (application).
        @type r1: C{float}
        @param r2: Acceptable resolution (application).
        @type r2: C{float}
        @param cmax: Maximum acceptable circle of confusion (application).
        @type cmax: C{float}
        @param delta: Fuzzification value for direction (application).
        @type delta: C{float}
        @param pose: Pose of the camera in space.
        @type pose: L{geometry.Pose}
        """
        # fuzzy sets for visibility
        ahl = atan( ( ou * su ) / ( 2. * f ) )
        ahr = atan( ( ( w - ou ) * su ) / ( 2. * f ) )
        self.Cvh = TrapezoidalFuzzyNumber( \
                   ( -sin( ahl ) + gamma, sin( ahr ) - gamma ),
                   ( -sin( ahl ), sin( ahr ) ) )
        avl = atan( ( ov * sv ) / ( 2. * f ) )
        avr = atan( ( ( h - ov ) * sv ) / ( 2. * f ) )
        self.Cvv = TrapezoidalFuzzyNumber( \
                   ( -sin( avl ) + gamma, sin( avr ) - gamma ),
                   ( -sin( avl ), sin( avr ) ) )

        # fuzzy set for resolution
        mr = min( ( w / ( 2. * sin( ( ahl + ahr ) / 2. ) ) ), 
                  ( h / ( 2. * sin( ( avl + avr ) / 2. ) ) ) )
        zr1 = ( 1. / r1 ) * mr
        zr2 = ( 1. / r2 ) * mr
        self.Cr = TrapezoidalFuzzyNumber( ( 0, zr1 ), ( 0, zr2 ) )

        # fuzzy set for focus
        zl = ( A * f * zS ) / ( A * f + min( su, sv ) * ( zS - f ) )
        zr = ( A * f * zS ) / ( A * f - min( su, sv ) * ( zS - f ) )
        zn = ( A * f * zS ) / ( A * f + cmax * ( zS - f ) )
        zf = ( A * f * zS ) / ( A * f - cmax * ( zS - f ) )
        self.Cf = TrapezoidalFuzzyNumber( ( zl, zr ) , ( zn, zf ) )
        
        # fuzzy set for direction
        self.Cd = TrapezoidalFuzzyNumber( \
                  ( Angle( pi / 2. + delta ), Angle( 3 * pi / 2. - delta ) ),
                  ( Angle( pi / 2. ), Angle( 3 * pi / 2. ) ) )

        # pose
        self.pose = pose

    def mu( self, point, direction ):
        """\
        Return the membership degree (coverage) for a directional point.
    
        @param point: The point location.
        @type point: L{geometry.Point}
        @param direction: The direction of the point.
        @type direction: C{tuple} of L{geometry.Angle}
        @return: The membership degree (coverage) of the point.
        @rtype: C{float}
        """
        campoint = ( -self.pose ).map( point )

        # visibility
        mu_v = min( self.Cvh.mu( campoint.x / campoint.z ),
                    self.Cvv.mu( campoint.y / campoint.z ) )

        # resolution
        mu_r = self.Cr.mu( campoint.z )

        # focus
        mu_f = self.Cf.mu( campoint.z )

        # direction
        dirpose = Pose( Point( 0, 0, 0 ), rotation_matrix( direction ) )
        dirangle = ( point - self.pose.T ).normal.angle( dirpose.map_rotate( Point( 0, 0, 1 ) ) )
        mu_d = self.Cd.mu( dirangle )
       
        # return min( mu_v, mu_r, mu_f, mu_d )
        return mu_v * mu_r * mu_f * mu_d
