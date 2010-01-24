"""\
Single-camera coverage model module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from math import sqrt, sin, atan
from geometry import Pose


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
        # fuzzy set for visibility
        ahl = atan( ( ou * su ) / ( 2. * f ) )
        ahr = atan( ( ( w - ou ) * su ) / ( 2. * f ) )
        avl = atan( ( ov * sv ) / ( 2. * f ) )
        avr = atan( ( ( h - ov ) * sv ) / ( 2. * f ) )

        # fuzzy set for resolution
        mr = min( ( w / ( 2. * sin( ( ahl + ahr ) / 2. ) ) ), 
                  ( h / ( 2. * sin( ( avl + avr ) / 2. ) ) ) )
        zr1 = ( 1. / r1 ) * mr
        zr2 = ( 1. / r2 ) * mr

        # fuzzy set for focus
        zl = ( A * f * zS ) / ( A * f + min( su, sv ) * ( zS - f ) )
        zr = ( A * f * zS ) / ( A * f - min( su, sv ) * ( zS - f ) )
        zn = ( A * f * zS ) / ( A * f + cmax * ( zS - f ) )
        zf = ( A * f * zS ) / ( A * f - cmax * ( zS - f ) )
        
        # fuzzy set for direction

        # pose
        self.pose = pose
