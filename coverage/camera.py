"""\
Coverage model module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from numpy import arange
from math import sin, atan, pi
from fuzz import IndexedSet, RealRange, TrapezoidalFuzzyNumber, FuzzySet, FuzzyElement

from geometry import Angle, Point, Pose, rotation_matrix


class SpatialDirectionalRange( object ):
    """\
    Spatial-directional range class.
    """
    def __init__( self, x, y, z, pstep, dstep ):
        """\
        Constructor.

        @param x: Range in the x direction.
        @type x: L{fuzz.RealRange}
        @param y: Range in the y direction.
        @type y: L{fuzz.RealRange}
        @param z: Range in the z direction.
        @type z: L{fuzz.RealRange}
        @param pstep: Spatial resolution.
        @type pstep: C{float}
        @param dstep: Angular resolution.
        @type dstep: C{float}
        """
        self.x = RealRange( x )
        self.y = RealRange( y )
        self.z = RealRange( z )
        self.pstep = pstep
        self.dstep = dstep

    def generate_points( self ):
        """\
        Directional point generator.

        @return: The next directional point in the range.
        @rtype: L{geometry.Point}, C{tuple} of L{geometry.Angle}
        """
        for x in arange( self.x[ 0 ], self.x[ 1 ], self.pstep ):
            for y in arange( self.y[ 0 ], self.y[ 1 ], self.pstep ):
                for z in arange( self.z[ 0 ], self.z[ 1 ], self.pstep ):
                    for theta in arange( 0., 2 * pi, self.dstep ):
                        for phi in arange( 0., 2 * pi, self.dstep ):
                            for psi in arange( 0., 2 * pi, self.dstep ):
                                yield Point( x, y, z ),( Angle( theta ),
                                      Angle( phi ), Angle( psi ) )


class Camera( object ):
    """\
    Single-camera model.
    """
    def __init__( self, name, A, f, su, sv, ou, ov, h, w, zS, gamma,
                  r1, r2, cmax, delta, pose = Pose( None, None ) ):
        """\
        Constructor.

        @param name: A name for the camera.
        @type name: C{str}
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
        self.name = name

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

    def __hash__( self ):
        """\
        Hash function (based on name).
        """
        return hash( self.name )

    def __eq__( self, other ):
        """\
        Equality function (based on name).

        @param other: The other operand.
        @type other: L{Camera}
        @return: True if equal, false otherwise.
        @rtype: C{bool}
        """
        if not isinstance( other, Camera ):
            return False
        return self.name == other.name

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
        if not isinstance( point, Point ):
            raise TypeError, ( "invalid point" )
        if not len( direction ) == 3:
            raise ValueError, ( "direction must consist of three angles" )

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


class MultiCamera( IndexedSet ):
    """\
    Abstract base class for multi-camera model.
    """
    def __init__( self, range, cameras = set() ):
        """\
        Constructor.

        @param range: The spatial-directional range.
        @type range: L{SpatialDirectionalRange}
        @param cameras: The initial set of cameras.
        @type cameras: C{set}
        """
        if self.__class__ is MultiCamera:
            raise NotImplementedError, ( "please use one of the subclasses" )
        IndexedSet.__init__( self, 'name', cameras )
        self.inscene = {}
        for camera in self:
            self._update_inscene( camera.name )

    def add( self, item ):
        """\
        Add a camera to the set. Overrides the base class by verifying that
        the item to add is a Camera object, and updating the in-scene set.

        @param item: The item (camera) to add.
        @type item: L{Camera}
        """
        if isinstance( item, Camera ):
            IndexedSet.add( self, item )
        self._update_inscene( item.name )

    def _update_inscene( self, key ):
        """\
        Update an individual in-scene camera fuzzy set.

        @param key: The name of the camera to update.
        @type key: C{str}
        """
        points = []
        for point, direction in self.range.generate_points():
            mu = self[ key ].mu( point, direction )
            if mu > 0:
                points.append( FuzzyElement( ( point, direction ), mu ) )
        self.inscene[ key ] = FuzzySet( points )


class MultiCameraSimple( MultiCamera ):
    """\
    Simple (single-camera coverage) multi-camera model.
    """
    def __init__( self, range, cameras = set() ):
        """\
        Constructor.

        @param range: The spatial-directional range.
        @type range: L{SpatialDirectionalRange}
        @param cameras: The initial set of cameras.
        @type cameras: C{set}
        """
        MultiCamera.__init__( self, range, cameras )

    def mu( point, direction ):
        """\
        TODO
        """
        # this is not very efficient - cache the whole set?
        mu = 0.
        for camera in self:
            try:
                mu = max( mu, 
                        self.inscene[ camera.name ][ ( point, direction ) ].mu )
            except KeyError:
                pass
        return mu
                

class MultiCamera3D( MultiCamera ):
    """\
    3D (dual-camera coverage) multi-camera model.
    """
    def __init__( self, range, cameras = set() ):
        """\
        Constructor.

        @param range: The spatial-directional range.
        @type range: L{SpatialDirectionalRange}
        @param cameras: The initial set of cameras.
        @type cameras: C{set}
        """
        MultiCamera.__init__( self, range, cameras )
