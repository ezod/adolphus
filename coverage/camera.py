"""\
Coverage model module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from numpy import arange
from math import sin, atan, pi
from itertools import combinations
from fuzz import IndexedSet, RealRange, TrapezoidalFuzzyNumber, FuzzySet, FuzzyElement

from geometry import Angle, Point, Pose, rotation_matrix


class SpatialDirectionalRange( object ):
    """\
    Discrete spatial-directional range class.
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
        Discrete directional point generator.

        @return: The next directional point in the range.
        @rtype: L{geometry.DirectionalPoint}
        """
        for x in arange( self.x[ 0 ], self.x[ 1 ], self.pstep ):
            for y in arange( self.y[ 0 ], self.y[ 1 ], self.pstep ):
                for z in arange( self.z[ 0 ], self.z[ 1 ], self.pstep ):
                    for rho in arange( 0., pi + self.dstep, self.dstep ):
                        if rho in [ 0, pi ]:
                            yield DirectionalPoint( x, y, z, rho, 0 )
                            continue
                        for eta in arange( 0., 2 * pi, self.dstep ):
                            yield DirectionalPoint( x, y, z, rho, eta )


class Camera( object ):
    """\
    Single-camera model, using continous fuzzy sets.
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
        ahl = 2. * atan( ( ou * su ) / ( 2. * f ) )
        ahr = 2. * atan( ( ( w - ou ) * su ) / ( 2. * f ) )
        self.Cvh = TrapezoidalFuzzyNumber( \
                   ( -sin( ahl ) + gamma, sin( ahr ) - gamma ),
                   ( -sin( ahl ), sin( ahr ) ) )
        avl = 2. * atan( ( ov * sv ) / ( 2. * f ) )
        avr = 2. * atan( ( ( h - ov ) * sv ) / ( 2. * f ) )
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

    def mu( self, dpoint ):
        """\
        Return the membership degree (coverage) for a directional point.
    
        @param dpoint: The point location.
        @type dpoint: L{geometry.DirectionalPoint}
        @return: The membership degree (coverage) of the point.
        @rtype: C{float}
        """
        if not isinstance( dpoint, DirectionalPoint ):
            raise TypeError, ( "invalid directional point" )

        campoint = ( -self.pose ).map( dpoint )

        # visibility
        mu_v = min( self.Cvh.mu( campoint.x / campoint.z ),
                    self.Cvv.mu( campoint.y / campoint.z ) )

        # resolution
        mu_r = self.Cr.mu( campoint.z )

        # focus
        mu_f = self.Cf.mu( campoint.z )

        # direction
        r = sqrt( dpoint.x ** 2 + dpoint.y ** 2 )
        try:
            terma = ( dpoint.y / r ) * sin( dpoint.eta ) + \
                    ( dpoint.x / r ) * cos( dpoint.eta )
        except ZeroDivisionError:
            terma = 1.0
        try:
            termb = atan( r / dpoint.z )
        except ZeroDivisionError:
            termb = pi / 2.0
        mu_d = min( max( ( float( dpoint.rho ) - ( ( pi / 2.0 ) + \
                         terma * termb ) ) / zeta, 0.0 ), 1.0 )

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
        dpoints = []
        for dpoint in self.range.generate_points():
            mu = self[ key ].mu( dpoint )
            if mu > 0:
                dpoints.append( FuzzyElement( dpoint, mu ) )
        self.inscene[ key ] = FuzzySet( dpoints )


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

    def mu( point, direction ):
        """\
        TODO
        """
        pairs = set()
        for pair in combinations( self, 2 ):
            # intersection of pair -> set of intersections
            # need inscene for it? ugh
        # for intersection in set of intersections:
            # mu = max( mu, this.mu )
            pass
