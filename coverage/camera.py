"""\
Coverage model module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from numpy import arange
from math import sqrt, sin, cos, atan, pi
from itertools import combinations
from fuzz import IndexedSet, RealRange, TrapezoidalFuzzyNumber, FuzzySet, \
                 FuzzyElement

from geometry import Angle, Point, DirectionalPoint, Pose, rotation_matrix

try:
    import visual
    VIS = True
except ImportError:
    VIS = False


class Scene( object ):
    """\
    Discrete spatial-directional range with occlusion class.
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
        self.opaque = set()

    def make_opaque( self, p ):
        """\
        Add a point to

        @param p: Discrete point to make opaque.
        @type p: L{geometry.Point}
        """
        if p.x % self.pstep or p.y % self.pstep or p.z % self.pstep:
            raise ValueError( "discrete point must fall on the grid" )
        self.opaque.add( p )

    def generate_points( self ):
        """\
        Discrete directional point generator.

        @return: The next directional point in the range.
        @rtype: L{geometry.DirectionalPoint}
        """
        for x in arange( self.x[ 0 ], self.x[ 1 ], self.pstep ):
            for y in arange( self.y[ 0 ], self.y[ 1 ], self.pstep ):
                for z in arange( self.z[ 0 ], self.z[ 1 ], self.pstep ):
                    if Point( x, y, z ) in self.opaque:
                        continue
                    for rho in arange( 0., pi + self.dstep, self.dstep ):
                        if rho in [ 0, pi ]:
                            yield DirectionalPoint( x, y, z, rho, 0 )
                            continue
                        for eta in arange( 0., 2 * pi, self.dstep ):
                            yield DirectionalPoint( x, y, z, rho, eta )

    def occluded( self, p, cam = Point( 0, 0, 0 ) ):
        """\
        Find the 6-connectivity voxel traversal of the line segment between the
        specified point and the principal point of a camera, and return whether
        the point is occluded from the camera viewpoint by opaque voxels.

        @param p: The point to test.
        @type p: L{geometry.Point}
        @param cam: The camera principal point (translation component of pose). 
        @type cam: L{geometry.Point}
        @return: True if occluded, false otherwise.
        @rtype: C{bool}
        """
        # setup
        d = [ abs( p[ i ] - cam[ i ] ) for i in range( 3 ) ]
        u = [ cam[ i ] - int( cam[ i ] / self.pstep ) * self.pstep \
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
        P = [ int( cam[ i ] / self.pstep ) for i in range( 3 ) ]
        if Point( P[ 0 ], P[ 1 ], P[ 2 ] ) in self.opaque:
            return True
        P2 = int( p[ x ] / self.pstep )
        exy = ( u[ y ] - self.pstep ) * d[ x ] + \
              ( self.pstep - u[ x ] ) * d[ y ]
        exz = ( u[ z ] - self.pstep ) * d[ x ] + \
              ( self.pstep - u[ x ] ) * d[ z ]
        dxy = self.pstep * d[ y ]
        d1xy = dxy - self.pstep * d[ x ]
        dxz = self.pstep * d[ z ]
        d1xz = dxz - self.pstep * d[ x ]
        while P[ x ] < P2:
            if exy > 0:
                if exz > 0:
                    if exy * d[ z ] > exz * d[ y ]:
                        if Point( P[ 0 ] + ( y == 0 and s[ 0 ] or 0 ),
                                  P[ 1 ] + ( y == 1 and s[ 1 ] or 0 ),
                                  P[ 2 ] + ( y == 2 and s[ 2 ] or 0 ) ) \
                        in self.opaque:
                            return True
                    else:
                        if Point( P[ 0 ] + ( z == 0 and s[ 0 ] or 0 ),
                                  P[ 1 ] + ( z == 1 and s[ 1 ] or 0 ),
                                  P[ 2 ] + ( z == 2 and s[ 2 ] or 0 ) ) \
                        in self.opaque:
                            return True
                    if Point( P[ 0 ] + ( y == 0 and s[ 0 ] or 0 ) \
                                     + ( z == 0 and s[ 0 ] or 0 ),
                              P[ 1 ] + ( y == 1 and s[ 0 ] or 0 ) \
                                     + ( z == 1 and s[ 0 ] or 0 ),
                              P[ 2 ] + ( y == 2 and s[ 0 ] or 0 ) \
                                     + ( z == 2 and s[ 0 ] or 0 ) ) \
                    in self.opaque:
                        return True
                    P = [ P[ i ] + s[ i ] for i in range( 3 ) ]
                    if Point( P[ 0 ], P[ 1 ], P[ 2 ] ) in self.opaque:
                        return True
                    exy += d1xy
                    exz += d1xz
                else:
                    if Point( P[ 0 ] + ( y == 0 and s[ 0 ] or 0 ),
                              P[ 1 ] + ( y == 1 and s[ 1 ] or 0 ),
                              P[ 2 ] + ( y == 2 and s[ 2 ] or 0 ) ) \
                    in self.opaque:
                        return True
                    if Point( P[ 0 ] + ( x == 0 and s[ 0 ] or 0 ) \
                                     + ( y == 0 and s[ 0 ] or 0 ),
                              P[ 1 ] + ( x == 1 and s[ 0 ] or 0 ) \
                                     + ( y == 1 and s[ 0 ] or 0 ),
                              P[ 2 ] + ( x == 2 and s[ 0 ] or 0 ) \
                                     + ( y == 2 and s[ 0 ] or 0 ) ) \
                    in self.opaque:
                        return True
                    P[ x ] += s[ x ]
                    P[ y ] += s[ y ]
                    exy = exy + d1xy
                    exz = exz + dxz
            elif exz > 0:
                if Point( P[ 0 ] + ( z == 0 and s[ 0 ] or 0 ),
                          P[ 1 ] + ( z == 1 and s[ 1 ] or 0 ),
                          P[ 2 ] + ( z == 2 and s[ 2 ] or 0 ) ) \
                in self.opaque:
                    return True
                if Point( P[ 0 ] + ( x == 0 and s[ 0 ] or 0 ) \
                                 + ( z == 0 and s[ 0 ] or 0 ),
                          P[ 1 ] + ( x == 1 and s[ 0 ] or 0 ) \
                                 + ( z == 1 and s[ 0 ] or 0 ),
                          P[ 2 ] + ( x == 2 and s[ 0 ] or 0 ) \
                                 + ( z == 2 and s[ 0 ] or 0 ) ) \
                in self.opaque:
                    return True
                P[ x ] += s[ x ]
                P[ z ] += s[ z ]
                exy += dxy
                exz += d1xz
            else:
                if Point( P[ 0 ] + ( x == 0 and s[ 0 ] or 0 ),
                          P[ 1 ] + ( x == 1 and s[ 1 ] or 0 ),
                          P[ 2 ] + ( x == 2 and s[ 2 ] or 0 ) ) \
                in self.opaque:
                    return True
                P[ x ] += s[ x ]
                exy = exy + dxy
                exz = exz + dxz
        return False


class Camera( object ):
    """\
    Single-camera model, using continous fuzzy sets.
    """
    def __init__( self, name, A, f, su, sv, ou, ov, h, w, zS, gamma,
                  r1, r2, cmax, zeta, pose = Pose( None, None ) ):
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
        @param zeta: Fuzzification value for direction (application).
        @type zeta: C{float}
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

        # fuzzifier for direction
        self.zeta = zeta
        
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
                         terma * termb ) ) / self.zeta, 0.0 ), 1.0 )

        # return min( mu_v, mu_r, mu_f, mu_d )
        return mu_v * mu_r * mu_f * mu_d

    def visualize( self, color = ( 1, 1, 1 ) ):
        """\
        Plot the camera in a 3D visual model.

        @param color: The color in which to plot the point.
        @type color: C{tuple}
        """
        if not VIS:
            raise NotImplementedError( "visual module not loaded" )
        visual.pyramid( pos = self.pose.T.tuple, axis = \
            self.pose.map_rotate( Point( 0, 0, -50.0 ) ).tuple, \
            size = ( 50.0, 50.0, 50.0 ), color = color )


class MultiCamera( IndexedSet ):
    """\
    Abstract base class for multi-camera model.
    """
    def __init__( self, scene, cameras = set() ):
        """\
        Constructor.

        @param scene: The discrete scene model.
        @type scene: L{Scene}
        @param cameras: The initial set of cameras.
        @type cameras: C{set}
        """
        if self.__class__ is MultiCamera:
            raise NotImplementedError, ( "please use one of the subclasses" )
        IndexedSet.__init__( self, 'name', cameras )
        self.model = FuzzySet()
        self.scene = scene
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
        Update an individual in-scene camera fuzzy set (with occlusion).

        @param key: The name of the camera to update.
        @type key: C{str}
        """
        dpoints = []
        for dpoint in self.scene.generate_points():
            mu = self[ key ].mu( dpoint )
            if mu > 0 and not self.scene.occluded( dpoint, self[ key ].pose.T ):
                dpoints.append( FuzzyElement( dpoint, mu ) )
        self.inscene[ key ] = FuzzySet( dpoints )

    def visualize( self ):
        """\
        Visualize all cameras and the directional points of the coverage model
        (with opacity reflecting degree of coverage).
        """
        if not VIS:
            raise NotImplementedError( "visual module not loaded" )
        for camera in self:
            camera.visualize()
        for dpoint in self.scene.generate_points():
            if dpoint in self.model:
                dpoint.visualize( color = ( 1, 0, 0 ),
                                  opacity = self.model[ dpoint ].mu )


class MultiCameraSimple( MultiCamera ):
    """\
    Simple (single-camera coverage) multi-camera model.
    """
    def __init__( self, scene, cameras = set() ):
        """\
        Constructor.

        @param scene: The discrete scene model.
        @type scene: L{Scene}
        @param cameras: The initial set of cameras.
        @type cameras: C{set}
        """
        MultiCamera.__init__( self, scene, cameras )

    def _update( self ):
        """\
        Update the simple multi-camera network discrete spatial-directional
        fuzzy set (coverage model).
        """
        self.model = FuzzySet( [ FuzzyElement( dpoint, mu = 0.0 ) \
                     for dpoint in self.scene.generate_points() ] )
        for camera in self:
            self.model |= self.inscene[ camera.name ]


class MultiCamera3D( MultiCamera ):
    """\
    3D (dual-camera coverage) multi-camera model.
    """
    def __init__( self, scene, cameras = set() ):
        """\
        Constructor.

        @param scene: The discrete scene model.
        @type scene: L{Scene}
        @param cameras: The initial set of cameras.
        @type cameras: C{set}
        """
        MultiCamera.__init__( self, scene, cameras )

    def _update( self ):
        """\
        Update the 3D multi-camera network discrete spatial-directional fuzzy
        set (coverage model).
        """
        self.model = FuzzySet( [ FuzzyElement( dpoint, mu = 0.0 ) \
                     for dpoint in self.scene.generate_points() ] )
        pairs = set( [ self.inscene[ pair[ 0 ].name ] & \
                       self.inscene[ pair[ 1 ].name ] \
                       for pair in combinations( self, 2 ) ] )
        for pair in pairs:
            self.model |= pair
