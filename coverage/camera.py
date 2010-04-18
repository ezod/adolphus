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

from coverage.geometry import Point, DirectionalPoint, Pose

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
        self.D = FuzzySet()

    def make_opaque( self, x, y, z ):
        """\
        Add a range to the set of opaque voxels.

        @param x: Range in the x direction.
        @type x: L{fuzz.RealRange}
        @param y: Range in the y direction.
        @type y: L{fuzz.RealRange}
        @param z: Range in the z direction.
        @type z: L{fuzz.RealRange}
        """
        for xi in arange( x[ 0 ], x[ 1 ], self.pstep ):
            for yi in arange( y[ 0 ], y[ 1 ], self.pstep ):
                for zi in arange( z[ 0 ], z[ 1 ], self.pstep ):
                    self.opaque.add( Point( xi, yi, zi ) )
                    try:
                        self.D.remove( Point( xi, yi, zi ) )
                    except KeyError:
                        pass
                    for rho in arange( 0., pi + self.dstep, self.dstep ):
                        for eta in arange( 0., 2 * pi, self.dstep ):
                            try:
                                self.D.remove( DirectionalPoint( \
                                               xi, yi, zi, rho, eta ) )
                            except KeyError:
                                pass

    def make_desired( self, x, y, z, d = 'all', mu = 1.0 ):
        """\
        Add a range to the desired coverage set.

        @param x: Range in the x direction.
        @type x: L{fuzz.RealRange}
        @param y: Range in the y direction.
        @type y: L{fuzz.RealRange}
        @param z: Range in the z direction.
        @type z: L{fuzz.RealRange}
        @param d: Optional direction (will use all dstep otherwise).
        @type d: C{tuple} of L{geometry.Angle}
        @param mu: Membership value to assign to this range.
        @type mu: C{float}
        """
        for dpoint in self._generate_points_specified( x, y, z, d ):
            if not Point( dpoint.x, dpoint.y, dpoint.z ) in self.opaque:
                self.D.add( FuzzyElement( dpoint, mu ) )

    def generate_points( self, d = 'all' ):
        """\
        Discrete directional point generator for the entire scene.

        @return: The next directional point in the range.
        @rtype: L{geometry.DirectionalPoint}
        @param d: Optional direction (will use all dstep otherwise).
        @type d: C{tuple} of L{geometry.Angle}
        """
        return self._generate_points_specified( self.x, self.y, self.z, d = d )

    def _generate_points_specified( self, x, y, z, d = 'all' ):
        """\
        General discrete directional point generator.

        @param x: Range in the x direction.
        @type x: L{fuzz.RealRange}
        @param y: Range in the y direction.
        @type y: L{fuzz.RealRange}
        @param z: Range in the z direction.
        @type z: L{fuzz.RealRange}
        @param d: Optional direction (will use all dstep otherwise).
        @type d: C{tuple} of L{geometry.Angle}
        @return: The next directional point in the range.
        @rtype: L{geometry.DirectionalPoint}
        """
        for xi in arange( x[ 0 ], x[ 1 ], self.pstep ):
            for yi in arange( y[ 0 ], y[ 1 ], self.pstep ):
                for zi in arange( z[ 0 ], z[ 1 ], self.pstep ):
                    if Point( xi, yi, zi ) in self.opaque:
                        continue
                    if not d:
                        yield Point( xi, yi, zi )
                        continue
                    if d != 'all':
                        yield DirectionalPoint( xi, yi, zi, d[ 0 ], d[ 1 ] )
                        continue
                    for rho in arange( 0., pi + self.dstep, self.dstep ):
                        if rho in [ 0, pi ]:
                            yield DirectionalPoint( xi, yi, zi, rho, 0 )
                            continue
                        for eta in arange( 0., 2 * pi, self.dstep ):
                            yield DirectionalPoint( xi, yi, zi, rho, eta )

    def occluded( self, p, cam = Point( 0, 0, 0 ) ):
        """\
        Find the 3D Bresenham voxel traversal of the line segment between the
        specified point and the principal point of a camera, and return whether
        the point is occluded from the camera viewpoint by opaque voxels.

        @param p: The point to test.
        @type p: L{geometry.Point}
        @param cam: The camera principal point (translation component of pose). 
        @type cam: L{geometry.Point}
        @return: True if occluded, false otherwise.
        @rtype: C{bool}
        """
        P1 = Point( round( p.x / self.pstep ), \
                    round( p.y / self.pstep ), \
                    round( p.z / self.pstep ) )
        P2 = Point( round( cam.x / self.pstep ), \
                    round( cam.y / self.pstep ), \
                    round( cam.z / self.pstep ) )
        d = [ P2[ i ] - P1[ i ] for i in range( 3 ) ]
        a = [ abs( d[ i ] ) * 2 for i in range( 3 ) ]
        s = [ d[ i ] >= 0 and 1 or -1 for i in range( 3 ) ]
        P = Point( P1.x, P1.y, P1.z )
        if( a[ 0 ] >= max( a[ 1 ], a[ 2 ] ) ):
            x, y, z = 0, 1, 2
        elif( a[ 1 ] >= max( a[ 0 ], a[ 2 ] ) ):
            x, y, z = 1, 2, 0
        else:
            x, y, z = 2, 0, 1
        yd = a[ y ] - a[ x ] / 2
        zd = a[ z ] - a[ x ] / 2
        while True:
            if self.voxel( P.x, P.y, P.z ) in self.opaque:
                return True
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
        return False
 
    def voxel( self, x, y, z ):
        """\
        TODO
        """
        return Point( x * self.pstep, y * self.pstep, z * self.pstep )

    def visualize( self, color = ( 1, 1, 1 ) ):
        """\
        Visualize the opaque scene objects.

        @param color: The color of opaque scene objects.
        @type color: C{tuple}
        """
        if not VIS:
            raise NotImplementedError( "visual module not loaded" )
        for point in self.opaque:
            visual.box( pos = point.tuple, color = color, length = self.pstep,
                        height = self.pstep, width = self.pstep )


class Camera( object ):
    """\
    Single-camera model, using continous fuzzy sets.
    """
    def __init__( self, name, A, f, su, sv, ou, ov, w, h, zS, gamma,
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
        @type ou: C{float}
        @param ov: Pixel v-coordinate of principal point (intrinsic).
        @type ov: C{float}
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
        avl = 2. * atan( ( ov * sv ) / ( 2. * f ) )
        avr = 2. * atan( ( ( h - ov ) * sv ) / ( 2. * f ) )
        gamma_h = ( float( gamma ) / float( w ) ) * 2.0 \
                  * sin( ( ahl + ahr ) / 2. )
        gamma_v = ( float( gamma ) / float( h ) ) * 2.0 \
                  * sin( ( avl + avr ) / 2. )
        self.Cvh = TrapezoidalFuzzyNumber( \
                   ( -sin( ahl ) + gamma_h, sin( ahr ) - gamma_h ),
                   ( -sin( ahl ), sin( ahr ) ) )
        self.Cvv = TrapezoidalFuzzyNumber( \
                   ( -sin( avl ) + gamma_v, sin( avr ) - gamma_v ),
                   ( -sin( avl ), sin( avr ) ) )

        # fuzzy set for resolution
        mr = min( ( float( w ) / ( 2. * sin( ( ahl + ahr ) / 2. ) ) ), 
                  ( float( h ) / ( 2. * sin( ( avl + avr ) / 2. ) ) ) )
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
    
        @param dpoint: The (directional) point to test.
        @type dpoint: L{geometry.Point}
        @return: The membership degree (coverage) of the point.
        @rtype: C{float}
        """
        if not isinstance( dpoint, Point ):
            raise TypeError, ( "invalid point" )

        campoint = ( -self.pose ).map( dpoint )

        # visibility
        try:
            mu_v = min( self.Cvh.mu( campoint.x / campoint.z ),
                        self.Cvv.mu( campoint.y / campoint.z ) )
        except ZeroDivisionError:
            mu_v = 0.0

        # resolution
        mu_r = self.Cr.mu( campoint.z )

        # focus
        mu_f = self.Cf.mu( campoint.z )

        # direction
        if isinstance( campoint, DirectionalPoint ):
            r = sqrt( campoint.x ** 2 + campoint.y ** 2 )
            try:
                terma = ( campoint.y / r ) * sin( campoint.eta ) + \
                        ( campoint.x / r ) * cos( campoint.eta )
            except ZeroDivisionError:
                terma = 1.0
            try:
                termb = atan( r / campoint.z )
            except ZeroDivisionError:
                termb = pi / 2.0
            mu_d = min( max( ( float( campoint.rho ) - ( ( pi / 2.0 ) + \
                             terma * termb ) ) / self.zeta, 0.0 ), 1.0 )
        else:
            mu_d = 1.0

        # return min( mu_v, mu_r, mu_f, mu_d )
        return mu_v * mu_r * mu_f * mu_d

    def visualize( self, scale = 1.0, color = ( 1, 1, 1 ) ):
        """\
        Plot the camera in a 3D visual model.

        @param scale: The scale of the camera.
        @type scale: C{float}
        @param color: The color in which to plot the point.
        @type color: C{tuple}
        """
        if not VIS:
            raise NotImplementedError( "visual module not loaded" )
        visual.pyramid( pos = self.pose.T.tuple, axis = \
            self.pose.map_rotate( Point( 0, 0, -scale ) ).tuple, \
            size = ( scale, scale, scale ), color = color )


class MultiCamera( IndexedSet ):
    """\
    Abstract base class for multi-camera model.
    """
    def __init__( self, scene, cameras = set(), directional = True ):
        """\
        Constructor.

        @param scene: The discrete scene model.
        @type scene: L{Scene}
        @param cameras: The initial set of cameras.
        @type cameras: C{set}
        @param directional: Use directional points if true.
        @type directional: C{bool}
        """
        if self.__class__ is MultiCamera:
            raise NotImplementedError, ( "please use one of the subclasses" )
        self.directional = directional
        self.model = FuzzySet()
        self.scene = scene
        self.inscene = {}
        IndexedSet.__init__( self, 'name', cameras )

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
        for dpoint in self.scene.generate_points( d = self.directional \
                                                  and 'all' or None ):
            mu = self[ key ].mu( dpoint )
            if mu > 0 and not self.scene.occluded( dpoint, self[ key ].pose.T ):
                dpoints.append( FuzzyElement( dpoint, mu ) )
        self.inscene[ key ] = FuzzySet( dpoints )

    def performance( self ):
        """\
        Return the coverage performance of this multi-camera network.
        """
        return self.model.overlap( self.scene.D )

    def visualize( self, scale = 1.0, color = ( 1, 1, 1 ) ):
        """\
        Visualize all cameras and the directional points of the coverage model
        (with opacity reflecting degree of coverage).

        @param scale: The scale of the individual elements.
        @type scale: C{float}
        @param color: The color of cameras.
        @type color: C{tuple}
        """
        if not VIS:
            raise NotImplementedError( "visual module not loaded" )
        for camera in self:
            camera.visualize( scale = scale, color = color )
        self.scene.visualize( color = color )
        for dpoint in self.model.keys():
            dpoint.visualize( scale = scale, color = ( 1, 0, 0 ),
                              opacity = self.model[ dpoint ].mu )


class MultiCameraSimple( MultiCamera ):
    """\
    Simple (single-camera coverage) multi-camera model.
    """
    def __init__( self, scene, cameras = set(), directional = True ):
        """\
        Constructor.

        @param scene: The discrete scene model.
        @type scene: L{Scene}
        @param cameras: The initial set of cameras.
        @type cameras: C{set}
        @param directional: Use directional points if true.
        @type directional: C{bool}
        """
        MultiCamera.__init__( self, scene, cameras, directional )

    def update( self ):
        """\
        Update the simple multi-camera network discrete spatial-directional
        fuzzy set (coverage model).
        """
        self.model = FuzzySet()
        for camera in self:
            self.model |= self.inscene[ camera.name ]

    def mu( self, dpoint ):
        """\
        Return the individual membership degree of a point using the continuous
        in-scene camera coverage models (so the point does not necessarily need
        to fall on the discrete grid).

        @param dpoint: The (directional) point to test.
        @type dpoint: L{geometry.Point}
        @return: The membership degree (coverage) of the point.
        @rtype: C{float}
        """
        views = []
        for camera in self:
            if not self.scene.occluded( dpoint, camera.pose.T ):
                views.append( camera )
        try:
            return max( [ camera.mu( dpoint ) for camera in views ] )
        except ValueError:
            return 0.0


class MultiCamera3D( MultiCamera ):
    """\
    3D (dual-camera coverage) multi-camera model.
    """
    def __init__( self, scene, cameras = set(), directional = True ):
        """\
        Constructor.

        @param scene: The discrete scene model.
        @type scene: L{Scene}
        @param cameras: The initial set of cameras.
        @type cameras: C{set}
        @param directional: Use directional points if true.
        @type directional: C{bool}
        """
        MultiCamera.__init__( self, scene, cameras, directional )

    def update( self ):
        """\
        Update the 3D multi-camera network discrete spatial-directional fuzzy
        set (coverage model).
        """
        self.model = FuzzySet( [ FuzzyElement( dpoint, mu = 0.0 ) \
                     for dpoint in self.scene.generate_points() ] )
        pairs = set( [ self.inscene[ pair[ 0 ] ] & \
                       self.inscene[ pair[ 1 ] ] \
                       for pair in combinations( self.keys(), 2 ) ] )
        for pair in pairs:
            self.model |= pair

    def mu( self, dpoint ):
        """\
        Return the individual membership degree of a point using the continuous
        in-scene camera coverage models (so the point does not necessarily need
        to fall on the discrete grid).

        @param dpoint: The (directional) point to test.
        @type dpoint: L{geometry.Point}
        @return: The membership degree (coverage) of the point.
        @rtype: C{float}
        """
        # TODO: account for occlusion
        return max( [ min( self[ pair[ 0 ] ].mu( dpoint ), \
                           self[ pair[ 1 ] ].mu( dpoint ) ) \
                      for pair in combinations( self.keys(), 2 ) ] )
