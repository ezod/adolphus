"""\
Geometry module. Contains point (vector) and pose transformation classes, and
geometric descriptor functions for features.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from math import pi, sqrt, sin, cos, asin, acos, atan
import numpy


class Angle( float ):
    """\
    Angle class. All operations are modulo 2 * pi.
    """
    def __new__( cls, arg = 0.0 ):
        return float.__new__( cls, float( arg ) % ( 2 * pi ) )

    def __eq__( self, other ):
        return float( self ) == float( other ) % ( 2 * pi )

    def __ne__( self, other ):
        return not self == other

    def __gt__( self, other ):
        return float( self ) > float( other ) % ( 2 * pi )

    def __lt__( self, other ):
        return float( self ) < float( other ) % ( 2 * pi )

    def __ge__( self, other ):
        return float( self ) >= float( other ) % ( 2 * pi )

    def __le__( self, other ):
        return float( self ) <= float( other ) % ( 2 * pi )

    def __add__( self, other ):
        return Angle( float( self ) + float( other ) )

    def __sub__( self, other ):
        return Angle( float( self ) - float( other ) )

    def __neg__( self ):
        return Angle( -float( self ) )


class Point( object ):
    """\
    3D point (vector) class.
    """
    def __init__( self, x, y, z ):
        """\
        Constructor.
    
        @param x: The x coordinate.
        @type x: C{float}
        @param y: The y coordinate.
        @type y: C{float}
        @param z: The z coordinate.
        @type z: C{float}
        """
        self.x = float( x )
        self.y = float( y )
        self.z = float( z )

    def __getitem__( self, i ):
        """\
        Return x, y, and z via numerical indexing.
    
        @param i: The numerical index.
        @type i: C{int}
        @return: The indexed coordinate.
        @rtype: C{float}
        """
        if i == 0:
            return self.x
        elif i == 1:
            return self.y
        elif i == 2:
            return self.z

    def __setitem__( self, i, value ):
        """\
        Set x, y, and z via numerical indexing.
    
        @param i: The numerical index.
        @type i: C{int}
        @param value: The new coordinate value.
        @type value: C{float}
        """
        if i == 0:
            self.x = value
        elif i == 1:
            self.y = value
        elif i == 2:
            self.z = value

    def __add__( self, p ):
        """\
        Vector addition.

        @param p: The operand vector.
        @type p: L{Point}
        @return: Result vector.
        @rtype: L{Point}
        """
        return Point( self.x + p.x, self.y + p.y, self.z + p.z )

    def __sub__( self, p ):
        """\
        Vector subtraction.

        @param p: The operand vector.
        @type p: L{Point}
        @return: Result vector.
        @rtype: L{Point}
        """
        return Point( self.x - p.x, self.y - p.y, self.z - p.z )

    def __mul__( self, p ):
        """\
        Scalar multiplication (if p is a scalar) or dot product (if p is a
        vector).

        @param p: The operand scalar or vector.
        @type p: C{float} or L{Point}
        @return: Result vector.
        @rtype: L{Point}
        """
        if isinstance( p, Point ):
            return ( self.x * p.x + self.y * p.y + self.z * p.z )
        else:
            return Point( self.x * p, self.y * p, self.z * p )

    def __rmul__( self, p ):
        """\
        Scalar multiplication (if p is a scalar) or dot product (if p is a
        vector).

        @param p: The operand scalar or vector.
        @type p: C{float} or L{Point}
        @return: Result vector.
        @rtype: L{Point}
        """
        return self.__mul__( p )

    def __div__( self, p ):
        """\
        Scalar division.

        @param p: The scalar divisor.
        @type p: C{float}
        @return: Result vector.
        @rtype: L{Point}
        """
        return Point( self.x / p, self.y / p, self.z / p )

    def __neg__( self ):
        """\
        Negation.

        @return: Result vector.
        @rtype: L{Point}
        """
        return Point( -self.x, -self.y, -self.z )

    def __repr__( self ):
        """\
        String representation, displays in a tuple format.

        @return: Vector string.
        @rtype: C{string}
        """
        return "(" + str( self.x ) + ", " + str( self.y ) + ", " + \
        str( self.z ) + ")"

    @property
    def tuple( self ):
        """
        Return a tuple of this vector.

        @return: Vector tuple.
        @rtype: C{tuple}
        """
        return ( self.x, self.y, self.z )

    @property
    def array( self ):
        """\
        Return a NumPy array of this vector.

        @return: Vector array.
        @rtype: C{numpy.array}
        """
        return numpy.array( [ [ self.x ], [ self.y ], [ self.z ] ] )

    @property
    def magnitude( self ):
        """\
        Return the magnitude of this vector.

        @return: Vector magnitude.
        @rtype: C{float}
        """
        return sqrt( self.x ** 2 + self.y ** 2 + self.z ** 2 )

    @property
    def normal( self ):
        """\
        Return a unit vector in the direction of this vector.

        @return: Unit vector.
        @rtype: L{Point}
        """
        m = self.magnitude
        return Point( self.x / m, self.y / m, self.z / m )

    def euclidean( self, p ):
        """\
        Return the Euclidean distance from this point to another.

        @param p: The other point.
        @type p: L{Point}
        @return: Euclidean distance.
        @rtype: C{float}
        """
        return sqrt( ( self.x - p.x ) ** 2 + ( self.y - p.y ) ** 2 + \
        ( self.z - p.z ) ** 2 )

    def angle( self, p ):
        """
        Return the angle between this vector and another.
    
        @param p: The other vector.
        @type p: L{Point}
        @return: Angle in radians.
        @rtype: L{Angle}
        """
        return Angle( acos( p.normal * self.normal ) )


class DirectionalPoint( Point ):
    """\
    3D directional point (spatial-directional vector) class.
    """
    def __init__( self, x, y, z, rho, eta ):
        """\
        Constructor.
    
        @param x: The x coordinate.
        @type x: C{float}
        @param y: The y coordinate.
        @type y: C{float}
        @param z: The z coordinate.
        @type z: C{float}
        @param rho: The inclination angle, from positive z-axis.
        @type rho: L{Angle}
        @param eta: The azimuth angle, right-handed from positive x-axis.
        @type eta: L{Angle}
        """
        Point.__init__( self, x, y, z )
        self.rho = Angle( rho )
        self.eta = Angle( eta )
        self._normalize()

    def _normalize( self ):
        """\
        Normalize the inclination angle to within [0,pi), reversing the azimuth
        angle as necesary.
        """
        if self.rho > pi:
            self.rho -= 2. * ( self.rho - pi )
            self.eta += pi


class Pose( object ):
    """\
    Pose (rigid 3D Euclidean transformation) class.
    """
    def __init__( self, T, R ):
        """\
        Constructor.

        @param T: The 3-element translation vector.
        @type T: L{Point}
        @param R: The 3x3 rotation matrix.
        @type R: C{numpy.ndarray}
        """
        if isinstance( T, Point ):
            self.T = T
        elif T is None:
            self.T = Point( 0, 0, 0 )
        else:
            raise TypeError( "Translation vector must be a Point or None" )
        if isinstance( R, numpy.ndarray ):
            self.R = R
        elif R is None:
            self.R = numpy.array( [ [ 1.0, 0.0, 0.0 ], [ 0.0, 1.0, 0.0 ], \
            [ 0.0, 0.0, 1.0 ] ] )
        else:
            raise TypeError( "Rotation matrix must be a NumPy array or None" )

    def __add__( self, other ):
        """\
        Pose composition: PB(PA(x)) = (PA + PB)(x).

        @param other: The other pose transformation.
        @type other: L{Pose}
        @return: Composed transformation.
        @rtype: L{Pose}
        """
        if not isinstance( other, Pose ):
            raise TypeError( "Argument must be a pose" )
        Tnew = Point( ( other.R[ 0 ][ 0 ] * self.T.x + other.R[ 0 ][ 1 ] * \
                      self.T.y + other.R[ 0 ][ 2 ] * self.T.z ),
                      ( other.R[ 1 ][ 0 ] * self.T.x + other.R[ 1 ][ 1 ] * \
                      self.T.y + other.R[ 1 ][ 2 ] * self.T.z ),
                      ( other.R[ 2 ][ 0 ] * self.T.x + other.R[ 2 ][ 1 ] * \
                      self.T.y + other.R[ 2 ][ 2 ] * self.T.z ) ) + other.T
        Rnew = numpy.dot( other.R, self.R )
        return Pose( Tnew, Rnew )

    def __sub__( self, other ):
        """\
        Pose composition with the inverse.
    
        @param other: The other pose transformation.
        @type other: L{Pose}
        @return: Composed transformation.
        @rtype: L{Pose}
        """
        return self.__add__( -other )

    def __neg__( self ):
        """\
        Pose inversion.

        @return: Inverted pose.
        @rtype: L{Pose}
        """
        Rinv = self.R.transpose()
        Tinv = Point( 0, 0, 0 )
        for i in range( 3 ):
            Tinv[ i ] = -( Rinv[ i ][ 0 ] * self.T.x + \
                           Rinv[ i ][ 1 ] * self.T.y + \
                           Rinv[ i ][ 2 ] * self.T.z )
        return Pose( Tinv, Rinv )

    def __repr__( self ):
        """\
        String representation, display T and R.

        @return: String representations of T and R.
        @rtype: C{string}
        """
        return str( self.T ) + "\n" + str( self.R )

    @property
    def om( self ):
        """\
        Return the fixed-axis rotation angles from R.

        @return: Tuple containing roll, pitch, and yaw angles.
        @rtype: C{tuple}
        """
        phi = Angle( asin( -1.0 * self.R[ 2 ][ 0 ] ) )
        sign = cos( phi ) / abs( cos( phi ) )
        theta = Angle( atan( self.R[ 2 ][ 1 ] / self.R[ 2 ][ 2 ] ) )
        if sign * self.R[ 2 ][ 2 ] < 0:
            theta += pi
        psi = Angle( atan( self.R[ 1 ][ 0 ] / self.R[ 0 ][ 0 ] ) )
        if sign * self.R[ 0 ][ 0 ] < 0:
            psi += pi
        return ( theta, phi, psi )

    @property
    def nonzero( self ):
        """\
        Check if this pose transformation has any effect.

        @return: True if this is a nontrivial mapping.
        @rtype: C{bool}
        """
        if self.T.x == 0 and self.T.y == 0 and self.T.z == 0 \
        and ( self.R - numpy.diag( numpy.array( [ 1, 1, 1 ] ) ) ).any() == 0:
            return False
        else:
            return True

    def map( self, p ):
        """\
        Map a point/vector through this pose.

        @param p: The point/vector to transform.
        @type p: L{Point}
        @return: The mapped point/vector.
        @rtype: L{Point}
        """
        return self.map_translate( self.map_rotate( p ) )

    def map_rotate( self, p ):
        """\
        Rotation component of point/vector mapping.

        @param p: The point/vector to rotate.
        @type p: L{Point}
        @return: The rotated point/vector.
        @rtype: L{Point}
        """
        x = ( self.R[ 0 ][ 0 ] * p.x + \
              self.R[ 0 ][ 1 ] * p.y + \
              self.R[ 0 ][ 2 ] * p.z )
        y = ( self.R[ 1 ][ 0 ] * p.x + \
              self.R[ 1 ][ 1 ] * p.y + \
              self.R[ 1 ][ 2 ] * p.z )
        z = ( self.R[ 2 ][ 0 ] * p.x + \
              self.R[ 2 ][ 1 ] * p.y + \
              self.R[ 2 ][ 2 ] * p.z )
        if isinstance( p, DirectionalPoint ):
            rho = p.rho # TODO: calculate new rho
            eta = p.eta # TODO: calculate new eta
            return DirectionalPoint( x, y, z, rho, eta )
        else:
            return Point( x, y, z )

    def map_translate( self, p ):
        """\
        Translation component of point/vector mapping.

        @param p: The point/vector to translate.
        @type p: L{Point}
        @return: The translated point/vector.
        @rtype: L{Point}
        """
        q = p + self.T
        return q


def rotation_matrix( angle ):
    """\
    Generate R given a tuple of 3 fixed-axis rotation angles.

    @param angle: Rotation angles in radians.
    @type angle: C{tuple} of L{Angle}
    @return: Rotation matrix.
    @rtype: L{numpy.ndarray}
    """
    theta = Angle( angle[ 0 ] )
    phi = Angle( angle[ 1 ] )
    psi = Angle( angle[ 2 ] )

    R = numpy.ndarray( ( 3, 3 ) )

    R[ 0 ][ 0 ] = cos( phi ) * cos( psi )
    R[ 0 ][ 1 ] = sin( theta ) * sin( phi ) * \
                  cos( psi ) - cos( theta ) * sin( psi )
    R[ 0 ][ 2 ] = cos( theta ) * sin( phi ) * \
                  cos( psi ) + sin( theta ) * sin( psi )
    R[ 1 ][ 0 ] = cos( phi ) * sin( psi )
    R[ 1 ][ 1 ] = sin( theta ) * sin( phi ) * \
                  sin( psi ) + cos( theta ) * cos( psi )
    R[ 1 ][ 2 ] = cos( theta ) * sin( phi ) * \
                  sin( psi ) - sin( theta ) * cos( psi )
    R[ 2 ][ 0 ] = -sin( phi )
    R[ 2 ][ 1 ] = sin( theta ) * cos( phi )
    R[ 2 ][ 2 ] = cos( theta ) * cos( phi )

    return R
