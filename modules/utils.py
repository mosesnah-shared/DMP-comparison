import re
import sys
import math

import numpy as np

from scipy.spatial.transform import Rotation  as R
from       modules.constants import Constants as C

# Define functions to be imported when used "import *"
# __all__  = [ "my_parser", "min_jerk_traj", "str2float", "get_model_prop", "get_data_prop", "get_length", "make_whip_downwards", "quaternion2euler", "print_vars" ] 

def skew_sym( w ):
    assert len( w ) == 3

    wtilde = np.zeros( ( 3, 3 ) )

    wtilde[ 0, 1 ] = -w[ 2 ]
    wtilde[ 0, 2 ] =  w[ 1 ]
    wtilde[ 2, 1 ] = -w[ 0 ]

    wtilde[ 1, 0 ] =  w[ 2 ]
    wtilde[ 2, 0 ] = -w[ 1 ]
    wtilde[ 1, 2 ] =  w[ 0 ]

    return wtilde

def quat2angx( q ):

    assert q[ 0 ] <= 1

    theta = 2 * np.arccos( q[ 0 ] )

    axis = q[ 1: ]

    # If the axis values are super small, then 
    tmp = np.sum( axis**2 )

    if tmp != 0:
        axis = axis/ np.sqrt( tmp )

    else:
        axis = np.array( [ 1., 0., 0. ] )
        theta = 0 

    return theta, axis


def rotx( q ):
    Rx = np.array( [ [ 1,            0,            0 ], 
                     [ 0,  np.cos( q ), -np.sin( q ) ],
                     [ 0,  np.sin( q ),  np.cos( q ) ]  ] )

    return Rx

def roty( q ):
    Ry = np.array( [ [  np.cos( q ),  0,  np.sin( q ) ], 
                     [            0,  1,            0 ],
                     [ -np.sin( q ),  0,  np.cos( q ) ]  ] )

    return Ry

def rotz( q ):
    Rz = np.array( [ [ np.cos( q ), -np.sin( q ), 0 ], 
                     [ np.sin( q ),  np.cos( q ), 0 ],
                     [           0,            0, 1 ]  ] )

    return Rz



def rot2quat( R: np.ndarray ):
    # [REF] https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
    # [REF] From Johannes

    assert len( R ) == 3 and len( R[ 0 ] ) == 3

    q = np.zeros( 4 )

    R00 = np.trace( R )
    tmp = np.array( [ R00, R[ 0,0 ], R[ 1,1 ], R[ 2,2 ] ] )
    k = np.argmax( tmp )

    q[ k ] = 0.5 * np.sqrt( 1 + 2 * tmp[ k ] - R00 )

    if k == 0:
        q[ 1 ] = 0.25/q[ k ] * ( R[ 2, 1 ] - R[ 1, 2 ] )
        q[ 2 ] = 0.25/q[ k ] * ( R[ 0, 2 ] - R[ 2, 0 ] )
        q[ 3 ] = 0.25/q[ k ] * ( R[ 1, 0 ] - R[ 0, 1 ] )

    elif k == 1:
        q[ 0 ] = 0.25/q[ k ] * ( R[ 2, 1 ] - R[ 1, 2 ] )
        q[ 2 ] = 0.25/q[ k ] * ( R[ 1, 0 ] + R[ 0, 1 ] )
        q[ 3 ] = 0.25/q[ k ] * ( R[ 0, 2 ] + R[ 2, 0 ] )

    elif k == 2:
        q[ 0 ] = 0.25/q[ k ] * ( R[ 0, 2 ] - R[ 2, 0 ] )
        q[ 2 ] = 0.25/q[ k ] * ( R[ 1, 0 ] + R[ 0, 1 ] )
        q[ 3 ] = 0.25/q[ k ] * ( R[ 2, 1 ] + R[ 1, 2 ] )

    elif k == 3:
        q[ 0 ] = 0.25/q[ k ] * ( R[ 1, 0 ] - R[ 0, 1 ] )
        q[ 1 ] = 0.25/q[ k ] * ( R[ 0, 2 ] + R[ 2, 0 ] )
        q[ 2 ] = 0.25/q[ k ] * ( R[ 2, 1 ] + R[ 1, 2 ] )

    if q[ 0 ] < 0 : q = -q

    return q




def quat2rot( quat: np.ndarray ):

    # [REF] https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html

    assert len( quat ) == 4

    q0, q1, q2 ,q3  = quat[:]    

    R = np.zeros( ( 3, 3 ) )

    R[ 0, 0 ] = q0 ** 2 + q1 ** 2 - q2 ** 2 - q3 ** 2
    R[ 0, 1 ] = 2 * q1 * q2 - 2 * q0 * q3
    R[ 0, 2 ] = 2 * q1 * q3 + 2 * q0 * q2

    R[ 1, 0 ] = 2 * q1 * q2 + 2 * q0 * q3
    R[ 1, 1 ] = q0 ** 2 - q1 ** 2 + q2 ** 2 - q3 ** 2
    R[ 1, 2 ] = 2 * q2 * q3 - 2 * q0 * q1

    R[ 2, 0 ] = 2 * q1 * q3 - 2 * q0 * q2
    R[ 2, 1 ] = 2 * q2 * q3 + 2 * q0 * q1
    R[ 2, 2 ] = q0 ** 2 - q1 ** 2 - q2 ** 2 + q3 ** 2

    return R


def min_jerk_traj( t: float, ti: float,  pi: float, pf: float, D: float ):
    """
        Returning the 1D position and velocity data at time t of the minimum-jerk-trajectory ( current time )
        Time should start at t = 0
        Note that the minimum-jerk-trajectory remains at the initial (respectively, final) posture before (after) the movement.
        Arguments
        ---------
            [1] t : current time
            [2] ti: start of the movement
            [3] tf: end   of the movement
            [4] pi: initial ( reference ) posture
            [5] pf: final   ( reference ) posture
            [6]  D: duration
    """

    assert  t >=  0 and ti >= 0 and D >= 0

    if   t <= ti:
        pos = pi
        vel = 0
        acc = 0

    elif ti < t <= ti + D:
        tau = ( t - ti ) / D                               
        pos =             pi + ( pf - pi ) * ( 10. * tau ** 3 -  15. * tau ** 4 +   6. * tau ** 5 )
        vel =           1./D * ( pf - pi ) * ( 30. * tau ** 2 -  60. * tau ** 3 +  30. * tau ** 4 )
        acc =  ( 1./D ** 2 ) * ( pf - pi ) * ( 60. * tau ** 1 - 180. * tau ** 2 + 120. * tau ** 3 )

    else:
        pos = pf
        vel = 0
        acc = 0

    return pos, vel, acc

def quat2euler( quat: np.ndarray ):                                         
    """
        Description
        -----------
        Converting a R4 quaternion vector (w, x, y, z) to Euler Angle (Roll, Pitch, Yaw)
        This code is directly from the following reference
        [REF] https://computergraphics.stackexchange.com/questions/8195/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr

        Arguments
        ---------
            [NAME]          [TYPE]        [DESCRIPTION]
            (1) quatVec     List          The quaternion vector, ordered in w, x, y and z

        Outputs
        --------
            [NAME]                   [TYPE]        [DESCRIPTION]
            (1) yaw, pitch, roll                   The euler angles of the given quaternion vector.
    """

    assert len( quat ) == 4

    w, x, y ,z  = quat[:]

    t0     =       + 2.0 * ( w * x + y * z )
    t1     = + 1.0 - 2.0 * ( x * x + y * y )
    roll   = math.atan2( t0, t1 )

    t2     = + 2.0 * ( w * y - z * x )
    t2     = + 1.0 if t2 > +1.0 else t2
    t2     = - 1.0 if t2 < -1.0 else t2
    pitch  = math.asin( t2 )

    t3     =       + 2.0 * ( w * z + x * y )
    t4     = + 1.0 - 2.0 * ( y * y + z * z )
    yaw    = math.atan2( t3, t4 )

    return yaw, pitch, roll

def str2float( string2parse : str ):
    """
        Return A list of float that is parsed from given string s
    """

    return [ float( i ) for i in re.findall( r"[-+]?\d*\.\d+|[-+]?\d+", string2parse ) ]


def get_model_prop( mj_model, elem_name: str, name: str, prop_name:str ):
    """
        A method which simplifies the sentence for calling the property in interest
        If executes the following sentence.
            mj_model."elem_name" + "prop_name", 

            [Example] mj_data.body_mass

            name is needed for finding that value. 
    """

    # Saving the method (mth) that we will use. 
    mth = getattr( mj_model, "_".join( [ elem_name, "name2id" ] ) )

    # Returning the value.
    return getattr( mj_model, "_".join( [ elem_name, prop_name ] ) )[  mth( "_".join( [ elem_name, name ]  ) )  ]

def get_data_prop( mj_model, mj_data, elem_name: str, name: str, prop_name:str ):
    """
        A method which simplifies the sentence for calling the property in interest
        If executes the following sentence.
            mj_data."elem_name" + "prop_name", 

            [Example] mj_data.body_mass

            name is needed for finding that value. 
    """

    # Saving the method (mth) that we will use. 
    mth = getattr( mj_model, "_".join( [ elem_name, "name2id" ] ) )

    # Returning the value.
    return getattr( mj_data, "_".join( [ elem_name, prop_name ] ) )[  mth( "_".join( [ elem_name, name ]  ) )  ]


def get_length( mj_model, mj_data, elem1_type:str, elem1_name:str, elem2_type:str, elem2_name:str ):
    """
        Get the Euclidean distance between two elements. 

        Arguments
        --------
            [1] elem1_type: "site" or "body" or "geom" etc. 
            [2] elem1_name: name of element 1
            [3] elem2_type: "site" or "body" or "geom" etc. 
            [4] elem2_name: name of element 2

        This function will eventually derive the distance between 
        {elem1_type}_{elem1_name} and {elem2_type}_{elem2_name}.

        The crucial point is that we should use "xpos" rather than "pos", because the former one returns the Cartesian coord. 

        [Example]
            - length_elem2elem( mj_data, "site", "upper_arm_end", "site", "fore_arm_end" )
              returns the distance between "site_upper_arm_end" and "site_fore_arm_end".

    """

    return np.linalg.norm( get_data_prop( mj_model, mj_data, elem1_type, elem1_name, "xpos" ) - get_data_prop( mj_model, mj_data, elem2_type, elem2_name, "xpos" )  , ord = 2  )


def print_vars( vars2print: dict , save_dir = sys.stdout ):
    """
        Print out all the details of the variables to the standard output + file to save. 
    """

    # Iterate Through the dictionary for printing out the values. 
    for var_name, var_vals in vars2print.items( ):

        # Check if var_vals is a list or numpy's ndarray else just change it as string 
        if   isinstance( var_vals, ( list, np.ndarray ) ):
            
            # First, change the list to numpy array to make the problem easier 
            var_vals = np.array( var_vals ) if isinstance( var_vals, list ) else var_vals

            # If the numpy array is
            var_vals = np.array2string( var_vals.flatten( ), separator =', ', floatmode = 'fixed' )

        else:
            var_vals = str( var_vals )

        print( f'[{var_name}]: {var_vals}', file = save_dir )


def geodesicSO3( rot1, rot2, t0i, D, t):
    """
    Compute the geodesic curve between two rotation matrices with respect to time.

    :param rot1: A 3x3 rotation matrix at the start.
    :param tot2: A 3x3 rotation matrix at the end.
    :param t0i: Initial time for the interpolation.
    :param D: Duration of the interpolation.
    :param t: Current time.
    :return: A 3x3 rotation matrix representing the current state of rotation.
    """
    # Before t0i, use rotation_matrix1
    if t <= t0i:
        return rot1
    
    # After t0i + D, use rotation_matrix2
    elif t >= t0i + D:
        return rot2

    # During the interpolation interval
    else:
        # Normalized time factor for interpolation
        tau = (t - t0i) / D
        
        # Interpolate using slerp
        rot_interp = rot1 @ R3_to_SO3( tau * SO3_to_R3( rot1.T @ rot2 ) )

        return rot_interp

def rotx(angle_degrees):
    """
    Create a rotation matrix for a rotation about the x-axis.

    :param angle_degrees: Rotation angle in degrees.
    :return: 3x3 rotation matrix.
    """
    angle_radians = np.radians(angle_degrees)
    c = np.cos(angle_radians)
    s = np.sin(angle_radians)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s, c]])

def SO3_to_R3(rotation_matrix):
    """
    Logarithmic map from SO(3) to R3.

    :param rotation_matrix: A 3x3 rotation matrix.
    :return: A 3D vector representing the axis-angle representation.
    """

    # Ensure the matrix is close to a valid rotation matrix
    if not np.allclose(np.dot(rotation_matrix.T, rotation_matrix), np.eye(3)) or not np.allclose(np.linalg.det(rotation_matrix), 1):
        raise ValueError("The input matrix is not a valid rotation matrix.")
 
    angle = np.arccos((np.trace(rotation_matrix) - 1) / 2)
    
    # Check for the singularity (angle close to 0)
    if np.isclose(angle, 0) or np.isnan( angle ):
        return np.zeros(3)

    # Compute the skew-symmetric matrix
    skew_symmetric = (rotation_matrix - rotation_matrix.T) / (2 * np.sin(angle))
    
    # Extract the rotation axis
    axis = np.array([skew_symmetric[2, 1], skew_symmetric[0, 2], skew_symmetric[1, 0]])

    return axis * angle

def R3_to_SO3( r3_vector ):
    angle = np.linalg.norm(r3_vector)
    if np.isclose(angle, 0):
        return np.eye(3)
    
    axis = r3_vector / angle
    skew_symmetric = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    rotation_matrix = np.eye(3) + np.sin(angle) * skew_symmetric + (1 - np.cos(angle)) * np.dot(skew_symmetric, skew_symmetric)

    return rotation_matrix