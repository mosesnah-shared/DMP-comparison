"""

# ============================================================================= #
| Project:        [M3X Whip Project]
| Title:          Dynamic Motor Primitives, no Kinematic Redundancy
| Author:         Moses C. Nah
| Email:          [Moses] mosesnah@mit.edu
# ============================================================================= #

"""

import os
import sys
import shutil
import numpy      as np
import scipy.io
from datetime   import datetime
from matplotlib import pyplot as plt
from mujoco_py  import functions

# To Add Local Files, adding the directory via sys module
# __file__ saves the current directory of this file. 
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from simulation   import Simulation
from controllers  import CartesianImpedanceController, JointImpedanceController
from utils        import min_jerk_traj
from constants    import my_parser
from constants    import Constants as C

sys.path.append( os.path.join( os.path.dirname(__file__), "DMPmodules" ) )

from CanonicalSystem            import CanonicalSystem 
from DynamicMovementPrimitives  import DynamicMovementPrimitives

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       
                                                                                
# Generate the parser, which is defined 
parser = my_parser( )
args, unknown = parser.parse_known_args( )


def getC( qpos, qvel  ):
    q1 = qpos[ 0 ]
    q2 = qpos[ 1 ]
    q3 = qpos[ 2 ]
    q4 = qpos[ 3 ]
    q5 = qpos[ 4 ]

    dq1 = qvel[ 0 ]
    dq2 = qvel[ 1 ]
    dq3 = qvel[ 2 ]
    dq4 = qvel[ 3 ]
    dq5 = qvel[ 4 ]    

    C = np.zeros( ( 5, 5 ) )

    C[0, 0] = - (dq2*np.sin(q2 + q3 + q4 + q5))/2 - (dq3*np.sin(q2 + q3 + q4 + q5))/2 - (dq4*np.sin(q2 + q3 + q4 + q5))/2 - (dq5*np.sin(q2 + q3 + q4 + q5))/2 - (5*dq2*np.sin(q2 + q3))/2 - (5*dq3*np.sin(q2 + q3))/2 - (3*dq3*np.sin(q3 + q4))/2 - (3*dq4*np.sin(q3 + q4))/2 - (dq4*np.sin(q4 + q5))/2 - (dq5*np.sin(q4 + q5))/2 - (7*dq2*np.sin(q2))/2 - (5*dq3*np.sin(q3))/2 - (3*dq4*np.sin(q4))/2 - (dq5*np.sin(q5))/2 - (3*dq2*np.sin(q2 + q3 + q4))/2 - (3*dq3*np.sin(q2 + q3 + q4))/2 - (3*dq4*np.sin(q2 + q3 + q4))/2 - (dq3*np.sin(q3 + q4 + q5))/2 - (dq4*np.sin(q3 + q4 + q5))/2 - (dq5*np.sin(q3 + q4 + q5))/2
    C[0, 1] = - (dq1*np.sin(q2 + q3 + q4 + q5))/2 - (dq2*np.sin(q2 + q3 + q4 + q5))/2 - (dq3*np.sin(q2 + q3 + q4 + q5))/2 - (dq4*np.sin(q2 + q3 + q4 + q5))/2 - (dq5*np.sin(q2 + q3 + q4 + q5))/2 - (5*dq1*np.sin(q2 + q3))/2 - (5*dq2*np.sin(q2 + q3))/2 - (5*dq3*np.sin(q2 + q3))/2 - (3*dq3*np.sin(q3 + q4))/2 - (3*dq4*np.sin(q3 + q4))/2 - (dq4*np.sin(q4 + q5))/2 - (dq5*np.sin(q4 + q5))/2 - (7*dq1*np.sin(q2))/2 - (7*dq2*np.sin(q2))/2 - (5*dq3*np.sin(q3))/2 - (3*dq4*np.sin(q4))/2 - (dq5*np.sin(q5))/2 - (3*dq1*np.sin(q2 + q3 + q4))/2 - (3*dq2*np.sin(q2 + q3 + q4))/2 - (3*dq3*np.sin(q2 + q3 + q4))/2 - (3*dq4*np.sin(q2 + q3 + q4))/2 - (dq3*np.sin(q3 + q4 + q5))/2 - (dq4*np.sin(q3 + q4 + q5))/2 - (dq5*np.sin(q3 + q4 + q5))/2
    C[0, 2] = - (dq1*np.sin(q2 + q3 + q4 + q5))/2 - (dq2*np.sin(q2 + q3 + q4 + q5))/2 - (dq3*np.sin(q2 + q3 + q4 + q5))/2 - (dq4*np.sin(q2 + q3 + q4 + q5))/2 - (dq5*np.sin(q2 + q3 + q4 + q5))/2 - (5*dq1*np.sin(q2 + q3))/2 - (5*dq2*np.sin(q2 + q3))/2 - (3*dq1*np.sin(q3 + q4))/2 - (5*dq3*np.sin(q2 + q3))/2 - (3*dq2*np.sin(q3 + q4))/2 - (3*dq3*np.sin(q3 + q4))/2 - (3*dq4*np.sin(q3 + q4))/2 - (dq4*np.sin(q4 + q5))/2 - (dq5*np.sin(q4 + q5))/2 - (5*dq1*np.sin(q3))/2 - (5*dq2*np.sin(q3))/2 - (5*dq3*np.sin(q3))/2 - (3*dq4*np.sin(q4))/2 - (dq5*np.sin(q5))/2 - (3*dq1*np.sin(q2 + q3 + q4))/2 - (3*dq2*np.sin(q2 + q3 + q4))/2 - (3*dq3*np.sin(q2 + q3 + q4))/2 - (dq1*np.sin(q3 + q4 + q5))/2 - (3*dq4*np.sin(q2 + q3 + q4))/2 - (dq2*np.sin(q3 + q4 + q5))/2 - (dq3*np.sin(q3 + q4 + q5))/2 - (dq4*np.sin(q3 + q4 + q5))/2 - (dq5*np.sin(q3 + q4 + q5))/2
    C[0, 3] = - (dq1*np.sin(q2 + q3 + q4 + q5))/2 - (dq2*np.sin(q2 + q3 + q4 + q5))/2 - (dq3*np.sin(q2 + q3 + q4 + q5))/2 - (dq4*np.sin(q2 + q3 + q4 + q5))/2 - (dq5*np.sin(q2 + q3 + q4 + q5))/2 - (3*dq1*np.sin(q3 + q4))/2 - (3*dq2*np.sin(q3 + q4))/2 - (dq1*np.sin(q4 + q5))/2 - (3*dq3*np.sin(q3 + q4))/2 - (dq2*np.sin(q4 + q5))/2 - (3*dq4*np.sin(q3 + q4))/2 - (dq3*np.sin(q4 + q5))/2 - (dq4*np.sin(q4 + q5))/2 - (dq5*np.sin(q4 + q5))/2 - (3*dq1*np.sin(q4))/2 - (3*dq2*np.sin(q4))/2 - (3*dq3*np.sin(q4))/2 - (3*dq4*np.sin(q4))/2 - (dq5*np.sin(q5))/2 - (3*dq1*np.sin(q2 + q3 + q4))/2 - (3*dq2*np.sin(q2 + q3 + q4))/2 - (3*dq3*np.sin(q2 + q3 + q4))/2 - (dq1*np.sin(q3 + q4 + q5))/2 - (3*dq4*np.sin(q2 + q3 + q4))/2 - (dq2*np.sin(q3 + q4 + q5))/2 - (dq3*np.sin(q3 + q4 + q5))/2 - (dq4*np.sin(q3 + q4 + q5))/2 - (dq5*np.sin(q3 + q4 + q5))/2
    C[0, 4] = -((np.sin(q3 + q4 + q5) + np.sin(q2 + q3 + q4 + q5) + np.sin(q4 + q5) + np.sin(q5))*(dq1 + dq2 + dq3 + dq4 + dq5))/2
    C[1, 0] = (dq1*np.sin(q2 + q3 + q4 + q5))/2 + (5*dq1*np.sin(q2 + q3))/2 - (3*dq3*np.sin(q3 + q4))/2 - (3*dq4*np.sin(q3 + q4))/2 - (dq4*np.sin(q4 + q5))/2 - (dq5*np.sin(q4 + q5))/2 + (7*dq1*np.sin(q2))/2 - (5*dq3*np.sin(q3))/2 - (3*dq4*np.sin(q4))/2 - (dq5*np.sin(q5))/2 + (3*dq1*np.sin(q2 + q3 + q4))/2 - (dq3*np.sin(q3 + q4 + q5))/2 - (dq4*np.sin(q3 + q4 + q5))/2 - (dq5*np.sin(q3 + q4 + q5))/2
    C[1, 1] = - (3*dq3*np.sin(q3 + q4))/2 - (3*dq4*np.sin(q3 + q4))/2 - (dq4*np.sin(q4 + q5))/2 - (dq5*np.sin(q4 + q5))/2 - (5*dq3*np.sin(q3))/2 - (3*dq4*np.sin(q4))/2 - (dq5*np.sin(q5))/2 - (dq3*np.sin(q3 + q4 + q5))/2 - (dq4*np.sin(q3 + q4 + q5))/2 - (dq5*np.sin(q3 + q4 + q5))/2
    C[1, 2] = - (3*dq1*np.sin(q3 + q4))/2 - (3*dq2*np.sin(q3 + q4))/2 - (3*dq3*np.sin(q3 + q4))/2 - (3*dq4*np.sin(q3 + q4))/2 - (dq4*np.sin(q4 + q5))/2 - (dq5*np.sin(q4 + q5))/2 - (5*dq1*np.sin(q3))/2 - (5*dq2*np.sin(q3))/2 - (5*dq3*np.sin(q3))/2 - (3*dq4*np.sin(q4))/2 - (dq5*np.sin(q5))/2 - (dq1*np.sin(q3 + q4 + q5))/2 - (dq2*np.sin(q3 + q4 + q5))/2 - (dq3*np.sin(q3 + q4 + q5))/2 - (dq4*np.sin(q3 + q4 + q5))/2 - (dq5*np.sin(q3 + q4 + q5))/2
    C[1, 3] = - (3*dq1*np.sin(q3 + q4))/2 - (3*dq2*np.sin(q3 + q4))/2 - (dq1*np.sin(q4 + q5))/2 - (3*dq3*np.sin(q3 + q4))/2 - (dq2*np.sin(q4 + q5))/2 - (3*dq4*np.sin(q3 + q4))/2 - (dq3*np.sin(q4 + q5))/2 - (dq4*np.sin(q4 + q5))/2 - (dq5*np.sin(q4 + q5))/2 - (3*dq1*np.sin(q4))/2 - (3*dq2*np.sin(q4))/2 - (3*dq3*np.sin(q4))/2 - (3*dq4*np.sin(q4))/2 - (dq5*np.sin(q5))/2 - (dq1*np.sin(q3 + q4 + q5))/2 - (dq2*np.sin(q3 + q4 + q5))/2 - (dq3*np.sin(q3 + q4 + q5))/2 - (dq4*np.sin(q3 + q4 + q5))/2 - (dq5*np.sin(q3 + q4 + q5))/2
    C[1, 4] = -((np.sin(q3 + q4 + q5) + np.sin(q4 + q5) + np.sin(q5))*(dq1 + dq2 + dq3 + dq4 + dq5))/2
    C[2, 0] = (dq1*np.sin(q2 + q3 + q4 + q5))/2 + (5*dq1*np.sin(q2 + q3))/2 + (3*dq1*np.sin(q3 + q4))/2 + (3*dq2*np.sin(q3 + q4))/2 - (dq4*np.sin(q4 + q5))/2 - (dq5*np.sin(q4 + q5))/2 + (5*dq1*np.sin(q3))/2 + (5*dq2*np.sin(q3))/2 - (3*dq4*np.sin(q4))/2 - (dq5*np.sin(q5))/2 + (3*dq1*np.sin(q2 + q3 + q4))/2 + (dq1*np.sin(q3 + q4 + q5))/2 + (dq2*np.sin(q3 + q4 + q5))/2
    C[2, 1] = (3*dq1*np.sin(q3 + q4))/2 + (3*dq2*np.sin(q3 + q4))/2 - (dq4*np.sin(q4 + q5))/2 - (dq5*np.sin(q4 + q5))/2 + (5*dq1*np.sin(q3))/2 + (5*dq2*np.sin(q3))/2 - (3*dq4*np.sin(q4))/2 - (dq5*np.sin(q5))/2 + (dq1*np.sin(q3 + q4 + q5))/2 + (dq2*np.sin(q3 + q4 + q5))/2
    C[2, 2] = - (dq4*np.sin(q4 + q5))/2 - (dq5*np.sin(q4 + q5))/2 - (3*dq4*np.sin(q4))/2 - (dq5*np.sin(q5))/2
    C[2, 3] = - (dq1*np.sin(q4 + q5))/2 - (dq2*np.sin(q4 + q5))/2 - (dq3*np.sin(q4 + q5))/2 - (dq4*np.sin(q4 + q5))/2 - (dq5*np.sin(q4 + q5))/2 - (3*dq1*np.sin(q4))/2 - (3*dq2*np.sin(q4))/2 - (3*dq3*np.sin(q4))/2 - (3*dq4*np.sin(q4))/2 - (dq5*np.sin(q5))/2
    C[2, 4] = -((np.sin(q4 + q5) + np.sin(q5))*(dq1 + dq2 + dq3 + dq4 + dq5))/2
    C[3, 0] = (dq1*np.sin(q2 + q3 + q4 + q5))/2 + (3*dq1*np.sin(q3 + q4))/2 + (3*dq2*np.sin(q3 + q4))/2 + (dq1*np.sin(q4 + q5))/2 + (dq2*np.sin(q4 + q5))/2 + (dq3*np.sin(q4 + q5))/2 + (3*dq1*np.sin(q4))/2 + (3*dq2*np.sin(q4))/2 + (3*dq3*np.sin(q4))/2 - (dq5*np.sin(q5))/2 + (3*dq1*np.sin(q2 + q3 + q4))/2 + (dq1*np.sin(q3 + q4 + q5))/2 + (dq2*np.sin(q3 + q4 + q5))/2
    C[3, 1] = (3*dq1*np.sin(q3 + q4))/2 + (3*dq2*np.sin(q3 + q4))/2 + (dq1*np.sin(q4 + q5))/2 + (dq2*np.sin(q4 + q5))/2 + (dq3*np.sin(q4 + q5))/2 + (3*dq1*np.sin(q4))/2 + (3*dq2*np.sin(q4))/2 + (3*dq3*np.sin(q4))/2 - (dq5*np.sin(q5))/2 + (dq1*np.sin(q3 + q4 + q5))/2 + (dq2*np.sin(q3 + q4 + q5))/2
    C[3, 2] = (dq1*np.sin(q4 + q5))/2 + (dq2*np.sin(q4 + q5))/2 + (dq3*np.sin(q4 + q5))/2 + (3*dq1*np.sin(q4))/2 + (3*dq2*np.sin(q4))/2 + (3*dq3*np.sin(q4))/2 - (dq5*np.sin(q5))/2
    C[3, 3] = -(dq5*np.sin(q5))/2
    C[3, 4] = -(np.sin(q5)*(dq1 + dq2 + dq3 + dq4 + dq5))/2
    C[4, 0] = (dq1*np.sin(q2 + q3 + q4 + q5))/2 + (dq1*np.sin(q4 + q5))/2 + (dq2*np.sin(q4 + q5))/2 + (dq3*np.sin(q4 + q5))/2 + (dq1*np.sin(q5))/2 + (dq2*np.sin(q5))/2 + (dq3*np.sin(q5))/2 + (dq4*np.sin(q5))/2 + (dq1*np.sin(q3 + q4 + q5))/2 + (dq2*np.sin(q3 + q4 + q5))/2
    C[4, 1] = (dq1*np.sin(q4 + q5))/2 + (dq2*np.sin(q4 + q5))/2 + (dq3*np.sin(q4 + q5))/2 + (dq1*np.sin(q5))/2 + (dq2*np.sin(q5))/2 + (dq3*np.sin(q5))/2 + (dq4*np.sin(q5))/2 + (dq1*np.sin(q3 + q4 + q5))/2 + (dq2*np.sin(q3 + q4 + q5))/2
    C[4, 2] = (dq1*np.sin(q4 + q5))/2 + (dq2*np.sin(q4 + q5))/2 + (dq3*np.sin(q4 + q5))/2 + (dq1*np.sin(q5))/2 + (dq2*np.sin(q5))/2 + (dq3*np.sin(q5))/2 + (dq4*np.sin(q5))/2
    C[4, 3] = (np.sin(q5)*(dq1 + dq2 + dq3 + dq4))/2
    C[4, 4] = 0

    return C 


def getdJ( qpos, qvel ):
    q1 = qpos[ 0 ]
    q2 = qpos[ 1 ]
    q3 = qpos[ 2 ]
    q4 = qpos[ 3 ]
    q5 = qpos[ 4 ]

    dq1 = qvel[ 0 ]
    dq2 = qvel[ 1 ]
    dq3 = qvel[ 2 ]
    dq4 = qvel[ 3 ]
    dq5 = qvel[ 4 ]

    dJ = np.zeros( ( 3, 5 ) )

    dJ[0, 0] = - np.cos(q1 + q2 + q3)*(dq1 + dq2 + dq3) - np.cos(q1 + q2)*(dq1 + dq2) - dq1*np.cos(q1) - np.cos(q1 + q2 + q3 + q4 + q5)*(dq1 + dq2 + dq3 + dq4 + dq5) - np.cos(q1 + q2 + q3 + q4)*(dq1 + dq2 + dq3 + dq4)
    dJ[0, 1] = - np.cos(q1 + q2 + q3)*(dq1 + dq2 + dq3) - np.cos(q1 + q2)*(dq1 + dq2) - np.cos(q1 + q2 + q3 + q4 + q5)*(dq1 + dq2 + dq3 + dq4 + dq5) - np.cos(q1 + q2 + q3 + q4)*(dq1 + dq2 + dq3 + dq4)
    dJ[0, 2] = - np.cos(q1 + q2 + q3)*(dq1 + dq2 + dq3) - np.cos(q1 + q2 + q3 + q4 + q5)*(dq1 + dq2 + dq3 + dq4 + dq5) - np.cos(q1 + q2 + q3 + q4)*(dq1 + dq2 + dq3 + dq4)
    dJ[0, 3] = - np.cos(q1 + q2 + q3 + q4 + q5)*(dq1 + dq2 + dq3 + dq4 + dq5) - np.cos(q1 + q2 + q3 + q4)*(dq1 + dq2 + dq3 + dq4)
    dJ[0, 4] = -np.cos(q1 + q2 + q3 + q4 + q5)*(dq1 + dq2 + dq3 + dq4 + dq5)
    dJ[1, 0] = - np.sin(q1 + q2 + q3)*(dq1 + dq2 + dq3) - np.sin(q1 + q2)*(dq1 + dq2) - dq1*np.sin(q1) - np.sin(q1 + q2 + q3 + q4 + q5)*(dq1 + dq2 + dq3 + dq4 + dq5) - np.sin(q1 + q2 + q3 + q4)*(dq1 + dq2 + dq3 + dq4)
    dJ[1, 1] = - np.sin(q1 + q2 + q3)*(dq1 + dq2 + dq3) - np.sin(q1 + q2)*(dq1 + dq2) - np.sin(q1 + q2 + q3 + q4 + q5)*(dq1 + dq2 + dq3 + dq4 + dq5) - np.sin(q1 + q2 + q3 + q4)*(dq1 + dq2 + dq3 + dq4)
    dJ[1, 2] = - np.sin(q1 + q2 + q3)*(dq1 + dq2 + dq3) - np.sin(q1 + q2 + q3 + q4 + q5)*(dq1 + dq2 + dq3 + dq4 + dq5) - np.sin(q1 + q2 + q3 + q4)*(dq1 + dq2 + dq3 + dq4)
    dJ[1, 3] = - np.sin(q1 + q2 + q3 + q4 + q5)*(dq1 + dq2 + dq3 + dq4 + dq5) - np.sin(q1 + q2 + q3 + q4)*(dq1 + dq2 + dq3 + dq4)
    dJ[1, 4] = -np.sin(q1 + q2 + q3 + q4 + q5)*(dq1 + dq2 + dq3 + dq4 + dq5)
    dJ[2, 0] = 0
    dJ[2, 1] = 0
    dJ[2, 2] = 0
    dJ[2, 3] = 0
    dJ[2, 4] = 0

    return dJ
        

def run_motor_primitives( mov_type ):

    args.model_name = "5DOF_planar_torque"

    # Set the camera position of the simulation
    # Lookat [3] Distance, Elevation, Azimuth
    args.cam_pos = np.array( [ 0, 0, 0, 10, -90, 90 ] )

    my_sim = Simulation( args )

    # Define the controller 
    ctrl1 = CartesianImpedanceController( my_sim, args, name = "task_imp" )
    ctrl1.set_impedance( Kx = 300 * np.eye( 3 ), Bx = 100 * np.eye( 3 ) )

    # Define the controller 
    ctrl2 = JointImpedanceController( my_sim, args, name = "joint_imp" )

    # The joint stiffness and damping matrices
    n = my_sim.n_act
    ctrl2.set_impedance( Kq = 0 * np.eye( n ), Bq = 30 * np.eye( n ) )
    
    my_sim.add_ctrl( ctrl1 )
    my_sim.add_ctrl( ctrl2 )

    if mov_type == "discrete":
        # Set the initial posture of the robot
        q1 = 0

        # Reference posture 
        ref_pos = np.array( [ q1, np.pi/2 - q1, 0, 0, np.pi/2-q1 ] )
        init_cond = { "qpos": ref_pos ,  "qvel": np.zeros( n ) }
        my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

        # Get the initial position of the robot 
        xEEi = my_sim.mj_data.get_site_xpos( "site_end_effector" )
        
        # Define the final posture 
        xEEf = xEEi + np.array( [ 3.0, 0, 0 ] )

        ctrl1.add_mov_pars( x0i = xEEi, x0f = xEEf, D = 1, ti = args.start_time  )    

        # Set the joint controller as zero
        ctrl2.add_mov_pars( q0i = ref_pos, q0f = ref_pos, D = 2, ti = 0 )    

        
    else:
        r = 0.5 
        omega0 = np.pi

        q1 = np.pi * 1/6

        ref_pos = np.array( [ q1, np.pi/2 - q1, 0, 0, np.pi/2-q1 ] )
        init_cond = { "qpos": ref_pos ,  "qvel": np.zeros( n ) }

        my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

        # Get the initial position of the robot 
        xEEi = my_sim.mj_data.get_site_xpos( "site_end_effector" )
        c = xEEi[ 1 ]        

        ctrl1.add_rhythmic_mov( amp = r, center = [ 0, c + r ], omega = omega0 )

        # Set the joint controller as zero
        ctrl2.add_mov_pars( q0i = ref_pos, q0f = ref_pos, D = 2, ti = 0 )    


    # Run the simulation
    my_sim.run( )

    if args.is_save_data:  
        ctrl1.export_data( my_sim.tmp_dir )
        ctrl2.export_data( my_sim.tmp_dir )

    my_sim.close( )


def run_movement_primitives( mov_type ):

    args.model_name = "5DOF_planar_torque"
    my_sim = Simulation( args )

    cs = CanonicalSystem( mov_type = mov_type )

    # Dynamic Movement Primitives
    # First  along the x-axis
    # Second along the y-axis
    dmp1 = DynamicMovementPrimitives( mov_type = mov_type, alpha_z = 10, beta_z = 2.5 )
    dmp2 = DynamicMovementPrimitives( mov_type = mov_type, alpha_z = 10, beta_z = 2.5 )

    # Adding the canonical system
    # DMPs share the same canonical system 
    dmp1.add_canonical_system( cs )
    dmp2.add_canonical_system( cs )

    T = args.run_time   

    # The duration of the movement
    D = 1

    if mov_type == "discrete":

        # Set the initial posture of the robot
        q1 = 0

        # The initial posture and setting the initial condition of the robot
        ref_pos = np.array( [ q1, np.pi/2 - q1, 0, 0, np.pi/2-q1 ] )
        init_cond = { "qpos": ref_pos ,  "qvel": np.zeros( len( ref_pos ) ) }
        my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

        # Get the initial position of the end-effector
        xEEi = my_sim.mj_data.get_site_xpos( "site_end_effector" )
        
        # Define the final posture 
        xEEf = xEEi + np.array( [ 3.0, 0, 0 ] )

        # The time step for imitation learning is 0.01. 
        dt = 0.01
        N = round( T/dt )
        t_arr = dt * np.arange( N )        

        # The first DMP is for the x direction
        x_des   = np.zeros( N )
        dx_des  = np.zeros( N )
        ddx_des = np.zeros( N )

        for i, t in enumerate( t_arr ):
            tmp_pos, tmp_vel, tmp_acc = min_jerk_traj( t, 0.0, xEEi[ 0 ], xEEf[ 0 ], D  )
            x_des[   i ] = tmp_pos
            dx_des[  i ] = tmp_vel
            ddx_des[ i ] = tmp_acc

        # The second DMP is for the y direction
        y_des   = np.zeros( N )
        dy_des  = np.zeros( N )
        ddy_des = np.zeros( N )

        for i, t in enumerate( t_arr ):
            tmp_pos, tmp_vel, tmp_acc = min_jerk_traj( t, 0.0, xEEi[ 1 ], xEEf[ 1 ], D  )
            y_des[   i ] = tmp_pos
            dy_des[  i ] = tmp_vel
            ddy_des[ i ] = tmp_acc            

        # Setting up the canonical system's parameters
        cs.tau = D

        n_bfs = 100
        dmp1.imitation_learning( t_arr, x_des, dx_des, ddx_des, n_bfs = n_bfs )
        dmp2.imitation_learning( t_arr, y_des, dy_des, ddy_des, n_bfs = n_bfs )

        t_arr1, y_arr1, z_arr1, dy_arr1, dz_arr1 = dmp1.integrate( xEEi[ 0 ], 0, xEEf[ 0 ], dt, round( D/dt ) )   
        t_arr2, y_arr2, z_arr2, dy_arr2, dz_arr2 = dmp2.integrate( xEEi[ 1 ], 0, xEEf[ 1 ], dt, round( D/dt ) )   
        
        # Redefine the new time step 
        dt  = my_sim.mj_model.opt.timestep         
        
        tmp1 = round( args.start_time/ dt )
        tmp2 = round( ( T - D - args.start_time )/ dt )
        print( tmp1, tmp2 )
        x_arr = np.hstack( ( y_arr1[ 0 ] * np.ones( tmp1 ), y_arr1, y_arr1[ -1 ] * np.ones( tmp2 + 1) )  )
        y_arr = np.hstack( ( y_arr2[ 0 ] * np.ones( tmp1 ), y_arr2, y_arr2[ -1 ] * np.ones( tmp2+ 1 ) )  )

        dx_arr = np.hstack( ( np.zeros( tmp1 ), dy_arr1, np.zeros( tmp2+ 1 ) )  )
        dy_arr = np.hstack( ( np.zeros( tmp1 ), dy_arr2, np.zeros( tmp2 + 1) )  )       

        ddx_arr = np.hstack( ( np.zeros( tmp1 ), dz_arr1/cs.tau, np.zeros( tmp2 + 1) )  )
        ddy_arr = np.hstack( ( np.zeros( tmp1 ), dz_arr2/cs.tau, np.zeros( tmp2 + 1) )  )        

        # Save the p_des, dp_des, ddp_des as a 3 by N array
        p_des   = np.vstack( (   x_arr,   y_arr, np.zeros( round( T/dt ) + 1) ) )
        dp_des  = np.vstack( (  dx_arr,  dy_arr, np.zeros( round( T/dt ) + 1) ) )
        ddp_des = np.vstack( ( ddx_arr, ddy_arr, np.zeros( round( T/dt ) + 1) ) )


        t = 0 
        idx = 0 
        frames = [ ]
        nq = my_sim.mj_model.nq

        Mtmp = np.zeros( nq * nq )

        # Define the raw simulation 
        while t <= T + 1e-7:

            # Render the simulation if mj_viewer exists        
            my_sim.mj_viewer.render( )

            if args.is_record_vid: 
                # Read the raw rgb image
                rgb_img = my_sim.mj_viewer.read_pixels( my_sim.mj_viewer.width, my_sim.mj_viewer.height, depth = False )

                # Convert BGR to RGB and flip upside down.
                rgb_img = np.flip( rgb_img, axis = 0 )
                
                # Add the frame list, this list will be converted to a video via moviepy library
                frames.append( rgb_img )  

            # If reset button (BACKSPACE) is pressed
            if my_sim.mj_viewer.is_reset:
                my_sim.mj_viewer.is_reset = False
            
            # If SPACE BUTTON is pressed
            if my_sim.mj_viewer.is_paused:    continue

            # Get current end-effector position and velocity 
            p  = np.copy( my_sim.mj_data.get_site_xpos(   "site_end_effector" ) )
            dp = np.copy( my_sim.mj_data.get_site_xvelp(  "site_end_effector" ) )

            q  = np.copy( my_sim.mj_data.qpos[ : ] )
            dq = np.copy( my_sim.mj_data.qvel[ : ] )

            # Define the reference p trajectory 
            dpr  =  dp_des[ :, idx ] + 10 * np.eye( 3 ) @ (  p_des[ :, idx ] -  p )
            ddpr = ddp_des[ :, idx ] + 10 * np.eye( 3 ) @ ( dp_des[ :, idx ] - dp )

            jac_new = np.copy( my_sim.mj_data.get_site_jacp(  "site_end_effector" ).reshape( 3, -1 ) )
            jac_pinv = np.linalg.pinv( jac_new )

            # Define the reference q trajectory 
            dJ = getdJ( q, dq )
            dqr  = jac_pinv @ dpr 
            ddqr = jac_pinv @ ( ddpr - dJ @ dq )

            functions.mj_fullM( my_sim.mj_model, Mtmp, my_sim.mj_data.qM )
            Mmat = np.copy( Mtmp.reshape( nq, -1 ) )
            Cmat = getC( q, dq )

            tau = Mmat @ ddqr + Cmat @ dqr - 10 * np.eye( nq ) @ ( dq - dqr )

            my_sim.mj_data.ctrl[ : ] = tau 

            my_sim.step( )

            t += dt
            idx += 1          


    else:
        # For rhythmic movement 
        # The x and y trajectory 
        r = 0.5 
        omega0 = np.pi
        c = np.sqrt( 2 )

        # The period of the system
        Tp = 2 * np.pi / omega0 
        dt = 0.01
        N  = round( Tp/dt )
        t_arr = dt * np.arange( N )

        # The first DMP is for the x direction
        x_des   = np.zeros( N )
        dx_des  = np.zeros( N )
        ddx_des = np.zeros( N )

        for i, t in enumerate( t_arr ):
            tmp_pos, tmp_vel, tmp_acc = r * np.sin( omega0 * t ), r * omega0 * np.cos( omega0 * t ), -r * ( omega0 ** 2 ) * np.sin( omega0 * t )
            x_des[   i ] = tmp_pos
            dx_des[  i ] = tmp_vel
            ddx_des[ i ] = tmp_acc

        # The second DMP is for the x direction
        y_des   = np.zeros( N )
        dy_des  = np.zeros( N )
        ddy_des = np.zeros( N )            

        for i, t in enumerate( t_arr ):
            tmp_pos, tmp_vel, tmp_acc = r * np.cos( omega0 * t ) + c, -r * omega0 * np.sin( omega0 * t ), -r * ( omega0 ** 2 ) * np.cos( omega0 * t )
            y_des[   i ] = tmp_pos
            dy_des[  i ] = tmp_vel
            ddy_des[ i ] = tmp_acc

        # Setting up the canonical system's parameters
        cs.tau = Tp / ( 2 * np.pi)

        # The number of Basis Functions
        n_bfs = 40
        dmp1.imitation_learning( t_arr, x_des, dx_des, ddx_des, n_bfs = n_bfs )
        dmp2.imitation_learning( t_arr, y_des, dy_des, ddy_des, n_bfs = n_bfs )

        t_arr1, y_arr1, z_arr1, dy_arr1, dz_arr1 = dmp1.integrate( x_des[ 0 ], dx_des[ 0 ], 0, 0.001, round( 2 * Tp/0.001 ) )   
        t_arr2, y_arr2, z_arr2, dy_arr2, dz_arr2 = dmp2.integrate( y_des[ 0 ], dy_des[ 0 ], c, 0.001, round( 2 * Tp/0.001 ) )   



if __name__ == "__main__":

    # Generate an instance of our Simulation
    # The model is generated since the model name is passed via arguments

    mov_type = "discrete"
    ctrl_type = "movement"

    if    ctrl_type == "motor"   :    run_motor_primitives( mov_type )
    elif  ctrl_type == "movement": run_movement_primitives( mov_type )

    

