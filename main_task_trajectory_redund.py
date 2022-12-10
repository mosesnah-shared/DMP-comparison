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
import math
import shutil
import numpy             as np
import matplotlib.pyplot as plt
import scipy.io
import moviepy.editor  as mpy
from datetime  import datetime
from mujoco_py import functions



# ======================================================================== #
# ======================================================================== #
#                                                                          #
#                            ADDING LOCAL MODULES                          #
#                                                                          #
# ======================================================================== #
# ======================================================================== #
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from simulation   import Simulation
from controllers  import CartesianImpedanceController, JointImpedanceController
from utils        import min_jerk_traj
from constants    import my_parser
from constants    import Constants as C

# ======================================================================== #
# ======================================================================== #
#                                                                          #
#               ADDING MODULES FOR DYNAMIC MOVEMENT PRIMITIVES             #
#                                                                          #
# ======================================================================== #
# ======================================================================== #
sys.path.append( os.path.join( os.path.dirname(__file__), "DMPmodules" ) )

from CanonicalSystem            import CanonicalSystem 
from DynamicMovementPrimitives  import DynamicMovementPrimitives

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       

# Helper Functions generated from MATLAB
# For a 5DOF Robotå
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
        

def run_motor_primitives( my_sim ):

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

    # Set the initial posture of the robot
    q1      = 0
    ref_pos = np.array( [ q1, np.pi/2 - q1, 0, 0, np.pi/2-q1 ] )
    init_cond = { "qpos": ref_pos ,  "qvel": np.zeros( n ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Get the initial position of the robot 
    p0i = my_sim.mj_data.get_site_xpos( "site_end_effector" )
    
    # Define the final posture 
    p0f = p0i + np.array( [ 3.0, 0, 0 ] )

    # Superposition of mechanical impedances
    ctrl1.add_mov_pars( x0i = p0i, x0f = p0f, D = 2, ti = args.start_time  )    
    ctrl2.add_mov_pars( q0i = ref_pos, q0f = ref_pos, D = 2, ti = 0 )    

    # Run the simulation
    my_sim.run( )

    if args.is_save_data:  
        ctrl1.export_data( my_sim.tmp_dir )
        ctrl2.export_data( my_sim.tmp_dir )

    my_sim.close( )


def run_movement_primitives( my_sim ):

    # Define the canonical system
    cs = CanonicalSystem( mov_type = "discrete" )

    # The number of degrees of freedom of the tobot 
    n = my_sim.nq

    # The time step of the simulation 
    dt = my_sim.dt

    # Dynamic Movement Primitives 
    # Define for x and y trajectory
    dmp_list = [] 

    # The number of basis functions
    N = 20

    for _ in range( 2 ):
        dmp = DynamicMovementPrimitives( mov_type = "discrete", cs = cs, n_bfs = N, alpha_z = 10, beta_z = 2.5, tau = 1.0 )
        dmp_list.append( dmp )
        
    # Set the initial posture of the robot
    q1 = 0
    ref_pos = np.array( [ q1, np.pi/2 - q1, 0, 0, np.pi/2-q1 ] )
    init_cond = { "qpos": ref_pos ,  "qvel": np.zeros( len( ref_pos ) ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Get the initial position of the end-effector
    p0i = my_sim.mj_data.get_site_xpos( "site_end_effector" )
    
    # Define the final posture 
    p0f = p0i + np.array( [ 3.0, 0, 0 ] )

    D = 2.0       

    # The time constant tau is the duration of the movement. 
    cs.tau = D        

    # The number of sample points for imitation learning
    P = 100

    # The time step of imitation learning
    # This is simply defined by D/P
    tmp_dt = D/P

    # The P samples points of p_des, dp_des, ddp_dex
    p_des   = np.zeros( ( 3, P + 1 ) )
    dp_des  = np.zeros( ( 3, P + 1 ) ) 
    ddp_des = np.zeros( ( 3, P + 1 ) )

    for i in range( 2 ):
        for j in range( P + 1 ):
            t = tmp_dt * j
            p_des[ i, j ], dp_des[ i, j ], ddp_des[ i, j ] = min_jerk_traj( t, 0.0, p0i[ i ], p0f[ i ], D  )



    # The time array for imitation learning
    # This learns the best fit weight of the dmp
    # For this, we need to define the 

    for i in range( 2 ):
        t_arr = tmp_dt * np.arange( P + 1 )
        dmp = dmp_list[ i ]
        dmp.imitation_learning( t_arr, p_des[ i, : ], dp_des[ i, : ], ddp_des[ i, : ] )

    # Now, we integrate this solution
    # For this, the initial and final time of the simulation is important
    N_sim = round( args.run_time/dt ) + 1

    p_command   = np.zeros( ( 3, N_sim ) )
    dp_command  = np.zeros( ( 3, N_sim ) )
    ddp_command = np.zeros( ( 3, N_sim ) )

    # Iterating through the dmps
    for i in range( 2 ):

        dmp = dmp_list[ i ]

        y_curr = p0i[ i ]
        z_curr = 0

        for j in range( N_sim  ):
            t = dt * j 

            if t <= args.start_time: 
                p_command[ i, j ]   = p0i[ i ]
                dp_command[ i, j ]  = 0
                ddp_command[ i, j ] = 0

            else:
                # Integrate the solution 
                # Calculate the force from weights
                # Get the current canonical function value
                s = cs.get_value( t - args.start_time )

                f = dmp.basis_functions.calc_nonlinear_forcing_term( s, dmp.weights )
                f *= s * ( p0f[ i ] - p0i[ i ] ) 
                
                y_new, z_new, dy, dz = dmp.step( p0f[ i ], y_curr, z_curr, f, dt )
                p_command[ i, j ]   = y_new
                dp_command[ i, j ]  = dy #z_new / cs.tau
                ddp_command[ i, j ] = dz/cs.tau # dz / cs.tau
                y_curr = y_new
                z_curr = z_new 


    # Since we now know the q_command, looping through the simulation 
    t = 0.
    T = args.run_time 

    n_steps = 0
    frames = [ ]
    
    if args.cam_pos is not None: my_sim.set_camera_pos( ) 

    Mtmp = np.zeros( n * n )

    q_arr    = []
    dq_arr   = []
    p_arr    = []
    dp_arr   = []
    dpr_arr  = []
    ddpr_arr = []        
    dqr_arr  = []
    ddqr_arr = []


    # Define the raw simulation 
    while t <= T + 1e-7:

        # Render the simulation if mj_viewer exists        
        if my_sim.mj_viewer is not None and n_steps % my_sim.vid_step == 0:

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

        p_arr.append( p )
        dp_arr.append( dp )

        q  = np.copy( my_sim.mj_data.qpos[ : ] )
        dq = np.copy( my_sim.mj_data.qvel[ : ] )

        q_arr.append( q )
        dq_arr.append( dq )

        # Define the reference p trajectory 
        dpr  =  dp_command[ :, n_steps ] + 80 * np.eye( 3 ) @ (  p_command[ :, n_steps ] -  p )
        ddpr = ddp_command[ :, n_steps ] + 80 * np.eye( 3 ) @ ( dp_command[ :, n_steps ] - dp )

        dpr_arr.append( dpr )
        ddpr_arr.append( ddpr )


        jac_new = np.copy( my_sim.mj_data.get_site_jacp(  "site_end_effector" ).reshape( 3, -1 ) )

        jac_pinv = np.linalg.pinv( jac_new )


        # Define the reference q trajectory 
        dJ = getdJ( q, dq )
        dqr  = jac_pinv @ dpr 


        ddqr = jac_pinv @ ( ddpr - dJ @ dq )

        dqr_arr.append( dpr )
        ddqr_arr.append( ddpr )



        functions.mj_fullM( my_sim.mj_model, Mtmp, my_sim.mj_data.qM )
        Mmat = np.copy( Mtmp.reshape( n, -1 ) )
        Cmat = getC( q, dq )

        tau = Mmat @ ddqr + Cmat @ dqr - 100 * np.eye( n ) @ ( dq - dqr )

        my_sim.mj_data.ctrl[ : ] = tau 

        my_sim.step( )

        t += dt
        n_steps += 1      

    # If video should be recorded, write the video file. 
    if args.is_record_vid and frames is not None:
        clip = mpy.ImageSequenceClip( frames, fps = my_sim.fps )
        clip.write_videofile( my_sim.tmp_dir + "video.mp4", fps = my_sim.fps, logger = None )

    # If video recorded/save data is true, then copy the model, main file and the arguments passed
    if args.is_record_vid or args.is_save_data:
        shutil.copyfile( C.MODEL_DIR + my_sim.model_name + ".xml", my_sim.tmp_dir + "model.xml" )                    
    
    # Saving the data for analysis
    if args.is_save_data:
        
        # Packing up the arrays as a dictionary
        # DMP for the first joint
        dmp1 = dmp_list[ 0 ]
        dmp2 = dmp_list[ 1 ]

        dict  = { "mov_type" : "discrete", "dt": dt, "x" : p_command[ 0, : ], "y" : p_command[ 1, : ], "tau1": dmp1.tau, "tau2": dmp2.tau, "alpha_s": cs.alpha_s,
                "weights1": dmp1.weights, "centers1": dmp1.basis_functions.centers, "heights1": dmp1.basis_functions.heights, 
                "weights2": dmp2.weights, "centers2": dmp2.basis_functions.centers, "heights2": dmp2.basis_functions.heights, 
                    "q": q_arr, "dq": dq_arr, "p": p_arr, "dp": dp_arr, "dpr": dpr_arr, "ddpr": ddpr_arr, "dqr": dqr_arr, "ddqr": ddqr_arr,
                "alpha_z": dmp1.alpha_z, "beta_z": dmp1.beta_z }

        scipy.io.savemat( my_sim.tmp_dir + "/dmp.mat", { **dict } )    

    # Move the tmp folder to results if not empty, else just remove the tmp file. 
    shutil.move( my_sim.tmp_dir, C.SAVE_DIR  ) if len( os.listdir( my_sim.tmp_dir ) ) != 0 else os.rmdir( my_sim.tmp_dir )



if __name__ == "__main__":


    ctrl_type = "movement"
                                                                                
    # Generate the parser, which is defined 
    parser = my_parser( )
    args, unknown = parser.parse_known_args( )

    args.model_name = "5DOF_planar_torque"
    my_sim = Simulation( args )

    assert ctrl_type in [    "motor", "movement" ]

    # Define the robot that we will use 
    args.model_name = "5DOF_planar_torque"

    # Set the camera position of the simulation
    # Lookat [3] Distance, Elevation, Azimuth
    args.cam_pos = np.array( [ 2.0, 2.0, 0, 9, -90, 90 ] )    

    if    ctrl_type == "motor"   :    run_motor_primitives( my_sim )
    elif  ctrl_type == "movement": run_movement_primitives( my_sim )

    

