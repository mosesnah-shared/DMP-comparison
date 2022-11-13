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
        

def run_motor_primitives( my_sim, mov_type ):

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

        ctrl1.add_mov_pars( x0i = xEEi, x0f = xEEf, D = 2, ti = args.start_time  )    

        # Set the joint controller as zero
        ctrl2.add_mov_pars( q0i = ref_pos, q0f = ref_pos, D = 2, ti = 0 )    

    elif mov_type == "rhythmic":
        NotImplementedError( )


    # Run the simulation
    my_sim.run( )

    if args.is_save_data:  
        ctrl1.export_data( my_sim.tmp_dir )
        ctrl2.export_data( my_sim.tmp_dir )

    my_sim.close( )


def run_movement_primitives( my_sim, mov_type ):


    # Define the canonical system
    cs = CanonicalSystem( mov_type = mov_type )

    n  = my_sim.nq
    dt = my_sim.dt

    # Dynamic Movement Primitives 
    # Define for x and y trajectory
    dmp_list = [] 
    for _ in range( 2 ):
        dmp = DynamicMovementPrimitives( mov_type = mov_type, alpha_z = 10, beta_z = 2.5 )
        dmp_list.append( dmp )
        
        # Adding the canonical system
        dmp.add_canonical_system( cs )      


    if mov_type == "discrete":

        # Set the initial posture of the robot
        q1 = 0

        # The initial posture and setting the initial condition of the robot
        ref_pos = np.array( [ q1, np.pi/2 - q1, 0, 0, np.pi/2-q1 ] )
        init_cond = { "qpos": ref_pos ,  "qvel": np.zeros( len( ref_pos ) ) }
        my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

        # Get the initial position of the end-effector
        p0i = my_sim.mj_data.get_site_xpos( "site_end_effector" )
        
        # Define the final posture 
        p0f = p0i + np.array( [ 3.0, 0, 0 ] )

        D = 2.0        # Duration D = 2 

        # The time constant tau is the duration of the movement. 
        cs.tau = D        

        # The number of sample points for imitation learning
        P = 100

        # The time step of imitation learning
        # This is simply defined by D/P
        tmp_dt = D/P

        # The P samples points of p_des, dp_des, ddp_dex
        p_des   = np.zeros( ( 2, P + 1 ) )
        dp_des  = np.zeros( ( 2, P + 1 ) ) 
        ddp_des = np.zeros( ( 2, P + 1 ) )

        for i in range( 2 ):
            for j in range( P + 1 ):
                t = tmp_dt * j
                p_des[ i, j ], dp_des[ i, j ], ddp_des[ i, j ] = min_jerk_traj( t, 0.0, p0i[ i ], p0f[ i ], D  )

        # The number of basis functions
        N = 20

        # The time array for imitation learning
        # This learns the best fit weight of the dmp
        # For this, we need to define the 

        for i in range( 2 ):
            t_arr = tmp_dt * np.arange( P + 1 )
            dmp = dmp_list[ i ]
            dmp.imitation_learning( t_arr, p_des[ i, : ], dp_des[ i, : ], ddp_des[ i, : ], n_bfs = N )

        # Now, we integrate this solution
        # For this, the initial and final time of the simulation is important
        N_sim = round( args.run_time/dt  )

        p_command   = np.zeros( ( 3, N_sim + 1 ) )
        dp_command  = np.zeros( ( 3, N_sim + 1 ) )
        ddp_command = np.zeros( ( 3, N_sim + 1 ) )

        for i in range( 2 ):

            dmp = dmp_list[ i ]

            y_curr = p0i[ i ]
            z_curr = 0

            for j in range( N_sim + 1 ):
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

                    psi_arr = np.array( [ dmp.basis_functions.calc_activation( k, s ) for k in np.arange( dmp.basis_functions.n_bfs ) ] )

                    # if psi_arr is super small, then just set for as zero since this implies there is no activation
                    if np.sum( psi_arr ) != 0:
                        f = np.sum( dmp.weights * psi_arr ) / np.sum( psi_arr )
                    else:
                        f = 0

                    # In case if f is nan, then just set f as 0 
                    if math.isnan( f ): f = 0 

                    f *= s * ( p0f[ i ] - p0i[ i ] ) 

                    y_new, z_new, dy, dz = dmp.step( p0f[ i ], y_curr, z_curr, f, dt )
                    p_command[ i, j ]   = y_new
                    dp_command[ i, j ]  = z_new / cs.tau
                    ddp_command[ i, j ] = dz / cs.tau

                    y_curr = y_new
                    z_curr = z_new 


        # Since we now know the q_command, looping through the simulation 
        # We assume pure position control 
        t = 0.
        nstep = 0 
        T = args.run_time 
        idx = 0 
        frames = [ ]
        
        if args.cam_pos is not None: my_sim.set_camera_pos( ) 

        Mtmp = np.zeros( n * n )

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
            dpr  =  dp_command[ :, idx ] + 10 * np.eye( 3 ) @ (  p_command[ :, idx ] -  p )
            ddpr = ddp_command[ :, idx ] + 10 * np.eye( 3 ) @ ( dp_command[ :, idx ] - dp )

            jac_new = np.copy( my_sim.mj_data.get_site_jacp(  "site_end_effector" ).reshape( 3, -1 ) )
            jac_pinv = np.linalg.pinv( jac_new )

            # Define the reference q trajectory 
            dJ = getdJ( q, dq )
            dqr  = jac_pinv @ dpr 
            ddqr = jac_pinv @ ( ddpr - dJ @ dq )

            functions.mj_fullM( my_sim.mj_model, Mtmp, my_sim.mj_data.qM )
            Mmat = np.copy( Mtmp.reshape( n, -1 ) )
            Cmat = getC( q, dq )

            tau = Mmat @ ddqr + Cmat @ dqr - 10 * np.eye( n ) @ ( dq - dqr )

            my_sim.mj_data.ctrl[ : ] = tau 

            my_sim.step( )

            t += dt
            idx += 1          

    elif mov_type == "rhythmic":
        NotImplementedError( )

if __name__ == "__main__":

    mov_type = "discrete"
    ctrl_type = "movement"
                                                                                
    # Generate the parser, which is defined 
    parser = my_parser( )
    args, unknown = parser.parse_known_args( )

    args.model_name = "5DOF_planar_torque"
    my_sim = Simulation( args )

    assert mov_type  in [ "discrete", "rhythmic" ]
    assert ctrl_type in [    "motor", "movement" ]

    # Define the robot that we will use 
    args.model_name = "5DOF_planar_torque"

    # Set the camera position of the simulation
    # Lookat [3] Distance, Elevation, Azimuth
    args.cam_pos = np.array( [ 0, 2.5, 0, 10, -90, 90 ] )    

    if    ctrl_type == "motor"   :    run_motor_primitives( my_sim, mov_type )
    elif  ctrl_type == "movement": run_movement_primitives( my_sim, mov_type )

    

