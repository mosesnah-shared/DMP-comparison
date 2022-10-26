"""

# ============================================================================= #
| Project:        [M3X Whip Project]
| Title:          Dynamic Motor Primitives, Kinematic Redundancy
| Author:         Moses C. Nah
| Email:          [Moses] mosesnah@mit.edu
# ============================================================================= #

"""


import os
import sys
import numpy      as np

# To Add Local Files, adding the directory via sys module
# __file__ saves the current directory of this file. 
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from simulation   import Simulation
from controllers  import CartesianImpedanceController, JointImpedanceController
from constants    import my_parser

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       
                                                                                
# Generate the parser, which is defined 
parser = my_parser( )
args, unknown = parser.parse_known_args( )

def run_motor_primitives( my_sim ):

    # Define the controller 
    ctrl = CartesianImpedanceController( my_sim, args, name = "task_imp" )

    ctrl.set_impedance( Kx = 300 * np.eye( 3 ), Bx = 100 * np.eye( 3 ) )

    n = my_sim.n_act
    q1 = np.pi * 1/6
    init_cond = { "qpos": np.array( [ q1, np.pi/2 -q1, np.pi/2 - q1 ] ) ,  "qvel": np.zeros( n ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    xEEi = np.copy( my_sim.mj_data.get_site_xpos(  "site_end_effector" ) ) 
    r = 0.5
    ctrl.add_rhythmic_mov( r = r, center = xEEi + np.array( [ 0., r, 0. ] ), w = 0.8 ) 

    # Superimpose another joint-space impedance controller
    # The position is on the mid point of the circle 
    ctrl2 = JointImpedanceController( my_sim, args, name = "joint_imp" )

    ctrl2.set_impedance( Kq = 10 * np.eye( n ), Bq =  6 * np.eye( n ) )
    q2 = np.arcsin(  np.sin( q1 ) + r/2. )
    q0_set = np.array( [ q2, np.pi/2 -q2, np.pi/2 - q2 ] )
    ctrl2.add_mov_pars( q0i = q0_set , q0f = q0_set, D = 1, ti = args.start_time  )    
   
    # Add the controller and objective of the simulation
    my_sim.add_ctrl( ctrl )
    my_sim.add_ctrl( ctrl2 )

    # Run the simulation
    my_sim.run( )

    if args.is_save_data:  ctrl.export_data( my_sim.tmp_dir )

    my_sim.close( )


def run_movement_primitives( my_sim ):
    NotImplementedError

if __name__ == "__main__":

    # Generate an instance of our Simulation
    # The model is generated since the model name is passed via arguments

    args.model_name = "3DOF_planar_torque"

    my_sim = Simulation( args )

    idx = 1

    if idx == 1: run_motor_primitives( my_sim )
    else: run_movement_primitives( my_sim )
