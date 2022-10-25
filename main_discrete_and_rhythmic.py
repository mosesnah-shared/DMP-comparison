"""

# ============================================================================= #
| Project:        [M3X Whip Project]
| Title:          Dynamic Motor Primitives, the most basic script
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
from controllers  import JointImpedanceController
from constants    import my_parser

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       
                                                                                
# Generate the parser, which is defined 
parser = my_parser( )
args, unknown = parser.parse_known_args( )

def run_motor_primitives( my_sim ):

    # Define the controller 
    ctrl = JointImpedanceController( my_sim, args, name = "joint_imp" )

    ctrl.set_impedance( Kq = 10 * np.eye( ctrl.n_act ), Bq = 5 *np.eye( ctrl.n_act ) )
    n = my_sim.n_act

    mov_arrs1  = np.array(  [ -0.5, -0.5,  0.1,  0.1, 0.6  ] )     
    mov_arrs2  = np.array(  [    0,    0, -0.6, -0.6, 0.7  ] )     

    ctrl.add_mov_pars( q0i = mov_arrs1[ :n ], q0f = mov_arrs1[ n:2*n ], D = mov_arrs1[ -1 ], ti = args.start_time  )    
    ctrl.add_mov_pars( q0i = mov_arrs2[ :n ], q0f = mov_arrs2[ n:2*n ], D = mov_arrs2[ -1 ], ti = 0.4  )    

    # Add rhythmic movements too

    # Add the controller and objective of the simulation
    my_sim.add_ctrl( ctrl )

    init_cond = { "qpos": mov_arrs1[ :n ] ,  "qvel": np.zeros( n ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Run the simulation
    my_sim.run( )

    if args.is_save_data:  ctrl.export_data( my_sim.tmp_dir )

    my_sim.close( )

def run_movement_primitives( my_sim ):
    NotImplementedError( )

if __name__ == "__main__":

    # Generate an instance of our Simulation
    # The model is generated since the model name is passed via arguments
    my_sim = Simulation( args )

    args.model_name = "2DOF_planar_torque"
    my_sim = Simulation( args )

    idx = 1

    if idx == 1: run_motor_primitives( my_sim )
    else: run_movement_primitives( my_sim )

