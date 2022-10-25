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
from constants    import Constants  as C

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       
                                                                                
# Generate the parser, which is defined 
parser = my_parser( )
args, unknown = parser.parse_known_args( )


def run_motor_primitives( my_sim ):

    # Define the controller 
    ctrl = JointImpedanceController( my_sim, args, name = "joint_imp" )

    ctrl.set_impedance( Kq = np.array( [10] ), Bq = np.array( [ 5 ] ) )

    ctrl.add_mov_pars( q0i = np.array( [ 0 ] ) , q0f = np.array( [ 1 ] ), D = 1, ti = args.start_time  )    

    # Add the controller and objective of the simulation
    my_sim.add_ctrl( ctrl )

    init_cond = { "qpos": np.array( [ 0 ] ) ,  "qvel": np.array( [ 0 ] ) }
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

    args.model_name = "1DOF_planar_torque"
    my_sim = Simulation( args )

    idx = 1

    if idx == 1: run_motor_primitives( my_sim )
    else: run_movement_primitives( my_sim )

    

