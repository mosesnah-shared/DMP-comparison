"""

# ============================================================================= #
| Project:        [M3X Whip Project]
| Title:          Dynamic Motor Primitives, multiple discrete movement
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
from controllers  import CartesianImpedanceController
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
    q1 = np.pi * 1/4
    init_cond = { "qpos": np.array( [ q1, np.pi-2*q1 ] ) ,  "qvel": np.zeros( n ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    xEEi = np.copy( my_sim.mj_data.get_site_xpos(  "site_end_effector" ) ) 
    xEEf = xEEi + 0.5 * np.array( [ np.cos( n * np.pi/4 ), np.sin( n * np.pi/4 ), 0 ] )

    # This is for rhythmic movement
    ctrl.add_mov_pars( x0i = xEEi, x0f = xEEf, D = 2, ti = args.start_time  )    
    # ctrl.add_mov_pars( x0i = xEEi, x0f = xEEf, D = 2, ti = args.start_time  )    


    init_cond = { "qpos": np.zeros( n ) ,  "qvel": np.zeros( n ) }
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
    args.model_name = "2DOF_planar_torque"    
    my_sim = Simulation( args )


    idx = 1

    if idx == 1: run_motor_primitives( my_sim )
    else: run_movement_primitives( my_sim )

