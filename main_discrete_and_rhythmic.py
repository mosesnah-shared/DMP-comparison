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

    ctrl.set_impedance( Kq = np.diag( [ 300, 150 ] ), Bq = np.diag( [ 30, 3 ] ) )
    n = my_sim.n_act

    mov_arrs  = np.array(  [  0., 0., 0., 1., 1. ] )     

    ctrl.add_mov_pars( q0i = mov_arrs[ :n ], q0f = mov_arrs[ n:2*n ], D = mov_arrs[ -1 ], ti = args.start_time + 3  )    
    ctrl.add_mov_pars( q0i = np.zeros( n ), q0f = mov_arrs[ :n ] - mov_arrs[ n:2*n ], D = mov_arrs[ -1 ], ti = args.start_time + 4* mov_arrs[ -1 ] )        

    # Add rhythmic movements too
    ctrl.add_rhythmic_mov( amp = np.array( [ 0., 0.3 ] ), w = 3 )    

    # Add the controller and objective of the simulation
    my_sim.add_ctrl( ctrl )

    init_cond = { "qpos": np.zeros( n ) ,  "qvel": np.zeros( n ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Run the simulation
    my_sim.run( )

    if args.is_save_data:  ctrl.export_data( my_sim.tmp_dir )

    my_sim.close( )

def run_movement_primitives( my_sim ):
    NotImplementedError( )

    # Generate rhythmic DMP

    # Generate discrete DMP
    # This will be the goal 


if __name__ == "__main__":

    # Generate an instance of our Simulation
    # The model is generated since the model name is passed via arguments
    ctrl_type = "movement"
                                                                                
    # Generate the parser, which is defined 
    parser = my_parser( )
    args, unknown = parser.parse_known_args( )

    args.model_name = "1DOF_planar_torque"
    my_sim = Simulation( args )

    assert ctrl_type in [    "motor", "movement" ]

    # Set the camera position of the simulation
    # Lookat [3] Distance, Elevation, Azimuth
    args.cam_pos = np.array( [ 0, 1, 0, 5, -90, 90 ] )    

    if    ctrl_type == "motor"   :    run_motor_primitives( my_sim )
    elif  ctrl_type == "movement": run_movement_primitives( my_sim )

