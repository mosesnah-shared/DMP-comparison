"""

# ============================================================================= #
| Project:        [M3X Whip Project]
| Title:          Obstacle Avoidance
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
from controllers  import CartesianImpedanceController, CartesianImpedanceControllerObstacle
from constants    import my_parser


np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       
parser = my_parser( )
args, unknown = parser.parse_known_args( )

if __name__ == "__main__":

    # Generate an instance of our Simulation
    # We use a 2-DOF robot arm with obstacle geometry
    args.model_name = "2DOF_planar_torque"

    my_sim = Simulation( args )

    # Add the movements of the Cartesian Impedance Controller 
    # Set the initial posture 
    n = my_sim.n_act
    init_cond = { "qpos": np.array( [ np.pi/4, np.pi/2 ] ) ,  "qvel": np.zeros( n ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # The end-effector position for the initial position
    xEEi = my_sim.mj_data.get_site_xpos(  "site_end_effector" )

    # The end-effector position for the final position
    xEEf = xEEi + np.array( [ 0.0, -1.0, 0.0 ] )

    # Define a Task-space controller 1
    ctrl  = CartesianImpedanceController( my_sim, args, name = "task_imp" )

    # Define an impedance controller for obstacle avoidance
    ctrl2 = CartesianImpedanceControllerObstacle( my_sim, args, name = "task_imp2", obs_pos = np.array( [ 0, 1.0, 0 ] ) )

    ctrl.add_mov_pars( x0i = xEEi, x0f = xEEf, D = 8, ti = args.start_time  )    
    ctrl.set_impedance( Kx = 300 * np.eye( 3 ), Bx = 30 * np.eye( 3 ) )

    # Add the controller and objective of the simulation
    my_sim.add_ctrl( ctrl )
    my_sim.add_ctrl( ctrl2 )

    # Run the simulation
    my_sim.run( )

    if args.is_save_data:  ctrl.export_data( my_sim.tmp_dir )

    my_sim.close( )
