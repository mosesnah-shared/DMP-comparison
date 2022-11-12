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
from datetime  import datetime
from matplotlib  import pyplot as plt

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

def run_motor_primitives(  ):

    args.model_name = "2DOF_planar_torque"
    my_sim = Simulation( args )

    # Define the controller 
    ctrl1 = CartesianImpedanceController( my_sim, args, name = "task_imp" )
    ctrl1.set_impedance( Kx = 300 * np.eye( 3 ), Bx = 100 * np.eye( 3 ) )

    my_sim.add_ctrl( ctrl1 )

    # Set initial position
    q0 = np.pi/6

    init_cond = { "qpos": np.array( [ q0, np.pi - q0 ] ),  "qvel": np.zeros( 2 ) }

    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Get the initial position of the robot 
    xEEi = my_sim.mj_data.get_site_xpos( "site_end_effector" )
        
    # Define the final posture 
    xEEf = xEEi + np.array( [ 0.0, 0.5, 0 ] )    

    # Now, we assume there is 
    # We run a manual loop here

    # Run the simulation
    my_sim.run( )

    if args.is_save_data:  
        ctrl1.export_data( my_sim.tmp_dir )

    my_sim.close( )


def run_movement_primitives(  ):

    cs = CanonicalSystem( mov_type = "discrete" )

    # Dynamic Movement Primitives
    # First  along the x-axis
    # Second along the y-axis
    dmp1 = DynamicMovementPrimitives( mov_type = "discrete", alpha_z = 10, beta_z = 2.5 )
    dmp2 = DynamicMovementPrimitives( mov_type = "discrete", alpha_z = 10, beta_z = 2.5 )

    # Adding the canonical system
    # DMPs share the same canonical system 
    dmp1.add_canonical_system( cs )
    dmp2.add_canonical_system( cs )

    # ADD CODE
    NotImplementedError( )
            

if __name__ == "__main__":

    # Generate an instance of our Simulation
    # The model is generated since the model name is passed via arguments

    mov_type = "rhythmic"
    ctrl_type = "motor"

    if    ctrl_type == "motor"   :    run_motor_primitives(  )
    elif  ctrl_type == "movement": run_movement_primitives(  )

    

