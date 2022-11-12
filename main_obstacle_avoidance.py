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
import matplotlib.pyplot as plt

# To Add Local Files, adding the directory via sys module
# __file__ saves the current directory of this file. 
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from simulation   import Simulation
from controllers  import CartesianImpedanceController, CartesianImpedanceControllerObstacle
from constants    import my_parser

sys.path.append( os.path.join( os.path.dirname(__file__), "DMPmodules" ) )

from CanonicalSystem            import CanonicalSystem 
from DynamicMovementPrimitives  import DynamicMovementPrimitives

np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       
parser = my_parser( )
args, unknown = parser.parse_known_args( )

def run_movement_primitives( my_sim ):

    cs = CanonicalSystem( mov_type = "discrete" )

    # Add the movements of the Cartesian Impedance Controller 
    # Set the initial posture 
    n = my_sim.n_act
    q1 = np.pi * 1/12
    init_cond = { "qpos": np.array( [ q1, np.pi-2*q1- 0.02] ) ,  "qvel": np.zeros( n ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    xEEi = np.copy( my_sim.mj_data.get_site_xpos(  "site_end_effector" ) ) 

    # The end-effector position for the final position
    xEEf = xEEi + np.array( [ 0.0, 1.0, 0.0 ] )
    g = np.copy( xEEf )
    dmp1 = DynamicMovementPrimitives( mov_type = "discrete", alpha_z = 10, beta_z = 2.5 )
    dmp2 = DynamicMovementPrimitives( mov_type = "discrete", alpha_z = 10, beta_z = 2.5 )

    # Adding the canonical system
    # DMPs share the same canonical system 
    dmp1.add_canonical_system( cs )
    dmp2.add_canonical_system( cs )

    # Redefine the new time step 
    dt  = my_sim.mj_model.opt.timestep      
    t   = 0
    tau = 1
    T   = args.run_time 

    x_arr = np.zeros( round( T/dt ) + 1 )
    y_arr = np.zeros( round( T/dt ) + 1 )

    idx = 0 

    xold ,  yold = xEEi[ 0 ], xEEi[ 1 ]
    dxold, dyold =         0,         0    

    # BEST TO HAVE A FORCE TERM    

    while t <= T + 1e-7:
        
        xnew, dxnew = dmp1.step( g[ 0 ], xold, dxold, 0, dt )
        ynew, dynew = dmp2.step( g[ 1 ], yold, dyold, 0, dt )

        x_arr[ idx ] = xnew
        y_arr[ idx ] = ynew

        xold, dxold = xnew, dxnew
        yold, dyold = ynew, dynew

        idx += 1
        t += dt

    plt.plot( x_arr )
    plt.plot( y_arr )
    plt.show( )



def run_motor_primitives( my_sim  ):

    # Add the movements of the Cartesian Impedance Controller 
    # Set the initial posture 
    n = my_sim.n_act
    q1 = np.pi * 1/12
    init_cond = { "qpos": np.array( [ q1, np.pi-2*q1- 0.02] ) ,  "qvel": np.zeros( n ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    xEEi = np.copy( my_sim.mj_data.get_site_xpos(  "site_end_effector" ) ) 

    # The end-effector position for the final position
    xEEf = xEEi + np.array( [ 0.0, 1.0, 0.0 ] )

    # Define a Task-space controller 1
    ctrl  = CartesianImpedanceController( my_sim, args, name = "task_imp" )

    # Define an impedance controller for obstacle avoidance
    ctrl2 = CartesianImpedanceControllerObstacle( my_sim, args, name = "task_imp2", obs_pos = np.array( [ 0, 0.5 * (xEEi[ 1 ] + xEEf[ 1 ]), 0  ])   )
    ctrl2.set_impedance( k = args.k )
    ctrl2.set_order( n = args.order )

    ctrl.add_mov_pars( x0i = xEEi, x0f = xEEf, D = 8, ti = args.start_time  )    
    ctrl.set_impedance( Kx = 300 * np.eye( 3 ), Bx = 30 * np.eye( 3 ) )

    # Add the controller and objective of the simulation
    my_sim.add_ctrl( ctrl )
    my_sim.add_ctrl( ctrl2 )

    # Run the simulation
    my_sim.run( )

    if args.is_save_data:  ctrl.export_data( my_sim.tmp_dir )

    my_sim.close( )

if __name__ == "__main__":

    # Generate an instance of our Simulation
    # The model is generated since the model name is passed via arguments
    ctrl_type = "movement"

    # Generate an instance of our Simulation
    # We use a 2-DOF robot arm with obstacle geometry
    args.model_name = "2DOF_planar_torque"

    my_sim = Simulation( args )

    if    ctrl_type == "motor"   :    run_motor_primitives( my_sim )
    elif  ctrl_type == "movement": run_movement_primitives( my_sim )

    

