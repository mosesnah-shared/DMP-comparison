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
import numpy as np

# Adding local modules
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from simulation   import Simulation
from controllers  import CartesianImpedanceController, CartesianImpedanceControllerObstacle, DMPTaskController2DOF
from utils        import min_jerk_traj
from constants    import my_parser

# Adding DMP modules
sys.path.append( os.path.join( os.path.dirname(__file__), "DMPmodules" ) )

from CanonicalSystem            import CanonicalSystem 
from DynamicMovementPrimitives  import DynamicMovementPrimitives

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       

def run_movement_primitives( my_sim ):

    args = my_sim.args

    # The number of joints of the robot
    nq = my_sim.nq

    # The time-step of the actual simulation
    dt = my_sim.dt

    # Set initial condition of the simulation
    q0 = np.pi/12
    init_cond = { "qpos": np.array( [ q0, np.pi - 2*q0 - 0.02 ] ),  "qvel": np.zeros( nq ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Define the canonical system
    cs = CanonicalSystem( mov_type = "discrete" )

    # The number of basis functions
    N = 30

    dmp_list = [] 
    tmp_str  = [ "x", "y" ]
    for i in range( 2 ):
        dmp = DynamicMovementPrimitives( mov_type = "discrete", name = "dmp" + tmp_str[ i ], cs = cs, n_bfs = N, alpha_z = 10, beta_z = 2.5 )
        dmp_list.append( dmp )
        
    # The parameters of min-jerk-traj
    p0i = np.copy( my_sim.mj_data.get_site_xpos(  "site_end_effector" ) ) 
    p0f = p0i + np.array( [ 0., 1.2, 0. ] )
    o   = np.array( [ 0, 0.5 * ( p0i[ 1 ] + p0f[ 1 ] ), 0 ] )

    # The goal location
    g   = np.copy( p0f )
    D1  = 3.0

    # The time constant tau is the duration of the movement. 
    cs.tau = D1 

    # The number of sample points for imitation learning
    P = 100

    # The time step of imitation learning
    # This is simply defined by D/P
    tmp_dt = D1/P

    # The P samples points of p_des, dp_des, ddp_dex
    p_des   = np.zeros( ( 2, P + 1 ) )
    dp_des  = np.zeros( ( 2, P + 1 ) ) 
    ddp_des = np.zeros( ( 2, P + 1 ) )

    for i in range( 2 ):
        for j in range( P + 1 ):
            t = tmp_dt * j
            p_des[ i, j ], dp_des[ i, j ], ddp_des[ i, j ] = min_jerk_traj( t, 0.0, p0i[ i ], p0f[ i ], D1  )

        t_arr = tmp_dt * np.arange( P + 1 )
        dmp = dmp_list[ i ]
        dmp.imitation_learning( t_arr, p_des[ i, : ], dp_des[ i, : ], ddp_des[ i, : ] )

    # Now, we integrate this solution
    # For this, the initial and final time of the simulation is important
    N_sim = round( args.run_time/dt  ) + 1
    p_command   = np.zeros( ( 3, N_sim ) )
    dp_command  = np.zeros( ( 3, N_sim ) )
    ddp_command = np.zeros( ( 3, N_sim ) )

    # The initial conditions of the end-effector
    p_command[  :, 0 ] = np.array( [ p0i[ 0 ], p0i[ 1 ], 0 ] )
    dp_command[ :, 0 ] = np.zeros( 3 )

    # Simple substitution of the variable name
    p_curr  =  p_command[ :, 0 ]
    dp_curr = dp_command[ :, 0 ]

    for i in range( N_sim ):

        theta = np.arccos( np.inner( o - p_curr ,dp_curr  ) / ( np.linalg.norm( o - p_curr ) * np.linalg.norm( dp_curr )  )   )
        R = np.array( [ [ 0, 1, 0 ], [ -1, 0, 0 ], [0 ,0, 1 ] ] )            

        # If tmp_dp is a zero vector, then halt the simulation 
        Cp = 300 * R @ dp_curr * np.exp( -3 * theta ) if np.sum( dp_curr ) != 0 else np.zeros( 3 )

        for j in range( 2 ):

            dmp = dmp_list[ j ]
            t = dt * i

            if t <= args.start_time: 
                p_command[ j, i ] = p0i[ j ]

            else:

                # Integrate the solution 
                # Calculate the force from weights

                # Get the current canonical function value
                s = cs.get_value( t - args.start_time )

                f = dmp.basis_functions.calc_nonlinear_forcing_term( s, dmp.weights )
                f *= s * ( p0f[ j ] - p0i[ j ] ) 
                f += Cp[ j ]

                y_new, z_new, dy, dz = dmp.step( p0f[ j ], p_curr[ j  ], dp_curr[ j ], f, dt )
                p_command[   j, i ] = y_new
                dp_command[  j, i ] = dy
                ddp_command[ j, i ] = dz
                p_curr[ j ]  = y_new
                dp_curr[ j ] = z_new 

    # Define the controller
    dmp_ctrl = DMPTaskController2DOF( my_sim, args, name = "task_dmp" )
    dmp_ctrl.set_traj( p_command, dp_command, ddp_command )

    # Add the controller to the simulation
    my_sim.add_ctrl( dmp_ctrl )

    # Run the simulation
    my_sim.run( )

    if args.is_save_data or args.is_record_vid:  
        for i in range( nq ):
            dmp_list[ i ].save_mat_data( my_sim.tmp_dir )

    my_sim.close( )    


def run_motor_primitives( my_sim  ):

    # Set the initial condition of the 2DOF robot
    n = my_sim.n_act
    q1 = np.pi * 1/12
    init_cond = { "qpos": np.array( [ q1, np.pi-2*q1- 0.02] ) ,  "qvel": np.zeros( n ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # The initial/final posture of the robot
    p0i = np.copy( my_sim.mj_data.get_site_xpos(  "site_end_effector" ) ) 
    p0f = p0i + np.array( [ 0.0, 1.2, 0.0 ] )

    # The obstacle location
    o = np.array( [ 0, 0.5 * (p0i[ 1 ] + p0f[ 1 ]), 0 ] )

    # Define the Task-space controller and repeller
    ctrl1 = CartesianImpedanceController( my_sim, args, name = "task_imp" )
    ctrl1.add_mov_pars( p0i = p0i, p0f = p0f, D = 3.0, ti = args.start_time  )    
    ctrl1.set_impedance( Kp = 300 * np.eye( 3 ), Bp = 100 * np.eye( 3 ) )

    ctrl2 = CartesianImpedanceControllerObstacle( my_sim, args, name = "task_imp2", obs_pos = o )
    ctrl2.set_impedance( k = 0.0 )
    ctrl2.set_order( n = 6 )

    # Superimposing mechanical impedances
    my_sim.add_ctrl( ctrl1 )
    my_sim.add_ctrl( ctrl2 )

    # Run the simulation
    my_sim.run( )

    if args.is_save_data:  
        ctrl1.export_data( my_sim.tmp_dir )
        ctrl2.export_data( my_sim.tmp_dir )

    my_sim.close( )

if __name__ == "__main__":
                                                                                
    # Generate the parser, which is defined 
    parser = my_parser( )
    args, unknown = parser.parse_known_args( )

    args.model_name = "2DOF_planar_torque"
    my_sim = Simulation( args )

    assert args.sim_type in [    "motor", "movement" ]

    # Set the camera position of the simulation
    # Lookat [3] Distance, Elevation, Azimuth
    args.cam_pos = np.array( [ 0, 1, 0, 5, -90, 90 ] )    

    if    args.sim_type == "motor"   :    run_motor_primitives( my_sim )
    elif  args.sim_type == "movement": run_movement_primitives( my_sim )

    

