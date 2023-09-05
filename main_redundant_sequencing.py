"""

# ============================================================================= #
| Project:        [M3X Whip Project]
| Title:          Dynamic Motor Primitives, with Kinematic Redundancy
|                 Section 3.4 
| Author:         Moses C. Nah
| Email:          [Moses] mosesnah@mit.edu
# ============================================================================= #

"""

import os
import sys
import numpy as np

# The Local Modules
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from simulation   import Simulation
from controllers  import CartesianImpedanceController, JointImpedanceController, DMPTaskController5DOF
from utils        import min_jerk_traj
from constants    import my_parser

# Modules for DMP
sys.path.append( os.path.join( os.path.dirname(__file__), "DMPmodules" ) )

from CanonicalSystem            import CanonicalSystem 
from DynamicMovementPrimitives  import DynamicMovementPrimitives

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       

def run_motor_primitives( my_sim ):

    # The number of actuators
    n = my_sim.n_act

    # Define the impedances
    ctrl1 = CartesianImpedanceController( my_sim, args, name =  "task_imp" )
    ctrl2 =     JointImpedanceController( my_sim, args, name = "joint_imp" )

    # The values of the impedances 
    ctrl1.set_impedance( Kp = 300 * np.eye( 3 ), Bp = 100 * np.eye( 3 ) )
    ctrl2.set_impedance( Kq =   0 * np.eye( n ), Bq =  30 * np.eye( n ) )

    # Superposition of mechanical impedances
    my_sim.add_ctrl( ctrl1 )
    my_sim.add_ctrl( ctrl2 )
    
    # Set the initial posture of the high-DOF robot
    ref_pos = np.ones( 5 ) * 0.9
    init_cond = { "qpos": ref_pos ,  "qvel": np.zeros( n ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Get the initial/final end-effector position of the robot 
    p0i = np.copy( my_sim.mj_data.get_site_xpos( "site_end_effector" ) )
    p0f = p0i + np.array( [ -2.0, 1.0, 0. ] )

    # Adding the reference trajectories
    # For the joint impedances, the reference posture remains stationary.

    ctrl2.add_mov_pars( q0i = np.zeros( 5 ), q0f = np.zeros( 5 ), D = 2, ti = 0 )    

    # The movement durations of the submovements
    D1, D2 = 2.0, 3.0

    # The new goal position 
    # We assume that at some specific time, a new goal location
    # appears, and therefore we are superimposing a new movement onto it.
    # The new goal position 
    g_new = p0f + np.array( [ 6, 1.5, 0. ] )

    ctrl1.add_mov_pars( p0i =           p0i, p0f =         p0f, D = D1, ti = args.start_time        )    
    ctrl1.add_mov_pars( p0i = np.zeros( 3 ), p0f = g_new - p0f, D = D2, ti = args.start_time + D1/2 )    

    # Run the simulation
    my_sim.run( )

    if args.is_save_data:  
        ctrl1.export_data( my_sim.tmp_dir )
        ctrl2.export_data( my_sim.tmp_dir )

    my_sim.close( )


def run_movement_primitives( my_sim ):

    args = my_sim.args


    # The number of degrees of freedom of the tobot 
    nq = my_sim.nq

    # Define the canonical system
    cs = CanonicalSystem( mov_type = "discrete" )

    # Set initial position
    ref_pos = np.ones( 5 ) * 0.9
    init_cond = { "qpos": ref_pos ,  "qvel": np.zeros( 5 ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # The time step of the simulation 
    dt = my_sim.dt

    # Dynamic Movement Primitives 
    # Define for x and y trajectory
    dmp_list = [] 

    # The number of basis functions
    N = 20

    tmp_str = [ "x", "y" ]
    for i in range( 2 ):
        dmp = DynamicMovementPrimitives( mov_type = "discrete", name = "dmp" + tmp_str[ i ], cs = cs, n_bfs = N, alpha_z = 10, beta_z = 2.5 )
        dmp_list.append( dmp )

    # The parameters of min-jerk-traj
    p0i = np.copy( my_sim.mj_data.get_site_xpos(  "site_end_effector" ) ) 
    p0f = p0i  + np.array( [ -2.0, 1.0, 0. ] )
    D1  = 2.0
    cs.tau = D1 

    # For this, the goal is changing.
    g_old = np.copy( p0f )
    g_new = p0f + np.array( [ 6, 1.5, 0. ] )
    
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

    # Usually we have a separate function "integrate"
    # However, for this example the goal position changes, hence manually coding.
    for i in range( 2 ):

        dmp = dmp_list[ i ]

        y_curr = p0i[ i ]
        z_curr = 0

        for j in range( N_sim ):
            t = dt * j 

            if t <= args.start_time: 
                p_command[   i, j ] = p0i[ i ]
                dp_command[  i, j ] = 0
                ddp_command[ i, j ] = 0

            else:

                # Get the current canonical function value
                s = cs.get_value( t - args.start_time )
                f = dmp.basis_functions.calc_nonlinear_forcing_term( s, dmp.weights )
                f *= s * ( p0f[ i ] - p0i[ i ] ) 

                # If the new goal appear:
                if t >= args.start_time + D1/2:
                    g = g_new + ( g_old - g_new ) * np.exp( -cs.tau * ( t - ( args.start_time + D1/2 ) ) )

                else:
                    g = g_old                    

                y_new, z_new, dy, dz = dmp.step( g[ i ], y_curr, z_curr, f, dt )
                p_command[   i, j ] = y_new
                dp_command[  i, j ] = dy
                ddp_command[ i, j ] = dz
                y_curr = y_new
                z_curr = z_new 

    # Define the controller
    dmp_ctrl = DMPTaskController5DOF( my_sim, args, name = "task_dmp" )
    dmp_ctrl.set_traj( p_command, dp_command, ddp_command )

    # Add the controller to the simulation
    my_sim.add_ctrl( dmp_ctrl )

    # Run the simulation
    my_sim.run( )

    if args.is_save_data or args.is_record_vid:  
        dmp_list[ 0 ].save_mat_data( my_sim.tmp_dir )
        dmp_list[ 1 ].save_mat_data( my_sim.tmp_dir )

    my_sim.close( )    

if __name__ == "__main__":
                                                                                
    # Generate the parser, which is defined 
    parser = my_parser( )
    args, unknown = parser.parse_known_args( )

    args.model_name = "5DOF_planar_torque"
    my_sim = Simulation( args )

    assert args.sim_type in [    "motor", "movement" ]

    # Define the robot that we will use 
    args.model_name = "5DOF_planar_torque"

    # Set the camera position of the simulation
    # Lookat [3] Distance, Elevation, Azimuth
    args.cam_pos = np.array( [ -1.0, 2.0, 0, 9, -90, 90 ] )    

    if    args.sim_type == "motor"   :    run_motor_primitives( my_sim )
    elif  args.sim_type == "movement": run_movement_primitives( my_sim )


