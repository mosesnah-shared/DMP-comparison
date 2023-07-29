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
import scipy.io
import numpy     as np

# Adding local modules
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from simulation   import Simulation
from controllers  import CartesianImpedanceController, DMPTaskController2DOF
from constants    import my_parser

# Adding DMP modules
sys.path.append( os.path.join( os.path.dirname(__file__), "DMPmodules" ) )

from CanonicalSystem            import CanonicalSystem 
from DynamicMovementPrimitives  import DynamicMovementPrimitives

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       
 
def run_motor_primitives( my_sim ):

    # Arguments of the simulation 
    args = my_sim.args

    # Define the controller 
    ctrl = CartesianImpedanceController( my_sim, args, name = "joint_imp" )

    # Setting the initial conditino of the robots
    q1 = np.pi * 1/12
    init_cond = { "qpos": np.array( [ q1, np.pi-2*q1 ] ) ,  "qvel": np.zeros( 2 ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Get the initial end-effector position, and 8 targets in total
    pi = np.copy( my_sim.mj_data.get_site_xpos(  "site_end_effector" ) ) 
    pf = pi + np.array( [ 0.0, 1.2, 0.0 ] )

    # Setting the joint-space impedances
    ctrl.set_impedance( Kp = 300 * np.eye( 3 ), Bp = 100 * np.eye( 3 ) )
    n = my_sim.n_act

    # The movement arrays of the postures
    mov_arrs  = np.array(  [  pi[ 0 ], pi[ 1 ], pf[ 0 ], pf[ 1 ], 3.0 ] )     
    
    t_off = 3.5

    # Adding the reference trajectories
    ctrl.add_mov_pars( p0i = pi, p0f = pf , D = mov_arrs[ -1 ], ti = t_off + 1  )    
    ctrl.add_mov_pars( p0i = np.zeros( 3 ), p0f = pi - pf, D = mov_arrs[ -1 ], ti = 2 * t_off + 1  )    
    ctrl.add_mov_pars( p0i = np.zeros( 3 ), p0f = pf - pi, D = mov_arrs[ -1 ], ti = 3 * t_off + 1  )    
    ctrl.add_mov_pars( p0i = np.zeros( 3 ), p0f = pi - pf, D = mov_arrs[ -1 ], ti = 4 * t_off + 1  )    

    # Superimposing a rhythmic movement
    # ctrl.add_rhythmic_mov( amp = np.array( [ 0., 0.3 ] ), offset = np.array( [ 0., 0. ] ), omega = 2 * np.pi  )    

    # Add the controller and objective of the simulation
    my_sim.add_ctrl( ctrl )

    # Run the simulation
    my_sim.run( )

    if args.is_save_data:  ctrl.export_data( my_sim.tmp_dir )

    my_sim.close( )

def run_movement_primitives( my_sim ):

    # Generate DMP for rhythmic and goal-directed 
    args = my_sim.args

    # Set movement parameters
    mov_arrs  = np.array(  [  0.6, 0.5, 0.6, 1.5, 1. ] )     

    # Set the initial condition of the movements
    n = my_sim.n_act


    # Define the canonical system
    cs   = CanonicalSystem( mov_type = "rhythmic" )
    cs_g = CanonicalSystem( mov_type = "discrete" )

    # The number of basis functions
    N = 50
    dmp   = DynamicMovementPrimitives( mov_type = "rhythmic", name = "dmp_rhythmic", cs = cs,   alpha_z = 10, beta_z = 2.5, n_bfs = N )
    dmp_g = DynamicMovementPrimitives( mov_type = "discrete", name = "dmp_discrete", cs = cs_g, alpha_z = 10, beta_z = 2.5, n_bfs = N )


    # The number of sample points for imitation learning
    P = 100

    # The angular velocity of the rhythmic movement and the period
    omega = 2 * np.pi
    Tp    = 2 * np.pi / omega

    # The time step of imitation learning
    # This is simply defined by D/P
    tmp_dt = Tp/P
    cs.tau = Tp/(2*np.pi)

    # The P samples points of p_des, dp_des, ddp_dex
    q_des   = np.zeros( P + 1 )
    dq_des  = np.zeros( P + 1 ) 
    ddq_des = np.zeros( P + 1 )

    amp = 0.3

    for i in range( P + 1 ):
        t = tmp_dt * i
        q_des[   i ] =  mov_arrs[ 1 ] + amp * np.sin( omega * t )             
        dq_des[  i ] =  amp * np.cos( omega * t ) * omega     
        ddq_des[ i ] = -amp * np.sin( omega * t ) * omega ** 2

    # We conduction imitation learning on the rhytmic trajectory 
    t_arr = tmp_dt * np.arange( P + 1 )
    dmp.imitation_learning( t_arr, q_des[ : ], dq_des[ : ], ddq_des[ : ] )

    # Since the best-fit weights are learned,
    # integrate the solution
    dt = my_sim.dt
    N_sim = round( args.run_time/dt ) + 1

    # The shoulder and elbow joints
    q_command   = np.zeros( ( n, N_sim ) )
    dq_command  = np.zeros( ( n, N_sim ) )
    ddq_command = np.zeros( ( n, N_sim ) )

    y_curr =  q_des[ 0 ]
    z_curr = dq_des[ 0 ]

    # The elbow's initial posture 
    g0 = mov_arrs[ 1 ]

    # The initial condition of the goal dynamics
    g_curr  = g0
    dg_curr = 0 

    t_off = 3.5
    n_cnt = 1

    # g0: The goal desired position 
    # The actual goal dynamics
    # For saving the details 
    g0_arr = []
    g_arr  = []

    for i in range( N_sim ):

        t = dt * i

        # Get the current canonical function value
        s = cs.get_value( t )
        f = dmp.basis_functions.calc_nonlinear_forcing_term( s, dmp.weights )

        # If the time-threshold is met, then conduct discrete movements. 
        if t >= t_off * n_cnt:

            # Switch the sign and amplitude for even/odd numbers
            if   np.mod( n_cnt, 2 ) == 1: g0 += 1.0
            elif np.mod( n_cnt, 2 ) == 0: g0 -= 1.0

            n_cnt += 1 

        # The goal array 
        g0_arr.append( g0     )
        g_arr.append(  g_curr )

        # Updating the goal dynamics
        # No forcing term 
        g_new, dg_new, _, _ = dmp_g.step( g0, g_curr, dg_curr, 0, dt )
        g_curr  =  g_new 
        dg_curr = dg_new 

        # Using the current goal location as the step for the rhythmic movement 
        y_new, z_new, dy, dz = dmp.step( g_curr, y_curr, z_curr, f, dt )

        # The command of the q joints 
        q_command[   1, i ] = y_new
        dq_command[  1, i ] = dy
        ddq_command[ 1, i ] = dz

        y_curr = y_new
        z_curr = z_new 

    # Define the controller
    dmp_ctrl = DMPJointController2DOF( my_sim, args, name = "joint_dmp" )
    dmp_ctrl.set_traj( q_command, dq_command, ddq_command )

    init_cond = { "qpos": mov_arrs[ :n ],  "qvel": dq_command[ :, 0 ] }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Add the controller to the simulation
    my_sim.add_ctrl( dmp_ctrl )

    # Run the simulation
    my_sim.run( )

    if args.is_save_data or args.is_record_vid:  
        dmp.save_mat_data(   my_sim.tmp_dir )
        dmp_g.save_mat_data( my_sim.tmp_dir )
        scipy.io.savemat( my_sim.tmp_dir + "/dmp_goal_details.mat", { "g0": g0_arr, "g": g_arr } )   

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

