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
from InverseDynamicsModel       import get2DOF_J

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       
 
def run_motor_primitives( my_sim ):

    # Arguments of the simulation 
    args = my_sim.args

    # Define the controller 
    ctrl = CartesianImpedanceController( my_sim, args, name = "task_imp" )

    # Setting the initial conditino of the robots 
    init_cond = { "qpos": np.array( [ 1.1, 2 ] ) ,  "qvel": np.zeros( 2 ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Get the initial end-effector position, and 8 targets in total
    pi = np.copy( my_sim.mj_data.get_site_xpos(  "site_end_effector" ) ) 
    tmp_pi = np.copy( pi )
    # Add the rotational radius 
    r = 0.3
    omega0 = np.pi 

    pi += np.array( [0.0, r, 0.0])
    pf = pi + np.array( [1.0, 0, 0.0])

    # Solve the inverse kinematics 
    q2 = np.pi - np.arccos( 0.5 * ( 2 - pi[ 0 ] ** 2 - pi[ 1 ] ** 2  ) )
    q1 = np.arctan2( pi[ 1 ], pi[ 0 ] ) - q2/2 

    q_init = np.array( [ q1, q2 ] )

    # The joint velocities
    J   = get2DOF_J( q_init )
    dq_init = np.linalg.inv( J ) @ np.array( [ -r * omega0, 0 ] )

    # Setting the initial conditino of the robots 
    init_cond = { "qpos": q_init ,  "qvel": dq_init }

    # Solve the inverse kinematics and reposition

    # Setting the joint-space impedances
    ctrl.set_impedance( Kp = 300 * np.eye( 3 ), Bp = 100 * np.eye( 3 ) )
    n = my_sim.n_act

    # The movement arrays of the postures
    mov_arrs  = np.array(  [  pi[ 0 ], pi[ 1 ], pf[ 0 ], pf[ 1 ], 1.0 ] )     
    
    # Adding the reference trajectories
    ctrl.add_mov_pars( p0i = np.zeros( 3 ), p0f = pf - pi , D = mov_arrs[ -1 ], ti = 3.0  )    
    ctrl.add_mov_pars( p0i = np.zeros( 3 ), p0f = pi - pf, D = mov_arrs[ -1 ], ti = 1 + 7.0  )    
    ctrl.add_mov_pars( p0i = np.zeros( 3 ), p0f = pf - pi, D = mov_arrs[ -1 ], ti = 2 + 11.0 )    
    ctrl.add_mov_pars( p0i = np.zeros( 3 ), p0f = pi - pf, D = mov_arrs[ -1 ], ti = 3 + 15.0 )    

    # Superimposing a rhythmic movement
    ctrl.add_rhythmic_mov( amp = r, center = tmp_pi, omega = omega0, offset = np.array( [ np.pi/2, np.pi/2  ] ) )

    # Add the controller and objective of the simulation
    my_sim.add_ctrl( ctrl )

    # Run the simulation
    my_sim.run( )

    if args.is_save_data:  ctrl.export_data( my_sim.tmp_dir )

    my_sim.close( )

def run_movement_primitives( my_sim ):
    # Define the canonical system
    cs = CanonicalSystem( mov_type = "rhythmic" )

    # The number of degrees of freedom of the tobot 
    nq = my_sim.nq

    # The number of basis functions
    N = 40

    # Setting the initial conditino of the robots 
    init_cond = { "qpos": np.array( [ 1.1, 2 ] ) ,  "qvel": np.zeros( 2 ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Get the initial end-effector position, and 8 targets in total
    pi = np.copy( my_sim.mj_data.get_site_xpos(  "site_end_effector" ) ) 
    tmp_pi = np.copy( pi )

    # Dynamic Movement Primitives 
    dmp_list = [] 

    tmp_str = [ "x", "y" ]
    for i in range( 2 ): 
        dmp = DynamicMovementPrimitives( mov_type = "rhythmic", name = "dmp_rhythmic" + tmp_str[ i ], cs = cs, n_bfs = N, alpha_z = 10, beta_z = 2.5 )
        dmp_list.append( dmp )

    # For rhythmic movement 
    # The x and y trajectory 
    r, omega0, c = 0.3, np.pi, tmp_pi

    # The period of the system
    Tp = 2 * np.pi / omega0 
    P  = 100
    dt = Tp/P    
    t_des  = dt * np.arange( P + 1 )
    cs.tau = Tp/(2*np.pi)

    # The P samples points of p_des, dp_des, ddp_dex
    p_des   = np.zeros( ( 2, P + 1 ) )
    dp_des  = np.zeros( ( 2, P + 1 ) ) 
    ddp_des = np.zeros( ( 2, P + 1 ) )

    for i, t in enumerate( t_des ):
        p_des[   0,i ] = - r * np.sin( omega0 * t ) + c[ 0 ]
        dp_des[  0,i ] = - r * omega0 * np.cos( omega0 * t )
        ddp_des[ 0,i ] =   r * ( omega0 ** 2 ) * np.sin( omega0 * t )

        p_des[   1,i ] =   r * np.cos( omega0 * t ) + c[ 1 ]
        dp_des[  1,i ] = - r * omega0 * np.sin( omega0 * t )
        ddp_des[ 1,i ] = - r * ( omega0 ** 2 ) * np.cos( omega0 * t )

    for i in range( 2 ):
        dmp = dmp_list[ i ]
        dmp.imitation_learning( t_des, p_des[ i, : ], dp_des[ i, : ], ddp_des[ i, : ] )

    # Now, we integrate this solution
    # For this, the initial and final time of the simulation is important
    dt = my_sim.dt    
    N_sim = round( args.run_time/dt ) + 1

    # The q, dq, ddq of the robot 
    p_command   = np.zeros( ( 2, N_sim ) )
    dp_command  = np.zeros( ( 2, N_sim ) )
    ddp_command = np.zeros( ( 2, N_sim ) )


    # Now the discrete movement
    cs_g = CanonicalSystem( mov_type = "discrete" )
    dmp_g = DynamicMovementPrimitives( mov_type = "discrete", name = "dmp_discrete", cs = cs_g, alpha_z = 10, beta_z = 2.5, n_bfs = N )

    # The initial x position
    g0 = tmp_pi[ 0 ]

    # The initial condition of the goal dynamics
    g_curr  = g0
    dg_curr = 0

    y_curr1 = tmp_pi[ 0 ]
    z_curr1 = -r * omega0

    y_curr2 = tmp_pi[ 1 ]
    z_curr2 = 0 

    g0_arr = []   
    g_arr  = []

    t_off = 3.0
    n_cnt = 1

    for i in range( N_sim ):

        t = dt * i

        # Get the current canonical function value
        s = cs.get_value( t )
        f1 = dmp_list[ 0 ].basis_functions.calc_nonlinear_forcing_term( s, dmp_list[ 0 ].weights )
        f2 = dmp_list[ 1 ].basis_functions.calc_nonlinear_forcing_term( s, dmp_list[ 1 ].weights )

        # If the time-threshold is met, then conduct discrete movements. 
        if t >= 3 + 5 * (n_cnt -1) :

            # Switch the sign and amplitude for even/odd numbers
            if   np.mod( n_cnt, 2 ) == 1: 
                g0 += 1.0

            elif np.mod( n_cnt, 2 ) == 0: 
                g0 -= 1.0

            n_cnt += 1 

        # The goal array 
        g0_arr.append( np.copy( g0 )  )
        g_arr.append( np.copy( g_curr ) )

        # Updating the goal dynamics
        # No forcing term 
        g_new, dg_new, _, _ = dmp_g.step( g0, g_curr, dg_curr, 0, dt )
        g_curr  =  g_new 
        dg_curr = dg_new

        # Using the current goal location as the step for the rhythmic movement 
        y_new1, z_new1, dy1, dz1 = dmp_list[ 0 ].step( g_curr, y_curr1, z_curr1, f1, dt )

        # The command of the q joints 
        p_command[   0, i ] = y_new1
        dp_command[  0, i ] = dy1
        ddp_command[ 0, i ] = dz1

        y_curr1 = y_new1
        z_curr1 = z_new1

        # Updating the goal dynamics
        # No forcing term 


        # Using the current goal location as the step for the rhythmic movement 
        y_new2, z_new2, dy2, dz2 = dmp_list[ 1 ].step( tmp_pi[ 1 ], y_curr2, z_curr2, f2, dt )

        # The command of the q joints 
        p_command[   1, i ] = y_new2
        dp_command[  1, i ] = dy2
        ddp_command[ 1, i ] = dz2

        y_curr2 = y_new2
        z_curr2 = z_new2

    px = np.copy( p_command[ 0, 0 ])
    py = np.copy( p_command[ 1, 0 ])

    # Solve the inverse kinematics 
    q2 = np.pi - np.arccos( 0.5 * ( 2 - px ** 2 - py ** 2  ) )
    q1 = np.arctan2( py, px ) - q2/2 

    q_init = np.array( [ q1, q2 ] )

    # The joint velocities
    J   = get2DOF_J( q_init )
    dq_init = np.linalg.inv( J ) @ dp_command[ :, 0 ]

    init_cond = { "qpos": q_init ,  "qvel": dq_init }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )


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

