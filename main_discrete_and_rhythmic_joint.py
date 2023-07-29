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
from controllers  import JointImpedanceController, DMPJointController2DOF
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
    ctrl = JointImpedanceController( my_sim, args, name = "joint_imp" )

    # Setting the joint-space impedances
    ctrl.set_impedance( Kq = np.diag( [ 300, 100 ] ), Bq = np.diag( [ 300, 30 ] ) )
    n = my_sim.n_act

    # The movement arrays of the postures
    mov_arrs  = np.array(  [  0.5, 0.5, 1.5, 1.5, 1. ] )     

    # Adding the reference trajectories
    ctrl.add_mov_pars( q0i =  np.zeros( n ), q0f = mov_arrs[ n:2*n ] - mov_arrs[  :n   ], D = mov_arrs[ -1 ], ti = 4 * 0 + 3.5 )    
    ctrl.add_mov_pars( q0i =  np.zeros( n ), q0f = mov_arrs[  :n   ] - mov_arrs[ n:2*n ], D = mov_arrs[ -1 ], ti = 4 * 1 + 3.5 + 1 )        
    ctrl.add_mov_pars( q0i =  np.zeros( n ), q0f = mov_arrs[ n:2*n ] - mov_arrs[  :n   ], D = mov_arrs[ -1 ], ti = 4 * 2 + 3.5 + 2 )        
    ctrl.add_mov_pars( q0i =  np.zeros( n ), q0f = mov_arrs[  :n   ] - mov_arrs[ n:2*n ], D = mov_arrs[ -1 ], ti = 4 * 3 + 3.5 + 3 )        
    ctrl.add_mov_pars( q0i =  np.zeros( n ), q0f = mov_arrs[ n:2*n ] - mov_arrs[  :n   ], D = mov_arrs[ -1 ], ti = 4 * 4 + 3.5 + 4 )        
    

    # Add the controller and objective of the simulation
    my_sim.add_ctrl( ctrl )

    q_init  = np.array( [ 0.5, 0.5 ] )
    qa_init = np.array( [ 0.1, 0.3 ] )
    omega0  = 1 * np.pi
    dq_init = omega0 * qa_init 
    
    init_cond = { "qpos": q_init ,  "qvel": dq_init }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )
    # Superimposing a rhythmic movement
    ctrl.add_rhythmic_mov(  amp = qa_init, offset = np.zeros( 2 ), offset2 = q_init, omega = omega0  )


    init_cond = { "qpos": mov_arrs[ :n ] ,  "qvel": np.zeros( n ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Run the simulation
    my_sim.run( )

    if args.is_save_data:  ctrl.export_data( my_sim.tmp_dir )

    my_sim.close( )

def run_movement_primitives( my_sim ):

    # Generate DMP for rhythmic and goal-directed 
    args = my_sim.args

    # Set movement parameters
    mov_arrs  = np.array(  [  0.5, 0.5, 1.5, 1.5, 1. ] )      

    # Set the initial condition of the movements
    n = my_sim.n_act

    # Define the canonical system
    cs   = CanonicalSystem( mov_type = "rhythmic" )
    cs_g = CanonicalSystem( mov_type = "discrete" )

    # The number of basis functions
    N = 50
    dmp1   = DynamicMovementPrimitives( mov_type = "rhythmic", name = "dmp_rhythmic", cs = cs,   alpha_z = 10, beta_z = 2.5, n_bfs = N )
    dmp2   = DynamicMovementPrimitives( mov_type = "rhythmic", name = "dmp_rhythmic", cs = cs,   alpha_z = 10, beta_z = 2.5, n_bfs = N )

    dmp_g1 = DynamicMovementPrimitives( mov_type = "discrete", name = "dmp_discrete", cs = cs_g, alpha_z = 10, beta_z = 2.5, n_bfs = N )
    dmp_g2 = DynamicMovementPrimitives( mov_type = "discrete", name = "dmp_discrete", cs = cs_g, alpha_z = 10, beta_z = 2.5, n_bfs = N )
    
    # The number of sample points for imitation learning
    P = 300

    # The angular velocity of the rhythmic movement and the period
    omega = 1 * np.pi
    Tp    = 2 * np.pi / omega

    # The time step of imitation learning
    # This is simply defined by D/P
    tmp_dt = Tp/P
    cs.tau = Tp/(2*np.pi)

    # The P samples points of p_des, dp_des, ddp_dex
    q_des   = np.zeros( ( 2, P + 1 ) )
    dq_des  = np.zeros( ( 2, P + 1 ) ) 
    ddq_des = np.zeros( ( 2, P + 1 ) )

    amp1 = 0.1
    amp2 = 0.3

    for i in range( P + 1 ):
        t = tmp_dt * i
        q_des[   0, i ] =  mov_arrs[ 0 ] + amp1 * np.sin( omega * t )             
        dq_des[  0, i ] =  amp1 * np.cos( omega * t ) * omega     
        ddq_des[ 0, i ] = -amp1 * np.sin( omega * t ) * omega ** 2

    for i in range( P + 1 ):
        t = tmp_dt * i
        q_des[   1, i ] =  mov_arrs[ 1 ] + amp2 * np.sin( omega * t )             
        dq_des[  1, i ] =  amp2 * np.cos( omega * t ) * omega     
        ddq_des[ 1, i ] = -amp2 * np.sin( omega * t ) * omega ** 2

    # We conduction imitation learning on the rhytmic trajectory 
    t_arr = tmp_dt * np.arange( P + 1 )
    dmp1.imitation_learning( t_arr, q_des[ 0, : ], dq_des[ 0, : ], ddq_des[ 0, : ] )
    dmp2.imitation_learning( t_arr, q_des[ 1, : ], dq_des[ 1, : ], ddq_des[ 1, : ] )

    # Since the best-fit weights are learned,
    # integrate the solution
    dt = my_sim.dt
    N_sim = round( args.run_time/dt ) + 1

    # The shoulder and elbow joints
    q_command   = np.zeros( ( n, N_sim ) )
    dq_command  = np.zeros( ( n, N_sim ) )
    ddq_command = np.zeros( ( n, N_sim ) )

    y_curr1 =  q_des[ 0, 0 ]
    z_curr1 = dq_des[ 0, 0 ]

    y_curr2 =  q_des[ 1, 0 ]
    z_curr2 = dq_des[ 1, 0 ]    

    # The elbow's initial posture 
    g0 = mov_arrs[ 0:2 ]

    # The initial condition of the goal dynamics
    g_curr1  = g0[ 0 ]
    g_curr2  = g0[ 1 ]

    dg_curr1 = 0
    dg_curr2 = 0

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
        f1 = dmp1.basis_functions.calc_nonlinear_forcing_term( s, dmp1.weights )
        f2 = dmp1.basis_functions.calc_nonlinear_forcing_term( s, dmp2.weights )

        # If the time-threshold is met, then conduct discrete movements. 
        if t >= t_off + 5 * (n_cnt -1) :

            # Switch the sign and amplitude for even/odd numbers
            if   np.mod( n_cnt, 2 ) == 1: 
                g0[ 0 ] += 1.0
                g0[ 1 ] += 1.0

            elif np.mod( n_cnt, 2 ) == 0: 
                g0[ 0 ] -= 1.0
                g0[ 1 ] -= 1.0

            n_cnt += 1 

        # The goal array 
        g0_arr.append( np.copy( g0 )  )
        g_arr.append( np.array([ g_curr1, g_curr2 ] ) )

        # Updating the goal dynamics
        # No forcing term 
        g_new1, dg_new1, _, _ = dmp_g1.step( g0[ 0 ], g_curr1, dg_curr1, 0, dt )
        g_curr1  =  g_new1 
        dg_curr1 = dg_new1 

        # Using the current goal location as the step for the rhythmic movement 
        y_new1, z_new1, dy1, dz1 = dmp1.step( g_curr1, y_curr1, z_curr1, f1, dt )


        # The command of the q joints 
        q_command[   0, i ] = y_new1
        dq_command[  0, i ] = dy1
        ddq_command[ 0, i ] = dz1

        y_curr1 = y_new1
        z_curr1 = z_new1

        # Updating the goal dynamics
        # No forcing term 
        g_new2, dg_new2, _, _ = dmp_g2.step( g0[ 1 ], g_curr2, dg_curr2, 0, dt )
        g_curr2  =  g_new2
        dg_curr2 = dg_new2

        # Using the current goal location as the step for the rhythmic movement 
        y_new2, z_new2, dy2, dz2 = dmp2.step( g_curr2, y_curr2, z_curr2, f2, dt )

        # The command of the q joints 
        q_command[   1, i ] = y_new2
        dq_command[  1, i ] = dy2
        ddq_command[ 1, i ] = dz2

        y_curr2 = y_new2
        z_curr2 = z_new2

    # Define the controller
    dmp_ctrl = DMPJointController2DOF( my_sim, args, name = "joint_dmp" )
    dmp_ctrl.set_traj( q_command, dq_command, ddq_command )

    init_cond = { "qpos": q_command[ :, 0 ],  "qvel": dq_command[ :, 0 ] }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Add the controller to the simulation
    my_sim.add_ctrl( dmp_ctrl )

    # Run the simulation
    my_sim.run( )

    if args.is_save_data or args.is_record_vid:  
        dmp1.save_mat_data(   my_sim.tmp_dir )
        dmp2.save_mat_data(   my_sim.tmp_dir )
        dmp_g1.save_mat_data( my_sim.tmp_dir )
        dmp_g2.save_mat_data( my_sim.tmp_dir )
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

