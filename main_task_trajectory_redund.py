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
import shutil
import scipy.io

import numpy             as np
import matplotlib.pyplot as plt
import moviepy.editor    as mpy




# The Local Modules
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from simulation   import Simulation
from controllers  import CartesianImpedanceController, JointImpedanceController, DMPTaskController5DOF
from utils        import min_jerk_traj
from constants    import my_parser
from constants    import Constants as C

# Modules for DMP
sys.path.append( os.path.join( os.path.dirname(__file__), "DMPmodules" ) )

from CanonicalSystem            import CanonicalSystem 
from DynamicMovementPrimitives  import DynamicMovementPrimitives
from InverseDynamicsModel       import get5DOF_C, get5DOF_dJ

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       


def run_motor_primitives( my_sim ):

    # Define the impedances
    ctrl1 = CartesianImpedanceController( my_sim, args, name =  "task_imp" )
    ctrl2 =     JointImpedanceController( my_sim, args, name = "joint_imp" )

    # The values of the impedances 
    ctrl1.set_impedance( Kx = 300 * np.eye( 3 ), Bx = 100 * np.eye( 3 ) )
    ctrl2.set_impedance( Kq =   0 * np.eye( n ), Bq =  30 * np.eye( n ) )

    # Superposition of mechanical impedances
    my_sim.add_ctrl( ctrl1 )
    my_sim.add_ctrl( ctrl2 )

    # The number of actuators
    n = my_sim.n_act
    
    # Set the initial posture of the high-DOF robot
    q1      = 0
    ref_pos = np.array( [ q1, np.pi/2 - q1, 0, 0, np.pi/2-q1 ] )
    init_cond = { "qpos": ref_pos ,  "qvel": np.zeros( n ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Get the initial/final end-effector position of the robot 
    p0i = my_sim.mj_data.get_site_xpos( "site_end_effector" )
    p0f = p0i + np.array( [ 3.0, 0, 0 ] )

    # Adding the reference trajectories
    # For the joint impedances, the reference posture remains stationary.
    ctrl1.add_mov_pars( x0i = p0i, x0f = p0f, D = 2, ti = args.start_time  )    
    ctrl2.add_mov_pars( q0i = ref_pos, q0f = ref_pos, D = 2, ti = 0 )    

    # Run the simulation
    my_sim.run( )

    if args.is_save_data:  
        ctrl1.export_data( my_sim.tmp_dir )
        ctrl2.export_data( my_sim.tmp_dir )

    my_sim.close( )


def run_movement_primitives( my_sim ):

    # Define the canonical system
    cs = CanonicalSystem( mov_type = "discrete" )

    # The number of degrees of freedom of the tobot 
    nq = my_sim.nq

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
        
    # Set the initial posture of the robot
    q1 = 0
    ref_pos = np.array( [ q1, np.pi/2 - q1, 0, 0, np.pi/2-q1 ] )
    init_cond = { "qpos": ref_pos ,  "qvel": np.zeros( len( ref_pos ) ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Get the initial/final end-effector position 
    p0i = my_sim.mj_data.get_site_xpos( "site_end_effector" )
    p0f = p0i + np.array( [ 3.0, 0, 0 ] )
    D   = 2.0       

    # The time constant tau is the duration of the movement. 
    cs.tau = D        

    # The number of sample points for imitation learning and the time-step
    P      = 100
    tmp_dt = D/P

    # The P samples points of p_des, dp_des, ddp_dex
    p_des   = np.zeros( ( 3, P + 1 ) )
    dp_des  = np.zeros( ( 3, P + 1 ) ) 
    ddp_des = np.zeros( ( 3, P + 1 ) )

    for i in range( 2 ):
        for j in range( P + 1 ):
            t = tmp_dt * j
            p_des[ i, j ], dp_des[ i, j ], ddp_des[ i, j ] = min_jerk_traj( t, 0.0, p0i[ i ], p0f[ i ], D  )

        t_arr = tmp_dt * np.arange( P + 1 )
        dmp = dmp_list[ i ]
        dmp.imitation_learning( t_arr, p_des[ i, : ], dp_des[ i, : ], ddp_des[ i, : ] )

    # Now, we integrate this solution
    # For this, the initial and final time of the simulation is important
    N_sim = round( args.run_time/dt ) + 1

    p_command   = np.zeros( ( 3, N_sim ) )
    dp_command  = np.zeros( ( 3, N_sim ) )
    ddp_command = np.zeros( ( 3, N_sim ) )

    # Iterating through the dmps
    for i in range( 2 ):

        dmp = dmp_list[ i ]

        # y, z, dy, dz
        t_arr, y_arr, z_arr, dy_arr, dz_arr = dmp.integrate( p0i[ i ], 0, p0f[ i ], dt, args.start_time, N_sim )

        p_command[   i, : ] =   y_arr
        dp_command[  i, : ] =  dy_arr
        ddp_command[ i, : ] =  dz_arr 

    # Define the controller
    dmp_ctrl = DMPTaskController5DOF( my_sim, args, name = "task_dmp" )
    dmp_ctrl.set_traj( p_command, dp_command, ddp_command )

    # Add the controller to the simulation
    my_sim.add_ctrl( dmp_ctrl )

    # Run the simulation
    my_sim.run( )

    if args.is_save_data or args.is_record_vid:  
        for i in range( nq ):
            dmp_list[ i ].save_mat_data( my_sim.tmp_dir )

    my_sim.close( )

if __name__ == "__main__":


    ctrl_type = "movement"
                                                                                
    # Generate the parser, which is defined 
    parser = my_parser( )
    args, unknown = parser.parse_known_args( )

    args.model_name = "5DOF_planar_torque"
    my_sim = Simulation( args )

    assert ctrl_type in [    "motor", "movement" ]

    # Define the robot that we will use 
    args.model_name = "5DOF_planar_torque"

    # Set the camera position of the simulation
    # Lookat [3] Distance, Elevation, Azimuth
    args.cam_pos = np.array( [ 2.0, 2.0, 0, 9, -90, 90 ] )    

    if    ctrl_type == "motor"   :    run_motor_primitives( my_sim )
    elif  ctrl_type == "movement": run_movement_primitives( my_sim )

    

