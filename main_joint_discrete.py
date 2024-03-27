"""

# ============================================================================= #
| Project:        [M3X Whip Project]
| Title:          Dynamic Motor Primitives, Joint-space trajectory planning.
|                 Section 3.2 
| Author:         Moses C. Nah
| Email:          [Moses] mosesnah@mit.edu
# ============================================================================= #

"""

import os
import sys
import numpy           as np

# Adding local modules
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from simulation   import Simulation
from controllers  import JointImpedanceController, DMPJointController2DOF
from utils        import min_jerk_traj
from constants    import my_parser

# The DMP Modules
sys.path.append( os.path.join( os.path.dirname(__file__), "DMPmodules" ) )

from CanonicalSystem            import CanonicalSystem 
from DynamicMovementPrimitives  import DynamicMovementPrimitives

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       

# Dynamic Motor Primitives
def run_motor_primitives( my_sim ):

    # Define the joint-space impedance controller 
    ctrl = JointImpedanceController( my_sim, args, name = "joint_imp" )

    # Get the number of actuators of the robot 
    nq = ctrl.nq

    # The joint stiffness and damping matrices
    ctrl.set_impedance( Kq = 150 * np.eye( nq ), Bq = 100 * np.eye( nq ) )

    # The parameters of min-jerk-traj, Initial joint posture, final joint posture, and duration.
    q0i = np.zeros( nq )
    q0f =  np.ones( nq )
    D   = 1.0

    # Use minimum-jerk trajectory as the reference trajectory 
    ctrl.add_mov_pars( q0i = q0i , q0f = q0f, D = D, ti = args.start_time  )    

    # Add the controller of the simulation
    my_sim.add_ctrl( ctrl )

    # The initial condition of the robot and its setup
    my_sim.init( qpos = q0i, qvel = np.zeros( nq ) )

    # Run the simulation
    my_sim.run( )

    if args.is_save_data or args.is_record_vid:  ctrl.export_data( my_sim.tmp_dir )

    my_sim.close( )


def run_movement_primitives( my_sim ):    

    # Define the canonical system
    cs = CanonicalSystem( mov_type = "discrete" )

    # The number of degrees of freedom of the tobot 
    nq = my_sim.nq

    # The time step of the simulation 
    dt = my_sim.dt

    # Dynamic Movement Primitives (DMP)
    dmp_list = [] 

    # The number of basis functions for the imitation learning
    N = 50

    # Iterating over the number of joints to each attach DMP
    for i in range( nq ): 
        dmp = DynamicMovementPrimitives( mov_type = "discrete", name = "dmp_joint" + str( i + 1 ), cs = cs, n_bfs = N, alpha_z = 10, beta_z = 2.5 )
        dmp_list.append( dmp )

    # The parameters of min-jerk-traj
    q0i = np.zeros( nq )
    q0f =  np.ones( nq )
    D   = 1.0

    # The time constant tau is the duration of the movement. 
    cs.tau = D        

    # The number of sample points for imitation learning
    P = 100

    # The time step of imitation learning
    # This is simply defined by D/P
    # Adding tmp for the dt
    td = np.linspace( 0, D, P )

    # The P samples points of q_des, dq_des, ddq_des
    q_des   = np.zeros( ( nq, P ) )
    dq_des  = np.zeros( ( nq, P ) ) 
    ddq_des = np.zeros( ( nq, P ) )

    for i in range( nq ):
        for j in range( P ):
            t = td[ j ]
            q_des[ i, j ], dq_des[ i, j ], ddq_des[ i, j ] = min_jerk_traj( t, 0.0, q0i[ i ], q0f[ i ], D  )

        # Once the trajectory is defined, conduct imitation learning.
        dmp = dmp_list[ i ]
        dmp.imitation_learning( td, q_des[ i, : ], dq_des[ i, : ], ddq_des[ i, : ] )

        print( np.array( dmp.weights ) )
    
    exit( )

    # Now, we integrate the simulation via the learned weights
    # For this, the initial and final time of the simulation is important.
    N_sim = round( args.run_time/dt ) + 1

    # The q, dq, ddq of the robot 
    q_command   = np.zeros( ( nq, N_sim ) )
    dq_command  = np.zeros( ( nq, N_sim ) )
    ddq_command = np.zeros( ( nq, N_sim ) )

    # Iterating through the dmps
    for i in range( nq ):

        dmp = dmp_list[ i ]

        # t, y, z, dy, dz
        t_arr, y_arr, _, dy_arr, dz_arr = dmp.integrate( q0i[ i ], 0, q0f[ i ], dt, args.start_time, N_sim )

        q_command[   i, : ] =   y_arr
        dq_command[  i, : ] =  dy_arr
        ddq_command[ i, : ] =  dz_arr

    # Define the controller
    dmp_ctrl = DMPJointController2DOF( my_sim, args, name = "joint_dmp" )
    dmp_ctrl.set_traj( q_command, dq_command, ddq_command )

    # Add the controller to the simulation
    my_sim.add_ctrl( dmp_ctrl )

    # Initialization of the simulation
    my_sim.init( qpos = q0i, qvel = np.zeros( nq ) )

    # Run the simulation
    my_sim.run( )

    if args.is_save_data or args.is_record_vid:  
        for i in range( nq ):
            dmp_list[ i ].save_mat_data( my_sim.tmp_dir )

    my_sim.close( )

if __name__ == "__main__":
                                                                                
    # Parse arguments given from the terminal
    parser = my_parser( )
    args, unknown = parser.parse_known_args( )

    # Define the type of movement and its control method
    assert args.sim_type in [ "motor", "movement" ]

    # Define the robot that we will use 
    args.model_name = "2DOF_planar_torque"

    # Set the camera position of the simulation
    # Lookat [3] Distance, Elevation, Azimuth
    args.cam_pos = np.array( [ 0.3, 0.8, 0, 4, -90, 90 ] )

    my_sim = Simulation( args )    

    if    args.sim_type == "motor"   :    run_motor_primitives( my_sim )
    elif  args.sim_type == "movement": run_movement_primitives( my_sim )

    

