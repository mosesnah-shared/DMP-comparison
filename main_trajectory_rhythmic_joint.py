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
import moviepy.editor  as mpy

from matplotlib  import pyplot as plt

# Modules
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from simulation   import Simulation
from controllers  import JointImpedanceController, DMPJointController2DOF
from constants    import my_parser

sys.path.append( os.path.join( os.path.dirname(__file__), "DMPmodules" ) )

from CanonicalSystem            import CanonicalSystem 
from DynamicMovementPrimitives  import DynamicMovementPrimitives

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       

def run_motor_primitives( my_sim ):

    # Define the controller 
    ctrl = JointImpedanceController( my_sim, args, name = "joint_imp" )

    # Setting the joint-space impedances
    ctrl.set_impedance( Kq = np.diag( [ 150, 150 ] ), Bq = np.diag( [ 50, 50 ] ) )

    q_init  = np.array( [ 0.5, 0.5 ] )
    qa_init = np.array( [ 0.1, 0.3 ] )
    omega0  = 1 * np.pi
    dq_init = omega0 * qa_init 
    
    init_cond = { "qpos": q_init ,  "qvel": dq_init }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )
    ctrl.add_rhythmic_mov(  amp = qa_init, offset = np.zeros( 2 ), offset2 = q_init, omega = omega0  )

    # Add the controller and objective of the simulation
    my_sim.add_ctrl( ctrl )

    # Run the simulation
    my_sim.run( )

    if args.is_save_data or args.is_record_vid: 
        ctrl.export_data( my_sim.tmp_dir )

    my_sim.close( )

def run_movement_primitives( my_sim ):    

    # Define the canonical system
    cs = CanonicalSystem( mov_type = "rhythmic" )

    # The number of degrees of freedom of the tobot 
    nq = my_sim.nq

    # The number of basis functions
    N = 40

    # Dynamic Movement Primitives 
    dmp_list = [] 

    tmp_str = [ "q1", "q2" ]
    for i in range( 2 ): 
        dmp = DynamicMovementPrimitives( mov_type = "rhythmic", name = "dmp_rhythmic" + tmp_str[ i ], cs = cs, n_bfs = N, alpha_z = 10, beta_z = 2.5 )
        dmp_list.append( dmp )

    # For rhythmic movement 
    omega0 = 1 * np.pi

    # The period of the system
    Tp = 2 * np.pi / omega0 
    P  = 100
    dt = Tp/P    
    t_des  = dt * np.arange( P + 1 )
    cs.tau = Tp/(2*np.pi)

    # Movement Parameters
    q_init  = np.array( [ 0.5, 0.5 ] )
    qa_init = np.array( [ 0.1, 0.3 ] )
    dq_init = omega0 * qa_init 
    
    # The P samples points of p_des, dp_des, ddp_dex
    q_des   = np.zeros( ( 2, P + 1 ) )
    dq_des  = np.zeros( ( 2, P + 1 ) ) 
    ddq_des = np.zeros( ( 2, P + 1 ) )

    for i, t in enumerate( t_des ):
        q_des[   0,i ] =      q_init[ 0 ] + qa_init[ 0 ] * np.sin( omega0 * t )
        dq_des[  0,i ] =           omega0 * qa_init[ 0 ] * np.cos( omega0 * t )
        ddq_des[ 0,i ] = -( omega0 ** 2 ) * qa_init[ 0 ] * np.sin( omega0 * t )

        q_des[   1,i ] =      q_init[ 1 ] + qa_init[ 1 ] * np.sin( omega0 * t )
        dq_des[  1,i ] =           omega0 * qa_init[ 1 ] * np.cos( omega0 * t )
        ddq_des[ 1,i ] = -( omega0 ** 2 ) * qa_init[ 1 ] * np.sin( omega0 * t )

    for i in range( 2 ):
        dmp = dmp_list[ i ]
        dmp.imitation_learning( t_des, q_des[ i, : ], dq_des[ i, : ], ddq_des[ i, : ] )

    # Now, we integrate this solution
    # For this, the initial and final time of the simulation is important
    dt = my_sim.dt    
    N_sim = round( args.run_time/dt ) + 1

    # The q, dq, ddq of the robot 
    q_command   = np.zeros( ( 2, N_sim ) )
    dq_command  = np.zeros( ( 2, N_sim ) )
    ddq_command = np.zeros( ( 2, N_sim ) )

    # Iterating through the dmps
    for i in range( 2 ):

        dmp = dmp_list[ i ]
        _, y_arr, _, dy_arr, dz_arr = dmp.integrate( q_des[ i, 0 ], dq_des[ i, 0 ], q_init[ i ], dt, 0, N_sim )

        q_command[   i, : ] =   y_arr
        dq_command[  i, : ] =  dy_arr
        ddq_command[ i, : ] =  dz_arr

    # Define the controller
    dmp_ctrl = DMPJointController2DOF( my_sim, args, name = "joint_dmp" )
    dmp_ctrl.set_traj( q_command, dq_command, ddq_command )

    # Add the controller to the simulation
    my_sim.add_ctrl( dmp_ctrl )

    # Initialization of the simulation
    my_sim.init( qpos = q_init, qvel = dq_init )

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

    assert args.sim_type in [ "motor", "movement" ]

    # Define the robot that we will use 
    args.model_name = "2DOF_planar_torque"
    my_sim = Simulation( args )    

    # Set the camera position of the simulation
    # Lookat [3] Distance, Elevation, Azimuth
    args.cam_pos = np.array( [ 0.3, 0.6, 0, 4, -90, 90 ] )

    if    args.sim_type == "motor"   :    run_motor_primitives( my_sim )
    elif  args.sim_type == "movement": run_movement_primitives( my_sim )

    

