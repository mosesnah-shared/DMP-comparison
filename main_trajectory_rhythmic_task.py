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
from controllers  import CartesianImpedanceController, DMPTaskController2DOF
from constants    import my_parser

sys.path.append( os.path.join( os.path.dirname(__file__), "DMPmodules" ) )

from CanonicalSystem            import CanonicalSystem 
from DynamicMovementPrimitives  import DynamicMovementPrimitives
from InverseDynamicsModel       import get2DOF_J

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       

def run_motor_primitives( my_sim ):

    # Define the joint-space impedance controller 
    ctrl = CartesianImpedanceController( my_sim, args, name = "task_imp" )

    # The joint stiffness and damping matrices
    ctrl.set_impedance( Kp = 90 * np.eye( 3 ), Bp = 60 * np.eye( 3 ) )

    # The parameters of the rhythmic controller
    r, omega0, c = 0.5, np.pi, np.sqrt( 2 )
     
    n  = my_sim.n_act

    # The initial condition of the robot. 
    q1 = np.arcsin( 0.5 * ( c + r ) )

    # The joint velocities
    q_init =  np.array( [ q1, np.pi-2*q1 ] )
    J   = get2DOF_J( q_init )
    dq_init = np.linalg.inv( J ) @ np.array( [ -r * omega0, 0 ] )

    init_cond = { "qpos": q_init,  "qvel": dq_init }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )
    ctrl.add_rhythmic_mov( amp = r, center = [ 0, c ], omega = omega0, offset = np.array( [ np.pi/2, np.pi/2  ] ) )

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

    tmp_str = [ "x", "y" ]
    for i in range( 2 ): 
        dmp = DynamicMovementPrimitives( mov_type = "rhythmic", name = "dmp_rhythmic" + tmp_str[ i ], cs = cs, n_bfs = N, alpha_z = 10, beta_z = 2.5 )
        dmp_list.append( dmp )

    # For rhythmic movement 
    # The x and y trajectory 
    r, omega0, c = 0.5, np.pi, np.sqrt( 2 )

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
        p_des[   0,i ] = - r * np.sin( omega0 * t )
        dp_des[  0,i ] = - r * omega0 * np.cos( omega0 * t )
        ddp_des[ 0,i ] =   r * ( omega0 ** 2 ) * np.sin( omega0 * t )

        p_des[   1,i ] =   r * np.cos( omega0 * t ) + c
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

    # Iterating through the dmps
    ttmp = [ 0 , c ]
    for i in range( 2 ):

        dmp = dmp_list[ i ]
        _, y_arr, _, dy_arr, dz_arr = dmp.integrate( p_des[ i, 0 ], dp_des[ i, 0 ], ttmp[ i ], dt, 0, N_sim )

        p_command[   i, : ] =   y_arr
        dp_command[  i, : ] =  dy_arr
        ddp_command[ i, : ] =  dz_arr

    # Define the controller
    dmp_ctrl = DMPTaskController2DOF( my_sim, args, name = "task_dmp" )
    dmp_ctrl.set_traj( p_command, dp_command, ddp_command )

    # The initial condition of the robot. 
    px = p_command[ 0, 0 ]
    py = p_command[ 1, 0 ]
    
    # Solve the inverse kinematics 
    q2 = np.pi - np.arccos( 0.5 * ( 2 - px ** 2 - py ** 2  ) )
    q1 = np.arctan2( py, px ) - q2/2 

    q_init = np.array( [ q1, q2 ] )

    # The joint velocities
    J   = get2DOF_J( q_init )
    dq_init = np.linalg.inv( J ) @ dp_command[ :, 0 ]

    init_cond = { "qpos": q_init ,  "qvel": dq_init }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Add the controller to the simulation
    my_sim.add_ctrl( dmp_ctrl )

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

    

