"""

# ============================================================================= #
| Project:        [M3X Whip Project]
| Title:          Dynamic Motor Primitives, Joint-space trajectory planning.
| Author:         Moses C. Nah
| Email:          [Moses] mosesnah@mit.edu
# ============================================================================= #

"""

import os
import sys
import numpy      as np
from matplotlib  import pyplot as plt

# To Add Local Files, adding the directory via sys module
# __file__ saves the current directory of this file. 
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from simulation   import Simulation
from controllers  import JointImpedanceController
from constants    import my_parser

sys.path.append( os.path.join( os.path.dirname(__file__), "DMP_references/pydmps-master" ) )

# DMP Libraries from Travis DeWolf
# [REF] https://github.com/studywolf/pydmps
from pydmps.dmp_discrete  import DMPs_discrete

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       
                                                                                
# Generate the parser, which is defined 
parser = my_parser( )
args, unknown = parser.parse_known_args( )


def run_motor_primitives( my_sim ):

    # Define the controller 
    ctrl = JointImpedanceController( my_sim, args, name = "joint_imp" )

    # The joint stiffness and damping matrices
    ctrl.set_impedance( Kq = np.array( [20] ), Bq = np.array( [ 10 ] ) )

    # Minimum Jerk Trajectory
    ctrl.add_mov_pars( q0i = np.array( [ 0 ] ) , q0f = np.array( [ 1 ] ), D = 1, ti = args.start_time  )    

    # Add the controller and objective of the simulation
    my_sim.add_ctrl( ctrl )

    # The initial condition of the robot and its setup
    init_cond = { "qpos": np.array( [ 0 ] ) ,  "qvel": np.array( [ 0 ] ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Run the simulation
    my_sim.run( )

    if args.is_save_data:  ctrl.export_data( my_sim.tmp_dir )

    my_sim.close( )


def run_movement_primitives( my_sim ):    
    
    # The number of basis functions
    n_bfs = 10

    # Define the Minimum Jerk Trajectory
    # The movement parameters of minimum-jerk-trajectory
    p0i = 0.0
    p0f = 1.0
    D   = 1.0
    N   = 100
    t_arr = np.linspace( 0, D, N )
    y_des   = np.zeros( ( 1, N ) )
    dy_des  = np.zeros( ( 1, N ) )
    ddy_des = np.zeros( ( 1, N ) )

    y_des[ 0, : ]   =          p0i + ( p0f - p0i ) * ( 10 * ( t_arr / D ) ** 3 -  15 * ( t_arr / D ) ** 4 +   6 * ( t_arr / D ) ** 5 )
    dy_des[ 0, : ]  =         1./D * ( p0f - p0i ) * ( 30 * ( t_arr / D ) ** 2 -  60 * ( t_arr / D ) ** 3 +  30 * ( t_arr / D ) ** 4 )
    ddy_des[ 0, : ] = 1./( D** 2 ) * ( p0f - p0i ) * ( 60 * ( t_arr / D )      - 180 * ( t_arr / D ) ** 2 + 120 * ( t_arr / D ) ** 3 )

    dmp = DMPs_discrete( n_dmps = 1, n_bfs=n_bfs, dt=D/N )
    dmp.imitate_path( y_des = y_des )
    # dmp.imitate_path2( y_des = y_des, dy_des = dy_des, ddy_des = ddy_des )
    
    # change the scale of the movement
    # dmp.goal[ 0] = 3
    # dmp.goal[ 1] = 2

    y_track, dy_track, ddy_track = dmp.rollout()

    plt.figure(2)
    # plt.subplot(211)
    plt.plot(y_track, lw=2)
    plt.plot(dy_track, lw=2)
    plt.plot(ddy_track, lw=2)
    # plt.subplot(212)
    # plt.plot(y_track[:, 1], lw=2)
    plt.show( )

if __name__ == "__main__":

    # Generate an instance of our Simulation
    # The model is generated since the model name is passed via arguments

    args.model_name = "1DOF_planar_torque"

    idx = 2
    my_sim = Simulation( args )

    if idx == 1: run_motor_primitives( my_sim )
    else: run_movement_primitives( my_sim )

    

