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
import numpy      as np
from matplotlib  import pyplot as plt

# To Add Local Files, adding the directory via sys module
# __file__ saves the current directory of this file. 
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from simulation   import Simulation
from controllers  import JointImpedanceController
from constants    import my_parser

sys.path.append( os.path.join( os.path.dirname(__file__), "DMP_references/dmpbbo-master" ) )


# The DMP Modules
# Library from: https://github.com/stulp/dmpbbo
# COMMIT NUMBER: 1941694ce81645390c677a44af75cec31e632999
from dmpbbo.dmps.Dmp                                       import Dmp
from dmpbbo.dmps.Trajectory                                import Trajectory
from dmpbbo.functionapproximators.FunctionApproximatorLWR import FunctionApproximatorLWR

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       
                                                                                
# Generate the parser, which is defined 
parser = my_parser( )
args, unknown = parser.parse_known_args( )


def run_motor_primitives( my_sim ):


    # Define the controller 
    ctrl = JointImpedanceController( my_sim, args, name = "joint_imp" )

    ctrl.set_impedance( Kq = np.array( [20] ), Bq = np.array( [ 10 ] ) )

    ctrl.add_mov_pars( q0i = np.array( [ 0 ] ) , q0f = np.array( [ 1 ] ), D = 1, ti = args.start_time  )    

    # Add the controller and objective of the simulation
    my_sim.add_ctrl( ctrl )

    init_cond = { "qpos": np.array( [ 0 ] ) ,  "qvel": np.array( [ 0 ] ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Run the simulation
    my_sim.run( )

    if args.is_save_data:  ctrl.export_data( my_sim.tmp_dir )

    my_sim.close( )


def run_movement_primitives( my_sim ):    
    
    # Define the Minimum Jerk Trajectory
    N  = 1000
    ts = np.linspace( 0, args.run_time, N ) 

    # Initial and Final Joint Posture
    q0i = np.array( [ 0.0 ] )
    q0f = np.array( [ 0.9 ] )

    traj_min_jerk = Trajectory.from_min_jerk( ts, q0i, q0f )

    # traj_min_jerk.plot( )    
    n_dims = 1

    # Locally Weighted Regression
    # There are Three Options for Function Fitting
    # We have FunctionApproximatorLWR  (Locally Weighted Regression)
    # We have FunctionApproximatorRBFN (Radial Bounded Function Network)
    # We have FunctionApproximatorWLS  (Weighted Least Square)

    # FunctionApproximatorLWR( n_bfs_per_dim, intersection_height=0.5, regularization=0.0):
    # The intersection height is the height of the Guassian
    # height = meta_params["intersection_height"]
    # centers, widths = Gaussian.get_centers_and_widths(inputs, n_bfs_per_dim, height)
    # for cc in range(n_bfs - 1):
    #     w = np.sqrt(np.square(cur_centers[cc + 1] - cur_centers[cc]) / (-8 * np.log(h)))
    #     cur_widths[cc] = w    
    # Zero Regularization

    function_apps = [ FunctionApproximatorLWR( 40 ) for _ in range( n_dims ) ]

    # We use the "IJSPEERT_2002_MOVEMENT" MOVEMENT DMP TYPE, for the Comparison
    dmp = Dmp.from_traj( traj_min_jerk, function_apps, dmp_type="IJSPEERT_2002_MOVEMENT", forcing_term_scaling="G_MINUS_Y0_SCALING" )

    # You don't need this in detail
    xs_ana, xds_ana, forcing_terms_ana, fa_outputs_ana = dmp.analytical_solution(ts)

    dt = ts[1]
    # dim_x = xs_ana.shape[1]
    dim_x = 5 
    # dim_dmp = 3 * y_init.size + 2
    # The reason for 3 is ( y, z, goal, phase, gating ), where phase and gating are s (canonical system)

    xs_step  = np.zeros([N, dim_x])
    xds_step = np.zeros([N, dim_x])

    x, xd = dmp.integrate_start( )
    xs_step[ 0 , : ] = x
    xds_step[ 0, : ] = xd

    for tt in range(1, N):
        xs_step[tt, :], xds_step[tt, :] = dmp.integrate_step(dt, xs_step[tt - 1, :])

    dmp.plot(ts, xs_ana, xds_ana, forcing_terms=forcing_terms_ana, fa_outputs=fa_outputs_ana)
    # dmp.plot(ts, xs_step, xds_step)
    
    lines, axs = traj_min_jerk.plot()
    plt.setp(lines, linestyle="-", linewidth=4, color=(0.8, 0.8, 0.8))
    plt.setp(lines, label="demonstration")

    traj_reproduced = dmp.states_as_trajectory( ts, xs_step, xds_step)
    lines, axs = traj_reproduced.plot(axs)
    plt.setp(lines, linestyle="--", linewidth=2, color=(0.0, 0.0, 0.5))
    plt.setp(lines, label="reproduced")

    plt.legend()
    plt.show( )


if __name__ == "__main__":

    # Generate an instance of our Simulation
    # The model is generated since the model name is passed via arguments

    args.model_name = "1DOF_planar_torque"

    idx = 2
    my_sim = Simulation( args )


    if idx == 1: run_motor_primitives( my_sim )
    else: run_movement_primitives( my_sim )

    

