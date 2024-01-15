"""

# ============================================================================= #
| Project:        [M3X Whip Project]
| Title:          Controlling Position and Orientation
| Author:         Moses C. Nah
| Email:          [Moses] mosesnah@mit.edu
# ============================================================================= #

"""

import os
import sys
import shutil
import numpy      as np
import mujoco_py  as mjPy
import scipy.io
import matlab.engine


from datetime  import datetime

# The Local Modules
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )
from utils       import min_jerk_traj, geodesicSO3, rotx, SO3_to_R3, R3_to_SO3, SO3_to_quat, skew_sym, quaternion_multiply, quaternion_conj
from constants   import Constants as C
from constants   import my_parser
from simulation  import Simulation

# Modules for DMP
sys.path.append( os.path.join( os.path.dirname(__file__), "DMPmodules" ) )
sys.path.append( os.path.join( os.path.dirname(__file__), "inversemodel" ) )

from CanonicalSystem            import CanonicalSystem 
from DynamicMovementPrimitives  import DynamicMovementPrimitives
from InverseDynamicsModel       import getiiwa_dJ

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       

def run_motor_primitives( args ):

    # The model name
    model_name =  "iiwa14/iiwa14"

    tmp_dir  = C.TMP_DIR + datetime.now( ).strftime( "%Y%m%d_%H%M%S/" )    
    os.mkdir( tmp_dir )  

    # Construct the basic mujoco attributes
    mj_model  = mjPy.load_model_from_path( C.MODEL_DIR + model_name + ".xml" ) 
    mj_sim    = mjPy.MjSim( mj_model )    
    mj_data   = mj_sim.data                                              
    mj_viewer = mjPy.MjViewerBasic( mj_sim ) if not args.is_vid_off else None  
    
    args.save_freq = 1000

    # The basic info of the model
    n_act     = len( mj_model.actuator_names )
    nq        = len( mj_model.joint_names    )
    dt        = mj_model.opt.timestep                               
    T         = args.run_time    
    fps       = 60   
    n_steps   = 0
    vid_step  = round( ( 1. / dt ) / ( fps / args.vid_speed )  )
    save_step = round( ( 1. / dt ) / args.save_freq  )                        

    # Set the camera viewer
    tmp = args.cam_pos
    mj_viewer.cam.lookat[ 0 : 3 ] = tmp[ 0 : 3 ]
    mj_viewer.cam.distance        = tmp[ 3 ]
    mj_viewer.cam.elevation       = tmp[ 4 ]
    mj_viewer.cam.azimuth         = tmp[ 5 ]    

    # Setting the initial position and velocity of the robot 
    qpos = np.array( [-0.5000, 0.8236,0,-1.0472,0.8000, 1.5708, 0 ] )
    qvel = np.zeros( nq )

    # If the array is shorter than the actual self.nq in the model, just fill it with zero 
    if n_act != 1:
        mj_data.qpos[ : ] = qpos[ : nq ] if len( qpos ) >= nq else np.concatenate( ( qpos, np.zeros( nq - len( qpos ) ) ) , axis = None )
        mj_data.qvel[ : ] = qvel[ : nq ] if len( qvel ) >= nq else np.concatenate( ( qvel, np.zeros( nq - len( qvel ) ) ) , axis = None )
    else:
        mj_data.qpos[ 0 ] = qpos
        mj_data.qvel[ 0 ] = qvel

    # Forward the simulation to update the posture 
    mj_sim.forward( )

    # The end-effector name
    EE_name = "iiwa14_right_hand"

    # Get the initial end-effector position and orientation
    p_init = np.copy( mj_data.get_body_xpos( EE_name ) )
    R_init = np.copy( mj_data.get_body_xmat( EE_name ) )

    # The goal location and orientation
    p_goal = p_init - 2 * np.array( [0.0, p_init[ 1 ], 0.0])
    Rdelta = 80
    R_goal = R_init @ rotx( Rdelta*np.pi/180 ) 
    
    # Duration and starting time
    t0i = 0.0
    D   = 3.0

    # We only need to save the q_data for visualization
    t_arr_save = [ ]
    q_arr_save = [ ]
    p_save     = [ ]
    R_save     = [ ]
    p0_save    = [ ]
    R0_save    = [ ]

    # Also save the robot's link position and rotation matrices 
    p_links_save, R_links_save = [], [] 

    # The main loop of the simulation 
    while mj_data.time <= T + 1e-7:

        # Render the simulation if mj_viewer exists        
        if mj_viewer is not None and n_steps % vid_step == 0:
            mj_viewer.render( )
            print( mj_data.time )

        # Joint position and velocity
        q_curr  = np.copy( mj_data.qpos[ : n_act ] )
        dq_curr = np.copy( mj_data.qvel[ : n_act ] )

        # End-effector position and velocity, translational
        p_curr  = np.copy( mj_data.get_body_xpos(  EE_name ) )
        dp_curr = np.copy( mj_data.get_body_xvelp( EE_name ) )
        Jp      = np.copy( mj_data.get_body_jacp(  EE_name ).reshape( 3, -1 ) )

        # Get also the rotational part
        Jr     = np.copy( mj_data.get_body_jacr(  EE_name ).reshape( 3, -1 ) )
        w_curr = np.copy( mj_data.get_body_xvelr( EE_name ) )
        R_curr = np.copy( mj_data.get_body_xmat(  EE_name ) )

        # The end-effector virtual translational trajectory
        # For this example, we use the minimum-jerk trajectory 
        p0, dp0, _ = min_jerk_traj( mj_data.time, t0i, p_init, p_goal, D )

        # End-effector position and velocity, rotational 
        R0 = geodesicSO3( R_init, R_goal, t0i, D, mj_data.time )

        # Add the Controller here
        # Controller 1: Joint-space Damping
        tau1 = -20 * dq_curr
        tau2 = Jp.T @ ( 1600 * ( p0 - p_curr ) + 800 * ( dp0 - dp_curr ) )
        tau3 = Jr.T @ ( 80 * R_curr @ SO3_to_R3( R_curr.T @ R0 ) - 8 * w_curr )

        tau = tau1 + tau2 + tau3
        mj_data.ctrl[ :n_act ] = tau

        # Run a single simulation 
        mj_sim.step( )
        n_steps += 1

        # Render the simulation if mj_viewer exists        
        if args.is_save_data and n_steps % save_step == 0:
            t_arr_save.append( mj_data.time )
            q_arr_save.append( q_curr ) 
            p_save.append( p_curr )
            p0_save.append( p0 )
            
            R_save.append( R_curr )
            R0_save.append( R0 )

            # Also save the robot's link position and rotation matrices 
            p_tmp = []
            R_tmp = []
            for i in range( 7 ):
                p_tmp.append( np.copy( mj_data.get_body_xpos( "iiwa14_link_" + str( i + 1 ) ) ) )
                R_tmp.append( np.copy( mj_data.get_body_xmat( "iiwa14_link_" + str( i + 1 ) ) ) )

            # Also save the robot's link position and rotation matrices 
            p_links_save.append( p_tmp )
            R_links_save.append( R_tmp )       
                        

    # If video recorded/save data is true, then copy the model, main file and the arguments passed
    if args.is_save_data:
        shutil.copyfile( C.MODEL_DIR + model_name + ".xml", tmp_dir + "model.xml" )

        file_name = tmp_dir + "/data.mat"
        
        # Packing up the arrays as a dictionary
        # For the data, we simply take the transpose

        scipy.io.savemat( file_name, { "q_arr": q_arr_save, "t_arr": t_arr_save, "p_arr": p_save, "R_arr": R_save, "p0_arr": p0_save, "R0_arr":R0_save , 
                                       "p_links": p_links_save, "R_links": R_links_save } )        
        
    # Move the tmp folder to results if not empty, else just remove the tmp file. 
    shutil.move( tmp_dir, C.SAVE_DIR  ) if len( os.listdir( tmp_dir ) ) != 0 else os.rmdir( tmp_dir )
            

def run_movement_primitives( args ):
    
    # We need to communicate with the MATLAB script for the calculation of the Coriolis matrix
    # Note that the controller involves:
    # M(q)ddqr + C(q)dqr ...
    # Hence, we need a "separate" Coriolis/centrifugal matrix, that is done numerically via MATLAB

    # This act was necessary, as importing the symbolic term of the Coriolis 
    # was too complex and long, hence impractical.
    eng = matlab.engine.start_matlab( )
    eng.addpath( '/Users/mosesnah/Documents/projects/Explicit-MATLAB/examples'         )
    eng.addpath( '/Users/mosesnah/Documents/projects/Explicit-MATLAB/robots'           )
    eng.addpath( '/Users/mosesnah/Documents/projects/Explicit-MATLAB/helpers_geometry' )
    eng.addpath( '/Users/mosesnah/Documents/projects/Explicit-MATLAB/graphics'         )
    eng.addpath( '/Users/mosesnah/Documents/projects/Explicit-MATLAB/utils'            )

    # The model name
    model_name =  "iiwa14/iiwa14"

    tmp_dir  = C.TMP_DIR + datetime.now( ).strftime( "%Y%m%d_%H%M%S/" )    
    os.mkdir( tmp_dir )  

    # Construct the basic mujoco attributes
    mj_model  = mjPy.load_model_from_path( C.MODEL_DIR + model_name + ".xml" ) 
    mj_sim    = mjPy.MjSim( mj_model )    
    mj_data   = mj_sim.data                                              
    mj_viewer = mjPy.MjViewerBasic( mj_sim ) if not args.is_vid_off else None  
    
    args.save_freq = 1000

    # The basic info of the model
    n_act     = len( mj_model.actuator_names )
    nq        = len( mj_model.joint_names    )
    dt        = mj_model.opt.timestep                               
    T         = args.run_time    
    dt        = 0.0001
    fps       = 60   
    n_steps   = 0
    vid_step  = round( ( 1. / dt ) / ( fps / args.vid_speed )  )
    save_step = round( ( 1. / dt ) / args.save_freq  )                        

    # Set the camera viewer
    tmp = args.cam_pos
    mj_viewer.cam.lookat[ 0 : 3 ] = tmp[ 0 : 3 ]
    mj_viewer.cam.distance        = tmp[ 3 ]
    mj_viewer.cam.elevation       = tmp[ 4 ]
    mj_viewer.cam.azimuth         = tmp[ 5 ]    

    # Setting the initial position and velocity of the robot 
    qpos = np.array( [ -0.5000, 0.8236, 0.0000, -1.0472, 0.8000, 1.5708, 0 ] )
    # qpos = np.zeros( nq )
    qvel = np.zeros( nq )

    # If the array is shorter than the actual self.nq in the model, just fill it with zero 
    if n_act != 1:
        mj_data.qpos[ : ] = qpos[ : nq ] if len( qpos ) >= nq else np.concatenate( ( qpos, np.zeros( nq - len( qpos ) ) ) , axis = None )
        mj_data.qvel[ : ] = qvel[ : nq ] if len( qvel ) >= nq else np.concatenate( ( qvel, np.zeros( nq - len( qvel ) ) ) , axis = None )
    else:
        mj_data.qpos[ 0 ] = qpos
        mj_data.qvel[ 0 ] = qvel

    # Forward the simulation to update the posture 
    mj_sim.forward( )

    # The end-effector name
    EE_name = "iiwa14_right_hand"

    # Get the initial end-effector position and orientation
    p_init = np.copy( mj_data.get_body_xpos( EE_name ) )
    R_init = np.copy( mj_data.get_body_xmat( EE_name ) )

    deltaR = 80
    p_goal = p_init - 2 * np.array( [0.0, p_init[ 1 ], 0.0])
    R_goal = R_init @ rotx( deltaR*np.pi/180 )
    quat_goal = SO3_to_quat( R_goal )

    # Duration and starting time
    t0i = 0.0
    D   = 3.0

    wd = -SO3_to_R3( R_init.T @ R_goal )/D

    # We only need to save the q_data for visualization
    t_arr_save = []
    q_arr_save = []
    p_save     = []
    R_save     = []
    p0_save    = []
    R0_save    = []

    # Also save the robot's link position and rotation matrices 
    p_links_save, R_links_save = [], [] 

    # The main loop of the simulation 
    while mj_data.time <= T + 1e-7:

        # Render the simulation if mj_viewer exists        
        if mj_viewer is not None and n_steps % vid_step == 0:
            mj_viewer.render( )
            print( mj_data.time )

        # Get current end-effector position and velocity 
        p    = np.copy( mj_data.get_body_xpos(  EE_name ) )
        dp   = np.copy( mj_data.get_body_xvelp( EE_name ) )
        w    = np.copy( mj_data.get_body_xvelr( EE_name ) )
        R    = np.copy( mj_data.get_body_xmat(  EE_name ) )
        quat = SO3_to_quat( R )

        # Get the current robot's joint position/velocity trajectories
        q   = np.copy( mj_data.qpos[ :n_act ] )
        dq  = np.copy( mj_data.qvel[ :n_act ] )

        # Get the reference trajectory
        p_des, dp_des, ddp_des = min_jerk_traj( mj_data.time, t0i, p_init, p_goal, D )         

        # Get the p and w difference
        p_del  =  p_des -  p
        dp_del = dp_des - dp
        w_del  = wd - w

        # End-effector position and velocity, rotational    
        R0 = geodesicSO3( R_init, R_goal, t0i, D, mj_data.time )
        quat_des = SO3_to_quat( R0 )

        # Get the quaternion difference epsilon difference
        quat_diff = quaternion_multiply( quat_des, quaternion_conj( quat )  )
        eta_del = quat_diff[ 0  ]
        eps_del = quat_diff[ 1: ]

        # Gains
        Kp = 50
        Ko = 50

        # Define xd
        xd = np.hstack( ( dp_des + Kp*p_del, wd + Ko*eps_del ) )
        
        # Get the Jacobian by matrix
        mat_q = matlab.double( q.tolist( ) )
        J_mat = eng.Jmat( mat_q )

        # The Jacobian inverse
        Jp = np.copy( mj_data.get_body_jacp( EE_name ).reshape( 3, -1 ) )
        Jr = np.copy( mj_data.get_body_jacr( EE_name ).reshape( 3, -1 ) )
        jac_pinv = np.linalg.pinv( np.vstack( ( Jp, Jr ) ) )        

        # print( J_mat - np.vstack( ( Jp, Jr ) ) )

        vd = jac_pinv @ xd

        tmp1 = ddp_des + Kp*dp_del
        tmp2 = Ko * ( 0.5 * ( eta_del * np.eye( 3 ) + skew_sym( eps_del ) ) @ w_del - skew_sym( eps_del ) @ wd ) 
        dxd  = np.hstack( [ tmp1, tmp2 ] )

        # Numerically get the difference
        J_mat1 = eng.Jmat( mat_q )
        mat_q2 = matlab.double( ( q + dq * dt ).tolist( ) ) 
        J_mat2 = eng.Jmat( mat_q2 )
        dJ = ( np.linalg.pinv( J_mat2 ) - np.linalg.pinv( J_mat1 ))/dt

        dvd = dJ @ xd + jac_pinv @ dxd

        # The mass/Coriolis matrices
        nq = mj_model.nq
        Mtmp = np.zeros( nq * nq )
        mjPy.cymj._mj_fullM( mj_model, Mtmp, mj_data.qM )
        M_mat = np.copy( Mtmp.reshape( nq, -1 ) )

        tau = M_mat @ ( dvd + 50 * ( vd - dq ) ) + mj_data.qfrc_bias
        mj_data.ctrl[ :n_act ] = tau

        # Run a single simulation 
        mj_sim.step( )
        n_steps += 1

        # Render the simulation if mj_viewer exists        
        if args.is_save_data and n_steps % save_step == 0:
            t_arr_save.append( mj_data.time )
            q_arr_save.append( q ) 
            p_save.append( p )
            p0_save.append( p_des )
            
            R_save.append( R )
            R0_save.append( R0 )

            # Also save the robot's link position and rotation matrices 
            p_tmp = []
            R_tmp = []
            for i in range( 7 ):
                p_tmp.append( np.copy( mj_data.get_body_xpos( "iiwa14_link_" + str( i + 1 ) ) ) )
                R_tmp.append( np.copy( mj_data.get_body_xmat( "iiwa14_link_" + str( i + 1 ) ) ) )

            # Also save the robot's link position and rotation matrices 
            p_links_save.append( p_tmp )
            R_links_save.append( R_tmp )              

    # If video recorded/save data is true, then copy the model, main file and the arguments passed
    if args.is_save_data:
        shutil.copyfile( C.MODEL_DIR + model_name + ".xml", tmp_dir + "model.xml" )

        file_name = tmp_dir + "/data.mat"
        
        # Packing up the arrays as a dictionary
        # For the data, we simply take the transpose

        scipy.io.savemat( file_name, { "q_arr": q_arr_save, "t_arr": t_arr_save, "p_arr": p_save, "R_arr": R_save, "p0_arr": p0_save, "R0_arr":R0_save ,
                                       "p_links": p_links_save, "R_links": R_links_save } )        
        
    # Move the tmp folder to results if not empty, else just remove the tmp file. 
    shutil.move( tmp_dir, C.SAVE_DIR  ) if len( os.listdir( tmp_dir ) ) != 0 else os.rmdir( tmp_dir )
            
    eng.quit( )

if __name__ == "__main__":
                                                                                
    # Generate the parser, which is defined 
    parser = my_parser( )
    args, unknown = parser.parse_known_args( )

    args.cam_pos = np.array( [ 0.0, 0.0, 0.3, 2, 0, 180 ] )  

    assert args.sim_type in [  "motor", "movement" ]

    # Set the camera position of the simulation
    if    args.sim_type == "motor"   :    run_motor_primitives( args )
    elif  args.sim_type == "movement": run_movement_primitives( args )
