import numpy    as np
import scipy.io
from   modules.utils     import min_jerk_traj

class Controller:
    """
        Description:
        -----------
            Parent class for the controllers

    """
    def __init__( self, mj_sim, args, name ): 

        # Saving the reference of mujoco model and data for 
        self.mj_sim   = mj_sim
        self.mj_model = mj_sim.mj_model
        self.mj_data  = mj_sim.mj_data
        self.name     = name

        # Saving the arguments passed via ArgumentParsers
        self.mj_args  = args        

        # Get the number of actuators of the model
        self.n_act = len( self.mj_model.actuator_names )        

        # Get the number of generalized coordinate of the model
        self.nq = len( self.mj_model.joint_names )                

        # The list of controller parameters
        self.names_ctrl_pars = None

        # Get the number of geom names of the simulation
        self.n_geoms = len( self.mj_model.geom_names )        

        # Saving the name of the data as a string array for "save_data" method
        self.names_data = None

    def init( self ):
        """
            Initialize an empty list of conroller parameters
        """

        # Generate an empty arrays with data and controller parameters 
        if self.names_data is not None:
            [ setattr( self, name + "_arr", [ ] ) for name in self.names_data  ]

        if self.names_ctrl_pars is not None:
            [ setattr( self,          name, [ ] ) for name in self.names_ctrl_pars  ]

    def save_data( self ):
        """
            Update the saved, which will be defined in method "input_calc"
        """
        
        for name in self.names_data:
            val = getattr( self, name )
            getattr( self, name + "_arr" ).append( val )
        

    def export_data( self, dir_name ):
        """
            Export the data as mat file
        """
        file_name = dir_name + "/ctrl_" + self.name + ".mat"
        
        # Packing up the arrays as a dictionary
        dict1 = { name + "_arr": getattr( self, name + "_arr" ) for name in self.names_data      }
        dict2 = { name: getattr( self, name )                   for name in self.names_ctrl_pars }
        
        scipy.io.savemat( file_name, { **dict1, **dict2 } )

    def input_calc( self, t ):
        """
            Calculating the torque input for the given time 
        """
        raise NotImplementedError


class ImpedanceController( Controller ):
    """
        Description:
        -----------
            Parent class for the Impedance Controller

    """

    def __init__( self, mj_sim, args, name ): 

        super( ).__init__( mj_sim, args, name )

        # There are crucial parameters which can be calculated from the given model. 
        # Hence, "parsing" the xml model file
        # The model name should be within the following list 



class JointImpedanceController( ImpedanceController ):

    """
        Description:
        ----------
            Class for a Joint (i.e., Generalized coordinate) Impedance Controller
            First order impedance controller with gravity compenation

    """

    def __init__( self, mj_sim, mj_args, name ):

        super( ).__init__( mj_sim, mj_args, name )

        # The name of the controller parameters 
        self.names_ctrl_pars = ( "Kq", "Bq", "q0i", "q0f", "D", "ti" )

        # The name of variables that will be saved 
        self.names_data = ( "t", "tau", "q", "q0", "dq", "dq0", "ddq"  )

        # Generate an empty lists names of parameters
        self.init( )

        # The number of submovements
        self.n_movs = 0 
        self.is_rhythmic = False

    def set_impedance( self, Kq: np.ndarray, Bq:np.ndarray ):

        # Make sure the given input is (self.n_act x self.n_act )
        assert len( Kq      ) == self.n_act and len( Bq      ) == self.n_act 

        if self.n_act != 1: assert len( Kq[ 0 ] ) == self.n_act and len( Bq[ 0 ] ) == self.n_act 

        self.Kq = Kq
        self.Bq = Bq 
        

    def add_mov_pars( self, q0i : np.ndarray, q0f: np.ndarray, D:float, ti: float ):

        # Make sure the given input is (self.n_act x self.n_act )
        assert len( q0i ) == self.n_act and len( q0f ) == self.n_act
        assert D > 0 and ti >= 0

        # If done, append the mov_parameters
        self.q0i.append( q0i )
        self.q0f.append( q0f )
        self.D.append(     D )
        self.ti.append(   ti )

        self.n_movs += 1


    def add_rhythmic_mov( self, amp : np.ndarray, w:int ):

        # The center must be a 3D location
        assert len( amp ) == self.n_act 
        assert w > 0 

        self.amp = amp
        self.w   = w

        self.is_rhythmic = True

    def input_calc( self, t ):
        """
            Descriptions
            ------------
                We implement the controller. 
                The controller generates torque with the following equation 

                tau = K( q0 - q ) + B( dq0 - dq ) + tau_G 

                (d)q0: The zero-torque trajectory, which follows a minimum-jerk profile. 
                 (d)q: current angular position (velocity) of the robot

            Arguments
            ---------
                t: The current time of the simulation. 
        """
        assert self.Kq is not None and self.Bq is not None
        assert self.n_movs >= 1 or self.is_rhythmic

        # Save the current time 
        self.t = t 

        # Get the current angular position and velocity of the robot arm only
        self.q   = np.copy( self.mj_data.qpos[ : self.n_act ] )
        self.dq  = np.copy( self.mj_data.qvel[ : self.n_act ] )
        self.ddq = np.copy( self.mj_data.qacc[ : self.n_act ] )        
 
        self.q0  = np.zeros( self.n_act )
        self.dq0 = np.zeros( self.n_act )


        if self.n_movs != 0:    
            for i in range( self.n_movs ):
                for j in range( self.n_act ):
                    tmp_q0, tmp_dq0 = min_jerk_traj( t, self.ti[ i ], self.ti[ i ] + self.D[ i ], self.q0i[ i ][ j ], self.q0f[ i ][ j ], self.D[ i ] )

                    self.q0[ j ]  += tmp_q0 
                    self.dq0[ j ] += tmp_dq0

        if self.is_rhythmic:
            for i in range( self.n_act ):
                self.q0[ i ] += self.amp[ i ] * np.sin( self.w * t )


        tau_imp   = self.Kq @ ( self.q0 - self.q ) + self.Bq @ ( self.dq0 - self.dq )
        self.tau  = tau_imp

        #     (1) index array       (3) The tau value
        return  np.arange( self.n_act ), self.tau

                
    def reset( self ):
        """
            Initialize all ctrl variables  
        """

        self.init( )
        self.n_movs = 0 

class CartesianImpedanceController( ImpedanceController ):

    def __init__( self, mj_sim, mj_args, name ):
        super( ).__init__( mj_sim, mj_args, name )

        # The name of the controller parameters 
        self.names_ctrl_pars = ( "Kx", "Bx", "x0i", "x0f", "D", "ti" )

        # The name of variables that will be saved 
        self.names_data = ( "t", "tau", "q", "dq", "J", "x0", "dx0", "xEE", "dxEE" )

        # Generate an empty lists names of parameters
        self.init( )

        # The number of submovements
        self.n_movs = 0 

        # Set rhythmic movement as false
        self.is_rhythmic = False

    def set_impedance( self, Kx: np.ndarray, Bx:np.ndarray ):

        # Regardless of planar/spatial robot, DOF = 3
        assert len( Kx      ) == 3 and len( Bx      ) == 3
        assert len( Kx[ 0 ] ) == 3 and len( Bx[ 0 ] ) == 3

        self.Kx = Kx
        self.Bx = Bx 

    def add_mov_pars( self, x0i : np.ndarray, x0f: np.ndarray, D:float, ti: float ):

        # Make sure the given input is 3 dimension
        # For Planar robot, all we need is 2, but for generalization.
        assert len( x0i ) == 3 and len( x0f ) == 3
        assert D > 0 and ti >= 0

        # If done, append the mov_parameters
        self.x0i.append( x0i )
        self.x0f.append( x0f )
        self.D.append(     D )
        self.ti.append(   ti )

        self.n_movs += 1

    def add_rhythmic_mov( self, r : int, center: np.ndarray, w:float ):

        # The center must be a 3D location
        assert len( center ) == 3 

        # The radius and angular velocity should be positive values
        assert r > 0 and w > 0

        self.center = center
        self.r = r 
        self.w = w

        self.is_rhythmic = True

    def input_calc( self, t ):
        """
            Descriptions
            ------------
                We implement the controller. 
                The controller generates torque with the following equation 

                tau = J^T( Kx( x0 - L(q) ) + Bx( dx0 - Jdq )  )

                L(q) is the forward kinematics map of the end-effector

            Arguments
            ---------
                t: The current time of the simulation. 
        """
        assert self.Kx is not None and self.Bx is not None
        assert self.n_movs >= 1 or self.is_rhythmic

        # Save the current time 
        self.t = t 

        # Get the current angular position and velocity of the robot arm only
        self.q  = np.copy( self.mj_data.qpos[ : self.n_act ] )
        self.dq = np.copy( self.mj_data.qvel[ : self.n_act ] )

        # Get the Jacobian of the end-effector
        # The Jacobian is 3-by-nq, although we only need the first two components
        self.J    = np.copy( self.mj_data.get_site_jacp(  "site_end_effector" ).reshape( 3, -1 ) )

        # Get the end-effector trajectories
        self.xEE  = np.copy( self.mj_data.get_site_xpos(  "site_end_effector" ) )
        self.dxEE = np.copy( self.mj_data.get_site_xvelp( "site_end_effector" ) )
 
        # The zero-force traejctory (3D)
        self.x0  = np.zeros( 3 )
        self.dx0 = np.zeros( 3 )

        if self.n_movs != 0:
            for i in range( self.n_movs ):
                for j in range( 3 ):
                    tmp_x0, tmp_dx0 = min_jerk_traj( t, self.ti[ i ], self.ti[ i ] + self.D[ i ], self.x0i[ i ][ j ], self.x0f[ i ][ j ], self.D[ i ] )

                    self.x0[ j ]  += tmp_x0 
                    self.dx0[ j ] += tmp_dx0

        if self.is_rhythmic:
            self.x0 += self.center + np.array( [ self.r * np.sin( self.w * t ), -self.r * np.cos( self.w * t ), 0] )

        self.tau  = self.J.T @ ( self.Kx @ ( self.x0 - self.xEE ) + self.Bx @ (self.dx0 - self.dxEE ) )

        if self.mj_args.is_save_data: self.save_data( )

        #     (1) index array       (3) The tau value
        return  np.arange( self.n_act ), self.tau

    def reset( self ):
        """
            Initialize all ctrl variables  
        """

        self.init( )
        self.n_movs = 0 

# Cartesian Impedance Controller for Obstacle avoidance
# This is separately needed
class CartesianImpedanceControllerObstacle( ImpedanceController ):

    def __init__( self, mj_sim, mj_args, name, obs_pos ):

        super( ).__init__( mj_sim, mj_args, name )

        # The name of the controller parameters 
        self.names_ctrl_pars = ( "k", "n", "obs_pos" )

        # The name of the controller parameters 
        self.names_data = ( "t", "F" )        

        # Generate an empty lists names of parameters
        self.init( )

        # The position of the obstacle
        # [TODO] [2022.10.22] [Moses C. Nah]
        # We can make this as a "moving obstacle"
        self.obs_pos = obs_pos        

    def set_impedance( self, k ):

        assert k >= 0 

        self.k = k

    def set_order( self, n : int ):

        assert n >= 1

        self.n = n        

    def input_calc( self, t ):
        """
            Descriptions
            ------------
                We Modified the Cartesian Controller

            Arguments
            ---------
                t: The current time of the simulation. 
        """

        assert self.k is not None

        # Save the current time 
        self.t = t 

        # Get the current robot end-effector's Jacobian, position and velocity
        J    = self.mj_data.get_site_jacp(  "site_end_effector" ).reshape( 3, -1 )
        xEE  = self.mj_data.get_site_xpos(  "site_end_effector" )

        # The displacement between the current position and obstacle 
        delta_x = self.obs_pos - xEE

        # Getting the radial displacement 
        r2 = np.linalg.norm( delta_x , ord = 2 ) 

        # The magnitude and direction of the repulsive force 
        self.F = - self.k * ( ( 1 / r2 ) ** self.n )  * delta_x 

        self.tau  = J.T @ self.F

        #     (1) index array       (3) The tau value
                # FILL IN
        return  np.arange( self.n_act ), self.tau

    def reset( self ):
        """
            Initialize all ctrl variables  
        """

        self.init( )
        self.n_movs = 0 

# Dynamic Movement Primitives
class DMP:
    def __init__( self, mj_sim, args, name ): 
        NotImplementedError( )
