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

        assert self.names_data      is not None
        assert self.names_ctrl_pars is not None 
        
        # Generate an empty arrays with data and controller parameters 
        [ setattr( self, name + "_arr", [ ] ) for name in self.names_data       ]
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
        self.names_data = ( "t", "tau", "q", "q0", "dq", "dq0", "Jp", "Jr"  )

        # Generate an empty lists names of parameters
        self.init( )

        # The number of submovements
        self.n_movs = 0 

    def set_impedance( self, Kq: np.ndarray, Bq:np.ndarray ):

        # Make sure the given input is (self.n_act x self.n_act )
        assert len( Kq      ) == self.n_act and len( Bq      ) == self.n_act 
        assert len( Kq[ 0 ] ) == self.n_act and len( Bq[ 0 ] ) == self.n_act 

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

    def input_calc( self, t, is_gravity_comp = True, is_noise = False ):
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
        assert self.n_movs >= 1 

        # Save the current time 
        self.t = t 

        # Get the current angular position and velocity of the robot arm only
        self.q  = np.copy( self.mj_data.qpos[ : self.n_act ] )
        self.dq = np.copy( self.mj_data.qvel[ : self.n_act ] )
 
        self.q0  = np.zeros( self.n_act )
        self.dq0 = np.zeros( self.n_act )

        for i in range( self.n_movs ):
            for j in range( self.n_act ):
                tmp_q0, tmp_dq0 = min_jerk_traj( t, self.ti[ i ], self.ti[ i ] + self.D[ i ], self.q0i[ i ][ j ], self.q0f[ i ][ j ], self.D[ i ] )

                self.q0[ j ]  += tmp_q0 
                self.dq0[ j ] += tmp_dq0

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

    def __init__( self, mj_sim, mj_args ):
        raise NotImplementedError( )

    def input_calc( self, t:float ):
        raise NotImplementedError( )

    def reset( self ):
        raise NotImplementedError( )



# Dynamic Movement Primitives
class DMP:
    def __init__( self, mj_sim, args, name ): 
        NotImplementedError( )
