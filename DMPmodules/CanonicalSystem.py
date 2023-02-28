import numpy as np
import matplotlib.pyplot as plt

class CanonicalSystem:
    """ 
        Descriptions
        ------------    
            Canonical System of the Dynamic Movement Primitives 

            Discrete Movement: A stable first-order linear differential equation Equation 2.2 of [REF]
            Rhythmic Movement: A linear function with modulo 2pi, Equation 2.5 of [REF]
            [REF]: Ijspeert, Auke Jan, et al. "Dynamical movement primitives: learning attractor models for motor behaviors." Neural computation 25.2 (2013): 328-373.
    """

    def __init__( self, mov_type: str, **kwargs ):
        """
        Descriptions
        ------------
            Constructor of the Canonical System

        Parameters
        ----------
            (1) mov_type : str
                    Either "discrete" or "rhythmic" canonical system

            (*) kwargs : Check the following list 
                    alpha_s - the decay rate for "discrete canonical system"
                    tau     - The time constant of the transformation system. Equation 2.1 of [REF]     
                            - Refer to "DynamicMovementPrimitives.py"
        """
        assert mov_type in [ "rhythmic", "discrete" ]
        self.mov_type = mov_type

        # for rhythmic movement just saving the value as None
        self.alpha_s  = kwargs.get( 'alpha_s', 1.0 ) if self.mov_type == "discrete" else None
        self.tau      = kwargs.get( 'tau'    , 1.0 )

        # Initialization
        self.init( )

    def init( self ):
        """
        Descriptions
        ------------
            Initialize the canonical system
            The initial condition is:
                - For discrete movement, s(0) = 1
                - For rhythmic movement, s(0) = 0                        
        """

        self.s = 1 if self.mov_type == "discrete" else 0

    def get_value( self, t ):
        """
        Descriptions
        ------------
            The value of canonical system at time t
            Discrete Canonical System: s(0)exp( -alpha_s/tau * t ), Equation 2.2 of [REF]
            Rhythmic Canonical System: t/tau mod (2 i)            , Equation 2.5 of [REF]

        Parameters
        ----------
            (1) t : float (scalar or array)
                    
        Returns
        ------- 
            s( t ): float (scalar or array)

        """   

        return np.exp( -self.alpha_s/self.tau * t ) if self.mov_type == "discrete" else np.mod( t/self.tau, 2 * np.pi )    
        

if __name__ == "__main__":

    # Creating the time-array for the plot
    N     = 1000
    tmin  = 0.0
    tmax  = 4*np.pi
    t_arr = np.linspace( tmin, tmax, N )

    # Creating 1x2 subplot 
    fig, ( ax1, ax2 ) = plt.subplots( nrows = 1, ncols = 2, figsize = (8, 4) )

    # ======================================================== #
    # =============== DISCRETE CANONICAL SYSTEM ============== #
    # ======================================================== #

    cs1 = CanonicalSystem( mov_type = "discrete" )
    s_arr = cs1.get_value( t_arr )

    ax1.plot( t_arr, s_arr, lw = 4 )
    ax1.set_title( "Discrete Canonical System" )
    ax1.set_xlabel( "$t~(sec)$" )
    ax1.set_ylabel( "$s~(-)$"   )
    ax1.set_xlim( [ tmin, tmax ] )

    # ======================================================== #
    # =============== RHYTHMIC CANONICAL SYSTEM ============== #
    # ======================================================== #    
    cs2 = CanonicalSystem( mov_type = "rhythmic" )
    s_arr = cs2.get_value( t_arr )

    ax2.plot( t_arr, s_arr, lw = 4 )
    ax2.set_title( "Rhythmic Canonical System" )
    ax2.set_xlabel( "$t~(sec)$" )
    ax2.set_ylabel( "$s~(-)$"   )
    ax2.set_xlim( [ tmin, tmax ] )

    plt.show( )