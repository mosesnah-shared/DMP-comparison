import numpy as np
import matplotlib.pyplot as plt

class CanonicalSystem:
    """ 
        Canonical System of the Dynamic Movement Primitives 

        Discrete Movement: A stable first-order linear differential equation Equation 2.2 of [REF]
        Rhythmic Movement: A linear function with modulo 2pi, Equation 2.5 of [REF]
        [REF]: Ijspeert, Auke Jan, et al. "Dynamical movement primitives: learning attractor models for motor behaviors." Neural computation 25.2 (2013): 328-373.
    """

    def __init__( self, mov_type: str, **kwargs ):

        assert mov_type in [ "rhythmic", "discrete" ]
        self.mov_type = mov_type

        self.alpha_s  = kwargs.get( 'alpha_s', 1.0 ) if self.mov_type == "discrete" else None
        self.tau      = kwargs.get( 'tau'    , 1.0 )

        self.reset( )

    def reset( self ):
        """
            Reset the system state
        """
        # If movement is discrete, s( 0 ) = 1
        # If movement is rhythmic, s( 0 ) = 0 
        self.s = 1 if self.mov_type == "discrete" else 0

    def get_value( self, t ):
        """
            The value at time t

            Discrete Movement: Equation 2.2 of [REF]

        """   
        return np.exp( -self.alpha_s/self.tau * t ) if self.mov_type == "discrete" else np.mod( t/self.tau, 2 * np.pi )    
        
if __name__ == "__main__":

    N  = 1000
    cs = CanonicalSystem( mov_type = "discrete" )

    t_arr = np.linspace( 0.0, 4 * np.pi, N )
    s_arr = cs.get_value( t_arr )

    plt.plot( t_arr, s_arr )
    plt.show( )