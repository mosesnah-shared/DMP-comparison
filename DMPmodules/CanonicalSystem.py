import numpy as np
import matplotlib.pyplot as plt

class CanonicalSystem:
    """ 
        Canonical System of the Dynamic Movement Primitives 

        Discrete Movement: Equation 2.2 of [REF]
        Rhythmic Movement: Equation 2.5 of [REF]
        [REF]: Ijspeert, Auke Jan, et al. "Dynamical movement primitives: learning attractor models for motor behaviors." Neural computation 25.2 (2013): 328-373.
    """

    def __init__( self, mov_type: str, **kwargs ):

        assert mov_type in [ "rhythmic", "discrete" ]
        
        self.mov_type = mov_type

        # If movement is discrete
        if self.mov_type == "discrete":
            self.alpha_s = kwargs.get( 'alpha_s', 1.0 )
            self.tau     = kwargs.get( 'tau'    , 1.0 )

        else:
            self.tau     = kwargs.get( 'tau', 1.0 )

        self.reset( )


    def reset( self ):
        """
            Reset the system state
        """
        # If movement is discrete
        if self.mov_type == "discrete":
            self.s = 1

        else:
            self.s = 0 


    def get_value( self, t ):
        """
            The value at time t

            Discrete Movement: Equation 2.2 of [REF]

        """   
        if self.mov_type == "discrete":
            return np.exp( -self.alpha_s/self.tau * t )
        else:
            return np.mod( t/self.tau, 2 * np.pi )
        
if __name__ == "__main__":
    
    N = 100
    t_arr = np.linspace( 0, 30, N )

    discrete_cs = CanonicalSystem( mov_type = "rhythmic" )

    s_arr =  discrete_cs.get_value( t_arr )

    plt.plot( s_arr )
    plt.show( )