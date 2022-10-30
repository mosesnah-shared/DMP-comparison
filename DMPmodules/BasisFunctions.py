import numpy as np
import matplotlib.pyplot as plt

class BasisFunctions:
    """ 
        The Basis Function of the Nonlinear forcing term

        Discrete Movement: Equation 2.4 of [REF]
        Rhythmic Movement: Equation 2.7 of [REF]
        [REF]: Ijspeert, Auke Jan, et al. "Dynamical movement primitives: learning attractor models for motor behaviors." Neural computation 25.2 (2013): 328-373.
    """    

    def __init__( self, mov_type: str, n_bfs:int, cs ):
        """
            alpha_s is the gain of the canonical system 
            This value is required for discrete movement 
        """

        assert mov_type in [ "rhythmic", "discrete" ]
        assert n_bfs >= 2

        self.mov_type = mov_type    
        self.n_bfs    = n_bfs

        # The canonical system 
        self.cs = cs 

        # Depending on the movement type, define the heights and centers 
        # of the basis functions
        if self.mov_type == "discrete":

            # The center and width of the Gaussian basis function is simply 
            # Page 5 of [REF]
            # [REF] Dynamic Movement Primitives in Robotics: A Tutorial Survey

            self.centers = np.array( [ np.exp( -self.cs.alpha_s * i / ( self.n_bfs - 1 ) ) for i in range( self.n_bfs  ) ] )
            self.heights = 1.0 / ( np.diff( self.centers ) ** 2 )

            # Append the final value again 
            self.heights = np.append( self.heights, self.heights[ -1 ] )

            # The lambda basis function of the discrete movement is a Gaussian Function
            self.basis_func = lambda hi, ci, s : np.exp( - hi * ( s - ci ) ** 2 )

        # If rhythmic movement, von-Mises
        else:

            self.centers = np.linspace( 0, 2 * np.pi, self.n_bfs )
            self.heights = self.n_bfs * np.ones( self.n_bfs )

            # The lambda basis function of the discrete movement is a von-Mises Function
            self.basis_func = lambda hi, ci, s : np.exp( hi * ( np.cos( s - ci ) - 1 ) )             

    def calc_activation( self, i, t ):
        """
            Calculate the activation of the i-th basis function of the time arr
        """
    
        assert i >= 0 and i <= self.n_bfs - 1
        assert isinstance( t, float )

        return self.basis_func( self.heights[ i ], self.centers[ i ], t ) 
