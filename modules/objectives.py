import numpy as np
from modules.utils import *

class ObjectiveFunction:
    """
        The objective function that we are planning to maximize/minimize 
    """

    def __init__( self, mj_model, mj_data, args ):
        self.mj_model = mj_model
        self.mj_data  = mj_data
        self.args     = args

        # Point the output_func to the output_calc
        self.output_func = self.output_calc

    def output_calc( self, mj_model, mj_data, args ):
        """
            Define the function that will be called/calculated every time step. 

            The details of how to calculate the output values must be implemented in the subclasses

        """
        return self.output_func( mj_model, mj_data, args )

    def set_success_trig( self, cond ):
        """
            This defines an act that will be used called when objective is successful.  

        """
        # If condition is satisifed 
        # FILL IN THE DETAILS FOR THE CODE!
        pass


    # ============== Magic Methods ============== # 
    # These methods will help us write intuitive syntax. For instances
    # [Example] 2 * obj1 + obj2
    # =========================================== #

    def __add__( self, other_obj ):
        """
            +: Enables obj1 + obj2
        """        
        new_obj = ObjectiveFunction( self.mj_model, self.mj_data, self.args  )
        new_obj.output_func = lambda model, data, args: self.output_calc( model, data, args ) + other_obj.output_calc( model, data, args )
        return new_obj

    def __radd__( self, other_obj ):
        """
            + is a commutative operator, hence simply overwriting it with __add__
        """
        return self.__add__( other_obj )

    def __mul__( self, w: float ):    
        """
            Enables 3 * obj1 
        """        
        new_obj = ObjectiveFunction( self.mj_model, self.mj_data, self.args  )
        new_obj.output_func = lambda model, data, args: w * self.output_func( model, data, args ) 
        return new_obj

    def __rmul__( self, w: float ):    
        """
            A scalar multiplication is a commutative operator, hence simply overwriting it with __mul__
        """        
        return self.__mul__( w )

    def __truediv__( self, w: float ):
        """
            Enables obj1 / 2
        """        

        new_obj = ObjectiveFunction( self.mj_model, self.mj_data, self.args  )
        new_obj.output_func = lambda model, data, args: self.output_calc( model, data, args ) / w

        return new_obj    

    def __rtruediv__( self, w: float ):
        """
            Enables 2 / obj1
        """                

        new_obj = ObjectiveFunction( self.mj_model, self.mj_data, self.args  )
        new_obj.output_func = lambda model, data, args: w / self.output_calc( model, data, args ) 

        return new_obj    


