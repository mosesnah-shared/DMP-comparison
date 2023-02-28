import numpy as np
import matplotlib.pyplot as plt

# Functions for calculating the inverse dynamics model
# =================================================== #
# ================= 2-DOF Robot ===================== #
# =================================================== #

# Derived from EXPLICIT
def get2DOF_M( q_arr ):
    q1, q2 = q_arr

    M = np.zeros( ( 2, 2 ) )
    M[0, 0] = np.cos(q2) + 7/2
    M[0, 1] = np.cos(q2)/2 + 5/4
    M[1, 0] = np.cos(q2)/2 + 5/4
    M[1, 1] = 5/4

    return M

def get2DOF_C( q_arr, dq_arr ):
    q1,   q2 = q_arr
    dq1, dq2 = dq_arr

    C = np.zeros( ( 2, 2 ) )
    C[0, 0] = -(dq2*np.sin(q2))/2
    C[0, 1] = -(np.sin(q2)*(dq1 + dq2))/2
    C[1, 0] = (dq1*np.sin(q2))/2
    C[1, 1] = 0    

    return C

def get2DOF_J( q_arr ):
    q1,   q2 = q_arr
    J = np.zeros( ( 2, 2 ) )
    J[0, 0] = - np.sin(q1 + q2) - np.sin(q1)
    J[0, 1] = -np.sin(q1 + q2)
    J[1, 0] = np.cos(q1 + q2) + np.cos(q1)
    J[1, 1] = np.cos(q1 + q2)

    return J   

def get2DOF_dJ( q_arr, dq_arr ):
    q1,   q2 = q_arr
    dq1, dq2 = dq_arr

    dJ = np.zeros( ( 2, 2 ) )

    dJ[0, 0] = - np.cos(q1 + q2)*(dq1 + dq2) - dq1*np.cos(q1)
    dJ[0, 1] = -np.cos(q1 + q2)*(dq1 + dq2)
    dJ[1, 0] = - np.sin(q1 + q2)*(dq1 + dq2) - dq1*np.sin(q1)
    dJ[1, 1] = -np.sin(q1 + q2)*(dq1 + dq2)

    return dJ


# Helper Functions generated from MATLAB
# For a 5DOF Robot
def get5DOF_C( qpos, qvel  ):
    q1 = qpos[ 0 ]
    q2 = qpos[ 1 ]
    q3 = qpos[ 2 ]
    q4 = qpos[ 3 ]
    q5 = qpos[ 4 ]

    dq1 = qvel[ 0 ]
    dq2 = qvel[ 1 ]
    dq3 = qvel[ 2 ]
    dq4 = qvel[ 3 ]
    dq5 = qvel[ 4 ]    

    C = np.zeros( ( 5, 5 ) )

    C[0, 0] = - (dq2*np.sin(q2 + q3 + q4 + q5))/2 - (dq3*np.sin(q2 + q3 + q4 + q5))/2 - (dq4*np.sin(q2 + q3 + q4 + q5))/2 - (dq5*np.sin(q2 + q3 + q4 + q5))/2 - (5*dq2*np.sin(q2 + q3))/2 - (5*dq3*np.sin(q2 + q3))/2 - (3*dq3*np.sin(q3 + q4))/2 - (3*dq4*np.sin(q3 + q4))/2 - (dq4*np.sin(q4 + q5))/2 - (dq5*np.sin(q4 + q5))/2 - (7*dq2*np.sin(q2))/2 - (5*dq3*np.sin(q3))/2 - (3*dq4*np.sin(q4))/2 - (dq5*np.sin(q5))/2 - (3*dq2*np.sin(q2 + q3 + q4))/2 - (3*dq3*np.sin(q2 + q3 + q4))/2 - (3*dq4*np.sin(q2 + q3 + q4))/2 - (dq3*np.sin(q3 + q4 + q5))/2 - (dq4*np.sin(q3 + q4 + q5))/2 - (dq5*np.sin(q3 + q4 + q5))/2
    C[0, 1] = - (dq1*np.sin(q2 + q3 + q4 + q5))/2 - (dq2*np.sin(q2 + q3 + q4 + q5))/2 - (dq3*np.sin(q2 + q3 + q4 + q5))/2 - (dq4*np.sin(q2 + q3 + q4 + q5))/2 - (dq5*np.sin(q2 + q3 + q4 + q5))/2 - (5*dq1*np.sin(q2 + q3))/2 - (5*dq2*np.sin(q2 + q3))/2 - (5*dq3*np.sin(q2 + q3))/2 - (3*dq3*np.sin(q3 + q4))/2 - (3*dq4*np.sin(q3 + q4))/2 - (dq4*np.sin(q4 + q5))/2 - (dq5*np.sin(q4 + q5))/2 - (7*dq1*np.sin(q2))/2 - (7*dq2*np.sin(q2))/2 - (5*dq3*np.sin(q3))/2 - (3*dq4*np.sin(q4))/2 - (dq5*np.sin(q5))/2 - (3*dq1*np.sin(q2 + q3 + q4))/2 - (3*dq2*np.sin(q2 + q3 + q4))/2 - (3*dq3*np.sin(q2 + q3 + q4))/2 - (3*dq4*np.sin(q2 + q3 + q4))/2 - (dq3*np.sin(q3 + q4 + q5))/2 - (dq4*np.sin(q3 + q4 + q5))/2 - (dq5*np.sin(q3 + q4 + q5))/2
    C[0, 2] = - (dq1*np.sin(q2 + q3 + q4 + q5))/2 - (dq2*np.sin(q2 + q3 + q4 + q5))/2 - (dq3*np.sin(q2 + q3 + q4 + q5))/2 - (dq4*np.sin(q2 + q3 + q4 + q5))/2 - (dq5*np.sin(q2 + q3 + q4 + q5))/2 - (5*dq1*np.sin(q2 + q3))/2 - (5*dq2*np.sin(q2 + q3))/2 - (3*dq1*np.sin(q3 + q4))/2 - (5*dq3*np.sin(q2 + q3))/2 - (3*dq2*np.sin(q3 + q4))/2 - (3*dq3*np.sin(q3 + q4))/2 - (3*dq4*np.sin(q3 + q4))/2 - (dq4*np.sin(q4 + q5))/2 - (dq5*np.sin(q4 + q5))/2 - (5*dq1*np.sin(q3))/2 - (5*dq2*np.sin(q3))/2 - (5*dq3*np.sin(q3))/2 - (3*dq4*np.sin(q4))/2 - (dq5*np.sin(q5))/2 - (3*dq1*np.sin(q2 + q3 + q4))/2 - (3*dq2*np.sin(q2 + q3 + q4))/2 - (3*dq3*np.sin(q2 + q3 + q4))/2 - (dq1*np.sin(q3 + q4 + q5))/2 - (3*dq4*np.sin(q2 + q3 + q4))/2 - (dq2*np.sin(q3 + q4 + q5))/2 - (dq3*np.sin(q3 + q4 + q5))/2 - (dq4*np.sin(q3 + q4 + q5))/2 - (dq5*np.sin(q3 + q4 + q5))/2
    C[0, 3] = - (dq1*np.sin(q2 + q3 + q4 + q5))/2 - (dq2*np.sin(q2 + q3 + q4 + q5))/2 - (dq3*np.sin(q2 + q3 + q4 + q5))/2 - (dq4*np.sin(q2 + q3 + q4 + q5))/2 - (dq5*np.sin(q2 + q3 + q4 + q5))/2 - (3*dq1*np.sin(q3 + q4))/2 - (3*dq2*np.sin(q3 + q4))/2 - (dq1*np.sin(q4 + q5))/2 - (3*dq3*np.sin(q3 + q4))/2 - (dq2*np.sin(q4 + q5))/2 - (3*dq4*np.sin(q3 + q4))/2 - (dq3*np.sin(q4 + q5))/2 - (dq4*np.sin(q4 + q5))/2 - (dq5*np.sin(q4 + q5))/2 - (3*dq1*np.sin(q4))/2 - (3*dq2*np.sin(q4))/2 - (3*dq3*np.sin(q4))/2 - (3*dq4*np.sin(q4))/2 - (dq5*np.sin(q5))/2 - (3*dq1*np.sin(q2 + q3 + q4))/2 - (3*dq2*np.sin(q2 + q3 + q4))/2 - (3*dq3*np.sin(q2 + q3 + q4))/2 - (dq1*np.sin(q3 + q4 + q5))/2 - (3*dq4*np.sin(q2 + q3 + q4))/2 - (dq2*np.sin(q3 + q4 + q5))/2 - (dq3*np.sin(q3 + q4 + q5))/2 - (dq4*np.sin(q3 + q4 + q5))/2 - (dq5*np.sin(q3 + q4 + q5))/2
    C[0, 4] = -((np.sin(q3 + q4 + q5) + np.sin(q2 + q3 + q4 + q5) + np.sin(q4 + q5) + np.sin(q5))*(dq1 + dq2 + dq3 + dq4 + dq5))/2
    C[1, 0] = (dq1*np.sin(q2 + q3 + q4 + q5))/2 + (5*dq1*np.sin(q2 + q3))/2 - (3*dq3*np.sin(q3 + q4))/2 - (3*dq4*np.sin(q3 + q4))/2 - (dq4*np.sin(q4 + q5))/2 - (dq5*np.sin(q4 + q5))/2 + (7*dq1*np.sin(q2))/2 - (5*dq3*np.sin(q3))/2 - (3*dq4*np.sin(q4))/2 - (dq5*np.sin(q5))/2 + (3*dq1*np.sin(q2 + q3 + q4))/2 - (dq3*np.sin(q3 + q4 + q5))/2 - (dq4*np.sin(q3 + q4 + q5))/2 - (dq5*np.sin(q3 + q4 + q5))/2
    C[1, 1] = - (3*dq3*np.sin(q3 + q4))/2 - (3*dq4*np.sin(q3 + q4))/2 - (dq4*np.sin(q4 + q5))/2 - (dq5*np.sin(q4 + q5))/2 - (5*dq3*np.sin(q3))/2 - (3*dq4*np.sin(q4))/2 - (dq5*np.sin(q5))/2 - (dq3*np.sin(q3 + q4 + q5))/2 - (dq4*np.sin(q3 + q4 + q5))/2 - (dq5*np.sin(q3 + q4 + q5))/2
    C[1, 2] = - (3*dq1*np.sin(q3 + q4))/2 - (3*dq2*np.sin(q3 + q4))/2 - (3*dq3*np.sin(q3 + q4))/2 - (3*dq4*np.sin(q3 + q4))/2 - (dq4*np.sin(q4 + q5))/2 - (dq5*np.sin(q4 + q5))/2 - (5*dq1*np.sin(q3))/2 - (5*dq2*np.sin(q3))/2 - (5*dq3*np.sin(q3))/2 - (3*dq4*np.sin(q4))/2 - (dq5*np.sin(q5))/2 - (dq1*np.sin(q3 + q4 + q5))/2 - (dq2*np.sin(q3 + q4 + q5))/2 - (dq3*np.sin(q3 + q4 + q5))/2 - (dq4*np.sin(q3 + q4 + q5))/2 - (dq5*np.sin(q3 + q4 + q5))/2
    C[1, 3] = - (3*dq1*np.sin(q3 + q4))/2 - (3*dq2*np.sin(q3 + q4))/2 - (dq1*np.sin(q4 + q5))/2 - (3*dq3*np.sin(q3 + q4))/2 - (dq2*np.sin(q4 + q5))/2 - (3*dq4*np.sin(q3 + q4))/2 - (dq3*np.sin(q4 + q5))/2 - (dq4*np.sin(q4 + q5))/2 - (dq5*np.sin(q4 + q5))/2 - (3*dq1*np.sin(q4))/2 - (3*dq2*np.sin(q4))/2 - (3*dq3*np.sin(q4))/2 - (3*dq4*np.sin(q4))/2 - (dq5*np.sin(q5))/2 - (dq1*np.sin(q3 + q4 + q5))/2 - (dq2*np.sin(q3 + q4 + q5))/2 - (dq3*np.sin(q3 + q4 + q5))/2 - (dq4*np.sin(q3 + q4 + q5))/2 - (dq5*np.sin(q3 + q4 + q5))/2
    C[1, 4] = -((np.sin(q3 + q4 + q5) + np.sin(q4 + q5) + np.sin(q5))*(dq1 + dq2 + dq3 + dq4 + dq5))/2
    C[2, 0] = (dq1*np.sin(q2 + q3 + q4 + q5))/2 + (5*dq1*np.sin(q2 + q3))/2 + (3*dq1*np.sin(q3 + q4))/2 + (3*dq2*np.sin(q3 + q4))/2 - (dq4*np.sin(q4 + q5))/2 - (dq5*np.sin(q4 + q5))/2 + (5*dq1*np.sin(q3))/2 + (5*dq2*np.sin(q3))/2 - (3*dq4*np.sin(q4))/2 - (dq5*np.sin(q5))/2 + (3*dq1*np.sin(q2 + q3 + q4))/2 + (dq1*np.sin(q3 + q4 + q5))/2 + (dq2*np.sin(q3 + q4 + q5))/2
    C[2, 1] = (3*dq1*np.sin(q3 + q4))/2 + (3*dq2*np.sin(q3 + q4))/2 - (dq4*np.sin(q4 + q5))/2 - (dq5*np.sin(q4 + q5))/2 + (5*dq1*np.sin(q3))/2 + (5*dq2*np.sin(q3))/2 - (3*dq4*np.sin(q4))/2 - (dq5*np.sin(q5))/2 + (dq1*np.sin(q3 + q4 + q5))/2 + (dq2*np.sin(q3 + q4 + q5))/2
    C[2, 2] = - (dq4*np.sin(q4 + q5))/2 - (dq5*np.sin(q4 + q5))/2 - (3*dq4*np.sin(q4))/2 - (dq5*np.sin(q5))/2
    C[2, 3] = - (dq1*np.sin(q4 + q5))/2 - (dq2*np.sin(q4 + q5))/2 - (dq3*np.sin(q4 + q5))/2 - (dq4*np.sin(q4 + q5))/2 - (dq5*np.sin(q4 + q5))/2 - (3*dq1*np.sin(q4))/2 - (3*dq2*np.sin(q4))/2 - (3*dq3*np.sin(q4))/2 - (3*dq4*np.sin(q4))/2 - (dq5*np.sin(q5))/2
    C[2, 4] = -((np.sin(q4 + q5) + np.sin(q5))*(dq1 + dq2 + dq3 + dq4 + dq5))/2
    C[3, 0] = (dq1*np.sin(q2 + q3 + q4 + q5))/2 + (3*dq1*np.sin(q3 + q4))/2 + (3*dq2*np.sin(q3 + q4))/2 + (dq1*np.sin(q4 + q5))/2 + (dq2*np.sin(q4 + q5))/2 + (dq3*np.sin(q4 + q5))/2 + (3*dq1*np.sin(q4))/2 + (3*dq2*np.sin(q4))/2 + (3*dq3*np.sin(q4))/2 - (dq5*np.sin(q5))/2 + (3*dq1*np.sin(q2 + q3 + q4))/2 + (dq1*np.sin(q3 + q4 + q5))/2 + (dq2*np.sin(q3 + q4 + q5))/2
    C[3, 1] = (3*dq1*np.sin(q3 + q4))/2 + (3*dq2*np.sin(q3 + q4))/2 + (dq1*np.sin(q4 + q5))/2 + (dq2*np.sin(q4 + q5))/2 + (dq3*np.sin(q4 + q5))/2 + (3*dq1*np.sin(q4))/2 + (3*dq2*np.sin(q4))/2 + (3*dq3*np.sin(q4))/2 - (dq5*np.sin(q5))/2 + (dq1*np.sin(q3 + q4 + q5))/2 + (dq2*np.sin(q3 + q4 + q5))/2
    C[3, 2] = (dq1*np.sin(q4 + q5))/2 + (dq2*np.sin(q4 + q5))/2 + (dq3*np.sin(q4 + q5))/2 + (3*dq1*np.sin(q4))/2 + (3*dq2*np.sin(q4))/2 + (3*dq3*np.sin(q4))/2 - (dq5*np.sin(q5))/2
    C[3, 3] = -(dq5*np.sin(q5))/2
    C[3, 4] = -(np.sin(q5)*(dq1 + dq2 + dq3 + dq4 + dq5))/2
    C[4, 0] = (dq1*np.sin(q2 + q3 + q4 + q5))/2 + (dq1*np.sin(q4 + q5))/2 + (dq2*np.sin(q4 + q5))/2 + (dq3*np.sin(q4 + q5))/2 + (dq1*np.sin(q5))/2 + (dq2*np.sin(q5))/2 + (dq3*np.sin(q5))/2 + (dq4*np.sin(q5))/2 + (dq1*np.sin(q3 + q4 + q5))/2 + (dq2*np.sin(q3 + q4 + q5))/2
    C[4, 1] = (dq1*np.sin(q4 + q5))/2 + (dq2*np.sin(q4 + q5))/2 + (dq3*np.sin(q4 + q5))/2 + (dq1*np.sin(q5))/2 + (dq2*np.sin(q5))/2 + (dq3*np.sin(q5))/2 + (dq4*np.sin(q5))/2 + (dq1*np.sin(q3 + q4 + q5))/2 + (dq2*np.sin(q3 + q4 + q5))/2
    C[4, 2] = (dq1*np.sin(q4 + q5))/2 + (dq2*np.sin(q4 + q5))/2 + (dq3*np.sin(q4 + q5))/2 + (dq1*np.sin(q5))/2 + (dq2*np.sin(q5))/2 + (dq3*np.sin(q5))/2 + (dq4*np.sin(q5))/2
    C[4, 3] = (np.sin(q5)*(dq1 + dq2 + dq3 + dq4))/2
    C[4, 4] = 0

    return C 

def get5DOF_dJ( qpos, qvel ):
    q1 = qpos[ 0 ]
    q2 = qpos[ 1 ]
    q3 = qpos[ 2 ]
    q4 = qpos[ 3 ]
    q5 = qpos[ 4 ]

    dq1 = qvel[ 0 ]
    dq2 = qvel[ 1 ]
    dq3 = qvel[ 2 ]
    dq4 = qvel[ 3 ]
    dq5 = qvel[ 4 ]

    dJ = np.zeros( ( 3, 5 ) )

    dJ[0, 0] = - np.cos(q1 + q2 + q3)*(dq1 + dq2 + dq3) - np.cos(q1 + q2)*(dq1 + dq2) - dq1*np.cos(q1) - np.cos(q1 + q2 + q3 + q4 + q5)*(dq1 + dq2 + dq3 + dq4 + dq5) - np.cos(q1 + q2 + q3 + q4)*(dq1 + dq2 + dq3 + dq4)
    dJ[0, 1] = - np.cos(q1 + q2 + q3)*(dq1 + dq2 + dq3) - np.cos(q1 + q2)*(dq1 + dq2) - np.cos(q1 + q2 + q3 + q4 + q5)*(dq1 + dq2 + dq3 + dq4 + dq5) - np.cos(q1 + q2 + q3 + q4)*(dq1 + dq2 + dq3 + dq4)
    dJ[0, 2] = - np.cos(q1 + q2 + q3)*(dq1 + dq2 + dq3) - np.cos(q1 + q2 + q3 + q4 + q5)*(dq1 + dq2 + dq3 + dq4 + dq5) - np.cos(q1 + q2 + q3 + q4)*(dq1 + dq2 + dq3 + dq4)
    dJ[0, 3] = - np.cos(q1 + q2 + q3 + q4 + q5)*(dq1 + dq2 + dq3 + dq4 + dq5) - np.cos(q1 + q2 + q3 + q4)*(dq1 + dq2 + dq3 + dq4)
    dJ[0, 4] = -np.cos(q1 + q2 + q3 + q4 + q5)*(dq1 + dq2 + dq3 + dq4 + dq5)
    dJ[1, 0] = - np.sin(q1 + q2 + q3)*(dq1 + dq2 + dq3) - np.sin(q1 + q2)*(dq1 + dq2) - dq1*np.sin(q1) - np.sin(q1 + q2 + q3 + q4 + q5)*(dq1 + dq2 + dq3 + dq4 + dq5) - np.sin(q1 + q2 + q3 + q4)*(dq1 + dq2 + dq3 + dq4)
    dJ[1, 1] = - np.sin(q1 + q2 + q3)*(dq1 + dq2 + dq3) - np.sin(q1 + q2)*(dq1 + dq2) - np.sin(q1 + q2 + q3 + q4 + q5)*(dq1 + dq2 + dq3 + dq4 + dq5) - np.sin(q1 + q2 + q3 + q4)*(dq1 + dq2 + dq3 + dq4)
    dJ[1, 2] = - np.sin(q1 + q2 + q3)*(dq1 + dq2 + dq3) - np.sin(q1 + q2 + q3 + q4 + q5)*(dq1 + dq2 + dq3 + dq4 + dq5) - np.sin(q1 + q2 + q3 + q4)*(dq1 + dq2 + dq3 + dq4)
    dJ[1, 3] = - np.sin(q1 + q2 + q3 + q4 + q5)*(dq1 + dq2 + dq3 + dq4 + dq5) - np.sin(q1 + q2 + q3 + q4)*(dq1 + dq2 + dq3 + dq4)
    dJ[1, 4] = -np.sin(q1 + q2 + q3 + q4 + q5)*(dq1 + dq2 + dq3 + dq4 + dq5)
    dJ[2, 0] = 0
    dJ[2, 1] = 0
    dJ[2, 2] = 0
    dJ[2, 3] = 0
    dJ[2, 4] = 0

    return dJ
        