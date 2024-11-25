import numpy as np
import matplotlib.pyplot as plt
import random
import spatialmath as sp

# Define the DH parameters
alpha3 = alpha6 = np.radians(90) # Link twist angles
D2 = 0.5
D3 = 0
L0 = L4 = 0
L1 = 0.2
L2 = L3 = 0.3
L5 = 0.05
theta1 = theta4 = theta5 = theta6 = np.radians(0) # Joint Angles initially set to 0

def InitialiseDH(alpha3, alpha6, D2, D3, L0, L1, L2, L3, L4, L5, theta1, theta4, theta5, theta6):
    # The D-H Table is as follows aplha, a, d, theta:
    DH = np.array([[0, 0, L0, 0], # 0
                    [0, 0, 0, theta1], # 1
                    [0, 0, D2, 0], # 2
                    [alpha3, 0, D3, 0], # 3
                    [0, 0, L1, theta4], # 4
                    [0, L2, L5, theta5], # 5    
                    [alpha6, 0, L3, theta6], # 6
                    [0, 0, L4, 0]]) # T
    return DH

DH = InitialiseDH(alpha3, alpha6, D2, D3, L0, L1, L2, L3, L4, L5, theta1, theta4, theta5, theta6)
 
# Forward Kinematics Calculations

def forwardKinematics(DH):

    def TransformMatrix(i):
        return np.array([[np.cos(DH[i, 3]), -np.sin(DH[i, 3]) * np.cos(DH[i, 0]), np.sin(DH[i, 3]) * np.sin(DH[i, 0]), DH[i, 1] * np.cos(DH[i, 3])], 
                        [np.sin(DH[i, 3]), np.cos(DH[i, 3]) * np.cos(DH[i, 0]), -np.cos(DH[i, 3]) * np.sin(DH[i, 0]), DH[i, 1] * np.sin(DH[i, 3])], 
                        [0, np.sin(DH[i, 0]), np.cos(DH[i, 0]), DH[i, 2]], 
                        [0, 0, 0, 1]])
    

    T0_1 = TransformMatrix(0)
    T1_2 = TransformMatrix(1)
    T2_3 = TransformMatrix(2)
    T3_4 = TransformMatrix(3)
    T4_5 = TransformMatrix(4)
    T5_6 = TransformMatrix(5)
    T6_T = TransformMatrix(6)
    print(T0_1, ("\n"), T1_2, ("\n"), T2_3, ("\n"), T3_4, ("\n"), T4_5, ("\n"), T5_6, ("\n"), T6_T)


    #The Transformation matrix from the Base to the End Effector of the Robot
    T0_6 = T0_1.dot(T1_2).dot(T2_3).dot(T3_4).dot(T4_5).dot(T5_6).dot(T6_T) # dot performs multiplication on the Matrices
    return T0_6
    

T0_6 = forwardKinematics(DH)
# Print out the Base to EE Transformation Matrix
print ('\n', 'Base to End-Effector Homogeneous Transformation Matrix (T0_6):')
print (T0_6)

