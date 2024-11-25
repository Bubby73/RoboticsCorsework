import numpy as np
import matplotlib.pyplot as plt
import random
import spatialmath as sp
import roboticstoolbox as rtb


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
    DH = np.array([[L0, 0, 0, 0], # 0
                    [0, 0, 0, theta1], # 1
                    [D2, 0, 0, 0], # 2
                    [D3, 0, alpha3, 0], # 3
                    [L1, 0, 0, theta4], # 4
                    [L5, L2, 0, theta5], # 5    
                    [L3, 0, alpha6, theta6], # 6
                    [L4, 0, 0, 0]]) # T
    return DH

DH = InitialiseDH(alpha3, alpha6, D2, D3, L0, L1, L2, L3, L4, L5, theta1, theta4, theta5, theta6)


robot = rtb.SerialLink(
    [
        rtb
    ]
)
 
# Forward Kinematics Calculations

def forwardKinematics(DH):

    def TransformMatrix(i):

       
    return T0_6
    

T0_6 = forwardKinematics(DH)
# Print out the Base to EE Transformation Matrix
print ('\n', 'Base to End-Effector Homogeneous Transformation Matrix (T0_6):')
print (T0_6)

