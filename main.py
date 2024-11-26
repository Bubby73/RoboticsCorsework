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
    # The D-H Table is as follows d, a, alpha, theta:
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
        rtb.RevoluteDH(d=DH[1][0], a = DH[1][1], alpha = DH[1][2], offset = DH[1][3]),
        rtb.PrismaticDH(d = DH[2][0], a = DH[2][1], alpha = DH[2][2], offset = DH[2][3]),
        rtb.PrismaticDH(d = DH[3][0], a = DH[3][1], alpha = DH[3][2], offset = DH[3][3]),
        rtb.RevoluteDH(d=DH[4][0], a = DH[4][1], alpha = DH[4][2], offset = DH[4][3]),
        rtb.RevoluteDH(d=DH[5][0], a = DH[5][1], alpha = DH[5][2], offset = DH[5][3]),
        rtb.RevoluteDH(d=DH[6][0], a = DH[6][1], alpha = DH[6][2], offset = DH[6][3]),
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

