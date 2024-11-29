import numpy as np
import matplotlib.pyplot as plt
import random
import spatialmath as sp
import roboticstoolbox as rtb


# Link twist angles
alpha3 = alpha6 = np.radians(90) 

# Lengths and offsets
L0 = L4 = 0
L1 = 0.2
L2 = L3 = 0.3
L5 = 0.05

# Initial joint angles
theta1 = theta4 = theta5 = theta6 = np.radians(0)

# Prismatic joint displacements
d2 = 0.5
d3 = 0

def InitialiseDH(alpha3, alpha6, L0, L1, L2, L3, L4, L5):
    # The D-H Table is as follows d, a, alpha, theta:
    DH = np.array([[L0, 0, 0, 0], # 0
                    [0, 0, 0, 0], # 1
                    [0, 0, 0, 0], # 2
                    [0, 0, alpha3, 0], # 3
                    [L1, 0, 0, 0], # 4
                    [L5, L2, 0, 0], # 5    
                    [L3, 0, alpha6, 0], # 6
                    [L4, 0, 0, 0]]) # T
    return DH

DH = InitialiseDH(alpha3, alpha6, L0, L1, L2, L3, L4, L5)

# Define D-H links (d, a, alpha, theta)
links = [
    rtb.RevoluteDH(d=L0, a=0, alpha=0),            # Joint 1 (Revolute)
    rtb.PrismaticDH(theta=0, a=0, alpha=0),                 # Joint 2 (Prismatic)
    rtb.PrismaticDH(theta=0, a=0, alpha=DH[3,2]),            # Joint 3 (Prismatic)
    rtb.RevoluteDH(d=DH[4,0], a=0, alpha=0),            # Joint 4 (Revolute)
    rtb.RevoluteDH(d=DH[5,0], a=L2, alpha=0),           # Joint 5 (Revolute)
    rtb.RevoluteDH(d=DH[6,0], a=0, alpha=DH[6,2]),       # Joint 6 (Revolute)
    rtb.RevoluteDH(d=DH[7,0], a=0, alpha=0)             # End effector (Revolute)
]

# Create the robot model
robot = rtb.DHRobot(links, name="RPPRRR Robot")

# Joint configuration
q = [theta1, d2, d3, theta4, theta5, theta6, 0]  # [theta1, d2, d3, theta4, theta5, theta6]

# Forward kinematics
T = robot.fkine(q)
print(T)

# Plot the robot's configuration
robot.plot(q, block=True)




