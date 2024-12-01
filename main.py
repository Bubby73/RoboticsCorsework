import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
import matplotlib.pyplot as plt

# Define symbolic parameters for link lengths and angles
L0, L1, L2, L3, L4, L5 = 0.1, 0.2, 0.3, 0.3, 0.1, 0.05 
alpha3 = alpha6 = np.pi/2                    
theta1, d2, d3, theta4, theta5, theta6 = 0, 0.5, 0, 0, 0, 0  # Initial joint variables

# Define DH table based on the joint type (R for revolute, P for prismatic)
DH = [
    [0, L0, 0, 0],  # Base (Revolute)
    [0, 0, 0, theta1],   # Joint 1 (Revolute)
    [0, d2, 0, 0],       # Joint 2 (Prismatic)
    [0, d3, 0, 0],       # Joint 3 (Prismatic)
    [alpha3, 0, 0, theta4], # Joint 4 (Revolute)
    [0, L1, 0, theta5],  # Joint 5 (Revolute)
    [0, L2, 0, theta6],  # Joint 6 (Revolute)
]

# Define the robot using the DH convention
robot = rtb.DHRobot([
    rtb.RevoluteDH(a=0, alpha=0, d=0),       # Joint 1
    rtb.PrismaticDH(a=0, alpha=0, theta=0),   # Joint 2
    rtb.PrismaticDH(a=0, alpha=alpha3, theta=0),   # Joint 3
    rtb.RevoluteDH(a=0, alpha=0, d=L1),   # Joint 4
    rtb.RevoluteDH(a=L2, alpha=0, d=L5),       # Joint 5
    rtb.RevoluteDH(a=0, alpha=alpha6, d=L3),   # Joint 6
    rtb.RevoluteDH(a=0, alpha=0, d=L4)    # End-effector
], name="RPPRRR_Robot")

print(robot)

# Define example joint values (modify based on your needs)
joint_values = [theta1, d2, d3, theta4, theta5, theta6, 0]  

# Compute forward kinematics
T = robot.fkine(joint_values)
print(T)
# Plot the robot using an example configuration
robot.plot(joint_values, block=True)

plt.show()
