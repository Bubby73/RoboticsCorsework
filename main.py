import numpy as np
import matplotlib.pyplot as plt
import random
import spatialmath as sp

# Define the DH parameters
theta1 = np.radians(30)
theta2 = np.radians(50)
L0 = 0
L1 = L2 = 0.5
# The D-H Table is as follows:
DH = np.array([[L0, 0, 0, theta1], [L1, 0, 0, theta2], [L2, 0, 0, 0]])

# Perform the forward kinematics operation by multiplying the Transformation matrices T0_1, T1_2, T2_3
T0_1 = np.array([[np.cos(DH[0, 3]), -np.sin(DH[0, 3]), 0, DH[0, 0]], [np.sin(DH[0, 3]), np.cos(DH[0, 3]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
T1_2 = np.array([[np.cos(DH[1, 3]), -np.sin(DH[1, 3]), 0, DH[1, 0]], [np.sin(DH[1, 3]), np.cos(DH[1, 3]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
T2_3 = np.array([[np.cos(DH[2, 3]), -np.sin(DH[2, 3]), 0, DH[2, 0]], [np.sin(DH[2, 3]), np.cos(DH[2, 3]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

#The Transformation matrix from the Base to the End Effector of the Robot
T0_3 = T0_1.dot(T1_2).dot(T2_3) # dot performs multiplication on the Matrices
# Print out the Base to EE Transformation Matrix
print ('\n', 'Base to End-Effector Homogeneous Transformation Matrix (T0_3):')
print (T0_3)

T0_32 = T0_1*T1_2*T2_3
print (T0_32)