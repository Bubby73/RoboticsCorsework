import numpy as np
import matplotlib.pyplot as plt
import random
import spatialmath as sp
import roboticstoolbox as rtb
import sympy as sp


def dh_transform(a, alpha, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha),  np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0,              np.sin(alpha),                 np.cos(alpha),                d],
        [0,              0,                             0,                            1]
    ])


def forward_kinematics(joint_values, link_lengths):
    """
    Calculate the forward kinematics for an RPPRRR robot.
    
    joint_values: [theta1, d2, d3, theta4, theta5, theta6]
    link_lengths: [L0, L1, L2, L3, L4, L5, alpha3, alpha6]
    """
    # Extract joint values and link lengths
    theta1, d2, d3, theta4, theta5, theta6 = joint_values
    L0, L1, L2, L3, L4, L5, alpha3, alpha6 = link_lengths

    # Define individual transformation matrices based on DH parameters
    T01 = dh_transform(0, 0, L0, theta1)
    T12 = dh_transform(0, 0, d2, 0)
    T23 = dh_transform(0, 0, d3, 0)
    T34 = dh_transform(0, alpha3, 0, theta4)
    T45 = dh_transform(L1, 0, 0, theta5)
    T56 = dh_transform(L2, 0, L5, 0)
    T67 = dh_transform(L3, alpha6, 0, theta6)
    T7T = dh_transform(L4, 0, 0, 0)

    # Compute the cumulative transformation from base to end-effector
    T = T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ T67 @ T7T
    T_rounded = np.round(T, 4)
    return T_rounded


# Example joint values (in radians for revolute joints and meters for prismatic joints)
joint_values = [0, 0.5, 0, 0, 0, 0]

# Example link lengths
link_lengths = [0.1, 0.2, 0.3, 0.3, 0.1, 0.05, np.pi/2, np.pi/2]

# Calculate the forward kinematics
T_end_effector = forward_kinematics(joint_values, link_lengths)
print("End-effector Transformation Matrix:")
print(T_end_effector)






