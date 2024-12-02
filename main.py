import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
import matplotlib.pyplot as plt

# Define symbolic parameters for link lengths and angles
L0, L1, L2, L3, L4, L5 = 0.1, 0.2, 0.3, 0.3, 0.1, 0.05 
alpha3 = alpha6 = np.pi/2                    
theta1, d2, d3, theta4, theta5, theta6 = 0, 0.5, 0, 0, 0, 0  # Initial joint variables

target_1 = [[1, 0, 0, -0.3],
            [0, -1, 0, -0.25],
            [0, 0, -1, 0.2],
            [0, 0, 0, 1]]

target_2 = [[-0.8419, 0, 0.5396, 0.7684],
            [0, -1, 0, -0.25],
            [0.5396, 0, -0.8419, 1.925],
            [0, 0, 0, 1]]

target_3 = [[-0.0023, -1, 0.1, -0.257],
            [-0.002, 1, 0.2, -0.299],
            [-1, 0, -1, 3.34],
            [0, 0, 0, 1]]





# Define DH table (Alpha, d, a, Theta)
DH = [
    [0, L0, 0, 0],  # Base (Revolute)
    [0, 0, 0, theta1],   # Joint 1 (Revolute)
    [0, d2, 0, 0],       # Joint 2 (Prismatic)
    [alpha3, d3, 0, 0],       # Joint 3 (Prismatic)
    [0, L1, 0, theta4], # Joint 4 (Revolute)
    [0, L5, L2, theta5],  # Joint 5 (Revolute)
    [alpha6, L3, 0, theta6],  # Joint 6 (Revolute)
    [0, L4, 0, 0]        # End-effector
]


# Define the robot using the DH convention
robot = rtb.DHRobot([
    rtb.RevoluteMDH(a = DH[1][2], alpha = DH[1][0], d = DH[1][1], qlim=[-np.pi, np.pi]),       # Joint 1
    rtb.PrismaticMDH(a = DH[2][2], alpha = DH[2][0], theta = DH[2][3], qlim=[0, 0.5]),   # Joint 2
    rtb.PrismaticMDH(a = DH[3][2], alpha = DH[3][0], theta = DH[3][3], qlim=[-0.1, 0.1]),   # Joint 3
    rtb.RevoluteMDH(a = DH[4][2], alpha = DH[4][0], d = DH[4][1], qlim=[-np.pi/2, np.pi/2]),   # Joint 4
    rtb.RevoluteMDH(a = DH[5][2], alpha = DH[5][0], d = DH[5][1], qlim=[-np.pi, np.pi]),       # Joint 5
    rtb.RevoluteMDH(a = DH[6][2], alpha = DH[6][0], d = DH[6][1], qlim=[-np.pi/2, np.pi/2]),   # Joint 6
], name="RPPRRR_Robot")



# T_B0 = SE3.Trans(0, 0, L0) # Base frame
# T_6T = SE3.Trans(0, 0, L4) # Tool frame

robot.base = SE3(0, 0, L0)
robot.tool = SE3(0, 0, L4)


# Define example joint values (modify based on your needs)
joint_values = [theta1, d2, d3, theta4, theta5, theta6]  

# Function to compute and print link poses {Part 1}
def computeLinkPoses(robot, joint_values):
    T_base = robot.base

    T1 = robot[0].A(joint_values[0])
    T_base_1 = T_base * T1
    print("\nTransformation matrix from base to joint 1:\n", T_base_1)

    T2 = robot[1].A(joint_values[1])
    T_base_2 = T_base_1 * T2
    print("\nTransformation matrix from base to joint 2:\n", T_base_2)

    T3 = robot[2].A(joint_values[2])
    T_base_3 = T_base_2 * T3
    print("\nTransformation matrix from base to joint 3:\n", T_base_3)

    T4 = robot[3].A(joint_values[3])
    T_base_4 = T_base_3 * T4
    print("\nTransformation matrix from base to joint 4:\n", T_base_4)

    T5 = robot[4].A(joint_values[4])
    T_base_5 = T_base_4 * T5
    print("\nTransformation matrix from base to joint 5:\n", T_base_5)

    T6 = robot[5].A(joint_values[5])
    T_base_6 = T_base_5 * T6
    print("\nTransformation matrix from base to joint 6:\n", T_base_6)

    T_tool = robot.tool
    T_base_tool = T_base_6 * T_tool
    print("\nTransformation matrix from base to tool:\n", T_base_tool)


# Function to compute and print inverse kinematics solution(s) {Part 2}
def compute_inverse_kinematics(robot, target_pose, guess_values):
    target_SE3 = SE3(target_pose) # Convert target pose to SE3 format
    q_initial = guess_values # Initial guess for the joint values
    solutions = robot.ikine_LM(target_SE3, q0 = q_initial, joint_limits=True) # Compute inverse kinematics

    roundedSol = np.round(solutions.q, 3) # round to 3 decimal places
    
    if solutions.success: # Check if a solution is found
        print("Inverse kinematics solution found within joint limits:")
        print("Theta 1: ", roundedSol[0], "d2: ", roundedSol[1], "d3: ", 
              roundedSol[2], "Theta 4: ", roundedSol[3], "Theta 5: ", 
              roundedSol[4], "Theta 6: ", roundedSol[5])
        
    else:
        print('\033[31mInverse kinematics solution within join limits not found\033[0m') # Print in red colour
        print("Closest solution found:")
        print("Theta 1: ", roundedSol[0], "d2: ", roundedSol[1], "d3: ", 
              roundedSol[2], "Theta 4: ", roundedSol[3], "Theta 5: ", 
              roundedSol[4], "Theta 6: ", roundedSol[5])



# Define joint values and velocities {Part 3}
joint_values_v = [np.radians(-60), 0.4, 0.1, np.radians(90), np.radians(180), np.radians(90)]
joint_velocities = [np.radians(15), 0, 0.1, np.radians(-30), np.radians(15), np.radians(10)]




# Outputs

print(robot)

# {Part 1}
# Compute forward kinematics
print("\nForward kinematics:")
T = robot.base * robot.fkine(joint_values) * robot.tool
print(T)

print("\nLink poses:")
# computeLinkPoses(robot, joint_values) # Compute link poses

# {Part 2}
print("\nInverse kinematics for each target:")
# Compute inverse kinematics for the target poses
guess_1 = [0, 0, 0, 0, 0, 0] # Initial guess for inverse kinematics
print("Initial guess for inverse kinematics: ", guess_1)
print("\nResult for target 1:")
compute_inverse_kinematics(robot, target_1, guess_1)
print("\nResult for target 2:")
compute_inverse_kinematics(robot, target_2, guess_1)
print("\nResult for target 3:")
compute_inverse_kinematics(robot, target_3, guess_1)


# Plot the robot
# robot.plot(joint_values, block=True)
# plt.show()
