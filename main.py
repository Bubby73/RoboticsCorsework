# Import the necessary libraries
import numpy as np
from numpy import random
import roboticstoolbox as rtb
from spatialmath import SE3
import matplotlib.pyplot as plt


# Define all variables used in the code including join variables, target poses and transformation matricies

# Define symbolic parameters for link lengths and angles
L0, L1, L2, L3, L4, L5 = 0.1, 0.2, 0.3, 0.3, 0.1, 0.05 
alpha3 = alpha6 = np.pi/2                    
theta1, d2, d3, theta4, theta5, theta6 = 0, 0.5, 0, 0, 0, 0  # Initial joint variables

# Define DH table (Alpha, d, a, Theta)
DH = [
    [0, L0, 0, 0],  # Base (Revolute)
    [0, 0, 0, 0],   # Joint 1 (Revolute)
    [0, 0, 0, 0],       # Joint 2 (Prismatic)
    [alpha3, 0, 0, 0],       # Joint 3 (Prismatic)
    [0, L1, 0, 0], # Joint 4 (Revolute)
    [0, L5, L2, 0],  # Joint 5 (Revolute)
    [alpha6, L3, 0, 0],  # Joint 6 (Revolute)
    [0, L4, 0, 0]        # End-effector
]

# Define target poses {Part 2}
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


# Define the transformation matrix from the base to the tool in Cartesian space {Part 3}
btT_f = np.array([[0.6791, -0.6403, 0.359, -0.4475],
         [-0.6403, 0.2775, -0.7162, -0.335],
         [0.359, 0.7162, 0.5985, 0.599],
         [0, 0, 0, 1]])

mass = 0.2 # Define mass and gravity constants
g = 9.81 


# Define parameters for part 4, centres of mass, link masses, tensor manipulator links, load, {Part 4}
CoMs = [[0, 0, 0.5], [0, 0, 0.3], [0.0526, -1.526e-4, 0.4998], 
        [0.0526, -1.526e-4, 0.4998], [0.2203, 0.1271, 0.4761], 
        [0.2208, 0.2812, 0.2578], [0.2207, 0.2671, 0.0583]]

linkMasses = [6.16, 9.81, 4.767, 4.767, 3.7632, 1.96, 0]

inertial_Manipulator_links = [[[0.0224, 0, 0], [0, 0.0224, 0], [0, 0, 0.0077]],
                              [[1.088, 0, 0], [0, 1.0882, 0], [0, 0, 0.004]],
                              [[1.1932, 0.0009, -0.1254], [0.0009, 1.2268, 0.0003], [-0.1254, 0.0003, 0.0357]],
                              [[1.1932, 0.0009, -0.1254], [0.0009, 1.2268, 0.0003], [-0.1254, 0.0003, 0.0357]],
                              [[0.9429, -0.1055, -0.3949], [-0.1055, 1.038, -0.2229], [-0.3949,-0.2229, 0.2714]],
                              [[0.3116, -0.1217, -0.1116], [-0.1217, 0.252, -0.144], [-0.1116, -0.144, 0.2509]],
                              [[0.0110, -0.0087, -0.0019], [-0.0087, 0.0077, -0.0023], [-0.0019, -0.0023, 0.00177]]]

load = 0.2

#pose_d = [pi.]

# Define the robot taking values from the DH table
robot = rtb.DHRobot([
    rtb.RevoluteMDH(a = DH[1][2], alpha = DH[1][0], d = DH[1][1], qlim=[-np.pi, np.pi]),       # Joint 1
    rtb.PrismaticMDH(a = DH[2][2], alpha = DH[2][0], theta = DH[2][3], qlim=[0, 0.5]),   # Joint 2
    rtb.PrismaticMDH(a = DH[3][2], alpha = DH[3][0], theta = DH[3][3], qlim=[-0.1, 0.1]),   # Joint 3
    rtb.RevoluteMDH(a = DH[4][2], alpha = DH[4][0], d = DH[4][1], qlim=[-np.pi/2, np.pi/2]),   # Joint 4
    rtb.RevoluteMDH(a = DH[5][2], alpha = DH[5][0], d = DH[5][1], qlim=[-np.pi, np.pi]),       # Joint 5
    rtb.RevoluteMDH(a = DH[6][2], alpha = DH[6][0], d = DH[6][1], qlim=[-np.pi/2, np.pi/2]),   # Joint 6
], name="RPPRRR_Robot")

robot.base = SE3(0, 0, L0) # Set the base of the robot
robot.tool = SE3(0, 0, L4) # Set the tool of the robot


# Define example joint values {Part 1}
joint_values = [theta1, d2, d3, theta4, theta5, theta6]  

# Function to compute and print link poses
def computeLinkPoses(robot, joint_values):
    T_Pose = robot.base # start from the robot base
    for i in range(len(joint_values)):
        Tn = robot[i].A(joint_values[i]) # Calcuate the DH transformation matrix from the givern joint angle
        T_Pose = T_Pose * Tn
        print("\nTransformation matrix from base to joint ",i+1,":\n", T_Pose) # Output link pose relative to the robot base


    T_tool = robot.tool # end effector is calculated seperatly
    T_base_tool = T_Pose * T_tool
    print("\nTransformation matrix from base to tool:\n", T_base_tool)

# Compute the forward kinematics
T = robot.base * robot.fkine(joint_values) * robot.tool


# Function to compute and print inverse kinematics solution(s) {Part 2}
def compute_inverse_kinematics(robot, target_pose, guess_values):
    target_SE3 = SE3(target_pose) # Convert target pose to SE3 format
    for i in range(len(guess_values)):
        q_initial = guess_values[i] # Initial guess for the joint values
        solutions = robot.ikine_LM(target_SE3, q0 = q_initial, joint_limits=True) # Compute inverse kinematics
        clippedSol = np.clip(solutions.q, robot.qlim[0], robot.qlim[1]) # Clip joints to their limits
        roundedSol = np.round([np.degrees(clippedSol[0]), clippedSol[1], clippedSol[2], np.degrees(clippedSol[3]),
                                np.degrees(clippedSol[4]), np.degrees(clippedSol[5])], 3) # round to 3 decimal places and convert radians to degrees
        
        if solutions.success: # Check if a solution is found
            print('\033[32mInverse kinematics solution within join limits not found for guess:\033[0m',i + 1)
            print("\tTheta 1: ", roundedSol[0], "\td2: ", roundedSol[1], "\td3: ", 
                roundedSol[2], "\tTheta 4: ", roundedSol[3], "\tTheta 5: ", 
                roundedSol[4], "\tTheta 6: ", roundedSol[5])
            
        else:
            print('\033[31mInverse kinematics solution within join limits not found for guess:\033[0m',i + 1)
            print("Closest solution found (clipped):\tTheta 1: ", roundedSol[0], "\td2: ", roundedSol[1], "\td3: ", # Print clipped solutions
                roundedSol[2], "\tTheta 4: ", roundedSol[3], "\tTheta 5: ", 
                roundedSol[4], "\tTheta 6: ", roundedSol[5])



# Define joint values and velocities {Part 3}
joint_values_v = [np.radians(-60), 0.4, 0.1, np.radians(90), np.radians(180), np.radians(90)]
joint_velocities = [np.radians(15), 0, 0.1, np.radians(-30), np.radians(15), np.radians(10)]



# Compute the Jacobian in the base frame
def jacobians(robot, pose, mass, g):
    J = robot.jacob0(joint_values_v) # Calculate the jacobian matrix relative to the robot base
    print("\nJacobian matrix in the base frame of the robot:\n", np.round(J, 3))

    # Compute the end-effector velocities (linear and angular)
    velocities = J @ joint_velocities # matricies multiplied to find velocities
    linear_velocity = velocities[:3] # Extract linear velocity
    angular_velocity = velocities[3:] # Extract angular velocity
    print("\nEnd-effector linear velocities  [X, Y, Z]:\n", np.round(linear_velocity, 3))
    print("\nEnd-effector angular velocities [Wx, Wy,Wz]:\n", np.round(angular_velocity, 3))

    J = robot.jacob0(robot.ikine_LM(pose).q) # find the inverse kinematics of the static pose, then find the jacobian matrix
    F_ee = mass * np.array([0, 0, -g, 0, 0, 0]) # Compute the end-effector force
    tau = J.T @ F_ee # Compute the joint torques
    print("Joint torques/forces due to static load:", np.round(tau, 3))





# Outputs

print(robot)

# {Part 1}
# Compute forward kinematics
print("\nForward kinematics:\n", T)

print("\nLink poses:")
computeLinkPoses(robot, joint_values) # Compute link poses

# Plot the robot
robot.plot(joint_values, block=True)
plt.show()


# {Part 2}
print("\nInverse kinematics for each target:")
# Compute inverse kinematics for the target poses
# Generate 5 random guesses for the joint values
guess = []
guesses = []
for i in range(10):
    for i in range(6):
        guess.append(random.rand()) # append a random float between 0 and 1
    guesses.append(guess)
    guess = []

print("\nResult for target 1:")
compute_inverse_kinematics(robot, target_1, guesses)
print("\nResult for target 2:")
compute_inverse_kinematics(robot, target_2, guesses)
print("\nResult for target 3:")
compute_inverse_kinematics(robot, target_3, guesses)


# Jacobians, Velocity and Static Force Analysis {Part 3} 
jacobians(robot, btT_f, mass, g) 


