# Require libraries
import numpy as np
import matplotlib.pyplot as plt
import swift
from spatialmath.base import *
from roboticstoolbox import models, jtraj, trapezoidal
from ir_support import RobotCow
from ir_support.plyprocess import *
from spatialgeometry import Sphere

# Useful variables
from math import pi

class Lab4Exercises:
    def __init__(self):
        print('Lab 4 Exercises - Questions 2, 3 (and extension) Starting Point')


    def question2(self):
        '''
        2) Inverse Kinematics to Determine Joint Angles of Puma 560
        '''
        # # 2. Inverse Kinematics to Determine Joint Angles of Puma 560
        # 2.1) Load a model of the Puma 560 robot
        p560 = models.Puma560()

        # 2.2) Define our first end-effector pose as a 4x4 Homogeneous Transformation Matrix:
        T1 = transl(0.5,-0.4,0.5)
        q0 = np.zeros([1,6]) 

        # 2.3) Solve the inverse kinematics to get the required joint angles
        # Hint: If confused on function + input parameters - inspect previous uses of ikine_LM
        # Note that the returned solution may not be within joint limits. In this case, you should try a more accurate q0 (guess)
        # E.g. You can use teach to estimate a joint solution for a cartesian XYZ location
        q1 = p560.ikine_LM(..., q0=...).q
        # Check if q1 within joint limits
        print(f'q1 = {q1}')
        q1_in_joint_limits = self.check_joint_limits(q1, p560)
        print(f"q1 within joint limits" if q1_in_joint_limits == True else f"q1 not within joint limits")

        # 2.4) Define the second end-effector pose as a 4x4 Homogeneous Transformation Matrix:
        T2 = transl(0.5,0.4,0.1)

        # 2.5) Solve the inverse kinematics to get the required joint angles
        # Hint: If confused on function + input parameters - inspect previous uses of ikine_LM
        q2 = ...
        # Check if q2 within joint limits
        print(f'q2 = {q2}')
        q2_in_joint_limits = self.check_joint_limits(q2, p560)
        print(f"q2 within joint limits" if q2_in_joint_limits == True else f"q2 not within joint limits\n")

        input(f'Press enter to continue to Question 3\n')
        
    
    def question3(self):
        '''
        3) Interpolation: Getting from A to B
        '''
        # Start a Swift Environment for simulation
        env = swift.Swift()
        env.launch(realtime=True)

        # 3.1) Download and read Lab 4 - Trajectory Interpolation Reference Guide.pdf
        # 3.2) Load a puma560 robot and add it to the Swift environment
        # Hint: The Puma Robot was created in Question 2
        p560 = ...
        env.add(...)

        # 3.3) Generate a matrix of interpolated joint angles with 50 steps between q1 and q2 generated from Question 2.
        # Use (uncomment) either of the following two methods then do steps 3.4 to 3.10 for each method
        steps = ...
        q1 = ...
        q2 = ...
        T2 = transl(0.5,0.4,0.1)

        # Method 1: Quintic polynomial
        # q_matrix = jtraj(q1, q2, steps).q

        # Method 2: Trapezoidal Velocity Profile
        # s = trapezoidal(0,1,steps).q
        # q_matrix = np.zeros([steps,6])
        # for i in range(steps):
        #     q_matrix[i,:] =  (1 - s[i]) * q1 + s[i]*q2

        p560.q = q1     # Initialise robot in q1 joint state
        env.step(0.05)  # Update swift environment
        input('Press enter to continue.\n')

        # 3.4) Animate the generated trajectory and the Puma 560 robot. 
        # Note the red line of the end-effector trajectory.
        for q in q_matrix:
            p560.q = q      # Update robot joint state
            env.step(0.01)  # Update the environment

            # Get end-effector pose
            end_effector_tr = ...

            # Get X, Y, Z positions of end effector
            # Hint: If end_effector_tr.A is a 4x4 transformation matrix, position (XYZ) data is encoded in the first 3 rows of the 4th column
            x = end_effector_tr.A[0, 3]     # E.g. X data located in 1st row, 4th column (zero indexed -> 0, 3)
            y = end_effector_tr.A[..., ...]
            z = end_effector_tr.A[..., ...]

            # Create a sphere at the end-effector to visualise the trajectory in the environment. We will make it red with a radius of 0.05m
            new_point = Sphere(radius=0.05, color=[1.0, 0.0, 0.0, 1.0])     # Create sphere
            new_point.T = transl(x, y, z)                                   # Set pose to end-effector position
            env.add(new_point)                                              # Add to swift environment
            env.step(0.05)

        # Note the end position of the trajectory and compare it to the desired location
        final_ee_tr = p560.fkine(p560.q)
        print(f'End effector pose at end of trajectory: \n{final_ee_tr}')
        print(f'Position error (X, Y, Z) = {np.round(T2[0, 3] - final_ee_tr.A[0, 3], 3)}, {np.round(T2[1, 3] - final_ee_tr.A[1, 3], 3)}, {np.round(T2[2, 3] - final_ee_tr.A[2, 3], 3)}')
        input('Press enter to continue.\n')

        # 3.5) Create matrices of the joint velocities and accelerations for analysis:
        velocity = np.zeros([steps,6])
        acceleration = np.zeros([steps,6])
        for i in range(1,steps):
            velocity[i,:] = q_matrix[i,:] - q_matrix[i-1,:]
            acceleration[i,:] = velocity[i,:] - velocity[i-1,:]

        # 3.6) Plot the joint angles, velocities, and accelerations. 
        # When plotting the joint angles, we will use qlim to draw reference
        # lines for the upper and lower joint limits.

        qlim = np.transpose(p560.qlim)
        for i in range(1, 7):
            plt.figure(2)
            plt.subplot(3, 2, i)
            plt.plot(q_matrix[:, i-1], 'k', linewidth=1)
            plt.title('Joint ' + str(i))
            plt.xlabel('Step')
            plt.ylabel('Joint Angle (rad)')
            plt.axhline(y=qlim[i-1, 0], color='r')  # Reference line on the lower joint limit for joint i
            plt.axhline(y=qlim[i-1, 1], color='r')  # Reference line on the upper joint limit for joint i
            
            plt.figure(3)
            plt.subplot(3, 2, i)
            plt.plot(velocity[:, i-1], 'k', linewidth=1)
            plt.title('Joint ' + str(i))
            plt.xlabel('Step')
            plt.ylabel('Joint Velocity')

            plt.figure(4)
            plt.subplot(3, 2, i)
            plt.plot(acceleration[:, i-1], 'k', linewidth=1)
            plt.title('Joint ' + str(i))
            plt.xlabel('Step')
            plt.ylabel('Joint Acceleration')

        # 3.7) Consider the following:
        # Joint limitations – Certain inverse kinematic solutions will give infeasible joint angles
        # Joint velocities – appreciation for the difference between Quintic Polynomials and Trapezoidal Velocity Profiles
        # End-effector path – The end-effector will not follow a straight line from one pose to another

        input("Enter to finish\n")
        env.close()
        plt.close('all')
    

    def extension(self):
        '''
        Extension: Mount one robot onto another
        '''
        # Now that we do forward kinematics, and move a robot base, and we can generate trajectories. So let's do something silly but fun.
        # What about if we wanted to have a robot cow as a robot gripper?
        # We can create a UR5 and a RobotCow as follows:
        plt.close('all')
        ur5_robot = models.DH.UR5()
        fig = ur5_robot.plot([0,-pi/2,0,-pi/2,0,0])
        cow_herd = RobotCow(1,'surface')
        cow_herd.cow_list[0]['base'] = ur5_robot.fkine([0,-pi/2,0,-pi/2,0,0]).A
        cow_herd.animate(0) # input is the cow index
        plotvol3([-1.5,1.5])
        q_matrix = jtraj([0,-pi/2,0,-pi/2,0,0],[pi/4,pi/4-pi/2,pi/4,pi/4-pi/2,pi/4,pi/4],50).q

        input('Enter to continue\n')
        for q in q_matrix:
            ur5_robot.q = q
            fig.step(0.05)
            cow_herd.cow_list[0]['base'] = ur5_robot.fkine(q).A
            cow_herd.animate(0)

        # Note: if there is an error 'ModuleNotFoundError' about 'fast-simplification', do pip install fast-simplification
        # Note: if there is an error 'ValueError: ``target_reduction`` must be between 0 and 1', go into 'RobotCow.py' and 
        # change 'self._mesh_simplify' to a value between 0-1 (e.g. 0.1)
        input('Enter to finish\n')
        plt.close('all')


    def check_joint_limits(self, Q, robot):
        """
        Helper function: check whether each row of Q (joint configs) is within joint limits.

        Parameters:
        - Q: np.ndarray of shape (N, n)
        - robot: robot object with .qlim attribute of shape (2, n)

        Prints info about any rows that are invalid.
        """
        success = True
        Q = np.atleast_2d(Q)
        qlim = robot.qlim  # shape (2, n), where row 0 = lower, row 1 = upper

        for i, q in enumerate(Q):
            lower_violation = q < qlim[0]
            upper_violation = q > qlim[1]

            if np.any(lower_violation) or np.any(upper_violation):
                print(f"q[{i}] is out of joint limits:")
                for j in range(len(q)):
                    if lower_violation[j]:
                        print(f"  Joint {j}: {q[j]:.3f} < lower limit {qlim[0, j]:.3f}")
                    elif upper_violation[j]:
                        print(f"  Joint {j}: {q[j]:.3f} > upper limit {qlim[1, j]:.3f}")
                success = False
                
        return success


# Main block: Comment out the function call to run code for a specific question
if __name__ == '__main__':
    lab = Lab4Exercises()
    #lab.question2()
    #lab.question3()
    #lab.extension()

    plt.close('all')