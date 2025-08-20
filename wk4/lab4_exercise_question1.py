# Require libraries
import numpy as np
import matplotlib.pyplot as plt
import swift
import threading
import time
from spatialmath.base import *
from roboticstoolbox import models, DHLink, DHRobot
from ir_support import CylindricalDHRobotPlot
from ir_support.plyprocess import *
from spatialgeometry import Sphere, Mesh
import os

# Useful variables
from math import pi

class Lab4Exercises:
    def __init__(self):
        print('Lab 4 Exercises - Question 1 Starting Point')
        self.stop_event = threading.Event()         # Event to end teach mode when 'enter' pressed

    def wait_for_enter(self):
        '''
        Helper threaded function to detect keypress without needing keyboard library
        '''
        try:
            print("Press Enter to stop.\n")
            input()
        except EOFError:
            pass
        self.stop_event.set()


    def question1(self):
        '''
        1) Drawing with a 3DOF Planar Arm
        '''
        #1.1) Make the 3DOF planar arm model
        # Hint: Use robot = DHRobot([link1, link2, link3], name="my_robot") where DHLink(d=..., a=..., alpha=..., offset=..., qlim=[lower, upper]) for each link
        # Hint 2: For the DH parameters of a planar robot, only translation is along the x-axis (a=1) and there is no joint rotation about x-axis (alpha)
        link1 = DHLink(d=0, a=1, alpha=0, qlim=[-pi, pi])
        link2 = DHLink(d=0, a=1, alpha=0, qlim=[-pi, pi])
        link3 = DHLink(d=0, a=1, alpha=0, qlim=[-pi, pi])
        robot = DHRobot([link1, link2, link3], name= 'my_robot')

        # Give the robot a cylinder mesh (links) to display in Swift environment
        cyl_viz = CylindricalDHRobotPlot(robot, cylinder_radius=0.05, color="#3478f6")
        robot = cyl_viz.create_cylinders()

        # 1.2) Rotate the base around the X axis so the Z axis faces down ways
        # Hint: We can use the 'trotx' function to create a 4x4 transform with only rotation about X (no translation)
        # Hint: If the Z-axis is originally facing up, what amount of rotation is needed to have it face down (in radians)?

        robot.base = trotx(pi)

        # 1.3) Set workspace, scale and initial joint state, then plot and teach
        # Hint: Set the initial joint state as zeroes (q = np.zeros(robot.n)) and workspace as [<-x>, <+x>, <-y>, <+y>, <-z>, <+z>]
        # Plot the robot in the workspace at the defined joint states with: robot.plot(q, limits= workspace)
        # Create 'teach mode' using robot.teach(q, limits=workspace, block=False). You need to use a while loop
        # with an exit condition and the update step 'fig.step(timestep)' to use the teach mode until no longer desired.
        q = np.zeros(robot.n)  # Initial joint state
        workspace = [-2, 2, -2, 2, -1, 1]  # Workspace limits
        robot.plot(q, limits=workspace)
        input("Enter to teach and hit Enter again to continue\n")
        plt.close()
        fig = robot.teach(q, limits= workspace, block = False)

        # 1.4) Move the robot around with “teach” and observe that there is no 
        # way to affect the Z, roll or pitch values, no matter what joint value you choose.
        # Continuously update the teach figure while it is being used (every 0.05s)

        # input_thread = threading.Thread(target=self.wait_for_enter)
        # input_thread.start()
        # while not self.stop_event.is_set():
        #     fig.step(0.05)

        # self.stop_event.clear()
        # input_thread.join()

        print("Use the teach interface to move the robot around.")
        print("Close the teach window or press Ctrl+C in terminal when done.")

        try:
            start_time = time.time()
            while True:
                try:
                    fig.step(0.05)
                    time.sleep(0.05)
                    
                    if time.time() - start_time > 1:
                        plt.pause(0.01)
                        
                except KeyboardInterrupt:
                    print("\nStopping teach mode...")
                    break
                except:
                    print("\nTeach window closed or error occurred, continuing...")
                    break
                    
        except Exception as e:
            print(f"Error in teach mode: {e}")
            print("Continuing with the rest of the exercise...")
        
        plt.close('all')

        # 1.5) Consider a pen that is mounted to the Z-axis of the final joint. Note how this means we don't care 
        # about the yaw angle (if it's a pen where the Z axis is rotating, it doesn't affect the drawing result.

        # 1.6) Get a joint state solution for the end effector at [-0.75,-0.5,0], and make sure you mask out the impossible-to-alter 
        # values (i.e. z, roll and pitch) and the value we don’t care about (i.e. yaw). Thus we need a mask of [1,1,0,0,0,0]:
        # Hint: Try one or more of the inverse kinematics solvers in RobotKinematics.py, in this form "result = robot.ikine(T)",
        # E.g. robot.ikine_LM(transl([-0.75, -0.5, 0]))
        new_q = robot.q
        new_q = robot.ikine_LM(transl([-0.75, -0.5, 0]), q0= new_q, mask= [1, 1, 0, 0, 0, 0]).q

        # 1.7) Plot the new joint state and check how close it got to the [x, y] in the goal transform (i.e. transl(-0.75,-0.5,0))
        # We will do this part of the exercise in the Swift simulator
        env = swift.Swift()
        env.launch(realtime=True)
        env.add(robot)                               # Add robot to environment
        env.set_camera_pose([2, -2, 2], [0, 0, 0])   # set camera (position, look-at)

        print("Fkine solution:\n", robot.fkine(new_q).A)
        input("Enter to continue\n")

        # 1.8) Go through a loop using the previous joint as the guess to draw a line from [-0.75,-0.5,0] to [-0.75,0.5,0] 
        # and animate each new joint state trajectory by updating the robot state, then update the figure.
        line = [] # list of plotted points

        # 1.9) Keep track of [x,y] using fkine. How straight is the actual line drawn if you see every point in between? 
        # Hint: To loop through a range of values, you can use either range(start, stop, step) or np.arange(start, stop, step)
        # where each loop iteration is start + iteration*step, starting from iteration = 0, and not including the stop value.
        # So to loop through y = -0.5 to 0.5, you may try start = -0.5, step = ?, stop = 0.5 + step, try different values for the step!
        # E.g. start = -0.5, stop = 0.5 + 0.25, step = 0.25: iterations = -0.5, -0.25, 0, 0.25, 0.5
        for y in np.arange(-0.5, 0.51, 0.1):
            # Hint: Get joint state for robot end effector at desired location using ikine_LM and update robot.q with joint state
            # Hint: As a default, using the current joint state as 'q0' is a good option when the point being moved to is close to the current position
            new_q = robot.ikine_LM(..., q0=..., mask=...).q
            robot.q = ...
            env.step(0.01)

            ee_pos = robot.fkine(...)
            # Plot a red sphere at the end-effector location
            new_point = Sphere(radius=0.05, color=[1.0, 0.0, 0.0, 1.0])     # Color = RGBA
            new_point.T = ...                                               # Set the 4x4 transformation matrix encoding the sphere's pose
            env.add(new_point)                                              # Add sphere to the environment
            line.append(new_point)                                          # Append sphere to a list to track number and ID for removal

            env.step()
            time.sleep(5)

        input("Press enter to continue to 1.10")

        # Reset the environment to remove spheres, have to re-add robot
        env.reset()
        line.clear()        # Clear 'line' list
        env.add(robot)
        env.set_camera_pose([2, 2, 2], [0, 0, 0])   # set camera (position, look-at)


        # 1.10) Using ikine to get the newQ and fkine to determine the actual point, move the robot to “draw” a circle around it with a radius of 0.5m
        # Hint: Two methods:
        # 1. Draw two semicircles using for loops - can calculate y by looping through x and using equation of a semicircle
        # 2. Loop through one angular revolution, at each step calculate x, y using trigonometry

        # for ... in ...:
        #     pass


        input("Press enter to continue to 1.11")

        # Reset the environment to remove spheres, have to re-add robot
        env.reset()
        line.clear()        # Clear 'line' list

        # 1.11) + 1.12) Let's add a pen (penVertexColour.dae - download from Canvas), which I modified and coloured. 
        # We will use the spatialgeometry 'Mesh' class to place the object into the Swift environment
        # Plot a 3-link planar robot from the toolbox
        p3 = models.DH.Planar3()
        p3.q = np.zeros([1, 3])
        # Give the robot a cylinder mesh (links) to display in Swift environment
        cyl_viz = CylindricalDHRobotPlot(p3, cylinder_radius=0.05, multicolor=True)
        robot = cyl_viz.create_cylinders()
        env.add(p3)

        # Create pen - if have a ply file, convert to DAE (or STL) to use with Swift
        current_dir = os.path.dirname(os.path.abspath(__file__))
        dae_path = os.path.join(current_dir, "pen.dae")
        pen_mesh = Mesh(filename=dae_path)

        # 1.13) Set pen transform to be at robot end-effector, translate along Z by 0.1m
        pen_mesh.T = p3.fkine(...).A @ transl(..., ..., ...)
        # Add pen to environment
        env.add(pen_mesh)
        env.set_camera_pose([3, 3, 2], [0, 0, 0])  # (position, look-at)

        # 1.14) Move the pen as if it were on the end-effector through a naively-created arm trajectory
        for i in np.arange(-pi/4, pi/4+0.01, 0.01):
            p3.q = [i,i,i]
            env.step(0.01)
            ee_pos = ...                # Get the position of the end-effector
            pen_mesh.T = ...            # Set the pose of the pen mesh

            # Create a red sphere with radius 0.025m at each point, set it to the end-effector location
            new_point = ...
            new_point.T = ...
            env.add(new_point)

            env.step(0.05)
            
        input("Press enter to finish\n")
        env.close()


# Main block: Comment out the function call to run code for a specific question
if __name__ == '__main__':
    lab = Lab4Exercises()
    lab.question1()
