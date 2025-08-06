
# Required libraries
import numpy as np
import matplotlib.pyplot as plt
import time
from pynput import keyboard
from scipy import linalg
from spatialmath import SE3
from spatialmath.base import transl, trotx, troty, tr2rpy, r2q
from roboticstoolbox import DHLink, DHRobot
from ir_support import RobotCow, tranimate_custom, place_fence, orthogonalize_rotation

from math import pi

def lab2_exercises_run():
    plt.close("all")
    input("Press Enter to begin\n")
    lab = Lab2Exercises()
    # Uncomment the function you want to test
    lab.question1()
    # lab.question2()
    # lab.question3()
    # lab.question3_point8()
    # lab.question4()


class Lab2Exercises:
    def __init__(self):
        pass

    def question1(self):
        """
        Question 1: Animate a UAV flying through a sequence of transforms.
        """
        print("TODO: Animate UAV transform sequence using tranimate_custom()")
        # 1.1)  Start at the origin and move up to 10m off the ground (positive Z)
        drone_tr_origin = transl(0, 0, 10)
        # 1.2)  Rotate (roll) around the X axis by -30 degrees so the Y axis is pointing more towards the ground than before
        drone_tr1 = drone_tr_origin @ trotx(-30 * pi/180)
        tranimate_custom(drone_tr_origin, drone_tr1)
        # 1.3)  Move in the direction of global Y to [0,2,10]
        drone_tr2 = transl(0, 2, 10) @ trotx(-30 * pi/180)
        tranimate_custom(drone_tr1, drone_tr2)
        # 1.4)  Roll back to level (so the orientation is now eye(3))
        drone_tr3 = drone_tr2 @ trotx(30 * pi/180)
        tranimate_custom(drone_tr2, drone_tr3)
        # 1.5)  Rotate (pitch) around the Y axis by 30 degrees so the X axis is pointing more towards the ground than before
        drone_tr4 = drone_tr3 @ troty(30 * pi/180)
        tranimate_custom(drone_tr3, drone_tr4)
        # 1.6) Move in the direction of global X to [2,2,10]
        drone_tr5 = transl(2, 0, 0) @ drone_tr4
        tranimate_custom(drone_tr4, drone_tr5)
        # 1.7) Roll back to level (so the orientation is now eye(3))
        drone_tr6 = drone_tr5 @ troty(-30 * pi/180)
        tranimate_custom(drone_tr5, drone_tr6)
        # 1.8) Go to the ground so that the new position is [2,2,0]
        drone_tr7 = drone_tr6 @ transl(0, 0, -10)
        tranimate_custom(drone_tr6, drone_tr7)
        # 1.9) Encode the steps 1.1-1.8 in a ‘for’ loop and use the 'fps' option in ‘tranimate’ to speed up the animation

        # 1.10)  Use the text tool from Week 1 Lab to plot in the left hand corner the RPY and quaternion value of the orientation at each step

        input("Press Enter to continue\n")

    def question2(self):
        """
        Question 2: Visualise cow herd and simulate movement.
        """
        print("TODO: Instantiate RobotCows and simulate random movements")
        # 2.1 Create an instance of RobotCows
        # cow_herd = RobotCow()

        # 2.2 Check the default cow with: cow_herd.num_cows        

        # 2.3 And plot the random walk movement of them with:
        # cow_herd.plot_single_random_step()

        # 2.4 Increase the number of cows
        # plt.close("all")
        # cow_herd = RobotCow(10)

        # 2.5 Test many random steps
        # num_steps = 100
        # delay = 0.01
        # cow_herd.test_plot_many_step(num_steps, delay)
        # plt.show() # Uncomment this line to keep the figure on

        # 2.6 Query the location of the 2nd cow with: cow_herd.cow_list[1]['base']        
        
        input("Press Enter to continue\n")

    def question3(self):
        """
        Question 3: Combine UAV animation, cow herd, and fences.
        """
        print("TODO: Animate UAV + herd in paddock with fences")

        # 3.1) place_fence at [5,0,0] and [-5,0,0]
        place_fence(position=[5,0,0], orientation= 0, plot_type='surface', scale=[1, 10, 1])
        place_fence(position=[-5,0,0], orientation= 0, plot_type='surface', scale=[1, 10, 1])

        # 3.2) Add more fences at [0,5,0] and [0,-5,0] rotated 90 deg
        place_fence(position=[0,5,0], orientation= 90, plot_type='surface', scale=[1, 10, 1])
        place_fence(position=[0,-5,0], orientation= 90, plot_type='surface', scale=[1, 10, 1])  

        # 3.3) Create a cow herd with more than two cows.

        # 3.4) Plot the transformation plot of the UAV starting at the origin (same as question 1)

        # 3.5) Determine the transformation between the UAV and each of the cows

        # 3.6) Each time the UAV moves, also move the cows randomly with:              	
        # cow_herd.plot_single_random_step()

        # 3.7) Fly through the flight path from question 1 and at each of the goal location determine the transformation between
        # the UAV and all the cows

        # 3.8) Create a cow herd with one cow and move your drone so that at each step the cow follows stays 5 meters above
        # it but directly overhead

        input("Press Enter to continue\n")

    def question4(self):
        """
        Question 4: Create 3-link manipulator using DH parameters
        """
        print("TODO: Define DH manipulator model and visualise it")
        # 4.1 Work out the DH Parameters by trial and error

        # 4.2 Generate the robot model with 3 links.
        # L1 = DHLink(d = __, a =__, alpha =__, offset =__, qlim= [__,__])
        # ... # More link definition in between (if required)
        # Ln = DHLink(d = __, a =__, alpha =__, offset =__, qlim= [__,__])
        # robot = DHRobot([L1 ... Ln], name ='myRobot')
        # q = np.zeros([1,n]) # This creates a vector of n joint angles at 0.
        # workspace = [-x +x –y +y –z +z]
        # robot.plot(q= q, limits= workspace) 

        # 4.3) You can manually play around with the robot:
       	# Close current figure because teach method will create a new one
        # plt.close() 
        # fig = robot.teach(robot.q, block = False)

        # 4.4) Get the current joint angles based on the position in the model.
        # q = robot.q

        # 4.5) Get the joint limits with
        # robot.qlim

        input("Press Enter to continue\n")


def on_press(key):
    pass  # Optional: handle key press

def on_release(key):
    if key == keyboard.Key.esc:
        # Stop listener when ESC is released
        return False

if __name__ == "__main__":
    # Start keyboard listener
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    lab2_exercises_run()

    listener.stop()  # Proper way to stop pynput keyboard listener
    plt.close("all")
    time.sleep(0.5)
