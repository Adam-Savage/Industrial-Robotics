# Require libraries
import numpy as np
import matplotlib.image as mpimg
from scipy import linalg
import matplotlib.pyplot as plt
from spatialmath import SE2
from spatialmath.base import trplot2 
from pathlib import Path
from ir_support import functions as ir

# Useful variables
from math import pi

class Lab1Exercise:
    def __init__(self):
        # Load track image (update path if needed)
        image_path = Path(__file__).parent / "Lab1CircularRaceTrack.jpg"
        self.img = mpimg.imread(str(image_path))

    def run(self):
        # self.question1()
        self.question2()
        # self.questions3_and_4()

    # ---------------------------------------------------------------------------------------
    def question1(self):
        # 1) Setup
        print("Download and install the Robotics Toolbox. Run some example demos.")
        input("Press Enter to continue...")

    # ---------------------------------------------------------------------------------------
    def question2(self):
        # 2) Consider a car driving around a circular track
        fig = plt.figure()        
        fig.canvas.manager.set_window_title("Question 2")

        # 2.1) Display the track image
        plt.imshow(self.img)

        # 2.2) Create and plot initial car transform here
        # Hint: Use car1_tr = SE2(300, 550, 0) and trplot2(car1_tr.A....) to create and plot the initial car transform

        # 2.3) Make the leg lengths of the transform 50 units long inside trplot2 (See: https://github.com/petercorke/spatialmath-python/blob/master/spatialmath/base/transforms2d.py )

        # 2.4) Loop to update the transform
        total_steps = 360 # steps per revolulation
        for _ in range(total_steps):
            # Hint: Update car transform using multiplication (*) with motion and turn SE2 transforms.
            # Hint: use ir.clean_SE2() to clean the transform after each update so errors in the rotation matrix are corrected.

            # Clear and redraw track and updated car transform
            plt.cla()
            plt.imshow(self.img)

            # 2.5) Display car transform as text in corner
            # Hint: message = 'Line #1\nThe second line.\nAnd finally a third line.'
            # Hint: plt.text(10, 50, message, fontsize=10, color=(0.6, 0.2, 0.6))

            plt.draw()
            plt.pause(0.01)

    # ---------------------------------------------------------------------------------------
    def questions3_and_4(self):
        for question in [3, 4]:
            fig = plt.figure()
            fig.canvas.manager.set_window_title(f"Question {question}")

            if question == 4:
                plt.subplot(1, 2, 1)

            plt.imshow(self.img)

            # 3.1) Plot car2 at [300,125], heading along X-axis, red frame, length 50
            # Hint: Use SE2(x, y, theta) for each car's pose (e.g., car2_tr = SE2(300, 125, 0)) and trplot2(...) to display it; set frame color and length
                        
            total_steps = 360 # steps per revolulation            
            # 4.1.1) define a distance array to store the distances
            # Hint: dist = np.zeros(total_steps)

            # 3.2) Make the second car incrementally drives in the opposite direction to the first car
            for i in range(total_steps):
                # Hint: Update car1 and car2 transforms using multiplication (*) with motion and turn SE2 transforms.
                # Hint: Use ir.clean_SE2(...) to clean the transforms after each update
                # Hint: Plot each car’s transform with a different color using trplot2(...)

                # 3.3) In each iteration, compute and optionally print the relative transform
                # Hint: Use the .inv() method to get the inverse of a transform
                
                if question == 4:
                    # 4.1.2) Plot a graph of the distances between the two cars as they drive around the track
                    plt.subplot(1, 2, 1)
                    
                plt.cla()
                plt.imshow(self.img)                

                if question == 4:
                    # 4.1.3) Plot the distance between cars (sensor reading)
                    plt.subplot(1, 2, 2)
                    plt.xlabel('Timestep')
                    plt.ylabel("Sensor reading - distance between cars")
                    # Hint: use np.linalg.norm(...) to compute the distance
                    
                plt.draw()
                plt.pause(0.01)

if __name__ == "__main__":
    lab = Lab1Exercise()
    lab.run()
