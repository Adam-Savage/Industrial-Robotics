# Require libraries
import numpy as np
import matplotlib.pyplot as plt
import time
from pynput import keyboard
from scipy import linalg
from roboticstoolbox import DHLink, DHRobot

# Useful variables
from math import pi

import threading

# Global flag
enter_pressed = False

def on_press(key):
    global enter_pressed
    if key == keyboard.Key.enter:
        enter_pressed = True

class Lab3Exercises:
    def __init__(self):
        print("Lab 3 Exercises Starting Point")

    def question1(self):
        print("=== Question 1: Robot Analysis ===\n")
        
        # 1.1 & 1.2) Create robot
        print("1.1-1.2) Creating 3-link robot...")
        link1 = DHLink(d=1, a=0, alpha=pi/2, qlim =[-pi, pi], offset=0)
        link2 = DHLink(d=0, a=1, alpha=0, qlim=[-pi, pi], offset=0)
        link3 = DHLink(d=0, a=1, alpha=0, qlim=[-pi, pi], offset=0)
        
        robot = DHRobot([link1, link2, link3], name="Robot1")
        q = np.zeros(robot.n)
        
        # Show initial robot
        robot.plot(q, limits=[-2, 2, -2, 2, -2, 2])
        plt.title("Initial Robot Configuration")
        
        # 1.3) Teach pendant
        print("1.3) Opening teach window - adjust joints and press ENTER when done...")
        fig = robot.teach(q, block=False)
        
        # Wait for Enter key
        while not enter_pressed: 
            fig.step(0.05)
            time.sleep(0.01)
        
        # 1.4) Get final configuration
        q = robot.q
        print(f"1.4) Final joint angles: {np.round(q, 3)}")
        
        # 1.5) Forward kinematics
        print("\n1.5) Forward kinematics:")
        T = robot.fkine(q)
        print(f"Position: {np.round(T.t, 3)}")
        print(f"Rotation:\n{np.round(T.R, 3)}")
        
        # 1.6) Inverse kinematics
        print("\n1.6) Testing inverse kinematics...")
        try:
            result = robot.ikine_LM(T)
            if result.success:
                q_ik = result.q
                print(f"IK solution: {np.round(q_ik, 3)}")
                
                # Verify
                T_check = robot.fkine(q_ik)
                position_error = np.linalg.norm(T.t - T_check.t)
                print(f"Position error: {position_error:.8f}")
            else:
                print(f"IK failed: {result.reason}")
        except Exception as e:
            print(f"IK error: {e}")
        
        # 1.7) Jacobian
        print(f"\n1.7) Jacobian matrix at q = {np.round(q, 3)}:")
        J = robot.jacob0(q)
        print(f"Shape: {J.shape}")
        print(f"Jacobian:\n{np.round(J, 3)}")
        
        # 1.8) Jacobian inversion
        print("\n1.8) Jacobian inversion tests:")
        
        # Test square part (3x3)
        if J.shape[1] == 3:  # 3-DOF robot
            J_square = J[0:3, 0:3]  # Position part only
            try:
                det_J = linalg.det(J_square)
                print(f"Determinant of 3x3 Jacobian: {det_J:.6f}")
                
                if abs(det_J) > 1e-6:  # Non-singular
                    J_inv = linalg.inv(J_square)
                    print("3x3 Jacobian is invertible")
                    print(f"Condition number: {linalg.cond(J_square):.2f}")
                else:
                    print("3x3 Jacobian is singular (not invertible)")
                    
            except Exception as e:
                print(f"Jacobian inversion failed: {e}")
        
        # 1.9) Test singular configuration
        print("\n1.9) Testing singular configuration (all joints = 0):")
        q_singular = np.zeros(3)
        J_singular = robot.jacob0(q_singular)
        J_sing_square = J_singular[0:3, 0:3]
        det_singular = linalg.det(J_sing_square)
        print(f"Determinant at q=[0,0,0]: {det_singular:.8f}")
        
        if abs(det_singular) < 1e-6:
            print("→ This is a SINGULAR configuration (robot loses mobility)")
        else:
            print("→ Not singular at this configuration")
        
        # 1.10-1.11) Velocity ellipse
        print("\n1.10-1.11) Velocity ellipse analysis:")
        
        # Test with current configuration
        try:
            print("Testing current configuration...")
            ellipse_current = robot.vellipse(q)
            
            # Create a single figure with subplots
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
            
            # Current configuration ellipse
            plt.sca(ax1)
            ellipse_current.plot()
            ax1.set_title(f'Current Config q={np.round(q, 2)}')
            ax1.grid(True)
            ax1.set_aspect('equal')
            
            # Singular configuration ellipse
            plt.sca(ax2)
            ellipse_singular = robot.vellipse(q_singular)
            ellipse_singular.plot()
            ax2.set_title('Singular Config q=[0,0,0]')
            ax2.grid(True)
            ax2.set_aspect('equal')
            
            plt.tight_layout()
            plt.suptitle('Velocity Ellipses Comparison')
            
            # Make sure the plot displays and stays open
            plt.show(block=False)
            plt.pause(0.5)  # Give it time to render
            
            print("✓ Velocity ellipses plotted successfully!")
            print("\nObservations:")
            print("- Normal config: Ellipse shows directional velocity capabilities")
            print("- Singular config: Degenerate ellipse shows loss of mobility")
            print("\n🔍 Look at the velocity ellipse window - it should show two plots side by side")
            
            # Keep the window open until user is ready
            input("\nPress Enter after examining the velocity ellipses...")
            
        except Exception as e:
            print(f"Velocity ellipse failed: {e}")
            print("This might be due to robotics toolbox version or configuration issues")
        
        print("\n=== Question 1 Complete! ===")
        plt.close('all')

    def question2(self):
        pass

    def question3(self):
        pass

if __name__ == "__main__":
    # Start keyboard listener
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    
    lab = Lab3Exercises()
    lab.question1()