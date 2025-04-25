# numpy provides import array and linear algebra utilities
import numpy as np

# the robotics toolbox provides robotics specific functionality
import roboticstoolbox as rtb

# spatial math provides objects for representing transformations
import spatialmath as sm

# swift is a lightweight browser-based simulator which comes eith the toolbox
from swift import Swift

# the Python math library
import math

# spatialgeometry is a utility package for dealing with geometric objects
import spatialgeometry as sg

# typing utilities
from typing import Tuple

import time

class hand():

    def __init__(self, arm_model, initial_pose, threshold,  time_step):
        self.arm_model = arm_model                #arm model to be loaded
        self.initial_pose = initial_pose          #initial/start/ready pose of the robot
        self.threshold = threshold                #threshold for acceptable error for the target pose
        self.time_step = time_step                # Specify our timestep
        self.arrived = False                      # A variable to specify when to break the loop
        self.miu = 1                              #set diagonal value for admittance matrix.

    def angle_axis(self, T: np.ndarray, Td: np.ndarray) -> np.ndarray:
        """
        Returns the error vector between T and Td in angle-axis form.
    
        :param T: The current pose
        :param Td: The desired pose
        :returns e: the error vector between T and Td
        """
    
        # a variable to hold the angle axis error also known as euler vector
        e = np.empty(6)
    
        # calculates the the position error by taking the difference between the x,y,z 
        # axis of the current and target pose of the end effector. Note that T and Td are 4x4 matrices
        e[:3] = Td[:3, -1] - T[:3, -1]
    
        # this extracts the rotational par of the transformation matrix of the desired and current pose 
        # this is followed by a matrix multiplication
        R = Td[:3, :3] @ T[:3, :3].T
    
         # the code below is a representaion of 
         #      (r32 - r23)
         # li = (r13 - r31)
         #      (r21 - r12)
        li = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    
        # If li is a zero vector (or very close to it)
        if np.linalg.norm(li) < 1e-6:
    
            # diagonal matrix case
            if np.trace(R) > 0:
                # (1,1,1) case
                a = np.zeros((3,))
            else:
                a = np.pi / 2 * (np.diag(R) + 1)
        else:
            # non-diagonal matrix case
            ln = np.linalg.norm(li)
            a = math.atan2(ln, np.trace(R) - 1) * li / ln
    
        e[3:] = a
        
        # returns an array of 6 values representing the angle axis error
        return e

    def run(self, target_pose):
 
        # Make a new environment to visualize the robot arm  
        env = Swift()
        env.launch(realtime=True)
        
        # adding the  end-effector axes to the visualization environment
        ee_axes = sg.Axes(0.1)
        env.add(ee_axes)
        
        # adding the  target pose axes to the visualization environment
        goal_axes = sg.Axes(0.1)
        env.add(goal_axes)
        
        # this section gets the robot model, it is set when initializing the hand class. 
        # the getattr method is used to feed in the robot model think of the first line like"rtb.models.Panda()"
         # This will make more sense when we use the code.
        model = getattr(rtb.models, self.arm_model)
        arm = model()        
 
        initial_pose = getattr(arm, self.initial_pose)
        arm.q = initial_pose            # Change the robot configuration to the ready/start/initial position
        
        # adding the robot arm to our visualization environment
        env.add(arm)                    #add our robot

        # compting the forward kinematics of the end-effector's initial pose 
        # (using .A to get a numpy array instead of an SE3 object)
        X = arm.fkine(arm.q).A
       
       # setting the target pose as a numpy array. The target pose is given as an input to this funtion
        Xt = target_pose.A
        
        # Set the initial/current end effector axes to Xt
        ee_axes.T = X
        # Set the goal axes to Xt
        goal_axes.T = Xt
        
        # Step the sim to view the robot in this configuration
        env.step(0)
             
        # this pauses the visualization for a better experience 
        time.sleep(3) 
        
        # After each step, the block below checks if the arm has arrrived its target pose
        # and continues in a loop untill it arrives at the target pose
        # the self.arrived variable is set to false by defaulf when initializing the class.
        while not self.arrived:
            
            # get the forward kinematics of the current pose after each step
            X = arm.fkine(arm.q).A
        
            # update the end effector axis
            ee_axes.T = X
           
             # this helps check the difference(error) between the current pose and target pose of the end effector.
            # the self.angle_axis functioned will be defined later  
            E = self.angle_axis(X, Xt)   #(Xt - X)
        
            #the arm is set to have arrived its target pose when the difference bet
            # calculated above is between an acceptable threshould as earlier defined
            self.arrived = True if np.sum(np.abs(E)) < self.threshold else False
            
        
            # check the number of DoF of the arm
            if arm.n == 6:           
                J = arm.jacob0(arm.q)   #for when arm has 6 joints and a transfrom is a square matrix
                #calculate the required velocity to get the arm to the target pose
                Qd= J.T @ E
    
                # move the arm using the calculated velocity        
                arm.qd = Qd
                
            else:
                J = arm.jacob0(arm.q)
                J = np.linalg.pinv(J)   # J_inv for when it has less than 6 joints or is a redundant robot with more than 6 joints (singularity)
        
                #calculate the required velocity to get the arm to the target pose
                Qd= J @ E
           
                # move the arm using the calculated velocity
                arm.qd = Qd
            
            # update the visualization by a timestep defined witht he class
            env.step(self.time_step)
