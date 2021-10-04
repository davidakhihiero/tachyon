#  Author: David Akhihiero
import numpy as np
from math import sin, cos
from scipy.optimize import fsolve
import rospy
from control_msgs.msg import JointControllerState

class Leg:
    def __init__(self, L1, H1, L2, L3, name, right=True):
        self.right = 1 if right else -1
        self.L1 = L1
        self.H1 = H1
        self.L2 = L2
        self.L3 = L3
        self.name = name
        self.theta1 = 0
        self.theta2 = 0
        self.theta3 = 0
        self.theta1_error = 0
        self.theta2_error = 0
        self.theta3_error = 0
        self.allowed_error = 0.03

        # Subscriber objects to monitor the error in the joint positions
        rospy.Subscriber("/tachyon/joint_base_to_" + self.name + "_controller/state", JointControllerState, self.body_to_swing_callback)
        rospy.Subscriber("/tachyon/joint_swing_link_to_upper_limb_" + self.name + "_controller/state", JointControllerState, self.swing_to_upper_callback)
        rospy.Subscriber("/tachyon/joint_upper_limb_to_lower_limb_" + self.name + "_controller/state", JointControllerState, self.upper_to_lower_callback)

    def body_to_swing_callback(self, msg):
        self.theta1_error = msg.error

    def swing_to_upper_callback(self, msg):
        self.theta2_error = msg.error
    
    def upper_to_lower_callback(self, msg):
        self.theta3_error = msg.error


    def get_joint_angles(self, x, y, z):
        # Method to compute the inverse kinematics of the leg
        def equations(thetas):
            L1 = self.L1
            H1 = self.H1
            L2 = self.L2
            L3 = self.L3
            theta1, theta2, theta3 = thetas
            eq1 = L2 * sin(theta2) + L3 * cos(theta2) * sin(theta3) + L3 * cos(theta3) * sin(theta2) - x
            eq2 = L3 * cos(theta1) * sin(theta2) * sin(theta3) - L1 * sin(theta1) - L2 * cos(theta1) * cos(theta2) - L3 * cos(theta1) * cos(theta2) * cos(theta3) - H1 * cos(theta1) - y
            eq3 = L1 * cos(theta1) - H1 * sin(theta1) - L2 * cos(theta2) * sin(theta1) - L3 * cos(theta2) * cos(theta3) * sin(theta1) + L3 * sin(theta1) * sin(theta2) * sin(theta3) - z
            return [eq1, eq2, eq3]

        solution = fsolve(equations, np.array([[self.theta1],
                                               [self.theta2],
                                               [self.theta3]]))
        joint_angles = []
        
        # Constraining the angles to be bounded between -180 degs and 180 degrees (-pi to pi)
        for angle in solution:
            joint_angle = (int(angle * 1000000) % 6283185) / 1000000
            if joint_angle > np.pi:
                joint_angle = - (2 * np.pi - joint_angle)

            joint_angles.append(joint_angle)

        # Invert theta1 and theta2 if the leg is on the left side 
        joint_angles[1] *= self.right
        joint_angles[2] *= self.right

        self.theta1 = joint_angles[0]
        # Reverse theta1 and theta2 to their original values to serve as initial values for the next IK computation
        self.theta2 = joint_angles[1] * self.right
        self.theta3 = joint_angles[2] * self.right

        return joint_angles, True if all(np.isclose(equations(solution), [0, 0, 0])) else False

    def at_desired_point(self):
        # Method to check if the errors in the joint positions are below an accepted threshold (allowed_error)
        return all(list(map(lambda x: True if abs(x) < self.allowed_error else False, [self.theta1_error, self.theta2_error, self.theta3_error])))
