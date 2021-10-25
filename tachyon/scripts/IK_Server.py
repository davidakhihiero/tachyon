#! /usr/bin/env python
# Author: David Akhihiero
import numpy as np
from math import sin, cos
from scipy.optimize import fsolve
import rospy
from tachyon.srv import IK
from tachyon.srv import IKRequest
from tachyon.srv import IKResponse


def get_joint_angles(x, y, z, L1, H1, L2, L3, thetas, right=1):
        # Method to compute the inverse kinematics of the leg
        def equations(thetas):
            theta1, theta2, theta3 = thetas
            eq1 = L2 * sin(theta2) + L3 * cos(theta2) * sin(theta3) + L3 * cos(theta3) * sin(theta2) - x
            eq2 = L3 * cos(theta1) * sin(theta2) * sin(theta3) - L1 * sin(theta1) - L2 * cos(theta1) * cos(theta2) - L3 * cos(theta1) * cos(theta2) * cos(theta3) - H1 * cos(theta1) - y
            eq3 = L1 * cos(theta1) - H1 * sin(theta1) - L2 * cos(theta2) * sin(theta1) - L3 * cos(theta2) * cos(theta3) * sin(theta1) + L3 * sin(theta1) * sin(theta2) * sin(theta3) - z
            return [eq1, eq2, eq3]

        solution = fsolve(equations, np.array(thetas))
        joint_angles = []
        
        # Constraining the angles to be bounded between -180 degs and 180 degrees (-pi to pi)
        for angle in solution:
            joint_angle = (int(angle * 1000000) % 6283185) / 1000000
            if joint_angle > np.pi:
                joint_angle = - (2 * np.pi - joint_angle)

            joint_angles.append(joint_angle)

        # Invert theta1 and theta2 if the leg is on the left side 
        joint_angles[1] *= right
        joint_angles[2] *= right

        return joint_angles, True if all(np.isclose(equations(solution), [0, 0, 0])) else False

def handle_ik_request(req):
    x, y, z = req.x, req.y, req.z
    L1, H1, L2, L3 = req.L1, req.H1, req.L2, req.L3
    thetas = [[req.old_theta1], [req.old_theta2], [req.old_theta3]]
    right = req.right

    x *= -1 # The X+ axis is in the opposite direction to the X+ for tachyon when using the Cpp nodes

    new_thetas, solution_found = get_joint_angles(x, y, z, L1, H1, L2, L3, thetas, right)
    response = IKResponse()
    response.theta1 = new_thetas[0]
    response.theta2 = new_thetas[1]
    response.theta3 = new_thetas[2]
    response.solution_found = solution_found

    return response


def ik_server():
    rospy.init_node("IK_Server_node")
    rospy.Service("ik_service", IK, handle_ik_request)
    print("Starting Inverse Kinematics Service")
    rospy.spin()


if __name__ == "__main__":
    ik_server()