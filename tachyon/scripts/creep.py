#! /usr/bin/env python

# Code to make tachyon creep

import rospy
import numpy as np
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from Leg import Leg


def creep():

    full_step_size = 0.25

    base_to_front_left_pub = rospy.Publisher("/tachyon/joint_base_to_front_left_controller/command", Float64, queue_size=10)
    swing_link_to_upper_limb_front_left_pub = rospy.Publisher("/tachyon/joint_swing_link_to_upper_limb_front_left_controller/command", 
    Float64, queue_size=10)
    upper_limb_to_lower_limb_front_left_pub = rospy.Publisher("/tachyon/joint_upper_limb_to_lower_limb_front_left_controller/command", 
    Float64, queue_size=10)

    front_right_leg = Leg (0.13002, 0.08274, 0.39825, 0.33, 'front_right', True)
    back_left_leg = Leg (0.13002, 0.08274, 0.39825, 0.33, 'back_left', False)
    back_right_leg = Leg (0.13002, 0.08274, 0.39825, 0.33, 'back_right', True)
    front_left_leg = Leg (0.13002, 0.08274, 0.39825, 0.33, 'front_left', False)

    legs = [front_right_leg, front_left_leg, back_right_leg, back_left_leg]

    base_to_front_right_pub = rospy.Publisher("/tachyon/joint_base_to_front_right_controller/command", Float64, queue_size=10)
    swing_link_to_upper_limb_front_right_pub = rospy.Publisher("/tachyon/joint_swing_link_to_upper_limb_front_right_controller/command", 
    Float64, queue_size=10)
    upper_limb_to_lower_limb_front_right_pub = rospy.Publisher("/tachyon/joint_upper_limb_to_lower_limb_front_right_controller/command", 
    Float64, queue_size=10)

    base_to_back_left_pub = rospy.Publisher("/tachyon/joint_base_to_back_left_controller/command", Float64, queue_size=10)
    swing_link_to_upper_limb_back_left_pub = rospy.Publisher("/tachyon/joint_swing_link_to_upper_limb_back_left_controller/command", 
    Float64, queue_size=10)
    upper_limb_to_lower_limb_back_left_pub = rospy.Publisher("/tachyon/joint_upper_limb_to_lower_limb_back_left_controller/command", 
    Float64, queue_size=10)

    base_to_back_right_pub = rospy.Publisher("/tachyon/joint_base_to_back_right_controller/command", Float64, queue_size=10)
    swing_link_to_upper_limb_back_right_pub = rospy.Publisher("/tachyon/joint_swing_link_to_upper_limb_back_right_controller/command", 
    Float64, queue_size=10)
    upper_limb_to_lower_limb_back_right_pub = rospy.Publisher("/tachyon/joint_upper_limb_to_lower_limb_back_right_controller/command", 
    Float64, queue_size=10)

    

    rate = rospy.Rate(5)

    # first move diagonal pair (1) forward half a step
    # then move diagonal pair (2) forward a full step, then (1) a full step in a loop

    steps_left_pair = generated_steps_for_a_step(full_step_size / 2, True, 0, True)
    x_values_left = steps_left_pair[0] 
    y_values_left = steps_left_pair[1]

    steps_right_pair = generated_steps_for_a_step(full_step_size / 2, False, 0, True)
    x_values_right = steps_right_pair[0] 
    y_values_right = steps_right_pair[1]


    for i in range(len(x_values_left)):
        angles = front_left_leg.get_joint_angles(x_values_left[i], y_values_left[i], 0.13002)
        theta1, theta2, theta3 = angles[0]

        base_to_front_left_pos = Float64(theta1)
        swing_link_to_upper_limb_front_left_pos = Float64(theta2)
        upper_limb_to_lower_limb_front_left_pos = Float64(theta3)


        angles = back_right_leg.get_joint_angles(x_values_right[i], y_values_right[i], 0.13002)
        theta1, theta2, theta3 = angles[0]

        base_to_back_right_pos = Float64(theta1)
        swing_link_to_upper_limb_back_right_pos = Float64(theta2)
        upper_limb_to_lower_limb_back_right_pos = Float64(theta3)


        angles = front_right_leg.get_joint_angles(x_values_right[i], y_values_right[i], 0.13002)
        theta1, theta2, theta3 = angles[0]

        base_to_front_right_pos = Float64(theta1)
        swing_link_to_upper_limb_front_right_pos = Float64(theta2)
        upper_limb_to_lower_limb_front_right_pos = Float64(theta3)


        angles = back_left_leg.get_joint_angles(x_values_left[i], y_values_left[i], 0.13002)
        theta1, theta2, theta3 = angles[0]

        base_to_back_left_pos = Float64(theta1)
        swing_link_to_upper_limb_back_left_pos = Float64(theta2)
        upper_limb_to_lower_limb_back_left_pos = Float64(theta3)

        at_desired = False

        while not at_desired: 
            base_to_front_left_pub.publish(base_to_front_left_pos)
            swing_link_to_upper_limb_front_left_pub.publish(swing_link_to_upper_limb_front_left_pos)
            upper_limb_to_lower_limb_front_left_pub.publish(upper_limb_to_lower_limb_front_left_pos)

            base_to_back_right_pub.publish(base_to_back_right_pos)
            swing_link_to_upper_limb_back_right_pub.publish(swing_link_to_upper_limb_back_right_pos)
            upper_limb_to_lower_limb_back_right_pub.publish(upper_limb_to_lower_limb_back_right_pos)

            base_to_front_right_pub.publish(base_to_front_right_pos)
            swing_link_to_upper_limb_front_right_pub.publish(swing_link_to_upper_limb_front_right_pos)
            upper_limb_to_lower_limb_front_right_pub.publish(upper_limb_to_lower_limb_front_right_pos)

            base_to_back_left_pub.publish(base_to_back_left_pos)
            swing_link_to_upper_limb_back_left_pub.publish(swing_link_to_upper_limb_back_left_pos)
            upper_limb_to_lower_limb_back_left_pub.publish(upper_limb_to_lower_limb_back_left_pos)

            rate.sleep()

            at_desired = True

            for leg in legs:
                at_desired = leg.at_desired_point()
                if not at_desired:
                    break


    while True and not rospy.is_shutdown():

        steps_left_pair = generated_steps_for_a_step(full_step_size, False, full_step_size / 2, True)
        x_values_left = steps_left_pair[0] 
        y_values_left = steps_left_pair[1]

        steps_right_pair = generated_steps_for_a_step(full_step_size, True, -full_step_size / 2, True)
        x_values_right = steps_right_pair[0] 
        y_values_right = steps_right_pair[1]

        for i in range(len(x_values_left)):
            angles = front_left_leg.get_joint_angles(x_values_left[i], y_values_left[i], 0.13002)
            theta1, theta2, theta3 = angles[0]

            base_to_front_left_pos = Float64(theta1)
            swing_link_to_upper_limb_front_left_pos = Float64(theta2)
            upper_limb_to_lower_limb_front_left_pos = Float64(theta3)


            angles = back_right_leg.get_joint_angles(x_values_right[i], y_values_right[i], 0.13002)
            theta1, theta2, theta3 = angles[0]

            base_to_back_right_pos = Float64(theta1)
            swing_link_to_upper_limb_back_right_pos = Float64(theta2)
            upper_limb_to_lower_limb_back_right_pos = Float64(theta3)


            angles = front_right_leg.get_joint_angles(x_values_right[i], y_values_right[i], 0.13002)
            theta1, theta2, theta3 = angles[0]

            base_to_front_right_pos = Float64(theta1)
            swing_link_to_upper_limb_front_right_pos = Float64(theta2)
            upper_limb_to_lower_limb_front_right_pos = Float64(theta3)


            angles = back_left_leg.get_joint_angles(x_values_left[i], y_values_left[i], 0.13002)
            theta1, theta2, theta3 = angles[0]

            base_to_back_left_pos = Float64(theta1)
            swing_link_to_upper_limb_back_left_pos = Float64(theta2)
            upper_limb_to_lower_limb_back_left_pos = Float64(theta3)

            at_desired = False
            while not at_desired: 
                base_to_front_left_pub.publish(base_to_front_left_pos)
                swing_link_to_upper_limb_front_left_pub.publish(swing_link_to_upper_limb_front_left_pos)
                upper_limb_to_lower_limb_front_left_pub.publish(upper_limb_to_lower_limb_front_left_pos)

                base_to_back_right_pub.publish(base_to_back_right_pos)
                swing_link_to_upper_limb_back_right_pub.publish(swing_link_to_upper_limb_back_right_pos)
                upper_limb_to_lower_limb_back_right_pub.publish(upper_limb_to_lower_limb_back_right_pos)

                base_to_front_right_pub.publish(base_to_front_right_pos)
                swing_link_to_upper_limb_front_right_pub.publish(swing_link_to_upper_limb_front_right_pos)
                upper_limb_to_lower_limb_front_right_pub.publish(upper_limb_to_lower_limb_front_right_pos)

                base_to_back_left_pub.publish(base_to_back_left_pos)
                swing_link_to_upper_limb_back_left_pub.publish(swing_link_to_upper_limb_back_left_pos)
                upper_limb_to_lower_limb_back_left_pub.publish(upper_limb_to_lower_limb_back_left_pos)


                rate.sleep()

                at_desired = True

                for leg in legs:
                    at_desired = leg.at_desired_point()
                    if not at_desired:
                        break


        steps_left_pair = generated_steps_for_a_step(full_step_size, True, -full_step_size / 2, True)
        x_values_left = steps_left_pair[0] 
        y_values_left = steps_left_pair[1]

        steps_right_pair = generated_steps_for_a_step(full_step_size, False, full_step_size / 2, True)
        x_values_right = steps_right_pair[0] 
        y_values_right = steps_right_pair[1]


        for i in range(len(x_values_left)):
            angles = front_left_leg.get_joint_angles(x_values_left[i], y_values_left[i], 0.13002)
            theta1, theta2, theta3 = angles[0]

            base_to_front_left_pos = Float64(theta1)
            swing_link_to_upper_limb_front_left_pos = Float64(theta2)
            upper_limb_to_lower_limb_front_left_pos = Float64(theta3)


            angles = back_right_leg.get_joint_angles(x_values_right[i], y_values_right[i], 0.13002)
            theta1, theta2, theta3 = angles[0]

            base_to_back_right_pos = Float64(theta1)
            swing_link_to_upper_limb_back_right_pos = Float64(theta2)
            upper_limb_to_lower_limb_back_right_pos = Float64(theta3)


            angles = front_right_leg.get_joint_angles(x_values_right[i], y_values_right[i], 0.13002)
            theta1, theta2, theta3 = angles[0]

            base_to_front_right_pos = Float64(theta1)
            swing_link_to_upper_limb_front_right_pos = Float64(theta2)
            upper_limb_to_lower_limb_front_right_pos = Float64(theta3)


            angles = back_left_leg.get_joint_angles(x_values_left[i], y_values_left[i], 0.13002)
            theta1, theta2, theta3 = angles[0]

            base_to_back_left_pos = Float64(theta1)
            swing_link_to_upper_limb_back_left_pos = Float64(theta2)
            upper_limb_to_lower_limb_back_left_pos = Float64(theta3)

            at_desired = False
            while not at_desired: 
                base_to_front_left_pub.publish(base_to_front_left_pos)
                swing_link_to_upper_limb_front_left_pub.publish(swing_link_to_upper_limb_front_left_pos)
                upper_limb_to_lower_limb_front_left_pub.publish(upper_limb_to_lower_limb_front_left_pos)

                base_to_back_right_pub.publish(base_to_back_right_pos)
                swing_link_to_upper_limb_back_right_pub.publish(swing_link_to_upper_limb_back_right_pos)
                upper_limb_to_lower_limb_back_right_pub.publish(upper_limb_to_lower_limb_back_right_pos)

                base_to_front_right_pub.publish(base_to_front_right_pos)
                swing_link_to_upper_limb_front_right_pub.publish(swing_link_to_upper_limb_front_right_pos)
                upper_limb_to_lower_limb_front_right_pub.publish(upper_limb_to_lower_limb_front_right_pos)

                base_to_back_left_pub.publish(base_to_back_left_pos)
                swing_link_to_upper_limb_back_left_pub.publish(swing_link_to_upper_limb_back_left_pos)
                upper_limb_to_lower_limb_back_left_pub.publish(upper_limb_to_lower_limb_back_left_pos)

                rate.sleep()

                at_desired = True

                for leg in legs:
                    at_desired = leg.at_desired_point()
                    if not at_desired:
                        break

    
def generated_steps_for_a_step(full_step_distance, forward=True, x_offset=0, direction_forward=True, default_height=0.7, n_substeps=1):
    # function to generate x, y (x is forward/backward, y[not z due to SolidWorks axis orientation] is upward/downward) coordinates
    # for a step. A full step is travelled in "n" steps/intervals. 
    # full_step_distance: this is the diameter of a circle, where the actual step distance is the length of a segment of this circle
    # subtended at an angle theta (theta is 150 deg by default)
    # forward: this parameter specifies if the step is in the forward direction. True indicates forward
    # x_offset: this parameter gives the offset value in the x direction at the start of the step
    # direction_forward: this parameter specifies the direction of motion of tachyon (forward or backward). True indicates forward
    # and vice versa
    # default_height: this parameter specifies the current height of tachyon/the height that is to be maintained
    # n_substeps: this parameter states the number of substeps required in one full step   

    x_values = []
    y_values = []
    x_dir = 1 if forward else -1
    lift = 0 if forward is direction_forward else 0 # zero lift if foot is supposed to be dragged

    r = full_step_distance / 2
    theta = (np.pi / 6) * 5 # 150 degrees
    d = np.sqrt(np.power(r, 2) + np.power(r, 2) - (2 * np.power(r, 2) * np.cos(theta)))
    h = r * np.cos(theta / 2)
    b = r - (d / 2)

    for alpha in np.arange(((np.pi / 2) - (theta / 2)), 1.001 * ((np.pi / 2) + (theta / 2)), theta / n_substeps):
        x = r - (r * np.cos(alpha))
        y = r * np.sin(alpha)

        x_values.append((x - b + x_offset) * x_dir)
        y_values.append((y - h) * lift - default_height)

    return x_values, y_values
    


if __name__ == "__main__":
    rospy.init_node("creep_node", anonymous=True)
    rospy.wait_for_message("/tachyon/joint_upper_limb_to_lower_limb_back_left_controller/state", JointControllerState)
    creep()