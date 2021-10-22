#! /usr/bin/env python
#  Author: David Akhihiero
# Code to make tachyon stand 

import rospy
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from tachyon.msg import JointAnglesErrorIsBelowTolerance
from Leg import Leg


is_at_desired = False
count = 0

def joint_angles_error_callback(msg):
    global is_at_desired
    is_at_desired = msg.is_at_desired

def stand():
    global is_at_desired, count

    base_to_front_left_pub = rospy.Publisher("/tachyon/joint_base_to_front_left_controller/command", Float64, queue_size=10)
    swing_link_to_upper_limb_front_left_pub = rospy.Publisher("/tachyon/joint_swing_link_to_upper_limb_front_left_controller/command", 
    Float64, queue_size=10)
    upper_limb_to_lower_limb_front_left_pub = rospy.Publisher("/tachyon/joint_upper_limb_to_lower_limb_front_left_controller/command", 
    Float64, queue_size=10)

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
    

    # setting default_height as 0.7m
    default_height = 0.7

    # create Leg objects for each leg, specifying the leg parameters: L1, H1, L2, L3, leg_name and if the leg
    # is on the right or on the left side of tachyon and use the get_joint_angles() method to compute the
    # inverse kinematics

    front_left_leg = Leg (0.13002, 0.08274, 0.39825, 0.33, 'front_left', False)
    angles = front_left_leg.get_joint_angles(0, -default_height, 0.13002)
    theta1, theta2, theta3 = angles[0]

    base_to_front_left_pos = Float64(theta1)
    swing_link_to_upper_limb_front_left_pos = Float64(theta2)
    upper_limb_to_lower_limb_front_left_pos = Float64(theta3)

    front_right_leg = Leg (0.13002, 0.08274, 0.39825, 0.33, 'front_right', True)
    angles = front_right_leg.get_joint_angles(0, -default_height, 0.13002)
    theta1, theta2, theta3 = angles[0]

    base_to_front_right_pos = Float64(theta1)
    swing_link_to_upper_limb_front_right_pos = Float64(theta2)
    upper_limb_to_lower_limb_front_right_pos = Float64(theta3)

    back_left_leg = Leg (0.13002, 0.08274, 0.39825, 0.33, 'back_left', False)
    angles = back_left_leg.get_joint_angles(0, -default_height, 0.13002)
    theta1, theta2, theta3 = angles[0]

    base_to_back_left_pos = Float64(theta1)
    swing_link_to_upper_limb_back_left_pos = Float64(theta2)
    upper_limb_to_lower_limb_back_left_pos = Float64(theta3)

    back_right_leg = Leg (0.13002, 0.08274, 0.39825, 0.33, 'back_right', True)
    angles = back_right_leg.get_joint_angles(0, -default_height, 0.13002)
    theta1, theta2, theta3 = angles[0]

    base_to_back_right_pos = Float64(theta1)
    swing_link_to_upper_limb_back_right_pos = Float64(theta2)
    upper_limb_to_lower_limb_back_right_pos = Float64(theta3)

    rate = rospy.Rate(50)

    standing = False
    while not standing and not rospy.is_shutdown(): 
        base_to_front_left_pub.publish(base_to_front_left_pos)
        swing_link_to_upper_limb_front_left_pub.publish(swing_link_to_upper_limb_front_left_pos)
        upper_limb_to_lower_limb_front_left_pub.publish(upper_limb_to_lower_limb_front_left_pos)

        base_to_front_right_pub.publish(base_to_front_right_pos)
        swing_link_to_upper_limb_front_right_pub.publish(swing_link_to_upper_limb_front_right_pos)
        upper_limb_to_lower_limb_front_right_pub.publish(upper_limb_to_lower_limb_front_right_pos)

        base_to_back_left_pub.publish(base_to_back_left_pos)
        swing_link_to_upper_limb_back_left_pub.publish(swing_link_to_upper_limb_back_left_pos)
        upper_limb_to_lower_limb_back_left_pub.publish(upper_limb_to_lower_limb_back_left_pos)

        base_to_back_right_pub.publish(base_to_back_right_pos)
        swing_link_to_upper_limb_back_right_pub.publish(swing_link_to_upper_limb_back_right_pos)
        upper_limb_to_lower_limb_back_right_pub.publish(upper_limb_to_lower_limb_back_right_pos)
            
        rate.sleep()
        standing = is_at_desired
    
    if count < 3:
        count += 1
        stand()

        


if __name__ == "__main__":
    rospy.init_node("stand_node", anonymous=True)
    rospy.wait_for_message("/tachyon/joint_upper_limb_to_lower_limb_back_right_controller/state", JointControllerState)
    rospy.Subscriber("joint_angles_error_publisher", JointAnglesErrorIsBelowTolerance, joint_angles_error_callback)
    stand()


