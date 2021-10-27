// Author: David Akhihiero

// Cpp Code to make tachyon stand 

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include <tachyon/JointAnglesErrorIsBelowTolerance.h>
#include "Leg.h"

bool isAtDesired = false;
int count = 0;
float L1 = 0.1, H1 = 0.0, L2 = 0.4, L3 = 0.4;

void jointAnglesErrorCallback(const tachyon::JointAnglesErrorIsBelowTolerance::ConstPtr &msg)
{
    isAtDesired = msg->is_at_desired;
}

void stand()
{
    ros::NodeHandle nh;

    ros::Publisher baseToFrontLeftPub = nh.advertise<std_msgs::Float64>("/tachyon/joint_base_to_front_left_controller/command", 10);
    ros::Publisher swingLinkToUpperLimbFrontLeftPub = nh.advertise<std_msgs::Float64>("/tachyon/joint_swing_link_to_upper_limb_front_left_controller/command", 10);
    ros::Publisher upperLimbToLowerLimbFrontLeftPub = nh.advertise<std_msgs::Float64>("/tachyon/joint_upper_limb_to_lower_limb_front_left_controller/command", 10);

    ros::Publisher baseToFrontRightPub = nh.advertise<std_msgs::Float64>("/tachyon/joint_base_to_front_right_controller/command", 10);
    ros::Publisher swingLinkToUpperLimbFrontRightPub = nh.advertise<std_msgs::Float64>("/tachyon/joint_swing_link_to_upper_limb_front_right_controller/command", 10);
    ros::Publisher upperLimbToLowerLimbFrontRightPub = nh.advertise<std_msgs::Float64>("/tachyon/joint_upper_limb_to_lower_limb_front_right_controller/command", 10);

    ros::Publisher baseToBackLeftPub = nh.advertise<std_msgs::Float64>("/tachyon/joint_base_to_back_left_controller/command", 10);
    ros::Publisher swingLinkToUpperLimbBackLeftPub = nh.advertise<std_msgs::Float64>("/tachyon/joint_swing_link_to_upper_limb_back_left_controller/command", 10);
    ros::Publisher upperLimbToLowerLimbBackLeftPub = nh.advertise<std_msgs::Float64>("/tachyon/joint_upper_limb_to_lower_limb_back_left_controller/command", 10);

    ros::Publisher baseToBackRightPub = nh.advertise<std_msgs::Float64>("/tachyon/joint_base_to_back_right_controller/command", 10);
    ros::Publisher swingLinkToUpperLimbBackRightPub = nh.advertise<std_msgs::Float64>("/tachyon/joint_swing_link_to_upper_limb_back_right_controller/command", 10);
    ros::Publisher upperLimbToLowerLimbBackRightPub = nh.advertise<std_msgs::Float64>("/tachyon/joint_upper_limb_to_lower_limb_back_right_controller/command", 10);

    // setting default_height as 0.6m
    float defaultHeight = 0.6;

    // create Leg objects for each leg, specifying the leg parameters: L1, H1, L2, L3, leg_name and if the leg
    // is on the right or on the left side of tachyon and use the get_joint_angles() method to compute the
    // inverse kinematics

    Leg frontLeftLeg(L1, H1, L2, L3, "front_left", false);
    std::array<float, 3> angles = frontLeftLeg.getJointAngles(0, -defaultHeight, L1);
    float theta1 = angles[0], theta2 = angles[1], theta3 = angles[2];

    std_msgs::Float64 baseToFrontLeftPos;
    std_msgs::Float64 swingLinkToUpperLimbFrontLeftPos;
    std_msgs::Float64 upperLimbToLowerLimbFrontLeftPos;
    baseToFrontLeftPos.data = theta1;
    swingLinkToUpperLimbFrontLeftPos.data = theta2;
    upperLimbToLowerLimbFrontLeftPos.data = theta3;
    


    Leg frontRightLeg(L1, H1, L2, L3, "front_right", true);
    angles = frontRightLeg.getJointAngles(0, -defaultHeight, L1);
    theta1 = angles[0], theta2 = angles[1], theta3 = angles[2];

    std_msgs::Float64 baseToFrontRightPos;
    std_msgs::Float64 swingLinkToUpperLimbFrontRightPos;
    std_msgs::Float64 upperLimbToLowerLimbFrontRightPos;
    baseToFrontRightPos.data = theta1;
    swingLinkToUpperLimbFrontRightPos.data = theta2;
    upperLimbToLowerLimbFrontRightPos.data = theta3;
    

    Leg backLeftLeg(L1, H1, L2, L3, "back_left", false);
    angles = backLeftLeg.getJointAngles(0, -defaultHeight, L1);
    theta1 = angles[0], theta2 = angles[1], theta3 = angles[2];

    std_msgs::Float64 baseToBackLeftPos;
    std_msgs::Float64 swingLinkToUpperLimbBackLeftPos;
    std_msgs::Float64 upperLimbToLowerLimbBackLeftPos;
    baseToBackLeftPos.data = theta1;
    swingLinkToUpperLimbBackLeftPos.data = theta2;
    upperLimbToLowerLimbBackLeftPos.data = theta3;
    


    Leg backRightLeg(L1, H1, L2, L3, "back_right", true);
    angles = backRightLeg.getJointAngles(0, -defaultHeight, L1);
    theta1 = angles[0], theta2 = angles[1], theta3 = angles[2];

    std_msgs::Float64 baseToBackRightPos;
    std_msgs::Float64 swingLinkToUpperLimbBackRightPos;
    std_msgs::Float64 upperLimbToLowerLimbBackRightPos;
    baseToBackRightPos.data = theta1;
    swingLinkToUpperLimbBackRightPos.data = theta2;
    upperLimbToLowerLimbBackRightPos.data = theta3;

    ros::Rate rate(50);

    bool standing = false;

    while (!standing && ros::ok())
    {
        baseToFrontLeftPub.publish(baseToFrontLeftPos);
        swingLinkToUpperLimbFrontLeftPub.publish(swingLinkToUpperLimbFrontLeftPos);
        upperLimbToLowerLimbFrontLeftPub.publish(upperLimbToLowerLimbFrontLeftPos);

        baseToFrontRightPub.publish(baseToFrontRightPos);
        swingLinkToUpperLimbFrontRightPub.publish(swingLinkToUpperLimbFrontRightPos);
        upperLimbToLowerLimbFrontRightPub.publish(upperLimbToLowerLimbFrontRightPos);

        baseToBackLeftPub.publish(baseToBackLeftPos);
        swingLinkToUpperLimbBackLeftPub.publish(swingLinkToUpperLimbBackLeftPos);
        upperLimbToLowerLimbBackLeftPub.publish(upperLimbToLowerLimbBackLeftPos);

        baseToBackRightPub.publish(baseToBackRightPos);
        swingLinkToUpperLimbBackRightPub.publish(swingLinkToUpperLimbBackRightPos);
        upperLimbToLowerLimbBackRightPub.publish(upperLimbToLowerLimbBackRightPos);
        rate.sleep();
        ros::spinOnce();

        standing = isAtDesired;

    }

    if (count < 3)
    {
        count++;
        stand();
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "stand_node_cpp");
    ros::NodeHandle nh;

    const std::string msg = "/tachyon/joint_upper_limb_to_lower_limb_back_right_controller/state";

    ros::topic::waitForMessage<control_msgs::JointControllerState>(msg);

    ros::Subscriber sub = nh.subscribe("joint_angles_error_publisher", 10, jointAnglesErrorCallback);

    ros::spinOnce();

    stand();

}

