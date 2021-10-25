// Author: David Akhihiero

// Cpp Code to make tachyon trot 

#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include <tachyon/JointAnglesErrorIsBelowTolerance.h>
#include "Leg.h"
#include "generateSteps.h"

bool isAtDesired = false;
int count = 0;
float L1 = 0.13002, H1 = 0.08274, L2 = 0.39825, L3 = 0.33;

void jointAnglesErrorCallback(const tachyon::JointAnglesErrorIsBelowTolerance::ConstPtr &msg)
{
    isAtDesired = msg->is_at_desired;
}

void trot()
{
    ros::NodeHandle nh;
    float fullStepSize = 0.20;

    Leg frontLeftLeg(L1, H1, L2, L3, "front_left", false);
    Leg frontRightLeg(L1, H1, L2, L3, "front_right", true);
    Leg backLeftLeg(L1, H1, L2, L3, "back_left", false);
    Leg backRightLeg(L1, H1, L2, L3, "back_right", true);

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

    // setting default_height as 0.7m
    float defaultHeight = 0.7;

    // create Leg objects for each leg, specifying the leg parameters: L1, H1, L2, L3, leg_name and if the leg
    // is on the right or on the left side of tachyon and use the get_joint_angles() method to compute the
    // inverse kinematics

    std_msgs::Float64 baseToFrontLeftPos;
    std_msgs::Float64 swingLinkToUpperLimbFrontLeftPos;
    std_msgs::Float64 upperLimbToLowerLimbFrontLeftPos;

    std_msgs::Float64 baseToFrontRightPos;
    std_msgs::Float64 swingLinkToUpperLimbFrontRightPos;
    std_msgs::Float64 upperLimbToLowerLimbFrontRightPos;
  
    std_msgs::Float64 baseToBackLeftPos;
    std_msgs::Float64 swingLinkToUpperLimbBackLeftPos;
    std_msgs::Float64 upperLimbToLowerLimbBackLeftPos;

    std_msgs::Float64 baseToBackRightPos;
    std_msgs::Float64 swingLinkToUpperLimbBackRightPos;
    std_msgs::Float64 upperLimbToLowerLimbBackRightPos;

    ros::Rate rate(20);

    // first move diagonal pair (1) forward half a step, (2) backward half a step
    // then move diagonal pair (2) forward a full step, then (1) backward a full step then the reverse, in a loop

    int nSubsteps = 3;

    std::array<std::array<float, 4>, 2> stepsDiagPair1 = genSteps(fullStepSize / 2, true, 0, true);
    float xOffset1 = stepsDiagPair1[0][3];
    std::array<std::array<float, 4>, 2> stepsDiagPair2 = genSteps(fullStepSize / 2, false, 0, true);
    float xOffset2 = stepsDiagPair2[0][3];

    std::array<float, 4> xVals1 = stepsDiagPair1[0];
    std::array<float, 4> yVals1 = stepsDiagPair1[1];

    std::array<float, 4> xVals2 = stepsDiagPair2[0];
    std::array<float, 4> yVals2 = stepsDiagPair2[1];


    int size = sizeof(xVals1) / sizeof(xVals1[0]);
    for (int i = 0;i < size;i++)
    {
        std::array<float, 3> angles = frontLeftLeg.getJointAngles(xVals1[i], yVals1[i], L1);
        float theta1 = angles[0], theta2 = angles[1], theta3 = angles[2];

        baseToFrontLeftPos.data = theta1;
        swingLinkToUpperLimbFrontLeftPos.data = theta2;
        upperLimbToLowerLimbFrontLeftPos.data = theta3;

        angles = backLeftLeg.getJointAngles(xVals2[i], yVals2[i], L1);
        theta1 = angles[0], theta2 = angles[1], theta3 = angles[2];

        baseToBackLeftPos.data = theta1;
        swingLinkToUpperLimbBackLeftPos.data = theta2;
        upperLimbToLowerLimbBackLeftPos.data = theta3;

        angles = frontRightLeg.getJointAngles(xVals2[i], yVals2[i], L1);
        theta1 = angles[0], theta2 = angles[1], theta3 = angles[2];

        baseToFrontRightPos.data = theta1;
        swingLinkToUpperLimbFrontRightPos.data = theta2;
        upperLimbToLowerLimbFrontRightPos.data = theta3;

        angles = backRightLeg.getJointAngles(xVals1[i], yVals1[i], L1);
        theta1 = angles[0], theta2 = angles[1], theta3 = angles[2];

        baseToBackRightPos.data = theta1;
        swingLinkToUpperLimbBackRightPos.data = theta2;
        upperLimbToLowerLimbBackRightPos.data = theta3;


        bool atDesired = false;
        while (!atDesired && ros::ok())
        {
            baseToFrontLeftPub.publish(baseToFrontLeftPos);
            swingLinkToUpperLimbFrontLeftPub.publish(swingLinkToUpperLimbFrontLeftPos);
            upperLimbToLowerLimbFrontLeftPub.publish(upperLimbToLowerLimbFrontLeftPos);

            baseToBackRightPub.publish(baseToBackRightPos);
            swingLinkToUpperLimbBackRightPub.publish(swingLinkToUpperLimbBackRightPos);
            upperLimbToLowerLimbBackRightPub.publish(upperLimbToLowerLimbBackRightPos);

            baseToFrontRightPub.publish(baseToFrontRightPos);
            swingLinkToUpperLimbFrontRightPub.publish(swingLinkToUpperLimbFrontRightPos);
            upperLimbToLowerLimbFrontRightPub.publish(upperLimbToLowerLimbFrontRightPos);

            baseToBackLeftPub.publish(baseToBackLeftPos);
            swingLinkToUpperLimbBackLeftPub.publish(swingLinkToUpperLimbBackLeftPos);
            upperLimbToLowerLimbBackLeftPub.publish(upperLimbToLowerLimbBackLeftPos);
            
            ros::spinOnce();
            rate.sleep();

            atDesired = isAtDesired;
        }
    }
    

    while (true && ros::ok())
    {
        // First phase of motion
        stepsDiagPair1 = genSteps(fullStepSize, false, xOffset1, true);
        stepsDiagPair2 = genSteps(fullStepSize, true, xOffset2, true);

        xVals1 = stepsDiagPair1[0];
        yVals1 = stepsDiagPair1[1];

        xVals2 = stepsDiagPair2[0];
        yVals2 = stepsDiagPair2[1];

        int size = sizeof(xVals1) / sizeof(xVals1[0]);
        for (int i = 0;i < size;i++)
        {
            std::array<float, 3> angles = frontLeftLeg.getJointAngles(xVals1[i], yVals1[i], L1);
            float theta1 = angles[0], theta2 = angles[1], theta3 = angles[2];

            baseToFrontLeftPos.data = theta1;
            swingLinkToUpperLimbFrontLeftPos.data = theta2;
            upperLimbToLowerLimbFrontLeftPos.data = theta3;

            angles = backLeftLeg.getJointAngles(xVals2[i], yVals2[i], L1);
            theta1 = angles[0], theta2 = angles[1], theta3 = angles[2];

            baseToBackLeftPos.data = theta1;
            swingLinkToUpperLimbBackLeftPos.data = theta2;
            upperLimbToLowerLimbBackLeftPos.data = theta3;

            angles = frontRightLeg.getJointAngles(xVals2[i], yVals2[i], L1);
            theta1 = angles[0], theta2 = angles[1], theta3 = angles[2];

            baseToFrontRightPos.data = theta1;
            swingLinkToUpperLimbFrontRightPos.data = theta2;
            upperLimbToLowerLimbFrontRightPos.data = theta3;

            angles = backRightLeg.getJointAngles(xVals1[i], yVals1[i], L1);
            theta1 = angles[0], theta2 = angles[1], theta3 = angles[2];

            baseToBackRightPos.data = theta1;
            swingLinkToUpperLimbBackRightPos.data = theta2;
            upperLimbToLowerLimbBackRightPos.data = theta3;


            bool atDesired = false;
            while (!atDesired && ros::ok())
            {
                baseToFrontLeftPub.publish(baseToFrontLeftPos);
                swingLinkToUpperLimbFrontLeftPub.publish(swingLinkToUpperLimbFrontLeftPos);
                upperLimbToLowerLimbFrontLeftPub.publish(upperLimbToLowerLimbFrontLeftPos);

                baseToBackRightPub.publish(baseToBackRightPos);
                swingLinkToUpperLimbBackRightPub.publish(swingLinkToUpperLimbBackRightPos);
                upperLimbToLowerLimbBackRightPub.publish(upperLimbToLowerLimbBackRightPos);

                baseToFrontRightPub.publish(baseToFrontRightPos);
                swingLinkToUpperLimbFrontRightPub.publish(swingLinkToUpperLimbFrontRightPos);
                upperLimbToLowerLimbFrontRightPub.publish(upperLimbToLowerLimbFrontRightPos);

                baseToBackLeftPub.publish(baseToBackLeftPos);
                swingLinkToUpperLimbBackLeftPub.publish(swingLinkToUpperLimbBackLeftPos);
                upperLimbToLowerLimbBackLeftPub.publish(upperLimbToLowerLimbBackLeftPos);
                
                ros::spinOnce();
                rate.sleep();

                atDesired = isAtDesired;
            }
        }

        // Second phase of motion
        stepsDiagPair1 = genSteps(fullStepSize, true, xOffset2, true);
        stepsDiagPair2 = genSteps(fullStepSize, false, xOffset1, true);

        xVals1 = stepsDiagPair1[0];
        yVals1 = stepsDiagPair1[1];

        xVals2 = stepsDiagPair2[0];
        yVals2 = stepsDiagPair2[1];

        size = sizeof(xVals1) / sizeof(xVals1[0]);
        for (int i = 0;i < size;i++)
        {
            std::array<float, 3> angles = frontLeftLeg.getJointAngles(xVals1[i], yVals1[i], L1);
            float theta1 = angles[0], theta2 = angles[1], theta3 = angles[2];

            baseToFrontLeftPos.data = theta1;
            swingLinkToUpperLimbFrontLeftPos.data = theta2;
            upperLimbToLowerLimbFrontLeftPos.data = theta3;

            angles = backLeftLeg.getJointAngles(xVals2[i], yVals2[i], L1);
            theta1 = angles[0], theta2 = angles[1], theta3 = angles[2];

            baseToBackLeftPos.data = theta1;
            swingLinkToUpperLimbBackLeftPos.data = theta2;
            upperLimbToLowerLimbBackLeftPos.data = theta3;

            angles = frontRightLeg.getJointAngles(xVals2[i], yVals2[i], L1);
            theta1 = angles[0], theta2 = angles[1], theta3 = angles[2];

            baseToFrontRightPos.data = theta1;
            swingLinkToUpperLimbFrontRightPos.data = theta2;
            upperLimbToLowerLimbFrontRightPos.data = theta3;

            angles = backRightLeg.getJointAngles(xVals1[i], yVals1[i], L1);
            theta1 = angles[0], theta2 = angles[1], theta3 = angles[2];

            baseToBackRightPos.data = theta1;
            swingLinkToUpperLimbBackRightPos.data = theta2;
            upperLimbToLowerLimbBackRightPos.data = theta3;


            bool atDesired = false;
            while (!atDesired && ros::ok())
            {
                baseToFrontLeftPub.publish(baseToFrontLeftPos);
                swingLinkToUpperLimbFrontLeftPub.publish(swingLinkToUpperLimbFrontLeftPos);
                upperLimbToLowerLimbFrontLeftPub.publish(upperLimbToLowerLimbFrontLeftPos);

                baseToBackRightPub.publish(baseToBackRightPos);
                swingLinkToUpperLimbBackRightPub.publish(swingLinkToUpperLimbBackRightPos);
                upperLimbToLowerLimbBackRightPub.publish(upperLimbToLowerLimbBackRightPos);

                baseToFrontRightPub.publish(baseToFrontRightPos);
                swingLinkToUpperLimbFrontRightPub.publish(swingLinkToUpperLimbFrontRightPos);
                upperLimbToLowerLimbFrontRightPub.publish(upperLimbToLowerLimbFrontRightPos);

                baseToBackLeftPub.publish(baseToBackLeftPos);
                swingLinkToUpperLimbBackLeftPub.publish(swingLinkToUpperLimbBackLeftPos);
                upperLimbToLowerLimbBackLeftPub.publish(upperLimbToLowerLimbBackLeftPos);
                
                ros::spinOnce();
                rate.sleep();

                atDesired = isAtDesired;
            }
        }
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "trot_node_cpp");
    ros::NodeHandle nh;

    const std::string msg = "/tachyon/joint_upper_limb_to_lower_limb_back_right_controller/state";

    ros::topic::waitForMessage<control_msgs::JointControllerState>(msg);

    ros::Subscriber sub = nh.subscribe("joint_angles_error_publisher", 10, jointAnglesErrorCallback);

    ros::spinOnce();

    trot();

}

