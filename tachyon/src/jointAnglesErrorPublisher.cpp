#include <ros/ros.h>
#include <tachyon/JointAnglesErrorIsBelowTolerance.h>
#include <control_msgs/JointControllerState.h>

// Author: David Akhihiero
/* Server for determining if the joints are close to their desired positions */

float theta1ErrorFrontLeft = 100, theta2ErrorFrontLeft = 100, theta3ErrorFrontLeft = 100;
float theta1ErrorFrontRight = 100, theta2ErrorFrontRight = 100, theta3ErrorFrontRight = 100;
float theta1ErrorBackLeft = 100, theta2ErrorBackLeft = 100, theta3ErrorBackLeft = 100;
float theta1ErrorBackRight = 100, theta2ErrorBackRight = 100, theta3ErrorBackRight = 100;

ros::Publisher publisher;

bool isAtDesired()
{
    float errors[] = {theta1ErrorFrontLeft, theta2ErrorFrontLeft, theta3ErrorFrontLeft,
                        theta1ErrorFrontRight, theta2ErrorFrontRight, theta3ErrorFrontRight,
                        theta1ErrorBackLeft, theta2ErrorBackLeft, theta3ErrorBackLeft,
                        theta1ErrorBackRight, theta2ErrorBackRight, theta3ErrorBackRight};

    float maxError = errors[0];
    int size = sizeof(errors) / sizeof(errors[0]);

    for (int i = 1;i < size;i++)
    {
        if (errors[i] > maxError)
            maxError = errors[i];
    } 

    return maxError < 0.03;                  
}

void bodyToSwingCallbackFrontLeft(const control_msgs::JointControllerState::ConstPtr &jointState)
{
    theta1ErrorFrontLeft = jointState->error;
    tachyon::JointAnglesErrorIsBelowTolerance msg;
    msg.is_at_desired = isAtDesired();
    publisher.publish(msg);
    ros::spinOnce();
}

void swingToUpperCallbackFrontLeft(const control_msgs::JointControllerState::ConstPtr &jointState)
{   
    theta2ErrorFrontLeft = jointState->error;
    tachyon::JointAnglesErrorIsBelowTolerance msg;
    msg.is_at_desired = isAtDesired();
    publisher.publish(msg);
    ros::spinOnce();
}

void upperToLowerCallbackFrontLeft(const control_msgs::JointControllerState::ConstPtr &jointState)
{
    theta3ErrorFrontLeft = jointState->error;
    tachyon::JointAnglesErrorIsBelowTolerance msg;
    msg.is_at_desired = isAtDesired();
    publisher.publish(msg);
    ros::spinOnce();
}

void bodyToSwingCallbackFrontRight(const control_msgs::JointControllerState::ConstPtr &jointState)
{    
    theta1ErrorFrontRight = jointState->error;
    tachyon::JointAnglesErrorIsBelowTolerance msg;
    msg.is_at_desired = isAtDesired();
    publisher.publish(msg);
    ros::spinOnce();
}

void swingToUpperCallbackFrontRight(const control_msgs::JointControllerState::ConstPtr &jointState)
{
    theta2ErrorFrontRight = jointState->error;
    tachyon::JointAnglesErrorIsBelowTolerance msg;
    msg.is_at_desired = isAtDesired();
    publisher.publish(msg);
    ros::spinOnce();
}

void upperToLowerCallbackFrontRight(const control_msgs::JointControllerState::ConstPtr &jointState)
{
    theta3ErrorFrontRight = jointState->error;
    tachyon::JointAnglesErrorIsBelowTolerance msg;
    msg.is_at_desired = isAtDesired();
    publisher.publish(msg);
    ros::spinOnce();
}


void bodyToSwingCallbackBackLeft(const control_msgs::JointControllerState::ConstPtr &jointState)
{
    theta1ErrorBackLeft = jointState->error;
    tachyon::JointAnglesErrorIsBelowTolerance msg;
    msg.is_at_desired = isAtDesired();
    publisher.publish(msg);
    ros::spinOnce();
}

void swingToUpperCallbackBackLeft(const control_msgs::JointControllerState::ConstPtr &jointState)
{
    theta2ErrorBackLeft = jointState->error;
    tachyon::JointAnglesErrorIsBelowTolerance msg;
    msg.is_at_desired = isAtDesired();
    publisher.publish(msg);
    ros::spinOnce();
}

void upperToLowerCallbackBackLeft(const control_msgs::JointControllerState::ConstPtr &jointState)
{
    theta3ErrorBackLeft = jointState->error;
    tachyon::JointAnglesErrorIsBelowTolerance msg;
    msg.is_at_desired = isAtDesired();
    publisher.publish(msg);
    ros::spinOnce();
}


void bodyToSwingCallbackBackRight(const control_msgs::JointControllerState::ConstPtr &jointState)
{
    theta1ErrorBackRight = jointState->error;
    tachyon::JointAnglesErrorIsBelowTolerance msg;
    msg.is_at_desired = isAtDesired();
    publisher.publish(msg);
    ros::spinOnce();
}

void swingToUpperCallbackBackRight(const control_msgs::JointControllerState::ConstPtr &jointState)
{
    theta2ErrorBackRight = jointState->error;
    tachyon::JointAnglesErrorIsBelowTolerance msg;
    msg.is_at_desired = isAtDesired();
    publisher.publish(msg);
    ros::spinOnce();
}

void upperToLowerCallbackBackRight(const control_msgs::JointControllerState::ConstPtr &jointState)
{
    theta3ErrorBackRight = jointState->error;
    tachyon::JointAnglesErrorIsBelowTolerance msg;
    msg.is_at_desired = isAtDesired();
    publisher.publish(msg);
    ros::spinOnce();
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "joint_angles_error_publisher_node");
    ros::NodeHandle nodeHandle;
    const std::string msg = "/tachyon/joint_upper_limb_to_lower_limb_back_right_controller/state";

    ros::topic::waitForMessage<control_msgs::JointControllerState>(msg);
  
    publisher = nodeHandle.advertise<tachyon::JointAnglesErrorIsBelowTolerance>("joint_angles_error_publisher", 5);

    ros::Subscriber sub1 = nodeHandle.subscribe("/tachyon/joint_base_to_front_left_controller/state", 10, bodyToSwingCallbackFrontLeft);
    ros::Subscriber sub2 = nodeHandle.subscribe("/tachyon/joint_swing_link_to_upper_limb_front_left_controller/state", 10, swingToUpperCallbackFrontLeft);
    ros::Subscriber sub3 = nodeHandle.subscribe("/tachyon/joint_upper_limb_to_lower_limb_front_left_controller/state", 10, upperToLowerCallbackFrontLeft);

    ros::Subscriber sub4 = nodeHandle.subscribe("/tachyon/joint_base_to_front_right_controller/state", 10, bodyToSwingCallbackFrontRight);
    ros::Subscriber sub5 = nodeHandle.subscribe("/tachyon/joint_swing_link_to_upper_limb_front_right_controller/state", 10, swingToUpperCallbackFrontRight);
    ros::Subscriber sub6 = nodeHandle.subscribe("/tachyon/joint_upper_limb_to_lower_limb_front_right_controller/state", 10, upperToLowerCallbackFrontRight);

    ros::Subscriber sub7 = nodeHandle.subscribe("/tachyon/joint_base_to_back_left_controller/state", 10, bodyToSwingCallbackBackLeft);
    ros::Subscriber sub8 = nodeHandle.subscribe("/tachyon/joint_swing_link_to_upper_limb_back_left_controller/state", 10, swingToUpperCallbackBackLeft);
    ros::Subscriber sub9 = nodeHandle.subscribe("/tachyon/joint_upper_limb_to_lower_limb_back_left_controller/state", 10, upperToLowerCallbackBackLeft);

    ros::Subscriber sub10 = nodeHandle.subscribe("/tachyon/joint_base_to_back_right_controller/state", 10, bodyToSwingCallbackBackRight);
    ros::Subscriber sub11 = nodeHandle.subscribe("/tachyon/joint_swing_link_to_upper_limb_back_right_controller/state", 10, swingToUpperCallbackBackRight);
    ros::Subscriber sub12 = nodeHandle.subscribe("/tachyon/joint_upper_limb_to_lower_limb_back_right_controller/state", 10, upperToLowerCallbackBackRight);

    ros::spin();

    return 0;
}

