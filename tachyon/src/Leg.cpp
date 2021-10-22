// Author: David Akhihiero
#include <ros/ros.h>
#include <control_msgs/JointControllerState.h>
#include "Leg.h"
#include <tachyon/IK.h>


Leg::Leg(float L1, float H1, float L2, float L3, std::string name, bool right)
{
    this->right = (right? -1.0: 1.0);
    this->L1 = L1;
    this->H1 = H1;
    this->L2 = L2;
    this->L3 = L3;
    this->name = name;
    this->theta1 = 0;
    this->theta2 = 0;
    this->theta3 = 0;

}

std::array<float, 3> Leg::getJointAngles(float x, float y, float z)
{
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<tachyon::IK>("ik_service");
    tachyon::IK srv;
    srv.request.x = x, srv.request.y = y, srv.request.z = z;
    srv.request.L1 = this->L1, srv.request.H1 = this->H1, srv.request.L2 = this->L2, srv.request.L3 = this->L3;
    srv.request.old_theta1 = this->theta1, srv.request.old_theta2 = this->theta2, srv.request.old_theta3 = this->theta3;
    srv.request.right = this->right;

    std::array<float, 3> jointAngles;
    
    if (client.call(srv))
    {
        jointAngles[0] = srv.response.theta1;
        jointAngles[1] = srv.response.theta2;
        jointAngles[2] = srv.response.theta3;
    }
    else
    {
        ROS_ERROR("Failed to call IK service");
    }
    // Reverse joint angle inversion (inversion done by IK Server)
    this->theta1 = jointAngles[0], this->theta2 = jointAngles[1] * this->right, this->theta3 = jointAngles[2] * this->right;
    
    return jointAngles;
}


