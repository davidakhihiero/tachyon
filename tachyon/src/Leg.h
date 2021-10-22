#include <iostream>
#include <control_msgs/JointControllerState.h>
#include <ros/ros.h>


class Leg
{
    public:
    Leg(float L1, float H1, float L2, float L3, std::string name, bool right);
    std::array<float, 3> getJointAngles(float x, float y, float z);

    private:
    float L1, H1, L2, L3, theta1, theta2, theta3;
    std::string name;
    float right;
   
};