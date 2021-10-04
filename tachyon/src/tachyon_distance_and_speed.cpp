#include <ros/ros.h>
#include <tachyon/TachyonDistanceAndSpeed.h>
#include <gazebo_msgs/ModelStates.h>
#include <math.h>

//  Author: David Akhihiero
/* Server for getting the distance travelled, instantaneous speed and average speed */

float initialDistance = 0;
float distance = 0;
float instantaneousSpeed = 0;
float speedSum = 0;
float averageSpeed = 0;
long speedCounts = 0;

bool distanceAndSpeed (tachyon::TachyonDistanceAndSpeed::Request &req, tachyon::TachyonDistanceAndSpeed::Response &res)
{
    res.distance = distance;
    res.instantaneous_speed = instantaneousSpeed;
    res.average_speed = averageSpeed;
    ROS_INFO_STREAM(std::to_string(distance) + "," + std::to_string(instantaneousSpeed) + "," + std::to_string(averageSpeed));
    return true;
}

int getIndex(std::vector<std::string> v, std::string value)
{
    for(int i = 0; i < v.size(); i++)
    {
        if(v[i].compare(value) == 0)
            return i;
    }
    return -1;
}

void modelStatesCallback (gazebo_msgs::ModelStates modelStates) 
{
    int index = getIndex(modelStates.name, "tachyon");

    float xPos = modelStates.pose[index].position.x;
    float yPos = modelStates.pose[index].position.y;
    float zPos = modelStates.pose[index].position.z;

    float xLinSpeed = modelStates.twist[index].linear.x;
    float yLinSpeed = modelStates.twist[index].linear.y;
    float zLinSpeed = modelStates.twist[index].linear.z;

    if (initialDistance == 0)
    {
        initialDistance = sqrt(xPos*xPos + yPos*yPos + zPos*zPos);
    }
    distance = sqrt(xPos*xPos + yPos*yPos) - initialDistance;
    instantaneousSpeed = sqrt(xLinSpeed*xLinSpeed + yLinSpeed*yLinSpeed);
    speedSum += instantaneousSpeed;

    averageSpeed = speedSum / (++speedCounts);
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "distance_and_speed_server_node");
    ros::NodeHandle nodeHandle;


    ros::Subscriber modelStatesSub = nodeHandle.subscribe("/gazebo/model_states", 100, modelStatesCallback);

    ros::ServiceServer service = nodeHandle.advertiseService("tachyon_distance_and_speed", distanceAndSpeed);
    
    ros::spin();
    return 0;
}

