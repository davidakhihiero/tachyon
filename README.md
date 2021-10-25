# tachyon
A 12- DOF quadruped (simulation) 

In this project, I modelled a quadruped mobile robot with four legs (3 revolute joints each) for simulation in Gazebo. The URDF was generated from a SolidWorks
assembly of the model and I edited it to suit my requirements.

ROS nodes are in both C++ and Python. For almost every Python node, there is an equivalent C++ node.

## Nodes:  
**stand_node, stand_node_cpp:** Python and C++ nodes to get the robot in stance.

**creep_node, creep_node_cpp:** Python and C++ nodes to get the robot to creep forward.

**trot_node, trot_node_cpp:** Python and C++ nodes to get the robot to trot forward.

**turn_left_node, turn_left_node_cpp:** Python and C++ nodes to get the robot to turn left.

**IK_Server_node:** Python server node to compute the inverse kinematics for a robot leg and return the result to a client.

**joint_angles_error_publisher_node:** C++ publisher node that subscribes to the joint states for all the revolute joints and publishes a boolean message; 
true if all joint errors are below a certain threshold, else false.

**tachyon_distance_and_speed_server_node:** C++ server node that computes the distance travelled, instantaneous speed and average speed of tachyon, in the world frame,
and returns the result to a client.

## Launch Files: There are 3 launch files  
**tachyon_rviz:** starts rviz, loads the robot model for display and starts the joint state publisher and robot state publisher.

**tachyon_gazebo launch file:** starts gazebo and loads the robot model and environment

**tachyon_control:** loads all the joint controllers, starts the robot state publisher, inverse kinematics server, joint_angles_error_publisher and 
tachyon_distance_and_speed_server nodes.

## Procedure for a Basic Simulation
1. Start the gazebo launch file: **roslaunch tachyon tachyon_gazebo.launch** 
2. Start the controller: **roslaunch tachyon tachyon_control.launch**
3. Set tachyon in stance: **rosrun tachyon stand (or rosrun tachyon stand.py)**
4. Get tachyon to creep, trot or turn left:  
**rosrun tachyon creep (or rosrun tachyon creep.py)**  
**rosrun tachyon trot (or rosrun tachyon trot.py)**  
**rosrun tachyon turnLeft (rosrun tachyon turn_left.py)**

Feel free to create more interesting features :)




