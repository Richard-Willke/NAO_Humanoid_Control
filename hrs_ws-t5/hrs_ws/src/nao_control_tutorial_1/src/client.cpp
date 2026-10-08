#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "nao_control_tutorial_1/MoveJoints.h"
#include <string.h>
#include <vector>
using namespace std;

// set these values according to the tasks. mode can take values between 1 or 2 (integer). for ex 3 please execute the move_client.py node 
// to use set angles, set angles_interpolate as false, otherwise set it to true to interpolate 
bool angles_interpolate = false;
int mode = 2;

class Nao_control
{
public:
    // ros handler
    ros::NodeHandle nh_;

    // subscriber to joint states
    ros::Subscriber sensor_data_sub;
    ros::ServiceClient client = nh_.serviceClient<nao_control_tutorial_1::MoveJoints>("move_joints");
    nao_control_tutorial_1::MoveJoints srv;
    double joints_values_;

    Nao_control()
    {
        sensor_data_sub=nh_.subscribe("/joint_states",1, &Nao_control::sensorCallback, this);
    }
    ~Nao_control()
    {
    }

    void callSrv(vector<double> &pose)
    {
        srv.request.target_pose.resize(pose.size());
        srv.request.set_angles = angles_interpolate;
        srv.request.mode = mode;

        for (int i=0; i < pose.size(); i++)
        {
            srv.request.target_pose[i] = pose[i];
        }
        

        if (client.call(srv))
        {
            for(int i; i <= sizeof(srv.response.current_pose); i++)
            {
                std::cout<< "curr joint value: " << srv.response.current_pose[i] <<std::endl;
            }   
        }

        else
        {
            ROS_ERROR("Failed to call service");
        }
    }

  //handler for joint states
    void sensorCallback(const sensor_msgs::JointState::ConstPtr& jointState)
    {
        // std::cout << "GOT MSG \n";
        joints_values_ = jointState->position[0];
    }

// TODO: create function for each task
// NOTE: instead of doing a function for each task, we defined a mode variable to change between different modes in the server function
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nao_tutorial_control_1");
    cout << "node on \n";
    Nao_control ic;
    // vector<double> pose = {0.2f, 0.2f, 0.2f, 0.2f};
    std::vector<double> pose;
    pose.reserve(4);
    pose.push_back(0.2);
    pose.push_back(0.2);
    pose.push_back(0.2);
    pose.push_back(0.2);
    // bool set_angle = false;
    ic.callSrv(pose);
    ros::spin();
    return 0;

}
