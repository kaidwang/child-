/*author:kaidi wang*/
/*date: 2020.12.17*/
/* define some functions which need to be used in decision_collection_node.cpp*/
#include <ros/ros.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>

#include <fstream>
// #include <iostream>

//attitude command make from master to child vehicle
mavros_msgs::AttitudeTarget attitude_command_from_master_to_child(geometry_msgs::Quaternion q,std_msgs::Float64 thrust)
{
    mavros_msgs::AttitudeTarget att_tar;
    att_tar.thrust=thrust.data;
    att_tar.orientation.w=q.w;
    att_tar.orientation.x=q.x;
    att_tar.orientation.y=q.y;
    att_tar.orientation.z=q.z;
    return att_tar;
}

//function of output log txt file 
//input: setpoint pos , real pos, 
//output: a txt file that record input information.
void decision_node_log_txt(mavros_msgs::AttitudeTarget att_tar, std::ofstream &out)
{
    //std::ofstream out;
    //write a timestamp
    out<<"Time is: "<<ros::Time::now();
    out<<"setpoint, thrust:"<<att_tar.thrust<<std::endl;
    out<<"setpoint, orientation w:"<<att_tar.orientation.w<<"  x:"<<att_tar.orientation.x<<"  y:"<<att_tar.orientation.y<<"  z:"<<att_tar.orientation.z<<std::endl;
    // //write a enter line as a 
    out<<std::endl;
    //return out;
}