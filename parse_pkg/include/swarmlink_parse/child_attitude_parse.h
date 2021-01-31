//author : kaidi wang 
//date : 2020.12.23
//parse the swarmlink attitude 

#include <ros/ros.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Quaternion.h>
//#include <swarmlink_parse/child_arming_parse.h>


//example   #3001090:0:0:0:0!
class child_attitude_parse
{
private:
    /* data */
public:
    std_msgs::Float64 thrust;
    geometry_msgs::Quaternion q;

    //get float from string
    float get_float_from_string(const std::string & x)
    {
        char *p = const_cast<char*>(x.c_str());
        float *_return = 0;
        _return = reinterpret_cast<float*>(p);
        return *_return;
    }

    //parse the data section
    void parse(std::string data_section)
    {
        ROS_INFO_STREAM("data_section : "<<data_section);
        //define ':' flag
        char flag = ':'; 
        int position=0;
        int pos_num[4];
        int pos_start = 0;
        int i = 0;//number of count
        //four ':' character

        //static const size_t npos = -1;
        //It is better to compare to npos instead of -1 because the code is more legible.

        //this is a wonderful function
        while ((position = data_section.find(flag,pos_start))!=std::string::npos)
        {
            //ROS_INFO_STREAM("positon : "<<position);
            pos_start = position+1;
            pos_num[i]=position;
            ROS_INFO_STREAM("pos_num : "<<i<<" "<<pos_num[i]);
            i++;
        }
        q.w=get_float_from_string(data_section.substr(0,pos_num[0]));
        q.x=get_float_from_string(data_section.substr(pos_num[0]+1,pos_num[1]-pos_num[0]-1));
        q.y=get_float_from_string(data_section.substr(pos_num[1]+1,pos_num[2]-pos_num[1]-1));
        q.z=get_float_from_string(data_section.substr(pos_num[2]+1,pos_num[3]-pos_num[2]-1));
        thrust.data=get_float_from_string(data_section.substr(pos_num[3]+1,data_section.length()-pos_num[3]-1));

        //original data parse...
        //q.w=atof(data_section.substr(0,pos_num[0]).c_str());
        ROS_INFO_STREAM("q.w: "<<q.w);
        //q.x=atof(data_section.substr(pos_num[0]+1,pos_num[1]-pos_num[0]-1).c_str());
        ROS_INFO_STREAM("q.x: "<<q.x);
        //q.y=atof(data_section.substr(pos_num[1]+1,pos_num[2]-pos_num[1]-1).c_str());
        ROS_INFO_STREAM("q.y: "<<q.y);
        //q.z=atof(data_section.substr(pos_num[2]+1,pos_num[3]-pos_num[2]-1).c_str());
        ROS_INFO_STREAM("q.z: "<<q.z);
        //thrust.data=atof(data_section.substr(pos_num[3]+1,data_section.length()-pos_num[3]-1).c_str());
        ROS_INFO_STREAM("thrust.data: "<<thrust.data);
    }

    void parse_new(std::string data_section)
    {
        int i = 0;//number of count
	std::string qw ;
	std::string qx;
	std::string qy;
	std::string qz;
	std::string thu;

	qw = qw + data_section[0]+data_section[1]+data_section[2]+data_section[3];	
        qx = qx + data_section[4]+data_section[5]+data_section[6]+data_section[7];
	qy = qy + data_section[8]+data_section[9]+data_section[10]+data_section[11];
	qz = qz + data_section[12]+data_section[13]+data_section[14]+data_section[15];
	thu = thu + data_section[16]+data_section[17]+data_section[18]+data_section[19];
	q.w=get_float_from_string(qw);
        q.x=get_float_from_string(qx);
        q.y=get_float_from_string(qy);
	q.z=get_float_from_string(qz);
        thrust.data=get_float_from_string(thu);

        //original data parse...
        //q.w=atof(data_section.substr(0,pos_num[0]).c_str());
        //ROS_INFO_STREAM("q.w: "<<q.w);
        //q.x=atof(data_section.substr(pos_num[0]+1,pos_num[1]-pos_num[0]-1).c_str());
        //ROS_INFO_STREAM("q.x: "<<q.x);
        //q.y=atof(data_section.substr(pos_num[1]+1,pos_num[2]-pos_num[1]-1).c_str());
        //ROS_INFO_STREAM("q.y: "<<q.y);
        //q.z=atof(data_section.substr(pos_num[2]+1,pos_num[3]-pos_num[2]-1).c_str());
        //ROS_INFO_STREAM("q.z: "<<q.z);
        //thrust.data=atof(data_section.substr(pos_num[3]+1,data_section.length()-pos_num[3]-1).c_str());
        //ROS_INFO_STREAM("thrust.data: "<<thrust.data);
    }
    child_attitude_parse(/* args */);
    ~child_attitude_parse();
};

child_attitude_parse::child_attitude_parse(/* args */)
{
    // thrust.data = 0.0;
    // q.w=0.0;
    // q.x=0.0;
    // q.y=0.0;
    // q.z=0.0;
}

child_attitude_parse::~child_attitude_parse()
{
}

