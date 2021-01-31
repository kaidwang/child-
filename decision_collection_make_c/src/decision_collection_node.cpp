//author : kaidi wang
//date : 2020.12.1
//child vehicle decision and collection main node file 

#include <ros/ros.h>
// #include <iostream>
// #include <fstream>//output file

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/BatteryState.h>//battery state include file

#include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Quaternion.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Char.h>

#include <decision_collection_make_c/param_list.h>
#include <decision_collection_make_c/decision_collection_node.h>
#include <tf/transform_datatypes.h>
//global param list
#define pi 3.14159
//callback function list
//cmd state
std_msgs::Char state_cmd;
void state_cmd_sub_cb(const std_msgs::Char::ConstPtr& msg)
{
	state_cmd = *msg;
}


//current state 
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	current_state = *msg;
}

//local pos callback function
geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	local_pos = *msg;
}

//arm_sub_cb
//define a message of whether arm the px4
std_msgs::Bool is_armed;
void arm_sub_cb(const std_msgs::Bool::ConstPtr& msg)
{
	is_armed = *msg;
}

//attitude_quat_sub_cb
geometry_msgs::Quaternion att_q;
geometry_msgs::Point aim_euler;
void attitude_quat_sub_cb(const geometry_msgs::Quaternion::ConstPtr& msg)
{
	att_q = *msg;
	double roll,pitch, yaw;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(att_q,quat);
	tf::Matrix3x3(quat).getEulerZYX(yaw,pitch,roll,1);
	ROS_INFO_STREAM("roll: "<< pitch);
	ROS_INFO_STREAM("pitch: "<< roll);
	ROS_INFO_STREAM("yaw: "<<-yaw+pi/2);

	aim_euler.x = roll;
	aim_euler.y = pitch;
	aim_euler.z = -yaw+pi/2;
}

//attitude_thrust_sub_cb
std_msgs::Float64 att_thrust;
float THRUST_FULL = 67.7;
void attitude_thrust_sub_cb(const std_msgs::Float64::ConstPtr& msg)
{
	att_thrust = *msg;
	//att_thrust.data = att_thrust.data / 1000;
	att_thrust.data = att_thrust.data / THRUST_FULL;
}


//battery state callback
sensor_msgs::BatteryState battery_state;
void battery_state_sub_cb(const sensor_msgs::BatteryState::ConstPtr& msg)
{
	battery_state = *msg;
}

//emergencyn callback function
std_msgs::Bool emg; 
void emg_sub_cb(const std_msgs::Bool::ConstPtr& msg)
{
	emg = *msg;
}


int if_the_PX4_armed = 0 ;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "decision_collection_node");
	ros::NodeHandle nh;

	//subscribe list, from the px4 message
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
		("mavros/local_position/pose", 10, local_pos_cb);
	ros::Subscriber state_sub     = nh.subscribe<mavros_msgs::State>
		("mavros/state", 10, state_cb);

	//arm or not, this signal from the master node 
	ros::Subscriber arm_sub = nh.subscribe<std_msgs::Bool>
		("Arm",10,arm_sub_cb);
	ros::Subscriber state_cmd_sub = nh.subscribe<std_msgs::Char>
		("state",10,state_cmd_sub_cb);

	//attitude quaternion from the master node 
	ros::Subscriber attitude_quat_sub = nh.subscribe<geometry_msgs::Quaternion>("Attitude_q",100,attitude_quat_sub_cb);
	//attitude thrust from the master node 
	ros::Subscriber attitude_thrust_sub = nh.subscribe<std_msgs::Float64>("Attitude_thrust",10,attitude_thrust_sub_cb);

	//define the subsrcibe topic, monitor child px4,first of battery
	ros::Subscriber battery_state_sub = nh.subscribe<sensor_msgs::BatteryState>("battery",10,battery_state_sub_cb);

	//emergency sub
	ros::Subscriber emg_sub = nh.subscribe<std_msgs::Bool>("emergency",10,emg_sub_cb);

	//publish list,command publish to mavros which connected with PX4 by usb cable
	//position control mode, setpoint which need GPS/Vicon/SLAM
	ros::Publisher local_pos_pub  = nh.advertise<geometry_msgs::PoseStamped>
		("mavros/setpoint_position/local", 10);
	//attitude control mode, setpoint which don't need GPS/Vicon/SLAM
	ros::Publisher raw_pub_att = nh.advertise<mavros_msgs::AttitudeTarget>
		("mavros/setpoint_raw/attitude",100);


	//advertise a bool param, named child/arming...
	ros::Publisher arm_child_flight    = nh.advertise<std_msgs::Bool>("child/arming",10);
	
	ros::Publisher record_euler_pub = nh.advertise<geometry_msgs::Point>("/record",10);
	
	//sercice list
	ros::ServiceClient arming_client   = nh.serviceClient<mavros_msgs::CommandBool>
		("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
		("mavros/set_mode");

	//var define list 
	mavros_msgs::AttitudeTarget att_target;
	mavros_msgs::AttitudeTarget raw_att;

	//wait the px4 flight to be connected 
	//if the current_state.connected=1, main px4 is connected.
	ros::Rate rate(100.0);
	//ROS_INFO("current_state_connected is %d",current_state.connected);

	while (ros::ok() && !current_state.connected) {
		ROS_INFO_STREAM("px4 is unconnected...,and the node id is:"<<node_id);
		//this place need to add a back message to the master node
		ros::spinOnce();
		rate.sleep();
	}
	

	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";

	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value=true;

	ros::Time last_request = ros::Time::now();

	std_msgs::Bool arm_bool;
	arm_bool.data =false;

	//publish some pose
	for(int i = 0;i<1000 && ros::ok();i++)
	{
		ROS_INFO("publish some pose");
		raw_pub_att.publish(raw_att);
		ros::spinOnce();
		rate.sleep();
	}
	//detect the RC command.
	//set offboard mode and then arm
	while(ros::ok())
	{
		if(aim_euler.x!=0 || aim_euler.y!=0 || aim_euler.z!=0)
		{
			record_euler_pub.publish(aim_euler);
		}
		//if (!emg.data)
		{
//			ROS_INFO("system running good...");
			if(state_cmd.data == '1'/*is_armed.data==true*/)
			{
				//ROS_INFO(" armed function........");
				if( current_state.mode != "OFFBOARD" &&(ros::Time::now() - last_request > ros::Duration(5.0)))
				{
					if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
					{
						ROS_INFO("Offboard enabled");
					}
					last_request = ros::Time::now();
				} 
				else
				{
					if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
					{
						if( arming_client.call(arm_cmd) &&arm_cmd.response.success)
						{
							ROS_INFO("Vehicle armed");
							if_the_PX4_armed++;
						}
						last_request = ros::Time::now();
					}
				}
				//publish attitude and thrust of child px4
				att_target = attitude_command_from_master_to_child(att_q,att_thrust);
				raw_pub_att.publish(att_target);


			}
			else if(state_cmd.data=='2')
			{
				break;
			}
			else if (state_cmd.data=='0')
			{
				att_target = attitude_command_from_master_to_child(att_q,att_thrust);
				raw_pub_att.publish(att_target);
			}
		}

		//safaty brocken , emg.data == true && PX4 is armed 
		if(emg.data)
		{
			if(if_the_PX4_armed > 0)
			{
				ROS_INFO("system CAN bus is unconnected ...");
				break;
			}
		}
	ros::spinOnce();
	rate.sleep();
		
	}

	return 0;
}

