/**
 * date: 2021.1.9
 * author: kaidi wang
 * mail:   1055080765@qq.com
 * describe: swarmlink parse 
**/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Quaternion.h>
#include <swarmlink_parse/child_arming_parse.h>
#include <swarmlink_parse/child_attitude_parse.h>
#include <serial/serial.h>

#define child1 '1'
#define child2 '2'
#define child3 '3'
#define master_id 'A'

//define some buffer number of serial port
#define sBUFFERSIZE 1000 // send buffer size串口发送缓存长度  
#define rBUFFERSIZE 32 // receive buffer size 串口接收缓存长度  
unsigned char s_buffer[sBUFFERSIZE]; // send buffer  
unsigned char r_buffer[rBUFFERSIZE];

//define a serial port
serial::Serial ser1;
std_msgs::String result;//define a string of parse data


std_msgs::Float64 thrust;
geometry_msgs::Quaternion q;
//define a message basic parse class 
msg_basic_info msg_info;
msg_basic_info msg_info_last;

char which_child = 'N';
std_msgs::String test_str;// I need to known when the CAN bus is broken. 
std::string data_yaw;

//define list of swarmlink structure
child_arming_parse arm_parse;
child_attitude_parse attitude_parse;

// void rece_sub_callback(const std_msgs::String::ConstPtr& msg)
// {
// 	std_msgs::String rece_str;
// 	rece_str = *msg;
// 	test_str = *msg;
// 	msg_info = msg_info.parse_get_basic_info(rece_str.data);
// 	// ROS_INFO_STREAM("target_id:"<<msg_info.target_id);
// 	// ROS_INFO_STREAM("source_id:"<<msg_info.source_id);
// 	ROS_INFO_STREAM("receive msg_id:"<<msg_info.msg_id);
// 	// ROS_INFO_STREAM("payload_length:"<<msg_info.payload_length);
// 	// ROS_INFO_STREAM("data_section:"<<msg_info.data_section);
// 	if(/* judge target id */msg_info.target_id=='A'||msg_info.target_id==MY_ID)
// 	{
// 		switch (msg_info.msg_id)
// 		{
// 		case 0:// msg id is 0
// 			/*parse arm */
// 			//ROS_INFO_STREAM("start to parse arm command");
// 			arm_parse.parse(msg_info.data_section);
// 			//ROS_INFO_STREAM("is arm or not:"<<arm_parse.is_armed);
// 			//ROS_INFO_STREAM("is state:"<<arm_parse.is_state);				
// 			//arm_pub.publish(arm_parse.is_armed);
// 			// state_pub.publish(arm_parse.is_state);
// 			break;
// 		case 1:
// 			/*parse attitude command*/
// 			//ROS_INFO_STREAM("start to parse attitude command");
// 			//this line is not good 
// 			attitude_parse.parse_new(msg_info.data_section);
// 			//
// 			q = attitude_parse.q;
// 			thrust = attitude_parse.thrust;

// 			// attitude_quat_pub.publish(q);
// 			// attitude_thrust_pub.publish(thrust);
// 			break;
// 		default:
// 			break;
// 		}
// 		/* code */
// 	}
// 	else
// 	{
// 		ROS_ERROR_STREAM("this command is not send to me");
// 	}
// 	if(rece_str.data[0] =='@' && rece_str.data[1]==master_id )
// 	{
// 		which_child = rece_str.data[2];
// 		data_yaw= rece_str.data.substr(3,4);
// 	}

// }

//define a function to parse the received data string
void parse_data(std_msgs::String msg)
{
	std_msgs::String rece_str;
	rece_str = msg;

	msg_info = msg_info.get_basic_info(rece_str);
	// ROS_INFO_STREAM("target_id:"<<msg_info.target_id);
	// ROS_INFO_STREAM("source_id:"<<msg_info.source_id);

	// ROS_INFO_STREAM("payload_length:"<<msg_info.payload_length);
	// ROS_INFO_STREAM("data_section:"<<msg_info.data_section);
	if(/* judge target id */msg_info.target_id=='A'||msg_info.target_id==MY_ID)
	{
		switch (msg_info.msg_id)
		{
		case 0:// msg id is 0
			/*parse arm */
			//ROS_INFO_STREAM("start to parse arm command");
			arm_parse.parse(msg_info.data_section);
			//ROS_INFO_STREAM("is arm or not:"<<arm_parse.is_armed);
			//ROS_INFO_STREAM("is state:"<<arm_parse.is_state);				
			//arm_pub.publish(arm_parse.is_armed);
			// state_pub.publish(arm_parse.is_state);
			break;
		case 1:
			/*parse attitude command*/
			//ROS_INFO_STREAM("start to parse attitude command");
			//this line is not good 
			attitude_parse.parse(msg_info.data_section);
			//
			q = attitude_parse.q;
			thrust = attitude_parse.thrust;

			// attitude_quat_pub.publish(q);
			// attitude_thrust_pub.publish(thrust);
			break;
		default:
			break;
		}
		/* code */
	}
	else
	{
		ROS_ERROR_STREAM("this command is not send to me");
	}
	if(rece_str.data[0] =='@' && rece_str.data[1]==master_id )
	{
		which_child = rece_str.data[2];
		data_yaw= rece_str.data.substr(3,4);
	}

}

// pass data through the message tube
// the new para function 
// define a function to parse the received data string
std::string redundancy_string_last;
void parse_data_fun(std_msgs::String msg)
{
 	
	std::string rece_str="";
	//redundancy_string_last = msg_info.redundancy_string;

	rece_str = msg.data;
	//ROS_INFO_STREAM("msg.data: "<<msg.data);
	//ROS_INFO_STREAM("redundancy_string_last: "<<redundancy_string_last);
	//ROS_INFO_STREAM("rece_str: "<<rece_str);


	msg_info = msg_info.parse_get_basic_info(rece_str);
	//msg_info_last = msg_info;

	redundancy_string_last = msg_info.redundancy_string;



	if(msg_info.target_id=='A'||msg_info.target_id==MY_ID)
	{
		switch (msg_info.msg_id)
		{
		case 0:// msg id is 0
			/*parse arm */
			//ROS_INFO_STREAM("start to parse arm command");
			arm_parse.parse(msg_info.data_section);
			//ROS_INFO_STREAM("is arm or not:"<<arm_parse.is_armed);
			//ROS_INFO_STREAM("is state:"<<arm_parse.is_state);				
			//arm_pub.publish(arm_parse.is_armed);
			// state_pub.publish(arm_parse.is_state);
			break;
		case 1:
			/*parse attitude command*/
			//ROS_INFO_STREAM("start to parse attitude command");
			//this line is not good 
			attitude_parse.parse_new(msg_info.data_section);
			//
			q = attitude_parse.q;
			thrust = attitude_parse.thrust;

			// attitude_quat_pub.publish(q);
			// attitude_thrust_pub.publish(thrust);
			break;
		default:
			break;
		}
		/* code */
	}
	else
	{
		ROS_ERROR_STREAM("this command is not send to me");
	}

}


float get_float_from_string_yaw(const std::string & x)
{
	char *p = const_cast<char*>(x.c_str());
	float *_return = 0;
	_return = reinterpret_cast<float*>(p);
	return *_return;
}

//receive callback
static int p;
void serial_receive_callback(const ros::TimerEvent&,std_msgs::Bool emg)
{
	if(ser1.available())
	{
		result.data = ser1.read(ser1.available());
		//ROS_INFO_STREAM("Read: " << result.data);
		ROS_INFO_STREAM("result length: "<<result.data);
		//parse the data from CAN bus
		parse_data_fun(result);
		//as a debug line
		//write_pub.publish(result);
		//read_pub.publish(result);
		// last_request = ros::Time::now();
		result.data = "";
	}
	else if(!ser1.available())
	{
		ROS_INFO_STREAM("CAN bus is not connected....");
		emg.data = true;
	}
}

//main funciton 
int main(int argc,char** argv)
{
	ros::init(argc,argv,"swarmlink_parse_node");
	ros::NodeHandle nh;
	//ros::Subscriber rece_sub  = nh.subscribe<std_msgs::String>("read",1000,rece_sub_callback);
	//ros::Subscriber test_sub = nh.subscribe<std_msgs::String>("");
	//publish to child vehicle 
	ros::Publisher arm_pub = nh.advertise<std_msgs::Bool>("Arm",10);
	ros::Publisher state_pub = nh.advertise<std_msgs::Char>("state",10);

	ros::Publisher attitude_quat_pub = nh.advertise<geometry_msgs::Quaternion>("Attitude_q",100);
	ros::Publisher attitude_thrust_pub = nh.advertise<std_msgs::Float64>("Attitude_thrust",10);

	ros::Publisher node_yaw_1_pub = nh.advertise<std_msgs::Float64>
		("node1_yaw",10);
	ros::Publisher node_yaw_2_pub = nh.advertise<std_msgs::Float64>
		("node2_yaw",10);
	ros::Publisher node_yaw_3_pub = nh.advertise<std_msgs::Float64>
		("node3_yaw",10);

	//emergency switch msg
	ros::Publisher emergenct_pub = nh.advertise<std_msgs::Bool>("emergency",100);

	// //define list of swarmlink structure
	// child_arming_parse arm_parse;
	// child_attitude_parse attitude_parse;
	try{
        //serial init
        ser1.setPort("/dev/ttyUSB0");
        ser1.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser1.setTimeout(to);
        ser1.open();
        if(ser1.isOpen())
        {
            ROS_INFO_STREAM("the serial port is opened succeed...");
        }
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port");
        return -1;
    }

    ros::Time last_request;
	std_msgs::Bool emg;
	ros::Timer timer = nh.createTimer(ros::Duration(0.005),boost::bind(&serial_receive_callback,_1,emg));

	//circle frequence
	ros::Rate rate(300);
	while (ros::ok())
	{
		emg.data = false;
		
		/*  */
		// if(/* judge target id */msg_info.target_id=='A'||msg_info.target_id==MY_ID)
		// {
		// 	switch (msg_info.msg_id)
		// 	{
		// 	case 0:// msg id is 0
		// 		/*parse arm */
		// 		//ROS_INFO_STREAM("start to parse arm command");
		// 		arm_parse.parse(msg_info.data_section);
		// 		//ROS_INFO_STREAM("is arm or not:"<<arm_parse.is_armed);
		// 		//ROS_INFO_STREAM("is state:"<<arm_parse.is_state);				
		// 		//arm_pub.publish(arm_parse.is_armed);
		state_pub.publish(arm_parse.is_state);
		// 		break;
		// 	case 1:
		// 		/*parse attitude command*/
		// 		//ROS_INFO_STREAM("start to parse attitude command");
		// 		//this line is not good 
		// 		attitude_parse.parse(msg_info.data_section);
		// 		//
		// 		q = attitude_parse.q;
		// 		thrust = attitude_parse.thrust;

		attitude_quat_pub.publish(q);
		attitude_thrust_pub.publish(thrust);
		// 		break;
		// 	default:
		// 		break;
		// 	}
		// 	/* code */
		// }
		// else
		// {
		// 	ROS_ERROR_STREAM("this command is not send to me");
		// }
		
		std_msgs::Float64 yaw;
		//process the msg which received from the child nodes
		if(which_child != 'N')
		{
			yaw.data = get_float_from_string_yaw(data_yaw);
			if(which_child == child1)
			{
				node_yaw_1_pub.publish(yaw);
			}
			if(which_child == child2)
			{
				node_yaw_2_pub.publish(yaw);

			}
			if(which_child == child3)
			{
				node_yaw_3_pub.publish(yaw);
			
			}
		}
		//ROS_ERROR_STREAM("string data is: "<<test_str.data);
		emergenct_pub.publish(emg);
		ros::spinOnce();
		rate.sleep();
	}
	
	return 0;
}

