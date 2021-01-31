#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Char.h>
#include <string.h>

#define STA '#'
#define END '!'
#define COLON ':'
#define MY_ID '3'

//define a basic class, to store basic info of every swarmlink msg
class msg_basic_info
{
private:
    /* data */
public:
    //define the head of every msg
    char target_id;
    char source_id;
    int msg_id;
    int payload_length;
    std::string data_section;
	std::string redundancy_string;


    //hex to number trans
    int trans_to_num(char c)
    {
        switch (c)
        {
        case '0': return 0;
        case '1': return 1;
        case '2': return 2;
        case '3': return 3;
        case '4': return 4;
        case '5': return 5;
        case '6': return 6;
        case '7': return 7;
        case '8': return 8;
        case '9': return 9;
        case 'A': return 10;
        case 'B': return 11;
        case 'C': return 12;
        case 'D': return 13;
        case 'E': return 14;
        case 'F': return 15;
            /* code */        
        }
    }

    //hex number to dec num
    int hex_to_dec(std::string hex)
    {
        int dec=0;
        int n=0;
        for (int i = hex.length()-1; i >=0; i--)
        {
            /* code */
            dec=dec+trans_to_num(hex[i])*pow(16,n);
            n++;
        }
        return dec;
    }


    //get basic info
    msg_basic_info get_basic_info(std_msgs::String msg)
    {
        msg_basic_info info;
        
        //std_msgs::String msg_str = ""+msg.data[3];
        if(msg.data[0]==STA)
        {
            //swarmlink format start char
            // if (msg.data[1])
            // {
            //     /* code */
            // }
            
            info.target_id=msg.data[1];
            info.source_id=msg.data[2];
            std::string msg_id_str="";
            //+msg.data[3]+msg.data[4]
            msg_id_str=msg_id_str+msg.data[3]+msg.data[4];
            //ROS_INFO_STREAM("msg_id_str:"<<msg_id_str);

            std::string msg_len_str="";
            msg_len_str=msg_len_str+msg.data[5]+msg.data[6];
            //ROS_INFO_STREAM("msg_len_str:"<<msg_len_str);

            info.msg_id=hex_to_dec(msg_id_str);
            info.payload_length=hex_to_dec(msg_len_str);

            int len=msg.data.length()-1;
            //ROS_INFO_STREAM("msg.data[msg.data.length()-1]:"<<msg.data[len]);
            if (msg.data[len] == END)
            {
                //get data section, string
		for(int i=7;i<=(7+info.payload_length);i++)
		{
			info.data_section=info.data_section+msg.data[7];
		}
                //info.data_section=msg.data.substr(7,info.payload_length);
		//for(int i=0;i<info.payload_length;i++)
		{
			
		}
            }
            else
            {
                ROS_ERROR_STREAM("error at the end of swarmlink...");
            }
            
            //
        }
        else
        {
            //
            ROS_ERROR_STREAM("this is not a standard swarmlink format...");
        }
        return info;
    }


    msg_basic_info parse_get_basic_info(std::string msg)
    {
		msg_basic_info info;
		info.redundancy_string = "";

		//define a array to memory STA place
		int msg_length = msg.length();
		ROS_INFO_STREAM("msg_length: "<<msg_length);
		if(msg_length>=1000)
		{
			msg = "";
			msg_length = 0;
		}
		//find start char of the can bus string
		int a[2] = {0,0};
		int j=0;//index of the array a
		for(int i=0;i<msg_length;i++)
		{
			if(msg[i]==0x41 && msg[i+1]==0x54)
			{
				ROS_INFO_STREAM("number of start: "<<i);
				a[j]=i;
				j++;
			}
		}

		if(a[1]!=0)
		{
			for(int i=a[1] ; i<msg_length; i++)
			{
				info.redundancy_string = info.redundancy_string+ msg[i];
			}
			ROS_INFO_STREAM("redundancy_string_kd: "<<info.redundancy_string);
		}
		//now it is time to parse the string and combination a new string 
		//if(a[0]==0)
		//{	
		//}

	//this line need to be changed 
        if(msg[a[0]]==0x41 && msg[a[0]+1]==0x54)
        {
            //ROS_INFO_STREAM("good");
            info.target_id=msg[a[0]+2];
            info.source_id=msg[a[0]+3];

            std::string msg_id_str="";
            msg_id_str=msg_id_str+msg[a[0]+4]+msg[a[0]+5];

            std::string msg_len_str="";
            msg_len_str=msg_len_str+msg[a[0]+6]+msg[a[0]+7];

            info.msg_id=hex_to_dec(msg_id_str);
            info.payload_length=hex_to_dec(msg_len_str);

            int len=8+info.payload_length+1;
            int num_crc=0;
            int right=0 ;
            for (int i = 0; i < len; i++)
            {
                num_crc = num_crc+msg[i];
            }
            //ROS_INFO_STREAM("num_crc: "<<num_crc);
            int get_crc=(num_crc & 255);
            //ROS_INFO_STREAM("get_crc: "<<get_crc);

            if(get_crc==0)
            {
                right=1;
                //ROS_INFO_STREAM("string right: ++++++++++++++++++++++++++++++");
            }
            
            if (msg[len] == END && right == 1)
            {
                //get data section, string
		for(int i=8;i<=(8+info.payload_length);i++)
		{
			info.data_section=info.data_section+msg.data[a[0]+i];
		}
                //info.data_section=msg.substr(8,info.payload_length);
		
		right = 0;
            }
            else
            {
                ROS_ERROR_STREAM("error occur ...");
            }
        }
        else
        {
            ROS_INFO_STREAM("this is not a standard swarmlink format...");
        }
        
        return info;
    }
    msg_basic_info(/* args */);
    ~msg_basic_info();
};

msg_basic_info::msg_basic_info(/* args */)
{
    target_id=0;
    source_id=0;
    msg_id=0;
    payload_length=0;
    data_section="";
    redundancy_string="";
}

msg_basic_info::~msg_basic_info()
{
}


//
class child_arming_parse 
{
private:
    /* data */
public:
    std_msgs::Bool is_armed;
    std_msgs::Char is_state;
    child_arming_parse(/* args */);
    void parse(std::string data_section)
    {
        if (data_section[0]=='1')
        {
            is_armed.data=true;
            is_state.data = '1';//arm
            ROS_INFO_STREAM("that right .....");
        }
        else if (data_section[0]=='0')
        {
            is_armed.data=false;
            is_state.data = '0';//takedown
        }
        else if (data_section[0]=='2')
        {
            /* code */
            is_state.data = '2';
        }
        
        
    }
    ~child_arming_parse();
};

child_arming_parse::child_arming_parse(/* args */)
{
    is_armed.data=false;
}

child_arming_parse::~child_arming_parse()
{
}
