#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

geometry_msgs::PoseStamped load_position_raw, multirotor_position_local, multirotor_position_raw;
geometry_msgs::TwistStamped load_velocity, load_velocity_raw, multirotor_velocity, multirotor_velocity_raw, multirotor_velocity_local;
bool load_positionRawSubFlag = false;
bool load_velocitySubFlag = false;
bool load_velocityRawSubFlag = false;
bool multirotor_positionLocalSubFlag = false;
bool multirotor_positionRawSubFlag = false;
bool multirotor_velocitySubFlag = false;
bool multirotor_velocityRawSubFlag = false;
bool multirotor_velocityLocalSubFlag = false;
ros::Subscriber load_positionRaw_sub, load_velocity_sub, multirotor_position_sub, multirotor_positionRaw_sub, multirotor_velocity_sub, multirotor_velocityLocal_sub, load_velocity_raw_sub, multirotor_velocity_raw_sub;
std::ofstream file_;

void multirotor_positionLocal_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void multirotor_positionRaw_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void multirotor_velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
void multirotor_velocityRaw_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
void multirotor_velocityLocal_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);

void load_positionRaw_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void load_velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
void load_velocityRaw_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
void addLine();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity_proof");
    ros::NodeHandle nh; 

    multirotor_positionRaw_sub = nh.subscribe<geometry_msgs::PoseStamped>
                                    ("mavros/mocap/pose", 10, multirotor_positionRaw_cb);
    multirotor_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
                                    ("mavros/local_position/pose", 10, multirotor_positionLocal_cb);
    multirotor_velocityLocal_sub = nh.subscribe<geometry_msgs::TwistStamped>
                                    ("mavros/local_position/velocity", 10, multirotor_velocityLocal_cb);
    multirotor_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>
                                    ("mpc/multirotor_velocity", 10, multirotor_velocity_cb);
	multirotor_velocity_raw_sub = nh.subscribe<geometry_msgs::TwistStamped>
									("mpc/multirotor_velocityRaw", 10, multirotor_velocityRaw_cb);
    load_positionRaw_sub = nh.subscribe<geometry_msgs::PoseStamped>
                                    ("syd/load_position", 10, load_positionRaw_cb);
    load_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>
                                    ("mpc/load_velocity", 10, load_velocity_cb);
	load_velocity_raw_sub = nh.subscribe<geometry_msgs::TwistStamped>
									("mpc/load_velocityRaw", 10, load_velocityRaw_cb);

	std::string fileName = "/home/youngdong/Desktop/velocity_proof.txt";
	file_.open(fileName.c_str());

    ros::Rate rate(100);
    while(ros::ok())
    {   
        ros::spinOnce();
        if(load_positionRawSubFlag       && load_velocitySubFlag     &&
           multirotor_velocitySubFlag && multirotor_positionRawSubFlag && multirotor_positionLocalSubFlag && multirotor_positionRawSubFlag &&
		   load_velocityRawSubFlag && multirotor_velocityRawSubFlag)
        {
			std::cout << "Logging..." << std::endl;
			addLine();
			load_positionRawSubFlag = false;
			load_velocitySubFlag = false;
			load_velocityRawSubFlag = false;
			multirotor_positionLocalSubFlag = false;
			multirotor_positionRawSubFlag = false;
			multirotor_velocitySubFlag = false;
			multirotor_velocityRawSubFlag = false;
			multirotor_velocityLocalSubFlag = false;
        }
		//std::cout << "Not Logging..." << std::endl;
		//std::cout << multirotor_positionLocalSubFlag << " " << multirotor_positionRawSubFlag << " " << multirotor_velocitySubFlag << " " << multirotor_velocityRawSubFlag << " " << multirotor_velocityLocalSubFlag << std::endl;
		//std::cout << load_positionRawSubFlag << " " << load_velocitySubFlag << " " << load_velocityRawSubFlag << std::endl;
        rate.sleep();
    }
	file_.close();
    return 0;
}

void load_positionRaw_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    load_position_raw = *msg;
    load_positionRawSubFlag = true;
}

void load_velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    load_velocity = *msg;
    load_velocitySubFlag = true;
}

void load_velocityRaw_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    load_velocity_raw = *msg;
    load_velocityRawSubFlag = true;
}

void multirotor_positionLocal_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    multirotor_position_local = *msg;
    multirotor_positionLocalSubFlag = true;
}

void multirotor_positionRaw_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    multirotor_position_raw = *msg;
    multirotor_positionRawSubFlag = true;
}

void multirotor_velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    multirotor_velocity = *msg;
    multirotor_velocitySubFlag = true;
}

void multirotor_velocityRaw_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    multirotor_velocity_raw = *msg;
    multirotor_velocityRawSubFlag = true;
}

void multirotor_velocityLocal_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    multirotor_velocity_local = *msg;
    multirotor_velocityLocalSubFlag = true;
}

void addLine()
{
	file_ << "1" << " " << multirotor_position_raw.pose.position.x
                 << " " << multirotor_position_raw.pose.position.y
                 << " " << multirotor_position_raw.pose.position.z << "\n";
	file_ << "2" << " " << multirotor_position_local.pose.position.x
                 << " " << multirotor_position_local.pose.position.y
                 << " " << multirotor_position_local.pose.position.z << "\n";


	file_ << "3" << " " << multirotor_velocity.twist.linear.x
                 << " " << multirotor_velocity.twist.linear.y
                 << " " << multirotor_velocity.twist.linear.z << "\n";
	file_ << "4" << " " << multirotor_velocity_raw.twist.linear.x
                 << " " << multirotor_velocity_raw.twist.linear.y
                 << " " << multirotor_velocity_raw.twist.linear.z << "\n";
	file_ << "5" << " " << multirotor_velocity_local.twist.linear.x
                 << " " << multirotor_velocity_local.twist.linear.y
                 << " " << multirotor_velocity_local.twist.linear.z << "\n";

	file_ << "6" << " " << load_position_raw.pose.position.x
                 << " " << load_position_raw.pose.position.y
                 << " " << load_position_raw.pose.position.z << "\n";
	file_ << "7" << " " << load_velocity.twist.linear.x
                 << " " << load_velocity.twist.linear.y
                 << " " << load_velocity.twist.linear.z << "\n";
	file_ << "8" << " " << load_velocity_raw.twist.linear.x
                 << " " << load_velocity_raw.twist.linear.y
                 << " " << load_velocity_raw.twist.linear.z << "\n";
}
