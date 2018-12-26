#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <keyboard/Key.h>

#define KEY_M 109
#define KEY_STOP_MPC 44

bool updated_path = false;
bool updated_velocity = false;
int dataNumPath = 1;
int dataNumVelocity = 1;
bool stopFlag = true;

std::vector<mavros_msgs::PositionTarget> setpointMsgs(100);

void path_cb(const nav_msgs::Path::ConstPtr& msg)
{
	updated_path = true;
	int dataNumber = atoi(msg->header.frame_id.c_str());
	std::cout << "Path number : " << dataNumber << std::endl;
	dataNumPath = dataNumber;
	for(int i=0; i<dataNumber; i++)
	{
		setpointMsgs.at(i).coordinate_frame = setpointMsgs.at(i).FRAME_LOCAL_NED;
		setpointMsgs.at(i).position.x = msg->poses[i].pose.position.x;
		setpointMsgs.at(i).position.y = msg->poses[i].pose.position.y;
		setpointMsgs.at(i).position.z = msg->poses[i].pose.position.z;
	}
}

void velocity_cb(const nav_msgs::Path::ConstPtr& msg)
{
	updated_velocity = true;
	int dataNumber = atoi(msg->header.frame_id.c_str());
	std::cout << "Velocity number : " << dataNumber << std::endl;
	dataNumVelocity = dataNumber;
	for(int i=0; i<dataNumber; i++)
	{
		setpointMsgs.at(i).coordinate_frame = setpointMsgs.at(i).FRAME_LOCAL_NED;
		setpointMsgs.at(i).velocity.x = msg->poses[i].pose.position.x;
		setpointMsgs.at(i).velocity.y = msg->poses[i].pose.position.y;
		setpointMsgs.at(i).velocity.z = msg->poses[i].pose.position.z;
		std::cout << msg->poses[i].pose.position.z << std::endl;
	}
}

void key_cb(const keyboard::Key::ConstPtr& msg)
{
	if(msg->code == KEY_M) stopFlag = false;
	else if(msg->code == KEY_STOP_MPC) stopFlag = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "trajectory_publisher");
	ros::NodeHandle nh;

	ros::Subscriber path_sub = nh.subscribe<nav_msgs::Path>
		("syd/mpc_path2", 10, path_cb);
	ros::Subscriber velocity_sub = nh.subscribe<nav_msgs::Path>
		("syd/mpc_velocity", 10, velocity_cb);
	ros::Subscriber keyboard_sub = nh.subscribe<keyboard::Key>
		("keyboard/keydown", 10, key_cb);

	ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>
		("mavros/setpoint_raw/local",1);

	ros::Rate rate(1/0.04);

	int pathIndex = 0;
	int velocityIndex = 0;
	bool pathPublishFlag = false;
	bool velocityPublishFlag = false;

	while(ros::ok() && !stopFlag)
	{
		ros::spinOnce();
		//Setting
		if(updated_path && updated_velocity)
		{
			for(int i=0; i<dataNumPath; i++) //Assume that path and velocity have same length
			{
				setpointMsgs.at(i).type_mask  = setpointMsgs.at(i).IGNORE_AFX | setpointMsgs.at(i).IGNORE_AFY | setpointMsgs.at(i).IGNORE_AFZ;
			}
			pathIndex = 0;
			velocityIndex = 0;
			updated_path = false;
			updated_velocity = false;
		}
		else if(updated_path && !updated_velocity)
		{
			for(int i=0; i<dataNumPath; i++) //Assume that path and velocity have same length
			{
				setpointMsgs.at(i).type_mask  = setpointMsgs.at(i).IGNORE_AFX | setpointMsgs.at(i).IGNORE_AFY | setpointMsgs.at(i).IGNORE_AFZ;
				setpointMsgs.at(i).type_mask |= setpointMsgs.at(i).IGNORE_VX | setpointMsgs.at(i).IGNORE_VY | setpointMsgs.at(i).IGNORE_VZ;
			}
			pathIndex = 0;
			pathPublishFlag = true;
			updated_path = false;
		}
		else if(updated_velocity && !updated_path)
		{
			for(int i=0; i<dataNumVelocity; i++)
			{
				setpointMsgs.at(i).type_mask  = setpointMsgs.at(i).IGNORE_AFX | setpointMsgs.at(i).IGNORE_AFY | setpointMsgs.at(i).IGNORE_AFZ;
				setpointMsgs.at(i).type_mask |= setpointMsgs.at(i).IGNORE_PX | setpointMsgs.at(i).IGNORE_PY | setpointMsgs.at(i).IGNORE_PZ;
			}
			velocityIndex = 0;
			velocityPublishFlag = true;
			updated_velocity = false;
		}
	
		//Publishing
		if(pathPublishFlag)
		{
			setpoint_pub.publish(setpointMsgs.at(pathIndex++));
			if(pathIndex==dataNumPath) --pathIndex;
		}
		if(velocityPublishFlag)
		{
			setpoint_pub.publish(setpointMsgs.at(velocityIndex++));
			if(velocityIndex==dataNumVelocity) --velocityIndex;
		}
	rate.sleep();
	}
}
