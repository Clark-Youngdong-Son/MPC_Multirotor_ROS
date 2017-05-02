#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#define STATE_MOVE    false
#define OBSTACLE_MOVE false
#define WAYPOINT_MOVE false
#define FINAL_MOVE    false

#define R_STATE 1.5
#define R_OBSTACLE 0.5
#define R_WAYPOINT 0.1
#define R_FINAL    0.5
#define F_STATE 2.0
#define F_OBSTACLE 3.0
#define F_WAYPOINT 1.5
#define F_FINAL    1.3
#define PI 3.14
#define HZ 100.0

int main(int argc, char **argv)
{
	ros::init(argc, argv, "environment_maker");
	ros::NodeHandle nh;
	ros::Publisher state_pub, obstacle_pub, waypoint_pub, final_pub;

	geometry_msgs::PoseStamped state_msg;
	geometry_msgs::TransformStamped obstacle_msg, waypoint_msg, final_msg;

	state_pub    = nh.advertise<geometry_msgs::PoseStamped>("mavros/local_position/pose",10);
	obstacle_pub = nh.advertise<geometry_msgs::TransformStamped>("obstacle1//pose",10);
	waypoint_pub = nh.advertise<geometry_msgs::TransformStamped>("waypoint/pose",10);
	final_pub = nh.advertise<geometry_msgs::TransformStamped>("final/pose",10);

	state_msg.pose.position.x = 0.0;
	state_msg.pose.position.y = 0.0;
	state_msg.pose.position.z = 0.0;
	double state_center_x = state_msg.pose.position.x - R_STATE;
	double state_center_y = state_msg.pose.position.y;

	obstacle_msg.transform.translation.x = 1.5;
	obstacle_msg.transform.translation.y = 1.5;
	obstacle_msg.transform.translation.z = 0.05;
	double obstacle_center_x = obstacle_msg.transform.translation.x - R_OBSTACLE;
	double obstacle_center_y = obstacle_msg.transform.translation.y;
	waypoint_msg.transform.translation.x = 3.0;
	waypoint_msg.transform.translation.y = 3.0;
	waypoint_msg.transform.translation.z = 0.0;
	double waypoint_center_x = waypoint_msg.transform.translation.x - R_WAYPOINT;
	double waypoint_center_y = waypoint_msg.transform.translation.y;
	final_msg.transform.translation.x = 0.0;
	final_msg.transform.translation.y = 6.0;
	final_msg.transform.translation.z = 0.0;
	double final_center_x = final_msg.transform.translation.x - R_FINAL;
	double final_center_y = final_msg.transform.translation.y;

	double t_i = ros::Time::now().toNSec();
	double t_now = 0.0;
	while(ros::ok())
	{
		if(STATE_MOVE)
		{
			state_msg.pose.position.x = state_center_x + R_STATE*cos(2*PI*F_STATE*t_now);
			state_msg.pose.position.y = state_center_y + R_STATE*sin(2*PI*F_STATE*t_now);
		}
		if(OBSTACLE_MOVE)
		{
			obstacle_msg.transform.translation.x = obstacle_center_x + R_OBSTACLE*cos(2*PI*F_OBSTACLE*t_now);
			obstacle_msg.transform.translation.y = obstacle_center_y + R_OBSTACLE*sin(2*PI*F_OBSTACLE*t_now);
		}
		if(WAYPOINT_MOVE)
		{
			waypoint_msg.transform.translation.x = waypoint_center_x + R_WAYPOINT*cos(2*PI*F_WAYPOINT*t_now);
			waypoint_msg.transform.translation.y = waypoint_center_y + R_WAYPOINT*sin(2*PI*F_WAYPOINT*t_now);
		}
		if(FINAL_MOVE)
		{
			final_msg.transform.translation.x = final_center_x + R_FINAL*cos(2*PI*F_FINAL*t_now);
			final_msg.transform.translation.y = final_center_y + R_FINAL*sin(2*PI*F_FINAL*t_now);
		}

		//state_pub.publish(state_msg);
		obstacle_pub.publish(obstacle_msg);
		waypoint_pub.publish(waypoint_msg);
		final_pub.publish(final_msg);

		ros::spinOnce();
		ros::Duration(0.004).sleep();

		t_now = (ros::Time::now().toNSec()-t_i)/1000000000.0;
	}
	return 0;
}
