#ifndef MESSAGEHANDLER
#define MESSAGEHANDLER
#include <sstream>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>

#include <mavros_msgs/AttitudeTarget.h>

#include "data_logger.h"
#include "privateFunctions.h"

typedef geometry_msgs::PoseStamped Pose;
typedef geometry_msgs::TwistStamped Twist;
typedef sensor_msgs::Imu Imu;
typedef geometry_msgs::PointStamped Point;
typedef nav_msgs::Odometry Odometry;
typedef geometry_msgs::PoseWithCovarianceStamped PoseCov;
typedef geometry_msgs::TwistWithCovarianceStamped TwistCov;
typedef sensor_msgs::NavSatFix GPS;
typedef mavros_msgs::AttitudeTarget AttSp;

namespace hss
{

class MessageHandler
{
public:
	MessageHandler();
	~MessageHandler();

private:

	DataLogger* log;
	mutex log_mutex;

	ros::Time startTime;
	ros::NodeHandle* nh;
	ros::NodeHandle* parser;
	ros::Subscriber* sub;

	void pose_cb(const Pose::ConstPtr& msg, const LogIDs& id);
	void twist_cb(const Twist::ConstPtr& msg, const LogIDs& id);
	void imu_cb(const Imu::ConstPtr& msg, const LogIDs& id);
	void point_cb(const Point::ConstPtr& msg, const LogIDs& id);
	void odometry_cb(const Odometry::ConstPtr& msg, const LogIDs& id);
	void pose_cov_cb(const PoseCov::ConstPtr& msg, const LogIDs& id);
	void twist_cov_cb(const TwistCov::ConstPtr& msg, const LogIDs& id);
	void gps_cb(const GPS::ConstPtr& msg, const LogIDs& id);
	void att_sp_cb(const AttSp::ConstPtr& msg, const LogIDs& id);

	std::string dir_;
};

};
#endif
