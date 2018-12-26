#include <message_handler.h>

using namespace hss;
using namespace TNT;

typedef string String;

MessageHandler::MessageHandler()
{
	nh = new ros::NodeHandle();

	startTime = ros::Time::now();

	parser = new ros::NodeHandle(ros::this_node::getName().c_str());

	std::string dir, name;
	if(parser->getParam("file_directory", dir) && 
		parser->getParam("file_name", name))
	{
		ROS_INFO_STREAM("Log file will be saved to " << dir << "/" << name << ".txt");
		dir_ = dir;
		log = new DataLogger(dir, name);
	}
	else
	{
		ROS_INFO("Log file will be saved to default directory");
		log = new DataLogger;
	}
	log->start();

	// subscriber setting
	sub = (ros::Subscriber*)malloc(30 * sizeof(ros::Subscriber));

	int count = 0;
	std::string buf;
	
	if(parser->getParam("vicon_pose", buf))
	{
		sub[count] = nh->subscribe<Pose>( buf.c_str(), 10, 
				boost::bind(&MessageHandler::pose_cb, this, _1, VICON_POSE) );
		ROS_INFO("Subscribe to %s", buf.c_str());
		count++;
	}
	if(parser->getParam("local_pose", buf))
	{
		sub[count] = nh->subscribe<Pose>( buf.c_str(), 10, 
				boost::bind(&MessageHandler::pose_cb, this, _1, LOCAL_POSE) );
		ROS_INFO("Subscribe to %s", buf.c_str());
		count++;
	}
	if(parser->getParam("vicon_twist", buf))
	{
		sub[count] = nh->subscribe<Twist>( buf.c_str(), 10, 
				boost::bind(&MessageHandler::twist_cb, this, _1, VICON_TWIST) );
		ROS_INFO("Subscribe to %s", buf.c_str());
		count++;
	}
	if(parser->getParam("local_twist", buf))
	{
		sub[count] = nh->subscribe<Twist>( buf.c_str(), 10, 
				boost::bind(&MessageHandler::twist_cb, this, _1, LOCAL_TWIST) );
		ROS_INFO("Subscribe to %s", buf.c_str());
		count++;
	}
	if(parser->getParam("imu_raw", buf))
	{
		sub[count] = nh->subscribe<Imu>( buf.c_str(), 10, 
				boost::bind(&MessageHandler::imu_cb, this, _1, IMU_RAW) );
		ROS_INFO("Subscribe to %s", buf.c_str());
		count++;
	}
	if(parser->getParam("imu", buf))
	{
		sub[count] = nh->subscribe<Imu>( buf.c_str(), 10, 
				boost::bind(&MessageHandler::imu_cb, this, _1, IMU) );
		ROS_INFO("Subscribe to %s", buf.c_str());
		count++;
	}
	if(parser->getParam("lidar", buf))
	{
		sub[count] = nh->subscribe<Point>( buf.c_str(), 10, 
				boost::bind(&MessageHandler::point_cb, this, _1, LIDAR) );
		ROS_INFO("Subscribe to %s", buf.c_str());
		count++;
	}
	if(parser->getParam("gps_mavros_fix", buf))
	{
		sub[count] = nh->subscribe<GPS>( buf.c_str(), 10, 
				boost::bind(&MessageHandler::gps_cb, this, _1, GPS_MAVROS_FIX) );
		ROS_INFO("Subscribe to %s", buf.c_str());
		count++;
	}
	if(parser->getParam("gps_mavros_fix_vel", buf))
	{
		sub[count] = nh->subscribe<Twist>( buf.c_str(), 10, 
				boost::bind(&MessageHandler::twist_cb, this, _1, GPS_MAVROS_FIX_VEL) );
		ROS_INFO("Subscribe to %s", buf.c_str());
		count++;
	}
	if(parser->getParam("att_sp", buf))
	{
		sub[count] = nh->subscribe<AttSp>( buf.c_str(), 10, 
				boost::bind(&MessageHandler::att_sp_cb, this, _1, SETPOINT_ATT) );
		ROS_INFO("Subscribe to %s", buf.c_str());
		count++;
	}
}

MessageHandler::~MessageHandler()
{
	delete log;
	delete nh;
	delete parser;
	free(sub);
}

void MessageHandler::pose_cb(const Pose::ConstPtr& msg, const LogIDs& id)
{
	//double timestamp = (msg->header.stamp - startTime).toSec();
	double timestamp = (ros::Time::now() - startTime).toSec();

	Array2D<double> quat = Zeros(4,1);
	quat[0][0] = msg->pose.orientation.w;
	quat[1][0] = msg->pose.orientation.x;
	quat[2][0] = msg->pose.orientation.y;
	quat[3][0] = msg->pose.orientation.z;

	Array2D<double> euler = so3::R2rpy( so3::q2R(quat) ); 

	Array2D<double> buffer = Zeros(6,1);
	buffer[0][0] = msg->pose.position.x;
	buffer[1][0] = msg->pose.position.y;
	buffer[2][0] = msg->pose.position.z;
	buffer[3][0] = euler[0][0];
	buffer[4][0] = euler[1][0];
	buffer[5][0] = euler[2][0];

	log_mutex.lock();
	log->addEntry( id, buffer, timestamp);
	log_mutex.unlock();
}

void MessageHandler::twist_cb(const Twist::ConstPtr& msg, const LogIDs& id)
{
	//double timestamp = (msg->header.stamp - startTime).toSec();
	double timestamp = (ros::Time::now() - startTime).toSec();

	Array2D<double> buffer = Zeros(6,1);
	buffer[0][0] = msg->twist.linear.x;
	buffer[1][0] = msg->twist.linear.y;
	buffer[2][0] = msg->twist.linear.z;
	buffer[3][0] = msg->twist.angular.x;
	buffer[4][0] = msg->twist.angular.y;
	buffer[5][0] = msg->twist.angular.z;

	log_mutex.lock();
	log->addEntry( id, buffer, timestamp);
	log_mutex.unlock();
}

void MessageHandler::imu_cb(const Imu::ConstPtr& msg, const LogIDs& id)
{
	//double timestamp = (msg->header.stamp - startTime).toSec();
	double timestamp = (ros::Time::now() - startTime).toSec();

	Array2D<double> quat = Zeros(4,1);
	quat[0][0] = msg->orientation.w;
	quat[1][0] = msg->orientation.x;
	quat[2][0] = msg->orientation.y;
	quat[3][0] = msg->orientation.z;
	Array2D<double> euler = so3::R2rpy( so3::q2R(quat) ); 
	
	Array2D<double> buffer = Zeros(9,1);
	buffer[0][0] = euler[0][0];
	buffer[1][0] = euler[1][0];
	buffer[2][0] = euler[2][0];
	buffer[3][0] = msg->angular_velocity.x;
	buffer[4][0] = msg->angular_velocity.y;
	buffer[5][0] = msg->angular_velocity.z;
	buffer[6][0] = msg->linear_acceleration.x;
	buffer[7][0] = msg->linear_acceleration.y;
	buffer[8][0] = msg->linear_acceleration.z;

	log_mutex.lock();
	log->addEntry( id, buffer, timestamp);
	log_mutex.unlock();
}

void MessageHandler::point_cb(const Point::ConstPtr& msg, const LogIDs& id)
{
	//double timestamp = (msg->header.stamp - startTime).toSec();
	double timestamp = (ros::Time::now() - startTime).toSec();
	
	Array2D<double> buffer = Zeros(3,1);
	buffer[0][0] = msg->point.x;
	buffer[1][0] = msg->point.y;
	buffer[2][0] = msg->point.z;

	log_mutex.lock();
	log->addEntry( id, buffer, timestamp);
	log_mutex.unlock();
}

void MessageHandler::odometry_cb(const Odometry::ConstPtr& msg, const LogIDs& id)
{
	//double timestamp = (msg->header.stamp - startTime).toSec();
	double timestamp = (ros::Time::now() - startTime).toSec();
	
	Array2D<double> quat = Zeros(4,1);
	quat[0][0] = msg->pose.pose.orientation.w;
	quat[1][0] = msg->pose.pose.orientation.x;
	quat[2][0] = msg->pose.pose.orientation.y;
	quat[3][0] = msg->pose.pose.orientation.z;
	Array2D<double> euler = so3::R2rpy( so3::q2R(quat) ); 

	Array2D<double> buffer = Zeros(12,1);
	buffer[0][0] = msg->pose.pose.position.x;	
	buffer[1][0] = msg->pose.pose.position.y;	
	buffer[2][0] = msg->pose.pose.position.z;	
	buffer[3][0] = euler[0][0];
	buffer[4][0] = euler[1][0];
	buffer[5][0] = euler[2][0];
	buffer[6][0] = msg->twist.twist.linear.x;
	buffer[7][0] = msg->twist.twist.linear.y;
	buffer[8][0] = msg->twist.twist.linear.z;
	buffer[9][0] = msg->twist.twist.angular.x;
	buffer[10][0] = msg->twist.twist.angular.y;
	buffer[11][0] = msg->twist.twist.angular.z;

//	buffer[12][0] = msg->pose.covariance[0];
//	buffer[13][0] = msg->pose.covariance[7];
//	buffer[14][0] = msg->pose.covariance[14];
//	buffer[15][0] = msg->pose.covariance[21];
//	buffer[16][0] = msg->pose.covariance[28];
//	buffer[17][0] = msg->pose.covariance[35];
//
//	buffer[18][0] = msg->twist.covariance[0];
//	buffer[19][0] = msg->twist.covariance[7];
//	buffer[20][0] = msg->twist.covariance[14];
//	buffer[21][0] = msg->twist.covariance[21];
//	buffer[22][0] = msg->twist.covariance[28];
//	buffer[23][0] = msg->twist.covariance[35];

	log_mutex.lock();
	log->addEntry( id, buffer, timestamp);
	log_mutex.unlock();
}

void MessageHandler::pose_cov_cb(const PoseCov::ConstPtr& msg, const LogIDs& id)
{
	//double timestamp = (msg->header.stamp - startTime).toSec();
	double timestamp = (ros::Time::now() - startTime).toSec();

	Array2D<double> quat = Zeros(4,1);
	quat[0][0] = msg->pose.pose.orientation.w;
	quat[1][0] = msg->pose.pose.orientation.x;
	quat[2][0] = msg->pose.pose.orientation.y;
	quat[3][0] = msg->pose.pose.orientation.z;

	Array2D<double> euler = so3::R2rpy( so3::q2R(quat) ); 

	Array2D<double> buffer = Zeros(12,1);
	buffer[0][0] = msg->pose.pose.position.x;
	buffer[1][0] = msg->pose.pose.position.y;
	buffer[2][0] = msg->pose.pose.position.z;
	buffer[3][0] = euler[0][0];
	buffer[4][0] = euler[1][0];
	buffer[5][0] = euler[2][0];
	buffer[6][0] = msg->pose.covariance[0];
	buffer[7][0] = msg->pose.covariance[7];
	buffer[8][0] = msg->pose.covariance[14];
	buffer[9][0] = msg->pose.covariance[21];
	buffer[10][0] = msg->pose.covariance[28];
	buffer[11][0] = msg->pose.covariance[35];

	log_mutex.lock();
	log->addEntry( id, buffer, timestamp);
	log_mutex.unlock();
}

void MessageHandler::twist_cov_cb(const TwistCov::ConstPtr& msg, const LogIDs& id)
{
	//double timestamp = (msg->header.stamp - startTime).toSec();
	double timestamp = (ros::Time::now() - startTime).toSec();

	Array2D<double> buffer = Zeros(12,1);
	buffer[0][0] = msg->twist.twist.linear.x;
	buffer[1][0] = msg->twist.twist.linear.y;
	buffer[2][0] = msg->twist.twist.linear.z;
	buffer[3][0] = msg->twist.twist.angular.x;
	buffer[4][0] = msg->twist.twist.angular.y;
	buffer[5][0] = msg->twist.twist.angular.z;
	buffer[6][0] = msg->twist.covariance[0];
	buffer[7][0] = msg->twist.covariance[7];
	buffer[8][0] = msg->twist.covariance[14];
	buffer[9][0] = msg->twist.covariance[21];
	buffer[10][0] = msg->twist.covariance[28];
	buffer[11][0] = msg->twist.covariance[35];

	log_mutex.lock();
	log->addEntry( id, buffer, timestamp);
	log_mutex.unlock();
}

void MessageHandler::gps_cb(const GPS::ConstPtr& msg, const LogIDs& id)
{
	//double timestamp = (msg->header.stamp - startTime).toSec();
	double timestamp = (ros::Time::now() - startTime).toSec();

	Array2D<double> buffer = Zeros(8,1);
	buffer[0][0] = msg->latitude;
	buffer[1][0] = msg->longitude;
	buffer[2][0] = msg->altitude;
	buffer[3][0] = msg->position_covariance[0];
	buffer[4][0] = msg->position_covariance[4];
	buffer[5][0] = msg->position_covariance[8];
	buffer[6][0] = msg->status.status;
	buffer[7][0] = msg->status.service;

	log_mutex.lock();
	log->addEntry( id, buffer, timestamp);
	log_mutex.unlock();
}

void MessageHandler::att_sp_cb(const AttSp::ConstPtr& msg, const LogIDs& id)
{
	//double timestamp = (msg->header.stamp - startTime).toSec();
	double timestamp = (ros::Time::now() - startTime).toSec();
	
	Array2D<double> quat = Zeros(4,1);
	quat[0][0] = msg->orientation.w;
	quat[1][0] = msg->orientation.x;
	quat[2][0] = msg->orientation.y;
	quat[3][0] = msg->orientation.z;

	Array2D<double> euler = so3::R2rpy( so3::q2R(quat) ); 

	Array2D<double> buffer = Zeros(7,1);
	buffer[0][0] = euler[0][0]; // roll 
	buffer[1][0] = euler[1][0]; // pitch
	buffer[2][0] = euler[2][0]; // yaw
	buffer[3][0] = msg->body_rate.x;
	buffer[4][0] = msg->body_rate.y;
	buffer[5][0] = msg->body_rate.z;
	buffer[6][0] = msg->thrust;

	log_mutex.lock();
	log->addEntry( id, buffer, timestamp);
	log_mutex.unlock();
}
