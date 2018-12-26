#ifndef _OBSTACLE_MANAGER_H_
#define _OBSTACLE_MANAGER_H_

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <cmath>
#include <random_numbers/random_numbers.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>

#define n 3
#define m 3

using namespace Eigen;

typedef Matrix<double, n, n> NNMatrix;
typedef Matrix<double, n, m> NMMatrix;
typedef Matrix<double, m, n> MNMatrix;
typedef Matrix<double, m, m> MMMatrix;
typedef Matrix<double, n, 1> NVector;
typedef Matrix<double, m, 1> MVector;

namespace SYD{
	class ObstacleManager {
	
	public:
		ObstacleManager();
		~ObstacleManager();
		void start();

	private:
		ros::NodeHandle nh;

		//Parameters
		int _rate;
		int OBSTACLE_NUMBER_VIRTUAL, OBSTACLE_MAX_NUMBER;
		std::vector<Vector3d> OBSTACLE_POSITIONS_VIRTUAL;
		bool ENABLE_VIRTUAL_OBSTACLE, ENABLE_DETECTION_NOISE, ENABLE_GROUND_TRUTH;
		double OBSTACLE_ENABLE_RANGE, OBSTACLE_REFRESH_RANGE;
		double OBSTACLE_REMOVE_TIME;
		std::vector<double> OBSTACLE_SCALE;

		//Subscribers
		ros::Subscriber obstacle_sub, obstacle_pose_sub, position_sub;		
		
		//Publishers
		ros::Publisher obstacle_pub;
		ros::Publisher obstacle_vis_virtual_pub, obstacle_vis_measure_pub, obstacle_vis_estimation_pub;
		ros::Publisher obstacle_vis_pub;
		
		//Variables
		bool initialized, justInitialized;
		bool obstacleSubFlag, obstaclePoseSubFlag, positionSubFlag, velocitySubFlag;
		int measureNumber, obstacleNumber;

		geometry_msgs::PoseStamped 	pose_now;
		geometry_msgs::TransformStamped pose_temp;
		std::vector<Vector3d> measure_raw;
		std::vector<NVector> x_hat, x_old, x_m, x_m_enable;
		NVector x_e, y;
		std::vector<NNMatrix> P_hat, P_old;
		NNMatrix S, K, P_e;

		NVector x_q_now, x_q_temp;
		std::vector<NVector> x_q_old;
		MVector u_m;
		NNMatrix Q, R;
		NNMatrix A, B, C;
		
		Matrix3d Rotation_now, Rotation_temp, Identity;
		std::vector<Matrix3d> Rotation_old;
		std::vector<double> time_old;

		std::vector<double> mapResult;

		int obstacleAdded;
		int enable_number;
		
		ros::Time t_init;
		//Functions
		void printDebug(const std::string &msg);
		bool loadParameters();
		void printParameters();
		void waitMeasure();
		void generateVirtualMeasurements();
		void obstacleListSetting();
		std::vector<double> obstacleMapping(const std::vector<NVector> &x_m_enable, const int &enable_number);
		void manageNewObstacle();
		void predict_update();
		void removeOldObstacle();
		void publishResult();
		//Callback Functions
		void obstacle_cb(const geometry_msgs::PoseArray::ConstPtr &msg);
		void obstacle_pose_cb(const geometry_msgs::TransformStamped::ConstPtr &msg);
		void position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
	};
}

#endif
