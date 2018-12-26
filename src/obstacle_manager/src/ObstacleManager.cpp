#include <ObstacleManager.h>

using namespace SYD;

ObstacleManager::ObstacleManager() : 
initialized(false), justInitialized(false), obstacleSubFlag(false), obstaclePoseSubFlag(false),
positionSubFlag(false),
measureNumber(0), obstacleAdded(0)
{
	std::string topicName;
	//Subscriber allocation
	nh.getParam("topicList/obstacle_sense", topicName);
	printDebug(topicName);
	obstacle_sub = nh.subscribe<geometry_msgs::PoseArray>
						("object_detection/info_array", 10, &ObstacleManager::obstacle_cb, this);
	nh.getParam("topicList/obstacle_sense_pose", topicName);
	obstacle_pose_sub = nh.subscribe<geometry_msgs::TransformStamped>
						(topicName, 10, &ObstacleManager::obstacle_pose_cb, this);
	nh.getParam("topicList/vehicle_pose", topicName);
	position_sub = nh.subscribe<geometry_msgs::PoseStamped>
						(topicName, 10, &ObstacleManager::position_cb, this);
	//Publisher allocation	
	obstacle_pub     = nh.advertise<geometry_msgs::PoseArray>
						("syd/obstacle_detector/obstacle_pub", 10);
	obstacle_vis_virtual_pub = nh.advertise<visualization_msgs::MarkerArray>
						("syd/obstacle_detector/virtual", 10);
	obstacle_vis_measure_pub = nh.advertise<visualization_msgs::MarkerArray>
						("syd/obstacle_detector/measure", 10);
	obstacle_vis_estimation_pub = nh.advertise<visualization_msgs::MarkerArray>
						("syd/obstacle_detector/estimation", 10);

	//Variables initialization
	Q       = NNMatrix::Zero();
	R       = NNMatrix::Zero();
	Identity= Matrix3d::Identity();

	//Parameter loading
	while(!loadParameters() && !ros::ok())
	{
		ros::Duration(0.1).sleep();
		printDebug("Parameter loading failed");
	}
	printParameters();

	//Checked completely - 2017.11.27
}

ObstacleManager::~ObstacleManager()
{

	//Checked completely - 2017.11.27
}

void ObstacleManager::start()
{
	ros::Rate rate(_rate);
	while(ros::ok())
	{
		waitMeasure();
		obstacleListSetting();
		if(initialized && !justInitialized) predict_update();
		publishResult();
		rate.sleep();
	}
}

void ObstacleManager::printDebug(const std::string &msg)
{
	std::cout << "\033[0;36m" << "[ObstacleManager] " << msg << "\033[0m" << std::endl;

	//Checked completely - 2017.11.27
}

bool ObstacleManager::loadParameters()
{
	XmlRpc::XmlRpcValue param_list;
	bool result = true;
	if(!nh.getParam("setting/algorithm/transmitRate", _rate)) result = false;

	if(!nh.getParam("setting/obstacleInfo/obstacleMaxNumber", OBSTACLE_MAX_NUMBER)) result = false;

	if(!nh.getParam("setting/function/enable_virtual_obstacle", ENABLE_VIRTUAL_OBSTACLE)) result = false;
	if(!nh.getParam("setting/function/enable_ground_truth", ENABLE_GROUND_TRUTH)) result = false;
	if(ENABLE_VIRTUAL_OBSTACLE || ENABLE_GROUND_TRUTH)
	{
		if(!nh.getParam("setting/obstacleInfo/obstaclePositions", param_list)) result = false;
		OBSTACLE_NUMBER_VIRTUAL = param_list.size()/3;
		OBSTACLE_POSITIONS_VIRTUAL.resize(OBSTACLE_NUMBER_VIRTUAL);
		for(int i=0; i<OBSTACLE_POSITIONS_VIRTUAL.size(); i++)
		{
			OBSTACLE_POSITIONS_VIRTUAL[i](0) = static_cast<double>(param_list[3*i+0]);
			OBSTACLE_POSITIONS_VIRTUAL[i](1) = static_cast<double>(param_list[3*i+1]);
			OBSTACLE_POSITIONS_VIRTUAL[i](2) = static_cast<double>(param_list[3*i+2]);
		}
	}
	if(!nh.getParam("setting/obstacleInfo/obstacleScale", param_list)) result = false;
	OBSTACLE_SCALE.resize(3);
	OBSTACLE_SCALE[0] = static_cast<double>(param_list[0]);
	OBSTACLE_SCALE[1] = static_cast<double>(param_list[1]);
	OBSTACLE_SCALE[2] = static_cast<double>(param_list[2]);
	if(!nh.getParam("setting/obstacleInfo/obstacleEnableRange", OBSTACLE_ENABLE_RANGE)) result = false;
	if(!nh.getParam("setting/obstacleInfo/obstacleRefreshRange", OBSTACLE_REFRESH_RANGE)) result = false;
	if(!nh.getParam("setting/obstacleInfo/obstacleRemoveTime", OBSTACLE_REMOVE_TIME)) result = false;
	if(!nh.getParam("setting/function/enable_detection_noise", ENABLE_DETECTION_NOISE)) result = false;
	
	if(!nh.getParam("setting/noiseParameter/process_noise", param_list)) result = false;
	for(int i=0; i<n; i++)
	{
		Q(i,i) = static_cast<double>(param_list[i]);
	}
	if(!nh.getParam("setting/noiseParameter/measurement_noise", param_list)) result = false;
	for(int i=0; i<m; i++)
	{
		R(i,i) = static_cast<double>(param_list[i]);
	}
	return result;

	//Checked completely - 2017.11.27
}

void ObstacleManager::printParameters()
{
	std::cout << "[ObstacleManager] Rate = " << _rate << std::endl;
	std::cout << "[ObstacleManager] ObstaclePositions = " << std::endl;
	for(int i=0; i<OBSTACLE_POSITIONS_VIRTUAL.size(); i++)
	{
		std::cout << OBSTACLE_POSITIONS_VIRTUAL[i] << std::endl;
	}
	std::cout << "[ObstacleManager] ProcessNoise = " << std::endl << Q << std::endl;
	std::cout << "[ObstacleManager] MeasurementNoise = " << std::endl << R << std::endl;

	std::cout << "[ObstacleManager] ENABLE_VIRTUAL_OBSTACLE = " << ENABLE_VIRTUAL_OBSTACLE << std::endl;
	std::cout << "[ObstacleManager] ENABLE_DETECTION_NOISE = " << ENABLE_DETECTION_NOISE << std::endl;

	//Checked completely - 2017.11.27
}

void ObstacleManager::waitMeasure()
{
	while(ros::ok())
	{
		ros::spinOnce();
		//printDebug("1: " + std::to_string(obstacleSubFlag));
		//printDebug("2: " + std::to_string(positionSubFlag));
		if((obstacleSubFlag || ENABLE_VIRTUAL_OBSTACLE) && positionSubFlag)
		{
			if(obstacleSubFlag && obstaclePoseSubFlag)
			{
				Rotation_now = Rotation_temp;
				x_q_now = x_q_temp;
			}
			if(ENABLE_VIRTUAL_OBSTACLE) generateVirtualMeasurements();
			obstacleSubFlag = false;
			positionSubFlag = false;
			break;
		}
		ros::Duration(0.01).sleep();
	}

	//Checked completely - 2017.11.27
}

void ObstacleManager::generateVirtualMeasurements()
{
	measureNumber = OBSTACLE_NUMBER_VIRTUAL;  //Assume all obstacles are measurable
	measure_raw.resize(measureNumber);
	for(int i=0; i<measureNumber; i++)
	{
		measure_raw[i] = Rotation_now.transpose()*(OBSTACLE_POSITIONS_VIRTUAL[i]-x_q_now);
		if(ENABLE_DETECTION_NOISE)
		{
			random_numbers::RandomNumberGenerator _obj;
			for(int j=0; j<m; j++)
			{
				measure_raw[i](j,0) += _obj.gaussian(0,R(j,j));
			}
		}
	}

	//Checked completely - 2017.11.27
}

void ObstacleManager::obstacleListSetting()
{
	//Find near measurements & connect to the nearest obstacles
	x_m_enable.resize(OBSTACLE_MAX_NUMBER);
	enable_number = 0;
	for(int i=0; i<measureNumber; i++)
	{
		double distance = measure_raw[i].norm();
		if(distance<=OBSTACLE_ENABLE_RANGE)
		{
			x_m_enable[enable_number++] = measure_raw[i];
		}
	}
	x_m_enable.resize(enable_number);
	x_m = x_m_enable;

	if(enable_number>0)
	{
		if(!initialized)
		{
			t_init = ros::Time::now();
			obstacleNumber = enable_number;

			x_old.resize(enable_number);
			x_hat.resize(enable_number);

			P_old.resize(enable_number);
			P_hat.resize(enable_number);

			Rotation_old.resize(enable_number);
			x_q_old.resize(enable_number);

			mapResult.resize(enable_number);

			time_old.resize(enable_number);
			for(int i=0; i<enable_number; i++)
			{
				x_old[i] = x_m_enable[i];
				x_hat[i] = x_m_enable[i];

				P_old[i] = R;
				P_hat[i] = R;

				Rotation_old[i] = Rotation_now;
				x_q_old[i] = x_q_now;

				mapResult[i] = i;

				time_old[i] = 0.0;
			}
			initialized = true;
			justInitialized = true;
			printDebug("Initialization finished with first " + std::to_string(enable_number) + " measurement(s)");
			printDebug("obstacleNumber : " + std::to_string(obstacleNumber));
		}
		else
		{
			mapResult = obstacleMapping(x_m_enable, enable_number);	
			manageNewObstacle();
			justInitialized = false;
		}
	}

	//Checked completely - 2017.11.29
}

std::vector<double> ObstacleManager::obstacleMapping(const std::vector<NVector> &x_m_enable, const int &enable_number)
{
    std::vector< std::vector<double> > distances(enable_number, std::vector<double>(obstacleNumber, 0.0));
	std::vector<double> allocation(enable_number, 0.0);

	//Compute distances to each obstacle from each measurement
	NVector x_m_inertia = NVector::Zero();
	for(int i=0; i<enable_number; i++)
	{
		x_m_inertia = x_q_now + Rotation_now*x_m_enable[i];
		for(int j=0; j<obstacleNumber; j++)
		{
			NVector x_old_inertia = x_q_old[j] + Rotation_old[j]*x_old[j];
			distances[i][j] = (x_m_inertia - x_old_inertia).norm();
		}
	}
	//for(int i=0; i<enable_number; i++)
	//{
	//	for(int j=0; j<obstacleNumber; j++)
	//	{
	//		std::cout << "i : " << i << " j : " << j << " distance : " << distances[i][j] << std::endl;
	//	}
	//}

	//Find the closest obstacle or allot new obstacle
	double minValue;
	int minIndex;
	for(int i=0; i<enable_number; i++)
	{
		for(int j=0; j<obstacleNumber; j++)
		{
			if(j==0)
			{
				minValue = distances[i][0];
				minIndex = 0;
			}
			else
			{
				if(distances[i][j] < minValue)
				{
					minIndex = j;
					minValue = distances[i][j];
				}
			}
		}
		if(minValue<OBSTACLE_REFRESH_RANGE)	allocation[i] = minIndex;
		else								allocation[i] = -1;
	}

	return allocation;

	//Checked completely - 2017.11.29
}

void ObstacleManager::manageNewObstacle()
{
	for(int i=0; i<mapResult.size(); i++)
	{
		if(mapResult[i] == -1 && !justInitialized)
		{
			if(obstacleNumber==OBSTACLE_MAX_NUMBER) printDebug("Maximum obstacle number reached");
			else
			{
				x_hat.push_back(x_m[i]);	
				P_hat.push_back(R);
				x_old.push_back(x_m[i]);
				P_old.push_back(R);

				x_q_old.push_back(x_q_now);
				Rotation_old.push_back(Rotation_now);
				time_old.push_back((ros::Time::now() - t_init).toSec());
				obstacleNumber++;
				obstacleAdded++;
				printDebug("New obstacle found. obstacleNumber : " + std::to_string(obstacleNumber));
			}	
		}
	}

	//Checked completely - 2017.11.29
}

void ObstacleManager::predict_update()
{
	//std::cout << x_q_old.size() << std::endl;
	//std::cout << Rotation_old.size() << std::endl;
	//std::cout << x_old.size() << std::endl;
	//std::cout << x_hat.size() << std::endl;
	//std::cout << P_old.size() << std::endl;
	//std::cout << P_hat.size() << std::endl;
	std::vector<bool> updatedIdx(obstacleNumber, false);
	if(obstacleAdded>0)
	{
		for(int i=obstacleNumber-1; i>obstacleNumber-1-obstacleAdded; i--)
		{
			updatedIdx[i] = true;
		}
		obstacleAdded = 0;
	}

	for(int i=0; i<x_m.size(); i++)
	{
		if(mapResult[i] != -1)
		{
			int idx = mapResult[i];
			updatedIdx[idx] = true;
			u_m = x_q_old[idx] - x_q_now;
			A = Rotation_now.transpose()*Rotation_old[idx];
			B = Rotation_now.transpose();
			C = Identity;

			x_e = A*x_old[idx] + B*u_m;
			P_e = A*P_old[idx]*A.transpose();

			y = x_m[i] - C*x_e;
			S = R + C*P_e*C.transpose();
			K = P_e*C.transpose()*S.inverse();

			x_hat[idx] = x_e + K*y;
			//std::cout << "idx : " << idx << std::endl;
			//std::cout << "x_hat : " << std::endl << x_hat[idx] << std::endl;
			P_hat[idx] = P_e - K*S*K.transpose();
			//std::cout << "P_hat : " << std::endl << P_hat[idx] << std::endl;

			time_old[idx] = (ros::Time::now()-t_init).toSec();
			x_old[idx] = x_hat[idx];
			P_old[idx] = P_hat[idx];
			x_q_old[idx] = x_q_now;
			Rotation_old[idx] = Rotation_now;
		}
	}
	for(int i=0; i<obstacleNumber; i++)
	{
		if(!updatedIdx[i])
		{
			u_m = x_q_old[i] - x_q_now;
			A = Rotation_now.transpose()*Rotation_old[i];
			B = Rotation_now.transpose();
			C = Identity;

			x_hat[i] = A*x_old[i] + B*u_m;
			P_hat[i] = A*P_old[i]*A.transpose();
			x_old[i] = x_hat[i];
			P_old[i] = P_hat[i];
			x_q_old[i] = x_q_now;
			Rotation_old[i] = Rotation_now;
		}
	}

	removeOldObstacle();
	//Checked completely - 2017.11.29
}

void ObstacleManager::removeOldObstacle()
{
	for(int i=obstacleNumber-1; i>-1; i--)
	{
		if(((ros::Time::now()-t_init).toSec()-time_old[i]) >= OBSTACLE_REMOVE_TIME)
		{
			x_hat.erase(x_hat.begin() + i);
			P_hat.erase(P_hat.begin() + i);
			x_old.erase(x_old.begin() + i);
			P_old.erase(P_old.begin() + i);
			x_q_old.erase(x_q_old.begin() + i);
			time_old.erase(time_old.begin() + i);
			obstacleNumber--;
			printDebug("Obstacle number decreased. Now : " + std::to_string(obstacleNumber));
		}
	}
}

void ObstacleManager::publishResult()
{
	visualization_msgs::MarkerArray marker_array_msg;
	geometry_msgs::PoseArray pose_array_msg;
	if(initialized)
	{
		marker_array_msg.markers.resize(obstacleNumber);
		for(int i=0; i<obstacleNumber; i++)
		{
			Vector3d x_hat_i = x_q_now + Rotation_now*x_hat[i];
			//std::cout << "x_hat : " << std::endl << x_hat[i] << std::endl;
			//std::cout << "x_hat_i : " << std::endl << x_hat_i << std::endl;
			
			pose_array_msg.header.frame_id = "map";
			pose_array_msg.header.stamp = ros::Time::now();
			pose_array_msg.header.seq = i;
			pose_array_msg.poses.resize(obstacleNumber);
			pose_array_msg.poses[i].position.x = x_hat_i(0,0);
			pose_array_msg.poses[i].position.y = x_hat_i(1,0);
			pose_array_msg.poses[i].position.z = x_hat_i(2,0);
			pose_array_msg.poses[i].orientation.x = 0.0;
			pose_array_msg.poses[i].orientation.y = 0.0;
			pose_array_msg.poses[i].orientation.z = 0.0;
			pose_array_msg.poses[i].orientation.w = 1.0;

			marker_array_msg.markers[i].header.stamp = ros::Time::now();
			marker_array_msg.markers[i].header.frame_id = "map";
			marker_array_msg.markers[i].header.seq = i;
			marker_array_msg.markers[i].ns = "";
			marker_array_msg.markers[i].id = i;
			marker_array_msg.markers[i].type = marker_array_msg.markers[i].CUBE;
			marker_array_msg.markers[i].action = marker_array_msg.markers[i].MODIFY;
			marker_array_msg.markers[i].pose.position.x = x_hat_i(0,0);
			marker_array_msg.markers[i].pose.position.y = x_hat_i(1,0);
			marker_array_msg.markers[i].pose.position.z = x_hat_i(2,0);
			marker_array_msg.markers[i].pose.orientation.x = 0.0;
			marker_array_msg.markers[i].pose.orientation.y = 0.0;
			marker_array_msg.markers[i].pose.orientation.z = 0.0;
			marker_array_msg.markers[i].pose.orientation.w = 1.0;
			marker_array_msg.markers[i].scale.x = OBSTACLE_SCALE[0];
			marker_array_msg.markers[i].scale.y = OBSTACLE_SCALE[1];
			marker_array_msg.markers[i].scale.z = OBSTACLE_SCALE[2];
			marker_array_msg.markers[i].color.r = 0.0;
			marker_array_msg.markers[i].color.g = 0.0;
			marker_array_msg.markers[i].color.b = 0.0;
			marker_array_msg.markers[i].color.a = 1.0;
			marker_array_msg.markers[i].lifetime = ros::Duration(1.0);

		}
		obstacle_pub.publish(pose_array_msg);	
		obstacle_vis_estimation_pub.publish(marker_array_msg);
	}

	if(ENABLE_VIRTUAL_OBSTACLE || ENABLE_GROUND_TRUTH)
	{
		marker_array_msg.markers.resize(OBSTACLE_NUMBER_VIRTUAL);
		for(int i=0; i<OBSTACLE_NUMBER_VIRTUAL; i++)
		{
			marker_array_msg.markers[i].header.stamp = ros::Time::now();
			marker_array_msg.markers[i].header.frame_id = "map";
			marker_array_msg.markers[i].header.seq = i;
			marker_array_msg.markers[i].ns = "";
			marker_array_msg.markers[i].id = i;
			marker_array_msg.markers[i].type = marker_array_msg.markers[i].CUBE;
			marker_array_msg.markers[i].action = marker_array_msg.markers[i].MODIFY;
			marker_array_msg.markers[i].pose.position.x = OBSTACLE_POSITIONS_VIRTUAL[i](0,0);
			marker_array_msg.markers[i].pose.position.y = OBSTACLE_POSITIONS_VIRTUAL[i](1,0);
			marker_array_msg.markers[i].pose.position.z = OBSTACLE_POSITIONS_VIRTUAL[i](2,0);
			marker_array_msg.markers[i].pose.orientation.x = 0.0;
			marker_array_msg.markers[i].pose.orientation.y = 0.0;
			marker_array_msg.markers[i].pose.orientation.z = 0.0;
			marker_array_msg.markers[i].pose.orientation.w = 1.0;
			marker_array_msg.markers[i].scale.x = OBSTACLE_SCALE[0];
			marker_array_msg.markers[i].scale.y = OBSTACLE_SCALE[1];
			marker_array_msg.markers[i].scale.z = OBSTACLE_SCALE[2];
			marker_array_msg.markers[i].color.r = 0.9;
			marker_array_msg.markers[i].color.g = 0.9;
			marker_array_msg.markers[i].color.b = 0.9;
			marker_array_msg.markers[i].color.a = 1.0;
			marker_array_msg.markers[i].lifetime = ros::Duration(1.0);
		}	
		obstacle_vis_virtual_pub.publish(marker_array_msg);
	}

	marker_array_msg.markers.resize(x_m.size());
	for(int i=0; i<x_m.size(); i++)
	{
		NVector x_m_i = x_q_now + Rotation_now*x_m[i];
		marker_array_msg.markers[i].header.stamp = ros::Time::now();
		marker_array_msg.markers[i].header.frame_id = "map";
		marker_array_msg.markers[i].header.seq = i;
		marker_array_msg.markers[i].ns = "";
		marker_array_msg.markers[i].id = i;
		marker_array_msg.markers[i].type = marker_array_msg.markers[i].CUBE;
		marker_array_msg.markers[i].action = marker_array_msg.markers[i].MODIFY;
		marker_array_msg.markers[i].pose.position.x = x_m_i(0,0);
		marker_array_msg.markers[i].pose.position.y = x_m_i(1,0);
		marker_array_msg.markers[i].pose.position.z = x_m_i(2,0);
		marker_array_msg.markers[i].pose.orientation.x = 0.0;
		marker_array_msg.markers[i].pose.orientation.y = 0.0;
		marker_array_msg.markers[i].pose.orientation.z = 0.0;
		marker_array_msg.markers[i].pose.orientation.w = 1.0;
		marker_array_msg.markers[i].scale.x = OBSTACLE_SCALE[0];
		marker_array_msg.markers[i].scale.y = OBSTACLE_SCALE[1];
		marker_array_msg.markers[i].scale.z = OBSTACLE_SCALE[2];
		marker_array_msg.markers[i].color.r = 0.45;
		marker_array_msg.markers[i].color.g = 0.45;
		marker_array_msg.markers[i].color.b = 0.45;
		marker_array_msg.markers[i].color.a = 0.6;
		marker_array_msg.markers[i].lifetime = ros::Duration(1.0);
	}
	obstacle_vis_measure_pub.publish(marker_array_msg);

	//Checked completely - 2017.11.29
}

//Callback Functions
void ObstacleManager::obstacle_cb(const geometry_msgs::PoseArray::ConstPtr &msg)
{
	measureNumber = msg->poses.size();
	if(measureNumber>0)
	{
		measure_raw.resize(measureNumber);
		for(int i=0; i<measureNumber; i++)
		{
			measure_raw[i](0,0) = msg->poses[i].position.x;
			measure_raw[i](1,0) = msg->poses[i].position.y;
			measure_raw[i](2,0) = msg->poses[i].position.z;
		}
		obstacleSubFlag = true;
	}
}

void ObstacleManager::obstacle_pose_cb(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
	pose_temp = *msg;	
	double q1 = pose_temp.transform.rotation.x;
	double q2 = pose_temp.transform.rotation.y;
	double q3 = pose_temp.transform.rotation.z;
	double q0 = pose_temp.transform.rotation.w;
    double r = atan2(2*(q0*q1 + q2*q3),1-2*(q1*q1+q2*q2));
    double p = asin(2*(q0*q2-q3*q1));
    double y = atan2(2*(q0*q3 + q1*q2),1-2*(q2*q2+q3*q3));
	Rotation_temp(0,0) = cos(p)*cos(y);
	Rotation_temp(0,1) = cos(y)*sin(p)*sin(r)-cos(r)*sin(y);
	Rotation_temp(0,2) = sin(r)*sin(y)+cos(r)*cos(y)*sin(p);
	Rotation_temp(1,0) = cos(p)*sin(y);
	Rotation_temp(1,1) = cos(r)*cos(y)+sin(p)*sin(r)*sin(y);
	Rotation_temp(1,2) = cos(r)*sin(p)*sin(y)-cos(y)*sin(r);
	Rotation_temp(2,0) = -sin(p);
	Rotation_temp(2,1) = cos(p)*sin(r);
	Rotation_temp(2,2) = cos(p)*cos(r);

	x_q_temp(0,0) = pose_temp.transform.translation.x;
	x_q_temp(1,0) = pose_temp.transform.translation.y;
	x_q_temp(2,0) = pose_temp.transform.translation.z;

	obstaclePoseSubFlag = true;
}

void ObstacleManager::position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	pose_now = *msg;
	double q1 = pose_now.pose.orientation.x;	
	double q2 = pose_now.pose.orientation.y;	
	double q3 = pose_now.pose.orientation.z;	
	double q0 = pose_now.pose.orientation.w;	
    double r = atan2(2*(q0*q1 + q2*q3),1-2*(q1*q1+q2*q2));
    double p = asin(2*(q0*q2-q3*q1));
    double y = atan2(2*(q0*q3 + q1*q2),1-2*(q2*q2+q3*q3));
	
	Rotation_now(0,0) = cos(p)*cos(y);
	Rotation_now(0,1) = cos(y)*sin(p)*sin(r)-cos(r)*sin(y);
	Rotation_now(0,2) = sin(r)*sin(y)+cos(r)*cos(y)*sin(p);
	Rotation_now(1,0) = cos(p)*sin(y);
	Rotation_now(1,1) = cos(r)*cos(y)+sin(p)*sin(r)*sin(y);
	Rotation_now(1,2) = cos(r)*sin(p)*sin(y)-cos(y)*sin(r);
	Rotation_now(2,0) = -sin(p);
	Rotation_now(2,1) = cos(p)*sin(r);
	Rotation_now(2,2) = cos(p)*cos(r);
	Matrix3d RotZ = Matrix3d::Zero();
	Matrix3d RotY = Matrix3d::Zero();

	RotZ(0,0) = -1.0;
	RotZ(1,1) = -1.0;
	RotZ(2,2) =  1.0;
	RotY(0,2) = -1.0;
	RotY(1,1) =  1.0;
	RotY(2,0) =  1.0;
	Rotation_now = Rotation_now*RotZ*RotY;
	//Rotation_now = Rotation_now;
	x_q_now(0,0) = pose_now.pose.position.x;
	x_q_now(1,0) = pose_now.pose.position.y;
	x_q_now(2,0) = pose_now.pose.position.z;
	positionSubFlag = true;
	
	//Checked completely - 2017.11.29
}
