#include <Mpc.h>
#include <math.h>
#include <vector>
#include <string>

using namespace SYD;

MPC::MPC() :
mass(4.0), mass_l(1.0), length(0.5), I_x(1.0), I_y(1.0), I_z(1.0),
a_w(0.005), b_w(1000.0), alpha(8000.0), beta(2.0), d_des(0.0),
dt(0.04), g(9.81),
cost_new(0.0), cost_old(0.0), t_now(0.0), t_virtual(0.0), t_w(0.0), t_compute(0.0),
initialized(false), stopFlag(true),
stateSubFlag(false), velocitySubFlag(false), obstacleSubFlag(false),
waypointSubFlag(false), finalSubFlag(false),
simStopFlag(false), min_index(0)
{
	I(0,0) = I_x;
	I(1,1) = I_y;
	I(2,2) = I_z;

	//Make weight matrices
	L = NNMatrix::Zero();	
	Q = NNMatrix::Zero();	
	R = MMMatrix::Zero();	
	W = NNMatrix::Zero();

	NVector L_array, Q_array, W_array;
	MVector R_array;

	if(PLATFORM==MULTIROTOR)
	{
		L_array << L_1, L_2, L_3, L_4, L_5, L_6, L_7, L_8, L_9, L_10, L_11, L_12;
		Q_array << Q_1, Q_2, Q_3, Q_4, Q_5, Q_6, Q_7, Q_8, Q_9, Q_10, Q_11, Q_12;
		R_array << R_1, R_2, R_3, R_4;
		W_array << W_1, W_2, W_3, W_4, W_5, W_6, W_7, W_8, W_9, W_10, W_11, W_12;
	}
	else if(PLATFORM==SLUNGLOAD)
	{
		L_array << L_1, L_2, L_3, L_4, L_5, L_6, L_7, L_8, L_9, L_10, L_11, L_12, L_13, L_14, L_15, L_16, L_17, L_18;
		Q_array << Q_1, Q_2, Q_3, Q_4, Q_5, Q_6, Q_7, Q_8, Q_9, Q_10, Q_11, Q_12, Q_13, Q_14, Q_15, Q_16, Q_17, Q_18;
		R_array << R_1, R_2, R_3, R_4;
		W_array << W_1, W_2, W_3, W_4, W_5, W_6, W_7, W_8, W_9, W_10, W_11, W_12, W_13, W_14, W_15, W_16, W_17, W_18;
	}
	
	for(int i=0; i<n; i++)
	{
		L(i,i) = L_array(i)*100.0;
		Q(i,i) = Q_array(i)*100.0/5000.0;
		W(i,i) = W_array(i)*1000.0;
		if(i<m)
		{
			R(i,i) = R_array(i)*0.2;
		}
	}
	
	x_nominal = StateNominal::Zero();
	u_nominal = InputNominal::Zero();	
	x_inter   = NVector::Zero();
	x_inter(8) = -1.0;
	u_inter   = MVector::Zero();
	x_final   = NVector::Zero(n);
	x_final(2) = 1.5;////
	x_waypt   = NVector::Zero(n);
	t_span    = TimeNominal::Zero();
	x_obstacle= Vector3d::Zero();
	
	A  = NNMatrix::Zero();
	B  = NMMatrix::Zero();

	s1     = NVector::Zero();
	s1_new = NVector::Zero();
	S2     = NNMatrix::Zero();
	S2_new = NNMatrix::Zero();
	q1     = NVector::Zero();
	Q2     = NNMatrix::Zero();
	r1     = MVector::Zero();
	R2     = MMMatrix::Zero();
	G_     = MNMatrix::Zero();
	H_     = MMMatrix::Zero();
	H_inv  = MMMatrix::Zero();
	g_     = MVector::Zero();

	K.reserve(N);
	l.reserve(N);
	for(int i=0; i<N; i++)
	{
		K[i].setZero();
		l[i].setZero();
	}

	decay_coefficient = LineVector::Zero();
	for(int i=0; i<MAX_LINE; i++)
	{
		decay_coefficient(i) = -i*(3.0/(double)(MAX_LINE-1));
	}

	cost_array = LineVector::Zero();

	X_series.reserve(MAX_LINE);
	U_series.reserve(MAX_LINE);
	for(int i=0; i<MAX_LINE; i++)
	{
		X_series[i].setZero();
		U_series[i].setZero();
	}
	current_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &MPC::current_pose_callback, this);
	current_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity", 10, &MPC::current_vel_callback, this);
	start_command_sub = nh.subscribe<keyboard::Key>("keyboard/keydown", 10, &MPC::start_command_callback, this);
	obstacle_pose_sub = nh.subscribe<geometry_msgs::TransformStamped>("obstacle1/pose", 10, &MPC::obstacle_pose_callback, this);
	waypoint_pose_sub = nh.subscribe<geometry_msgs::TransformStamped>("waypoint/pose", 10, &MPC::waypoint_pose_callback, this);
	final_pose_sub = nh.subscribe<geometry_msgs::TransformStamped>("final/pose", 10, &MPC::final_pose_callback, this);

	setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",1);

	if(SIMULATION==true)
	{
		stateSubFlag = true;
		velocitySubFlag = true;
		obstacleSubFlag = true;
		waypointSubFlag = true;	
		finalSubFlag = true;
		x_initial.fill(0);
		if(PLATFORM==MULTIROTOR)
		{
			x_initial.fill(0.0);
		//	x_initial(3) = 0.05;
		//	x_initial(4) = 0.05;
		//	x_initial(5) = 0.05;
	
			x_waypt.fill(0.0);
			x_waypt(0) = 3.0;
			x_waypt(1) = 3.0;
	
			x_obstacle = Vector3d(1.5,1.5,0.05);
	
			x_final.fill(0.0);
			x_final(1) = 6.0;
	
			x_inter.fill(0);
	
			t_w = 2.0;
		}
		else if(PLATFORM==SLUNGLOAD)
		{
		//	x_initial(3) = 0.05;
		//	x_initial(4) = 0.05;
		//	x_initial(5) = 0.05;
			x_initial(8) = -1.0;
	
			x_waypt.fill(0);
			x_waypt(0) = 3.0;
			x_waypt(1) = 3.0;
			x_waypt(8) = -1.0;
	
			x_obstacle = Vector3d(1.5,1.5,0.05);
	
			x_final.fill(0);
			x_final(1) = 6.0;
			x_final(8) = -1.0;
	
			x_inter.fill(0);
			x_inter(8) = -1.0;
	
			t_w = 2.0;
		}
		//std::cout << "x_initial" << std::endl;
		//std::cout << x_initial << std::endl;
		//std::cout << "x_inter" << std::endl;
		//std::cout << x_inter << std::endl;
		//std::cout << "x_waypt" << std::endl;
		//std::cout << x_waypt << std::endl;
		//std::cout << "x_obstacle" << std::endl;
		//std::cout << x_obstacle << std::endl;
	}
	if(ENABLE_LOGGING)
	{
		logger = new MPCDataLogger();
	}
	std::cout << "[MPC] Initialization Finished." << std::endl;
}

MPC::~MPC()
{
	delete logger;
}

void MPC::waitStart()
{
	std::cout << "[MPC] Waiting States from VICON" << std::endl;
	waitSubscription();
	while(ros::ok())
	{
		ros::spinOnce();
		if(!stopFlag && stateSubFlag && velocitySubFlag) break;
	}
	start();
}

void MPC::start()
{
	ros::Rate _rate(1.0/dt);
	double i_init = 0.0;
	double t_init_SLQ = 0.0;
	while(!stopFlag && ros::ok())
	{
		ros::spinOnce();
		if(!initialized)
		{
			if(SIMULATION) x_nominal.col(0) = x_initial;
			if(PLATFORM==MULTIROTOR)		u_inter(0) = mass*g;
			else if(PLATFORM==SLUNGLOAD)	u_inter(0) = (mass+mass_l)*g;
			//Initial nominal input trajectory
			for(int i=0; i<N; i++)
			{
				if(PLATFORM==MULTIROTOR)
				{
					u_nominal(0,i)  = mass*g;		
				}
				else if(PLATFORM==SLUNGLOAD) 
				{
					u_nominal(0,i) = (mass+mass_l)*g;
				}
			}
			computeTSpan(t_span, t_now, dt, N);
			forwardSimulation(x_nominal, u_nominal, dt);	
			cost_old = computeCost(x_nominal, u_nominal, t_span); //TODO t_span & t_now
			t_init = ros::Time::now().toNSec();
			t_init_SLQ = t_init;
			initialized = true;
		}
		else
		{
			if(SIMULATION && VIRTUAL_ENVIRONMENT)
			{
				for(int i=0; i<N; i++)
				{
					if(PLATFORM==MULTIROTOR)
					{
						u_nominal(0,i)  = mass*g;		
					}
					else if(PLATFORM==SLUNGLOAD) 
					{
						u_nominal(0,i) = (mass+mass_l)*g;
					}
				}
				computeTSpan(t_span, t_now, dt, N);
				forwardSimulation(x_nominal, u_nominal, dt);	
				cost_old = computeCost(x_nominal, u_nominal, t_span); //TODO t_span & t_now
			}
			for(int i=0; i<MAX_SLQ; i++)
			{
				s1 = L*(x_nominal.col(N)-x_final);;
				S2 = L;
				if(!ros::ok()) break;
				solveSLQ();
				double cost_delta = cost_new - cost_old;
				std::cout << "ITER : " << i+1 << " cost old : " << cost_old << " cost new : " << cost_new << " cost delta : " << cost_delta << std::endl;
				if(isNaN(cost_new))
				{
					cost_new = cost_old;
					break;
				}
				else
				{
					if(cost_delta>STOP_SLQ)
					{
						//std::cout << "Converged" << std::endl;
						if(cost_delta>0) 
						{
							cost_new = cost_old;
							break;
						}
						else
						{ 
							x_nominal = X_series[min_index];
							u_nominal = U_series[min_index];
							cost_old = cost_new;
							break;
						}
					}
					else
					{
						x_nominal = X_series[min_index];
						u_nominal = U_series[min_index];
						cost_old = cost_new;
					}
				}
			}
			_rate.sleep();
			double t_temp = ros::Time::now().toNSec();
			t_compute = (t_temp - t_init_SLQ)/1000000000;
			std::cout << "t_compute : " << t_compute << std::endl;
			t_now = (t_temp - t_init)/1000000000;
			if(ENABLE_LOGGING) saveData();
			if(SIMULATION) 
			{
				t_now += dt;
				computeTSpan(t_span, t_now, dt, N);
				StateNominal x_temp = x_nominal;
				InputNominal u_temp = u_nominal;
				for(int i=0; i<N-1; i++)
				{
					x_nominal.col(i) = x_temp.col(i+1);
					u_nominal.col(i) = u_temp.col(i+1);
				}
				u_nominal.col(N-1) = u_nominal.col(N-2);
				x_nominal.col(N-1) = x_temp.col(N);

				double error = sqrt((x_nominal(0,0)-x_final(0))*(x_nominal(0,0)-x_final(0))+(x_nominal(1,0)-x_final(1))*(x_nominal(1,0)-x_final(1))+(x_nominal(2,0)-x_final(2))*(x_nominal(2,0)-x_final(2)));
				//std::cout << "Distance to final : " << error << std::endl;
				if(error<=SIM_EPSILON)
				{
					simStopFlag = true;
					break;
				}	
				if(PLATFORM==MULTIROTOR) 
					x_nominal.col(N) = dynamics_multirotor(x_nominal.col(N-1), u_nominal.col(N-1), dt); 
				else if(PLATFORM==SLUNGLOAD)
					x_nominal.col(N) = dynamics_slungload(x_nominal.col(N-1), u_nominal.col(N-1), dt); 
				cost_old = computeCost(x_nominal, u_nominal, t_span); //TODO t_span & t_now
				if(VIRTUAL_ENVIRONMENT)
				{
					obstacleSubFlag = false;
					waypointSubFlag = false;
					waitSubscription();
				}
			}
			else
			{
				stateSubFlag = false;
				velocitySubFlag = false;
				obstacleSubFlag = false;
				waypointSubFlag = false;
				waitSubscription();
				t_now = (ros::Time::now().toNSec()-t_init)/1000000000.0;
				InputNominal u_temp = u_nominal;
				for(int i=0; i<N-1; i++)
				{
					u_nominal.col(i) = u_temp.col(i+1);
				}
				u_nominal.col(N-1) = u_nominal.col(N-2);
				computeTSpan(t_span, t_now, dt, N);
				forwardSimulation(x_nominal, u_nominal, dt);	
				cost_old = computeCost(x_nominal, u_nominal, t_span); //TODO t_span & t_now
			}
			if(simStopFlag && SIMULATION)
			{
				std::cout << "Simulation Terminated" << std::endl;
				break;
			}
		}
	}
}

void MPC::computeTSpan(TimeNominal &t_span, const double &t_now, const double &dt, const int &_N)
{
	t_span(0) = t_now;
	for(int i=0; i<_N; i++)
	{
		t_span(i+1) = t_span(i) + dt;
	}
}

void MPC::forwardSimulation(StateNominal &x_n, const InputNominal &u_n, const double &dt)
{
	if(PLATFORM == MULTIROTOR)
	{
		for(int i=0; i<N; i++)
		{
			x_n.col(i+1) = dynamics_multirotor(x_n.col(i), u_n.col(i), dt);
		}
	}
	else if(PLATFORM == SLUNGLOAD)
	{
		for(int i=0; i<N; i++)
		{
			x_n.col(i+1) = dynamics_slungload(x_n.col(i), u_n.col(i), dt);
		}
	}
}

NVector MPC::dynamics_multirotor(const NVector &x_now, const MVector &u_now, const double &dt)
{
	NVector temp = NVector::Zero();

	Vector3d position = x_now.block(0,0,3,1);
	Vector3d attitude = x_now.block(3,0,3,1);
	Vector3d velocity = x_now.block(6,0,3,1);
	Vector3d pqr	  = x_now.block(9,0,3,1);

	Matrix3d R = rpy2R(attitude);
	Vector3d rpy_dot = pqr2rpyDot(pqr, attitude);

	Vector3d acceleration = R*Vector3d(0,0,1)*u_now(0)/mass - Vector3d(0,0,1)*g; 
	Vector3d acceleration_rotation = I.inverse()*(u_now.block(1,0,3,1)-pqr.cross(I*pqr)); 

	temp.block(0,0,3,1) = position + velocity*dt;
	temp.block(3,0,3,1) = attitude + rpy_dot*dt;
	temp.block(6,0,3,1) = velocity + acceleration*dt;
	temp.block(9,0,3,1) = pqr 	   + acceleration_rotation*dt;

	return temp;
}

NVector MPC::dynamics_slungload(const NVector &x_now, const MVector &u_now, const double &dt)
{
	NVector temp = NVector::Zero(18,1);

	Vector3d position = x_now.block(0,0,3,1);
	Vector3d attitude = x_now.block(3,0,3,1);
	Vector3d PQR      = x_now.block(6,0,3,1);
	//PQR = PQR/PQR.norm();
	PQR(0) = PQR(0)/sqrt(PQR(0)*PQR(0) + PQR(1)*PQR(1) + PQR(2)*PQR(2));
	PQR(1) = PQR(1)/sqrt(PQR(0)*PQR(0) + PQR(1)*PQR(1) + PQR(2)*PQR(2));
	PQR(2) = PQR(2)/sqrt(PQR(0)*PQR(0) + PQR(1)*PQR(1) + PQR(2)*PQR(2));
	Vector3d velocity = x_now.block(9,0,3,1);
	Vector3d pqr	  = x_now.block(12,0,3,1);
	Vector3d PQR_omega= x_now.block(15,0,3,1);

	Matrix3d R = rpy2R(attitude);
	Vector3d rpy_dot = pqr2rpyDot(pqr, attitude);
	Vector3d PQR_dot = PQR_omega.cross(PQR);
	Vector3d acceleration = ((PQR.dot(u_now(0)*R*Vector3d(0,0,1))-mass*length*PQR_dot.dot(PQR_dot))/(mass+mass_l))*PQR-Vector3d(0,0,g);
	Vector3d acceleration_rotation = I.inverse()*(u_now.block(1,0,3,1)-pqr.cross(I*pqr)); 
	temp.block(0,0,3,1)  = position + velocity*dt;
	temp.block(3,0,3,1)  = attitude + rpy_dot*dt;
	temp.block(6,0,3,1)  = PQR      + PQR_dot*dt;
	temp.block(9,0,3,1)  = velocity + acceleration*dt;
	temp.block(12,0,3,1) = pqr      + acceleration_rotation*dt;
	temp.block(15,0,3,1) = PQR_omega+ (-PQR.cross(u_now(0)*R*Vector3d(0,0,1)/mass/length))*dt;
	return temp;
}

Matrix3d MPC::rpy2R(const Vector3d &rpy)
{
	Matrix3d temp = Matrix3d::Zero();
	double r = rpy(0);
	double p = rpy(1);
	double y = rpy(2);
	temp(0,0) = cos(p)*cos(y);
	temp(0,1) = cos(y)*sin(p)*sin(r)-cos(r)*sin(y);
	temp(0,2) = sin(r)*sin(y)+cos(r)*cos(y)*sin(p);
	temp(1,0) = cos(p)*sin(y);
	temp(1,1) = cos(r)*cos(y)+sin(p)*sin(r)*sin(y);
	temp(1,2) = cos(r)*sin(p)*sin(y)-cos(y)*sin(r);
	temp(2,0) = -sin(p);
	temp(2,1) = cos(p)*sin(r);
	temp(2,2) = cos(p)*cos(r);
	return temp;
}

Vector3d MPC::pqr2rpyDot(const Vector3d &pqr, const Vector3d &rpy)
{
	double phi = rpy(0);
	double theta = rpy(1);
	
	Matrix3d conversion = Matrix3d::Zero();
	conversion(0,0) = 1;
	conversion(0,1) = sin(phi)*tan(theta);
	conversion(0,2) = cos(phi)*tan(theta);
	conversion(1,0) = 0;
	conversion(1,1) = cos(phi);
	conversion(1,2) = -sin(phi);
	conversion(2,0) = 0;
	conversion(2,1) = sin(phi)/cos(theta);
	conversion(2,2) = cos(phi)/cos(theta);
	return conversion*pqr;
}

void MPC::start_command_callback(const keyboard::Key::ConstPtr& msg)
{
	if(msg->code == 109) stopFlag = false; //109 equals M
	else if(msg->code == 44) stopFlag = true; //44 equals ,
}

void MPC::current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	if(!SIMULATION)
	{
		if(PLATFORM==MULTIROTOR)
		{
			x_nominal(0,0) = msg->pose.position.x;
			x_nominal(1,0) = msg->pose.position.y;
			x_nominal(2,0) = msg->pose.position.z;
			double q1 = msg->pose.orientation.x;
			double q2 = msg->pose.orientation.y;
			double q3 = msg->pose.orientation.z;
			double q0 = msg->pose.orientation.w;
			Matrix3d R = Matrix3d::Zero();
			R(0,0) = q0*q0 + q1*q1 - q2*q2 - q3*q3;
			R(0,1) = 2*(q1*q2-q0*q3);
			R(0,2) = 2*(q1*q3+q0*q2);
			R(1,0) = 2*(q1*q2+q0*q3);
			R(1,1) = q0*q0 - q1*q1 + q2*q2 - q3*q3;
			R(1,2) = 2*(q2*q3-q0*q1);
			R(2,0) = 2*(q1*q3-q0*q2);
			R(2,1) = 2*(q2*q3+q0*q1);
			R(2,2) = q0*q0 - q1*q1 - q2*q2 + q3*q3;
			x_nominal(3,0) = atan2(R(2,1),R(2,2));
			x_nominal(4,0) = atan2(-R(2,0),sqrt(R(2,1)*R(2,1)+R(2,2)*R(2,2)));
			x_nominal(5,0) = atan2(R(1,0),R(0,0));
		}
		stateSubFlag = true;
	}
}

void MPC::current_vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	if(!SIMULATION)
	{
		if(PLATFORM==MULTIROTOR)
		{
			x_nominal(6,0) = msg->twist.linear.x;
			x_nominal(7,0) = msg->twist.linear.y;
			x_nominal(8,0) = msg->twist.linear.z;
			x_nominal(9,0) = msg->twist.angular.x;
			x_nominal(10,0) = msg->twist.angular.y;
			x_nominal(11,0) = msg->twist.angular.z;
		}
		velocitySubFlag = true;
	}
}

void MPC::obstacle_pose_callback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	if(VIRTUAL_ENVIRONMENT || !SIMULATION)
	{
		x_obstacle.fill(0.0);
		x_obstacle(0) = msg->transform.translation.x;
		x_obstacle(1) = msg->transform.translation.y;
		x_obstacle(2) = msg->transform.translation.z;
		obstacleSubFlag = true;
	}
}

void MPC::waypoint_pose_callback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	x_waypt(0) = msg->transform.translation.x;
	if(VIRTUAL_ENVIRONMENT || !SIMULATION)
	{
		x_waypt.fill(0.0);
		x_waypt(0) = msg->transform.translation.x;
		x_waypt(1) = msg->transform.translation.y;
		x_waypt(2) = msg->transform.translation.z+1.5;
		if(PLATFORM==SLUNGLOAD)	x_waypt(8) = -1.0;
		waypointSubFlag = true;
	}
}

void MPC::final_pose_callback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	if(VIRTUAL_ENVIRONMENT || !SIMULATION)
	{
		x_final.fill(0.0);
		x_final(0) = msg->transform.translation.x;
		x_final(1) = msg->transform.translation.y;
		x_final(2) = msg->transform.translation.z;
		if(PLATFORM==SLUNGLOAD)	x_final(8) = -1.0;
		finalSubFlag = true;
	}
}

double MPC::computeCost(const StateNominal &x, const InputNominal &u, const TimeNominal &t_span)
{
	double temp_L = 0;
	double temp_Q = 0;
	double temp_R = 0;
	double temp_W = 0;
	double temp_O = 0;

	temp_L = 0.5*(x.col(N)-x_final).transpose()*L*(x.col(N)-x_final);
	for(int i=0; i<N; i++)
	{
		temp_Q += 0.5*dt*(x.col(i)-x_inter).transpose()*Q*(x.col(i)-x_inter);
		temp_R += 0.5*dt*(u.col(i)-u_inter).transpose()*R*(u.col(i)-u_inter);
		if(waypointSubFlag)
		{
			double temp1 = 0.5*dt*(x.col(i)-x_waypt).transpose()*W*(x.col(i)-x_waypt);
			double temp2 = exp(-b_w*(t_span(i)-t_w)*(t_span(i)-t_w))/a_w;
			temp_W += temp1*temp2;
		}
		if(obstacleSubFlag)
		{
			temp_O += alpha*exp(-beta*(((x.col(i).block(0,0,3,1)-x_obstacle).transpose()*(x.col(i).block(0,0,3,1)-x_obstacle)).value()-d_des*d_des));
		}
	}
	//std::cout << "temp_L : " << temp_L << std::endl;
	//std::cout << "temp_Q : " << temp_Q << std::endl;
	//std::cout << "temp_R : " << temp_R << std::endl;
	//std::cout << "temp_W : " << temp_W << std::endl;
	//std::cout << "temp_O : " << temp_O << std::endl;
	return temp_L + temp_Q + temp_R + temp_W + temp_O;
}

void MPC::solveSLQ()
{
	double dX = 0.005;
	double dU = 0.005;

	for(int i=N-1; i>-1; i--)
	{
		//x_nominal.col(i) << 4.089204, -3.704099, -0.184873, 0.05, 0.05, 0.05, 0.258043, -0.233741, -0.937687, 1.765352, -1.599098, -0.216978, 0.0, 0.0, 0.0, 0.652743, 0.720607, 0.0;
		computeDynamicsGradient(A, B, dX, x_nominal.col(i), dU, u_nominal.col(i));
		double t_backward = t_span(i);
		double g_old = compute_g(x_nominal.col(i), u_nominal.col(i), t_backward);
		computeCostGradient1x(q1, t_backward, g_old, dX, x_nominal.col(i), u_nominal.col(i));
		computeCostGradient1u(r1, t_backward, g_old, x_nominal.col(i), dU, u_nominal.col(i));
		computeCostGradient2(q1, r1, Q2, R2, t_backward, dX, x_nominal.col(i), dU, u_nominal.col(i));			
		//std::cout << "A" << std::endl;
		//std::cout << A << std::endl;
		//std::cout << "B" << std::endl;
		//std::cout << B << std::endl;
		//std::cout << "q1" << std::endl;
		//std::cout << q1 << std::endl;
		//std::cout << "r1" << std::endl;
		//std::cout << r1 << std::endl;
		//std::cout << "Q2" << std::endl;
		//std::cout << Q2 << std::endl;
		//std::cout << "R2" << std::endl;
		//std::cout << R2 << std::endl;
		//std::cout << "R" << std::endl;
		//std::cout << R << std::endl;
		G_    = (B.transpose())*S2*A;
		H_    = R2+(B.transpose())*S2*B;
		H_inv = H_.inverse();
		g_    = r1+(B.transpose())*s1;
		//std::cout << "G_" << std::endl;
		//std::cout << G_ << std::endl;
		//std::cout << "H_" << std::endl;
		//std::cout << H_ << std::endl;
		//std::cout << "H_inv" << std::endl;
		//std::cout << H_inv << std::endl;
		//std::cout << "g_" << std::endl;
		//std::cout << g_ << std::endl;
		//break;

		s1_new = q1+(A.transpose())*s1-(G_.transpose())*H_inv*g_;
		S2_new = Q2+(A.transpose())*S2*A-(G_.transpose())*H_inv*G_;
		s1 = s1_new;
		S2 = S2_new;
		K[i]   = -H_inv*G_;
		l[i]   = -H_inv*g_;
		//std::cout << "s1_new" << std::endl;
		//std::cout << s1_new << std::endl;
		//std::cout << "s2_new" << std::endl;
		//std::cout << S2_new << std::endl;
		//std::cout << "K" << std::endl;
		//std::cout << K[i] << std::endl;
		//std::cout << "l" << std::endl;
		//std::cout << l[i] << std::endl;
		//break;

		if(!ros::ok()) break;
	}
	lineSearch(cost_old, x_nominal, u_nominal, K, l, MAX_LINE);
}

void MPC::computeDynamicsGradient(NNMatrix &A, NMMatrix &B, const double &dX, const NVector &x, const double &dU, const MVector &u)
{
	if(PLATFORM==MULTIROTOR)
	{
		NVector dynamics_old = dynamics_multirotor(x,u,dt);
		for(int i=0; i<n; i++)
		{
			NVector dx = NVector::Zero();
			dx(i) = dX;
			A.col(i) = (dynamics_multirotor(x+dx,u,dt)-dynamics_old)/dX;
		}
		for(int i=0; i<m; i++)
		{
			MVector du = MVector::Zero();
			du(i) = dU;
			B.col(i) = (dynamics_multirotor(x,u+du,dt)-dynamics_old)/dX;
	}

	}
	else if(PLATFORM==SLUNGLOAD)
	{
		NVector dynamics_old = dynamics_slungload(x,u,dt);
		for(int i=0; i<n; i++)
		{
			NVector dx = NVector::Zero();
			dx(i) = dX;
			A.col(i) = (dynamics_slungload(x+dx,u,dt)-dynamics_old)/dX;
		}
		for(int i=0; i<m; i++)
		{
			MVector du = MVector::Zero();
			du(i) = dU;
			B.col(i) = (dynamics_slungload(x,u+du,dt)-dynamics_old)/dX;
		}
	}
}

void MPC::computeCostGradient1x(NVector &q1, const double &t_backward, const double &g_old, const double &dX, const NVector &x, const MVector &u)
{
	if(DERIVATIVE==NUMERICAL)
	{
		NVector dx;
		for(int i=0; i<n; i++)
		{
			dx = NVector::Zero();
			dx(i) = dX;
			q1(i) = (compute_g(x+dx,u,t_backward)-g_old)/dX;
		}
	}
	else if(DERIVATIVE==ANALYTIC)
	{
		if(PLATFORM==MULTIROTOR)
		{
			q1 = Q*(x-x_inter);
			if(waypointSubFlag)	q1 += W*(x-x_waypt)*exp(-b_w*(t_backward-t_w)*(t_backward-t_w))/a_w;

			if(obstacleSubFlag)
			{
				Vector3d temp = alpha*exp(-beta*((x.block(0,0,3,1)-x_obstacle).transpose()*(x.block(0,0,3,1)-x_obstacle)).value()-d_des*d_des)*(-2.0*beta*(x.block(0,0,3,1)-x_obstacle));
				NVector temp2 = NVector::Zero();
				temp2 << temp(0), temp(1), temp(2), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
				q1 += temp2;
			}
		}
		else if(PLATFORM==SLUNGLOAD)
		{
			q1 = Q*(x-x_inter);
			if(waypointSubFlag)	q1 += W*(x-x_waypt)*exp(-b_w*(t_backward-t_w)*(t_backward-t_w))/a_w;

			if(obstacleSubFlag)
			{
				Vector3d temp = alpha*exp(-beta*((x.block(0,0,3,1)-x_obstacle).transpose()*(x.block(0,0,3,1)-x_obstacle)).value()-d_des*d_des)*(-2.0*beta*(x.block(0,0,3,1)-x_obstacle));
				NVector temp2 = NVector::Zero();
				temp2 << temp(0), temp(1), temp(2), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
				q1 += temp2;
			}
		}
	}	
}

void MPC::computeCostGradient1u(MVector &r1, const double &t_backward, const double &g_old, const NVector &x, const double &dU, const MVector &u)
{
	if(DERIVATIVE==NUMERICAL)
	{
		MVector du;
		for(int i=0; i<m; i++)
		{
			du = MVector::Zero();
			du(i) = dU;
			r1(i) = (compute_g(x,u+du,t_backward)-g_old)/dU;
		}
	}
	else if(DERIVATIVE==ANALYTIC)
	{
		if(PLATFORM==MULTIROTOR)
		{
			r1 = R*(u-u_inter);
		}
		else if(PLATFORM==SLUNGLOAD)
		{
			r1 = R*(u-u_inter);
		}
	}
}

void MPC::computeCostGradient2(const NVector &q1, const MVector &r1, NNMatrix &Q2, MMMatrix &R2, const double &t_backward, const double &dX, const NVector &x, const double &dU, const MVector &u)
{
	if(DERIVATIVE==NUMERICAL)
	{
		NVector gradient_newX = NVector::Zero();
		NVector dx;
		for(int i=0; i<n; i++)
		{
			dx = NVector::Zero();
			dx(i) = dX;
			double g_old = compute_g(x+dx, u, t_backward);
			computeCostGradient1x(gradient_newX, t_backward, g_old, dX, x+dx, u);
			Q2.col(i) = (gradient_newX-q1)/dX;	
		}
		Q2 = (Q2 + Q2.transpose())*0.5;
		MVector gradient_newU = MVector::Zero();
		MVector du;
		for(int i=0; i<m; i++)
		{
			du = MVector::Zero();
			du(i) = dU;
			double g_old = compute_g(x, u+du, t_backward);
			computeCostGradient1u(gradient_newU, t_backward, g_old, x, dU, u+du);
			R2.col(i) = (gradient_newU-r1)/dU;
		}
		R2 = (R2 + R2.transpose())*0.5;
	}
	else if(DERIVATIVE==ANALYTIC)
	{
		if(PLATFORM==MULTIROTOR)
		{
			Q2 = Q;
			if(waypointSubFlag)	Q2 += W*exp(-b_w*(t_backward-t_w)*(t_backward-t_w))/a_w;

			if(obstacleSubFlag)
			{
				Matrix3d temp = alpha*exp(-beta*((x.block(0,0,3,1)-x_obstacle).transpose()*(x.block(0,0,3,1)-x_obstacle)).value()-d_des*d_des)*(4.0*beta*beta*(x.block(0,0,3,1)-x_obstacle)*(x.block(0,0,3,1)-x_obstacle).transpose());
				NNMatrix temp2 = NNMatrix::Zero();
				for(int i=0; i<3; i++)
				{
					for(int j=0; j<3; j++)
					{
						temp2(i,j) = temp(i,j);
					}
				}
				Q2 += temp2;
			}
			R2 = R;			
		}
		else if(PLATFORM==SLUNGLOAD)
		{
			Q2 = Q;
			if(waypointSubFlag)	Q2 += W*exp(-b_w*(t_backward-t_w)*(t_backward-t_w))/a_w;

			if(obstacleSubFlag)
			{
				Matrix3d temp = alpha*exp(-beta*((x.block(0,0,3,1)-x_obstacle).transpose()*(x.block(0,0,3,1)-x_obstacle)).value()-d_des*d_des)*(4.0*beta*beta*(x.block(0,0,3,1)-x_obstacle)*(x.block(0,0,3,1)-x_obstacle).transpose());
				NNMatrix temp2 = NNMatrix::Zero();
				for(int i=0; i<3; i++)
				{
					for(int j=0; j<3; j++)
					{
						temp2(i,j) = temp(i,j);
					}
				}
				Q2 += temp2;
			}
			R2 = R;			
		}
	}
}

double MPC::compute_g(const NVector &x, const MVector &u, const double t_backward)
{
	double temp = 0.0f;
	temp += (0.5*(x-x_inter).transpose()*Q*(x-x_inter)).value();
	temp += (0.5*(u-u_inter).transpose()*R*(u-u_inter)).value();
	if(waypointSubFlag)
	{
		double temp2 = (0.5*(x-x_waypt).transpose()*W*(x-x_waypt)).value();
		double temp3 = exp(-b_w*(t_backward-t_w)*(t_backward-t_w))/a_w;
		temp += temp2*temp3;
	}
	if(obstacleSubFlag)
	{
		temp += alpha*exp(-beta*(((x.block(0,0,3,1)-x_obstacle).transpose()*(x.block(0,0,3,1)-x_obstacle)).value()-d_des*d_des));
	}
	return temp;
}

void MPC::lineSearch(const double cost_old, const StateNominal &x_n, const InputNominal &u_n, const std::vector<MNMatrix> &K, const std::vector<MVector> &l, const int _MAX_LINE)
{
	min_index = 0;
	double cost_min_temp = cost_old;
	for(int i=0; i<MAX_LINE; i++)
	{
		X_series[i].col(0) = x_n.col(0);

		double forward_ratio = pow(10.0, decay_coefficient(i));
		for(int j=0; j<N; j++)
		{
			U_series[i].col(j) = u_n.col(j) + forward_ratio*l[j] + K[j]*(X_series[i].col(j)-x_n.col(j));

			if(PLATFORM==MULTIROTOR)
			{
				X_series[i].col(j+1) = dynamics_multirotor(X_series[i].col(j),U_series[i].col(j),dt);
			}
			else if(PLATFORM==SLUNGLOAD)
			{
				X_series[i].col(j+1) = dynamics_slungload(X_series[i].col(j),U_series[i].col(j),dt);
			}
		}
		if(!ros::ok()) break;
		cost_array[i] = computeCost(X_series[i],U_series[i],t_span); //TODO t_span
		if(cost_array[i]<cost_min_temp)
		{
			min_index = i;
			cost_min_temp = cost_array[i];
		}
	}
	//std::cout << std::endl;
	//std::cout << "cost_old : " << cost_old << std::endl;
	//std::cout << "cost_array" << std::endl;
	//for(int i=0; i<MAX_LINE; i++)
	//{
	//	std::cout << cost_array(i) << std::endl;
	//}
	//std::cout << "min_index : " << min_index << std::endl;
	//std::cout << "x(0) : " << std::endl;
	//std::cout << X_series[min_index].row(0) << std::endl;
	cost_new = cost_array(min_index);
}

void MPC::waitSubscription()
{
	while(ros::ok())
	{
		ros::spinOnce();
		if(!SIMULATION)
		{
			if(stateSubFlag && velocitySubFlag) break;
		}
		else
		{
			break;
		}
	}
}

void MPC::saveData()
{
	logger->addLine(t_now, x_nominal, u_nominal, x_waypt, x_final, x_obstacle, t_compute, cost_new, t_w); 
}

bool MPC::isNaN(double number)
{
	return (number != number);
}
