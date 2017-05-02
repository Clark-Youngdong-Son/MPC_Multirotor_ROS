#ifndef _MPC_H_
#define _MPC_H_

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <keyboard/Key.h>

#include <Params.h>
#include <MPCDataLogger.h>

namespace SYD{
	class MPC {

	public:
		MPC();
		~MPC();
		void waitStart();
		void start();
	private:
		ros::NodeHandle nh;

		ros::Subscriber current_pose_sub, current_vel_sub, start_command_sub, obstacle_pose_sub, waypoint_pose_sub, final_pose_sub;
		ros::Publisher setpoint_pub;
	
		void start_command_callback(const keyboard::Key::ConstPtr&); 
		void current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr&);
		void current_vel_callback(const geometry_msgs::TwistStamped::ConstPtr&);
		void obstacle_pose_callback(const geometry_msgs::TransformStamped::ConstPtr&);
		void waypoint_pose_callback(const geometry_msgs::TransformStamped::ConstPtr&);
		void final_pose_callback(const geometry_msgs::TransformStamped::ConstPtr&);

		NNMatrix L, Q, W;
		MMMatrix R;
		double mass, mass_l, length, I_x, I_y, I_z;
		Matrix3d I;
		double a_w, b_w, alpha, beta, d_des;
		double dt, g;
		double cost_new, cost_old;
		double t_init, t_now;
		double t_virtual;
		int iter_SLQ;

		StateNominal x_nominal;
		InputNominal u_nominal;
		Vector3d x_obstacle;
		bool initialized, stopFlag, stateSubFlag, velocitySubFlag, obstacleSubFlag, waypointSubFlag, finalSubFlag, simStopFlag, regularizationFlag;

		void forwardSimulation(StateNominal &x_n, const InputNominal &u_n, const double &dt);
		NVector dynamics_multirotor(const NVector &x_now, const MVector &u_now, const double &dt);	
		NVector dynamics_slungload(const NVector &x_now, const MVector &u_now, const double &dt);	
		Matrix3d rpy2R(const Vector3d &rpy);
		Vector3d pqr2rpyDot(const Vector3d &pqr, const Vector3d &rpy);

		double computeCost(const StateNominal &x, const InputNominal &u, const TimeNominal &t_span);

		NVector x_initial, x_final, x_inter, x_waypt;
		MVector u_inter;
		TimeNominal t_span;
		double t_w;

		void computeTSpan(TimeNominal &t_span, const double &t_now, const double &dt, const int &_N);
		void solveSLQ();
		void computeDynamicsGradient(NNMatrix &A, NMMatrix &B, const double &dX, const NVector &x, const double &dU, const MVector &u);	
		void computeCostGradient1x(NVector &q1, const double &t_backward, const double &g_old, const double &dX, const NVector &x, const MVector &u);
		void computeCostGradient1u(MVector &r1, const double &t_backward, const double &g_old, const NVector &x, const double &dU, const MVector &u);
		void computeCostGradient2(const NVector &q1, const MVector &r1, NNMatrix &Q2, MMMatrix &R2, const double &t_backward, const double &dX, const NVector &x, const double &dU, const MVector &u);
		double compute_g(const NVector &x, const MVector &u, const double t_backward);
		void lineSearch(const double cost_old, const StateNominal &x_n, const InputNominal &u_n, const std::vector<MNMatrix> &K, const std::vector<MVector> &l, const int _MAX_LINE);
		LineVector decay_coefficient, cost_array;
		int min_index;

		NNMatrix A;
		NMMatrix B;
		NVector s1, s1_new, q1;
		NNMatrix S2, S2_new, Q2;
		MVector r1, g_;
		MMMatrix R2, H_, H_inv;
		MNMatrix G_;
		EigenValues minEigValue_old, minEigValue_new;

		std::vector<MNMatrix>      K;
		std::vector<MVector>       l;
		std::vector<StateNominal>  X_series;
		std::vector<InputNominal>  U_series;

		double t_compute;

		MPCDataLogger *logger;
		void saveData();

		void waitSubscription();

		bool isNaN(double number);
	};
}

#endif
