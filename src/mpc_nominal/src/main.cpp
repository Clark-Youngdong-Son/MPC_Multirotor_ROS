#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>
#include <Mpc.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mpc_nominal");
	SYD::MPC _MPC;
	_MPC.waitStart();

	return 0;
}
