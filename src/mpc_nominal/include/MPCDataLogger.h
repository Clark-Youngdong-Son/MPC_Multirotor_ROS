#ifndef _MPC_DATA_LOGGER_H_
#define _MPC_DATA_LOGGER_H_

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <ctime>
#include <string>
#include <Params.h>

using namespace Eigen;
typedef Matrix<double, n, N+1> StateNominal;
typedef Matrix<double, m, N>   InputNominal;
typedef Matrix<double, 1, N+1> TimeNominal;
typedef Matrix<double, n, n>   NNMatrix;
typedef Matrix<double, m, m>   MMMatrix;
typedef Matrix<double, n, m>   NMMatrix;
typedef Matrix<double, m, n>   MNMatrix;
typedef Matrix<double, n, 1>   NVector;
typedef Matrix<double, m, 1>   MVector;

namespace SYD {
	class MPCDataLogger {

	public:
	MPCDataLogger();
	~MPCDataLogger();
	void addLine(const double &t_now, const StateNominal &x_n, const InputNominal &u_n, const NVector &x_waypt, const NVector &x_final, const Vector3d &x_obstacle, const double &t_compute, const double &cost_new, const double &t_w);

	private:
	std::ofstream file_;
	std::string fileName;
	};
}
#endif
