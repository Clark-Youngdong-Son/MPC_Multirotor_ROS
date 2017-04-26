#ifndef _PARAMS_H_
#define _PARAMS_H_

#define ENABLE_LOGGING true

#define MULTIROTOR 0
#define SLUNGLOAD  1

#define PLATFORM MULTIROTOR

#if PLATFORM == MULTIROTOR
	#define n		12
	#define m		4
#endif
#if PLATFORM == SLUNGLOAD
	#define n		18
	#define m		4
#endif

#define NUMERICAL 0
#define ANALYTIC  1

#define DERIVATIVE ANALYTIC

#define SIMULATION true
#define SIM_EPSILON 0.01

#define VIRTUAL_ENVIRONMENT true
#define VIRTUAL_MAX_TIME    6.0

#define N		 200
#define MAX_LINE 8
#define MAX_SLQ  10
#define STOP_SLQ -1.0
//Cost weights
//Final
#if PLATFORM == MULTIROTOR
	#define L_1		100.0
	#define L_2		100.0
	#define L_3		300.0
	#define L_4		10.0
	#define L_5		10.0
	#define L_6		10.0
	#define L_7		10.0
	#define L_8		10.0
	#define L_9		10.0
	#define L_10	1.0
	#define L_11	1.0
	#define L_12	1.0


	#define L_13	0.0
	#define L_14	0.0
	#define L_15	0.0
	#define L_16	0.0
	#define L_17	0.0
	#define L_18	0.0
#endif

#if PLATFORM == SLUNGLOAD
	#define L_1		100.0
	#define L_2		100.0
	#define L_3		300.0
	#define L_4		10.0
	#define L_5		10.0
	#define L_6		10.0
	#define L_7		50.0
	#define L_8		50.0
	#define L_9		200.0
	#define L_10	10.0
	#define L_11	10.0
	#define L_12	10.0
	#define L_13	1.0
	#define L_14	1.0
	#define L_15	1.0
	#define L_16	10.0
	#define L_17	10.0
	#define L_18	10.0
#endif
//Intermediate
#if PLATFORM == MULTIROTOR
	#define Q_1		1.0	
	#define Q_2		1.0
	#define Q_3		1.0
	#define Q_4		1.0
	#define Q_5		1.0
	#define Q_6		1.0
	#define Q_7		0.0
	#define Q_8		0.0
	#define Q_9		0.0
	#define Q_10	1.0
	#define Q_11	1.0
	#define Q_12	1.0


	#define Q_13	0.0
	#define Q_14	0.0
	#define Q_15	0.0
	#define Q_16	0.0
	#define Q_17	0.0
	#define Q_18	0.0
#endif

#if PLATFORM == SLUNGLOAD
	#define Q_1		0.0	
	#define Q_2		0.0
	#define Q_3		0.0
	#define Q_4		1.0
	#define Q_5		1.0
	#define Q_6		1.0
	#define Q_7		10.0
	#define Q_8		10.0
	#define Q_9		10.0
	#define Q_10	0.0
	#define Q_11	0.0
	#define Q_12	0.0
	#define Q_13	1.0
	#define Q_14	1.0
	#define Q_15	1.0
	#define Q_16	0.0
	#define Q_17	0.0
	#define Q_18	0.0
#endif
//Input
#if PLATFORM == MULTIROTOR
	#define R_1		40.0	
	#define R_2		40.0
	#define R_3		40.0
	#define R_4		600.0
#endif

#if PLATFORM == SLUNGLOAD
	#define R_1		40.0	
	#define R_2		40.0
	#define R_3		40.0
	#define R_4		600.0
#endif
//Waypoint
#if PLATFORM == MULTIROTOR
	#define W_1		60.0	
	#define W_2		60.0
	#define W_3		120.0
	#define W_4		1.0
	#define W_5		1.0
	#define W_6		1.0
	#define W_7		0.01
	#define W_8		0.01
	#define W_9		0.01
	#define W_10	0.01
	#define W_11	0.01
	#define W_12	0.01

	#define W_13	0.0
	#define W_14	0.0
	#define W_15	0.0
	#define W_16	0.0
	#define W_17	0.0
	#define W_18	0.0
#endif

#if PLATFORM == SLUNGLOAD
	#define W_1		60.0	
	#define W_2		60.0
	#define W_3		120.0
	#define W_4		1.0
	#define W_5		1.0
	#define W_6		1.0
	#define W_7		10.0
	#define W_8		10.0
	#define W_9		60.0
	#define W_10	0.01
	#define W_11	0.01
	#define W_12	0.01
	#define W_13	0.01
	#define W_14	0.01
	#define W_15	0.01
	#define W_16	0.01
	#define W_17	0.01
	#define W_18	0.01
#endif
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
typedef Matrix<double, n, N+1> StateSeries;
typedef Matrix<double, m, N>   InputSeries;
typedef Matrix<double, MAX_LINE, 1> LineVector;

#endif
