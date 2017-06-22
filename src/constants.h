#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__
#include <cmath>

// TODO: Set the timestep length and duration
const size_t N 				= 11;
const double dt 			= 0.1;
// Set the constants. These have been hand-tuned.
const double SMOOTH_FACTOR	= 100.0; 
const double CTE_SMOOTH		= SMOOTH_FACTOR / 2.0;
const double EPSI_SMOOTH	= SMOOTH_FACTOR / 240.0;
const double V_SMOOTH		= SMOOTH_FACTOR / 340.0;
const double DELTA_SMOOTH	= SMOOTH_FACTOR * 3450.0;
const double A_SMOOTH		= SMOOTH_FACTOR / 2.8;
const double D2_SMOOTH		= SMOOTH_FACTOR / 1000.0;
const double A2_SMOOTH		= SMOOTH_FACTOR / 2000.0;
const double LATENCY_SMOOTH	= SMOOTH_FACTOR * 2.3;
// Set the reference constants. Set the positive values only.
const double REF_VELOCITY 	= 80.0;
const double NON_ACTUATORS	= 1.0e19; 
const double D2R_25			= 25.0 * M_PI / 180.0;
const double ACCEL_BOUND	= 1.0;
const double DEFAULT_ACCEL	= 0.3;
// Set the the indices for the variables.
const size_t X_START 		= 0;
const size_t Y_START 		= X_START + N;
const size_t PSI_START	 	= Y_START + N;
const size_t V_START  		= PSI_START + N;
const size_t CTE_START   	= V_START + N;
const size_t EPSI_START 	= CTE_START + N;
const size_t DELTA_START 	= EPSI_START + N;
const size_t A_START 		= DELTA_START + N - 1;
// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double LF = 2.67;

#endif