#include <iostream>
#include <cmath>
#include "constants.h"
#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

class FG_eval {
public:
	// Fitted polynomial coefficients
	Eigen::VectorXd coeffs;
	// Constructor.
	FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

	typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
	void operator()(ADvector& fg, const ADvector& vars) {
		// TODO: implement MPC
		// `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
		// NOTE: You'll probably go back and forth between this function and
		// the Solver function below.
		setCost(fg, vars);
		setConstraints(fg, vars);
	}
  
private:
	/**
	* 	It sets the cost of the fg vector at position 0 (i.e., fg[0]).
	*
	*	@param fg is the cost constraints vector.
	*	@param vars is the array containing the variables.
	*/
	void setCost(ADvector& fg, const ADvector& vars) const {
		size_t t = 0;
		fg[0] = 0.0;
		// The part of the cost based on the reference state.
		for (; t < N; ++t) {
		  fg[0] += CTE_SMOOTH * CppAD::pow(vars[CTE_START + t], 2);
		  fg[0] += EPSI_SMOOTH * CppAD::pow(vars[EPSI_START + t], 2);
		  fg[0] += V_SMOOTH * CppAD::pow(vars[V_START + t] - REF_VELOCITY, 2);
		}
		// Minimize the use of actuators.
		for (t = 0; t < N - 1; ++t) {
		  fg[0] += DELTA_SMOOTH * CppAD::pow(vars[DELTA_START + t], 2);
		  fg[0] += A_SMOOTH * CppAD::pow(vars[A_START + t], 2);
		  fg[0] += LATENCY_SMOOTH * CppAD::pow(vars[A_START + t] + vars[DELTA_START + t], 3); // Latency penalty.
		}
		// Minimize the value gap between sequential actuations.
		for (t = 0; t < N - 2; ++t) {
		  fg[0] += D2_SMOOTH * CppAD::pow(vars[DELTA_START + t + 1] - vars[DELTA_START + t], 2);
		  fg[0] += A2_SMOOTH * CppAD::pow(vars[A_START + t + 1] - vars[A_START + t], 2);
		}
	}
	
	/**
	* 	It sets the constraints in the fg vector. Notice it uses
	*	the 'coeff' member attribute.
	*
	*	@param fg is the cost constraints vector.
	*	@param vars is the array containing the variables.
	*/
	void setConstraints(ADvector& fg, const ADvector& vars) {
		fg[1 + X_START] = vars[X_START];
		fg[1 + Y_START] = vars[Y_START];
		fg[1 + PSI_START] = vars[PSI_START];
		fg[1 + V_START] = vars[V_START];
		fg[1 + CTE_START] = vars[CTE_START];
		fg[1 + EPSI_START] = vars[EPSI_START];
		// The rest of the constraints
		for (size_t t = 0; t < N - 1; ++t) {
			// The state at time t.
			AD<double> x0 = vars[X_START + t];
			AD<double> y0 = vars[Y_START + t];
			AD<double> psi0 = vars[PSI_START + t];
			AD<double> v0 = vars[V_START + t];
			AD<double> cte0 = vars[CTE_START + t];
			AD<double> epsi0 = vars[EPSI_START + t];
			AD<double> delta0 = vars[DELTA_START + t];
			AD<double> a0 = vars[A_START + t];
			if (t > 1) {   //For taking into account the latency!
				delta0 = vars[DELTA_START + t - 2];
				a0 = vars[A_START + t - 2];
			}
			// The state at time t + 1.
			AD<double> x1 = vars[X_START + t + 1];
			AD<double> y1 = vars[Y_START + t + 1];
			AD<double> psi1 = vars[PSI_START + t + 1];
			AD<double> v1 = vars[V_START + t + 1];
			AD<double> cte1 = vars[CTE_START + t + 1];
			AD<double> epsi1 = vars[EPSI_START + t + 1];
			// Third degree polynomial fit.
			AD<double> x0_2 = x0 * x0;
			AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0_2 + coeffs[3] * x0_2 * x0;
			AD<double> psides0 = CppAD::atan(coeffs[1] + 2.0 * coeffs[2] * x0 + 3.0 * coeffs[3] * x0_2);
			// Set the kinematic model in form of constraints.
			fg[2 + X_START + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
			fg[2 + Y_START + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
			fg[2 + PSI_START + t] = psi1 - (psi0 + v0 / LF * delta0 * dt);
			fg[2 + V_START + t] = v1 - (v0 + a0 * dt);
			fg[2 + CTE_START + t] =
			  cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
			fg[2 + EPSI_START + t] =
			  epsi1 - ((psi0 - psides0) + v0 / LF * delta0 * dt);
		}
	}
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
	bool ok = true;
	size_t i = 0;
	typedef CPPAD_TESTVECTOR(double) Dvector;
	double x = state[0];
	double y = state[1];
	double psi = state[2];
	double v = state[3];
	double cte = state[4];
	double epsi = state[5];
	// TODO: Set the number of model variables (includes both states and inputs).
	// For example: If the state is a 4 element vector, the actuators is a 2
	// element vector and there are 10 timesteps. The number of variables is:
	// 4 * 10 + 2 * 9
	size_t n_vars = 6 * N + 2 * (N - 1);
	// TODO: Set the number of constraints
	size_t n_constraints = N * 6;
	// Initial value of the independent variables.
	// SHOULD BE 0 besides initial state.
	Dvector vars(n_vars);
	for (; i < n_vars; ++i) {
		vars[i] = 0.0;
	}
	// Set the initial variable values.
	vars[X_START] = x;
	vars[Y_START] = y;
	vars[PSI_START] = psi;
	vars[V_START] = v;
	vars[CTE_START] = cte;
	vars[EPSI_START] = epsi;
	// Lower and upper limits for x.
	Dvector vars_lowerbound(n_vars);
	Dvector vars_upperbound(n_vars);
	// TODO: Set lower and upper limits for variables.
	// Set all non-actuators upper and lowerlimits
	// to the max negative and positive values.
	for (i = 0; i < DELTA_START; ++i) {
		vars_lowerbound[i] = -NON_ACTUATORS;
		vars_upperbound[i] = NON_ACTUATORS;
	}
	// The upper and lower limits of delta are set to -25 and 25
	// degrees (values in radians).
	for (i = DELTA_START; i < A_START; ++i) {
		vars_lowerbound[i] = -D2R_25;
		vars_upperbound[i] = D2R_25;
	}
	// Acceleration/decceleration upper and lower limits.
	for (i = A_START; i < n_vars; ++i) {
		vars_lowerbound[i] = -ACCEL_BOUND;
		vars_upperbound[i] = ACCEL_BOUND;
	}
	// Lower and upper limits for the constraints
	// Should be 0 besides initial state.
	Dvector constraints_lowerbound(n_constraints);
	Dvector constraints_upperbound(n_constraints);
	for (i = 0; i < n_constraints; ++i) {
		constraints_lowerbound[i] = 0;
		constraints_upperbound[i] = 0;
	}
	constraints_lowerbound[X_START] = x;
	constraints_lowerbound[Y_START] = y;
	constraints_lowerbound[PSI_START] = psi;
	constraints_lowerbound[V_START] = v;
	constraints_lowerbound[CTE_START] = cte;
	constraints_lowerbound[EPSI_START] = epsi;
	
	constraints_upperbound[X_START] = x;
	constraints_upperbound[Y_START] = y;
	constraints_upperbound[PSI_START] = psi;
	constraints_upperbound[V_START] = v;
	constraints_upperbound[CTE_START] = cte;
	constraints_upperbound[EPSI_START] = epsi;
	// object that computes objective and constraints
	FG_eval fg_eval(coeffs);
	//
	// NOTE: You don't have to worry about these options
	//
	// options for IPOPT solver
	std::string options;
	// Uncomment this if you'd like more print information
	options += "Integer print_level  0\n";
	// NOTE: Setting sparse to true allows the solver to take advantage
	// of sparse routines, this makes the computation MUCH FASTER. If you
	// can uncomment 1 of these and see if it makes a difference or not but
	// if you uncomment both the computation time should go up in orders of
	// magnitude.
	options += "Sparse  true        forward\n";
	options += "Sparse  true        reverse\n";
	// NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
	// Change this as you see fit.
	options += "Numeric max_cpu_time          0.5\n";
	// place to return solution
	CppAD::ipopt::solve_result<Dvector> solution;
	// solve the problem
	CppAD::ipopt::solve<Dvector, FG_eval>(
	  options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
	  constraints_upperbound, fg_eval, solution);
	// Check some of the solution values
	ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
	// Cost
	//auto cost = solution.obj_value;
	//std::cout << "Cost " << cost << std::endl;
	waypoints_x = {};
	waypoints_y = {};
	for (i = 0; i < N - 1; ++i) {
		waypoints_x.push_back(solution.x[X_START + i]);
		waypoints_y.push_back(solution.x[Y_START + i]);
	}
	// TODO: Return the first actuator values. The variables can be accessed with
	// `solution.x[i]`.
	//
	// {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
	// creates a 2 element double vector.	
	// We just need the Steering wheel angle (delta) and the throttle (a).
	return {solution.x[DELTA_START], solution.x[A_START]};
}
