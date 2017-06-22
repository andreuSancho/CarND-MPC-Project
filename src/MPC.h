#ifndef MPC_H
#define MPC_H

#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  std::vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  /**
  * It returns the results for the X component.
  * @return a std::vector with the results of the fitting for the X component.
  */
  std::vector<double> getResultsX() const { return waypoints_x; }
  /**
  * It returns the results for the Y component.
  * @return a std::vector with the results of the fitting for the Y component.
  */
  std::vector<double> getResultsY() const { return waypoints_y; }
  
  private:
	// Store results for avoiding re-computations.
	std::vector<double> waypoints_x;
	std::vector<double> waypoints_y;
};

#endif /* MPC_H */
