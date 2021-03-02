// Header file with the declaration of the trajectory planning function and its auxiliaries

#ifndef TRAJ
#define TRAJ


// Sin and tan functions
#include <Eigen/Dense>
#include <vector>


using namespace std;

// Sign function
int sign (double a);


// Trajectory planner function
vector<double> trajectoryPlanner (double time, 
				   double x_0, 
				   double v_0, 
				   int v_max, 
				   int a_max, 
				   double x_goal);

#endif


