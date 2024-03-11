#include "EIF.h"
#include "Mav.h"
#include <state_estimation/EIFpairStamped.h>
#include <state_estimation/Plot.h>

state_estimation::Plot compare(MAV_eigen GT, Eigen::VectorXd est);
void state2MavEigen(Eigen::VectorXd state, MAV_eigen& ME);