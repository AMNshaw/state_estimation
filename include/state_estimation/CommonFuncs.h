#include "EIF.h"
#include "Mav.h"
#include <state_estimation/EIFpairStamped.h>
#include <state_estimation/RMSE.h>
#include <state_estimation/Plot.h>


EIF_data eifMsg2Eigen(state_estimation::EIFpairStamped eifMsg);
state_estimation::EIFpairStamped eigen2EifMsg(EIF_data est_object, int self_id);
MAV_eigen mavMsg2Eigen(MAV Mav);
std::vector<MAV_eigen> mavsMsg2Eigen(MAV* Mavs, int mavNum);
std::vector<MAV_eigen> mavsMsg2Eigen(std::vector<MAV> Mavs);
state_estimation::Plot compare(MAV_eigen GT, Eigen::VectorXf est);
state_estimation::RMSE compare(MAV_eigen GT, MAV_eigen est);
state_estimation::RMSE compare(Eigen::VectorXf GT, Eigen::VectorXf est);
void state2MavEigen(Eigen::VectorXf state, MAV_eigen& ME);