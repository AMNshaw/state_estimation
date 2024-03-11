#include "CommonFuncs.h"

state_estimation::Plot compare(MAV_eigen GT, Eigen::VectorXd est)
{
	Eigen::Vector3d E_p = GT.r - est.segment(0, 3);
	Eigen::Vector3d E_v = GT.v - est.segment(3, 3);
	state_estimation::Plot Plot_data;
	
	std::cout << "State: \n" << est << "\n\n";
	std::cout << "RMS_p: " << E_p.norm() << "\nRMS_v: " << E_v.norm() << "\n\n";
	Plot_data.header.stamp = ros::Time::now();

	Plot_data.GT_pose.position.x = GT.r(0);
	Plot_data.GT_pose.position.y = GT.r(1);
	Plot_data.GT_pose.position.z = GT.r(2);
	Plot_data.GT_twist.linear.x = GT.v(0);
	Plot_data.GT_twist.linear.y = GT.v(1);
	Plot_data.GT_twist.linear.z = GT.v(2);

	Plot_data.est_pose.position.x = est(0);
	Plot_data.est_pose.position.y = est(1);
	Plot_data.est_pose.position.z = est(2);
	Plot_data.est_twist.linear.x = est(3);
	Plot_data.est_twist.linear.y = est(4);
	Plot_data.est_twist.linear.z = est(5);

	Plot_data.RMSE_p = E_p.norm();
	Plot_data.RMSE_v = E_v.norm();
	return Plot_data;
}

void state2MavEigen(Eigen::VectorXd state, MAV_eigen& ME)
{
	ME.r = state.segment(0, 3);
	if(state.size() >= 6)
		ME.v = state.segment(3, 3);
}

