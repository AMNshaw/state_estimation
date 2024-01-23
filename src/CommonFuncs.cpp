#include "CommonFuncs.h"

state_estimation::EIFpairStamped eigen2EifMsg(EIF_data est_object, int self_id)
{
	state_estimation::EIFpairStamped EIFpairs;
	std::vector<float> P_hat_vec(est_object.P_hat.data(), est_object.P_hat.data() + est_object.P_hat.size());
	std::vector<float> X_hat_vec(est_object.X_hat.data(), est_object.X_hat.data() + est_object.X_hat.size());
	std::vector<float> s_vec(est_object.s.data(), est_object.s.data() + est_object.s.size());
	std::vector<float> y_vec(est_object.y.data(), est_object.y.data() + est_object.y.size());

	EIFpairs.P_hat = P_hat_vec;
	EIFpairs.X_hat = X_hat_vec;
	EIFpairs.s = s_vec;
	EIFpairs.y = y_vec;
	EIFpairs.header.stamp.sec = ros::Time::now().sec;
	EIFpairs.header.stamp.nsec = ros::Time::now().nsec;
	EIFpairs.id = self_id;
	return EIFpairs;
}

EIF_data eifMsg2Eigen(state_estimation::EIFpairStamped eifMsg)
{
	EIF_data est_object;
	int state_size = eifMsg.X_hat.size();
	est_object.P_hat = Eigen::Map<Eigen::MatrixXf>(eifMsg.P_hat.data(), state_size, state_size);
	est_object.X_hat = Eigen::Map<Eigen::VectorXf>(eifMsg.X_hat.data(), state_size);
	est_object.s = Eigen::Map<Eigen::MatrixXf>(eifMsg.s.data(), state_size, state_size);
	est_object.y = Eigen::Map<Eigen::VectorXf>(eifMsg.y.data(), state_size);
	est_object.ID = eifMsg.id;

	return est_object;
}

std::vector<MAV_eigen> mavsMsg2Eigen(MAV* Mavs, int mavNum)
{
    std::vector<MAV_eigen> Mavs_eigen(mavNum);
	for(int i = 0; i < mavNum; i++)
	{
		Mavs_eigen[i].r(0) = Mavs[i].getPose().pose.position.x;
		Mavs_eigen[i].r(1) = Mavs[i].getPose().pose.position.y;
		Mavs_eigen[i].r(2) = Mavs[i].getPose().pose.position.z;
		Mavs_eigen[i].v(0) = Mavs[i].getVel().twist.linear.x;
		Mavs_eigen[i].v(1) = Mavs[i].getVel().twist.linear.y;
		Mavs_eigen[i].v(2) = Mavs[i].getVel().twist.linear.z;
		Mavs_eigen[i].a(0) = Mavs[i].getAcc().x;
		Mavs_eigen[i].a(1) = Mavs[i].getAcc().y;
		Mavs_eigen[i].a(2) = Mavs[i].getAcc().z;
	}
	Mavs_eigen[MAV::self_index].omega_c(0) = Mavs[MAV::self_index].getVel().twist.angular.x;
	Mavs_eigen[MAV::self_index].omega_c(1) = Mavs[MAV::self_index].getVel().twist.angular.y;
	Mavs_eigen[MAV::self_index].omega_c(2) = Mavs[MAV::self_index].getVel().twist.angular.z;
	Mavs_eigen[MAV::self_index].R_w2b = Eigen::Quaternionf(
		Mavs[MAV::self_index].getPose().pose.orientation.w,
		Mavs[MAV::self_index].getPose().pose.orientation.x,
		Mavs[MAV::self_index].getPose().pose.orientation.y,
		Mavs[MAV::self_index].getPose().pose.orientation.z
	).toRotationMatrix().inverse();

	Mavs[MAV::self_index].getRPY(Mavs_eigen[MAV::self_index].RPY(0),
								Mavs_eigen[MAV::self_index].RPY(1),
								Mavs_eigen[MAV::self_index].RPY(2));

    return Mavs_eigen;
}

MAV_eigen mavMsg2Eigen(MAV Mav)
{
	MAV_eigen Mav_eigen;

	Mav_eigen.r(0) = Mav.getPose().pose.position.x;
	Mav_eigen.r(1) = Mav.getPose().pose.position.y;
	Mav_eigen.r(2) = Mav.getPose().pose.position.z;
	Mav_eigen.v(0) = Mav.getVel().twist.linear.x;
	Mav_eigen.v(1) = Mav.getVel().twist.linear.y;
	Mav_eigen.v(2) = Mav.getVel().twist.linear.z;
	Mav_eigen.a(0) = Mav.getAcc().x;
	Mav_eigen.a(1) = Mav.getAcc().y;
	Mav_eigen.a(2) = Mav.getAcc().z;
	
	Mav_eigen.omega_c(0) = Mav.getVel().twist.angular.x;
	Mav_eigen.omega_c(1) = Mav.getVel().twist.angular.y;
	Mav_eigen.omega_c(2) = Mav.getVel().twist.angular.z;
	Mav_eigen.R_w2b = Eigen::Quaternionf(
		Mav.getPose().pose.orientation.w,
		Mav.getPose().pose.orientation.x,
		Mav.getPose().pose.orientation.y,
		Mav.getPose().pose.orientation.z
	).toRotationMatrix().inverse();
	Mav.getRPY(Mav_eigen.RPY(0),
				Mav_eigen.RPY(1),
				Mav_eigen.RPY(2));
	
	return Mav_eigen;
}

std::vector<MAV_eigen> mavsMsg2Eigen(std::vector<MAV> Mavs)
{
    std::vector<MAV_eigen> Mavs_eigen(Mavs.size());
	for(int i=0; i< Mavs.size(); i++)
		Mavs_eigen[i] = mavMsg2Eigen(Mavs[i]);

    return Mavs_eigen;
}

state_estimation::RMSE compare(MAV_eigen GT, Eigen::VectorXf est)
{
	Eigen::Vector3f E_p = GT.r - est.segment(0, 3);
	Eigen::Vector3f E_v = GT.v - est.segment(3, 3);
	state_estimation::RMSE RMSE_data;
	
	std::cout << "State: \n" << est << "\n\n";
	std::cout << "RMS_p: " << E_p.norm() << "\nRMS_v: " << E_v.norm() << "\n\n";
	RMSE_data.header.stamp = ros::Time::now();
	RMSE_data.RMSE_p = E_p.norm();
	RMSE_data.RMSE_v = E_v.norm();
	return RMSE_data;
}

state_estimation::RMSE compare(MAV_eigen GT, MAV_eigen est)
{
	Eigen::Vector3f E_p = GT.r - est.r;
	Eigen::Vector3f E_v = GT.v - est.v;
	state_estimation::RMSE RMSE_data;
	
	std::cout << "State: \n" << est.r << "\n" << est.v << "\n" << est.a << "\n\n";
	std::cout << "RMS_p: " << E_p.norm() << "\nRMS_v: " << E_v.norm() << "\n\n";
	RMSE_data.header.stamp = ros::Time::now();
	RMSE_data.RMSE_p = E_p.norm();
	RMSE_data.RMSE_v = E_v.norm();
	return RMSE_data;
}

state_estimation::RMSE compare(Eigen::VectorXf GT, Eigen::VectorXf est)
{
	Eigen::Vector3f E_p = GT.segment(0, 3) - est.segment(0, 3);
	Eigen::Vector3f E_v = GT.segment(3, 3) - est.segment(3, 3);
	state_estimation::RMSE RMSE_data;
	
	std::cout << "State: \n" << est << "\n\n";
	std::cout << "RMS_p: " << E_p.norm() << "\nRMS_v: " << E_v.norm() << "\n\n";
	RMSE_data.header.stamp = ros::Time::now();
	RMSE_data.RMSE_p = E_p.norm();
	RMSE_data.RMSE_v = E_v.norm();
	return RMSE_data;
}

void state2MavEigen(Eigen::VectorXf state, MAV_eigen& ME)
{
	ME.r = state.segment(0, 3);
	if(state.size() >= 6)
		ME.v = state.segment(3, 3);
	if(state.size() >= 9)
		ME.a = state.segment(6, 3);

}