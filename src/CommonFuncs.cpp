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