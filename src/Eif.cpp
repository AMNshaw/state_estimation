#include "Eif.h"

Eif::Eif(int x_size, int z_size)
{

	state_size = x_size;
	measurement_size = z_size;

	X_T.setZero(state_size);
	X_T_hat.setZero(state_size);
	E.resize(6);
	E << 0, 0, 1, 0, 0, 0;
	xi.setZero(state_size);
	xi_hat.setZero(state_size);

	t_w2b.resize(3);
	X_B.resize(6);

	F.setIdentity(state_size, state_size);

	Omega = (1e-3*Eigen::MatrixXd::Identity(x_size, x_size)).inverse();
	Q = 5e-4*Eigen::MatrixXd::Identity(state_size, state_size);
	R = 5e-7*Eigen::MatrixXd::Identity(measurement_size, measurement_size);

	C = Eigen::MatrixXd::Zero(measurement_size, state_size);
	C << 1, 0, 0, 0, 0, 0,
		 0, 1, 0, 0, 0, 0,
		 0, 0, 1, 0, 0, 0;

	Intrinsic.setZero(measurement_size, measurement_size);
	Intrinsic << 565.6008952774197, 0.0, 320.5,
				 0.0, 565.6008952774197, 240.5,
				 0.0, 0.0, 1.0;

	t_b2c.resize(3);
	t_b2c<< 0.1, 0, 0; 
	R_b2c.setZero(3, 3);

	R_b2c<< 0, -1, 0,
			0, 0, -1,
			1, 0, 0;

}
Eif::~Eif(){}

void Eif::set_process_noise(Eigen::MatrixXd matrix){Q = matrix;}
void Eif::set_measurement_noise(Eigen::MatrixXd matrix){R = matrix;}

void Eif::setSelfState(geometry_msgs::PoseStamped P, geometry_msgs::TwistStamped V)
{
	t_w2b << P.pose.position.x, P.pose.position.y, P.pose.position.z;

	Eigen::Quaterniond q(P.pose.orientation.w, P.pose.orientation.x, P.pose.orientation.y, P.pose.orientation.z);
	R_w2b = q.toRotationMatrix().inverse();

	X_B << P.pose.position.x, P.pose.position.y, P.pose.position.z, V.twist.linear.x, V.twist.linear.y, V.twist.linear.z;

}

void Eif::predict(double dt)
{
	F(0, 3) = dt;
	F(1, 4) = dt;
	F(2, 5) = dt;

	E_hat = F*E;
	//X_T_hat = E_hat;
	Omega_hat = (F*Omega.inverse()*F.transpose() + Q).inverse();
	xi_hat = Omega_hat*E_hat;
}

void Eif::correct(Eigen::VectorXd z)
{
	Eigen::VectorXd E_cam = R_b2c*((R_w2b*C*E_hat))+ t_b2c;
	Intrinsic(2, 2) = E_cam(2);

	h_X = Intrinsic*E_cam/E_cam(2);
	H = Intrinsic*R_b2c*R_w2b*C/E_cam(2);
	S = H.transpose()*R.inverse()*H;

	Omega = Omega_hat + S;
	xi = xi_hat + H.transpose()*R.inverse()*(z - h_X + H*E_hat);
	E = Omega.inverse()*xi;

	X_T = X_B + E;

}

void Eif::compare(Eigen::VectorXd groundTruth, Eigen::VectorXd measurement)
{
	Eigen::VectorXd E = groundTruth - X_T;

	double RMS_p = sqrt(pow(E(0), 2) + pow(E(1), 2) + pow(E(2), 2));
	double RMS_v = sqrt(pow(E(3), 2) + pow(E(4), 2) + pow(E(5), 2));

	std::cout << "X_T: \n" << X_T << "\n\n";
	std::cout << "RMS_p: " << RMS_p << "\nRMS_v: " << RMS_v << "\n\n";
} 