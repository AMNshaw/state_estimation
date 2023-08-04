#include "Eif.h"

Eif::Eif(int x_size, int z_size)
{
	state_size = x_size;
	measurement_size = z_size;

	X.setZero(state_size);
	X_hat.setZero(state_size);
	xi.setZero(state_size);
	xi_hat.setZero(state_size);
	z.setZero(measurement_size);

	F.setIdentity(state_size, state_size);

	Omega = (1e-3*Eigen::MatrixXd::Identity(x_size, x_size)).inverse();
	Q = 5e-7*Eigen::MatrixXd::Identity(state_size, state_size);
	R = 5e-4*Eigen::MatrixXd::Identity(measurement_size, measurement_size);

	C = Eigen::MatrixXd::Zero(measurement_size, state_size);
	C << Eigen::MatrixXd::Identity(state_size, state_size), Eigen::MatrixXd::Zero(state_size, state_size);
}

void Eif::set_process_noise(Eigen::MatrixXd matrix){Q = matrix;}
void Eif::set_measurement_noise(Eigen::MatrixXd matrix){R = matrix;}

void Eif::predict(double dt)
{
	F(0, 3) = dt;
	F(1, 4) = dt;
	F(2, 5) = dt;

	X_hat = F*X;
	Omega_hat = (F*Omega.inverse()*F.transpose() + Q).inverse();
	xi_hat = Omega_hat*X_hat;
}

void Eif::correct(Eigen::VectorXd measurement)
{
	z = measurement;
	q.normalize();
	Eigen::MatrixXd R_cam = q.toRotationMatrix();

	H = Intrinsic*R_cam*C/depth;
	S = H.transpose()*R.inverse()*H;


}
