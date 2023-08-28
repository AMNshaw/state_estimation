#include "EIF.h"

EIF::EIF(int x_size, int z_size)
{

	state_size = x_size;
	measurement_size = z_size;

	X_t.setZero(state_size);
	E.resize(6);
	E << 0, 0, 1, 0, 0, 0;
	xi.setZero(state_size);
	xi_hat.setZero(state_size);

	t_w2b.resize(3);
	X_b.resize(6);

	F.setIdentity(state_size, state_size);

	P = 1e-3*Eigen::MatrixXd::Identity(x_size, x_size);
	Q = 5e-4*Eigen::MatrixXd::Identity(state_size, state_size);
	R = 5e-7*Eigen::MatrixXd::Identity(measurement_size, measurement_size);

	C = Eigen::MatrixXd::Zero(measurement_size, state_size);
	C << 1, 0, 0, 0, 0, 0,
		 0, 1, 0, 0, 0, 0,
		 0, 0, 1, 0, 0, 0;

	Intrinsic.setZero(3, 3);
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
EIF::~EIF(){}

void EIF::set_process_noise(Eigen::MatrixXd matrix){Q = matrix;}
void EIF::set_measurement_noise(Eigen::MatrixXd matrix){R = matrix;}

void EIF::setSelfState(geometry_msgs::PoseStamped P, geometry_msgs::TwistStamped V)
{
	t_w2b << P.pose.position.x, P.pose.position.y, P.pose.position.z;
	Eigen::Quaterniond q(P.pose.orientation.w, P.pose.orientation.x, P.pose.orientation.y, P.pose.orientation.z);
	R_w2b = q.toRotationMatrix().inverse();

	X_b << P.pose.position.x, P.pose.position.y, P.pose.position.z, V.twist.linear.x, V.twist.linear.y, V.twist.linear.z;

}

void EIF::predict(double dt)
{
	F(0, 3) = dt;
	F(1, 4) = dt;
	F(2, 5) = dt;

	E_hat = F*E;
	P_hat = F*P*F.transpose() + Q;
	xi_hat = P_hat.inverse()*E_hat;
}

void EIF::correct(Eigen::VectorXd z)
{
	Eigen::VectorXd E_cam = R_b2c*((R_w2b*C*E_hat))+ t_b2c;
	Intrinsic(2, 2) = E_cam(2);

	h = Intrinsic*E_cam/E_cam(2);
	H = Intrinsic*R_b2c*R_w2b*C/E_cam(2);
	s = H.transpose()*R.inverse()*H;
	y = H.transpose()*R.inverse()*(z - h + H*E_hat);

	P = (P_hat.inverse() + s).inverse();
	xi = xi_hat + y;
	E = P*xi;

	X_t = X_b + E;
}

void EIF::compare(Eigen::VectorXd groundTruth)
{
	Eigen::VectorXd E = groundTruth - X_t;
	Eigen::VectorXd E_p(3), E_v(3);
	E_p << E(0), E(1), E(2);
	E_v << E(3), E(4), E(5);

	std::cout << "X_t: \n" << X_t << "\n\n";
	std::cout << "RMS_p: " << E_p.norm() << "\nRMS_v: " << E_v.norm() << "\n\n";
} 