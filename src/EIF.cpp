#include "EIF.h"

EIF::EIF(int x_size, int z_size, bool Consensus)
{
	state_size = x_size;
	measurement_size = z_size;
	consensus = Consensus;

	X_t.setZero(state_size);
	X_t(2) = 1.0;
	E.resize(state_size);
	xi.setZero(state_size);
	xi_hat.setZero(state_size);

	t_w2b.resize(3);
	X_b.resize(state_size);
	X_b_last.setZero(state_size);

	F.setIdentity(state_size, state_size);
	pre_measurement.setZero(measurement_size);
	pre_Omega= 1e-3*Eigen::MatrixXd::Identity(x_size, x_size);

	Omega = 1e-3*Eigen::MatrixXd::Identity(x_size, x_size);
	Q = 3e-4*Eigen::MatrixXd::Identity(state_size, state_size);
	R = 7e-4*Eigen::MatrixXd::Identity(measurement_size, measurement_size);

	C = Eigen::MatrixXd::Zero(measurement_size, state_size);
	C.block(0, 0, measurement_size, measurement_size) = Eigen::MatrixXd::Identity(measurement_size, measurement_size);


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

	X_b(0) = P.pose.position.x;
	X_b(1) = P.pose.position.y;
	X_b(2) = P.pose.position.z;
	X_b(3) = V.twist.linear.x;
	X_b(4) = V.twist.linear.y; 
	X_b(5) = V.twist.linear.z;
}

void EIF::setSelfState(geometry_msgs::PoseStamped P, geometry_msgs::TwistStamped V, Eigen::VectorXd A)
{
	setSelfState(P, V);
	X_b(6) = A(0);
	X_b(7) = A(1);
	X_b(8) = A(2);
}

void EIF::computePredPairs(double dt)
{
	E = X_t - X_b_last;
	F.block(0, 3, 3, 3) = dt*Eigen::MatrixXd::Identity(3, 3);
	if(state_size == 9)
	{
		F.block(0, 6, 3, 3) = 0.5*pow(dt, 2)*Eigen::MatrixXd::Identity(3, 3);
		F.block(3, 6, 3, 3) = dt*Eigen::MatrixXd::Identity(3, 3);
	}

	E_hat = F*E;
	Omega_hat = (F*Omega.inverse()*F.transpose() + Q).inverse();
	X_t_hat = E_hat + X_b;
	xi_hat = Omega_hat*X_t_hat;
}

void EIF::computeCorrPairs(Eigen::VectorXd z)
{
	Eigen::VectorXd E_cam = R_b2c*((R_w2b*C*E_hat))+ t_b2c;
	Intrinsic(2, 2) = E_cam(2);

	h = Intrinsic*E_cam/E_cam(2);
	H = Intrinsic*R_b2c*R_w2b*C/E_cam(2);
	s = H.transpose()*R.inverse()*H;
	y = H.transpose()*R.inverse()*(z - h + H*X_t_hat);
	X_b_last = X_b;
}

void EIF::correct(Eigen::VectorXd z)
{
	computeCorrPairs(z);
	Omega = (Omega_hat + s);
	xi = xi_hat + y;
	X_t = Omega.inverse()*xi;
}

void EIF::predict_fused(double dt, Eigen::MatrixXd fusedOmega, Eigen::VectorXd fusedXi, bool flag)
{
	if(flag)
	{
		Omega = fusedOmega;
		xi = fusedXi;
		X_t = Omega.inverse()*xi;
		E = X_t - X_b_last;
	}
	computePredPairs(dt);
}

void EIF::process(double dt, Eigen::VectorXd z, Eigen::MatrixXd fusedOmega, Eigen::VectorXd fusedXi, bool flag)
{
	if(consensus == false)
	{
		computePredPairs(dt);
		if(z != pre_measurement)
			correct(z);
		pre_measurement = z;
	}
	else
	{ 
		predict_fused(dt, fusedOmega, fusedXi, flag);
		if(z != pre_measurement)
			computeCorrPairs(z);
		pre_measurement = z;
	}

}

Eigen::VectorXd EIF::getTargetState(){return X_t;}

void EIF::getPredictionPairs(Eigen::MatrixXd* infoMat, Eigen::VectorXd* infoVec)
{
	*infoMat = Omega_hat;
	*infoVec = xi_hat;
}
void EIF::getCorrectionPairs(Eigen::MatrixXd* infoMat, Eigen::VectorXd* infoVec)
{
	*infoMat = s;
	*infoVec = y;
}