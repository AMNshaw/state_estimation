#include "TEIF.h"

target_EIF::target_EIF(int state_size)
{
	target_state_size = state_size;
	target_measurement_size = 3;
	EIF_data_init(target_state_size, target_measurement_size, &T);
	Q.block(0, 0, 3, 3) = 7e-4*Eigen::MatrixXf::Identity(3, 3);
	Q.block(3, 3, 3, 3) = 7e-2*Eigen::MatrixXf::Identity(3, 3);
	R = 1e-5*Eigen::MatrixXf::Identity(3, 3);

	fx = 565.6008952774197;
	fy = 565.6008952774197;
	cx = 320.5;
	cy = 240.5;

	Mav_curr.v.setZero();
}
target_EIF::~target_EIF(){}

void target_EIF::setData(MAV_eigen MAV, 
						Eigen::Vector3f bBox, 
						EIF_data self_data)
{
	Mav_curr = MAV;
	Mav_curr.r_c = Mav_curr.r + Mav_curr.R_w2b.inverse()*t_b2c;
	boundingBox = bBox;
	self = self_data;
}


void target_EIF::computePredPairs(double delta_t)
{
	float dt = static_cast<float>(delta_t);
	
	///////////////////////////// X, F ////////////////////////////////

	T.F.setIdentity();
	T.F.block(0, 3, 3, 3) = Eigen::Matrix3f::Identity(3, 3)*dt;

	T.P_hat = T.F*T.P*T.F.transpose() + Q;
	T.X_hat = T.F*T.X;
}
void target_EIF::computeCorrPairs()
{
	T.z = boundingBox;

	T.s.setZero();
	T.y.setZero();
	self.s.setZero();
	self.y.setZero();
	if(T.z != T.pre_z)
	{
		Eigen::MatrixXf R_hat, R_bar;
		Eigen::Matrix3f R_w2c = R_b2c*Mav_curr.R_w2b;
		Eigen::Vector3f r_qc_c = R_w2c*(T.X_hat.segment(0, 3) - Mav_curr.r_c);

		X = r_qc_c(0)/r_qc_c(2);
		Y = r_qc_c(1)/r_qc_c(2);
		Z = r_qc_c(2);

		T.h(0) = fx*X + cx;
		T.h(1) = fy*Y + cy;
		T.h(2) = Z;
		self.z = T.z;
		self.h = T.h;

		T.H(0, 0) = (fx/Z)*(R_w2c(0, 0) - R_w2c(2, 0)*X);
		T.H(0, 1) = (fx/Z)*(R_w2c(0, 1) - R_w2c(2, 1)*X);
		T.H(0, 2) = (fx/Z)*(R_w2c(0, 2) - R_w2c(2, 2)*X);
		T.H(1, 0) = (fy/Z)*(R_w2c(1, 0) - R_w2c(2, 0)*Y);
		T.H(1, 1) = (fy/Z)*(R_w2c(1, 1) - R_w2c(2, 1)*Y);
		T.H(1, 2) = (fy/Z)*(R_w2c(1, 2) - R_w2c(2, 2)*Y);
		T.H(2, 0) = R_w2c(2, 0);
		T.H(2, 1) = R_w2c(2, 1);
		T.H(2, 2) = R_w2c(2, 2);

		self.H = -T.H;

		R_hat = R + self.H*self.P_hat*self.H.transpose();
		R_bar = R + T.H*T.P_hat*T.H.transpose();

		T.s = T.H.transpose()*R_hat.inverse()*T.H;
		T.y = T.H.transpose()*R_hat.inverse()*(T.z - T.h + T.H*T.X_hat);

		self.s = self.H.transpose()*R_bar.inverse()*self.H;
		self.y = self.H.transpose()*R_bar.inverse()*(self.z - self.h + self.H*self.X_hat);
	}
	T.P = (T.P_hat.inverse() + T.s).inverse();
	T.X = T.P*(T.P_hat.inverse()*T.X_hat + T.y);
	T.pre_z = T.z;
}

void target_EIF::setFusionPairs(Eigen::MatrixXf fusedP, Eigen::VectorXf fusedX)
{
    T.P = fusedP;
    T.X = fusedX;
}

EIF_data target_EIF::getTgtData(){return T;}
EIF_data target_EIF::getSelfData(){return self;}