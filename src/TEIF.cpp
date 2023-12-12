#include "TEIF.h"

target_EIF::target_EIF(int selfPointer, int MavNum, bool est_target_acc) : EIF(selfPointer, MavNum)
{
	target_state_size = 9;
	target_measurement_size = 3;
	EIF_data_init(target_state_size, target_measurement_size, &T);

	fx = 565.6008952774197;
	fy = 565.6008952774197;
	cx = 320.5;
	cy = 240.5;

	Intrinsic.setZero(3, 3);
	Intrinsic << fx, 0.0, cx,
				 0.0, fy, cy,
				 0.0, 0.0, 1.0;

	for(int i = 0; i < mavNum; i++)
		Mavs_curr[i].v.setZero();			 
	
	prevXi.setZero(target_state_size);
}
target_EIF::~target_EIF(){}

void target_EIF::setData(MAV_eigen* MAVs, Eigen::Vector3f bBox)
{
	Mavs_last = Mavs_curr;
	Mavs_curr = MAVs;
	Mavs_curr[self_pointer].r_c = Mavs_curr[self_pointer].r + Mavs_curr[self_pointer].R_w2b.inverse()*t_b2c;
	boundingBox = bBox;
}

void target_EIF::setFusedPairs(Eigen::MatrixXf fusedOmega, Eigen::VectorXf fusedXi)
{
	if(prevXi != fusedXi)
	{
		T.Omega = fusedOmega;
		T.X = fusedOmega.inverse()*fusedXi;
	}
	prevXi = fusedXi;
}

EIF_data target_EIF::getTgtData(){return T;}

void target_EIF::computePredPairs(double delta_t, EIF_data* Rbs)
{
	float dt = static_cast<float>(delta_t);
	
	///////////////////////////// X, F ////////////////////////////////

	T.F.setIdentity();
	T.F.block(0, 3, 3, 3) = Eigen::Matrix3f::Identity(3, 3)*dt;
	T.F.block(0, 6, 3, 3) = 0.5*Eigen::Matrix3f::Identity(3, 3)*dt*dt;
	T.F.block(3, 6, 3, 3) = Eigen::Matrix3f::Identity(3, 3)*dt;

	T.X_hat = T.F*T.X;

	///////////////////////////// Xi_hat, Omega_hat ////////////////////////////////
	T.Omega_hat = (T.F*T.Omega.inverse()*T.F.transpose() + Q).inverse();
	T.xi_hat = T.Omega_hat*T.X_hat;
	
}
void target_EIF::computeCorrPairs()
{
	T.z = boundingBox;

	if(T.z == T.pre_z)
	{
		T.s.setZero();
		T.y.setZero();
	}
	else
	{
		Eigen::Matrix3f R_w2c = R_b2c*Mavs_curr[self_pointer].R_w2b;
		Eigen::Vector3f r_qc_c = R_w2c*(T.X_hat.segment(0, 3) - Mavs_curr[self_pointer].r_c);

		X = r_qc_c(0)/r_qc_c(2);
		Y = r_qc_c(1)/r_qc_c(2);
		Z = r_qc_c(2);

		Intrinsic(2, 2) = Z;
		T.h(0) = fx*X + cx;
		T.h(1) = fy*Y + cy;
		T.h(2) = Z;

		T.H(0, 0) = (fx/Z)*(R_w2c(0, 0) - R_w2c(2, 0)*X);
		T.H(0, 1) = (fx/Z)*(R_w2c(0, 1) - R_w2c(2, 1)*X);
		T.H(0, 2) = (fx/Z)*(R_w2c(0, 2) - R_w2c(2, 2)*X);
		T.H(1, 0) = (fy/Z)*(R_w2c(1, 0) - R_w2c(2, 0)*Y);
		T.H(1, 1) = (fy/Z)*(R_w2c(1, 1) - R_w2c(2, 1)*Y);
		T.H(1, 2) = (fy/Z)*(R_w2c(1, 2) - R_w2c(2, 2)*Y);
		T.H(2, 0) = R_w2c(2, 0);
		T.H(2, 1) = R_w2c(2, 1);
		T.H(2, 2) = R_w2c(2, 2);

		T.s = T.H.transpose()*R.inverse()*T.H;
		T.y = T.H.transpose()*R.inverse()*(T.z - T.h + T.H*T.X_hat);
	}
	T.Omega = T.Omega_hat + T.s;
	T.xi = T.xi_hat + T.y;
	T.X = T.Omega.inverse()*T.xi;

	T.pre_z = T.z; 
}

