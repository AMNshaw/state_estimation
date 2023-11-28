
#include "TEIF.h"

target_EIF::target_EIF(int selfPointer, int MavNum, bool est_target_acc) : EIF(selfPointer, MavNum)
{
	target_state_size = (2+est_target_acc)*3 + mavNum*3;
	target_measurement_size = 6;
	EIF_data_init(target_state_size, target_measurement_size, &T);

	float fx = 565.6008952774197;
	float fy = 565.6008952774197;
	float cx = 320.5;
	float cy = 240.5;

	Intrinsic.setZero(3, 3);
	Intrinsic << fx, 0.0, cx,
				 0.0, fy, cy,
				 0.0, 0.0, 1.0;
}

void target_EIF::setData(MAV_eigen* MAVs, Eigen::Vector3f bBox)
{
	Mavs_curr = MAVs;
	Mavs_curr[self_pointer].r_c = Mavs_curr[self_pointer].r + Mavs_curr[self_pointer].R_w2b.inverse()*t_b2c;
	boundingBox = bBox;
}

target_EIF::~target_EIF(){}

void target_EIF::computePredPairs(double delta_t, EIF_data* Rbs)
{
	float dt = static_cast<float>(delta_t);
	
	///////////////////////////// f: r_q, v_q //////////////////////////////////	
	Eigen::MatrixXf f_rv = Eigen::MatrixXf::Identity(6, 6);
	f_rv.block(0, 3, 3, 3) = Eigen::MatrixXf::Identity(3, 3) * dt;
	T.X_hat.segment(0, 6) = f_rv*T.X.segment(0, 6);

	//////////////////////////// f: rq_c1 ~ rq_ci //////////////////////////////
	Eigen::Matrix3f R_w2c = R_b2c*Mavs_last[self_pointer].R_w2b;

	T.X_hat.segment((self_pointer+2)*3, 3) = T.X.segment((self_pointer+2)*3, 3)  
											+ (T.X.segment(3, 3) - Mavs_last[self_pointer].v
											- 2*R_w2c.inverse()*skew(Mavs_last[self_pointer].omega_c)*R_w2c*T.X.segment((self_pointer+2)*3, 3))*dt;
	for(int i=0; i<mavNum ; i++)
		if(i!=self_pointer)
			T.X_hat.segment((i+2)*3, 3) = T.X_hat.segment((self_pointer+2)*3, 3) - (Rbs[i].X.segment(0, 3) - Mavs_curr[self_pointer].r_c);

	///////////////////////////// F: r_q, v_q //////////////////////////////////
	T.F.block(0, 0, 6, 6) = Eigen::MatrixXf::Identity(6, 6);
	T.F.block(0, 3, 3, 3) = Eigen::MatrixXf::Identity(3, 3) * dt;
	
	///////////////////////////// F: rq_c1 ~ rq_c3 //////////////////////////////////
	Eigen::MatrixXf dM = Eigen::MatrixXf::Identity(3, 3) - 2*R_w2c.inverse()*skew(Mavs_last[self_pointer].omega_c)*R_w2c*dt;
	for(int i = 0; i < mavNum; i++)
	{
		T.F.block((i+2)*3, 3, 3, 3) = Eigen::MatrixXf::Identity(3, 3);
		T.F.block((i+2)*3, (self_pointer+2)*3, 3, 3) = dM;
	}

	///////////////////////////// Xi_hat, Omega_hat ////////////////////////////////
	T.Omega_hat = (T.F*T.Omega.inverse()*T.F.transpose() + Q).inverse();
	T.xi_hat = T.Omega_hat*T.X_hat;
}
void target_EIF::computeCorrPairs(){}

