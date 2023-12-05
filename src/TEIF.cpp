
#include "TEIF.h"

target_EIF::target_EIF(int selfPointer, int MavNum, bool est_target_acc) : EIF(selfPointer, MavNum)
{
	target_state_size = (2+est_target_acc)*3 + mavNum*3;
	target_measurement_size = 6;
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
	
}

void target_EIF::setData(MAV_eigen* MAVs, Eigen::Vector3f bBox)
{
	Mavs_last = Mavs_curr;
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

	// T.X_hat.segment((self_pointer+2)*3, 3) = T.X.segment((self_pointer+2)*3, 3)  
	// + (T.X.segment(3, 3) - Mavs_last[self_pointer].v
	// - 2*R_w2c.inverse()*skew(Mavs_last[self_pointer].omega_c)*R_w2c*T.X.segment((self_pointer+2)*3, 3))*dt;

	T.X_hat.segment((self_pointer+2)*3, 3) = T.X.segment((self_pointer+2)*3, 3) + (T.X.segment(3, 3) - Mavs_last[self_pointer].v)*dt;

	for(int i=0; i<mavNum ; i++)
		if(i!=self_pointer)
			T.X_hat.segment((i+2)*3, 3) = T.X_hat.segment((self_pointer+2)*3, 3) - (Rbs[i].X.segment(0, 3) - Mavs_curr[self_pointer].r_c);

	///////////////////////////// F: r_q, v_q //////////////////////////////////
	T.F.block(0, 0, 6, 6) = Eigen::MatrixXf::Identity(6, 6);
	T.F.block(0, 3, 3, 3) = Eigen::MatrixXf::Identity(3, 3) * dt;
	
	///////////////////////////// F: rq_c1 ~ rq_c3 //////////////////////////////////
	//Eigen::MatrixXf dM = Eigen::MatrixXf::Identity(3, 3) - 2*R_w2c.inverse()*skew(Mavs_last[self_pointer].omega_c)*R_w2c*dt;
	Eigen::MatrixXf dM = Eigen::MatrixXf::Identity(3, 3);

	for(int i = 0; i < mavNum; i++)
	{
		T.F.block((i+2)*3, 3, 3, 3) = Eigen::MatrixXf::Identity(3, 3)*dt;
		T.F.block((i+2)*3, (self_pointer+2)*3, 3, 3) = dM;
	}

	///////////////////////////// Xi_hat, Omega_hat ////////////////////////////////
	T.Omega_hat = (T.F*T.Omega.inverse()*T.F.transpose() + Q).inverse();
	T.xi_hat = T.Omega_hat*T.X_hat;

	
}
void target_EIF::computeCorrPairs()
{
	Eigen::Matrix3f R_w2c = R_b2c*Mavs_curr[self_pointer].R_w2b;
	Eigen::Vector3f r_q_ci_c = R_w2c*T.X_hat.segment((self_pointer+2)*3, 3);
	X = r_q_ci_c(0)/r_q_ci_c(2);
	Y = r_q_ci_c(1)/r_q_ci_c(2);
	Z = r_q_ci_c(2);

	Intrinsic(2, 2) = boundingBox(2);
	T.z.segment(0, 3) = boundingBox;
	T.z.segment(3, 3) = Mavs_curr[self_pointer].R_w2b.inverse()*(R_b2c.inverse()*boundingBox(2)*Intrinsic.inverse()*boundingBox - t_b2c);

	if(T.z.segment(0, 3) == T.pre_z.segment(0, 3))
	{
		T.s.setZero();
		T.y.setZero();
	}
	else
	{
		Eigen::Matrix3f H_uvz;
		Intrinsic(2, 2) = r_q_ci_c(2);
		T.h.segment(0, 3) = Intrinsic*r_q_ci_c/r_q_ci_c(2);
		T.h.segment(3, 3) = T.X_hat.segment((self_pointer+2)*3, 3);

		H_uvz(0, 0) = (fx/Z)*(R_w2c(0, 0) - R_w2c(2, 0)*X);
		H_uvz(0, 1) = (fx/Z)*(R_w2c(0, 1) - R_w2c(2, 1)*X);
		H_uvz(0, 2) = (fx/Z)*(R_w2c(0, 2) - R_w2c(2, 2)*X);
		H_uvz(1, 0) = (fy/Z)*(R_w2c(1, 0) - R_w2c(2, 0)*Y);
		H_uvz(1, 1) = (fy/Z)*(R_w2c(1, 1) - R_w2c(2, 1)*Y);
		H_uvz(1, 2) = (fy/Z)*(R_w2c(1, 2) - R_w2c(2, 2)*Y);
		H_uvz(2, 0) = R_w2c(2, 0);
		H_uvz(2, 1) = R_w2c(2, 1);
		H_uvz(2, 2) = R_w2c(2, 2);

		for(int i=0; i<mavNum; i++)
			T.H.block(0, (self_pointer+2 + i)*3, 3, 3) = H_uvz;

		T.H.block(3, (self_pointer+2)*3, 3, 3) = Eigen::Matrix3f::Identity();

		T.s = T.H.transpose()*R.inverse()*T.H;
		T.y = T.H.transpose()*R.inverse()*(T.z - T.h + T.H*T.X_hat);
		//std::cout << "T.X_hat:\n" << T.X_hat << std::endl;
		//std::cout << "h:\n" << T.h << std::endl;
		//std::cout << "z:\n" << T.z << std::endl;
		//std::cout << "T.H*T.X_hat:\n" << T.H*T.X_hat << std::endl;
		//std::cout << "H:\n" << T.H << std::endl;
		//std::cout << "r_q_ci_c:\n" << r_q_ci_c << std::endl;
		//std::cout << "H:\n" << T.H << std::endl;
	}
	T.Omega = T.Omega_hat + T.s;
	T.xi = T.xi_hat + T.y;
	T.X = T.Omega.inverse()*T.xi;
	std::cout << "Omega:\n" << T.Omega << std::endl;

	T.pre_z = T.z; 
}

