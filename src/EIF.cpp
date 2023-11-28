#include "EIF.h"

EIF::EIF(int selfPointer, int MavNum)
{
	self_pointer = selfPointer;
	mavNum = MavNum;
	Mavs_curr = new MAV_eigen[mavNum];
	Mavs_last = new MAV_eigen[mavNum];
	for(int i = 0; i < mavNum; i++)
		Mavs_last[i].R_w2b = Mavs_curr[i].R_w2b = Eigen::MatrixXf::Identity(3, 3);

	t_b2c.resize(3);
	t_b2c<< 0.1, 0, 0; 

	R_b2c.setZero(3, 3);
	R_b2c<< 0, -1, 0,
			0, 0, -1,
			1, 0, 0;

}

EIF::~EIF()
{
	delete Mavs_curr;
	delete Mavs_last;
}

void EIF::EIF_data_init(int x_size, int z_size, EIF_data* est_object)
{
	est_object->X.setZero(x_size); est_object->X(2) = 1.0; est_object->X(3) = 1.0;
	est_object->X_hat.setZero(x_size);
	est_object->xi.setZero(x_size);
	est_object->xi_hat.setZero(x_size);
	est_object->z.setZero(z_size);
	est_object->h.setZero(z_size);
	est_object->pre_z.setZero(z_size);

	est_object->F.setZero(x_size, x_size);
	est_object->H.setZero(z_size, x_size);
	est_object->Omega = 1e-3*Eigen::MatrixXf::Identity(x_size, x_size);
	
	Q = 3e-4*Eigen::MatrixXf::Identity(x_size, x_size);
	R = 3e-4*Eigen::MatrixXf::Identity(z_size, z_size);
}

void EIF::set_process_noise(Eigen::MatrixXf matrix){Q = matrix;}
void EIF::set_measurement_noise(Eigen::MatrixXf matrix){R = matrix;}

Eigen::Matrix3f EIF::skew(Eigen::Vector3f vec)
{
	Eigen::Matrix3f vec_skew;
	vec_skew << 0, -vec(0), vec(1),
				vec(0), 0, -vec(2),
				-vec(1), vec(2), 0;
	return vec_skew;
}

void EIF::setData(){std::cout << "Nothing here" << std::endl;}

void EIF::computePredPairs()
{
	std::cout << "Virtual" << std::endl;
	
}

void EIF::computeCorrPairs()
{
	Mavs_last = Mavs_curr;
}

void EIF::correct(Eigen::MatrixXf z)
{
	// computeCorrPairs(z);
	// Omega = (Omega_hat + s);
	// xi = xi_hat + y;
	// X_t = Omega.inverse()*xi;
}

void EIF::predict_fused(double dt, Eigen::MatrixXf fusedOmega, Eigen::MatrixXf fusedXi, bool flag)
{
	// if(flag)
	// {
	// 	Omega = fusedOmega;
	// 	xi = fusedXi;
	// 	X_t = Omega.inverse()*xi;
	// 	E = X_t - X_b_last;
	// }
	// computePredPairs(dt);
}

void EIF::getPredictionPairs(Eigen::MatrixXf* infoMat, Eigen::MatrixXf* infoVec)
{
}
void EIF::getCorrectionPairs(Eigen::MatrixXf* infoMat, Eigen::MatrixXf* infoVec)
{
}
/////////////////////////////////////////////////////////// target EIF ///////////////////////////////////////////////////////////
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

/////////////////////////////////////////////////////////// robots EIF ///////////////////////////////////////////////////////////
robots_EIF::robots_EIF(int selfPointer, int MavNum) : EIF(selfPointer, MavNum)
{
	robots_state_size = 6;
	robots_measurement_size = 3;

	Rbs = new EIF_data[mavNum];
	for(int i=0; i < mavNum; i++)
		if(i != self_pointer)
	 		EIF_data_init(robots_state_size, robots_measurement_size, &Rbs[i]);	
}
robots_EIF::~robots_EIF()
{
	delete Rbs;
}

void robots_EIF::setData(MAV_eigen* MAVs)
{
	Mavs_last = Mavs_curr;
	Mavs_curr = MAVs;
	Mavs_curr[self_pointer].r_c = Mavs_curr[self_pointer].r + Mavs_curr[self_pointer].R_w2b.inverse()*t_b2c;
}

EIF_data* robots_EIF::getRbsData(){return Rbs;}

void robots_EIF::computePredPairs(double delta_t)
{
	float dt = static_cast<float>(delta_t);
	Eigen::Matrix3f R_w2c = R_b2c*Mavs_last[self_pointer].R_w2b;

	for(int i=0; i<mavNum ; i++)
	{
		if(i!=self_pointer)
		{
			Rbs[i].X_hat.segment(0, 3) = Mavs_curr[self_pointer].r_c
			+ (Rbs[i].X.segment(0, 3) - Mavs_last[self_pointer].r_c)
			+ ((Rbs[i].X.segment(3, 3) - Mavs_last[self_pointer].v 
			- 2*R_w2c.inverse()*skew(Mavs_last[self_pointer].omega_c)*R_w2c*(Rbs[i].X.segment(0, 3) - Mavs_last[self_pointer].r_c)))*dt;
			Rbs[i].X_hat.segment(3, 3) = Rbs[i].X.segment(3, 3);

			Rbs[i].F.block(0, 0, 3, 3) = Eigen::Matrix3f::Identity(3, 3) - 2*R_w2c.inverse()*skew(Mavs_last[self_pointer].omega_c)*R_w2c*dt;
			Rbs[i].F.block(0, 0, 3, 3) = Eigen::Matrix3f::Identity(3, 3) + Eigen::Matrix3f::Identity(3, 3)*dt;
			Rbs[i].F.block(0, 3, 3, 3) = Eigen::Matrix3f::Identity(3, 3)*dt;
			Rbs[i].F.block(3, 3, 3, 3) = Eigen::Matrix3f::Identity(3, 3);

			Rbs[i].Omega_hat = (Rbs[i].F*Rbs[i].Omega*Rbs[i].F.transpose() + Q).inverse();
			Rbs[i].xi_hat = Rbs[i].Omega_hat*Rbs[i].X_hat;
		}
		
	}
	std::cout << "hat: " << 1 << std::endl << Rbs[1].X_hat << std::endl;

}

void robots_EIF::computeCorrPairs()
{
	for(int i=0; i<mavNum ; i++)
	{
		if(i!=self_pointer)
		{
			Eigen::VectorXf rcc = Mavs_curr[i].r - Mavs_curr[self_pointer].r_c;
			Rbs[i].z.segment(0, 3) = Mavs_curr[self_pointer].r_c + rcc;
			if(Rbs[i].z == Rbs[i].pre_z)
			{
				Rbs[i].s.setZero(robots_state_size, robots_state_size);
				Rbs[i].y.setZero(robots_state_size);
			}
			else
			{
				Rbs[i].h.segment(0, 3) = Rbs[i].X_hat.segment(0, 3);
				//Rbs[i].h.segment(3, 3) = Rbs[i].X_hat.segment(0, 3);
				
				Rbs[i].H.block(0, 0, 3, 3) = Eigen::Matrix3f::Identity(3, 3);
				//Rbs[i].H.block(3, 0, 3, 3) = Eigen::Matrix3f::Identity(3, 3);

				Rbs[i].s = Rbs[i].H.transpose()*R.inverse()*Rbs[i].H;
				Rbs[i].y = Rbs[i].H.transpose()*R.inverse()*(Rbs[i].z - Rbs[i].h + Rbs[i].H*Rbs[i].X_hat);
			}
			Rbs[i].Omega = Rbs[i].Omega_hat + Rbs[i].s;
			Rbs[i].xi = Rbs[i].xi_hat + Rbs[i].y;
			Rbs[i].X = Rbs[i].Omega.inverse()*Rbs[i].xi;
		}	
		
	}
	std::cout << "y: "<< std::endl << Rbs[1].z - Rbs[1].h + Rbs[1].H*Rbs[1].X_hat << std::endl;
	std::cout << "H: "<< std::endl << Rbs[1].H << std::endl;
}