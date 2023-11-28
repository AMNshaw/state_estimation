
#include "REIF.h"

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