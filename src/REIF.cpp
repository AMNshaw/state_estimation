#include "REIF.h"

/////////////////////////////////////////////////////////// robots EIF ///////////////////////////////////////////////////////////
robots_EIF::robots_EIF(int selfPointer, int MavNum) : EIF(selfPointer, MavNum)
{
	robots_state_size = 9;
	robots_measurement_size = 6;

	Mavs_curr.resize(MavNum);

	Rbs = new EIF_data[mavNum];
	for(int i=0; i < mavNum; i++)
		if(i != self_pointer)
	 		EIF_data_init(robots_state_size, robots_measurement_size, &Rbs[i]);

	//////////////////////// Covariance Tuning ////////////////////////

	Q.block(0, 0, 3, 3) = 7e-4*Eigen::MatrixXf::Identity(3, 3); // position
	Q.block(3, 3, 3, 3) = 1e-2*Eigen::MatrixXf::Identity(3, 3); // velocity
	Q.block(6, 6, 3, 3) = 7e-2*Eigen::MatrixXf::Identity(3, 3); //acceleration
	
}
robots_EIF::~robots_EIF()
{
	delete[] Rbs;
}

void robots_EIF::setData(std::vector<MAV_eigen> MAVs)
{
	Mavs_curr = MAVs;
}

void robots_EIF::setData(std::vector<MAV_eigen> MAVs, Eigen::VectorXf self_state)
{
	Mavs_curr = MAVs;
	Mavs_curr[self_pointer].r = self_state.segment(0, 3);
}

EIF_data* robots_EIF::getRbsData(){return Rbs;}

void robots_EIF::computePredPairs(double delta_t)
{
	float dt = static_cast<float>(delta_t);

	for(int i=0; i<mavNum ; i++)
	{
		if(i!=self_pointer)
		{
			Rbs[i].F.setIdentity();
			Rbs[i].F.block(0, 3, 3, 3) = Eigen::Matrix3f::Identity(3, 3)*dt;
			Rbs[i].F.block(0, 6, 3, 3) = 1/2*Eigen::Matrix3f::Identity(3, 3)*dt*dt;
			Rbs[i].F.block(3, 6, 3, 3) = Eigen::Matrix3f::Identity(3, 3)*dt;
			
			Rbs[i].P_hat = Rbs[i].F*Rbs[i].P*Rbs[i].F.transpose() + Q;
			Rbs[i].X_hat = Rbs[i].F*Rbs[i].X;
		}
	}
}

void robots_EIF::computeCorrPairs()
{
	for(int i=0; i<mavNum ; i++)
	{
		if(i!=self_pointer)
		{
			Eigen::VectorXf rpj_pi = Mavs_curr[i].r - Mavs_curr[self_pointer].r;
			Rbs[i].z.segment(0, 3) = rpj_pi;
			Rbs[i].z.segment(3, 3) = Mavs_curr[self_pointer].r + rpj_pi;

			if(Rbs[i].z == Rbs[i].pre_z)
			{
				Rbs[i].s.setZero();
				Rbs[i].y.setZero();
			}
			else
			{		
				Rbs[i].h.segment(0, 3) = Rbs[i].X_hat.segment(0, 3) - Mavs_curr[self_pointer].r;
				Rbs[i].h.segment(3, 3) = Rbs[i].X_hat.segment(0, 3);
				
				Rbs[i].H.block(0, 0, 3, 3) = Eigen::Matrix3f::Identity(3, 3);
				Rbs[i].H.block(3, 0, 3, 3) = Eigen::Matrix3f::Identity(3, 3);

				Rbs[i].s = Rbs[i].H.transpose()*R.inverse()*Rbs[i].H;
				Rbs[i].y = Rbs[i].H.transpose()*R.inverse()*(Rbs[i].z - Rbs[i].h + Rbs[i].H*Rbs[i].X_hat);
			}
			Rbs[i].P = (Rbs[i].P_hat.inverse() + Rbs[i].s).inverse();
			Rbs[i].X = Rbs[i].P*(Rbs[i].P_hat.inverse()*Rbs[i].X_hat + Rbs[i].y);
			
			Rbs[i].pre_z = Rbs[i].z;
		}	
	}
}