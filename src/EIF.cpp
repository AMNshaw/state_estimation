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
