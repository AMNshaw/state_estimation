#include "EIF.h"

EIF::EIF(){}

EIF::~EIF(){}

void EIF::EIF_data_init(int x_size, int z_size, EIF_data* est_object)
{
	est_object->X.setZero(x_size);
	for(int i=0; i < x_size; i++)
		est_object->X(i) = 1.0;
	est_object->X_hat.setZero(x_size);
	est_object->xi.setZero(x_size);
	est_object->xi_hat.setZero(x_size);
	est_object->z.setZero(z_size);
	est_object->h.setZero(z_size);
	est_object->pre_z.setZero(z_size);
	est_object->y.setZero(x_size);

	est_object->P = 1e-3*Eigen::MatrixXd::Identity(x_size, x_size);
	est_object->F.setZero(x_size, x_size);
	est_object->H.setZero(z_size, x_size);
	est_object->s.setZero(x_size, x_size);
	est_object->Omega = 1e3*Eigen::MatrixXd::Identity(x_size, x_size);
	
	Q = 7e-4*Eigen::MatrixXd::Identity(x_size, x_size);
	R = 1e-4*Eigen::MatrixXd::Identity(z_size, z_size);
}

void EIF::EIF_prediction_init(int x_size, EIF_data* est_object)
{
	est_object->X.setZero(x_size);
	for(int i=0; i < x_size; i++)
		est_object->X(i) = 1.0;
	est_object->X_hat.setZero(x_size);
	est_object->xi.setZero(x_size);
	est_object->xi_hat.setZero(x_size);

	est_object->P = 1e-3*Eigen::MatrixXd::Identity(x_size, x_size);
	est_object->F.setZero(x_size, x_size);
	est_object->Omega = 1e3*Eigen::MatrixXd::Identity(x_size, x_size);
	
	Q = 7e-4*Eigen::MatrixXd::Identity(x_size, x_size);
}

void EIF::EIF_measurement_init(int x_size, int z_size, EIF_data* est_object)
{
	est_object->z.setZero(z_size);
	est_object->pre_z.setZero(z_size);
	est_object->y.setZero(x_size);
	est_object->h.setZero(z_size);
	
	est_object->H.setZero(z_size, x_size);
	est_object->s.setZero(x_size, x_size);

	R = 1e-4*Eigen::MatrixXd::Identity(z_size, z_size);
}

void EIF::set_process_noise(Eigen::MatrixXd matrix){Q = matrix;}
void EIF::set_measurement_noise(Eigen::MatrixXd matrix){R = matrix;}

void EIF::setMavSelfData(MAV_eigen MES)
{
	Mav_eigen_self = MES;
}

Eigen::Matrix3d EIF::skew(Eigen::Vector3d vec)
{
	Eigen::Matrix3d vec_skew;
	vec_skew << 0, -vec(2), vec(1),
				vec(2), 0, -vec(0),
				-vec(1), vec(0), 0;
	return vec_skew;
}
