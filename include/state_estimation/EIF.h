#ifndef EIF_H
#define EIF_H
#pragma once
#include "Mav.h"


struct EIF_data
{
    Eigen::VectorXd X; //state
	Eigen::VectorXd X_hat;
	Eigen::VectorXd xi;
	Eigen::VectorXd xi_hat;
	Eigen::VectorXd y;
	Eigen::VectorXd h;
    Eigen::VectorXd z;
    Eigen::VectorXd pre_z;

    Eigen::MatrixXd s;
	Eigen::MatrixXd F;
	Eigen::MatrixXd H;
    Eigen::MatrixXd P;
    Eigen::MatrixXd P_hat;
    Eigen::MatrixXd Omega;
    Eigen::MatrixXd Omega_hat;
    
    int ID;
};

class EIF
{
protected:

    int mavNum;

    Eigen::MatrixXd Q; //noise matrix
    Eigen::MatrixXd R; //noise matrix
    Eigen::VectorXd t_b2c;
    Eigen::MatrixXd R_b2c;

    Eigen::MatrixXd fusedOmega;
    Eigen::VectorXd fusedXi;

    MAV_eigen Mav_eigen_self;
public:
	EIF();
	~EIF();

    Eigen::Matrix3d skew(Eigen::Vector3d vec);

    void EIF_data_init(int x_size, int z_size, EIF_data* est_object);
    void EIF_prediction_init(int x_size, EIF_data* est_object);
    void EIF_measurement_init(int x_size, int z_size, EIF_data* est_object);
	void set_process_noise(Eigen::MatrixXd matrix);
    void set_measurement_noise(Eigen::MatrixXd matrix);
    void setMavSelfData(MAV_eigen MES);
};




#endif