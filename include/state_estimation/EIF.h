#ifndef EIF_H
#define EIF_H
#pragma once
#include "Mav.h"


struct EIF_data
{
    Eigen::VectorXf X; //state
	Eigen::VectorXf X_hat;
	Eigen::VectorXf xi;
	Eigen::VectorXf xi_hat;
	Eigen::VectorXf y;
	Eigen::VectorXf h;
    Eigen::VectorXf z;
    Eigen::VectorXf pre_z;

    Eigen::MatrixXf s;
	Eigen::MatrixXf F;
	Eigen::MatrixXf H;
    Eigen::MatrixXf P;
    Eigen::MatrixXf P_hat;
    Eigen::MatrixXf Omega;
    Eigen::MatrixXf Omega_hat;
    
    int ID;
};

class EIF
{
protected:

    int mavNum;

    Eigen::MatrixXf Q; //noise matrix
    Eigen::MatrixXf R; //noise matrix
    Eigen::VectorXf t_b2c;
    Eigen::MatrixXf R_b2c;

    Eigen::MatrixXf fusedOmega;
    Eigen::VectorXf fusedXi;
    
public:
	EIF();
	~EIF();

    Eigen::Matrix3f skew(Eigen::Vector3f vec);

    void EIF_data_init(int x_size, int z_size, EIF_data* est_object);
    void EIF_prediction_init(int x_size, EIF_data* est_object);
    void EIF_measurement_init(int x_size, int z_size, EIF_data* est_object);
	void set_process_noise(Eigen::MatrixXf matrix);
    void set_measurement_noise(Eigen::MatrixXf matrix);
};




#endif