#ifndef EIF_H
#define EIF_H
#pragma once
#include <Eigen/Dense>


class Eif
{
private:

	int state_size;
	int measurement_size;

	Eigen::VectorXd X; //state
	Eigen::VectorXd X_hat; //predicted state
	Eigen::VectorXd xi;
	Eigen::VectorXd xi_hat; // information vector
	Eigen::VectorXd y;
	Eigen::VectorXd z;
	Eigen::VectorXd h_X;

	Eigen::MatrixXd Omega;
	Eigen::MatrixXd Omega_hat;
	Eigen::MatrixXd S;
	Eigen::MatrixXd F;
	Eigen::MatrixXd H;
	Eigen::MatrixXd Q; //noise matrix
    Eigen::MatrixXd R; //noise matrix
    Eigen::MatrixXd C;

    Eigen::MatrixXd Intrinsic;
    Eigen::Quaterniond q;

    double depth;
    int u;
    int v;

public:
	Eif(int x_size, int z_size);
	~Eif();
	void predict(double dt);
	void correct(Eigen::VectorXd measurement);
	void set_process_noise(Eigen::MatrixXd matrix);
    void set_measurement_noise(Eigen::MatrixXd matrix);
    void set_intrinsic_matrix(Eigen::MatrixXd matrix);


};





#endif