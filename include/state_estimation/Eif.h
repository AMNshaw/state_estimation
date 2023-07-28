#ifndef EIF_H
#define EIF_H
#pragma once
#include <Eigen/Dense>


class Eif
{
private:
	Eigen::VectorXd X; //state
	Eigen::VectorXd X_hat; //predicted state
	Eigen::VectorXd xi_hat; // information vector
	Eigen::VectorXd y;
	Eigen::VectorXd z;
	Eigen::VectorXd h_X;

	Eigen::MatrixXd Omega;
	Eigen::MatrixXd Omega_hat;
	Eigen::MatrixXd S;
	Eigen::MatrixXd F;
	Eigen::MatrixXd Q; //noise matrix
    Eigen::MatrixXd R; //noise matrix

public:
	void predict(double dt);
	void correct(Eigen::VectorXd measure);
	void set_process_noise(Eigen::MatrixXd matrix);
    void set_measurement_noise(Eigen::MatrixXd matrix);


};





#endif