#ifndef EIF_H
#define EIF_H
#pragma once
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

class Eif
{
private:

	int state_size;
	int measurement_size;

	Eigen::VectorXd X_T; //state
	Eigen::VectorXd X_T_hat; //predicted state
	Eigen::VectorXd X_B;
	Eigen::VectorXd E;
	Eigen::VectorXd E_hat;
	Eigen::VectorXd xi;
	Eigen::VectorXd xi_hat; // information vector
	Eigen::VectorXd y;
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
    Eigen::VectorXd t_w2b;
    Eigen::VectorXd t_b2c;
    Eigen::MatrixXd R_w2b;
    Eigen::MatrixXd R_b2c;

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
    void setSelfState(geometry_msgs::PoseStamped P, geometry_msgs::TwistStamped V);
    void compare(Eigen::VectorXd groundTruth, Eigen::VectorXd measurement);

};





#endif