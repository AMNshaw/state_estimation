#ifndef EIF_H
#define EIF_H
#pragma once
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

class EIF
{
private:

	int state_size;
	int measurement_size;

	Eigen::VectorXd X_t; //state
	Eigen::VectorXd X_t_hat;
	Eigen::VectorXd X_b;
	Eigen::VectorXd X_b_last;
	Eigen::VectorXd E;
	Eigen::VectorXd E_hat;
	Eigen::VectorXd xi;
	Eigen::VectorXd xi_hat; // information vector
	Eigen::VectorXd y;
	Eigen::VectorXd h;

	Eigen::MatrixXd s;
	Eigen::MatrixXd F;
	Eigen::MatrixXd H;
	Eigen::MatrixXd Q; //noise matrix
    Eigen::MatrixXd R; //noise matrix
    Eigen::MatrixXd C;
    Eigen::MatrixXd P;
    Eigen::MatrixXd P_hat;
    Eigen::MatrixXd Omega;
    Eigen::MatrixXd Omega_hat;
    Eigen::VectorXd pre_measurement;
    Eigen::MatrixXd pre_Omega;

    Eigen::MatrixXd Intrinsic;
    Eigen::VectorXd t_w2b;
    Eigen::VectorXd t_b2c;
    Eigen::MatrixXd R_w2b;
    Eigen::MatrixXd R_b2c;

    int u;
    int v;
    bool consensus;

public:
	EIF(int x_size, int z_size, bool Consensus);
	~EIF();
	void computePredPairs(double dt);
	void computeCorrPairs(Eigen::VectorXd z);
	void correct(Eigen::VectorXd measurement);
	void predict_fused(double dt, Eigen::MatrixXd Omega, Eigen::VectorXd xi, bool flag);
	void set_process_noise(Eigen::MatrixXd matrix);
    void set_measurement_noise(Eigen::MatrixXd matrix);
    void set_intrinsic_matrix(Eigen::MatrixXd matrix);
    void setSelfState(geometry_msgs::PoseStamped P, geometry_msgs::TwistStamped V);
    void setSelfState(geometry_msgs::PoseStamped P, geometry_msgs::TwistStamped V, Eigen::VectorXd A);
    void compare(Eigen::VectorXd groundTruth);
    Eigen::VectorXd getTargetState();
    void process(double dt, Eigen::VectorXd z, Eigen::MatrixXd fusedOmega, Eigen::VectorXd fusedXi, bool flag);

    void getPredictionPairs(Eigen::MatrixXd* infoMat, Eigen::VectorXd* infoVec);
    void getCorrectionPairs(Eigen::MatrixXd* infoMat, Eigen::VectorXd* infoVec);
};





#endif