#ifndef HEIF_H
#define HEIF_H
#pragma once

#include <Eigen/Dense>
#include <state_estimation/EIFpairStamped.h>

class HEIF
{
private:
	Eigen::MatrixXf* Omega_hat;
	Eigen::MatrixXf* s;
	Eigen::VectorXf* xi_hat;
	Eigen::VectorXf* y;

	Eigen::MatrixXf weightedOmega_hat;
	Eigen::MatrixXf weightedS;
	Eigen::VectorXf weightedXi_hat;
	Eigen::VectorXf weightedY;

	Eigen::MatrixXf fusedOmega;
	Eigen::VectorXf fusedXi;
	Eigen::VectorXf fusedX_t;

	int groupsNum;
	int state_size;
public:
	HEIF(int num, int stateSize);
	~HEIF();
	void inputFusionPairs(state_estimation::EIFpairStamped* fusionPairs_Vec);
	void covarianceIntercection(Eigen::MatrixXd infoMat, Eigen::VectorXd infoVec);
	void weightFusionPairs(Eigen::MatrixXf* infoMat, Eigen::VectorXf* infoVec, Eigen::MatrixXf& weightedInfoMat, Eigen::VectorXf& weightedInfoVec);
	void CI();
	void compare(Eigen::VectorXf targetState_GT);
	state_estimation::EIFpairStamped getFusedPairs();
};







#endif