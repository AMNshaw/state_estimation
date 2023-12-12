#ifndef HEIF_H
#define HEIF_H
#pragma once

#include <Eigen/Dense>
#include <state_estimation/EIFpairStamped.h>
#include "EIF.h"

class HEIF
{
private:

	Eigen::MatrixXf weightedOmega_hat;
	Eigen::MatrixXf weightedS;
	Eigen::VectorXf weightedXi_hat;
	Eigen::VectorXf weightedY;

	Eigen::MatrixXf fusedOmega;
	Eigen::VectorXf fusedXi;
	Eigen::VectorXf fusedX_t;

	EIF_data* T;

	int fusionNum;
	int state_size;
public:
	HEIF(int num, int stateSize);
	~HEIF();
	void setData(EIF_data* target);
	void CI();
	Eigen::VectorXf getTargetState();
	state_estimation::EIFpairStamped getFusedPairs();
};







#endif