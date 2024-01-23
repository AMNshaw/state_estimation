#ifndef HEIF_H
#define HEIF_H
#pragma once

#include <Eigen/Dense>
#include <vector>
#include "EIF.h"

class HEIF
{
protected:

	Eigen::MatrixXf weightedOmega_hat;
	Eigen::MatrixXf weightedS;
	Eigen::VectorXf weightedXi_hat;
	Eigen::VectorXf weightedY;
	Eigen::MatrixXf fusedP;
	Eigen::VectorXf fusedX;

	int fusionNum;
	int state_size;
public:
	HEIF(int x_size);
	~HEIF();
	void initialize();
	virtual void CI_combination();
	Eigen::MatrixXf getFusedCov();
	Eigen::VectorXf getFusedState();
};
#endif