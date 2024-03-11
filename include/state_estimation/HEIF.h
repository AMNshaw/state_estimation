#ifndef HEIF_H
#define HEIF_H
#pragma once

#include <Eigen/Dense>
#include <vector>
#include "EIF.h"

class HEIF
{
protected:

	Eigen::MatrixXd weightedOmega_hat;
	Eigen::MatrixXd weightedS;
	Eigen::VectorXd weightedXi_hat;
	Eigen::VectorXd weightedY;
	Eigen::MatrixXd fusedP;
	Eigen::VectorXd fusedX;

	int fusionNum;
	int state_size;
public:
	HEIF(int x_size);
	~HEIF();
	void initialize();
	virtual void CI_combination();
	Eigen::MatrixXd getFusedCov();
	Eigen::VectorXd getFusedState();
};
#endif