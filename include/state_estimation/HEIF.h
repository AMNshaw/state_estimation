#ifndef HEIF_H
#define HEIF_H
#pragma once

#include <Eigen/Dense>
#include <vector>
#include "EIF.h"
#include <dlib/optimization.h>

class HEIF
{
private:

	Eigen::MatrixXf weightedOmega_hat;
	Eigen::MatrixXf weightedS;
	Eigen::VectorXf weightedXi_hat;
	Eigen::VectorXf weightedY;
	Eigen::MatrixXf fusedP;
	Eigen::VectorXf fusedX;
	Eigen::MatrixXf K;
	Eigen::MatrixXf L;
	Eigen::MatrixXf Gamma;

	std::vector<EIF_data> est_data;
	EIF_data self_est;

	int fusionNum;
	int state_size;
public:
	HEIF(int stateSize);
	~HEIF();
	void setData(std::vector<EIF_data> est_Data);
	void setData(std::vector<EIF_data> est_Data, EIF_data self);
	void process();
	void CI();
	void CI_combination();
	void CI_combination_with_selfEst();
	double ICI_optimize_func(double weight);
	void ICI();
	Eigen::MatrixXf getFusedCov();
	Eigen::VectorXf getFusedState();
};
#endif