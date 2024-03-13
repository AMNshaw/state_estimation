#ifndef HEIF_TARGET_H
#define HEIF_TARGET_H
#pragma once

#include "HEIF.h"
#include <deque>
#include "OsqpEigen/OsqpEigen.h"

class HEIF_target : public HEIF
{
private:
	std::vector<EIF_data> est_data;
	std::vector<double> weight;

	int fusionNum;
	int state_size;
	
	std::deque<Eigen::VectorXd> est_X;
	std::deque<Eigen::MatrixXd> est_P;
    int std_filter_size;
	int mean_filter_size;

	/*=================================================================================================================================
		Quadratic Programming
	=================================================================================================================================*/
	struct QP
	{
		Eigen::MatrixXd A;
		Eigen::MatrixXd P;
		Eigen::VectorXd q;
		Eigen::VectorXd Y;
		Eigen::VectorXd lower_bound;
		Eigen::VectorXd upper_bound;
		Eigen::VectorXd solution;
		int dataNum;
		int functionOrder;

		OsqpEigen::Solver solver;
	} qp;
	
	std::deque<double> t;
	std::deque<Eigen::Vector3d> positions;
	Eigen::Vector3d qpAcc;
public:
	HEIF_target(int x_size);
	~HEIF_target();
	void setTargetEstData(std::vector<EIF_data> est_Data);
	void TargetEstDataCI();
	void CI_combination();
	void stdDevFilter();
	void process();

	/*=================================================================================================================================
		Quadratic Programming
	=================================================================================================================================*/
	void QP_pushData(double t, Eigen::Vector3d position);
	bool QP_init(int dataNum, int order);
	bool computeQP();
	Eigen::Vector3d getQpAcc();
};

#endif