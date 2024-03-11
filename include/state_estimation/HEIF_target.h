#ifndef HEIF_TARGET_H
#define HEIF_TARGET_H
#pragma once

#include "HEIF.h"
#include <deque>

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

public:
	HEIF_target(int x_size);
	~HEIF_target();
	void setTargetEstData(std::vector<EIF_data> est_Data);
	void TargetEstDataCI();
	void CI_combination();
	void stdDevFilter();
	void process();

	
};
#endif