#ifndef HEIF_SELF_H
#define HEIF_SELF_H
#pragma once

#include "HEIF.h"

class HEIF_self : public HEIF
{
private:

	std::vector<EIF_data> self_est_neighbor;
	std::vector<float> weight;
	EIF_data self_est;
	EIF_data self_est_target;

	int fusionNum;
	int state_size;
public:
	HEIF_self(int x_size);
	~HEIF_self();
	void setNeighborEstData(std::vector<EIF_data> est_Data);
	void setSelfEstData(EIF_data self);
	void setSelfEstData(EIF_data self, EIF_data self_est_target);
	void eighborEstDataCI();
	void CI_combination();
	void process();
};
#endif