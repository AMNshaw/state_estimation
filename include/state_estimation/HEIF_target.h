#ifndef HEIF_TARGET_H
#define HEIF_TARGET_H
#pragma once

#include "HEIF.h"

class HEIF_target : public HEIF
{
private:
	std::vector<EIF_data> est_data;
	std::vector<float> weight;

	int fusionNum;
	int state_size;
public:
	HEIF_target(int x_size);
	~HEIF_target();
	void setTargetEstData(std::vector<EIF_data> est_Data);
	void TargetEstDataCI();
	void CI_combination();
	void process();
};
#endif