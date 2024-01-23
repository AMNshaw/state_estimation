#include "HEIF_target.h"

HEIF_target::HEIF_target(int x_size=6) : HEIF(x_size)
{
	fusionNum = 0;
}

HEIF_target::~HEIF_target(){}

void HEIF_target::setTargetEstData(std::vector<EIF_data> est_Data)
{
	est_data = est_Data;
	fusionNum = 0;
	fusionNum = est_data.size();
    weight.resize(fusionNum);
}

void HEIF_target::TargetEstDataCI()
{
	float trace_sum = 0.0;

	//////////////////////////// X_hat, P_hat ////////////////////////////
    for(int i=0; i<fusionNum; i++)
		trace_sum += 1/est_data[i].P_hat.trace();
	for(int i=0; i<fusionNum; i++)
	{
		weight[i] = 1/est_data[i].P_hat.trace()/trace_sum;
		weightedOmega_hat += weight[i]*est_data[i].P_hat.inverse();
		weightedXi_hat += weight[i]*(est_data[i].P_hat.inverse()*est_data[i].X_hat);
	}

	//////////////////////////// s, y ////////////////////////////
	for(int i=0; i<fusionNum; i++)
		trace_sum += est_data[i].s.trace();
	for(int i=0; i<fusionNum; i++)
	{
		if(trace_sum !=0)
		{
			weight[i] = est_data[i].s.trace()/trace_sum;
			weightedS += weight[i]*est_data[i].s;
			weightedY += weight[i]*est_data[i].y;
		}
	}
}

void HEIF_target::CI_combination()
{
	fusedP = (weightedOmega_hat + weightedS).inverse();
	fusedX = fusedP*(weightedXi_hat + weightedY);
}

void HEIF_target::process()
{
	initialize();
	if(fusionNum > 0)
		TargetEstDataCI();
	CI_combination();
}