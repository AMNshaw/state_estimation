#include "HEIF_self.h"

HEIF_self::HEIF_self(int x_size=6) : HEIF(x_size)
{
	fusionNum = 0;
}

HEIF_self::~HEIF_self(){}

void HEIF_self::setNeighborEstData(std::vector<EIF_data> est_Data)
{
	est_data = est_Data;
	fusionNum = 0;
	fusionNum = est_data.size();
    weight.resize(fusionNum);
}

void HEIF_self::setSelfEstData(EIF_data self)
{
	if(self.X_hat.size() > 0)
		state_size = self.X_hat.size();

	self_est = self;
}

void HEIF_self::eighborEstDataCI()
{
	float trace_sum = 0.0;

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

void HEIF_self::CI_combination()
{
	weightedOmega_hat += self_est.P_hat.inverse();
	weightedXi_hat += self_est.P_hat.inverse()*self_est.X_hat;
	weightedS += self_est.s;
	weightedY += self_est.y;

	fusedP = (weightedOmega_hat + weightedS).inverse();
	fusedX = fusedP*(weightedXi_hat + weightedY);
}

void HEIF_self::process()
{
	initialize();
	if(fusionNum > 0)
		eighborEstDataCI();
	CI_combination();
}