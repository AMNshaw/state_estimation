#include "HEIF.h"
#include "EIF.h"

HEIF::HEIF(int stateSize)
{
	state_size = stateSize;
	printf("[HEIF]: Fusion state size: %i\n", state_size);

	weightedOmega_hat.setZero(state_size, state_size);
	weightedXi_hat.setZero(state_size);
	weightedS.setZero(state_size, state_size);
	weightedY.setZero(state_size);
}

HEIF::~HEIF(){}

void HEIF::setData(std::vector<EIF_data> est_Data)
{
	est_data = est_Data;
	fusionNum = est_data.size();
}

void HEIF::setData(std::vector<EIF_data> est_Data, EIF_data self)
{
	self_est = self;
	est_data = est_Data;
	fusionNum = est_data.size();
}

double HEIF::ICI_optimize_func(double weight)
{
	Gamma = weightedS*(weight*weightedS + (1-weight)*weightedOmega_hat).inverse()*weightedOmega_hat;
	Eigen::MatrixXf p = (weightedS + weightedOmega_hat - Gamma).inverse();
	return static_cast<double>(p.trace());
}

void HEIF::ICI()
{
	double w = 0.5;
	dlib::find_min_single_variable(
		[this](double weight){return ICI_optimize_func(weight);},
		w,
		0.0,
		0.999,
		0.001,
		100,
		0.1);
	
	Gamma = weightedS*(w*weightedS + (1-w)*weightedOmega_hat).inverse()*weightedOmega_hat;

	K = weightedOmega_hat - w*Gamma;
	L = weightedS - (1-w)*Gamma;

	fusedP = (weightedS + weightedOmega_hat - Gamma).inverse();
	fusedX = fusedP*(K*weightedOmega_hat.inverse()*weightedXi_hat + L*weightedY);
	std::cout << "weight:" << w << std::endl;
	std::cout << "fusedX:\n" << fusedX << "\n\n";
}

void HEIF::CI()
{
	float trace_sum = 0.0;
	float* weight = new float[fusionNum];
	
	//////////////////////////// P_hat, X_hat ////////////////////////////
	for(int i=0; i<fusionNum; i++)
		trace_sum += 1/est_data[i].P_hat.trace();
	for(int i=0; i<fusionNum; i++)
	{
		weight[i] = 1/est_data[i].P_hat.trace()/trace_sum;
		weightedOmega_hat += weight[i]*est_data[i].P_hat.inverse();
		weightedXi_hat += weight[i]*(est_data[i].P_hat.inverse()*est_data[i].X_hat);
	}

	// for(int i=0; i<fusionNum; i++)
	// {
	// 	std::cout << "weight" << i+1 << ": " << weight[i] << "\n";
	// }

	//////////////////////////// s, y ////////////////////////////
	trace_sum = 0.0;
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

	

	delete[] weight;
}

void HEIF::CI_combination()
{
	fusedP = (weightedOmega_hat + weightedS).inverse();
	fusedX = fusedP*(weightedXi_hat + weightedY);
	//std::cout << "X:\n" << fusedX << "\n\n";
}

void HEIF::CI_combination_with_selfEst()
{
	weightedOmega_hat += self_est.P_hat.inverse();
	weightedS += self_est.s;
	weightedXi_hat += self_est.P_hat.inverse()*self_est.X_hat;
	weightedY += self_est.y;
	CI_combination();
}

void HEIF::process()
{
	weightedOmega_hat.setZero();
	weightedXi_hat.setZero();
	weightedS.setZero();
	weightedY.setZero();
	if(est_data.size() > 0)
		CI();
	if(self_est.X_hat.size() > 0)
		CI_combination_with_selfEst();
	else
		CI_combination();
}

Eigen::MatrixXf HEIF::getFusedCov(){return fusedP;}
Eigen::VectorXf HEIF::getFusedState(){return fusedX;}
