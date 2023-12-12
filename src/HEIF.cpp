#include "HEIF.h"
#include "EIF.h"

HEIF::HEIF(int num, int stateSize)
{
	fusionNum = num;
	state_size = stateSize;
	printf("[HEIF]: Fusion quantity: %i\n", fusionNum);
	printf("[HEIF]: Fusion state size: %i\n", state_size);

	T = new EIF_data[fusionNum];

	weightedOmega_hat.setZero(state_size, state_size);
	weightedXi_hat.setZero(state_size);
	weightedS.setZero(state_size, state_size);
	weightedY.setZero(state_size);
}

HEIF::~HEIF()
{
	delete[] T;
}

void HEIF::setData(EIF_data* target)
{
	T = target;
}

void HEIF::CI()
{
	float trace_sum = 0.0;
	float* weight = new float[fusionNum];
	weightedOmega_hat.setZero();
	weightedXi_hat.setZero();
	weightedS.setZero();
	weightedY.setZero();
	//////////////////////////// omega_hat, xi_hat ////////////////////////////
	for(int i=0; i<fusionNum; i++)
		trace_sum += T[i].Omega_hat.trace();
	
	for(int i = 0; i < fusionNum; i++)
	{
		if(trace_sum == 0)
			weight[i] = 0;
		else
			weight[i] = T[i].Omega_hat.trace()/trace_sum;
	}
	
	for(int i = 0; i < fusionNum; i++)
		weightedOmega_hat += weight[i]*T[i].Omega_hat;
	for(int i = 0; i < fusionNum; i++)
		weightedXi_hat += weight[i]*T[i].xi_hat;

	// for(int i=0; i<fusionNum; i++)
	// 	std::cout << "weight" << i << ": "<< weight[i];
	// std::cout<<std::endl;

	//////////////////////////// s, y ////////////////////////////
	trace_sum = 0.0;
	for(int i=0; i<fusionNum; i++)
		trace_sum += T[i].s.trace();
	
	for(int i = 0; i < fusionNum; i++)
	{
		if(trace_sum == 0)
			weight[i] = 0;
		else
			weight[i] = T[i].s.trace()/trace_sum;
	}
	
	for(int i = 0; i < fusionNum; i++)
		weightedS += weight[i]*T[i].s;
		
	for(int i = 0; i < fusionNum; i++)
		weightedY += weight[i]*T[i].y;

	fusedOmega = weightedOmega_hat + weightedS;
	fusedXi = weightedXi_hat + weightedY;
	fusedX_t = fusedOmega.inverse()*fusedXi;

	delete[] weight;
}

state_estimation::EIFpairStamped HEIF::getFusedPairs()
{
	state_estimation::EIFpairStamped fusionPairs_Vec;

	std::vector<float> Omega_vec(fusedOmega.data(), fusedOmega.data() + fusedOmega.size());
	std::vector<float> xi_vec(fusedXi.data(), fusedXi.data() + fusedXi.size());

	fusionPairs_Vec.fusedInfoMat = Omega_vec;
	fusionPairs_Vec.fusedInfoVec = xi_vec;
	return fusionPairs_Vec;
}

Eigen::VectorXf HEIF::getTargetState(){return fusedX_t;}
