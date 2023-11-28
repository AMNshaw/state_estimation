#include "HEIF.h"

HEIF::HEIF(int num, int stateSize)
{
	groupsNum = num;
	state_size = stateSize;
	printf("[HEIF]: Fusion quantity: %i\n", groupsNum);
	printf("[HEIF]: Fusion state size: %i\n", state_size);

	Omega_hat = new Eigen::MatrixXf[num];
	xi_hat = new Eigen::VectorXf[num];
	s = new Eigen::MatrixXf[num];
	y = new Eigen::VectorXf[num];

	for(int i = 0; i < groupsNum; i++)
	{
		Omega_hat[i].resize(state_size, state_size);
		xi_hat[i].resize(state_size);
		s[i].resize(state_size, state_size);
		y[i].resize(state_size);
	}

	weightedOmega_hat.setZero(state_size, state_size);
	weightedXi_hat.setZero(state_size);
	weightedS.setZero(state_size, state_size);
	weightedY.setZero(state_size);
}

HEIF::~HEIF()
{
	delete[] Omega_hat;
	delete[] xi_hat;
	delete[] s;
	delete[] y;
}

void HEIF::inputFusionPairs(state_estimation::EIFpairStamped* fusionPairs_Vec)
{
	for(int i = 0; i < groupsNum; i++)
	{
		Eigen::Map<Eigen::MatrixXf> predInfoMat(fusionPairs_Vec[i].predInfoMat.data(), state_size, state_size);
		Omega_hat[i] = predInfoMat;
		Eigen::Map<Eigen::VectorXf> predInfoVec(fusionPairs_Vec[i].predInfoVec.data(), state_size);
		xi_hat[i] = predInfoVec;
		Eigen::Map<Eigen::MatrixXf> corrInfoMat(fusionPairs_Vec[i].corrInfoMat.data(), state_size, state_size);
		s[i] = corrInfoMat;
		Eigen::Map<Eigen::VectorXf> corrInfoVec(fusionPairs_Vec[i].corrInfoVec.data(), state_size);
		y[i] = corrInfoVec;
	}

	// for(int i = 0; i < groupsNum; i++)
	// {
	// 	std::cout << i << " pred: \n" << Omega_hat[i].inverse()*xi_hat[i] << std::endl;
	// }
	// for(int i = 0; i < groupsNum; i++)
	// {
	// 	if(y[i](0) != 0)
	// 		std::cout << i << " corr: \n" << y[i] << std::endl;
	// }
}


void HEIF::weightFusionPairs(Eigen::MatrixXf* infoMat, Eigen::VectorXf* infoVec, Eigen::MatrixXf& weightedInfoMat, Eigen::VectorXf& weightedInfoVec)
{
	weightedInfoMat.setZero();
	weightedInfoVec.setZero();
	float trace_sum = 0.0;
	float* weight = new float[groupsNum];
	for(int i = 0; i < groupsNum; i++)
		trace_sum += infoMat[i].trace();;
	for(int i = 0; i < groupsNum; i++)
	{
		if(trace_sum == 0)
			weight[i] = 0;
		else
			weight[i] = infoMat[i].trace()/trace_sum;
	}
	for(int i = 0; i < groupsNum; i++)
		weightedInfoMat += weight[i]*infoMat[i];
	for(int i = 0; i < groupsNum; i++)
		weightedInfoVec += weight[i]*infoVec[i];

	for(int i = 0; i < groupsNum; i++)
		if(weight[i] != 0)
			std::cout << i << " weight: "<< weight[i] << "\n\n";

	delete[] weight;
}

void HEIF::CI()
{
	printf("pred:\n");
	weightFusionPairs(Omega_hat, xi_hat, weightedOmega_hat, weightedXi_hat);
	printf("corr:\n");
	weightFusionPairs(s, y, weightedS, weightedY);

	//double trace_sum = weightedOmega_hat.trace() + weightedS.trace();
	//double weight_1 = weightedOmega_hat.trace()/trace_sum;
	//double weight_2 = weightedS.trace()/trace_sum;
	
	/*
	fusedOmega = weight_1*weightedOmega_hat + weight_2*weightedS;
	fusedXi = weight_1*weightedXi_hat + weight_2*weightedY;
	fusedX_t = fusedOmega.inverse()*fusedXi;
	*/

	//std::cout << "weighted pred: \n" << weightedOmega_hat.inverse()*weightedXi_hat << "\n\n";
	// if(weightedY(0) != 0)
	// 	std::cout << "weighted corr: \n" << weightedY << "\n\n";
	//std::cout << "S: \n" << weightedS << "\n";
	//std::cout << "weight_1:" << weight_1 << "\n";
	//std::cout << "weight_2 " << weight_2 << "\n";
	
	fusedOmega = weightedOmega_hat + weightedS;
	fusedXi = weightedXi_hat + weightedY;
	fusedX_t = fusedOmega.inverse()*fusedXi;
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
