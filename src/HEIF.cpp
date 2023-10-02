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

	std::cout << "DEIF constructed" << "\n";
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
		weight[i] = infoMat[i].trace()/trace_sum;
	for(int i = 0; i < groupsNum; i++)
		weightedInfoMat += weight[i]*infoMat[i];
	for(int i = 0; i < groupsNum; i++)
		weightedInfoVec += weight[i]*infoVec[i];

	/*
	std::cout << "weight1: " << weight[0] << "\n";
	std::cout << "weight2: " << weight[1] << "\n";
	std::cout << "trace1: \n" << infoMat[0] << "\n";
	std::cout << "trace2: \n" << infoMat[1] << "\n";

	std::cout << "traceSum: " << trace_sum << "\n";
	*/
}

void HEIF::CI()
{
	weightFusionPairs(Omega_hat, xi_hat, weightedOmega_hat, weightedXi_hat);
	weightFusionPairs(s, y, weightedS, weightedY);

	double trace_sum = weightedOmega_hat.trace() + weightedS.trace();
	double weight_1 = weightedOmega_hat.trace()/trace_sum;
	double weight_2 = weightedS.trace()/trace_sum;
	
	/*
	fusedOmega = weight_1*weightedOmega_hat + weight_2*weightedS;
	fusedXi = weight_1*weightedXi_hat + weight_2*weightedY;
	fusedX_t = fusedOmega.inverse()*fusedXi;
	*/

	//std::cout << "Omega: \n" << weightedOmega_hat << "\n";
	//std::cout << "S: \n" << weightedS << "\n";
	//std::cout << "weight_1:" << weight_1 << "\n";
	//std::cout << "weight_2 " << weight_2 << "\n";
	
	fusedOmega = weightedOmega_hat + weightedS;
	fusedXi = weightedXi_hat + weightedY;
	fusedX_t = fusedOmega.inverse()*fusedXi;
	
}

void HEIF::compare(Eigen::VectorXf targetState_GT)
{
	Eigen::VectorXf err = targetState_GT - fusedX_t;
	Eigen::VectorXf err_p(state_size/2);
	Eigen::VectorXf err_v(state_size/2);
	err_p << err(0), err(1), err(2);
	err_v << err(3), err(4), err(5);

	std::cout << "[HEIF]: X_t\n" << fusedX_t << "\n\n";
	std::cout << "[HEIF]: RMS_p: " << err_p.norm() << "\n[HEIF]: RMS_v: " << err_v.norm() << "\n\n";
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