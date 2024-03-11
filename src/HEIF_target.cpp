#include "HEIF_target.h"

HEIF_target::HEIF_target(int x_size=6) : HEIF(x_size)
{
	fusionNum = 0;

	std_filter_size = 20;
	mean_filter_size = 5;
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
	double trace_sum = 0.0;

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

	est_X.push_back(fusedX);
	est_P.push_back(fusedP);
	if(est_X.size() > mean_filter_size)
	{
		est_X.pop_front();
		est_P.pop_front();
	}
	//stdDevFilter();
}

void HEIF_target::stdDevFilter()
{
	if(est_X.size() == std_filter_size)
	{
		Eigen::VectorXd mean = Eigen::VectorXd::Zero(3);
		Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(3, 3);
		for(const auto& vec : est_X)
			mean += vec;
		mean /= static_cast<double>(est_X.size());
		for(const auto& vec : est_X)
			covariance += (vec - mean)*(vec - mean).transpose();
		covariance /= static_cast<double>(est_X.size() - 1);
		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(covariance);
		if (solver.info() != Eigen::Success)
        	std::cerr << "Eigenvalue decomposition failed!" << std::endl;
		else
		{
			Eigen::VectorXd std_dev = solver.eigenvalues().cwiseSqrt();
			if((fusedX.segment(3, 3) - mean).sum() > std_dev.sum()*0.5)
			{
				fusedP = weightedOmega_hat.inverse();
				fusedX = fusedP*weightedXi_hat;
			}
		}
    }
}


void HEIF_target::process()
{
	initialize();
	if(fusionNum > 0)
		TargetEstDataCI();
	CI_combination();
}