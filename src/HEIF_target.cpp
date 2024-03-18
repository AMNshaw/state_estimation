#include "HEIF_target.h"

HEIF_target::HEIF_target(int x_size=6) : HEIF(x_size)
{
	fusionNum = 0;

	std_filter_size = 20;
	mean_filter_size = 5;

	qpAcc.setZero();
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

/*=================================================================================================================================
		Quadratic Programming
=================================================================================================================================*/
	
bool HEIF_target::QP_init(int dataNum, int order)
{
	qp.dataNum = dataNum;
	qp.functionOrder = order;
	if(dataNum+1 < order)
	{
		std::cout << "QP init failed, dataNum is not sufficient\n";
		return false;
	}
	qp.solver.clearSolver();
	qp.A.resize(qp.dataNum*3, (qp.functionOrder+1)*3);
	qp.Y.resize(qp.dataNum*3);
	qp.lower_bound.resize(qp.dataNum*3);
	qp.upper_bound.resize(qp.dataNum*3);
	
	qp.solver.data()->setNumberOfVariables((qp.functionOrder+1)*3); // for a, b, c, d
    qp.solver.data()->setNumberOfConstraints(0);
	qp.solver.settings()->setWarmStart(true);
	qp.solver.settings()->setMaxIteration(1000);
	qp.solver.settings()->setRelativeTolerance(1e-3);
    qp.solver.settings()->setVerbosity(false); // Set to true if you want to see OSQP's output
    
	qp.solver.initSolver();
	
	return true;
}

void HEIF_target::QP_pushData(double time, Eigen::Vector3d position)
{
	if(abs(position(0)) > 0)
	{
		t.push_back(time);
		positions.push_back(position);
	}
	if(t.size() > qp.dataNum)
	{
		t.pop_front();
		positions.pop_front();
	}
}

bool HEIF_target::computeQP()
{
	if(t.size() != qp.dataNum)
		return false;
	
	std::vector<double> t_(qp.dataNum);
	for(int i=0; i<qp.dataNum; i++)
		t_[i] = t[i] - t[0];

	for(int i=0; i<qp.dataNum; i++)
	{
		for(int j=0; j<qp.functionOrder+1; j++)
			qp.A(i, j) = pow(t_[i], j);
	}
	qp.A.block(qp.dataNum, qp.functionOrder+1, qp.dataNum, qp.functionOrder+1) = qp.A.block(0, 0, qp.dataNum, qp.functionOrder+1);
	qp.A.block(qp.dataNum*2, (qp.functionOrder+1)*2, qp.dataNum, qp.functionOrder+1) = qp.A.block(0, 0, qp.dataNum, qp.functionOrder+1);

	for(int i=0; i<qp.dataNum; i++)
	{
		qp.Y(i) = positions[i](0);
		qp.Y(i+qp.dataNum) = positions[i](1);
		qp.Y(i+qp.dataNum*2) = positions[i](2);
	}
	qp.P = 2*qp.A.transpose()*qp.A;
	qp.q = -2*qp.A.transpose()*qp.Y;

	Eigen::SparseMatrix<double> P_sparse = qp.P.sparseView();

	qp.solver.data()->clearHessianMatrix();
    if(!qp.solver.data()->setHessianMatrix(P_sparse)) return false;
    if(!qp.solver.data()->setGradient(qp.q)) return false;

	if (qp.solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError) 
	{
        qp.solution = qp.solver.getSolution();
		qpAcc(0) = qp.solution(2);
		qpAcc(1) = qp.solution(2+(qp.functionOrder+1));
		qpAcc(2) = 0;
		// qpAcc(2) = qp.solution(2+(qp.functionOrder+1)*2);
		//std::cout << "QPsolution:\n" << qp.solution << "\n";
		return true;
    }
	else 
	{
        std::cout << "Problem in solving" << std::endl;
		return false;
    }
	return true;
}

Eigen::Vector3d HEIF_target::getQpAcc(){return qpAcc;}