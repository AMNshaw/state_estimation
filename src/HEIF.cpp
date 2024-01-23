#include "HEIF.h"

HEIF::HEIF(int x_size=9)
{
	state_size = x_size;
	initialize();
}

HEIF::~HEIF(){}

void HEIF::initialize()
{
	weightedOmega_hat.setZero(state_size, state_size);
	weightedXi_hat.setZero(state_size);
	weightedS.setZero(state_size, state_size);
	weightedY.setZero(state_size);
}

void HEIF::CI_combination()
{
	fusedP = (weightedOmega_hat + weightedS).inverse();
	fusedX = fusedP*(weightedXi_hat + weightedY);
}

Eigen::MatrixXf HEIF::getFusedCov(){return fusedP;}
Eigen::VectorXf HEIF::getFusedState(){return fusedX;}
