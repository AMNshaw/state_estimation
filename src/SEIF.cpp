#include "SEIF.h"

Self_EIF::Self_EIF()
{
	self_state_size = 6;
	self_measurement_size = 3;
	
	EIF_prediction_init(self_state_size, &self);
	Q.block(0, 0, 3, 3) = 1e-6*Eigen::MatrixXf::Identity(3, 3); // position
    Q.block(3, 3, 3, 3) = 6e-3*Eigen::MatrixXf::Identity(3, 3); // velocity
}
Self_EIF::~Self_EIF(){}

void Self_EIF::setData(){}

void Self_EIF::computePredPairs(){}

void Self_EIF::computeCorrPairs(){}
