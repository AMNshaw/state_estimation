#include "EIF.h"

class target_EIF : public EIF
{
private:

    int target_state_size;
	int target_measurement_size;

    float fx;
	float fy;
	float cx;
	float cy;
    float X;
    float Y;
    float Z;

    EIF_data T;
    EIF_data self;
    Eigen::Vector3f boundingBox;

    MAV_eigen Mav_curr;

public:
    target_EIF(int state_size);
    ~target_EIF();
    void computePredPairs(double delta_t);
    void computeCorrPairs();
    void setData(MAV_eigen MAV, Eigen::Vector3f bBox, EIF_data self);
    void setFusionPairs(Eigen::MatrixXf fusedP, Eigen::VectorXf fusedX);

    EIF_data getTgtData();
};