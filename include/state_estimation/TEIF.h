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
    Eigen::MatrixXf Intrinsic;
    Eigen::Vector3f boundingBox;

    MAV_eigen Mav_curr;

public:
    target_EIF(int selfPointer, int MavNum, bool est_target_acc);
    ~target_EIF();
    void computePredPairs(double delta_t, EIF_data* Rbs);
    void computeCorrPairs();
    void setData(MAV_eigen MAV, Eigen::Vector3f bBox);
    void setFusionPairs(Eigen::MatrixXf fusedP, Eigen::VectorXf fusedX);

    EIF_data getTgtData();
};