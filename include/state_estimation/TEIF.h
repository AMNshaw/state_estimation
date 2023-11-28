#include "EIF.h"

class target_EIF : public EIF
{
private:

    int target_state_size;
	int target_measurement_size;
    EIF_data T;
    Eigen::MatrixXf Intrinsic;
    Eigen::Vector3f boundingBox;


public:
    target_EIF(int selfPointer, int MavNum, bool est_target_acc);
    ~target_EIF();
    void computePredPairs(double delta_t, EIF_data* Rbs);
    void computeCorrPairs();
    void setData(MAV_eigen* MAVs, Eigen::Vector3f bBox);
};