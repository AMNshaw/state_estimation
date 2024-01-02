#include "EIF.h"

class Self_acc_EIF : public EIF
{
private:
    int self_state_size;
    int self_measurement_size;
    EIF_data self;
    Eigen::Vector3f curr_acc;
public:
    Self_acc_EIF(int selfPointer, int MavNum);
    ~Self_acc_EIF();
    void computePredPairs(double delta_t);
    void computeCorrPairs();
    void setData(MAV_eigen MAV);
    void setFusionPairs(Eigen::MatrixXf fusedP, Eigen::VectorXf fusedX);
    void setCurrState(MAV_eigen MAV);
    EIF_data getSelfData();
};