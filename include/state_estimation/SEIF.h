#include "EIF.h"

class Self_acc_EIF : public EIF
{
private:
    int robots_state_size;
    int robots_measurement_size;
    EIF_data self;
    Eigen::Vector3f curr_acc;
public:
    Self_acc_EIF(int selfPointer, int MavNum);
    ~Self_acc_EIF();
    void process(double delta_t);
    void computePredPairs(double delta_t);
    void computeCorrPairs();
    void setData(MAV_eigen MAV);
    void setFusionPairs(Eigen::MatrixXf fusedP, Eigen::VectorXf fusedX);
    EIF_data getSelfData();
};