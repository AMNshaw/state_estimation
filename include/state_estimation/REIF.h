#include "EIF.h"

class robots_EIF : public EIF
{
private:
    int robots_state_size;
    int robots_measurement_size;
    EIF_data* Rbs;

    std::vector<MAV_eigen> Mavs_curr;
public:
    robots_EIF(int selfPointer, int MavNum);
    ~robots_EIF();
    void computePredPairs(double delta_t);
    void computeCorrPairs();
    void setData(std::vector<MAV_eigen> MAVs);
    void setData(std::vector<MAV_eigen> MAVs, Eigen::VectorXf self_state);
    EIF_data* getRbsData();
};
