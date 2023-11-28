#include "EIF.h"

class robots_EIF : public EIF
{
private:
    int robots_state_size;
    int robots_measurement_size;
    EIF_data* Rbs;
public:
    robots_EIF(int selfPointer, int MavNum);
    ~robots_EIF();
    void process(double delta_t);
    void computePredPairs(double delta_t);
    void computeCorrPairs();
    void setData(MAV_eigen* MAVs);
    EIF_data* getRbsData();
};