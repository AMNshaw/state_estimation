#ifndef SEIF_NIEGHBORS_H
#define SEIF_NIEGHBORS_H

#include "SEIF.h"

class Self_rel_EIF : public Self_EIF
{
private:
    MAV_eigen mav_self_data;
    std::vector<MAV_eigen> neighbors_GT;
    std::vector<EIF_data> neighbors_pred;
    std::vector<EIF_data> selfWRTneighbors;
    std::vector<Eigen::Vector4f> lidarMeasurements;
    int neighbor_num_curr;
public:
    Self_rel_EIF();
    ~Self_rel_EIF();
    void setData(std::vector<Eigen::Vector4f> LMs, std::vector<EIF_data> robots, MAV_eigen mav_self);
    void setPrediction(EIF_data pred);
    EIF_data computeCorrPair(Eigen::Vector4f LM, EIF_data& neighbor);
    void computeCorrPairs();
    std::vector<EIF_data> getEIFData();
};


#endif