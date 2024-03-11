#ifndef SEIF_NIEGHBORS_H
#define SEIF_NIEGHBORS_H

#include "SEIF.h"

class Self_rel_EIF : public Self_EIF
{
private:
    MAV_eigen mav_self_data;
    std::vector<EIF_data> neighbors_pred;
    std::vector<EIF_data> selfWRTneighbors;
    std::vector<Eigen::Vector4d> lidarMeasurements;
    std::vector<Eigen::Vector4d> pre_lidarMeasurements;
    int neighbor_num_curr;
public:
    Self_rel_EIF();
    ~Self_rel_EIF();
    void setNeighborData(std::vector<EIF_data> robots);
    void setLidarMeasurements(std::vector<Eigen::Vector4d> LMs);
    void setEIFpredData(EIF_data pred);
    EIF_data computeCorrPair(Eigen::Vector4d LM, EIF_data& neighbor);
    void computeCorrPairs();
    std::vector<EIF_data> getEIFData();
    void setPreMeasurement(Eigen::Vector4d LM);
    bool checkPreMeasurement(Eigen::Vector4d LM);
};


#endif