#ifndef SEIF_POSE_H
#define SEIF_POSE_H

#include "SEIF.h"

class Self_pose_EIF : public Self_EIF
{
private:
    Eigen::Vector3f curr_pose;
    Eigen::Vector3f curr_acc;
    Eigen::VectorXf u;
public:
    Self_pose_EIF();
    ~Self_pose_EIF();
    void computePredPairs(double delta_t);
    void computeCorrPairs();
    void setData(Eigen::Vector3f acc, Eigen::Vector3f pose);
    void setFusionPairs(Eigen::MatrixXf fusedP, Eigen::VectorXf fusedX);
    void setCurrState(MAV_eigen MAV);
    EIF_data getEIFData();
};


#endif