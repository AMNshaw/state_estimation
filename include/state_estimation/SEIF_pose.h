#ifndef SEIF_POSE_H
#define SEIF_POSE_H

#include "SEIF.h"

class Self_pose_EIF : public Self_EIF
{
private:
    Eigen::VectorXd u;
    Eigen::Vector3d measurement;
public:
    Self_pose_EIF();
    ~Self_pose_EIF();
    void computePredPairs(double delta_t);
    void computeCorrPairs();
    void setMeasurement(Eigen::Vector3d z);
    void setFusionPairs(Eigen::MatrixXd fusedP, Eigen::VectorXd fusedX);
    void setCurrState(MAV_eigen MAV);
    EIF_data getEIFData();
};


#endif