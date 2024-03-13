#include "EIF.h"
#include "Camera.h"

class target_EIF : public EIF
{
private:

    int target_state_size;
	int target_measurement_size;
    

    Camera cam;
    double X;
    double Y;
    double Z;

    EIF_data T;
    EIF_data self;
    Eigen::Vector3d boundingBox;

    MAV_eigen Mav_curr;

    Eigen::Vector3d u;

public:
    target_EIF(int state_size);
    ~target_EIF();
    void setInitialState(Eigen::Vector3d Bbox);
    void computePredPairs(double delta_t);
    void computeCorrPairs();
    void setMeasurement(Eigen::Vector3d bBox);
    void setSEIFpredData(EIF_data self);
    void setFusionPairs(Eigen::MatrixXd fusedP, Eigen::VectorXd fusedX, double time);
    void setEstAcc(Eigen::Vector3d acc);

    EIF_data getTgtData();
    EIF_data getSelfData();

    bool filter_init;
};