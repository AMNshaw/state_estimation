#include "SEIF_pose.h"

Self_pose_EIF::Self_pose_EIF()
{
	self_measurement_size = 3;
    EIF_measurement_init(self_state_size, self_measurement_size, &self);
    u.setZero(self_state_size);
    //////////////////////// Covariance Tuning ////////////////////////

    R = 1e-5*Eigen::MatrixXf::Identity(self_measurement_size, self_measurement_size);
}
Self_pose_EIF::~Self_pose_EIF(){}

void Self_pose_EIF::setData(Eigen::Vector3f acc, Eigen::Vector3f pose)
{
    curr_pose = pose;
    curr_acc = acc;
}

void Self_pose_EIF::computePredPairs(double delta_t)
{
    float dt = static_cast<float>(delta_t);

    self.F.setIdentity();
    self.F.block(0, 3, 3, 3) = Eigen::Matrix3f::Identity(3, 3)*dt;

    u.segment(0, 3) = 1/2*curr_acc*dt*dt;
    u.segment(3, 3) = curr_acc*dt;
    self.X_hat = self.F*self.X + u;

    self.P_hat = self.F*self.P*self.F.transpose() + Q;
}

void Self_pose_EIF::computeCorrPairs()
{
    self.z = curr_pose ;

    self.s.setZero();
    self.y.setZero();

    if(self.z != self.pre_z)
    {
        self.h = self.X_hat.segment(0, 3);
        self.H.block(0, 0, 3, 3).setIdentity();

        self.s = self.H.transpose()*R.inverse()*self.H;
        self.y = self.H.transpose()*R.inverse()*(self.z - self.h + self.H*self.X_hat);
    }

    self.P = (self.P_hat.inverse() + self.s).inverse();
    self.X = self.P*(self.P_hat.inverse()*self.X_hat + self.y);
    self.pre_z = self.z;
}

EIF_data Self_pose_EIF::getEIFData(){return self;}
void Self_pose_EIF::setFusionPairs(Eigen::MatrixXf fusedP, Eigen::VectorXf fusedX)
{
    self.P = fusedP;
    self.X = fusedX;
}

void Self_pose_EIF::setCurrState(MAV_eigen MAV)
{
    self.X.segment(0, 3) = MAV.r;
    self.X.segment(3, 3) = MAV.v;
}
