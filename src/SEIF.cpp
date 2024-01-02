#include "SEIF.h"

/////////////////////////////////////////////////////////// robots EIF ///////////////////////////////////////////////////////////
Self_acc_EIF::Self_acc_EIF(int selfPointer, int MavNum) : EIF(selfPointer, MavNum)
{
	self_state_size = 9;
	self_measurement_size = 3;
	EIF_data_init(self_state_size, self_measurement_size, &this->self);

    //////////////////////// Covariance Tuning ////////////////////////

    Q.block(0, 0, 3, 3) = 7e-4*Eigen::MatrixXf::Identity(3, 3); // position
    Q.block(3, 3, 3, 3) = 7e-4*Eigen::MatrixXf::Identity(3, 3); // velocity
	Q.block(6, 6, 3, 3) = 1e-1*Eigen::MatrixXf::Identity(3, 3); //acceleration

}
Self_acc_EIF::~Self_acc_EIF(){}

void Self_acc_EIF::setData(MAV_eigen MAV)
{
    curr_acc = MAV.a;
}

void Self_acc_EIF::computePredPairs(double delta_t)
{
    float dt = static_cast<float>(delta_t);

    self.F.setIdentity();
    self.F.block(0, 3, 3, 3) = Eigen::Matrix3f::Identity(3, 3)*dt;
	self.F.block(0, 6, 3, 3) = 1/2*Eigen::Matrix3f::Identity(3, 3)*dt*dt;
	self.F.block(3, 6, 3, 3) = Eigen::Matrix3f::Identity(3, 3)*dt;

    self.P_hat = self.F*self.P*self.F.transpose() + Q;
    self.X_hat = self.F*self.X;
}

void Self_acc_EIF::computeCorrPairs()
{
    self.z = curr_acc;

    if(self.z == self.pre_z)
    {
        self.s.setZero();
        self.y.setZero();
    }
    else
    {
        self.h = self.X_hat.segment(6, 3);
        self.H.block(0, 6, 3, 3).setIdentity();

        self.s = self.H.transpose()*R.inverse()*self.H;
        self.y = self.H.transpose()*R.inverse()*(self.z - self.h + self.H*self.X_hat);
    }

    self.P = (self.P_hat.inverse() + self.s).inverse();
    self.X = self.P*(self.P_hat.inverse()*self.X_hat + self.y);
    self.pre_z = self.z;
}

EIF_data Self_acc_EIF::getSelfData(){return self;}
void Self_acc_EIF::setFusionPairs(Eigen::MatrixXf fusedP, Eigen::VectorXf fusedX)
{
    self.P = fusedP;
    self.X = fusedX;
}

void Self_acc_EIF::setCurrState(MAV_eigen MAV)
{
    self.X.segment(0, 3) = MAV.r;
    self.X.segment(3, 3) = MAV.v;
    self.X.segment(6, 3) = MAV.a;
}