#include "SEIF_neighbors.h"

Self_rel_EIF::Self_rel_EIF()
{
	self_measurement_size = 3;
    neighbor_num_curr = 0;
	EIF_measurement_init(self_state_size, self_measurement_size, &self);
    
    //////////////////////// Covariance Tuning ////////////////////////

    R(0, 0) = 1e-4;
    R(1, 1) = 2e-1;
    R(2, 2) = 2e-1;
}
Self_rel_EIF::~Self_rel_EIF(){}

void Self_rel_EIF::setLidarMeasurements(std::vector<Eigen::Vector4d> LMs)
{
    lidarMeasurements = LMs;
}

void Self_rel_EIF::setNeighborData(std::vector<EIF_data> robots)
{ 
    neighbor_num_curr = 0;
    neighbor_num_curr = robots.size();
    neighbors_pred = robots;
}

void Self_rel_EIF::setEIFpredData(EIF_data pred)
{
    self = pred;
}

EIF_data Self_rel_EIF::computeCorrPair(Eigen::Vector4d LM, EIF_data& neighbor_pred)
{
    self.z = LM.segment(0, 3);

    self.s.setZero();
    self.y.setZero();
    if(checkPreMeasurement(LM))
    {
        Eigen::MatrixXd R_hat;
        Eigen::Matrix3d R_W2B_i = Mav_eigen_self.R_w2b;
        Eigen::Vector3d r_B_hat = R_W2B_i*(neighbor_pred.X_hat.segment(0, 3) - self.X_hat.segment(0, 3));
        
        double D = sqrt(pow(r_B_hat(0), 2) + pow(r_B_hat(1), 2) + pow(r_B_hat(2), 2));
        
        self.h(0) = D;
        self.h(1) = std::acos(r_B_hat(2)/D);
        self.h(2) = std::atan2(r_B_hat(1), r_B_hat(0));
        
        ////////////////////////////////////////////////// derivative w.r.t neighbor //////////////////////////////////////////////////
        
        neighbor_pred.H.setZero(self_measurement_size, self_state_size);

        neighbor_pred.H(0, 0) = (R_W2B_i(0, 0)*r_B_hat(0) + R_W2B_i(1, 0)*r_B_hat(1) + R_W2B_i(2, 0)*r_B_hat(2)) / D;
        neighbor_pred.H(0, 1) = (R_W2B_i(0, 1)*r_B_hat(0) + R_W2B_i(1, 1)*r_B_hat(1) + R_W2B_i(2, 1)*r_B_hat(2)) / D;
        neighbor_pred.H(0, 2) = (R_W2B_i(0, 2)*r_B_hat(0) + R_W2B_i(1, 2)*r_B_hat(1) + R_W2B_i(2, 2)*r_B_hat(2)) / D;
        
        neighbor_pred.H(1, 0) = (R_W2B_i(0, 0)*r_B_hat(0)*r_B_hat(2)
                                + R_W2B_i(1, 0)*r_B_hat(1)*r_B_hat(2)
                                - R_W2B_i(2, 0)*(r_B_hat(0)*r_B_hat(0) + r_B_hat(1)*r_B_hat(1)))
                                /(D*D * sqrt(r_B_hat(0)*r_B_hat(0) + r_B_hat(1)*r_B_hat(1)));
        neighbor_pred.H(1, 1) = (R_W2B_i(0, 1)*r_B_hat(0)*r_B_hat(2)
                                + R_W2B_i(1, 1)*r_B_hat(1)*r_B_hat(2)
                                - R_W2B_i(2, 1)*(r_B_hat(0)*r_B_hat(0) + r_B_hat(1)*r_B_hat(1)))
                                /(D*D * sqrt(r_B_hat(0)*r_B_hat(0) + r_B_hat(1)*r_B_hat(1)));
        neighbor_pred.H(1, 2) = (R_W2B_i(0, 2)*r_B_hat(0)*r_B_hat(2)
                                + R_W2B_i(1, 2)*r_B_hat(1)*r_B_hat(2)
                                - R_W2B_i(2, 2)*(r_B_hat(0)*r_B_hat(0) + r_B_hat(1)*r_B_hat(1)))
                                /(D*D * sqrt(r_B_hat(0)*r_B_hat(0) + r_B_hat(1)*r_B_hat(1)));

        neighbor_pred.H(2, 0) = (-R_W2B_i(0, 0)*r_B_hat(1) + R_W2B_i(1, 0)*r_B_hat(0)) / (r_B_hat(0)*r_B_hat(0) + r_B_hat(1)*r_B_hat(1));
        neighbor_pred.H(2, 1) = (-R_W2B_i(0, 1)*r_B_hat(1) + R_W2B_i(1, 1)*r_B_hat(0)) / (r_B_hat(0)*r_B_hat(0) + r_B_hat(1)*r_B_hat(1));
        neighbor_pred.H(2, 2) = (-R_W2B_i(0, 2)*r_B_hat(1) + R_W2B_i(1, 2)*r_B_hat(0)) / (r_B_hat(0)*r_B_hat(0) + r_B_hat(1)*r_B_hat(1));

        ////////////////////////////////////////////////// derivative w.r.t self //////////////////////////////////////////////////
        self.H = -neighbor_pred.H;
        
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        R_hat = R + neighbor_pred.H*neighbor_pred.P_hat*neighbor_pred.H.transpose();

        self.s = self.H.transpose()*R_hat.inverse()*self.H;
        self.y = self.H.transpose()*R_hat.inverse()*(self.z - self.h + self.H*self.X_hat);
    }
    setPreMeasurement(LM);
    return self;
}

void Self_rel_EIF::computeCorrPairs()
{
    selfWRTneighbors.clear();
    for(int i=0; i< neighbor_num_curr; i++)
    {
        for(int j=0; j<lidarMeasurements.size(); j++)
            if(lidarMeasurements[j](3) == neighbors_pred[i].ID)
            {
                selfWRTneighbors.push_back(computeCorrPair(lidarMeasurements[j], neighbors_pred[i]));
                break;
            }
    }
}

std::vector<EIF_data> Self_rel_EIF::getEIFData(){ return selfWRTneighbors;}

void Self_rel_EIF::setPreMeasurement(Eigen::Vector4d LM)
{
    if(pre_lidarMeasurements.size() == 0)
    {
        pre_lidarMeasurements.push_back(LM);
    }
    else
    {
        bool found = false;
        for(int i=0; i< pre_lidarMeasurements.size(); i++)
        {
            if(pre_lidarMeasurements[i](3) == LM(3))
            {
                pre_lidarMeasurements[i] = LM;
                found = true;
                break;
            }    
        }
        if(!found)
            pre_lidarMeasurements.push_back(LM);
    }
}

bool Self_rel_EIF::checkPreMeasurement(Eigen::Vector4d LM)
{
    if(pre_lidarMeasurements.size() > 0)
    {
        for(int i=0; i<pre_lidarMeasurements.size(); i++)
        {
            if(pre_lidarMeasurements[i](3) == LM(3))
            {
                if(pre_lidarMeasurements[i].segment(0, 3) == LM.segment(0, 3))
                    return false;
                else
                    return true;
            }
        }
    }
    return true;
}