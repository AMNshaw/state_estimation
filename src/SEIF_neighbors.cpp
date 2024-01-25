#include "SEIF_neighbors.h"

Self_rel_EIF::Self_rel_EIF()
{
	self_measurement_size = 3;
    neighbor_num_curr = 0;
	EIF_measurement_init(self_state_size, self_measurement_size, &self);
    
    //////////////////////// Covariance Tuning ////////////////////////

    R(0, 0) = 1e-5;
    R(1, 1) = 1e-2;
    R(2, 2) = 1e-2;

    // R(0, 0) = 1e-5;
    // R(1, 1) = 1e-5;
    // R(2, 2) = 1e-5;

}
Self_rel_EIF::~Self_rel_EIF(){}

void Self_rel_EIF::setData(std::vector<Eigen::Vector4f> LMs
                        , std::vector<EIF_data> robots
                        , MAV_eigen mav_self)
{ 
    mav_self_data = mav_self;
    lidarMeasurements = LMs;
    neighbor_num_curr = 0;
    neighbor_num_curr = robots.size();
    neighbors_pred = robots;
    
}

void Self_rel_EIF::setPrediction(EIF_data pred)
{
    self = pred;
}

// EIF_data Self_rel_EIF::computeCorrPair(Eigen::Vector4f LM, EIF_data& neighbor_pred)
// {
//     self.z = LM.segment(0, 3);

//     self.s.setZero();
//     self.y.setZero();

//     if(self.z != self.pre_z)
//     {
//         Eigen::MatrixXf R_hat;
//         Eigen::Vector3f E_ns = neighbor_pred.X_hat.segment(0, 3) - self.X_hat.segment(0, 3);
                
//         self.h = E_ns;
        
//         ////////////////////////////////////////////////// derivative w.r.t neighbor //////////////////////////////////////////////////
        
//         neighbor_pred.H.setZero(self_measurement_size, self_state_size);
//         neighbor_pred.H.block(0, 0, 3, 3) = Eigen::Matrix3f::Identity(3, 3);
        
//         ////////////////////////////////////////////////// derivative w.r.t self //////////////////////////////////////////////////
//         self.H.setZero();
//         self.H.block(0, 0, 3, 3) = -Eigen::Matrix3f::Identity(3, 3);
        
//         ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//         R_hat = R + neighbor_pred.H*neighbor_pred.P_hat*neighbor_pred.H.transpose();

//         self.s = self.H.transpose()*R_hat.inverse()*self.H;
//         self.y = self.H.transpose()*R_hat.inverse()*(self.z - self.h + self.H*self.X_hat);
//     }
//     self.pre_z = self.z;
//     return self;
// }

EIF_data Self_rel_EIF::computeCorrPair(Eigen::Vector4f LM, EIF_data& neighbor_pred)
{
    self.z = LM.segment(0, 3);

    self.s.setZero();
    self.y.setZero();

    if(self.z != self.pre_z)
    {
        Eigen::MatrixXf R_hat, R_trans;
        Eigen::Vector3f E_ns = neighbor_pred.X_hat.segment(0, 3) - self.X_hat.segment(0, 3);
        
        float r = sqrt(pow(E_ns(0), 2) + pow(E_ns(1), 2) + pow(E_ns(2), 2));
        
        self.h(0) = r;
        self.h(1) = std::acos(E_ns(2)/r);
        self.h(2) = std::atan2(E_ns(1), E_ns(0));
        
        ////////////////////////////////////////////////// derivative w.r.t neighbor //////////////////////////////////////////////////
        
        neighbor_pred.H.setZero(self_measurement_size, self_state_size);
        neighbor_pred.H(0, 0) = E_ns(0)/r;
        neighbor_pred.H(0, 1) = E_ns(1)/r;
        neighbor_pred.H(0, 2) = E_ns(2)/r;

        neighbor_pred.H(1, 0) = (E_ns(2) * E_ns(0)) / (pow(r, 2) * sqrt(pow(E_ns(0), 2)+pow(E_ns(1), 2)));
        neighbor_pred.H(1, 1) = (E_ns(2) * E_ns(1)) / (pow(r, 2) * sqrt(pow(E_ns(0), 2)+pow(E_ns(1), 2)));
        neighbor_pred.H(1, 2) = -sqrt(pow(E_ns(0), 2)+pow(E_ns(1), 2)) / pow(r, 2);

        neighbor_pred.H(2, 0) = -E_ns(1)/((pow(E_ns(0), 2)+pow(E_ns(1), 2)));
        neighbor_pred.H(2, 1) = E_ns(0)/((pow(E_ns(0), 2)+pow(E_ns(1), 2)));
        neighbor_pred.H(2, 2) = 0;
        
        ////////////////////////////////////////////////// derivative w.r.t self //////////////////////////////////////////////////
        self.H = -neighbor_pred.H;
        
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        Eigen::MatrixXf R2 = 1e-5*Eigen::Matrix3f::Identity(3, 3);
        R_trans = neighbor_pred.H.block(0, 0, 3, 3)*R2*neighbor_pred.H.block(0, 0, 3, 3).transpose();
        //std::cout << "R_trans:\n" << R_trans << "\n";
        R_hat = R + neighbor_pred.H*neighbor_pred.P_hat*neighbor_pred.H.transpose();

        self.s = self.H.transpose()*R_hat.inverse()*self.H;
        self.y = self.H.transpose()*R_hat.inverse()*(self.z - self.h + self.H*self.X_hat);
    }
    self.pre_z = self.z;
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
