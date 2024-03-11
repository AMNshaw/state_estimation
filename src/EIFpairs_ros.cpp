#include "EIFpairs_ros.h"

EIFpairs_ros::EIFpairs_ros(ros::NodeHandle &nh, string vehicle, int ID, int mavnum)
{
	self_id = ID;
	self_index = ID-1;
	mavNum = mavnum;
	set_topic(vehicle, self_id);

	gotFusedPair = false;
	neighborsEIFpairs = new state_estimation::EIFpairStamped[mavNum];
	rbs2Tgt_EIFPairs = new state_estimation::EIFpairStamped[mavNum];

	/*=================================================================================================================================
		Subscriber
	=================================================================================================================================*/
  	
	neighborsEIFpairs_sub = new ros::Subscriber[mavNum];
	rbs2TgtEIFpairs_sub = new ros::Subscriber[mavNum];
	for(int i=0; i<mavNum; i++)
		if(i != self_index)
		{
			neighborsEIFpairs_sub[i] = nh.subscribe<state_estimation::EIFpairStamped>(neighborsEIFpairs_sub_topic[i], 1, &EIFpairs_ros::neighborsEIFpair_cb, this);
			rbs2TgtEIFpairs_sub[i] = nh.subscribe<state_estimation::EIFpairStamped>(rbs2TgtEIFpairs_sub_topic[i], 1, &EIFpairs_ros::rbs2TgtEIFpair_cb, this);
		}

	/*=================================================================================================================================
		Publisher
	=================================================================================================================================*/
  	
	tgtState_Plot_pub = nh.advertise<state_estimation::Plot>(tgtStatePlot_topic, 1);
	selfState_Plot_pub = nh.advertise<state_estimation::Plot>(selfStatePlot_topic, 1);
	self2TgtEIFpairs_pub = nh.advertise<state_estimation::EIFpairStamped>(self2TgtEIFpairs_pub_topic, 1);
	selfPredEIFpairs_pub = nh.advertise<state_estimation::EIFpairStamped>(selfPredEIFpairs_pub_topic, 1);
	////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

EIFpairs_ros::~EIFpairs_ros()
{
	delete[] neighborsEIFpairs_sub_topic;
	delete[] rbs2TgtEIFpairs_sub_topic;

	delete[] neighborsEIFpairs_sub;
	delete[] rbs2TgtEIFpairs_sub;

	delete[] neighborsEIFpairs;
	delete[] rbs2Tgt_EIFPairs;
}

void EIFpairs_ros::set_topic(std::string vehicle, int id)
{
	string prefix = std::string("/") + vehicle + std::string("_") + std::to_string(id);
	self2TgtEIFpairs_pub_topic = prefix + std::string("/TEIF/fusionPairs");
	tgtStatePlot_topic = prefix + std::string("/THEIF/Plot");
	selfStatePlot_topic = prefix + std::string("/SHEIF/Plot");
	selfPredEIFpairs_pub_topic = prefix + std::string("/SEIF_pred/fusionPairs");

	neighborsEIFpairs_sub_topic = new std::string[mavNum];
	rbs2TgtEIFpairs_sub_topic = new std::string[mavNum];
	for(int i=0; i<mavNum; i++)
		if(i != self_index)
		{
			neighborsEIFpairs_sub_topic[i] = std::string("/") + vehicle + std::string("_") + std::to_string(i+1)
			+ std::string("/SEIF_pred/fusionPairs");
			rbs2TgtEIFpairs_sub_topic[i] = std::string("/") + vehicle + std::string("_") + std::to_string(i+1)
			+ std::string("/TEIF") + std::string("/fusionPairs");
		}
}

std::vector<EIF_data> EIFpairs_ros::get_curr_fusing_data(state_estimation::EIFpairStamped* EIFPairs, double tolerance)
{
	est_data.clear();
	for(int i=0; i<mavNum; i++)
	{
		if(abs(EIFPairs[i].header.stamp.toSec() - ros::Time::now().toSec()) <= tolerance)
			est_data.push_back(eifMsg2Eigen(EIFPairs[i]));
	}
	return est_data;
}

void EIFpairs_ros::neighborsEIFpair_cb(const state_estimation::EIFpairStamped::ConstPtr& msg)
{
	int index = msg->id-1;
	neighborsEIFpairs[index] = *msg;
}

void EIFpairs_ros::rbs2TgtEIFpair_cb(const state_estimation::EIFpairStamped::ConstPtr& msg)
{
	int index = msg->id-1;
	rbs2Tgt_EIFPairs[index] = *msg;
}

/*=================================================================================================================================
    Conversions
=================================================================================================================================*/

state_estimation::EIFpairStamped eigen2EifMsg(EIF_data est_object, int self_id)
{
	state_estimation::EIFpairStamped EIFpairs;
	std::vector<double> P_hat_vec(est_object.P_hat.data(), est_object.P_hat.data() + est_object.P_hat.size());
	std::vector<double> X_hat_vec(est_object.X_hat.data(), est_object.X_hat.data() + est_object.X_hat.size());
	std::vector<double> s_vec(est_object.s.data(), est_object.s.data() + est_object.s.size());
	std::vector<double> y_vec(est_object.y.data(), est_object.y.data() + est_object.y.size());

	EIFpairs.P_hat = P_hat_vec;
	EIFpairs.X_hat = X_hat_vec;
	EIFpairs.s = s_vec;
	EIFpairs.y = y_vec;
	EIFpairs.header.stamp.sec = ros::Time::now().sec;
	EIFpairs.header.stamp.nsec = ros::Time::now().nsec;
	EIFpairs.id = self_id;
	return EIFpairs;
}

EIF_data eifMsg2Eigen(state_estimation::EIFpairStamped eifMsg)
{
	EIF_data est_object;
	int state_size = eifMsg.X_hat.size();
	est_object.P_hat = Eigen::Map<Eigen::MatrixXd>(eifMsg.P_hat.data(), state_size, state_size);
	est_object.X_hat = Eigen::Map<Eigen::VectorXd>(eifMsg.X_hat.data(), state_size);
	est_object.s = Eigen::Map<Eigen::MatrixXd>(eifMsg.s.data(), state_size, state_size);
	est_object.y = Eigen::Map<Eigen::VectorXd>(eifMsg.y.data(), state_size);
	est_object.ID = eifMsg.id;

	return est_object;
}