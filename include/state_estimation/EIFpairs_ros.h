#ifndef EIF_ROS_H
#define EIF_ROS_H
#pragma once

#include <string>

#include <ros/ros.h>
#include <state_estimation/EIFpairStamped.h>
#include <state_estimation/Plot.h>
#include "Mav.h"

#include "EIF.h"

class EIFpairs_ros
{
private:
	ros::Subscriber* neighborsEIFpairs_sub;
	ros::Subscriber* rbs2TgtEIFpairs_sub;

	std::string* neighborsEIFpairs_sub_topic;
	std::string self2TgtEIFpairs_pub_topic;
	std::string* rbs2TgtEIFpairs_sub_topic;
	std::string selfPredEIFpairs_pub_topic;
	std::string tgtStatePlot_topic;
	std::string selfStatePlot_topic;

	int state_size;
	int mavNum;
	int self_id;
	int self_index;
public:
	EIFpairs_ros(ros::NodeHandle &nh, string vehicle, int ID, int mavnum);
	~EIFpairs_ros();

	ros::Publisher tgtState_Plot_pub;
	ros::Publisher selfState_Plot_pub;
	ros::Publisher self2TgtEIFpairs_pub;
	ros::Publisher selfPredEIFpairs_pub;

	std_msgs::Header sync_header;
	state_estimation::EIFpairStamped* neighborsEIFpairs;
	state_estimation::EIFpairStamped* rbs2Tgt_EIFPairs;
	std::vector<EIF_data> est_data;

	bool gotFusedPair;

	void neighborsEIFpair_cb(const state_estimation::EIFpairStamped::ConstPtr& msg);
	void rbs2TgtEIFpair_cb(const state_estimation::EIFpairStamped::ConstPtr& msg);

	void set_topic(std::string group_ns, int id);
	std::vector<EIF_data> get_curr_fusing_data(state_estimation::EIFpairStamped* EIFPairs, double tolerance);
};

/*=================================================================================================================================
    Conversions, comparison
=================================================================================================================================*/

EIF_data eifMsg2Eigen(state_estimation::EIFpairStamped eifMsg);
state_estimation::EIFpairStamped eigen2EifMsg(EIF_data est_object, int self_id);
state_estimation::Plot compare(MAV_eigen GT, Eigen::VectorXd est);

#endif