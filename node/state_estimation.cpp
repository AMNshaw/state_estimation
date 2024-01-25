#include <string>
#include <sstream>
#include <vector>
#include <cctype>
#include <cmath>

#include <Eigen/Dense>
#include <ros/ros.h>
#include "ros/param.h"
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Header.h>
#include <state_estimation/EIFpairStamped.h>
#include <state_estimation/RMSE.h>
#include <state_estimation/Plot.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <gazebo_msgs/ModelStates.h>

#include "Mav.h"
#include "TEIF.h"
#include "HEIF_self.h"
#include "HEIF_target.h"
#include "CommonFuncs.h"
#include "SEIF_pose.h"
#include "SEIF_neighbors.h"

class Data_process
{
private:
	ros::Subscriber bboxes_sub;
	ros::Subscriber tgtFusedPair_sub;
	ros::Subscriber* neighborsEIFpairs_sub;
	ros::Subscriber* rbs2TgtEIFpairs_sub;
	ros::Subscriber groundTruth_sub;

	std::string bbox_topic;
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
	int lidar_hz;
	int lidar_count;
public:
	Data_process(ros::NodeHandle &nh, string vehicle, int ID, int mavnum);
	~Data_process();

	ros::Publisher tgtState_Plot_pub;
	ros::Publisher selfState_Plot_pub;
	ros::Publisher self2TgtEIFpairs_pub;
	ros::Publisher selfPredEIFpairs_pub;

	std_msgs::Header sync_header;
	state_estimation::EIFpairStamped* neighborsEIFpairs;
	state_estimation::EIFpairStamped* rbs2Tgt_EIFPairs;
	std::vector<EIF_data> est_data;

	std::vector<Eigen::Vector4f> lidarMeasurements;
	std::vector<MAV> GT;
	std::vector<MAV_eigen> GT_eigen;
	std::vector<float> bboxes;
	bool gotBbox;
	bool gotFusedPair;

	void bboxes_cb(const std_msgs::Float32MultiArray::ConstPtr& msg);
	void neighborsEIFpair_cb(const state_estimation::EIFpairStamped::ConstPtr& msg);
	void rbs2TgtEIFpair_cb(const state_estimation::EIFpairStamped::ConstPtr& msg);
	void groundTruth_cb(const gazebo_msgs::ModelStates::ConstPtr& msg);

	void set_topic(std::string group_ns, int id);
	Eigen::Vector3f bboxMsg2Eigen();
	std::vector<EIF_data> get_curr_fusing_data(state_estimation::EIFpairStamped* EIFPairs, float tolerance);
	std::vector<Eigen::Vector4f> lidarMeasure(std::vector<MAV_eigen> mav_eigen);
};

Data_process::Data_process(ros::NodeHandle &nh, string vehicle, int ID, int mavnum)
{
	self_id = ID;
	self_index = ID-1;
	mavNum = mavnum;
	set_topic(vehicle, self_id);
	lidar_hz = 5;
	lidar_count = 0;

	gotBbox  = false;
	gotFusedPair = false;
	neighborsEIFpairs = new state_estimation::EIFpairStamped[mavNum];
	rbs2Tgt_EIFPairs = new state_estimation::EIFpairStamped[mavNum];
	GT.resize(mavNum+1);

	/////////////////////////////////////////////////Subscriber/////////////////////////////////////////////////
	bboxes_sub = nh.subscribe<std_msgs::Float32MultiArray>(bbox_topic, 2, &Data_process::bboxes_cb, this);
	groundTruth_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 30, &Data_process::groundTruth_cb, this);

	neighborsEIFpairs_sub = new ros::Subscriber[mavNum];
	rbs2TgtEIFpairs_sub = new ros::Subscriber[mavNum];
	for(int i=0; i<mavNum; i++)
		if(i != self_index)
		{
			neighborsEIFpairs_sub[i] = nh.subscribe<state_estimation::EIFpairStamped>(neighborsEIFpairs_sub_topic[i], 1, &Data_process::neighborsEIFpair_cb, this);
			rbs2TgtEIFpairs_sub[i] = nh.subscribe<state_estimation::EIFpairStamped>(rbs2TgtEIFpairs_sub_topic[i], 1, &Data_process::rbs2TgtEIFpair_cb, this);
		}

	/////////////////////////////////////////////////Publisher /////////////////////////////////////////////////

	tgtState_Plot_pub = nh.advertise<state_estimation::Plot>(tgtStatePlot_topic, 1);
	selfState_Plot_pub = nh.advertise<state_estimation::Plot>(selfStatePlot_topic, 1);
	self2TgtEIFpairs_pub = nh.advertise<state_estimation::EIFpairStamped>(self2TgtEIFpairs_pub_topic, 1);
	selfPredEIFpairs_pub = nh.advertise<state_estimation::EIFpairStamped>(selfPredEIFpairs_pub_topic, 1);
	////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

Data_process::~Data_process()
{
	delete[] neighborsEIFpairs_sub_topic;
	delete[] rbs2TgtEIFpairs_sub_topic;

	delete[] neighborsEIFpairs_sub;
	delete[] rbs2TgtEIFpairs_sub;

	delete[] neighborsEIFpairs;
	delete[] rbs2Tgt_EIFPairs;
}

void Data_process::set_topic(std::string vehicle, int id)
{
	string prefix = std::string("/") + vehicle + std::string("_") + std::to_string(id);
	bbox_topic = prefix + std::string("/synchronizer/yolov7/boundingBox");
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

Eigen::Vector3f Data_process::bboxMsg2Eigen()
{
	Eigen::Vector3f bboxes_eigen;
	bboxes_eigen << bboxes[0], bboxes[1], bboxes[2];
	return bboxes_eigen;
}

std::vector<EIF_data> Data_process::get_curr_fusing_data(state_estimation::EIFpairStamped* EIFPairs, float tolerance)
{
	est_data.clear();
	for(int i=0; i<mavNum; i++)
	{
		if(abs(EIFPairs[i].header.stamp.toSec() - ros::Time::now().toSec()) <= tolerance)
			est_data.push_back(eifMsg2Eigen(EIFPairs[i]));

	}
	return est_data;
}

void Data_process::neighborsEIFpair_cb(const state_estimation::EIFpairStamped::ConstPtr& msg)
{
	int index = msg->id-1;
	neighborsEIFpairs[index] = *msg;
}

void Data_process::rbs2TgtEIFpair_cb(const state_estimation::EIFpairStamped::ConstPtr& msg)
{
	int index = msg->id-1;
	rbs2Tgt_EIFPairs[index] = *msg;
}

void Data_process::bboxes_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	if(!gotBbox)
		gotBbox = true;
	bboxes = msg->data;
}

void Data_process::groundTruth_cb(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
	lidar_count++;
	std::vector<string> name = msg->name;
	for(int i=0; i< name.size(); i++)
	{
		if(std::isdigit(name[i].back()))
		{
			GT[int(name[i].back()-'0')].setPose(msg->pose[i]);
			GT[int(name[i].back()-'0')].setTwist(msg->twist[i]);
		}
	}
	std::vector<MAV> nearRobots(GT.begin()+1, GT.begin()+GT.size());
	GT_eigen = mavsMsg2Eigen(GT);

	if(lidar_count == 500/lidar_hz)
	{
		lidarMeasurements = lidarMeasure(mavsMsg2Eigen(nearRobots));
		lidar_count = 0;
	}

}

std::vector<Eigen::Vector4f> Data_process::lidarMeasure(std::vector<MAV_eigen> mav_eigen)
{
	Eigen::Vector4f measurement;
	std::vector<Eigen::Vector4f> measurements;
	for(int i=0; i<mavNum; i++)
	{
		if(i != self_index)
		{
			measurement(0) = sqrt(pow(mav_eigen[i].r(0)-mav_eigen[self_index].r(0), 2)  // r
			+ pow(mav_eigen[i].r(1)-mav_eigen[self_index].r(1), 2) 
			+ pow(mav_eigen[i].r(2)-mav_eigen[self_index].r(2), 2)); 
			measurement(1) = std::acos((mav_eigen[i].r(2)-mav_eigen[self_index].r(2))/measurement(0)); // theta
			measurement(2) = std::atan2(mav_eigen[i].r(1)-mav_eigen[self_index].r(1), mav_eigen[i].r(0)-mav_eigen[self_index].r(0)); // phi
			
			//measurement.segment(0, 3) = mav_eigen[i].r - mav_eigen[self_index].r;

			measurement(3) = i+1; // ID
			measurements.push_back(measurement);
		}
	}
	return measurements;
}

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "state_estimation");
    ros::NodeHandle nh;

    std::string vehicle;
	bool est_target_acc = false;
    bool consensus = false;
	int mavNum = 3;
    int rosRate;
	int ID = 0;
	int state_size = 6;
	float targetTimeTol = 0.05;
	float pose_hz = 30;
    ros::param::get("vehicle", vehicle);
	ros::param::get("ID", ID);
    ros::param::get("rate", rosRate);
	ros::param::get("consensus", consensus);
	ros::param::get("stateSize", state_size);
	ros::param::get("targetTimeTol", targetTimeTol);
	ros::param::get("pose_hz", pose_hz);
	cout << "POSE_HZ: " << pose_hz << "\n";
	double last_t;
	double dt;
	MAV::self_index = ID-1;

	ros::Rate rate(rosRate);

	MAV mav(nh, vehicle, ID);
	mav.setPose_hz(pose_hz);
	
	MAV_eigen mav_eigen;
	Data_process dp(nh, vehicle, ID, mavNum);

	while(ros::ok())
	{
		int topics_count = 0;
		if(mav.pose_init)
			topics_count ++;
		else
			printf("[%s_%i]: Waiting for Pose topic...\n", vehicle.c_str(), ID);
		if(mav.vel_init)
			topics_count ++;
		else
			printf("[%s_%i]: Waiting for Vel topic...\n", vehicle.c_str(), ID);
		if(mav.imu_init)
			topics_count ++;
		else
			printf("[%s_%i]: Waiting for Imu topic...\n", vehicle.c_str(), ID);
		if(topics_count == 3)
			break;

		rate.sleep();
		ros::spinOnce();
	}
	for(int i=0; i< 50; i++)
	{
		rate.sleep();
		ros::spinOnce();
	}
	printf("\n[%s_%i EIF]: Topics all checked\n", vehicle.c_str(), ID);
	
	Self_pose_EIF SEIF_pose;
	Self_rel_EIF SEIF_neighbors;
	target_EIF teif(6);
	HEIF_self sheif(6);
	HEIF_target theif(6);

	mav_eigen = mavMsg2Eigen(mav);
	SEIF_pose.setCurrState(mav_eigen);

	printf("\n[%s_%i EIF]: EIF constructed\n\n", vehicle.c_str(), ID);
	
    last_t = ros::Time::now().toSec();
	dt = 0.001;
    while(ros::ok())
    {
		mav_eigen = mavMsg2Eigen(mav);

		//////////////////////////////////////////////////// Prediction ////////////////////////////////////////////////////
		SEIF_pose.setData(mav_eigen.a, mav_eigen.r);
		SEIF_pose.computePredPairs(dt);
		dp.selfPredEIFpairs_pub.publish(eigen2EifMsg(SEIF_pose.getEIFData(), ID));
		
		SEIF_neighbors.setPrediction(SEIF_pose.getEIFData());
		SEIF_neighbors.setData(dp.lidarMeasurements
							, dp.get_curr_fusing_data(dp.neighborsEIFpairs, 0.05)
							, mav_eigen);

		if(dp.gotBbox)
		{
			if(consensus)
				state2MavEigen(SEIF_pose.getEIFData().X_hat, mav_eigen);
		 	teif.setData(mav_eigen
						, dp.bboxMsg2Eigen()
						, SEIF_pose.getEIFData());
		 	teif.computePredPairs(dt);
		}

		// //////////////////////////////////////////////////// Correction ////////////////////////////////////////////////////
		SEIF_pose.computeCorrPairs();
		SEIF_neighbors.computeCorrPairs();
		if(dp.gotBbox)
		{
		 	teif.computeCorrPairs();
			dp.self2TgtEIFpairs_pub.publish(eigen2EifMsg(teif.getTgtData(), ID));
		}
		// //////////////////////////////////////////////////// Fusion ////////////////////////////////////////////////////
		sheif.setSelfEstData(SEIF_pose.getEIFData());
		sheif.setNeighborEstData(SEIF_neighbors.getEIFData());
		sheif.process();
		SEIF_pose.setFusionPairs(sheif.getFusedCov(), sheif.getFusedState());
		cout << "SEIF:\n";
		dp.selfState_Plot_pub.publish(compare(dp.GT_eigen[ID], sheif.getFusedState()));
		if(dp.gotBbox)
		{
			std::vector<EIF_data> allTgtEIFData;
			allTgtEIFData = dp.get_curr_fusing_data(dp.rbs2Tgt_EIFPairs, 0.03);
			allTgtEIFData.push_back(teif.getTgtData());
			theif.setTargetEstData(allTgtEIFData);
			theif.process();
			teif.setFusionPairs(theif.getFusedCov(), theif.getFusedState());
			cout << "TEIF:\n";
			dp.tgtState_Plot_pub.publish(compare(dp.GT_eigen[0], theif.getFusedState()));
		}
		/////////////////////////////////////////////////////////////////////////////////////
    	dt = ros::Time::now().toSec() - last_t;
    	last_t = ros::Time::now().toSec();
		
		rate.sleep();
    	ros::spinOnce();
    }
	
	return 0;
}